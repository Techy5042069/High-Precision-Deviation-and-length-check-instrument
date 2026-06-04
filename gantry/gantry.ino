// ─────────────────────────────────────────────────────────────────────────────
// gantry.ino  —  Single-Arduino Gantry Controller
//
// One Arduino R4 drives the CL86T stepper driver AND reads the distance
// sensor from A0.  No separate sensor board, no TCP/WiFi, no TICK protocol,
// no bulk buffering.  Each sample point is reported in a single telemetry
// line that carries both the motor position and the ADC reading.
//
// Wiring:
//   STEP → pin 6    DIR → pin 4    ENA → pin 5 (LOW = enabled)
//   SW_A → pin 9    (LEFT  limit, INPUT_PULLUP)
//   SW_B → pin 11   (RIGHT limit, INPUT_PULLUP)
//   SENSOR → A0
//
// ── PC → Arduino commands ─────────────────────────────────────────────────────
//   R               move right (manual)
//   L               move left  (manual)
//   S               stop
//   A               abort scan (emergency stop)
//   X               start scan (home left, then measure rightward)
//   +               speed up   (decrease step delay)
//   -               speed down (increase step delay)
//   V<delay_us>     set manual step delay (µs)
//   W<delay_us>     set scan   step delay (µs)
//   B<steps>        set SCAN_SAMPLE_STEPS
//   TICK            read ADC once (manual mode only) → ADC:<value>
//   START:<en>,<lo>,<hi>,<armCount>   configure OOR for next scan
//   PING:<ms>       → PONG:<ms>,<arduino_ms>
//
// ── Arduino → PC messages ─────────────────────────────────────────────────────
//   SS                        scan measuring started
//   ST:<seq>,<steps>,<adc>   sample telemetry (one per SCAN_SAMPLE_STEPS steps)
//   SD:<total_seq>            scan done (right limit reached)
//   SA                        scan aborted
//   LA / LB                   limit switch triggered
//   CA / CB                   limit switch cleared
//   ADC:<value>               manual TICK response
//   OOR:<seq>,<adc>           out-of-range event  (Arduino also self-stops)
//   SP:<delay>                step delay echo
//   SS_STEPS:<n>              SCAN_SAMPLE_STEPS echo
//   IN:<msg>                  informational
//   WN:<msg>                  warning
//   PONG:<pc_ms>,<ard_ms>
// ─────────────────────────────────────────────────────────────────────────────

// ── Motor / motion config ─────────────────────────────────────────────────────
#define STEP_DELAY_MIN_US        104
#define STEP_DELAY_MAX_US        2500
#define STEP_DELAY_DEFAULT       312
#define STEP_DELAY_SCAN          625
#define STEP_DELTA               10

#define SCAN_SAMPLE_STEPS_DEFAULT  10
#define SCAN_SAMPLE_STEPS_MIN       1
#define SCAN_SAMPLE_STEPS_MAX     200

#define DEBOUNCE_COUNT             5

// ── ADC ───────────────────────────────────────────────────────────────────────
// Arduino R4 supports 14-bit ADC via analogReadResolution(14).
// Full-scale count is 2^14 - 1 = 16383.
#define ADC_BITS   14
#define ADC_MAX    16383

// ── Sensor physics ────────────────────────────────────────────────────────────
// Mirrors the Python constants exactly.  The sensor maps 0V→25mm, 5V→35mm
// linearly across the ADC range.
#define SENSOR_CENTER_MM  30.0f
#define SENSOR_RANGE_MM    5.0f

// Inline conversion: raw ADC → deviation from centre (mm)
// deviation = ((adc / ADC_MAX) * RANGE * 2) - RANGE
inline float adcToDeviation(uint16_t adc) {
  return ((float)adc / ADC_MAX) * (SENSOR_RANGE_MM * 2.0f) - SENSOR_RANGE_MM;
}

// ── OOR defaults ──────────────────────────────────────────────────────────────
#define OOR_ARM_COUNT_DEFAULT      5

// ── Pins ──────────────────────────────────────────────────────────────────────
#define PIN_STEP    6
#define PIN_DIR     4
#define PIN_ENA     5
#define PIN_SW_A    9
#define PIN_SW_B   11
#define PIN_SENSOR A0

#define SWITCH_TRIGGERED HIGH

// ─────────────────────────────────────────────────────────────────────────────
// State
// ─────────────────────────────────────────────────────────────────────────────

enum Mode { MANUAL, SCAN_HOMING, SCAN_MEASURING };
enum Dir  { STOPPED, GOING_RIGHT, GOING_LEFT };

Mode mode      = MANUAL;
Dir  dir       = STOPPED;
int  stepDelay = STEP_DELAY_DEFAULT;
int  scanDelay = STEP_DELAY_SCAN;
int  scanSampleSteps = SCAN_SAMPLE_STEPS_DEFAULT;

long scanSteps = 0;   // cumulative motor steps during measuring pass
long sampleCtr = 0;   // steps since last sample
long seqNum    = 0;   // sample sequence counter

bool limitA_hit    = false;
bool limitB_hit    = false;
int  debounceA_low = 0;
int  debounceB_low = 0;

// ── OOR state ─────────────────────────────────────────────────────────────────
// lo/hi are mm deviations from sensor centre (negative = closer, positive = further).
// The Arduino converts each ADC reading to mm before comparing.
bool  oorEnabled  = false;
float oorLo       = -5.0f;   // default: full sensor range
float oorHi       =  5.0f;
uint16_t oorArmCount = OOR_ARM_COUNT_DEFAULT;
uint16_t oorConsecIn = 0;
bool  oorArmed    = false;
bool  oorFired    = false;   // latched once OOR fires; prevents double-report

// ── RX buffer ─────────────────────────────────────────────────────────────────
#define RX_BUF_SIZE 64
char    rxBuf[RX_BUF_SIZE];
uint8_t rxLen = 0;

// ─────────────────────────────────────────────────────────────────────────────
// setup / loop
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(921600);

  analogReadResolution(ADC_BITS);   // enable 14-bit ADC (0–16383)

  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR,  OUTPUT);
  pinMode(PIN_ENA,  OUTPUT);
  pinMode(PIN_SW_A, INPUT_PULLUP);
  pinMode(PIN_SW_B, INPUT_PULLUP);
  digitalWrite(PIN_ENA, LOW);   // enable driver
  delay(50);

  // ADC warmup — first few reads after boot can be noisy
  for (int i = 0; i < 10; i++) { analogRead(PIN_SENSOR); delay(5); }

  limitA_hit = (digitalRead(PIN_SW_A) == SWITCH_TRIGGERED);
  limitB_hit = (digitalRead(PIN_SW_B) == SWITCH_TRIGGERED);

  Serial.print("IN:gantry ready stepDelay=");
  Serial.print(stepDelay);
  Serial.print(" scanDelay=");
  Serial.print(scanDelay);
  Serial.print(" ss=");
  Serial.println(scanSampleSteps);

  if (limitA_hit) Serial.println("LA");
  if (limitB_hit) Serial.println("LB");
}

void loop() {
  handleSerial();
  checkLimits();

  if (dir != STOPPED) {
    doStep();

    if (mode == SCAN_MEASURING) {
      scanSteps++;
      sampleCtr++;

      if (sampleCtr >= scanSampleSteps) {
        sampleCtr = 0;
        takeSampleAndReport();
      }
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Sample — hot path, called every scanSampleSteps motor steps during scan
// ─────────────────────────────────────────────────────────────────────────────

void takeSampleAndReport() {
  uint16_t adc = (uint16_t)analogRead(PIN_SENSOR);
  long     seq = seqNum++;
  // The OOR logic runs BEFORE reporting so that if OOR fires, we can emit OOR:
  // and SA in the same step, stopping the motor immediately without any PC
  // round-trip.  This is the key fix: in the original split architecture the PC
  // received OOR: and then sent back "A"; here the Arduino self-stops.
  if (oorEnabled && !oorFired) {
    float dev = adcToDeviation(adc);
    bool inRange = (dev >= oorLo && dev <= oorHi);

    if (!oorArmed) {
      // Arming phase: count consecutive in-range readings
      if (inRange) {
        oorConsecIn++;
        if (oorConsecIn >= oorArmCount) {
          oorArmed = true;
          Serial.print("IN:OOR armed at seq=");
          Serial.print(seq);
          Serial.print(" dev=");
          Serial.println(dev, 3);
        }
      } else {
        oorConsecIn = 0;   // reset — still in pre-surface region
      }
    } else {
      // Armed phase: first out-of-range reading triggers abort
      if (!inRange) {
        oorFired = true;
        float firedDev = dev;

        // Report the OOR event: seq, raw ADC, and the mm deviation that triggered it
        Serial.print("OOR:");
        Serial.print(seq);
        Serial.print(',');
        Serial.print(adc);
        Serial.print(',');
        Serial.println(firedDev, 3);
        Serial.flush();   // ensure OOR: is fully transmitted before SA

        // Self-abort: stop immediately, no PC round-trip needed
        abortScan();
        return;   // skip ST: for this sample — scan is over
      }
    }
  }

  // ── Step telemetry: seq, cumulative steps, raw ADC ──────────────────────────
  // Format: ST:<seq>,<steps>,<adc>
  Serial.print("ST:");
  Serial.print(seq);
  Serial.print(',');
  Serial.print(scanSteps);
  Serial.print(',');
  Serial.println(adc);
}

// Manual live read (TICK command, outside scan only)
void takeSampleManual() {
  uint16_t adc = (uint16_t)analogRead(PIN_SENSOR);
  Serial.print("ADC:");
  Serial.println(adc);
}

// ─────────────────────────────────────────────────────────────────────────────
// Scan lifecycle
// ─────────────────────────────────────────────────────────────────────────────

void startScan() {
  // Reset motion state
  stepDelay     = scanDelay;
  mode          = SCAN_HOMING;
  limitA_hit    = false;
  debounceA_low = 0;
  dir           = GOING_LEFT;
  setDir(false);

  // Reset sensor / OOR state
  seqNum      = 0;
  sampleCtr   = 0;
  scanSteps   = 0;
  oorConsecIn = 0;
  oorArmed    = false;
  oorFired    = false;

  Serial.println("IN:homing");
}

void beginMeasuring() {
  scanSteps = 0;
  sampleCtr = 0;
  mode      = SCAN_MEASURING;
  dir       = GOING_RIGHT;
  setDir(true);
  Serial.println("SS");
}

void finishScan() {
  long total = seqNum;
  dir        = STOPPED;
  mode       = MANUAL;
  stepDelay  = STEP_DELAY_DEFAULT;
  debounceB_low = 0;
  Serial.print("SD:");
  Serial.println(total);
}

void abortScan() {
  dir       = STOPPED;
  mode      = MANUAL;
  stepDelay = STEP_DELAY_DEFAULT;
  Serial.println("SA");
  Serial.flush();
}

// ─────────────────────────────────────────────────────────────────────────────
// Limit switches (debounced)
// ─────────────────────────────────────────────────────────────────────────────

void checkLimits() {
  bool a = (digitalRead(PIN_SW_A) == SWITCH_TRIGGERED);
  bool b = (digitalRead(PIN_SW_B) == SWITCH_TRIGGERED);

  // ── Left limit ────────────────────────────────────────────────────────────
  if (a && !limitA_hit) {
    limitA_hit    = true;
    debounceA_low = 0;
    if (mode == SCAN_HOMING) {
      dir = STOPPED;
      beginMeasuring();
    } else if (dir == GOING_LEFT) {
      dir = STOPPED;
      Serial.println("LA");
    }
  }
  if (!a && limitA_hit) {
    if (++debounceA_low >= DEBOUNCE_COUNT) {
      limitA_hit    = false;
      debounceA_low = 0;
      if (mode == MANUAL) Serial.println("CA");
    }
  } else if (a) {
    debounceA_low = 0;
  }

  // ── Right limit ───────────────────────────────────────────────────────────
  if (b && !limitB_hit) {
    limitB_hit    = true;
    debounceB_low = 0;
    if (mode == SCAN_MEASURING) {
      finishScan();
    } else if (dir == GOING_RIGHT) {
      dir = STOPPED;
      Serial.println("LB");
    }
  }
  if (!b && limitB_hit) {
    if (++debounceB_low >= DEBOUNCE_COUNT) {
      limitB_hit    = false;
      debounceB_low = 0;
      if (mode == MANUAL) Serial.println("CB");
    }
  } else if (b) {
    debounceB_low = 0;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Motion
// ─────────────────────────────────────────────────────────────────────────────

void setDir(bool right) {
  digitalWrite(PIN_DIR, right ? HIGH : LOW);
  delayMicroseconds(5);
}

void doStep() {
  digitalWrite(PIN_STEP, HIGH);
  // digitalWrite(LED_BUILTIN, HIGH);
  delayMicroseconds(stepDelay);
  digitalWrite(PIN_STEP, LOW);
  // digitalWrite(LED_BUILTIN, LOW);
  delayMicroseconds(stepDelay);
}

// ─────────────────────────────────────────────────────────────────────────────
// Serial receive
// ─────────────────────────────────────────────────────────────────────────────

void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxLen > 0) {
        rxBuf[rxLen] = '\0';
        processCommand();
        rxLen = 0;
      }
    } else if (rxLen < RX_BUF_SIZE - 1) {
      rxBuf[rxLen++] = c;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// String helpers (no String class — zero heap fragmentation)
// ─────────────────────────────────────────────────────────────────────────────

static bool bufEq(const char* s) {
  uint8_t i = 0;
  while (s[i] && rxBuf[i]) { if (s[i] != rxBuf[i]) return false; i++; }
  return s[i] == '\0' && rxBuf[i] == '\0';
}

static bool bufStartsWith(const char* s) {
  uint8_t i = 0;
  while (s[i]) { if (rxBuf[i] != s[i]) return false; i++; }
  return true;
}

static long parseLongAt(uint8_t offset) {
  long v = 0; bool neg = false; uint8_t i = offset;
  if (rxBuf[i] == '-') { neg = true; i++; }
  while (rxBuf[i] >= '0' && rxBuf[i] <= '9') { v = v * 10 + (rxBuf[i] - '0'); i++; }
  return neg ? -v : v;
}

// Parse up to maxFields comma-separated longs after the first ':' in rxBuf.
static uint8_t parseCSVAfterColon(long* out, uint8_t maxFields) {
  uint8_t fi = 0, i = 0;
  while (rxBuf[i] && rxBuf[i] != ':') i++;
  if (!rxBuf[i]) return 0;
  i++;
  while (fi < maxFields && rxBuf[i]) {
    long v = 0; bool neg = false;
    if (rxBuf[i] == '-') { neg = true; i++; }
    if (rxBuf[i] < '0' || rxBuf[i] > '9') break;
    while (rxBuf[i] >= '0' && rxBuf[i] <= '9') { v = v * 10 + (rxBuf[i] - '0'); i++; }
    out[fi++] = neg ? -v : v;
    if (rxBuf[i] == ',') i++;
  }
  return fi;
}

// Parse up to maxFields comma-separated floats after the first ':' in rxBuf.
// Handles optional leading '-' and one decimal point.
static uint8_t parseFloatCSVAfterColon(float* out, uint8_t maxFields) {
  uint8_t fi = 0, i = 0;
  while (rxBuf[i] && rxBuf[i] != ':') i++;
  if (!rxBuf[i]) return 0;
  i++;
  while (fi < maxFields && rxBuf[i]) {
    bool neg = false;
    if (rxBuf[i] == '-') { neg = true; i++; }
    if (rxBuf[i] < '0' || rxBuf[i] > '9') break;
    long intPart = 0;
    while (rxBuf[i] >= '0' && rxBuf[i] <= '9') { intPart = intPart * 10 + (rxBuf[i] - '0'); i++; }
    float v = (float)intPart;
    if (rxBuf[i] == '.') {
      i++;
      float frac = 0.1f;
      while (rxBuf[i] >= '0' && rxBuf[i] <= '9') { v += (rxBuf[i] - '0') * frac; frac *= 0.1f; i++; }
    }
    out[fi++] = neg ? -v : v;
    if (rxBuf[i] == ',') i++;
  }
  return fi;
}

// ─────────────────────────────────────────────────────────────────────────────
// Command dispatcher
// ─────────────────────────────────────────────────────────────────────────────

void processCommand() {

  // ── PING ──────────────────────────────────────────────────────────────────
  if (bufStartsWith("PING:")) {
    Serial.print("PONG:"); Serial.print(rxBuf + 5);
    Serial.print(',');     Serial.println(millis());
    return;
  }

  // ── START:<en>,<lo_mm>,<hi_mm>,<armCount>  — configure OOR for next scan ──
  // lo/hi are mm deviations from sensor centre (e.g. -3.5,3.5).
  if (bufStartsWith("START")) {
    if (mode != MANUAL) { Serial.println("WN:scan active"); return; }
    // Parse en and armCount as longs, lo/hi as floats.
    // Format: START:<en>,<lo>,<hi>,<arm>
    // We parse all four as floats for simplicity; en and arm are cast to int.
    float fields[4] = {0, -SENSOR_RANGE_MM, SENSOR_RANGE_MM, (float)OOR_ARM_COUNT_DEFAULT};
    parseFloatCSVAfterColon(fields, 4);
    oorEnabled  = ((int)fields[0] == 1);
    oorLo       = fields[1];
    oorHi       = fields[2];
    oorArmCount = (uint16_t)max(1, (int)fields[3]);
    Serial.print("IN:oor en="); Serial.print(oorEnabled ? 1 : 0);
    Serial.print(" lo=");       Serial.print(oorLo, 3);
    Serial.print(" hi=");       Serial.print(oorHi, 3);
    Serial.print(" arm=");      Serial.println(oorArmCount);
    return;
  }

  // ── TICK — manual live ADC read ───────────────────────────────────────────
  if (bufEq("TICK")) {
    if (mode == MANUAL) takeSampleManual();
    else Serial.println("WN:scan active");
    return;
  }

  // ── A — abort scan (always accepted) ─────────────────────────────────────
  if (bufEq("A")) {
    if (mode != MANUAL) abortScan();
    return;
  }

  // ── All remaining commands rejected during scan ───────────────────────────
  if (mode != MANUAL) { Serial.println("WN:scan active"); return; }

  if      (bufEq("R")) {
    if (limitB_hit) { Serial.println("WN:right limit"); return; }
    dir = GOING_RIGHT; setDir(true); Serial.println("OK:R");
  }
  else if (bufEq("L")) {
    if (limitA_hit) { Serial.println("WN:left limit"); return; }
    dir = GOING_LEFT; setDir(false); Serial.println("OK:L");
  }
  else if (bufEq("S"))  { dir = STOPPED; Serial.println("OK:S"); }
  else if (bufEq("X"))  { startScan(); }
  else if (bufEq("+"))  {
    stepDelay = max(STEP_DELAY_MIN_US, stepDelay - STEP_DELTA);
    Serial.print("SP:"); Serial.println(stepDelay);
  }
  else if (bufEq("-"))  {
    stepDelay = min(STEP_DELAY_MAX_US, stepDelay + STEP_DELTA);
    Serial.print("SP:"); Serial.println(stepDelay);
  }
  else if (rxBuf[0] == 'V' && rxLen > 1) {
    stepDelay = (int)constrain(parseLongAt(1), STEP_DELAY_MIN_US, STEP_DELAY_MAX_US);
    Serial.print("SP:"); Serial.println(stepDelay);
  }
  else if (rxBuf[0] == 'W' && rxLen > 1) {
    scanDelay = (int)constrain(parseLongAt(1), STEP_DELAY_MIN_US, STEP_DELAY_MAX_US);
    Serial.print("IN:scanDelay="); Serial.println(scanDelay);
  }
  else if (rxBuf[0] == 'B' && rxLen > 1) {
    scanSampleSteps = (int)constrain(parseLongAt(1), SCAN_SAMPLE_STEPS_MIN, SCAN_SAMPLE_STEPS_MAX);
    Serial.print("SS_STEPS:"); Serial.println(scanSampleSteps);
  }
  else {
    Serial.print("WN:unknown "); Serial.println(rxBuf);
  }
}
