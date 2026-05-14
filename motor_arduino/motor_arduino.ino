// ─────────────────────────────────────────────────────────────────────────────
// Gantry Motor Controller — Arduino R4 (wired USB-Serial)
//
// KEY FIX: Removed all Arduino String heap allocation from hot-path functions.
// sendStepTelemetry() is called every scanSampleSteps steps throughout the
// entire scan.  Each String("ST:" + String(n) + ...) does a heap malloc/free.
// After hundreds of calls the heap fragments — allocations stall for tens of
// milliseconds, loop() stops running fast, checkLimits() gets skipped, and
// the right-limit trigger is missed or SD: is never sent.
//
// Solution: replace EVERY String concatenation with chained Serial.print()
// calls.  No heap allocation, no fragmentation, deterministic timing.
//
// Wiring:
//   STEP → pin 6    DIR → pin 4    ENA → pin 5 (LOW = enabled)
//   SW_A → pin 9   (LEFT  end, INPUT_PULLUP, NC → GND)
//   SW_B → pin 11  (RIGHT end, INPUT_PULLUP, NC → GND)
//
// PC → Arduino (newline terminated):
//   R / L / S / + / - / X / A
//   Vnnnn\n      set manual stepDelay µs
//   Wnnnn\n      set scan   stepDelay µs
//   Bnn\n        set SCAN_SAMPLE_STEPS (e.g. B10)
//   PING:nnn\n   clock sync (echoes PONG)
//
// Arduino → PC:
//   ST:<seq>,<steps>\n    step telemetry — seq increments each sample
//   SS\n                  scan start
//   SD:<seq_total>\n      scan done, total sequences sent
//   SA\n                  scan aborted
//   LA / LB               limit hit
//   CA / CB               limit cleared
//   SP:<delay>\n          stepDelay echo
//   SS_STEPS:<n>\n        echo of current SCAN_SAMPLE_STEPS
//   OK:<cmd>\n            ack
//   WN:<msg>\n            warning
//   IN:<msg>\n            info
//   PONG:<pc_ms>,<ard_ms>\n
// ─────────────────────────────────────────────────────────────────────────────

// ── Motor config ──────────────────────────────────────────────────────────────
#define MICROSTEP_DIVISOR    8
#define STEP_DELAY_MIN_US    104
#define STEP_DELAY_MAX_US    2500
#define STEP_DELAY_DEFAULT   312
#define STEP_DELAY_SCAN      625
#define STEP_DELTA           10

// ── Scan sampling ─────────────────────────────────────────────────────────────
#define SCAN_SAMPLE_STEPS_DEFAULT  10
#define SCAN_SAMPLE_STEPS_MIN       1
#define SCAN_SAMPLE_STEPS_MAX     200

// ── Limit switch debounce ─────────────────────────────────────────────────────
#define DEBOUNCE_COUNT  5

// ── Pins ──────────────────────────────────────────────────────────────────────
#define PIN_STEP  6
#define PIN_DIR   4
#define PIN_ENA   5
#define PIN_SW_A  9
#define PIN_SW_B  11

// NC + INPUT_PULLUP:
//   Pin HIGH = switch open  = TRIGGERED (gantry pressed switch open)
//   Pin LOW  = switch closed = not triggered
#define SWITCH_TRIGGERED  HIGH

// ─────────────────────────────────────────────────────────────────────────────
enum Mode { MANUAL, SCAN_HOMING, SCAN_MEASURING };
enum Dir  { STOPPED, GOING_RIGHT, GOING_LEFT };

Mode  mode      = MANUAL;
Dir   dir       = STOPPED;
int   stepDelay = STEP_DELAY_DEFAULT;
int   scanDelay = STEP_DELAY_SCAN;
int   scanSampleSteps = SCAN_SAMPLE_STEPS_DEFAULT;

long  scanSteps  = 0;
long  sampleCtr  = 0;
long  seqNum     = 0;

bool  limitA_hit     = false;
bool  limitB_hit     = false;
int   debounceA_low  = 0;
int   debounceB_low  = 0;

// ── RX buffer ────────────────────────────────────────────────────────────────
// Using a plain char array throughout — no String objects in command parsing.
#define RX_BUF_SIZE 32
char    rxBuf[RX_BUF_SIZE];
uint8_t rxLen = 0;

// ─────────────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(921600);

  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR,  OUTPUT);
  pinMode(PIN_ENA,  OUTPUT);
  pinMode(PIN_SW_A, INPUT_PULLUP);
  pinMode(PIN_SW_B, INPUT_PULLUP);
  digitalWrite(PIN_ENA, LOW);   // enable driver (active LOW)
  delay(50);

  limitA_hit = (digitalRead(PIN_SW_A) == SWITCH_TRIGGERED);
  limitB_hit = (digitalRead(PIN_SW_B) == SWITCH_TRIGGERED);

  // Startup banner — String concatenation OK here (runs once)
  Serial.print("IN:motor ready ms=");
  Serial.print(MICROSTEP_DIVISOR);
  Serial.print(" dmin=");
  Serial.print(STEP_DELAY_MIN_US);
  Serial.print(" dmax=");
  Serial.print(STEP_DELAY_MAX_US);
  Serial.print(" ss=");
  Serial.println(scanSampleSteps);

  if (limitA_hit) Serial.println("LA");
  if (limitB_hit) Serial.println("LB");
}

// ─────────────────────────────────────────────────────────────────────────────

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
        sendStepTelemetry();
      }
    }
  }
}

// ── Step telemetry ────────────────────────────────────────────────────────────
// HOT PATH — called every scanSampleSteps motor steps for the entire scan.
// Must not allocate heap.  Use chained Serial.print() instead of String.
void sendStepTelemetry() {
  Serial.print("ST:");
  Serial.print(seqNum);
  Serial.print(',');
  Serial.println(scanSteps);
  seqNum++;
}

// ── Serial receive ────────────────────────────────────────────────────────────
// Accumulate chars into rxBuf; process on newline.
// Operates on raw char arrays — no String heap allocation.
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

// ── Helpers: parse integer from rxBuf starting at offset ─────────────────────
static long parseLong(uint8_t offset) {
  long v = 0;
  bool neg = false;
  uint8_t i = offset;
  if (rxBuf[i] == '-') { neg = true; i++; }
  while (rxBuf[i] >= '0' && rxBuf[i] <= '9') {
    v = v * 10 + (rxBuf[i] - '0');
    i++;
  }
  return neg ? -v : v;
}

static bool bufEq(const char* s) {
  // Case-sensitive exact match against rxBuf
  uint8_t i = 0;
  while (s[i] && rxBuf[i]) {
    if (s[i] != rxBuf[i]) return false;
    i++;
  }
  return s[i] == '\0' && rxBuf[i] == '\0';
}

static bool bufStartsWith(const char* s) {
  uint8_t i = 0;
  while (s[i]) {
    if (rxBuf[i] != s[i]) return false;
    i++;
  }
  return true;
}

// ── Command parser ────────────────────────────────────────────────────────────
// No String objects — all comparisons on raw rxBuf char array.
void processCommand() {

  if (bufStartsWith("PING:")) {
    // PONG:<echo>,<millis>
    Serial.print("PONG:");
    Serial.print(rxBuf + 5);   // echo the token after PING:
    Serial.print(',');
    Serial.println(millis());
    return;
  }

  if (rxBuf[0] == 'V' && rxLen > 1) {
    stepDelay = (int)constrain(parseLong(1), STEP_DELAY_MIN_US, STEP_DELAY_MAX_US);
    Serial.print("SP:"); Serial.println(stepDelay);
    return;
  }
  if (rxBuf[0] == 'W' && rxLen > 1) {
    scanDelay = (int)constrain(parseLong(1), STEP_DELAY_MIN_US, STEP_DELAY_MAX_US);
    Serial.print("IN:scanDelay="); Serial.println(scanDelay);
    return;
  }
  if (rxBuf[0] == 'B' && rxLen > 1) {
    int ss = (int)constrain(parseLong(1), SCAN_SAMPLE_STEPS_MIN, SCAN_SAMPLE_STEPS_MAX);
    scanSampleSteps = ss;
    Serial.print("SS_STEPS:"); Serial.println(scanSampleSteps);
    return;
  }

  // During a scan, only A (abort) is accepted
  if (mode != MANUAL && !bufEq("A")) {
    Serial.println("WN:scan active");
    return;
  }

  if      (bufEq("R")) {
    if (limitB_hit) { Serial.println("WN:right limit"); return; }
    dir = GOING_RIGHT; setDir(true); Serial.println("OK:R");
  }
  else if (bufEq("L")) {
    if (limitA_hit) { Serial.println("WN:left limit"); return; }
    dir = GOING_LEFT; setDir(false); Serial.println("OK:L");
  }
  else if (bufEq("S")) { dir = STOPPED; Serial.println("OK:S"); }
  else if (bufEq("+")) {
    stepDelay = max(STEP_DELAY_MIN_US, stepDelay - STEP_DELTA);
    Serial.print("SP:"); Serial.println(stepDelay);
  }
  else if (bufEq("-")) {
    stepDelay = min(STEP_DELAY_MAX_US, stepDelay + STEP_DELTA);
    Serial.print("SP:"); Serial.println(stepDelay);
  }
  else if (bufEq("X")) { startScan(); }
  else if (bufEq("A")) { abortScan(); }
  else {
    Serial.print("WN:unknown ");
    Serial.println(rxBuf);
  }
}

// ── Scan control ──────────────────────────────────────────────────────────────
void startScan() {
  stepDelay     = scanDelay;
  mode          = SCAN_HOMING;
  limitA_hit    = false;
  debounceA_low = 0;
  dir           = GOING_LEFT;
  setDir(false);
  Serial.println("IN:homing");
}

void abortScan() {
  dir       = STOPPED;
  mode      = MANUAL;
  stepDelay = STEP_DELAY_DEFAULT;
  Serial.println("SA");
}

void beginMeasuring() {
  scanSteps = 0;
  sampleCtr = 0;
  seqNum    = 0;
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

// ── Limit switches — debounced ────────────────────────────────────────────────
void checkLimits() {
  bool a = (digitalRead(PIN_SW_A) == SWITCH_TRIGGERED);
  bool b = (digitalRead(PIN_SW_B) == SWITCH_TRIGGERED);

  // ── Left limit (SW_A) ────────────────────────────────────────────────────
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
    debounceA_low++;
    if (debounceA_low >= DEBOUNCE_COUNT) {
      limitA_hit    = false;
      debounceA_low = 0;
      if (mode == MANUAL) Serial.println("CA");
    }
  } else if (a) {
    debounceA_low = 0;
  }

  // ── Right limit (SW_B) ────────────────────────────────────────────────────
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
    debounceB_low++;
    if (debounceB_low >= DEBOUNCE_COUNT) {
      limitB_hit    = false;
      debounceB_low = 0;
      if (mode == MANUAL) Serial.println("CB");
    }
  } else if (b) {
    debounceB_low = 0;
  }
}

// ── Motion ────────────────────────────────────────────────────────────────────
void setDir(bool right) {
  digitalWrite(PIN_DIR, right ? HIGH : LOW);
  delayMicroseconds(5);
}

void doStep() {
  digitalWrite(PIN_STEP, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);
  delayMicroseconds(stepDelay);
  digitalWrite(PIN_STEP, LOW);
  digitalWrite(LED_BUILTIN, LOW);
  delayMicroseconds(stepDelay);
}
