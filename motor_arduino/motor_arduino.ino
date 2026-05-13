// ─────────────────────────────────────────────────────────────────────────────
// Gantry Motor Controller v5 — Arduino R4 (wired USB-Serial)
//
// Changes from v4:
//   - ST now includes sequence number:  ST:<seq>,<steps>
//   - SCAN_SAMPLE_STEPS is a variable (changeable via serial command)
//   - Right-limit CB spam bug fixed: limitB_hit only clears when
//     gantry has actually moved away (pin LOW for DEBOUNCE_COUNT loops)
//   - ENA active LOW (matches v4)
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
// Resolution (mm) = SCAN_SAMPLE_STEPS / steps_per_mm
// steps_per_mm = (FULL_STEPS × MICROSTEP) / (PULLEY_TEETH × PITCH_MM)
// Default 10 → 1mm resolution at 80T pulley, 8x microstep
#define SCAN_SAMPLE_STEPS_DEFAULT  10
#define SCAN_SAMPLE_STEPS_MIN       1
#define SCAN_SAMPLE_STEPS_MAX     200

// ── Limit switch debounce ─────────────────────────────────────────────────────
// Pin must read consistently for this many loop() calls before state changes.
// Prevents the right-limit CB spam caused by mechanical bounce after scan ends.
#define DEBOUNCE_COUNT  5

// ── Pins ──────────────────────────────────────────────────────────────────────
#define PIN_STEP  6
#define PIN_DIR   4
#define PIN_ENA   5
#define PIN_SW_A  9
#define PIN_SW_B  11

// ── Limit switch ─────────────────────────────────────────────────────────────
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

// Scan state
long  scanSteps  = 0;   // raw step counter
long  sampleCtr  = 0;   // steps since last sample
long  seqNum     = 0;   // sequence number sent to PC

// Limit state with debounce counters
bool  limitA_hit     = false;
bool  limitB_hit     = false;
int   debounceA_low  = 0;   // consecutive LOW readings for SW_A
int   debounceB_low  = 0;   // consecutive LOW readings for SW_B

// RX buffer
#define RX_BUF_SIZE 32
char    rxBuf[RX_BUF_SIZE];
uint8_t rxLen = 0;

// ─────────────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);

  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR,  OUTPUT);
  pinMode(PIN_ENA,  OUTPUT);
  pinMode(PIN_SW_A, INPUT_PULLUP);
  pinMode(PIN_SW_B, INPUT_PULLUP);
  digitalWrite(PIN_ENA, LOW);   // enable driver (active LOW)
  delay(50);

  // Initialise limit state from actual pin so no spurious messages on startup
  limitA_hit = (digitalRead(PIN_SW_A) == SWITCH_TRIGGERED);
  limitB_hit = (digitalRead(PIN_SW_B) == SWITCH_TRIGGERED);

  Serial.println("IN:motor v5 ready ms=" + String(MICROSTEP_DIVISOR) +
                 " dmin=" + String(STEP_DELAY_MIN_US) +
                 " dmax=" + String(STEP_DELAY_MAX_US) +
                 " ss=" + String(scanSampleSteps));
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
        sendStepTelemetry();
      }
    }
  }
}

// ── Step telemetry ────────────────────────────────────────────────────────────
void sendStepTelemetry() {
  // ST:<seq>,<cumulative_steps>
  // seq is the index PC uses to pair with sensor readings — no timestamps
  Serial.println("ST:" + String(seqNum) + "," + String(scanSteps));
  seqNum++;
}

// ── Serial receive ────────────────────────────────────────────────────────────
void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxLen > 0) {
        rxBuf[rxLen] = '\0';
        processCommand(String(rxBuf));
        rxLen = 0;
      }
    } else if (rxLen < RX_BUF_SIZE - 1) {
      rxBuf[rxLen++] = c;
    }
  }
}

// ── Command parser ────────────────────────────────────────────────────────────
void processCommand(const String& cmd) {
  if (cmd.startsWith("PING:")) {
    Serial.println("PONG:" + cmd.substring(5) + "," + String(millis()));
    return;
  }
  if (cmd.startsWith("V")) {
    stepDelay = constrain(cmd.substring(1).toInt(),
                          STEP_DELAY_MIN_US, STEP_DELAY_MAX_US);
    Serial.println("SP:" + String(stepDelay)); return;
  }
  if (cmd.startsWith("W")) {
    scanDelay = constrain(cmd.substring(1).toInt(),
                          STEP_DELAY_MIN_US, STEP_DELAY_MAX_US);
    Serial.println("IN:scanDelay=" + String(scanDelay)); return;
  }
  if (cmd.startsWith("B")) {
    // Set SCAN_SAMPLE_STEPS
    int ss = constrain(cmd.substring(1).toInt(),
                       SCAN_SAMPLE_STEPS_MIN, SCAN_SAMPLE_STEPS_MAX);
    scanSampleSteps = ss;
    Serial.println("SS_STEPS:" + String(scanSampleSteps)); return;
  }

  if (mode != MANUAL && cmd != "A") {
    Serial.println("WN:scan active"); return;
  }

  if      (cmd == "R") {
    if (limitB_hit) { Serial.println("WN:right limit"); return; }
    dir = GOING_RIGHT; setDir(true); Serial.println("OK:R");
  }
  else if (cmd == "L") {
    if (limitA_hit) { Serial.println("WN:left limit"); return; }
    dir = GOING_LEFT; setDir(false); Serial.println("OK:L");
  }
  else if (cmd == "S") { dir = STOPPED; Serial.println("OK:S"); }
  else if (cmd == "+") {
    stepDelay = max(STEP_DELAY_MIN_US, stepDelay - STEP_DELTA);
    Serial.println("SP:" + String(stepDelay));
  }
  else if (cmd == "-") {
    stepDelay = min(STEP_DELAY_MAX_US, stepDelay + STEP_DELTA);
    Serial.println("SP:" + String(stepDelay));
  }
  else if (cmd == "X") { startScan(); }
  else if (cmd == "A") { abortScan(); }
  else { Serial.println("WN:unknown " + cmd); }
}

// ── Scan control ──────────────────────────────────────────────────────────────
void startScan() {
  stepDelay = scanDelay;
  mode      = SCAN_HOMING;
  limitA_hit = false;
  debounceA_low = 0;
  dir       = GOING_LEFT;
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
  delay(50);
  Serial.println("SS");
}

void finishScan() {
  long total = seqNum;   // total ST packets sent
  dir        = STOPPED;
  mode       = MANUAL;
  stepDelay  = STEP_DELAY_DEFAULT;
  // Reset debounce so right limit clears cleanly after scan
  debounceB_low = 0;
  Serial.println("SD:" + String(total));
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
    debounceA_low = 0;   // reset counter if pin bounces back HIGH
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
  // FIX: only clear limitB and send CB when gantry has genuinely moved away.
  // Require DEBOUNCE_COUNT consecutive LOW readings before clearing.
  // This prevents the infinite CB spam caused by the switch bouncing
  // immediately after finishScan() sets mode=MANUAL while gantry is still
  // physically pressing the switch.
  if (!b && limitB_hit) {
    debounceB_low++;
    if (debounceB_low >= DEBOUNCE_COUNT) {
      limitB_hit    = false;
      debounceB_low = 0;
      if (mode == MANUAL) Serial.println("CB");
    }
  } else if (b) {
    debounceB_low = 0;   // pin bounced back HIGH — reset counter
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
