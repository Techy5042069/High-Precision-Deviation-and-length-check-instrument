// motor.ino — Motor Arduino R4 (wired USB-Serial)
//
// Controls stepper motor only. ADC is handled by a separate sensor Arduino.
// ST: carries seq and steps — no ADC field.
// OOR abort arrives from Python over serial ("A" command).
//
// Wiring:
//   STEP → pin 6   DIR → pin 4   ENA → pin 5 (LOW = enabled)
//   SW_A → pin 9  (LEFT  limit, INPUT_PULLUP)
//   SW_B → pin 11 (RIGHT limit, INPUT_PULLUP)
//
// PC → Arduino:
//   R / L / S / + / - / A / X
//   V<delay>    manual step delay µs
//   W<delay>    scan   step delay µs
//   B<steps>    SCAN_SAMPLE_STEPS
//   PING:<ms>
//
// Arduino → PC:
//   ST:<seq>,<steps>
//   SS / SD:<total> / SA
//   LA / LB / CA / CB
//   SP:<delay>  SS_STEPS:<n>  IN:<msg>  WN:<msg>  PONG:<pc>,<ard>

#define STEP_DELAY_MIN_US         104
#define STEP_DELAY_MAX_US         2500
#define STEP_DELAY_DEFAULT        312
#define STEP_DELAY_SCAN           625
#define STEP_DELTA                10
#define SCAN_SAMPLE_STEPS_DEFAULT  10
#define SCAN_SAMPLE_STEPS_MIN       1
#define SCAN_SAMPLE_STEPS_MAX     200
#define DEBOUNCE_COUNT              5

#define PIN_STEP    6
#define PIN_DIR     4
#define PIN_ENA     5
#define PIN_SW_A    9
#define PIN_SW_B   11
#define SWITCH_TRIGGERED HIGH

enum Mode { MANUAL, SCAN_HOMING, SCAN_MEASURING };
enum Dir  { STOPPED, GOING_RIGHT, GOING_LEFT };

Mode mode      = MANUAL;
Dir  dir       = STOPPED;
int  stepDelay = STEP_DELAY_DEFAULT;
int  scanDelay = STEP_DELAY_SCAN;
int  scanSampleSteps = SCAN_SAMPLE_STEPS_DEFAULT;
long scanSteps = 0;
long sampleCtr = 0;
long seqNum    = 0;
bool limitA_hit = false;
bool limitB_hit = false;
int  debounceA_low = 0;
int  debounceB_low = 0;

#define RX_BUF_SIZE 64
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
  digitalWrite(PIN_ENA, LOW);
  delay(50);

  limitA_hit = (digitalRead(PIN_SW_A) == SWITCH_TRIGGERED);
  limitB_hit = (digitalRead(PIN_SW_B) == SWITCH_TRIGGERED);

  Serial.print("IN:motor ready delay=");  Serial.print(stepDelay);
  Serial.print(" scan=");                 Serial.print(scanDelay);
  Serial.print(" ss=");                   Serial.println(scanSampleSteps);

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
        // ST: has two fields only — Python pairs it with the latest UDP ADC reading
        Serial.print("ST:"); Serial.print(seqNum);
        Serial.print(',');   Serial.println(scanSteps);
        seqNum++;
      }
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Scan lifecycle
// ─────────────────────────────────────────────────────────────────────────────

void startScan() {
  stepDelay = scanDelay; mode = SCAN_HOMING;
  limitA_hit = false; debounceA_low = 0;
  dir = GOING_LEFT; setDir(false);
  seqNum = 0; sampleCtr = 0; scanSteps = 0;
  Serial.println("IN:homing");
}

void beginMeasuring() {
  scanSteps = 0; sampleCtr = 0;
  mode = SCAN_MEASURING; dir = GOING_RIGHT; setDir(true);
  Serial.println("SS");
}

void finishScan() {
  long total = seqNum;
  dir = STOPPED; mode = MANUAL; stepDelay = STEP_DELAY_DEFAULT;
  debounceB_low = 0;
  Serial.print("SD:"); Serial.println(total);
}

void abortScan() {
  dir = STOPPED; mode = MANUAL; stepDelay = STEP_DELAY_DEFAULT;
  Serial.println("SA");
}

// ─────────────────────────────────────────────────────────────────────────────
// Limit switches
// ─────────────────────────────────────────────────────────────────────────────

void checkLimits() {
  bool a = (digitalRead(PIN_SW_A) == SWITCH_TRIGGERED);
  bool b = (digitalRead(PIN_SW_B) == SWITCH_TRIGGERED);

  if (a && !limitA_hit) {
    limitA_hit = true; debounceA_low = 0;
    if      (mode == SCAN_HOMING) { dir = STOPPED; beginMeasuring(); }
    else if (dir  == GOING_LEFT)  { dir = STOPPED; Serial.println("LA"); }
  }
  if (!a && limitA_hit) {
    if (++debounceA_low >= DEBOUNCE_COUNT) {
      limitA_hit = false; debounceA_low = 0;
      if (mode == MANUAL) Serial.println("CA");
    }
  } else if (a) { debounceA_low = 0; }

  // SD: must fire even if Python already sent abort — OOR can fire on the last
  // reading as the gantry leaves the surface, then the right limit is hit
  // milliseconds later. scan_done takes priority over scan_aborted in Python.
  if (b && !limitB_hit) {
    limitB_hit = true; debounceB_low = 0;
    if      (mode == SCAN_MEASURING) { finishScan(); }
    else if (dir  == GOING_RIGHT)    { dir = STOPPED; Serial.println("LB"); }
  }
  if (!b && limitB_hit) {
    if (++debounceB_low >= DEBOUNCE_COUNT) {
      limitB_hit = false; debounceB_low = 0;
      if (mode == MANUAL) Serial.println("CB");
    }
  } else if (b) { debounceB_low = 0; }
}

// ─────────────────────────────────────────────────────────────────────────────
// Serial receive
// ─────────────────────────────────────────────────────────────────────────────

void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxLen > 0) { rxBuf[rxLen] = '\0'; processCommand(); rxLen = 0; }
    } else if (rxLen < RX_BUF_SIZE - 1) {
      rxBuf[rxLen++] = c;
    }
  }
}

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
static long parseLongAt(uint8_t o) {
  long v = 0; bool neg = false; uint8_t i = o;
  if (rxBuf[i] == '-') { neg = true; i++; }
  while (rxBuf[i] >= '0' && rxBuf[i] <= '9') { v = v*10+(rxBuf[i]-'0'); i++; }
  return neg ? -v : v;
}

void processCommand() {
  if (bufStartsWith("PING:")) {
    Serial.print("PONG:"); Serial.print(rxBuf+5);
    Serial.print(',');     Serial.println(millis()); return;
  }
  // Abort always accepted
  if (bufEq("A")) { if (mode != MANUAL) abortScan(); return; }
  if (mode != MANUAL) { Serial.println("WN:scan active"); return; }

  if      (bufEq("R")) { if (limitB_hit) { Serial.println("WN:right limit"); return; } dir=GOING_RIGHT; setDir(true);  Serial.println("OK:R"); }
  else if (bufEq("L")) { if (limitA_hit) { Serial.println("WN:left limit");  return; } dir=GOING_LEFT;  setDir(false); Serial.println("OK:L"); }
  else if (bufEq("S")) { dir=STOPPED; Serial.println("OK:S"); }
  else if (bufEq("X")) { startScan(); }
  else if (bufEq("+")) { stepDelay=max(STEP_DELAY_MIN_US,stepDelay-STEP_DELTA); Serial.print("SP:"); Serial.println(stepDelay); }
  else if (bufEq("-")) { stepDelay=min(STEP_DELAY_MAX_US,stepDelay+STEP_DELTA); Serial.print("SP:"); Serial.println(stepDelay); }
  else if (rxBuf[0]=='V'&&rxLen>1) { stepDelay=(int)constrain(parseLongAt(1),STEP_DELAY_MIN_US,STEP_DELAY_MAX_US); Serial.print("SP:"); Serial.println(stepDelay); }
  else if (rxBuf[0]=='W'&&rxLen>1) { scanDelay=(int)constrain(parseLongAt(1),STEP_DELAY_MIN_US,STEP_DELAY_MAX_US); Serial.print("IN:scanDelay="); Serial.println(scanDelay); }
  else if (rxBuf[0]=='B'&&rxLen>1) { scanSampleSteps=(int)constrain(parseLongAt(1),SCAN_SAMPLE_STEPS_MIN,SCAN_SAMPLE_STEPS_MAX); Serial.print("SS_STEPS:"); Serial.println(scanSampleSteps); }
  else { Serial.print("WN:unknown "); Serial.println(rxBuf); }
}

void setDir(bool right) { digitalWrite(PIN_DIR, right ? HIGH : LOW); delayMicroseconds(5); }

void doStep() {
  digitalWrite(PIN_STEP, HIGH); digitalWrite(LED_BUILTIN, HIGH); delayMicroseconds(stepDelay);
  digitalWrite(PIN_STEP, LOW);  digitalWrite(LED_BUILTIN, LOW);  delayMicroseconds(stepDelay);
}
