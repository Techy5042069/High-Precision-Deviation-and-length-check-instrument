// sensor_udp.ino — Sensor Arduino R4 WiFi (UDP continuous stream)
//
// Streams raw 14-bit ADC readings to the PC over UDP as fast as possible.
// No TICK, no bulk buffering, no TCP. Python pairs each reading with the
// latest ST: from the motor Arduino.
//
// Flow:
//   1. Arduino boots, connects to WiFi, prints its IP over USB serial.
//   2. Python sends "HELLO\n" UDP packet to sensor_ip:LISTEN_PORT.
//   3. Arduino learns the PC IP from the packet source address, starts streaming.
//   4. Arduino sends 2-byte little-endian uint16_t ADC values to pc_ip:STREAM_PORT.
//   5. Python sends "STOP\n" to end streaming.
//
// UDP Commands (ASCII, sent to LISTEN_PORT):
//   HELLO       start streaming (PC IP learned from source)
//   STOP        stop  streaming
//   AVG:n       samples to average per packet (1–16, default 4)
//   PING:<ms>   echoed back as PONG:<ms>,<ard_ms>
//
// Wiring: SENSOR → A0  (0–5 V)

// ── Configure these ───────────────────────────────────────────────────────────
#define WIFI_SSID   "METALGEAR"
#define WIFI_PASS   "hideo999"
#define LISTEN_PORT  5001    // Arduino listens for commands on this port
#define STREAM_PORT  5002    // Python listens for ADC data on this port
// ─────────────────────────────────────────────────────────────────────────────

#include <WiFiS3.h>
#include <WiFiUdp.h>

#define PIN_SENSOR  A0
#define ADC_BITS    14
#define AVG_DEFAULT  4
#define AVG_MAX     16

WiFiUDP   udp;
IPAddress pcIP;
bool      pcKnown   = false;
bool      streaming = false;
uint8_t   adcAvg    = AVG_DEFAULT;

// ─────────────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(921600);
  analogReadResolution(ADC_BITS);

  // ADC warmup
  for (int i = 0; i < 20; i++) { analogRead(PIN_SENSOR); delay(2); }

  Serial.print("IN:sensor_udp connecting wifi ssid=");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries++ < 40) delay(500);

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WN:wifi failed — halting");
    while (true) delay(1000);
  }

  udp.begin(LISTEN_PORT);

  Serial.print("IN:sensor_udp ready ip=");
  Serial.print(WiFi.localIP());
  Serial.print(" listen=");  Serial.print(LISTEN_PORT);
  Serial.print(" stream=");  Serial.print(STREAM_PORT);
  Serial.print(" adc=");     Serial.print(ADC_BITS);
  Serial.print("bit avg=");  Serial.println(adcAvg);
}

void loop() {
  handleCommands();
  if (streaming && pcKnown) sendSample();
}

// ─────────────────────────────────────────────────────────────────────────────
// Read ADC with averaging
// ─────────────────────────────────────────────────────────────────────────────

uint16_t readADC() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < adcAvg; i++) sum += analogRead(PIN_SENSOR);
  return (uint16_t)(sum / adcAvg);
}

// ─────────────────────────────────────────────────────────────────────────────
// Stream one 2-byte sample to PC
// ─────────────────────────────────────────────────────────────────────────────

void sendSample() {
  uint16_t adc = readADC();
  udp.beginPacket(pcIP, STREAM_PORT);
  udp.write((uint8_t*)&adc, 2);
  udp.endPacket();
}

// ─────────────────────────────────────────────────────────────────────────────
// Handle incoming UDP commands from PC
// ─────────────────────────────────────────────────────────────────────────────

void handleCommands() {
  int sz = udp.parsePacket();
  if (sz <= 0) return;

  char buf[32];
  int  len = udp.read(buf, sizeof(buf) - 1);
  if (len <= 0) return;
  while (len > 0 && (buf[len-1] == '\n' || buf[len-1] == '\r')) len--;
  buf[len] = '\0';

  // Always learn PC IP from any incoming packet
  pcIP    = udp.remoteIP();
  pcKnown = true;

  if (strncmp(buf, "HELLO", 5) == 0) {
    streaming = true;
    Serial.print("IN:streaming to "); Serial.print(pcIP);
    Serial.print(":"); Serial.println(STREAM_PORT);

  } else if (strncmp(buf, "STOP", 4) == 0) {
    streaming = false;
    Serial.println("IN:streaming stopped");

  } else if (strncmp(buf, "AVG:", 4) == 0) {
    int n = atoi(buf + 4);
    if (n >= 1 && n <= AVG_MAX) {
      adcAvg = (uint8_t)n;
      Serial.print("IN:avg="); Serial.println(adcAvg);
    }

  } else if (strncmp(buf, "PING:", 5) == 0) {
    char resp[32];
    snprintf(resp, sizeof(resp), "PONG:%s,%lu", buf + 5, millis());
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write((uint8_t*)resp, strlen(resp));
    udp.endPacket();
  }
}
