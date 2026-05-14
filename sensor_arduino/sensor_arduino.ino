
// ─────────────────────────────────────────────────────────────────────────────
// Gantry Sensor Node v5 — Arduino R4 WiFi
// TCP server on port 5001.
//
// Features:
//   - No timestamps
//   - TICK-driven sampling
//   - Bulk packet streaming
//   - Full buffer dump
//   - Sensor-side OOR detection
//
// PC → Arduino:
//   START:en,lo,hi,armCount
//   STOP
//   TICK
//   DUMP
//   BULK:n
//   PING:nnn
//
// Arduino → PC:
//   BK:<seq_start>,<adc0>,<adc1>,...
//   ADC:<adc>
//   OOR:<seq>,<adc>
//   IN:<msg>
//   PONG:<pc_ms>,<ard_ms>
//
// OOR:
//   Sensor accumulates consecutive in-range readings.
//   Once armCount consecutive readings land in [lo, hi],
//   OOR detection is armed.  The first subsequent reading
//   outside [lo, hi] sends OOR:<seq>,<adc> once.
//
// ─────────────────────────────────────────────────────────────────────────────

#include <WiFiS3.h>

const char *WIFI_SSID = "METALGEAR";
const char *WIFI_PASS = "hideo999";

const int TCP_PORT = 5001;

// ── Buffer ──────────────────────────────────────────────────────────────────

#define MAX_BUFFER 4096

uint16_t adcBuf[MAX_BUFFER];
uint16_t bufCount = 0;

// ── Sequence ────────────────────────────────────────────────────────────────

uint16_t seqNum = 0;

// ── Bulk streaming ──────────────────────────────────────────────────────────

#define BULK_SIZE_DEFAULT 10
#define BULK_SIZE_MIN 1
#define BULK_SIZE_MAX 100

uint8_t bulkSize = BULK_SIZE_DEFAULT;
uint8_t bulkPending = 0;
uint16_t bulkStartSeq = 0;

// ── OOR ─────────────────────────────────────────────────────────────────────
// Arming strategy: data-driven consecutive-reading gate.
//
//   oorArmCount   — number of consecutive in-range readings required to arm
//   oorConsecIn   — running count of consecutive in-range readings seen so far
//   oorArmed      — true once the gate has been satisfied
//   oorFired      — true after the first OOR event (suppresses duplicates;
//                   the PC aborts the scan on the first OOR anyway)

bool     oorEnabled  = false;
uint16_t oorLo       = 0;
uint16_t oorHi       = 1023;
uint16_t oorArmCount = 5;
uint16_t oorConsecIn = 0;
bool     oorArmed    = false;
bool     oorFired    = false;

// ── Scan state ──────────────────────────────────────────────────────────────

bool scanning = false;

// ── Sensor ──────────────────────────────────────────────────────────────────

#define PIN_SENSOR A0

// ── RX buffer ───────────────────────────────────────────────────────────────

#define RX_BUF_SIZE 64

char rxBuf[RX_BUF_SIZE];
uint8_t rxLen = 0;

WiFiServer server(TCP_PORT);
WiFiClient client;

// ────────────────────────────────────────────────────────────────────────────

void setup()
{
  Serial.begin(115200);

  // ADC warmup
  for (int i = 0; i < 10; i++)
  {
    analogRead(PIN_SENSOR);
    delay(5);
  }

  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println();

  Serial.print("Sensor IP: ");
  Serial.println(WiFi.localIP());

  server.begin();

  Serial.print("TCP port: ");
  Serial.println(TCP_PORT);
}

// ────────────────────────────────────────────────────────────────────────────

void loop()
{
  // Accept new client

  if (!client || !client.connected())
  {
    scanning = false;
    bulkPending = 0;

    WiFiClient c = server.available();

    if (c)
    {
      client = c;

      sendMsg(
        "IN:sensor v5 ready bulk=" +
        String(bulkSize) +
        " maxbuf=" +
        String(MAX_BUFFER)
      );
    }
  }

  if (client && client.connected())
  {
    handleTCP();
  }
}

// ────────────────────────────────────────────────────────────────────────────
// Take one sample
// ────────────────────────────────────────────────────────────────────────────
// uint16_t tmp_seq = 0;
// uint16_t tmp_adc = 0;
void takeSample(bool store = true)
{
  uint16_t adc = (uint16_t)analogRead(PIN_SENSOR);

  // Manual mode live reading
  if (!store)
  {
    sendMsg("ADC:" + String(adc));
    return;
  }

  // if(oorFired){
  //   sendMsg(
  //       "OOR:" +
  //       String(tmp_seq) +
  //       "," +
  //       String(tmp_adc)
  //     );
  // }
  uint16_t seq = seqNum++;

  // Store in scan buffer

  if (bufCount < MAX_BUFFER)
  {
    adcBuf[bufCount++] = adc;
  }

  // ── OOR detection ───────────────────────────────────────────────────────
  // Phase 1 (not yet armed): count consecutive in-range readings.
  //   Each in-range reading increments oorConsecIn.
  //   An out-of-range reading resets the counter — the surface hasn't
  //   appeared yet, so we're still in the pre-object region.
  //   Once oorConsecIn reaches oorArmCount the detector is armed.
  //
  // Phase 2 (armed): any out-of-range reading fires OOR once and stops.

  if (oorEnabled && !oorFired)
  {
    bool inRange = (adc >= oorLo && adc <= oorHi);

    if (!oorArmed)
    {
      if (inRange)
      {
        oorConsecIn++;
        if (oorConsecIn >= oorArmCount)
        {
          oorArmed = true;
          sendMsg("IN:OOR armed at seq=" + String(seq));
        }
      }
      else
      {
        oorConsecIn = 0;   // reset — not on the surface yet
      }
    }
    else
    {
      // Armed: flag any out-of-range reading immediately
      if (!inRange)
      {
        oorFired = true;
        sendMsg(
          "OOR:" +
          String(seq) +
          "," +
          String(adc)
        );
      }
    }
  }

  // ── Bulk accumulation ───────────────────────────────────────────────────

  if (bulkPending == 0)
  {
    bulkStartSeq = seq;
  }

  bulkPending++;

  if (bulkPending >= bulkSize)
  {
    sendBulk();
  }
}

// ────────────────────────────────────────────────────────────────────────────
// Send one bulk packet
// ────────────────────────────────────────────────────────────────────────────

void sendBulk()
{
  if (bufCount == 0 && bulkPending == 0)
  {
    return;
  }

  // BK:<seq_start>,<adc0>,<adc1>,...

  String msg = "BK:" + String(bulkStartSeq);

  uint16_t start = bufCount - bulkPending;

  for (uint8_t i = 0; i < bulkPending; i++)
  {
    if (start + i < MAX_BUFFER)
    {
      msg += "," + String(adcBuf[start + i]);
    }
  }

  sendMsg(msg);

  bulkPending = 0;
}

// ────────────────────────────────────────────────────────────────────────────
// Dump entire buffer
// ────────────────────────────────────────────────────────────────────────────

void dumpBuffer()
{
  // Flush partial bulk first

  if (bulkPending > 0)
  {
    sendBulk();
  }

  // Send entire buffer

  uint16_t sent = 0;

  while (sent < bufCount)
  {
    uint8_t chunk =
      min(
        (uint16_t)bulkSize,
        (uint16_t)(bufCount - sent)
      );

    String msg = "BK:" + String(sent);

    for (uint8_t i = 0; i < chunk; i++)
    {
      msg += "," + String(adcBuf[sent + i]);
    }

    sendMsg(msg);

    sent += chunk;
  }

  sendMsg("IN:dump done count=" + String(bufCount));
}

// ────────────────────────────────────────────────────────────────────────────
// TCP helpers
// ────────────────────────────────────────────────────────────────────────────

void sendMsg(const String &msg)
{
  if (client && client.connected())
  {
    client.println(msg);
  }
}

void handleTCP()
{
  while (client.available())
  {
    char c = (char)client.read();

    if (c == '\n' || c == '\r')
    {
      if (rxLen > 0)
      {
        rxBuf[rxLen] = '\0';

        processCommand(String(rxBuf));

        rxLen = 0;
      }
    }
    else if (rxLen < RX_BUF_SIZE - 1)
    {
      rxBuf[rxLen++] = c;
    }
  }
}

// ────────────────────────────────────────────────────────────────────────────
// Command parser
// ────────────────────────────────────────────────────────────────────────────

void processCommand(const String &cmd)
{
  // ── START:en,lo,hi,ignoreTicks ─────────────────────────────────────────

  if (cmd.startsWith("START"))
  {
    scanning = true;

    bufCount = 0;
    seqNum = 0;

    bulkPending = 0;
    bulkStartSeq = 0;

    // Reset OOR state

    oorEnabled  = false;
    oorLo       = 0;
    oorHi       = 1023;
    oorArmCount = 5;
    oorConsecIn = 0;
    oorArmed    = false;
    oorFired    = false;

    // Parse START:en,lo,hi,armCount

    int p1 = cmd.indexOf(':');

    if (p1 > 0)
    {
      String rest = cmd.substring(p1 + 1);

      int c1 = rest.indexOf(',');
      int c2 = rest.indexOf(',', c1 + 1);
      int c3 = rest.indexOf(',', c2 + 1);

      if (c1 > 0 && c2 > 0 && c3 > 0)
      {
        oorEnabled =
          rest.substring(0, c1).toInt() == 1;

        oorLo =
          constrain(
            rest.substring(c1 + 1, c2).toInt(),
            0,
            1023
          );

        oorHi =
          constrain(
            rest.substring(c2 + 1, c3).toInt(),
            0,
            1023
          );

        oorArmCount =
          (uint16_t)max(
            1,
            rest.substring(c3 + 1).toInt()
          );
      }
    }

    sendMsg(
      "IN:scan started "
      "oor=" + String(oorEnabled ? 1 : 0) +
      " lo=" + String(oorLo) +
      " hi=" + String(oorHi) +
      " arm=" + String(oorArmCount)
    );
  }

  // ── STOP ───────────────────────────────────────────────────────────────

  else if (cmd == "STOP")
  {
    scanning = false;

    if (bulkPending > 0)
    {
      sendBulk();
    }

    sendMsg("IN:scan stopped seq=" + String(seqNum));
  }

  // ── TICK ───────────────────────────────────────────────────────────────

  else if (cmd == "TICK")
  {
    if (scanning)
      takeSample(true);
    else
      takeSample(false);
  }

  // ── DUMP ───────────────────────────────────────────────────────────────

  else if (cmd == "DUMP")
  {
    dumpBuffer();
  }

  // ── BULK:n ─────────────────────────────────────────────────────────────

  else if (cmd.startsWith("BULK:"))
  {
    int n =
      constrain(
        cmd.substring(5).toInt(),
        BULK_SIZE_MIN,
        BULK_SIZE_MAX
      );

    bulkSize = (uint8_t)n;

    sendMsg("IN:bulk=" + String(bulkSize));
  }

  // ── PING:nnn ───────────────────────────────────────────────────────────

  else if (cmd.startsWith("PING:"))
  {
    unsigned long pc_ms =
      cmd.substring(5).toInt();

    sendMsg(
      "PONG:" +
      String(pc_ms) +
      "," +
      String(millis())
    );
  }

  // ── RATE legacy compatibility ─────────────────────────────────────────

  else if (cmd.startsWith("RATE:"))
  {
    sendMsg("IN:RATE ignored in v5 (TICK-driven)");
  }
}
