#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <algorithm>

// ================== RS485 PINS (your wiring) ==================
#define RS485_RX_PIN 18   // RS485 module TXD -> ESP32 RX2
#define RS485_TX_PIN 19   // RS485 module RXD -> ESP32 TX2
// If you have DE/RE tied to one pin, uncomment and use it:
// #define RS485_DIR_PIN 4

// ================== Wi-Fi AP SETTINGS ==================
const char* AP_SSID = "ESP32-NPK";
const char* AP_PASS = "fertisense";
IPAddress   AP_IP(192,168,4,1);
IPAddress   AP_GW(192,168,4,1);
IPAddress   AP_MASK(255,255,255,0);

WebServer server(80);

// ================== MODBUS SETTINGS ==================
// These are the usual defaults for many 7-in-1 NPK probes.
static uint8_t  g_slave    = 1;
static uint16_t g_startReg = 0x0000;   // 7 regs from 0x0000: moist, temp, EC, pH*10, N, P, K
static uint16_t g_numRegs  = 0x0007;
static uint32_t g_baud     = 4800;     // change to 9600 if needed
static uint32_t g_fmt      = SERIAL_8N1;
static uint16_t g_timeout  = 2000;     // ms
static uint16_t g_turn     = 12;       // ms after TX before RX

// ================== DATA STRUCT ==================
struct LastReading {
  bool ok = false;
  unsigned long ts = 0;      // millis when read
  uint16_t words[7] = {0};   // raw 7 registers
  int n_ppm = 0;
  int p_ppm = 0;
  int k_ppm = 0;
  float ph = NAN;
  String error;
} last;

// ================== CORS & HTTP HELPERS ==================
void sendCORS() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

void handleOptions() {
  sendCORS();
  server.send(204);
}

void handleRoot() {
  sendCORS();
  server.send(
    200,
    "text/plain",
    "FertiSense ESP32 NPK\n"
    "GET /npk  -> last reading (JSON)\n"
    "GET /read -> NEW reading (JSON)\n"
  );
}

// Return JSON with only N, P, K and pH (plus raw registers).
void sendJSON(const LastReading& r) {
  sendCORS();
  if (!r.ok) {
    String err = r.error.length() ? r.error : "No valid reading yet";
    server.send(
      504,
      "application/json",
      String("{\"ok\":false,\"error\":\"") + err + "\"}"
    );
    return;
  }

  String j = "{";
  j += "\"ok\":true,";
  j += "\"ts\":" + String(r.ts) + ",";
  j += "\"n\":" + String(r.n_ppm) + ",";
  j += "\"p\":" + String(r.p_ppm) + ",";
  j += "\"k\":" + String(r.k_ppm) + ",";
  j += "\"ph\":" + String(r.ph, 1) + ",";

  j += "\"raw\":["
       + String(r.words[0]) + ","
       + String(r.words[1]) + ","
       + String(r.words[2]) + ","
       + String(r.words[3]) + ","
       + String(r.words[4]) + ","
       + String(r.words[5]) + ","
       + String(r.words[6]) + "]";

  j += "}";

  server.send(200, "application/json", j);
}

// ================== CRC + MODBUS FRAME ==================
uint16_t crc16(const uint8_t* d, size_t n) {
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    c ^= d[i];
    for (int j = 0; j < 8; j++) {
      c = (c & 1) ? (c >> 1) ^ 0xA001 : (c >> 1);
    }
  }
  return c;
}

void buildReq(uint8_t id, uint16_t start, uint16_t count, uint8_t* out) {
  out[0] = id;
  out[1] = 0x03;          // Read Holding Registers
  out[2] = start >> 8;
  out[3] = start & 0xFF;
  out[4] = count >> 8;
  out[5] = count & 0xFF;
  uint16_t cr = crc16(out, 6);
  out[6] = cr & 0xFF;
  out[7] = cr >> 8;
}

// ================== LOW-LEVEL READ ==================
bool readSensor(LastReading &out) {
  while (Serial2.available()) Serial2.read();

  uint8_t req[8];
  buildReq(g_slave, g_startReg, g_numRegs, req);

  // if using DE/RE:
  // digitalWrite(RS485_DIR_PIN, HIGH);
  Serial2.write(req, 8);
  Serial2.flush();
  delay(g_turn);
  // digitalWrite(RS485_DIR_PIN, LOW);

  const size_t NEED = 3 + 14 + 2;   // addr, func, bc, 14 data, 2 CRC
  unsigned long t0 = millis();
  uint8_t r[64];
  size_t got = 0;

  while (millis() - t0 < g_timeout) {
    int a = Serial2.available();
    if (a > 0) {
      got += Serial2.readBytes(r + got, std::min(a, (int)(sizeof(r) - got)));
    }

    if (got >= 5 && r[0] == g_slave && r[1] == 0x03 && r[2] == 14) {
      size_t total = NEED;
      if (got >= total) {
        uint16_t rxCRC = r[total - 2] | (uint16_t(r[total - 1]) << 8);
        uint16_t cc    = crc16(r, total - 2);
        if (cc != rxCRC) {
          out.ok = false;
          out.error = "CRC error";
          return false;
        }

        const uint8_t* d = &r[3];
        for (int i = 0; i < 7; i++) {
          out.words[i] = (uint16_t(d[2 * i]) << 8) | d[2 * i + 1];
        }

        // words[0]=moisture, [1]=temp, [2]=EC, [3]=pH*10, [4]=N, [5]=P, [6]=K
        out.ph    = out.words[3] * 0.1f;
        out.n_ppm = out.words[4];
        out.p_ppm = out.words[5];
        out.k_ppm = out.words[6];
        out.ts    = millis();
        out.ok    = true;
        out.error = "";
        return true;
      }
    }
    delay(1);
  }

  out.ok = false;
  out.error = got > 0 ? "Incomplete response" : "Timeout – no response";
  return false;
}

// ================== DEBUG PRINT ==================
void printConsole(const LastReading& r) {
  if (!r.ok) {
    Serial.print("Read error: ");
    Serial.println(r.error);
    return;
  }

  Serial.printf(
    "RAW: m=%u, t=%u, EC=%u, pH10=%u, N=%u, P=%u, K=%u\n",
    r.words[0], r.words[1], r.words[2], r.words[3],
    r.words[4], r.words[5], r.words[6]
  );

  Serial.println("--- SENSOR RESULT ---");
  Serial.printf("N:  %d ppm\n", r.n_ppm);
  Serial.printf("P:  %d ppm\n", r.p_ppm);
  Serial.printf("K:  %d ppm\n", r.k_ppm);
  Serial.printf("pH: %.1f\n", r.ph);
  Serial.println("---------------------");
}

// ================== HTTP HANDLERS ==================

// GET /read  -> perform a NEW Modbus read, send JSON, and store as "last"
void handleReadNow() {
  LastReading r;
  if (readSensor(r)) {
    last = r;
  }
  printConsole(r);   // optional
  sendJSON(r);
}

// GET /npk -> just send the last successful reading (no new Modbus call)
void handleNpk() {
  sendJSON(last);
}

// ================== SETUP & LOOP ==================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== FertiSense ESP32 NPK Reader (N, P, K, pH only, on-demand) ===");

  // If using DE/RE:
  // pinMode(RS485_DIR_PIN, OUTPUT);
  // digitalWrite(RS485_DIR_PIN, LOW);

  Serial2.begin(g_baud, g_fmt, RS485_RX_PIN, RS485_TX_PIN);
  delay(50);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setSleep(false);

  server.on("/",    HTTP_GET,      handleRoot);
  server.on("/npk", HTTP_GET,      handleNpk);
  server.on("/read",HTTP_GET,      handleReadNow);

  server.on("/",    HTTP_OPTIONS,  handleOptions);
  server.on("/npk", HTTP_OPTIONS,  handleOptions);
  server.on("/read",HTTP_OPTIONS,  handleOptions);

  server.onNotFound([]() {
    sendCORS();
    server.send(404, "application/json", "{\"error\":\"not found\"}");
  });

  server.begin();

  Serial.printf("AP SSID: %s  IP: %s\n",
                AP_SSID, WiFi.softAPIP().toString().c_str());
  Serial.printf("Modbus: id=%u, start=0x%04X, regs=%u, baud=%lu, %s\n",
                g_slave, g_startReg, g_numRegs,
                (unsigned long)g_baud,
                (g_fmt == SERIAL_8N1 ? "8N1" : "8E1"));
}

void loop() {
  // ✅ NO automatic sensor polling here.
  // The sensor is only read when the phone calls GET /read.
  server.handleClient();
  delay(5);
}
