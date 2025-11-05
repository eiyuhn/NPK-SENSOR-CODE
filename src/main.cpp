#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

// ====== Your original pins (unchanged) ======
#define RS485_RX_PIN    18  // RO -> ESP32 RX2
#define RS485_TX_PIN    19  // DI <- ESP32 TX2
#define RS485_DE_RE_PIN 4   // DE & /RE tied together

// ====== Your original Modbus params (unchanged) ======
const uint8_t  SENSOR_ID  = 0x01;
const uint8_t  FUNC_READ  = 0x03;
const uint8_t  NUM_REGS_H = 0x00;
const uint8_t  NUM_REGS_L = 0x07;     // read 7 registers
const uint16_t TIMEOUT_MS = 1000;     // ms

// Precomputed CRC for [01 03 00 00 00 07] (lo, hi)
uint8_t requestFrame[8] = {
  SENSOR_ID, FUNC_READ,
  0x00, 0x00,
  NUM_REGS_H, NUM_REGS_L,
  0x04, 0x08
};

const size_t RESP_SIZE = 1 + 1 + 1 + 7*2 + 2; // 19 bytes
uint8_t resp[RESP_SIZE];

// ppm -> kg/ha factor (unchanged)
const float PPM_TO_KG_HA = 2.0f;

// ====== Simple cache of last reading (for /npk) ======
struct LastReading {
  bool ok = false;
  unsigned long ts = 0;
  uint16_t ecRaw = 0;
  int n_ppm = 0, p_ppm = 0, k_ppm = 0;
  float phVal = NAN;
  float n_kg_ha = 0, p_kg_ha = 0, k_kg_ha = 0;
  String error;
} last;

// ====== Wi-Fi AP + HTTP ======
const char* AP_SSID = "ESP32-NPK";
const char* AP_PASS = "fertisense";
IPAddress   AP_IP(192,168,4,1), AP_GW(192,168,4,1), AP_MASK(255,255,255,0);
WebServer   server(80);

void sendCORS(){
  server.sendHeader("Access-Control-Allow-Origin","*");
  server.sendHeader("Access-Control-Allow-Methods","GET, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers","Content-Type");
}
void handleOptions(){ sendCORS(); server.send(204); }

void handleRoot(){
  sendCORS();
  server.send(200,"text/plain",
    "FertiSense ESP32 NPK\n"
    "GET /npk  -> latest reading as JSON\n"
    "GET /read -> fresh read now, JSON\n"
  );
}

void sendJSON(const LastReading& r){
  sendCORS();
  if (!r.ok){
    // Send 504 Gateway Timeout if the sensor (our "gateway") timed out
    server.send(504,"application/json", String("{\"ok\":false,\"error\":\"")+r.error+"\"}");
    return;
  }
  String json = "{";
  json += "\"ok\":true,";
  json += "\"ts\":" + String(r.ts) + ",";
  json += "\"ec\":" + String(r.ecRaw) + ",";
  json += "\"n\":"  + String(r.n_ppm) + ",";
  json += "\"p\":"  + String(r.p_ppm) + ",";
  json += "\"k\":"  + String(r.k_ppm) + ",";
  json += "\"ph\":" + String(r.phVal,1) + ",";
  json += "\"n_kg_ha\":" + String(r.n_kg_ha,1) + ",";
  json += "\"p_kg_ha\":" + String(r.p_kg_ha,1) + ",";
  json += "\"k_kg_ha\":" + String(r.k_kg_ha,1);
  json += "}";
  server.send(200,"application/json",json);
}

// HTTP: get the last cached reading
void handleGetLatest(){ sendJSON(last); }

// ====== Your original reading logic, wrapped into a function ======
// !! CRITICAL FIX !!
// This function must ONLY read the sensor.
// It should *not* call server.handleClient().
// Calling it here while the server is already handling a request can cause a crash.
LastReading doOneRead() {
  LastReading out;

  // 1) Transmit Modbus request
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  Serial2.write(requestFrame, sizeof(requestFrame));
  Serial2.flush();
  delay(2);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  // 2) Wait for response (blocking)
  unsigned long t0 = millis();
  while (Serial2.available() < (int)RESP_SIZE && (millis() - t0) < TIMEOUT_MS) {
    // !! REMOVED server.handleClient() !!
    delay(1); // Small delay to prevent task watchdog from firing
  }

  // 3) Check for timeout
  if (Serial2.available() < (int)RESP_SIZE) {
    out.ok = false;
    out.error = "Timeout — no response from sensor";
    // Clear the serial buffer in case of partial data
    while(Serial2.available()) Serial2.read();
    return out;
  }

  // 4) Read response
  for (size_t i = 0; i < RESP_SIZE; ++i) {
    resp[i] = Serial2.read();
  }

  // 5) Basic header check: addr, func, byte count
  if (resp[0] != SENSOR_ID || resp[1] != FUNC_READ || resp[2] != 7*2) {
    out.ok = false;
    out.error = "Invalid response header";
    return out;
  }

  // 6) Parse registers (big-endian)
  // [3,4] moisture (skip), [5,6] temp (skip)
  uint16_t ecRaw  = word(resp[7],  resp[8]);      // µS/cm
  float    phVal  = word(resp[9],  resp[10]) * 0.1f;  // pH
  int      n_ppm  = word(resp[11], resp[12]);      // N (ppm)
  int      p_ppm  = word(resp[13], resp[14]);      // P (ppm)
  int      k_ppm  = word(resp[15], resp[16]);      // K (ppm)

  // 7) Convert to kg/ha
  float n_kg_ha = n_ppm * PPM_TO_KG_HA;
  float p_kg_ha = p_ppm * PPM_TO_KG_HA;
  float k_kg_ha = k_ppm * PPM_TO_KG_HA;

  // 8) Fill struct
  out.ok = true;
  out.ts = millis();
  out.ecRaw = ecRaw;
  out.n_ppm = n_ppm; out.p_ppm = p_ppm; out.k_ppm = k_ppm;
  out.phVal = phVal;
  out.n_kg_ha = n_kg_ha; out.p_kg_ha = p_kg_ha; out.k_kg_ha = k_kg_ha;
  return out;
}

// HTTP: force a fresh read now
void handleReadNow(){
  LastReading r = doOneRead();
  if (r.ok) last = r;  // update cache if successful
  sendJSON(r);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW); // start in RX

  // UART2 for Modbus (RX2, TX2)
  Serial2.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  // ====== Start AP + HTTP (tiny + safe) ======
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setSleep(true);

  server.on("/",      HTTP_GET,     handleRoot);
  server.on("/npk",   HTTP_GET,     handleGetLatest);
  server.on("/read",  HTTP_GET,     handleReadNow);
  server.on("/",      HTTP_OPTIONS, handleOptions);
  server.on("/npk",   HTTP_OPTIONS, handleOptions);
  server.on("/read",  HTTP_OPTIONS, handleOptions);

  // quiet the common captive-portal hits
  server.on("/favicon.ico",       HTTP_ANY, [](){ sendCORS(); server.send(204); });
  server.on("/generate_204",      HTTP_ANY, handleRoot);
  server.on("/hotspot-detect.html",HTTP_ANY, handleRoot);

  server.onNotFound([](){ sendCORS(); server.send(404,"application/json","{\"error\":\"not found\"}"); });
  server.begin();

  Serial.println();
  Serial.println("=== Fertisense NPK Reader (no SD) + AP ===");
  Serial.print("AP SSID: "); Serial.println(AP_SSID);
  Serial.print("AP IP:   "); Serial.println(WiFi.softAPIP());
  Serial.println("Server is waiting for requests...");
  Serial.println("CSV output format: timestamp_ms,EC_uS_cm,N_ppm,P_ppm,K_ppm,pH,N_kg_ha,P_kg_ha,K_kg_ha");
}

void loop() {
  // !! CRITICAL FIX !!
  // The loop's ONLY job is to run the web server.
  // It should *not* call doOneRead() itself.
  // This prevents conflicts when the app calls /read.
  server.handleClient();
}

