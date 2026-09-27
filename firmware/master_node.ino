/*
  GNSS + FUSION MASTER NODE (gateway)
  ESP32 + LoRa02 (433 MHz) + WiFi + MQTT over TLS (HiveMQ Cloud)
*/

#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <esp_system.h>

// ---- LoRa ----
#define LORA_SS     5
#define LORA_RST    12
#define LORA_DIO0   13
#define LORA_SCK    18
#define LORA_MISO   19
#define LORA_MOSI   23
const long    LORA_FREQ     = 433E6;
const uint8_t LORA_SYNC     = 0xF3;
const int     LORA_TX_POWER = 10;

// ---- WiFi / MQTT ----
const char* WIFI_SSID = "Redmi Note 13";
const char* WIFI_PASS = "shehanharshana";

const char* MQTT_HOST = "496a541afb5c42e482acdea2535c4281.s1.eu.hivemq.cloud";
const int   MQTT_PORT = 8883;
const char* MQTT_USER = "temperature";
const char* MQTT_PASS = "temperature";
const char* CLIENT_ID = "gnss-master-01";

// ---- Topics ----
const char* TOPIC_LAT      = "temperature/location/latitude";
const char* TOPIC_LON      = "temperature/location/longitude";
const char* TOPIC_SATS     = "temperature/location/sats";
const char* TOPIC_READINGS = "temperature/location/readings";
const char* TOPIC_STATUS   = "temperature/location/status";
const char* TOPIC_LINK     = "temperature/location/link";
const char* TOPIC_COMMAND  = "temperature/location/command";

// ---- Fusion Dashboard Topics ----
const char* TOPIC_DTEMP  = "temperature/fusion/digital";
const char* TOPIC_ATEMP  = "temperature/fusion/analog";
const char* TOPIC_FTEMP  = "temperature/fusion/fused";
const char* TOPIC_DIFF   = "temperature/fusion/difference";
const char* TOPIC_STATE  = "temperature/fusion/state";
const char* TOPIC_FMIN   = "temperature/fusion/min";
const char* TOPIC_FMAX   = "temperature/fusion/max";
const char* TOPIC_FAVG   = "temperature/fusion/avg";

// ---- Timing ----
const unsigned long LINK_TIMEOUT_MS  = 15000;
const unsigned long BURST_GAP_MS     = 600;
const int           BURST_MAX        = 8;
const unsigned long LORA_CHECK_MS    = 1000;
const unsigned long SUMMARY_MS       = 15000;

WiFiClientSecure espClient;
PubSubClient client(espClient);

unsigned long lastPacketTime = 0, lastLoraCheck = 0, lastSummary = 0;
unsigned long lastWifiTry = 0, lastMqttTry = 0, lastLinkRetryPing = 0;
bool wifiWasUp = false;
int loraFaults = 0;

String linkState = "";
String lastStatus = "";

String pendingCmd = "";
bool cmdPending = false;
int burstSent = 0;
unsigned long lastBurst = 0, nextGap = 0;

struct Cached { const char* topic; String value; bool dirty; };
Cached cache[] = {
  { TOPIC_LAT, "", false }, { TOPIC_LON, "", false }, { TOPIC_SATS, "", false },
  { TOPIC_READINGS, "", false }, { TOPIC_STATUS, "", false }, { TOPIC_LINK, "", false },
  { TOPIC_DTEMP, "", false }, { TOPIC_ATEMP, "", false },
  { TOPIC_FTEMP, "", false }, { TOPIC_DIFF, "", false }, { TOPIC_STATE, "", false },
  { TOPIC_FMIN, "", false }, { TOPIC_FMAX, "", false }, { TOPIC_FAVG, "", false }
};

byte loraReadReg(byte addr) {
  SPI.beginTransaction(SPISettings(8E6, MSBFIRST, SPI_MODE0));
  digitalWrite(LORA_SS, LOW);
  SPI.transfer(addr & 0x7F);
  byte v = SPI.transfer(0);
  digitalWrite(LORA_SS, HIGH);
  SPI.endTransaction();
  return v;
}

void loraWriteReg(byte addr, byte val) {
  SPI.beginTransaction(SPISettings(8E6, MSBFIRST, SPI_MODE0));
  digitalWrite(LORA_SS, LOW);
  SPI.transfer(addr | 0x80);
  SPI.transfer(val);
  digitalWrite(LORA_SS, HIGH);
  SPI.endTransaction();
}

bool loraHealthy() {
  return loraReadReg(0x42) == 0x12 &&
         (loraReadReg(0x01) & 0x80) &&
         loraReadReg(0x39) == LORA_SYNC;
}

bool loraInit() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) return false;
  LoRa.setSyncWord(LORA_SYNC);
  LoRa.enableCrc();
  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.receive();
  return true;
}

void loraRecover(const char* why) {
  loraFaults++;
  Serial.printf("[LORA] %s. Re-initializing radio (fault #%d)...\n", why, loraFaults);
  if (loraInit()) Serial.println("[LORA] Recovered.");
}

void loraHealthCheck() {
  if (millis() - lastLoraCheck < LORA_CHECK_MS) return;
  lastLoraCheck = millis();
  if (!loraHealthy()) loraRecover("Chip configuration lost");
}

bool loraSend(const String &payload) {
  if (!loraHealthy()) loraRecover("Chip not healthy before TX");
  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket(true);
  unsigned long t0 = millis();
  while (!(loraReadReg(0x12) & 0x08)) {
    if (millis() - t0 > 1000) {
      loraRecover("TX timeout");
      return false;
    }
    delay(1);
  }
  loraWriteReg(0x12, 0x08);
  LoRa.receive();
  return true;
}

void pub(const char* topic, const String &value) {
  Cached *c = nullptr;
  for (auto &it : cache) {
    if (strcmp(it.topic, topic) == 0) { c = &it; break; }
  }
  if (!c) return;

  bool changed = (c->value != value);
  c->value = value;
  if (!changed && !c->dirty) return;

  if (client.connected() && client.publish(topic, value.c_str(), true)) {
    c->dirty = false;
    Serial.println("[MQTT TX] " + String(topic) + " -> " + value);
  } else {
    c->dirty = true;
  }
}

void flushCache() {
  for (auto &it : cache) {
    if (it.value.length() == 0) continue;
    it.dirty = !client.publish(it.topic, it.value.c_str(), true);
  }
  Serial.println("[MQTT] Cached values re-published.");
}

void setStatus(const String &s) { lastStatus = s; pub(TOPIC_STATUS, s); }
void setLink(const String &s)   { linkState = s;  pub(TOPIC_LINK, s); }

void startWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.println("[WIFI] Connecting to " + String(WIFI_SSID) + "...");
}

void connectMqtt() {
  Serial.print("[MQTT] Connecting... ");
  if (client.connect(CLIENT_ID, MQTT_USER, MQTT_PASS, TOPIC_LINK, 1, true, "offline")) {
    Serial.println("connected.");
    client.subscribe(TOPIC_COMMAND, 1);
    if (linkState.length() == 0) setLink("waiting");
    flushCache();
  } else {
    Serial.printf("failed, rc=%d. Will retry.\n", client.state());
  }
}

void maintainNetwork() {
  unsigned long now = millis();
  if (WiFi.status() != WL_CONNECTED) {
    wifiWasUp = false;
    if (now - lastWifiTry > 15000) {
      lastWifiTry = now;
      Serial.println("[WIFI] Not connected, retrying...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
    return;
  }
  if (!wifiWasUp) {
    wifiWasUp = true;
    Serial.println("[WIFI] Connected. IP: " + WiFi.localIP().toString());
  }
  if (!client.connected() && now - lastMqttTry > 5000) {
    lastMqttTry = now;
    connectMqtt();
  }
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();
  Serial.printf("\n[MQTT RX] Topic: %s | Payload: '%s'\n", topic, msg.c_str());

  if (msg == "LOCATE_UNIT") {
    pendingCmd = msg;
    cmdPending = true;
    burstSent = 0;
    lastBurst = 0;
    nextGap = 0;
    setStatus("requesting");
  }
}

void processCommandBurst() {
  if (!cmdPending) return;
  unsigned long now = millis();
  if (burstSent > 0 && now - lastBurst < nextGap) return;

  if (burstSent >= BURST_MAX) {
    cmdPending = false;
    setStatus("no response");
    return;
  }
  lastBurst = now;
  nextGap = BURST_GAP_MS + random(0, 250);
  burstSent++;
  loraSend("CMD:" + pendingCmd);
}

String fieldAfter(const String &rx, const String &key) {
  int i = rx.indexOf(key);
  if (i == -1) return "";
  i += key.length();
  int e = rx.indexOf(',', i);
  String v = (e == -1) ? rx.substring(i) : rx.substring(i, e);
  v.trim();
  return v;
}

void handleSlavePacket(const String &rx) {
  if (!(rx.startsWith("ACK:") || rx.startsWith("HB:") || rx.startsWith("GPS:"))) return;

  lastPacketTime = millis();
  if (linkState != "online") setLink("online");

  String dt = fieldAfter(rx, "DTEMP:");
  String at = fieldAfter(rx, "ATEMP:");
  String ft = fieldAfter(rx, "FTEMP:");
  String df = fieldAfter(rx, "DIFF:");
  String fs = fieldAfter(rx, "STATE:");
  String fmin = fieldAfter(rx, "FMIN:");
  String fmax = fieldAfter(rx, "FMAX:");
  String favg = fieldAfter(rx, "FAVG:");

  if (dt.length()) pub(TOPIC_DTEMP, dt);
  if (at.length()) pub(TOPIC_ATEMP, at);
  if (ft.length()) pub(TOPIC_FTEMP, ft);
  if (df.length()) pub(TOPIC_DIFF, df);
  if (fs.length()) pub(TOPIC_STATE, fs);
  if (fmin.length()) pub(TOPIC_FMIN, fmin);
  if (fmax.length()) pub(TOPIC_FMAX, fmax);
  if (favg.length()) pub(TOPIC_FAVG, favg);

  if (rx.startsWith("ACK:")) {
    cmdPending = false;
    if (lastStatus == "requesting") setStatus("searching");
  } else if (rx.startsWith("HB:")) {
    if (lastStatus == "" || lastStatus == "no response") setStatus("idle");
  } else if (rx.startsWith("GPS:SEARCHING")) {
    cmdPending = false;
    pub(TOPIC_SATS, fieldAfter(rx, "SATS:"));
    pub(TOPIC_READINGS, fieldAfter(rx, "READ:"));
    setStatus("searching");
  } else if (rx.startsWith("GPS:TIMEOUT")) {
    cmdPending = false;
    setStatus("timeout");
  } else if (rx.startsWith("GPS:LAT:")) {
    cmdPending = false;
    pub(TOPIC_LAT, fieldAfter(rx, "LAT:"));
    pub(TOPIC_LON, fieldAfter(rx, "LNG:"));
    pub(TOPIC_SATS, fieldAfter(rx, "SATS:"));
    pub(TOPIC_READINGS, "10");
    setStatus("fixed");
  }
}

void pollLoRa() {
  int size = LoRa.parsePacket();
  if (!size) return;
  String rx = "";
  rx.reserve(size);
  while (LoRa.available()) rx += (char)LoRa.read();
  rx.trim();
  Serial.printf("[LoRa RX] '%s' | RSSI %d dBm\n", rx.c_str(), LoRa.packetRssi());
  handleSlavePacket(rx);
}

void linkWatchdog() {
  unsigned long now = millis();
  if ((linkState == "online" || linkState == "waiting") && now - lastPacketTime > LINK_TIMEOUT_MS) {
    Serial.println("[LINK] No packet from slave for 15s. Setting link to 'no link'.");
    setLink("no link");
  }
  
  if (linkState == "no link" && now - lastLinkRetryPing > 4000) {
    lastLinkRetryPing = now;
    if (!loraHealthy()) {
      loraInit();
    } else {
      LoRa.receive();
    }
  }
}

void setup() {
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[INIT] Master node booting...");

  while (!loraInit()) {
    Serial.println("[ERROR] LoRa init failed. Retrying...");
    delay(2000);
  }
  Serial.println("[INIT] LoRa ready.");

  espClient.setInsecure();
  client.setServer(MQTT_HOST, MQTT_PORT);
  client.setCallback(onMqttMessage);
  client.setKeepAlive(30);
  client.setSocketTimeout(5);
  startWifi();
  lastPacketTime = millis();
}

void loop() {
  pollLoRa();
  maintainNetwork();
  if (client.connected()) client.loop();  
  processCommandBurst();
  loraHealthCheck();
  linkWatchdog();
}