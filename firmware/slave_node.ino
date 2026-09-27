/*
  GNSS + FUSION SLAVE NODE 
  ESP32 + LoRa02 (433 MHz) + NEO-8M + DS18B20 + NTC
*/

#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <esp_system.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>

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

// ---- GPS ----
#define GPS_RX_PIN  16   // ESP32 RX <- NEO-8M TX
#define GPS_TX_PIN  17   // ESP32 TX -> NEO-8M RX

// ---- Fusion Sensor Pins & Constants ----
#define DS18B20_PIN   4   
#define NTC_PIN       34  

const float SERIES_RESISTOR     = 10000.0;
const float NOMINAL_RESISTANCE  = 10000.0;
const float NOMINAL_TEMPERATURE = 25.0;
const float B_COEFFICIENT       = 3950.0;
const float ADC_MAX             = 4095.0;
const float SUPPLY_VOLTAGE      = 3.3;

const float CAL_SLOPE  = 1.01120;
const float CAL_OFFSET = -5.25259;
const float DELTA_THRESHOLD = 1.50;

OneWire oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);

// ---- Timing ----
const int             READINGS_NEEDED       = 10;
const unsigned long READING_SPACING_MS    = 1000;
const unsigned long REPORT_INTERVAL_MS    = 2000;
const unsigned long HEARTBEAT_INTERVAL_MS = 2000;
const unsigned long DETAILS_INTERVAL_MS   = 5000;
const unsigned long MAX_SEARCH_TIME_MS    = 0;     // 0 = SEARCH FOREVER
const int             FINAL_REPEATS       = 5;
const unsigned long FINAL_REPEAT_MS       = 1000;
const unsigned long LORA_CHECK_MS         = 1000;

const uint8_t UBX_BACKUP_MODE[] = { 0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x08, 0x00, 0x16, 0x74 };
const uint8_t UBX_WAKEUP_MODE[] = { 0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x09, 0x00, 0x17, 0x76 };

HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

enum State { IDLE, LOCATING, REPORTING };
State state = IDLE;

int readingsCount = 0;
double sumLat = 0.0, sumLng = 0.0;
unsigned long lastReport = 0, lastHeartbeat = 0, lastDetails = 0;
unsigned long locateStart = 0, lastReadingTime = 0, lastLoraCheck = 0;
uint32_t charsAtStart = 0;
int loraFaults = 0;

String finalPacket = "";
int finalSent = 0;
unsigned long lastFinalSend = 0;

unsigned long lastSensorRead = 0;
float tempDS18B20 = NAN;
float tempNtcCalibrated = NAN;
float tempFused = NAN;
float deltaT = NAN;
String modeState = "UNKNOWN";

// ---- Min / Max / Average Tracking Over Time ----
float fusedMin = 999.0;
float fusedMax = -999.0;
double fusedSum = 0.0;
unsigned long fusedCount = 0;
float fusedAvg = 0.0;

void printResetReason() {
  const char* s = "unknown";
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   s = "power-on"; break;
    case ESP_RST_SW:        s = "software reset"; break;
    case ESP_RST_PANIC:     s = "CRASH (panic)"; break;
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:       s = "WATCHDOG"; break;
    case ESP_RST_BROWNOUT:  s = "BROWNOUT (power dip!)"; break;
    default: break;
  }
  Serial.printf("[BOOT] Reset reason: %s\n", s);
}

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
  else            Serial.println("[LORA] Re-init failed, will retry.");
}

void loraHealthCheck() {
  if (millis() - lastLoraCheck < LORA_CHECK_MS) return;
  lastLoraCheck = millis();
  if (!loraHealthy()) loraRecover("Chip configuration lost (power dip?)");
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
  Serial.println("[LoRa TX] " + payload);
  return true;
}

void readSensors() {
  if (lastSensorRead != 0 && millis() - lastSensorRead < 1000) return;
  lastSensorRead = millis();

  ds18b20.requestTemperatures();
  tempDS18B20 = ds18b20.getTempCByIndex(0);

  int adcValue = analogRead(NTC_PIN);
  if (adcValue <= 0) adcValue = 1; 
  if (adcValue >= 4095) adcValue = 4094; 
  
  float voltage = (adcValue / ADC_MAX) * SUPPLY_VOLTAGE;
  float resistance = SERIES_RESISTOR / (SUPPLY_VOLTAGE / voltage - 1.0);
  float T0_K = NOMINAL_TEMPERATURE + 273.15;
  
  if (resistance > 0) {
    float invT = 1.0 / T0_K + (1.0 / B_COEFFICIENT) * log(resistance / NOMINAL_RESISTANCE);
    float tempBeta = (1.0 / invT) - 273.15;
    tempNtcCalibrated = (CAL_SLOPE * tempBeta) + CAL_OFFSET;
  } else {
    tempNtcCalibrated = -99.99;
  }

  deltaT = abs(tempDS18B20 - tempNtcCalibrated);
  float weightDS, weightNTC;

  if (deltaT <= DELTA_THRESHOLD) {
    weightDS = 0.80;
    weightNTC = 0.20;
    modeState = "STEADY";
  } else {
    weightDS = 0.20;
    weightNTC = 0.80;
    modeState = "TRANSIENT";
  }
  tempFused = (weightDS * tempDS18B20) + (weightNTC * tempNtcCalibrated);

  if (!isnan(tempFused)) {
    if (tempFused < fusedMin) fusedMin = tempFused;
    if (tempFused > fusedMax) fusedMax = tempFused;
    fusedSum += tempFused;
    fusedCount++;
    fusedAvg = fusedSum / fusedCount;
  }
}

String sensorSuffix() {
  String s = "";
  s += ",DTEMP:" + String(tempDS18B20, 2) + 
       ",ATEMP:" + String(tempNtcCalibrated, 2) + 
       ",FTEMP:" + String(tempFused, 2) + 
       ",DIFF:" + String(deltaT, 2) + 
       ",STATE:" + modeState +
       ",FMIN:" + String(fusedMin, 2) +
       ",FMAX:" + String(fusedMax, 2) +
       ",FAVG:" + String(fusedAvg, 2);
  return s;
}

void gpsSleep() {
  gpsSerial.write(UBX_BACKUP_MODE, sizeof(UBX_BACKUP_MODE));
  delay(100);
}

void gpsWake() {
  gpsSerial.write(0xFF);
  delay(50);
  gpsSerial.write(UBX_WAKEUP_MODE, sizeof(UBX_WAKEUP_MODE));
  delay(500);
  while (gpsSerial.available()) gpsSerial.read();
}

void startLocate() {
  Serial.println("\n[LOCATE] Started. Searching indefinitely for satellites...");
  readingsCount = 0;
  sumLat = 0.0;
  sumLng = 0.0;
  locateStart = millis();
  lastReport = 0;
  lastReadingTime = 0;
  lastDetails = millis();
  gpsWake();
  charsAtStart = gps.charsProcessed();
  state = LOCATING;
}

void beginFinalReports(const String &packet) {
  finalPacket = packet;
  finalSent = 0;
  lastFinalSend = 0;
  gpsSleep();
  state = REPORTING;
}

void finishLocate() {
  double avgLat = sumLat / READINGS_NEEDED;
  double avgLng = sumLng / READINGS_NEEDED;
  int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
  readSensors();
  Serial.printf("\n[LOCATE] DONE in %lu s. Averaged Lat: %.6f, Lng: %.6f, Sats: %d\n",
                (millis() - locateStart) / 1000, avgLat, avgLng, sats);
  beginFinalReports("GPS:LAT:" + String(avgLat, 6) + ",LNG:" + String(avgLng, 6) +
                    ",SATS:" + String(sats) + sensorSuffix());
}

void updateLocating() {
  unsigned long now = millis();
  if (MAX_SEARCH_TIME_MS > 0 && now - locateStart > MAX_SEARCH_TIME_MS) {
    return;
  }

  int sats = gps.satellites.value();

  // Locks and acquires readings when 4 or more satellites are connected and location is valid
  if (readingsCount < READINGS_NEEDED && now - lastReadingTime >= READING_SPACING_MS &&
      sats >= 4 && gps.location.isValid() && gps.location.age() < 5000) {
    sumLat += gps.location.lat();
    sumLng += gps.location.lng();
    readingsCount++;
    lastReadingTime = now;
    Serial.printf("[GPS] Reading %d/%d acquired (Lat %.6f, Lng %.6f, Sats: %d)\n",
                  readingsCount, READINGS_NEEDED, gps.location.lat(), gps.location.lng(), sats);
  }

  if (readingsCount >= READINGS_NEEDED) {
    finishLocate();
    return;
  }

  if (now - lastReport >= REPORT_INTERVAL_MS) {
    lastReport = now;
    readSensors();
    loraSend("GPS:SEARCHING,SATS:" + String(sats) + ",READ:" + String(readingsCount) + sensorSuffix());
  }
}

void updateReporting() {
  if (finalSent >= FINAL_REPEATS) {
    state = IDLE;
    lastHeartbeat = millis();
    return;
  }
  if (finalSent == 0 || millis() - lastFinalSend >= FINAL_REPEAT_MS) {
    lastFinalSend = millis();
    finalSent++;
    loraSend(finalPacket);
  }
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd == "LOCATE_UNIT") {
    loraSend("ACK:LOCATE_UNIT");
    if (state != LOCATING) startLocate();
  }
}

void checkLoRa() {
  int size = LoRa.parsePacket();
  if (!size) return;
  String rx = "";
  while (LoRa.available()) rx += (char)LoRa.read();
  rx.trim();
  if (rx.startsWith("CMD:")) handleCommand(rx.substring(4));
}

void setup() {
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  delay(500);

  ds18b20.begin();
  analogReadResolution(12);
  readSensors();

  gpsSerial.setRxBufferSize(1024);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(100);
  gpsSleep();

  while (!loraInit()) {
    delay(2000);
  }
  lastHeartbeat = millis();
}

void loop() {
  while (gpsSerial.available() > 0) gps.encode(gpsSerial.read());

  checkLoRa();
  loraHealthCheck();
  readSensors();

  if (state == LOCATING) {
    updateLocating();
  } else if (state == REPORTING) {
    updateReporting();
  } else if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeat = millis();
    loraSend("HB:IDLE" + sensorSuffix());
  }
}