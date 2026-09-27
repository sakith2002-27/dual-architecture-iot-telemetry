/*
  Sensor Noise Measurement Tool
  EE2120 Distributed Smart Temperature Monitoring Project
  Logs steady-state data to compute Mean, Standard Deviation, and Peak-to-Peak Noise
*/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>

// ---------- Sensor Pins ----------
#define DS18B20_PIN   4   
#define NTC_PIN       34  

// ---------- Circuit Constants ----------
const float SERIES_RESISTOR     = 10000.0;
const float NOMINAL_RESISTANCE  = 10000.0;
const float NOMINAL_TEMPERATURE = 25.0;
const float B_COEFFICIENT       = 3950.0;
const float ADC_MAX             = 4095.0;
const float SUPPLY_VOLTAGE      = 3.3;

OneWire oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);

// ---------- Noise Test Configuration ----------
const int NUM_SAMPLES = 60;        // 60 samples
const int SAMPLE_DELAY_MS = 1000;  // 1 sample per second (60 seconds total)

float dsTemps[NUM_SAMPLES];
float ntcTemps[NUM_SAMPLES];

// =========================================================
// Sensor Read Functions
// =========================================================
float readDS18B20() {
  ds18b20.requestTemperatures();
  return ds18b20.getTempCByIndex(0);
}

float readThermistorBetaTemp() {
  int adcValue = analogRead(NTC_PIN);
  if (adcValue == 0) adcValue = 1; // Prevent division by zero
  float voltage = (adcValue / ADC_MAX) * SUPPLY_VOLTAGE;
  float resistance = SERIES_RESISTOR / (SUPPLY_VOLTAGE / voltage - 1.0);

  float T0_K = NOMINAL_TEMPERATURE + 273.15;
  float invT = 1.0 / T0_K + (1.0 / B_COEFFICIENT) * log(resistance / NOMINAL_RESISTANCE);
  return (1.0 / invT) - 273.15;
}

// =========================================================
// Statistical Functions
// =========================================================
void calculateAndPrintStats(const char* sensorName, float data[], int n) {
  float sum = 0;
  float minVal = data[0];
  float maxVal = data[0];

  // Calculate Mean, Min, Max
  for (int i = 0; i < n; i++) {
    sum += data[i];
    if (data[i] < minVal) minVal = data[i];
    if (data[i] > maxVal) maxVal = data[i];
  }
  float mean = sum / n;

  // Calculate Standard Deviation
  float varianceSum = 0;
  for (int i = 0; i < n; i++) {
    varianceSum += pow((data[i] - mean), 2);
  }
  float stdDev = sqrt(varianceSum / n);
  float peakToPeak = maxVal - minVal;

  // Print Results
  Serial.print("--- "); Serial.print(sensorName); Serial.println(" ---");
  Serial.print("Mean: \t\t"); Serial.print(mean, 4); Serial.println(" C");
  Serial.print("Std Dev: \t"); Serial.print(stdDev, 4); Serial.println(" C");
  Serial.print("Peak-to-Peak: \t"); Serial.print(peakToPeak, 4); Serial.println(" C\n");
}

void runNoiseTest() {
  Serial.println("\nStarting 60-second noise test...");
  Serial.println("DO NOT touch the sensors or move them. Keep them in a stable room-temperature environment.");
  
  for (int i = 0; i < NUM_SAMPLES; i++) {
    dsTemps[i] = readDS18B20();
    ntcTemps[i] = readThermistorBetaTemp();
    
    Serial.print("Sample "); Serial.print(i + 1); Serial.print("/"); Serial.println(NUM_SAMPLES);
    delay(SAMPLE_DELAY_MS);
  }

  Serial.println("\n================ NOISE TEST RESULTS ================\n");
  calculateAndPrintStats("DS18B20 (Digital Reference)", dsTemps, NUM_SAMPLES);
  calculateAndPrintStats("NTC Thermistor (Analog Uncalibrated)", ntcTemps, NUM_SAMPLES);
  Serial.println("====================================================");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  ds18b20.begin();
  analogReadResolution(12);

  Serial.println("\n=== EE2120 Noise Measurement Tool ===");
  Serial.println("Type 'n' and press Enter to begin the noise test.");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    while (Serial.available()) Serial.read(); // Flush buffer

    if (cmd == 'n' || cmd == 'N') {
      runNoiseTest();
      Serial.println("Type 'n' to run the test again.");
    }
  }
}