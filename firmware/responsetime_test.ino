/*
  Sensor Response Time Measurement Tool
  EE2120 Distributed Smart Temperature Monitoring Project
  Measures the thermal time constant (tau, 63.2%) and 90% settling time (t90)
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

// ---------- Test Configuration ----------
const int NUM_SAMPLES = 150;       // 150 samples
const int SAMPLE_DELAY_MS = 200;   // 5 samples per second (30 seconds total)

float dsTemps[NUM_SAMPLES];
float ntcTemps[NUM_SAMPLES];
unsigned long timeMs[NUM_SAMPLES];

// =========================================================
// Sensor Read Functions
// =========================================================
float readDS18B20() {
  ds18b20.requestTemperatures();
  return ds18b20.getTempCByIndex(0);
}

float readThermistorBetaTemp() {
  int adcValue = analogRead(NTC_PIN);
  if (adcValue == 0) adcValue = 1; 
  float voltage = (adcValue / ADC_MAX) * SUPPLY_VOLTAGE;
  float resistance = SERIES_RESISTOR / (SUPPLY_VOLTAGE / voltage - 1.0);

  float T0_K = NOMINAL_TEMPERATURE + 273.15;
  float invT = 1.0 / T0_K + (1.0 / B_COEFFICIENT) * log(resistance / NOMINAL_RESISTANCE);
  return (1.0 / invT) - 273.15;
}

// =========================================================
// Math & Analysis
// =========================================================
float computeMean(float data[], int startIdx, int count) {
  float sum = 0;
  for (int i = startIdx; i < startIdx + count; i++) {
    sum += data[i];
  }
  return sum / count;
}

void analyzeResponse(const char* sensorName, float temps[], unsigned long times[], int n) {
  // Use first 5 samples as initial temp (T0) and last 5 as final temp (Tf)
  float t0_temp = computeMean(temps, 0, 5);
  float tf_temp = computeMean(temps, n - 5, 5);
  
  float deltaT = tf_temp - t0_temp;
  float target63 = t0_temp + (0.632 * deltaT); // 63.2% of change (Tau)
  float target90 = t0_temp + (0.900 * deltaT); // 90.0% of change (t90)
  
  float time63 = -1.0;
  float time90 = -1.0;
  bool heating = deltaT > 0;

  // Find when the temperature crossed the target thresholds
  for (int i = 1; i < n; i++) {
    bool crossed63 = heating ? (temps[i] >= target63) : (temps[i] <= target63);
    bool crossed90 = heating ? (temps[i] >= target90) : (temps[i] <= target90);
    
    if (time63 < 0 && crossed63) {
      // Linear interpolation for more accurate sub-sample timing
      float frac = (target63 - temps[i-1]) / (temps[i] - temps[i-1] + 1e-6);
      time63 = (times[i-1] + frac * (times[i] - times[i-1])) / 1000.0; 
    }
    if (time90 < 0 && crossed90) {
      float frac = (target90 - temps[i-1]) / (temps[i] - temps[i-1] + 1e-6);
      time90 = (times[i-1] + frac * (times[i] - times[i-1])) / 1000.0;
    }
  }

  Serial.print("--- "); Serial.print(sensorName); Serial.println(" ---");
  Serial.print("Initial Temp (T0): \t"); Serial.print(t0_temp, 2); Serial.println(" C");
  Serial.print("Final Temp (Tf): \t"); Serial.print(tf_temp, 2); Serial.println(" C");
  
  if (time63 > 0) {
    Serial.print("Time Constant (Tau 63.2%): \t"); Serial.print(time63, 2); Serial.println(" seconds");
  } else {
    Serial.println("Time Constant (Tau 63.2%): \tDid not reach threshold within 30s.");
  }
  
  if (time90 > 0) {
    Serial.print("Settling Time (t90): \t\t"); Serial.print(time90, 2); Serial.println(" seconds\n");
  } else {
    Serial.println("Settling Time (t90): \t\tDid not reach threshold within 30s.\n");
  }
}

void runResponseTest() {
  Serial.println("\n--- GET READY ---");
  Serial.println("1. Hold the sensors at room temperature.");
  Serial.println("2. Have a bath of hot or cold water ready.");
  Serial.println("3. Type 'g' and press Enter to start sampling.");
  
  while (!Serial.available()) delay(10);
  while (Serial.available()) Serial.read(); // Flush buffer

  Serial.println("\n[!] SAMPLING STARTED - PLUNGE SENSORS NOW!");
  
  unsigned long startTime = millis();
  for (int i = 0; i < NUM_SAMPLES; i++) {
    timeMs[i] = millis() - startTime;
    dsTemps[i] = readDS18B20();
    ntcTemps[i] = readThermistorBetaTemp();
    delay(SAMPLE_DELAY_MS);
  }

  Serial.println("\n================ RESPONSE TEST RESULTS ================\n");
  analyzeResponse("DS18B20 (Digital Reference)", dsTemps, timeMs, NUM_SAMPLES);
  analyzeResponse("NTC Thermistor (Analog Uncalibrated)", ntcTemps, timeMs, NUM_SAMPLES);
  Serial.println("=======================================================");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  ds18b20.begin();
  analogReadResolution(12);

  Serial.println("\n=== EE2120 Response Time Tool ===");
  Serial.println("Type 'r' and press Enter to begin the response time test protocol.");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    while (Serial.available()) Serial.read(); 

    if (cmd == 'r' || cmd == 'R') {
      runResponseTest();
      Serial.println("Type 'r' to run the test again.");
    }
  }
}