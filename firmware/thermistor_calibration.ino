/*
  DS18B20-Referenced Thermistor Calibration Tool
  EE2120 Distributed Smart Temperature Monitoring Project
  Applies the Beta equation to the NTC and uses Least Squares Fitting 
  against the DS18B20 reference to generate final correction factors.
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

  // Steinhart-Hart Beta Equation
  float T0_K = NOMINAL_TEMPERATURE + 273.15;
  float invT = 1.0 / T0_K + (1.0 / B_COEFFICIENT) * log(resistance / NOMINAL_RESISTANCE);
  return (1.0 / invT) - 273.15;
}

// =========================================================
// Math Helpers
// =========================================================
float computeMean(float x[], int n) {
  float sum = 0;
  for (int i = 0; i < n; i++) sum += x[i];
  return sum / n;
}

void linearFit(float x[], float y[], int n, float &m, float &c, float &r2) {
  float meanX = computeMean(x, n);
  float meanY = computeMean(y, n);

  float num = 0, den = 0;
  for (int i = 0; i < n; i++) {
    num += (x[i] - meanX) * (y[i] - meanY);
    den += (x[i] - meanX) * (x[i] - meanX);
  }
  m = num / den;
  c = meanY - m * meanX;

  float ssTot = 0, ssRes = 0;
  for (int i = 0; i < n; i++) {
    float pred = m * x[i] + c;
    ssRes += (y[i] - pred) * (y[i] - pred);
    ssTot += (y[i] - meanY) * (y[i] - meanY);
  }
  r2 = 1.0 - ssRes / ssTot;
}

// =========================================================
// Calibration Data Logging
// =========================================================
#define MAX_CAL_POINTS 10
float refDsTemp[MAX_CAL_POINTS];
float rawNtcTemp[MAX_CAL_POINTS];
int numCalPoints = 0;

void addCalibrationPoint() {
  if (numCalPoints >= MAX_CAL_POINTS) {
    Serial.println("Max points reached. Type 'f' to finish.");
    return;
  }

  Serial.println("\n--- Capturing Data Point ---");
  Serial.println("Averaging readings over 15 seconds. Keep probes steady...");

  const int N = 15;
  float dsBuf[N], ntcBuf[N];

  for (int i = 0; i < N; i++) {
    dsBuf[i]  = readDS18B20();
    ntcBuf[i] = readThermistorBetaTemp();
    delay(1000);
  }

  refDsTemp[numCalPoints]  = computeMean(dsBuf, N);
  rawNtcTemp[numCalPoints] = computeMean(ntcBuf, N);

  Serial.print("Point "); Serial.print(numCalPoints + 1);
  Serial.print(" | Ref (DS18B20): "); Serial.print(refDsTemp[numCalPoints], 2);
  Serial.print(" C | Raw NTC (Beta): "); Serial.print(rawNtcTemp[numCalPoints], 2);
  Serial.println(" C");

  numCalPoints++;
}

void finishCalibrationAndReport() {
  if (numCalPoints < 3) {
    Serial.println("\n[!] Need at least 3 points across different temperatures (e.g., Ice, Room Temp, Hot).");
    return;
  }

  float m_ntc, c_ntc, r2_ntc;
  // X = Raw NTC Beta Temp, Y = DS18B20 True Temp
  linearFit(rawNtcTemp, refDsTemp, numCalPoints, m_ntc, c_ntc, r2_ntc);

  Serial.println("\n================ CALIBRATION RESULTS ================\n");
  Serial.println("Apply this correction formula to your NTC readings in Phase 4:\n");

  Serial.print("Slope (m): \t"); Serial.println(m_ntc, 5);
  Serial.print("Offset (c): \t"); Serial.println(c_ntc, 5);
  Serial.print("Fit (R^2): \t"); Serial.println(r2_ntc, 5);
  
  Serial.println("\nFormula:");
  Serial.print("T_calibrated = ("); Serial.print(m_ntc, 5);
  Serial.print(" * T_beta) + ("); Serial.print(c_ntc, 5);
  Serial.println(")");
  Serial.println("\n=====================================================\n");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  ds18b20.begin();
  analogReadResolution(12);

  Serial.println("\n=== EE2120 Auto-Calibration Tool ===");
  Serial.println("  c - Capture a steady-state temperature point (averages 15s)");
  Serial.println("  f - Finish and calculate m & c correction constants");
  Serial.println("  x - Clear stored points");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    while (Serial.available()) Serial.read(); 

    switch (cmd) {
      case 'c': addCalibrationPoint(); break;
      case 'f': finishCalibrationAndReport(); break;
      case 'x': 
        numCalPoints = 0; 
        Serial.println("Points cleared."); 
        break;
      default: break;
    }
  }
}