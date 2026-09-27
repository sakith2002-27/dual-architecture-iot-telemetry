/*
  Phase 4: Adaptive Weighted Sensor Fusion Firmware
  EE2120 Distributed Smart Temperature Monitoring Project
  
  Logic:
  - Small Delta T (<= 1.0 C): Steady state -> 80% DS18B20 / 20% NTC
  - Large Delta T (> 1.0 C) : Transient state -> 20% DS18B20 / 80% NTC
*/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>

// ---------- Hardware Pin Definitions ----------
#define DS18B20_PIN   4   
#define NTC_PIN       34  

// ---------- Circuit Constants ----------
const float SERIES_RESISTOR     = 10000.0;
const float NOMINAL_RESISTANCE  = 10000.0;
const float NOMINAL_TEMPERATURE = 25.0;
const float B_COEFFICIENT       = 3950.0;
const float ADC_MAX             = 4095.0;
const float SUPPLY_VOLTAGE      = 3.3;

// ---------- Linear Calibration Constants (NTC) ----------
const float CAL_SLOPE  = 1.01120;
const float CAL_OFFSET = -5.25259;

// ---------- Fusion Settings ----------
const float DELTA_THRESHOLD = 1.50; // Threshold in C to switch modes

OneWire oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);

void setup() {
  Serial.begin(115200);
  ds18b20.begin();
  analogReadResolution(12);
  
  Serial.println("==========================================================================================");
  Serial.println("                  Phase 4: Adaptive Weighted Sensor Fusion System                        ");
  Serial.println("==========================================================================================");
  Serial.println("DS18B20 (C)\t|\tNTC Cal (C)\t|\tFused Temp (C)\t|\tDelta T (C)\t|\tState");
  Serial.println("------------------------------------------------------------------------------------------");
}

void loop() {
  // 1. Read Digital Sensor (DS18B20)
  ds18b20.requestTemperatures();
  float tempDS18B20 = ds18b20.getTempCByIndex(0);

  // 2. Read Analog Sensor (NTC Thermistor) and apply Beta Equation
  int adcValue = analogRead(NTC_PIN);
  if (adcValue == 0) adcValue = 1; 
  float voltage = (adcValue / ADC_MAX) * SUPPLY_VOLTAGE;
  float resistance = SERIES_RESISTOR / (SUPPLY_VOLTAGE / voltage - 1.0);

  float T0_K = NOMINAL_TEMPERATURE + 273.15;
  float invT = 1.0 / T0_K + (1.0 / B_COEFFICIENT) * log(resistance / NOMINAL_RESISTANCE);
  float tempBeta = (1.0 / invT) - 273.15;

  // 3. Apply Calibration Correction
  float tempNtcCalibrated = (CAL_SLOPE * tempBeta) + CAL_OFFSET;

  // 4. Calculate Absolute Difference (Delta T)
  float deltaT = abs(tempDS18B20 - tempNtcCalibrated);

  // 5. Adaptive Fusion Weight Assignment
  float weightDS, weightNTC;
  String modeState;

  if (deltaT <= DELTA_THRESHOLD) {
    // Steady State: Trust DS18B20 for accuracy
    weightDS = 0.80;
    weightNTC = 0.20;
    modeState = "STEADY (80/20)";
  } else {
    // Transient State: Trust NTC for fast response
    weightDS = 0.20;
    weightNTC = 0.80;
    modeState = "TRANSIENT (20/80)";
  }

  // 6. Compute Final Fused Output
  float tempFused = (weightDS * tempDS18B20) + (weightNTC * tempNtcCalibrated);

  // 7. Output Data
  Serial.print(tempDS18B20, 2);
  Serial.print("\tDS18B20\t|\t");
  Serial.print(tempNtcCalibrated, 2);
  Serial.print("\tThermistor\t|\t");
  Serial.print(tempFused, 2);
  Serial.print("\tFusion\t|\t");
  Serial.print(deltaT, 2);
  Serial.print("\tDifference\t|\t");
  Serial.println(modeState);

  delay(1000);
}