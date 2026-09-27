# Hardware Pinout and Wiring Reference

## Slave (Sensor) Node — ESP32 DOIT DevKit V1

| Signal | ESP32 Pin | Notes |
|---|---|---|
| DS18B20 data | GPIO4 | 4.7 kΩ pull-up to 3.3V; 1-Wire protocol |
| Thermistor divider midpoint | GPIO34 (ADC1) | Divider: 3.3V → 10 kΩ fixed resistor → GPIO34 → thermistor → GND |
| AHT10 SDA / SCL | GPIO21 / GPIO22 | I²C, address 0x38 |
| GPS module RX / TX | GPIO16 / GPIO17 | UART2, NEO-8M, 9600 baud, NMEA |
| LoRa SCK / MISO / MOSI | GPIO18 / GPIO19 / GPIO23 | SPI (VSPI) bus |
| LoRa NSS (CS) | GPIO5 | |
| LoRa RST | GPIO12 | |
| LoRa DIO0 | GPIO13 | |
| Power | 3.3V / GND | LoRa module is 3.3V ONLY — not 5V tolerant |

## Master (Gateway) Node — ESP32 DOIT DevKit V1

| Signal | ESP32 Pin | Notes |
|---|---|---|
| LoRa SCK / MISO / MOSI | GPIO18 / GPIO19 / GPIO23 | Same SPI pin assignment as slave, for hardware consistency |
| LoRa NSS (CS) | GPIO5 | |
| LoRa RST | GPIO12 | |
| LoRa DIO0 | GPIO13 | |
| WiFi | (internal) | Connects to local network for internet uplink |
| Power | 3.3V / GND | |

## Notes

- **Why GPIO34 for the thermistor**: GPIO34 is an ADC1-channel, input-only
  pin. ADC2 channels share hardware with the ESP32's WiFi radio and become
  unreliable once WiFi is active — ADC1 avoids this entirely.
- **LoRa frequency**: 433 MHz, matched to the ISM-band allocation used in
  this deployment region. Confirm your local regulatory allocation (FCC
  Part 15 in the US, or your national equivalent) before deploying.
- **LoRa RST/DIO0 pin choice** (GPIO12/13) differs from some earlier
  firmware drafts (GPIO14/26) — always check the pin definitions at the top
  of the `.ino` file match your actual physical wiring before flashing.
