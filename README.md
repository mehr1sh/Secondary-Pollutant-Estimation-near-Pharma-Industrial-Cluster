# Environmental Air Quality Monitoring System - Diwali Deployment

## Project Overview

This project is an **Environmental Monitoring System** designed to measure air quality and environmental parameters near pharmaceutical industries during festive periods like Diwali. The system uses multiple sensors connected to an ESP32 microcontroller to collect real-time data on pollutants, temperature, humidity, and UV radiation. All data is logged locally on an SD card and uploaded to ThingSpeak cloud platform for remote monitoring.

## Motivation

Diwali firecracker usage causes sudden spikes in air pollution, particularly particulate matter (PM2.5, PM10) and volatile organic compounds (VOCs). Our system was designed to accurately capture and analyze these pollution events near industrial areas to understand their environmental impact and validate sensor reliability.

## Hardware Components

| Sensor | Parameter | Communication Protocol | Pin(s) |
|--------|-----------|----------------------|--------|
| **SDS011** | PM2.5, PM10 | UART | 16 (RX), 17 (TX) |
| **SGP30** | TVOC, eCO2 | I²C | 21 (SDA), 22 (SCL) |
| **AHT10** | Temperature, Humidity | I²C | 25 (SDA), 26 (SCL) |
| **HW-131 UV Sensor** | UV Intensity | Analog | 34 |
| **MiCS-2714** | NO₂ (Estimated) | Analog | 35 |
| **ESP32** | Main Microcontroller | - | - |
| **SD Card Module** | Data Storage | SPI | 5 (CS) |

## Sensor Protocols Explained

- **UART (SDS011)**: Serial communication with fixed 9600 baud rate. Sends 10-byte packets containing PM readings.
- **I²C (SGP30, AHT10)**: Two-wire digital protocol using SDA (data) and SCL (clock) lines. Allows multiple sensors on same bus.
- **Analog (UV, MiCS-2714)**: Direct voltage reading via ADC pins. Output converted to physical units using calibration factors.

## Data Collection Process

1. **Sensor Initialization**: All sensors initialized during startup with error handling for missing devices.
2. **Continuous Polling**: SDS011 UART data polled in real-time in the main loop.
3. **Periodic Sampling**: Every 6 seconds, temperature, humidity, TVOC, eCO2, UV, and NO₂ are sampled.
4. **Local Logging**: All measurements timestamped and appended to `/env_data.csv` on SD card.
5. **Cloud Upload**: Every 15 seconds, data sent to ThingSpeak API (if WiFi connected).
6. **Offline Mode**: If WiFi unavailable, system continues logging to SD without cloud upload.

## Data Fields

The system logs the following parameters:

```
Date, Time, PM2.5, PM10, TVOC, eCO2, Temperature, Humidity, UV, NO2, RSSI
```

- **Date/Time**: Synced via NTP from time.google.com (IST: UTC+5:30). Falls back to uptime if offline.
- **PM2.5/PM10**: Particulate matter in μg/m³ from SDS011.
- **TVOC**: Total Volatile Organic Compounds in ppb from SGP30.
- **eCO2**: Estimated CO₂ in ppm from SGP30.
- **Temperature/Humidity**: From AHT10 in °C and %.
- **UV**: UV intensity estimation in arbitrary units.
- **NO₂**: NO₂ concentration estimation in arbitrary units.
- **RSSI**: WiFi signal strength indicator.

## Key Findings from Diwali Deployment

- **PM2.5 Peak**: 656.2 μg/m³ (21:25 IST on Oct 20) - **4275% above WHO guideline**.
- **PM10 Peak**: 736.3 μg/m³ - **636% above India NAAQS standard**.
- **Peak Duration**: High pollution levels sustained for 6-7 hours after initial firecracker burst.
- **Temporal Pattern**: Early morning peaks (02:00-06:00 UTC) due to atmospheric stability; evening peaks (16:00-20:00 UTC) from firecracker activity.
- **Sensor Reliability**: Digital I²C sensors (AHT10) showed consistent, reliable data. Analog MOX sensors exhibited volatility and drift.

## Dual-System Architecture

To address sensor unreliability issues:

- **System 1 (Core Reliability Node)**: SDS011 (UART) + AHT10 (I²C) + SGP30 (I²C) - Reliable, proven sensors.
- **System 2 (Experimental Node)**: MQ135 (Analog) + MiCS-2714 (Analog) - Testing ground for less stable sensors.

Each system has independent ESP32, power supply, and SD card to prevent single-point failure.

## Code Structure

### Main Components:

1. **Sensor Initialization** (`setup()`):
   - Initialize UART for SDS011
   - Initialize I²C buses for SGP30 and AHT10
   - Setup SD card and create CSV header
   - Connect to WiFi and sync NTP time

2. **Data Acquisition** (`loop()`):
   - `checkSDS()`: Parse UART packet from PM sensor
   - Read SGP30 (TVOC, eCO2) calibrated with AHT10 humidity
   - Read AHT10 (temperature, humidity)
   - `readUVSensor()`: Analog read with averaging (5 samples)
   - `readMICS2714()`: Analog read for NO₂ estimation

3. **Data Output**:
   - `printAllData()`: Serial output with network info
   - `logToSD()`: Append row to CSV file
   - `sendToThingSpeak()`: HTTP POST to cloud (8 fields max)

4. **Time Handling**:
   - Online: NTP sync via Google NTP server
   - Offline: Fallback to uptime counter

## File Structure

```
project/
├── final_codes/
│   └── main_sensor_code.cpp (or .ino)
├── test_codes/
│   └── AHT10_test_code.cpp
├── csv_files/
│   └── env_data.csv (generated at runtime)
├── diwali_report.pdf
├── README.md (this file)
└── plots/ (diurnal profiles)
```

## Libraries Used

```cpp
#include <Arduino.h>
#include <Wire.h>              // I²C communication
#include <SPI.h>               // SPI for SD card
#include <SD.h>                // SD card operations
#include <WiFi.h>              // WiFi connectivity
#include <HTTPClient.h>        // HTTP requests to ThingSpeak
#include <time.h>              // NTP time
#include "Adafruit_SGP30.h"    // SGP30 library
#include "Adafruit_AHTX0.h"    // AHT10 library
```

## WiFi & ThingSpeak Configuration

- **SSID**: Network name (configured in code)
- **Password**: Network password (configured in code)
- **ThingSpeak API Key**: `KU43UIWQ9Q6JJIAN`
- **Update Interval**: 15 seconds (ThingSpeak free tier minimum)
- **Rate Limit Handling**: 15-second minimum between uploads enforced

## Data Analysis Results

### Correlation Matrix:
- PM2.5 ↔ PM10: **r = 0.98** (very strong correlation, common sources)
- eVOC ↔ eCO2: **r = 0.82** (strong correlation, VOC-derived)
- PM ↔ Temperature: **r = -0.12** (weak inverse, atmospheric stability effect)
- PM ↔ Humidity: **r = -0.22** (weak inverse, particle growth effect)

### Key Conclusion:
The system successfully captured severe pollution during Diwali with PM concentrations exceeding international standards by 40-50x. The dual-system architecture proved essential in maintaining data reliability despite sensor challenges.

## Future Improvements

1. **Expand Monitoring**: Deploy additional nodes across different geographic locations.
2. **Real-time Dashboard**: Build web/mobile app for live data visualization.
3. **Cross-validation**: Compare ground data with satellite measurements and government stations.
4. **Public Alerts**: Integrate health advisories and automatic emergency alerts.
5. **AI/ML Models**: Predict secondary pollutant levels using primary pollutant data.
6. **Long-term Studies**: Extend analysis to other pollution events beyond Diwali.

## Deployment Notes

- **Location**: Kokapet, Hyderabad (17.3968°N, 78.3348°E)
- **Elevation**: 7th floor (~22-27m above ground)
- **Deployment Duration**: Oct 20-21, 2025 (22+ hours continuous)
- **Data Points Collected**: 1,527 measurements
- **Sampling Frequency**: 6 seconds (regular), 20 seconds during peak pollution

## Troubleshooting

| Issue | Cause | Fix |
|-------|-------|-----|
| SD card not initializing | Wrong CS pin or SD card corrupted | Check pin config, reformat SD |
| No UART data from SDS011 | Wrong baud rate or loose connection | Verify 9600 baud, check wiring |
| I²C sensor not detected | Address mismatch or wrong pins | Use I²C scanner, verify SDA/SCL |
| WiFi connects but ThingSpeak fails | API key expired or no internet | Verify API key and connectivity |
| Time shows uptime instead of NTP | NTP server unreachable | Check internet, verify ntpServer URL |

## References

- SDS011 Datasheet: UART PM sensor protocol
- SGP30 Datasheet: I²C VOC sensor calibration
- AHT10 Datasheet: I²C temperature/humidity sensor
- ESP32 Documentation: Pin mapping and peripheral config
- ThingSpeak API: Cloud data logging platform

---

**Last Updated**: November 12, 2025  
**Status**: Active / Tested during Diwali 2025  
**Team**: Environmental Systems Workshop (ESW), IIIT Hyderabad
