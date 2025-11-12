#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <time.h>
#include <HTTPClient.h>
#include "Adafruit_SGP30.h"
#include "Adafruit_AHTX0.h"

// --- ThingSpeak Configuration ---
const char* thingSpeakServer = "http://api.thingspeak.com/update";
String thingSpeakAPIKey = "KU43UIWQ9Q6JJIAN"; // Replace with your Write API Key
unsigned long lastThingSpeakUpdate = 0;
const long thingSpeakInterval = 15000; // 15 seconds minimum (ThingSpeak rate limit)

// --- NTP Time Configuration ---
const char* ntpServer = "time.google.com";
const long gmtOffset_sec = 19800; // IST (UTC +5:30)
const int daylightOffset_sec = 0;

// --- Wi-Fi Credentials ---
const char* ssid     = "JioFi_20F9980";
const char* password = "houmcfjg2h";


// --- Sensor Objects ---
HardwareSerial sdsSerial(2);
Adafruit_SGP30 sgp;
Adafruit_AHTX0 aht10;

// --- Pin Definitions ---
#define SDS_RX_PIN 16
#define SDS_TX_PIN 17
#define SGP30_SDA_PIN 21
#define SGP30_SCL_PIN 22
#define AHT10_SDA_PIN 25
#define AHT10_SCL_PIN 26
#define UV_PIN 34
#define MICS2714_PIN 35
#define SD_CS_PIN 5

// --- Global Variables ---
float globalPM25 = 0.0, globalPM10 = 0.0;
uint16_t tvoc = 0, eCO2_sgp = 0;
float temperature_aht10 = 0.0, humidity_aht10 = 0.0;
float uvIntensity = 0.0, no2_concentration = 0.0;
char timeBuffer[30];
char dateBuffer[20];
char clockBuffer[20];

// --- Timer ---
unsigned long prevLoopMillis = 0;
const long loopInterval = 6000; // 15 seconds for SD/Serial logging

// --- Forward Declarations ---
bool checkSDS();
void printAllData();
void logToSD();
void sendToThingSpeak();
uint32_t getAbsoluteHumidity(float temperature, float humidity);
float readUVSensor();
float readMICS2714();

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Initializing Environmental Node...");

  // --- 1. Initialize Sensors ---
  sdsSerial.begin(9600, SERIAL_8N1, SDS_RX_PIN, SDS_TX_PIN);
  Wire.begin(SGP30_SDA_PIN, SGP30_SCL_PIN);
  if (!sgp.begin()) Serial.println("Warning: SGP30 not found");
  Wire1.begin(AHT10_SDA_PIN, AHT10_SCL_PIN);
  if (!aht10.begin(&Wire1)) Serial.println("Warning: AHT10 not found");
  pinMode(UV_PIN, INPUT);
  pinMode(MICS2714_PIN, INPUT);

  // --- 2. Initialize SD Card ---
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("FAILED! (Continuing without SD logging)");
  } else {
    Serial.println("OK!");
    // Create CSV header if file doesn't exist
    if (!SD.exists("/env_data.csv")) {
      File file = SD.open("/env_data.csv", FILE_WRITE);
      if (file) {
        // Added RSSI to the header
        file.println("Date,Time,PM2.5,PM10,TVOC,eCO2,Temp,Humidity,UV,NO2,RSSI");
        file.close();
      }
    }
  }

  // --- 3. Connect Wi-Fi & Sync Time (Non-blocking 10s timeout) ---
  Serial.printf("Connecting to Wi-Fi: %s", ssid);
  WiFi.begin(ssid, password);
  
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(500); Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWi-Fi connected! Syncing time...");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    // Small delay to allow NTP packet to arrive
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 5000)) {
       Serial.println("Time synced successfully.");
    } else {
       Serial.println("NTP unreachable, will use uptime.");
    }
    Serial.println("ThingSpeak integration enabled.");
  } else {
    Serial.println("\nWi-Fi failed! Continuing in offline mode.");
    Serial.println("ThingSpeak upload DISABLED (no internet).");
  }

  Serial.println("\n--- Starting Measurements ---");
}

void loop() {
  checkSDS(); // Continuously poll PM sensor

  unsigned long currentMillis = millis();
  
  // --- Regular logging (6 seconds) ---
  if (currentMillis - prevLoopMillis >= loopInterval) {
    prevLoopMillis = currentMillis;

    // --- Robust Time Handling ---
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      strftime(timeBuffer, 30, "%Y-%m-%d %H:%M:%S", &timeinfo);
      strftime(dateBuffer, 20, "%Y-%m-%d", &timeinfo);
      strftime(clockBuffer, 20, "%H:%M:%S", &timeinfo);
    } else {
      // Offline fallback: show Uptime
      unsigned long seconds = currentMillis / 1000;
      int days = seconds / 86400;
      int hours = (seconds % 86400) / 3600;
      int mins = (seconds % 3600) / 60;
      int secs = seconds % 60;
      snprintf(dateBuffer, 20, "Day %d", days);
      snprintf(clockBuffer, 20, "%02d:%02d:%02d (Uptime)", hours, mins, secs);
      snprintf(timeBuffer, 30, "Uptime: %dd %02d:%02d:%02d", days, hours, mins, secs);
    }

    // --- Read Sensors ---
    sensors_event_t humidity, temp;
    if (aht10.getEvent(&humidity, &temp)) {
      temperature_aht10 = temp.temperature;
      humidity_aht10 = humidity.relative_humidity;
      // Use AHT10 data to calibrate SGP30 for better accuracy
      sgp.setHumidity(getAbsoluteHumidity(temperature_aht10, humidity_aht10));
    }

    if (sgp.IAQmeasure()) {
      tvoc = sgp.TVOC;
      eCO2_sgp = sgp.eCO2;
    }

    uvIntensity = readUVSensor();
    no2_concentration = readMICS2714();

    // --- Output ---
    printAllData();
    logToSD();
  }

  // --- ThingSpeak upload (15 seconds minimum) ---
  if (WiFi.status() == WL_CONNECTED && 
      (currentMillis - lastThingSpeakUpdate >= thingSpeakInterval)) {
    lastThingSpeakUpdate = currentMillis;
    sendToThingSpeak();
  }
}

void sendToThingSpeak() {
  HTTPClient http;
  
  // Build ThingSpeak URL with data fields
  // Field mapping:
  // field1 = PM2.5, field2 = PM10
  // field3 = TVOC, field4 = eCO2
  // field5 = Temperature, field6 = Humidity
  // field7 = UV Intensity, field8 = NO2
  // Note: RSSI is dropped to fit 8-field limit
  
  String url = String(thingSpeakServer) + "?api_key=" + thingSpeakAPIKey;
  url += "&field1=" + String(globalPM25, 1);
  url += "&field2=" + String(globalPM10, 1);
  url += "&field3=" + String(tvoc);
  url += "&field4=" + String(eCO2_sgp);
  url += "&field5=" + String(temperature_aht10, 1);
  url += "&field6=" + String(humidity_aht10, 1);
  url += "&field7=" + String(uvIntensity, 2);
  url += "&field8=" + String(no2_concentration, 2);

  http.begin(url);
  int httpCode = http.GET();
  
  if (httpCode > 0) {
    String payload = http.getString();
    if (httpCode == 200) {
      Serial.print(">> ThingSpeak: Data uploaded successfully (Entry ID: ");
      Serial.print(payload);
      Serial.println(")");
    } else {
      Serial.printf(">> ThingSpeak: Upload failed with code %d\n", httpCode);
    }
  } else {
    Serial.printf(">> ThingSpeak: Connection failed: %s\n", http.errorToString(httpCode).c_str());
  }
  
  http.end();
}

void logToSD() {
  File file = SD.open("/env_data.csv", FILE_APPEND);
  if (file) {
    file.print(dateBuffer); file.print(",");
    file.print(clockBuffer); file.print(",");
    file.print(globalPM25); file.print(",");
    file.print(globalPM10); file.print(",");
    file.print(tvoc); file.print(",");
    file.print(eCO2_sgp); file.print(",");
    file.print(temperature_aht10); file.print(",");
    file.print(humidity_aht10); file.print(",");
    file.print(uvIntensity); file.print(",");
    file.print(no2_concentration); file.print(",");
    
    // Log RSSI if connected, else 0
    if (WiFi.status() == WL_CONNECTED) {
        file.println(WiFi.RSSI());
    } else {
        file.println("0");
    }
    
    file.close();
    Serial.println(">> Data logged to SD");
  }
}

void printAllData() {
  Serial.println("=== ENVIRONMENTAL NODE ===");
  Serial.println(timeBuffer);

  // --- Network Details ---
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("IP: "); Serial.print(WiFi.localIP());
    Serial.print(" | MAC: "); Serial.print(WiFi.macAddress());
    Serial.print(" | RSSI: "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
  } else {
    Serial.println("Network: Disconnected (Offline Mode)");
  }
  Serial.println("--------------------------");

  // --- Sensor Data ---
  Serial.printf("PM2.5: %.1f | PM10: %.1f µg/m³\n", globalPM25, globalPM10);
  Serial.printf("TVOC: %d ppb | eCO2: %d ppm\n", tvoc, eCO2_sgp);
  Serial.printf("T: %.1f °C | H: %.1f %%\n", temperature_aht10, humidity_aht10);
  Serial.printf("UV: %.2f | NO2 (est): %.2f\n", uvIntensity, no2_concentration);
  Serial.println("==========================");
}

bool checkSDS() {
  byte buffer[10];
  while (sdsSerial.available() > 0) {
    if (sdsSerial.peek() != 0xAA) { sdsSerial.read(); continue; }
    if (sdsSerial.available() < 10) return false;
    sdsSerial.readBytes(buffer, 10);
    if (buffer[0] == 0xAA && buffer[1] == 0xC0 && buffer[9] == 0xAB) {
      globalPM25 = ((buffer[3] * 256) + buffer[2]) / 10.0;
      globalPM10 = ((buffer[5] * 256) + buffer[4]) / 10.0;
      return true;
    }
  }
  return false;
}

float readUVSensor() {
  int total = 0;
  for (int i = 0; i < 5; i++) { total += analogRead(UV_PIN); delay(5); }
  // Basic conversion: (Average Analog / Max ADC) * Ref Voltage * Scale Factor
  return (total / 5) * (3.3 / 4095.0) * 15.0;
}

float readMICS2714() {
  // Basic estimated conversion
  return analogRead(MICS2714_PIN) * (5.0 / 4095.0) * 1.8;
}

uint32_t getAbsoluteHumidity(float temperature, float humidity) {
  // Formula to calculate absolute humidity for SGP30 calibration
  const float absHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature));
  return static_cast<uint32_t>(1000.0f * absHumidity);
}
