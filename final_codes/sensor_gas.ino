// --- Include Libraries ---
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "Adafruit_SGP30.h"
#include "Adafruit_AHTX0.h"
#include <time.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>


// --- WiFi and ThingsBoard Configuration ---
const char* ssid = "Niviboo";// Replace with your WiFi SSID
const char* password = "Niviboo123";// Replace with your WiFi password


const char* tb_server = "demo.thingsboard.io";// ThingsBoard server
const int tb_port = 1883;// MQTT port
const char* tb_token = "s8q7hSQ5xyyqdUNVtHbV";// Replace with your gas sensor device token
const char* device_id = "ESP32Device"; // Device identifier


const float MICS_VCC = 3.3;
const int ADC_MAX = 4095;
const float MQ_VCC = 3.3;
const float MQ_RLOAD_KOHMS = 10.0;

// Calibration constants for MQ-135 (example, change accordingly)
const float MQ135_R0 = 20.0; // baseline resistance in clean air in kOhms, calibrate for your module
const float MQ135_A = 110.47;
const float MQ135_B = -2.862;

// Calibration constants for MICS-2714 (example, change accordingly)
const float MICS_A = 100.0;  // Example coefficient (calibrate!!)
const float MICS_B = 1.5;    // Example exponent (calibrate!!)


// --- Hardware Definitions & Objects ---
Adafruit_SGP30 sgp;
Adafruit_AHTX0 aht10;
#define SD_CS_PIN 5
#define MQ135_PIN 35
#define MICS2714_PIN 34


// --- WiFi and MQTT Objects ---
WiFiClient wifiClient;
PubSubClient client(wifiClient);


// --- Sensor Globals ---
uint16_t tvoc = 0, eCO2_sgp = 0;
int rawMQ135 = 0;
float resistance_mq135 = 0.0;
float gas_mq135_ppm = 0.0;
int rawMICS2714 = 0;
float gas_mics_ppm = 0.0;
float temperature_aht10 = 0.0;
float humidity_aht10 = 0.0;
char timeBuffer[30];


// --- Software Clock Globals ---
time_t startTimeUnix;
unsigned long startMillis;


// --- Non-Blocking Timers ---
unsigned long prevLoopMillis = 0;
const long loopInterval = 5000; // Log every 5 seconds
unsigned long prevThingsBoardMillis = 0;
const long thingsBoardInterval = 10000; // Send to ThingsBoard every 10 seconds


// --- ThingsBoard Connection Status ---
bool thingsBoardConnected = false;


// --- FORWARD DECLARATIONS ---
void printAllData();
void logDataToSD();
void connectToWiFi();
void connectToThingsBoard();
void sendTelemetryData();
void checkThingsBoardConnection();


void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Gas Sensor System Initializing (ESP32 #2)...");

  // --- TIMEZONE SETUP ---
  setenv("TZ", "IST-5:30", 1);
  tzset();

  // --- 1. Initialize I2C Bus and SGP30 ---
  Wire.begin(21, 22); // SDA=21, SCL=22 (standard ESP32 I2C pins)
  if (!sgp.begin()) {
    Serial.println("ERROR: Failed to find SGP30 sensor!");
  } else {
    Serial.println("SGP30 Found.");
  }

  // Initialize AHT10 on alternate I2C pins
  Wire1.begin(4, 5); // SDA=4, SCL=5 for AHT10
  if (!aht10.begin(&Wire1)) {
    Serial.println("ERROR: Failed to find AHT10 sensor!");
  } else {
    Serial.println("AHT10 Found.");
  }

  // --- 2. Initialize Software Clock ---
  Serial.println("Setting software clock from compile time...");
  const char* compileTimeStr = _DATE_ " " _TIME_;
  struct tm tm;

  if (strptime(compileTimeStr, "%b %d %Y %H:%M:%S", &tm) != NULL) {
    startTimeUnix = mktime(&tm);
    startMillis = millis();
    Serial.print("Software clock start time set to: ");
    Serial.println(compileTimeStr);
  } else {
    Serial.println("ERROR: Failed to parse compile time!");
  }

  // --- 3. Initialize Analog Pins ---
  pinMode(MQ135_PIN, INPUT);
  pinMode(MICS2714_PIN, INPUT);
  Serial.println("MQ-135 pin (GPIO 35) and MICS-2714 pin (GPIO 34) set to INPUT.");

  // --- 4. Initialize SD Card (SPI) ---
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("ERROR: SD Card initialization failed!");
  } else {
    Serial.println("SD Card Initialized.");
    File dataFile = SD.open("/gas_datalog.csv", FILE_APPEND);
    if (dataFile) {
      if (dataFile.size() == 0) {
        Serial.println("Gas data file is empty. Writing new CSV header...");
        dataFile.println("Timestamp,TVOC,eCO2,MQ135_Raw,MQ135_Resistance,MQ135_PPM,MICS2714_Raw,MICS2714_PPM,Temperature,Humidity,Device_ID");
      }
      dataFile.close();
    }
  }

  // --- 5. Connect to WiFi ---
  connectToWiFi();

  // --- 6. Setup ThingsBoard MQTT ---
  client.setServer(tb_server, tb_port);
  connectToThingsBoard();

  Serial.println("\n--- Gas Sensor Setup Complete. Starting Main Loop ---");
}


void loop() {
  // Task 1: Check ThingsBoard Connection
  checkThingsBoardConnection();

  // Task 2: Run all blocking reads and logging on a 5-second timer
  unsigned long currentMillis = millis();
  if (currentMillis - prevLoopMillis >= loopInterval) {
    prevLoopMillis = currentMillis;

    // --- A. Get I2C Sensor Data (SGP30) ---
    if (!sgp.IAQmeasure()) {
      Serial.println("SGP30 measurement failed!");
    } else {
      tvoc = sgp.TVOC;
      eCO2_sgp = sgp.eCO2;
    }

    // --- A2. Get AHT10 Data ---
    sensors_event_t humidity, temp;
    if (aht10.getEvent(&humidity, &temp)) {
      temperature_aht10 = temp.temperature;
      humidity_aht10 = humidity.relative_humidity;
    } else {
      Serial.println("AHT10 reading failed!");
      temperature_aht10 = 0.0;
      humidity_aht10 = 0.0;
    }

    // --- B. Calculate CURRENT Ticking Time ---
    unsigned long elapsedSeconds = (millis() - startMillis) / 1000;
    time_t currentTimeUnix = startTimeUnix + elapsedSeconds;
    struct tm* timeinfo = localtime(&currentTimeUnix);
    strftime(timeBuffer, 30, "%m/%d/%Y %H:%M:%S", timeinfo);

    // --- C. Get Analog Sensor Data (MQ-135 and MICS-2714) ---
    rawMQ135 = analogRead(MQ135_PIN);
    rawMICS2714 = analogRead(MICS2714_PIN);
    float vout_mq135 = rawMQ135 * (MQ_VCC / ADC_MAX);

    if (vout_mq135 == 0) {
      resistance_mq135 = 0;
      gas_mq135_ppm = 0;
    } else {
      resistance_mq135 = MQ_RLOAD_KOHMS * (MQ_VCC / vout_mq135 - 1.0);
      gas_mq135_ppm = MQ135_A * pow((resistance_mq135 / MQ135_R0), MQ135_B);
    }

    float vout_mics = rawMICS2714 * (MICS_VCC / ADC_MAX);
    gas_mics_ppm = MICS_A * pow(vout_mics, MICS_B);

    // --- D. Print All Data ---
    printAllData();

    // --- E. Log to SD Card ---
    logDataToSD();
  }

  // Task 3: Send data to ThingsBoard every 10 seconds
  if (currentMillis - prevThingsBoardMillis >= thingsBoardInterval && thingsBoardConnected) {
    prevThingsBoardMillis = currentMillis;
    sendTelemetryData();
  }
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("Failed to connect to WiFi!");
  }
}

void connectToThingsBoard() {
  Serial.print("Connecting to ThingsBoard...");

  if (client.connect(device_id, tb_token, NULL)) {
    Serial.println("Connected to ThingsBoard!");
    thingsBoardConnected = true;
  } else {
    Serial.print("Failed to connect to ThingsBoard, rc=");
    Serial.println(client.state());
    thingsBoardConnected = false;
  }
}

void checkThingsBoardConnection() {
  if (!client.connected() && WiFi.status() == WL_CONNECTED) {
    thingsBoardConnected = false;
    Serial.println("ThingsBoard connection lost. Attempting to reconnect...");
    connectToThingsBoard();
  }
  client.loop(); // Process MQTT messages
}

void sendTelemetryData() {
  if (!thingsBoardConnected) {
    Serial.println("Not connected to ThingsBoard. Cannot send data.");
    return;
  }

  StaticJsonDocument<600> telemetry;

  telemetry["timestamp"] = String(timeBuffer);
  telemetry["device_id"] = device_id;
  telemetry["tvoc"] = tvoc;
  telemetry["eco2"] = eCO2_sgp;
  telemetry["mq135_raw"] = rawMQ135;
  telemetry["mq135_resistance"] = resistance_mq135;
  telemetry["mq135_ppm"] = gas_mq135_ppm;
  telemetry["mics2714_raw"] = rawMICS2714;
  telemetry["mics2714_ppm"] = gas_mics_ppm;
  telemetry["temperature"] = temperature_aht10;
  telemetry["humidity"] = humidity_aht10;

  String gasQuality = "Good";
  if (tvoc > 2200) gasQuality = "Unhealthy";
  else if (tvoc > 660) gasQuality = "Moderate";
  telemetry["gas_quality"] = gasQuality;

  String co2Level = "Normal";
  if (eCO2_sgp > 1000) co2Level = "Poor";
  else if (eCO2_sgp > 800) co2Level = "Moderate";
  telemetry["co2_level"] = co2Level;

  char payload[600];
  serializeJson(telemetry, payload);

  if (client.publish("v1/devices/me/telemetry", payload)) {
    Serial.println("Gas Telemetry sent to ThingsBoard:");
    Serial.println(payload);
  } else {
    Serial.println("Failed to send gas telemetry to ThingsBoard");
  }
}

void printAllData() {
  Serial.println("=== GAS SENSOR DATA (ESP32 #2) ===");
  Serial.print("Timestamp: "); Serial.println(timeBuffer);
  Serial.println("-----------------------------------");
  Serial.print("VOC Index (SGP30): "); Serial.print(tvoc); Serial.println(" ppb");
  Serial.print("eCO2 (SGP30): "); Serial.print(eCO2_sgp); Serial.println(" ppm");
  Serial.print("Air Quality (MQ-135): "); Serial.print(rawMQ135); Serial.println(" (RAW ADC)");
  Serial.print("MQ-135 Resistance (Rs): "); Serial.print(resistance_mq135); Serial.println(" kOhms");
  Serial.print("MQ-135 Gas Concentration: "); Serial.print(gas_mq135_ppm); Serial.println(" ppm");
  Serial.print("MICS-2714 (Raw ADC): "); Serial.print(rawMICS2714); Serial.print(", Approx. PPM: "); Serial.println(gas_mics_ppm);
  Serial.print("Temperature (AHT10): "); Serial.print(temperature_aht10); Serial.println(" °C");
  Serial.print("Humidity (AHT10): "); Serial.print(humidity_aht10); Serial.println(" %");
  Serial.print("WiFi Status: "); Serial.println(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
  Serial.print("ThingsBoard Status: "); Serial.println(thingsBoardConnected ? "Connected" : "Disconnected");
  Serial.println("===================================\n");
}

void logDataToSD() {
  String dataString = "";
  dataString += String(timeBuffer);
  dataString += ",";
  dataString += String(tvoc);
  dataString += ",";
  dataString += String(eCO2_sgp);
  dataString += ",";
  dataString += String(rawMQ135);
  dataString += ",";
  dataString += String(resistance_mq135);
  dataString += ",";
  dataString += String(gas_mq135_ppm);
  dataString += ",";
  dataString += String(rawMICS2714);
  dataString += ",";
  dataString += String(gas_mics_ppm);
  dataString += ",";
  dataString += String(temperature_aht10);
  dataString += ",";
  dataString += String(humidity_aht10);
  dataString += ",";
  dataString += device_id;

  File dataFile = SD.open("/gas_datalog.csv", FILE_APPEND);

  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    Serial.println("Gas data logged to SD card.");
  } else {
    Serial.println("ERROR: Failed to open gas_datalog.csv on SD card.");
  }
}