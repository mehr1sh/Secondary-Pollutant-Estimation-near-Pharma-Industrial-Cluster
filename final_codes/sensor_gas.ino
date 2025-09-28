/*
* TEAM VENKY CHICKEN - GAS SENSOR SYSTEM (ESP32 #2)
*
* This ESP32 handles gas sensor measurements using SGP30 and MQ-135
* Sends data to ThingsBoard cloud platform via MQTT
*
* Hardware Set:
* 1. SGP30 (I2C) - TVOC and eCO2
* 2. MQ-135 (Analog) - Air Quality
* 3. SD Card Module (SPI)
* 4. WiFi for ThingsBoard connectivity
*
* Device ID: GAS_SENSOR_01
*/


// --- Include Libraries ---
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "Adafruit_SGP30.h"
#include <time.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>


// --- WiFi and ThingsBoard Configuration ---
const char* ssid = "Niviboo";           // Replace with your WiFi SSID
const char* password = "Niviboo123";   // Replace with your WiFi password


const char* tb_server = "demo.thingsboard.io";      // ThingsBoard server
const int tb_port = 1883;                      // MQTT port
const char* tb_token = "s8q7hSQ5xyyqdUNVtHbV";    // Replace with your gas sensor device token
const char* device_id = "ESP32Device";           // Device identifier


// --- Hardware Definitions & Objects ---
Adafruit_SGP30 sgp;
#define SD_CS_PIN 5
#define MQ135_PIN 35


#define MQ_VCC 3.3
#define MQ_RLOAD_KOHMS 10.0


// --- WiFi and MQTT Objects ---
WiFiClient wifiClient;
PubSubClient client(wifiClient);


// --- Sensor Globals ---
uint16_t tvoc = 0, eCO2_sgp = 0;
int rawMQ135 = 0;
float resistance_mq135 = 0.0;
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


  // --- 3. Initialize Analog Pin ---
  pinMode(MQ135_PIN, INPUT);
  Serial.println("MQ-135 pin (GPIO 35) set to INPUT.");


  // --- 4. Initialize SD Card (SPI) ---
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("ERROR: SD Card initialization failed!");
  } else {
    Serial.println("SD Card Initialized.");
    File dataFile = SD.open("/gas_datalog.csv", FILE_APPEND);
    if (dataFile) {
      if (dataFile.size() == 0) {
        Serial.println("Gas data file is empty. Writing new CSV header...");
        dataFile.println("Timestamp,TVOC,eCO2,MQ135_Raw,MQ135_Resistance,Device_ID");
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


    // --- B. Calculate CURRENT Ticking Time ---
    unsigned long elapsedSeconds = (millis() - startMillis) / 1000;
    time_t currentTimeUnix = startTimeUnix + elapsedSeconds;
    struct tm* timeinfo = localtime(&currentTimeUnix);
    strftime(timeBuffer, 30, "%m/%d/%Y %H:%M:%S", timeinfo);


    // --- C. Get Analog Sensor Data (MQ-135) ---
    rawMQ135 = analogRead(MQ135_PIN);
    float vout = rawMQ135 * (MQ_VCC / 4095.0);
    
    if (vout == 0) {
      resistance_mq135 = 0;
    } else {
      resistance_mq135 = MQ_RLOAD_KOHMS * (MQ_VCC / vout - 1.0);
    }


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


/**
 * @brief Connect to WiFi network
 */
void connectToWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}


/**
 * @brief Connect to ThingsBoard MQTT broker
 */
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


/**
 * @brief Check and maintain ThingsBoard connection
 */
void checkThingsBoardConnection() {
  if (!client.connected() && WiFi.status() == WL_CONNECTED) {
    thingsBoardConnected = false;
    Serial.println("ThingsBoard connection lost. Attempting to reconnect...");
    connectToThingsBoard();
  }
  client.loop(); // Process MQTT messages
}


/**
 * @brief Send telemetry data to ThingsBoard
 */
void sendTelemetryData() {
  if (!thingsBoardConnected) {
    Serial.println("Not connected to ThingsBoard. Cannot send data.");
    return;
  }


  // Create JSON payload
  StaticJsonDocument<400> telemetry;
  
  telemetry["timestamp"] = String(timeBuffer);
  telemetry["device_id"] = device_id;
  telemetry["tvoc"] = tvoc;
  telemetry["eco2"] = eCO2_sgp;
  telemetry["mq135_raw"] = rawMQ135;
  telemetry["mq135_resistance"] = resistance_mq135;
  
  // Calculate Gas Quality Index (simple classification based on TVOC)
  String gasQuality = "Good";
  if (tvoc > 2200) gasQuality = "Unhealthy";
  else if (tvoc > 660) gasQuality = "Moderate";
  telemetry["gas_quality"] = gasQuality;


  // CO2 level classification
  String co2Level = "Normal";
  if (eCO2_sgp > 1000) co2Level = "Poor";
  else if (eCO2_sgp > 800) co2Level = "Moderate";
  telemetry["co2_level"] = co2Level;


  char payload[400];
  serializeJson(telemetry, payload);


  // Publish to ThingsBoard
  if (client.publish("v1/devices/me/telemetry", payload)) {
    Serial.println("Gas Telemetry sent to ThingsBoard:");
    Serial.println(payload);
  } else {
    Serial.println("Failed to send gas telemetry to ThingsBoard");
  }
}


/**
 * @brief Prints all gas sensor data
 */
void printAllData() {
  Serial.println("=== GAS SENSOR DATA (ESP32 #2) ===");
  Serial.print("Timestamp: "); Serial.println(timeBuffer);
  Serial.println("-----------------------------------");
  Serial.print("VOC Index (SGP30): "); Serial.print(tvoc); Serial.println(" ppb");
  Serial.print("eCO2 (SGP30): "); Serial.print(eCO2_sgp); Serial.println(" ppm");
  Serial.print("Air Quality (MQ-135): "); Serial.print(rawMQ135); Serial.println(" (RAW ADC)");
  Serial.print("MQ-135 Resistance (Rs): "); Serial.print(resistance_mq135); Serial.println(" kOhms");
  Serial.print("WiFi Status: "); Serial.println(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
  Serial.print("ThingsBoard Status: "); Serial.println(thingsBoardConnected ? "Connected" : "Disconnected");
  Serial.println("===================================\n");
}


/**
 * @brief Logs gas sensor data to SD Card
 */
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