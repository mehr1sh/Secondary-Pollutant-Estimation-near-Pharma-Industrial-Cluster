/*
* TEAM VENKY CHICKEN - PM SENSOR SYSTEM (ESP32 #1)
*
* This ESP32 handles PM2.5 and PM10 measurements using SDS011 sensor
* Sends data to ThingsBoard cloud platform via MQTT
*
* Hardware Set:
* 1. SDS011 PM Sensor (UART)
* 2. SD Card Module (SPI)
* 3. WiFi for ThingsBoard connectivity
*
* Device ID: PM_SENSOR_01
*/

// --- Include Libraries ---
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <time.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// --- WiFi and ThingsBoard Configuration ---
const char* ssid = "Niviboo";           // Replace with your WiFi SSID
const char* password = "Niviboo123";   // Replace with your WiFi password

const char* tb_server = "demo.thingsboard.io";      // ThingsBoard server
const int tb_port = 1883;                      // MQTT port
const char* tb_token = "s8q7hSQ5xyyqdUNVtHbV";    // Replace with your PM sensor device token
const char* device_id = "ESP32Device";           // Device identifier

// --- Hardware Definitions & Objects ---
#define SD_CS_PIN 5
HardwareSerial sds(2);

// --- WiFi and MQTT Objects ---
WiFiClient wifiClient;
PubSubClient client(wifiClient);

// --- Sensor Globals ---
float globalPM25 = 0.0;
float globalPM10 = 0.0;
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
bool checkSDS();
void printAllData();
void logDataToSD();
void connectToWiFi();
void connectToThingsBoard();
void sendTelemetryData();
void checkThingsBoardConnection();

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("PM Sensor System Initializing (ESP32 #1)...");

  // --- TIMEZONE SETUP ---
  setenv("TZ", "IST-5:30", 1);
  tzset();

  // --- 1. Initialize HardwareSerial for SDS011 ---
  sds.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("HardwareSerial for SDS011 initialized (RX=16, TX=17).");

  // --- 2. Initialize Software Clock ---
  Serial.println("Setting software clock from compile time...");
  const char* compileTimeStr = __DATE__ " " __TIME__;
  struct tm tm;
  
  if (strptime(compileTimeStr, "%b %d %Y %H:%M:%S", &tm) != NULL) {
    startTimeUnix = mktime(&tm);
    startMillis = millis();
    Serial.print("Software clock start time set to: ");
    Serial.println(compileTimeStr);
  } else {
    Serial.println("ERROR: Failed to parse compile time!");
  }

  // --- 3. Initialize SD Card (SPI) ---
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("ERROR: SD Card initialization failed!");
  } else {
    Serial.println("SD Card Initialized.");
    File dataFile = SD.open("/pm_datalog.csv", FILE_APPEND);
    if (dataFile) {
      if (dataFile.size() == 0) {
        Serial.println("PM data file is empty. Writing new CSV header...");
        dataFile.println("Timestamp,PM2.5,PM10,Device_ID");
      }
      dataFile.close();
    }
  }

  // --- 4. Connect to WiFi ---
  connectToWiFi();

  // --- 5. Setup ThingsBoard MQTT ---
  client.setServer(tb_server, tb_port);
  connectToThingsBoard();

  Serial.println("\n--- PM Sensor Setup Complete. Starting Main Loop ---");
}

void loop() {
  // Task 1: Poll the SDS011 UART Buffer
  checkSDS();

  // Task 2: Check ThingsBoard Connection
  checkThingsBoardConnection();

  // Task 3: Run all blocking reads and logging on a 5-second timer
  unsigned long currentMillis = millis();
  if (currentMillis - prevLoopMillis >= loopInterval) {
    prevLoopMillis = currentMillis;

    // --- Calculate CURRENT Ticking Time ---
    unsigned long elapsedSeconds = (millis() - startMillis) / 1000;
    time_t currentTimeUnix = startTimeUnix + elapsedSeconds;
    struct tm* timeinfo = localtime(&currentTimeUnix);
    strftime(timeBuffer, 30, "%m/%d/%Y %H:%M:%S", timeinfo);

    // --- Print All Data ---
    printAllData();

    // --- Log to SD Card ---
    logDataToSD();
  }

  // Task 4: Send data to ThingsBoard every 10 seconds
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
  
  if (client.connect("ESP32Device" , tb_token, NULL)) {
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
  StaticJsonDocument<300> telemetry;
  
  telemetry["timestamp"] = String(timeBuffer);
  telemetry["device_id"] = device_id;
  telemetry["pm25"] = globalPM25;
  telemetry["pm10"] = globalPM10;
  
  // Calculate Air Quality Index based on PM2.5 (simple classification)
  String airQuality = "Good";
  if (globalPM25 > 35.4) airQuality = "Unhealthy";
  else if (globalPM25 > 12.0) airQuality = "Moderate";
  telemetry["air_quality_pm"] = airQuality;

  char payload[300];
  serializeJson(telemetry, payload);

  // Publish to ThingsBoard
  if (client.publish("v1/devices/me/telemetry", payload)) {
    Serial.println("PM Telemetry sent to ThingsBoard:");
    Serial.println(payload);
  } else {
    Serial.println("Failed to send PM telemetry to ThingsBoard");
  }
}

/**
 * @brief SDS011 Parser - Reads PM2.5 and PM10 values
 */
bool checkSDS() {
  byte buffer[10];
  while (sds.available() > 0) {
    if (sds.peek() != 0xAA) {
      sds.read();
      continue;
    }
    if (sds.available() < 10) {
      return false;
    }
    sds.readBytes(buffer, 10);
    if (buffer[0] == 0xAA && buffer[1] == 0xC0 && buffer[9] == 0xAB) {
      globalPM25 = ((buffer[3] * 256) + buffer[2]) / 10.0;
      globalPM10 = ((buffer[5] * 256) + buffer[4]) / 10.0;
      return true;
    }
  }
  return false;
}

/**
 * @brief Prints all PM sensor data
 */
void printAllData() {
  Serial.println("=== PM SENSOR DATA (ESP32 #1) ===");
  Serial.print("Timestamp: "); Serial.println(timeBuffer);
  Serial.println("----------------------------------");
  Serial.print("PM 2.5: "); Serial.print(globalPM25); Serial.println(" ug/m3");
  Serial.print("PM 10: "); Serial.print(globalPM10); Serial.println(" ug/m3");
  Serial.print("WiFi Status: "); Serial.println(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
  Serial.print("ThingsBoard Status: "); Serial.println(thingsBoardConnected ? "Connected" : "Disconnected");
  Serial.println("==================================\n");
}

/**
 * @brief Logs PM data to SD Card
 */
void logDataToSD() {
  String dataString = "";
  dataString += String(timeBuffer);
  dataString += ",";
  dataString += String(globalPM25);
  dataString += ",";
  dataString += String(globalPM10);
  dataString += ",";
  dataString += device_id ;

  File dataFile = SD.open("/pm_datalog.csv", FILE_APPEND);

  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    Serial.println("PM data logged to SD card.");
  } else {
    Serial.println("ERROR: Failed to open pm_datalog.csv on SD card.");
  }
}