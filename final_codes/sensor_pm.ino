/*
* TEAM VENKY CHICKEN - PM SENSOR SYSTEM (ESP32 #1)
*
* This ESP32 handles PM2.5 and PM10 measurements using SDS011 sensor
* Sends data to ThingsBoard cloud platform via MQTT
*
* Hardware Set:
* 1. SDS011 PM Sensor (UART) - Laser scattering particulate matter sensor
* 2. SD Card Module (SPI) - Local data storage and backup
* 3. WiFi for ThingsBoard connectivity - Cloud platform integration
*
* Device ID: PM_SENSOR_01 - Unique identifier for this particulate matter monitoring unit
*/

// --- Include Libraries ---
#include <Arduino.h>        // Core Arduino framework providing basic functions and data types
#include <SPI.h>            // Serial Peripheral Interface library for SD card communication
#include <SD.h>             // SD card file system library for reading/writing CSV data files
#include <time.h>           // Time manipulation functions for software clock and timezone handling
#include <WiFi.h>           // ESP32 WiFi connectivity library for wireless network communication
#include <PubSubClient.h>   // MQTT client library for ThingsBoard cloud platform communication
#include <ArduinoJson.h>    // JSON library for constructing structured telemetry payloads

// --- WiFi and ThingsBoard Configuration ---
const char* ssid = "Niviboo";                     // WiFi network name - replace with your network SSID
const char* password = "Niviboo123";              // WiFi network password - replace with your network password

const char* tb_server = "demo.thingsboard.io";    // ThingsBoard demo server URL for IoT cloud platform
const int tb_port = 1883;                         // Standard MQTT port for unencrypted communication
const char* tb_token = "s8q7hSQ5xyyqdUNVtHbV";   // Device authentication token from ThingsBoard - unique per device
const char* device_id = "ESP32Device";            // Human-readable device identifier for logging and dashboard

// --- Hardware Definitions & Objects ---
#define SD_CS_PIN 5         // Chip Select pin for SD card module SPI communication (GPIO 5)
HardwareSerial sds(2);      // Create second hardware serial port object for SDS011 sensor communication

// --- WiFi and MQTT Objects ---
WiFiClient wifiClient;              // TCP client object for WiFi network communication
PubSubClient client(wifiClient);    // MQTT client object using WiFi connection for ThingsBoard

// --- Sensor Global Variables ---
// Store current particulate matter readings accessible throughout the program
float globalPM25 = 0.0;     // PM2.5 concentration in micrograms per cubic meter (μg/m³)
float globalPM10 = 0.0;     // PM10 concentration in micrograms per cubic meter (μg/m³)
char timeBuffer[30];        // Character array to store formatted timestamp string

// --- Software Clock Global Variables ---
// Implement software-based real-time clock using compile time as reference point
time_t startTimeUnix;       // Unix timestamp when system started (seconds since January 1, 1970)
unsigned long startMillis;  // System uptime in milliseconds when software clock was initialized

// --- Non-Blocking Timer Variables ---
// Use millis() for timing to avoid blocking main program execution with delay()
unsigned long prevLoopMillis = 0;          // Last time sensor readings and logging were performed
const long loopInterval = 5000;            // Sensor reading and logging interval: 5 seconds (5000 milliseconds)
unsigned long prevThingsBoardMillis = 0;   // Last time data was transmitted to ThingsBoard cloud
const long thingsBoardInterval = 10000;    // ThingsBoard transmission interval: 10 seconds (10000 milliseconds)

// --- ThingsBoard Connection Status ---
bool thingsBoardConnected = false;  // Boolean flag to track MQTT connection status to ThingsBoard platform

// --- FORWARD DECLARATIONS ---
// Declare functions before main code to enable calling them from anywhere in the program
bool checkSDS();                    // Function to parse SDS011 sensor data from UART buffer
void printAllData();                // Function to display all sensor readings on serial monitor for debugging
void logDataToSD();                 // Function to append sensor data to CSV file on SD card
void connectToWiFi();               // Function to establish WiFi connection with network credentials
void connectToThingsBoard();        // Function to establish MQTT connection to ThingsBoard cloud platform
void sendTelemetryData();           // Function to format sensor data as JSON and send to cloud
void checkThingsBoardConnection();  // Function to monitor and maintain MQTT connection health

void setup() {
  Serial.begin(115200);             // Initialize serial communication at 115200 baud rate for debugging output
  while (!Serial);                  // Wait for serial port to be available (important for boards with native USB)
  Serial.println("PM Sensor System Initializing (ESP32 #1)..."); // Print system startup message

  // --- TIMEZONE SETUP ---
  setenv("TZ", "IST-5:30", 1);     // Set timezone environment variable to Indian Standard Time (UTC+5:30)
  tzset();                          // Apply timezone settings to all subsequent time functions

  // --- 1. Initialize HardwareSerial for SDS011 ---
  sds.begin(9600, SERIAL_8N1, 16, 17); // Configure UART: 9600 baud, 8 data bits, no parity, 1 stop bit, RX=GPIO16, TX=GPIO17
  Serial.println("HardwareSerial for SDS011 initialized (RX=16, TX=17)."); // Confirm UART initialization

  // --- 2. Initialize Software Clock ---
  Serial.println("Setting software clock from compile time..."); // Use compilation timestamp as time reference
  const char* compileTimeStr = __DATE__ " " __TIME__; // Get compile date and time as single string
  struct tm tm;                     // Create time structure for parsing date/time components
  
  if (strptime(compileTimeStr, "%b %d %Y %H:%M:%S", &tm) != NULL) { // Parse compile time string into time structure
    startTimeUnix = mktime(&tm);    // Convert time structure to Unix timestamp
    startMillis = millis();         // Record current system uptime in milliseconds
    Serial.print("Software clock start time set to: ");
    Serial.println(compileTimeStr); // Display the reference time used for software clock
  } else {
    Serial.println("ERROR: Failed to parse compile time!"); // Handle parsing failure
  }

  // --- 3. Initialize SD Card (SPI) ---
  if (!SD.begin(SD_CS_PIN)) {       // Attempt to initialize SD card with specified chip select pin
    Serial.println("ERROR: SD Card initialization failed!"); // Handle SD card initialization failure
  } else {
    Serial.println("SD Card Initialized."); // Confirm successful SD card initialization
    File dataFile = SD.open("/pm_datalog.csv", FILE_APPEND); // Open CSV file in append mode (create if doesn't exist)
    if (dataFile) {                 // Check if file was successfully opened
      if (dataFile.size() == 0) {   // Check if file is empty (newly created)
        Serial.println("PM data file is empty. Writing new CSV header...");
        dataFile.println("Timestamp,PM2.5,PM10,Device_ID"); // Write CSV header row with column names
      }
      dataFile.close();             // Close file to ensure data is written to SD card
    }
  }

  // --- 4. Connect to WiFi ---
  connectToWiFi();                  // Establish WiFi connection using configured network credentials

  // --- 5. Setup ThingsBoard MQTT ---
  client.setServer(tb_server, tb_port); // Configure MQTT client with ThingsBoard server address and port
  connectToThingsBoard();           // Establish MQTT connection to ThingsBoard cloud platform

  Serial.println("\n--- PM Sensor Setup Complete. Starting Main Loop ---"); // Confirm initialization complete
}

void loop() {
  // Task 1: Poll the SDS011 UART Buffer
  checkSDS();                       // Continuously check for new data packets from SDS011 sensor

  // Task 2: Check ThingsBoard Connection
  checkThingsBoardConnection();     // Monitor and maintain MQTT connection to cloud platform

  // Task 3: Run all blocking reads and logging on a 5-second timer
  unsigned long currentMillis = millis(); // Get current system uptime for non-blocking timing
  if (currentMillis - prevLoopMillis >= loopInterval) { // Check if 5 seconds have elapsed since last execution
    prevLoopMillis = currentMillis; // Update timer reference for next interval

    // --- Calculate CURRENT Ticking Time ---
    unsigned long elapsedSeconds = (millis() - startMillis) / 1000; // Calculate seconds since software clock initialization
    time_t currentTimeUnix = startTimeUnix + elapsedSeconds; // Add elapsed time to initial Unix timestamp
    struct tm* timeinfo = localtime(&currentTimeUnix); // Convert Unix time to local time structure
    strftime(timeBuffer, 30, "%m/%d/%Y %H:%M:%S", timeinfo); // Format time as human-readable string

    // --- Print All Data ---
    printAllData();                 // Display current sensor readings and system status on serial monitor

    // --- Log to SD Card ---
    logDataToSD();                  // Append current readings to CSV file on SD card for backup
  }

  // Task 4: Send data to ThingsBoard every 10 seconds
  if (currentMillis - prevThingsBoardMillis >= thingsBoardInterval && thingsBoardConnected) { // Check transmission timing and connection
    prevThingsBoardMillis = currentMillis; // Update timer reference for next transmission
    sendTelemetryData();            // Format and send sensor data to ThingsBoard cloud platform
  }
}

/**
 * @brief Connect to WiFi network using configured credentials
 * Continuously attempts connection until successful
 */
void connectToWiFi() {
  Serial.print("Connecting to WiFi"); // Display connection attempt message
  WiFi.begin(ssid, password);        // Start WiFi connection with network credentials
  
  while (WiFi.status() != WL_CONNECTED) { // Continue until connection established
    delay(500);                       // Wait 500 milliseconds between connection attempts
    Serial.print(".");               // Print dot to show connection progress
  }
  
  Serial.println();                   // New line after connection dots
  Serial.println("WiFi connected!");  // Confirm successful WiFi connection
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());    // Display assigned IP address for network identification
}

/**
 * @brief Connect to ThingsBoard MQTT broker using device credentials
 * Sets connection status flag based on result
 */
void connectToThingsBoard() {
  Serial.print("Connecting to ThingsBoard..."); // Display connection attempt message
  
  if (client.connect("ESP32Device" , tb_token, NULL)) { // Attempt MQTT connection with device ID and authentication token
    Serial.println("Connected to ThingsBoard!");        // Confirm successful connection
    thingsBoardConnected = true;      // Set connection status flag to true
  } else {
    Serial.print("Failed to connect to ThingsBoard, rc="); // Display connection failure
    Serial.println(client.state());  // Display MQTT client state code for troubleshooting
    thingsBoardConnected = false;     // Set connection status flag to false
  }
}

/**
 * @brief Check and maintain ThingsBoard MQTT connection
 * Automatically attempts reconnection if connection is lost
 */
void checkThingsBoardConnection() {
  if (!client.connected() && WiFi.status() == WL_CONNECTED) { // Check if MQTT disconnected but WiFi still connected
    thingsBoardConnected = false;     // Update connection status flag
    Serial.println("ThingsBoard connection lost. Attempting to reconnect..."); // Display reconnection message
    connectToThingsBoard();           // Attempt to reestablish MQTT connection
  }
  client.loop();                      // Process incoming MQTT messages and maintain connection heartbeat
}

/**
 * @brief Send telemetry data to ThingsBoard cloud platform
 * Formats sensor data as JSON and publishes via MQTT
 */
void sendTelemetryData() {
  if (!thingsBoardConnected) {        // Check if connected to ThingsBoard before attempting to send
    Serial.println("Not connected to ThingsBoard. Cannot send data.");
    return;                           // Exit function early if not connected
  }

  // Create JSON payload with sensor data
  StaticJsonDocument<300> telemetry; // Allocate JSON document with 300 bytes capacity
  
  telemetry["timestamp"] = String(timeBuffer);    // Add formatted timestamp to JSON
  telemetry["device_id"] = device_id;             // Add device identifier to JSON
  telemetry["pm25"] = globalPM25;                 // Add PM2.5 concentration reading to JSON
  telemetry["pm10"] = globalPM10;                 // Add PM10 concentration reading to JSON
  
  // Calculate Air Quality Index based on PM2.5 (simple classification)
  String airQuality = "Good";         // Default air quality classification
  if (globalPM25 > 35.4) airQuality = "Unhealthy";      // High pollution level (>35.4 μg/m³)
  else if (globalPM25 > 12.0) airQuality = "Moderate";  // Moderate pollution level (12.1-35.4 μg/m³)
  telemetry["air_quality_pm"] = airQuality;             // Add air quality classification to JSON

  char payload[300];                  // Character array to hold serialized JSON string
  serializeJson(telemetry, payload);  // Convert JSON document to string format

  // Publish to ThingsBoard using MQTT
  if (client.publish("v1/devices/me/telemetry", payload)) { // Send JSON payload to ThingsBoard telemetry API
    Serial.println("PM Telemetry sent to ThingsBoard:");    // Confirm successful transmission
    Serial.println(payload);          // Display sent payload for verification
  } else {
    Serial.println("Failed to send PM telemetry to ThingsBoard"); // Handle transmission failure
  }
}

/**
 * @brief SDS011 Parser - Reads PM2.5 and PM10 values from UART buffer
 * Parses 10-byte data packets according to SDS011 protocol
 * @return true if valid data packet was received and parsed
 */
bool checkSDS() {
  byte buffer[10];                    // Buffer to store 10-byte SDS011 data packet
  while (sds.available() > 0) {       // Continue while data is available in UART buffer
    if (sds.peek() != 0xAA) {         // Check if next byte is packet start marker (0xAA)
      sds.read();                     // Discard byte if not start marker
      continue;                       // Continue searching for start marker
    }
    if (sds.available() < 10) {       // Check if complete 10-byte packet is available
      return false;                   // Return false if incomplete packet
    }
    sds.readBytes(buffer, 10);        // Read complete 10-byte packet into buffer
    if (buffer[0] == 0xAA && buffer[1] == 0xC0 && buffer[9] == 0xAB) { // Validate packet format
      // Extract PM2.5 concentration: combine low byte (buffer[2]) and high byte (buffer[3])
      globalPM25 = ((buffer[3] * 256) + buffer[2]) / 10.0; // Convert to μg/m³ (divide by 10)
      // Extract PM10 concentration: combine low byte (buffer[4]) and high byte (buffer[5])
      globalPM10 = ((buffer[5] * 256) + buffer[4]) / 10.0; // Convert to μg/m³ (divide by 10)
      return true;                    // Return true indicating successful data extraction
    }
  }
  return false;                       // Return false if no valid packet found
}

/**
 * @brief Prints all PM sensor data to serial monitor
 * Displays formatted sensor readings and system status for debugging
 */
void printAllData() {
  Serial.println("=== PM SENSOR DATA (ESP32 #1) ===");    // Header for data display section
  Serial.print("Timestamp: "); Serial.println(timeBuffer); // Display current formatted timestamp
  Serial.println("----------------------------------");      // Visual separator line
  Serial.print("PM 2.5: "); Serial.print(globalPM25); Serial.println(" ug/m3");    // Display PM2.5 concentration with units
  Serial.print("PM 10: "); Serial.print(globalPM10); Serial.println(" ug/m3");     // Display PM10 concentration with units
  Serial.print("WiFi Status: "); Serial.println(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected"); // Display WiFi connection status
  Serial.print("ThingsBoard Status: "); Serial.println(thingsBoardConnected ? "Connected" : "Disconnected");   // Display MQTT connection status
  Serial.println("==================================\n");    // Footer and spacing for next display
}

/**
 * @brief Logs PM data to SD Card as CSV format
 * Appends current readings to pm_datalog.csv file with timestamp
 */
void logDataToSD() {
  String dataString = "";             // Initialize empty string for CSV row construction
  dataString += String(timeBuffer);  // Add formatted timestamp as first column
  dataString += ",";                  // Add CSV delimiter (comma)
  dataString += String(globalPM25);  // Add PM2.5 concentration as second column
  dataString += ",";                  // Add CSV delimiter (comma)
  dataString += String(globalPM10);  // Add PM10 concentration as third column
  dataString += ",";                  // Add CSV delimiter (comma)
  dataString += device_id ;          // Add device identifier as fourth column (no trailing comma)

  File dataFile = SD.open("/pm_datalog.csv", FILE_APPEND); // Open CSV file in append mode

  if (dataFile) {                     // Check if file was successfully opened
    dataFile.println(dataString);    // Write complete CSV row to file with newline
    dataFile.close();                 // Close file to ensure data is written to SD card
    Serial.println("PM data logged to SD card."); // Confirm successful data logging
  } else {
    Serial.println("ERROR: Failed to open pm_datalog.csv on SD card."); // Handle file operation failure
  }
}
