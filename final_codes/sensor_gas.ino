// Main sketch for ESP32 #2 - Gas sensor monitoring system
// Initializes I2C interfaces for digital sensors, software clock for timestamping, SD card logging for data persistence


// --- Include Libraries ---
#include <Arduino.h>            // Core Arduino framework functions and definitions
#include <Wire.h>               // I2C communication library for digital sensors (SGP30, AHT10)
#include <SPI.h>                // Serial Peripheral Interface library for SD card communication
#include <SD.h>                 // SD card file system operations (read, write, append)
#include "Adafruit_SGP30.h"     // High-level library for SGP30 TVOC/eCO2 sensor with I2C communication
#include "Adafruit_AHTX0.h"     // High-level library for AHT10 temperature/humidity sensor with I2C communication
#include <time.h>               // Time manipulation functions for software clock and timezone handling
#include <WiFi.h>               // ESP32 WiFi connectivity library for wireless communication
#include <PubSubClient.h>       // MQTT client library for ThingsBoard cloud platform communication
#include <ArduinoJson.h>        // JSON library for constructing telemetry payloads in standard format

// --- WiFi and ThingsBoard Configuration ---
const char* ssid = "realme";                      // Network SSID - replace with your WiFi network name
const char* password = "avaditya";                // Network password - replace with your WiFi password

const char* tb_server = "demo.thingsboard.io";   // ThingsBoard demo server URL for cloud data platform
const int tb_port = 1883;                        // Standard MQTT port for unencrypted communication
const char* tb_token = "ZnCXWNQanEGsKVJviiOy";  // Device authentication token from ThingsBoard - unique per device
const char* device_id = "ESP32Device";           // Human-readable device identifier for logging and identification

// --- Analog Sensor Hardware Constants ---
const float MICS_VCC = 3.3;        // Supply voltage for MiCS2714 sensor - ESP32 operates at 3.3V logic level
const int ADC_MAX = 4095;          // Maximum ADC value for ESP32 (12-bit ADC: 2^12 - 1 = 4095)
const float MQ_VCC = 3.3;          // Supply voltage for MQ135 sensor - matches ESP32 logic level
const float MQ_RLOAD_KOHMS = 10.0; // Load resistor value in kiloohms for MQ135 voltage divider circuit

// --- Calibration constants for MQ-135 gas sensor ---
// These constants convert sensor resistance to gas concentration using power law: PPM = A * (Rs/R0)^B
const float MQ135_R0 = 20.0;     // Baseline resistance in clean air (kΩ) - calibrate for specific module
const float MQ135_A = 110.47;    // Scaling coefficient from manufacturer datasheet/calibration
const float MQ135_B = -2.862;    // Power law exponent from manufacturer datasheet/calibration

// --- Calibration constants for MICS-2714 gas sensor ---
// These constants convert sensor voltage to gas concentration using power law: PPM = A * Vout^B
const float MICS_A = 100.0;      // Scaling coefficient - requires calibration with known gas concentrations
const float MICS_B = 1.5;        // Power law exponent - requires calibration with known gas concentrations

// --- Hardware Definitions & Objects ---
Adafruit_SGP30 sgp;              // Create SGP30 sensor object for TVOC/eCO2 measurements
Adafruit_AHTX0 aht10;            // Create AHT10 sensor object for temperature/humidity measurements
#define SD_CS_PIN 5              // Chip Select pin for SD card module SPI communication
#define MQ135_PIN 35             // Analog input pin for MQ135 gas sensor (ADC1_CH7)
#define MICS2714_PIN 34          // Analog input pin for MiCS2714 gas sensor (ADC1_CH6)

// --- WiFi and MQTT Objects ---
WiFiClient wifiClient;           // TCP client for WiFi network communication
PubSubClient client(wifiClient); // MQTT client using the WiFi connection for ThingsBoard communication

// --- Sensor Global Variables ---
// Store current sensor readings accessible throughout the program
uint16_t tvoc = 0, eCO2_sgp = 0;    // SGP30 readings: TVOC in ppb, eCO2 in ppm (16-bit unsigned integers)
int rawMQ135 = 0;                    // Raw ADC reading from MQ135 sensor (0-4095)
float resistance_mq135 = 0.0;       // Calculated sensor resistance in kiloohms
float gas_mq135_ppm = 0.0;          // Calculated gas concentration in parts per million
int rawMICS2714 = 0;                // Raw ADC reading from MiCS2714 sensor (0-4095)
float gas_mics_ppm = 0.0;           // Calculated gas concentration in parts per million
float temperature_aht10 = 0.0;      // Temperature reading from AHT10 in degrees Celsius
float humidity_aht10 = 0.0;         // Relative humidity reading from AHT10 in percentage
char timeBuffer[30];                // String buffer to store formatted timestamp

// --- Software Clock Global Variables ---
// Implement software-based real-time clock using compile time as reference
time_t startTimeUnix;            // Unix timestamp when system started (seconds since epoch)
unsigned long startMillis;      // System uptime in milliseconds when clock was initialized

// --- Non-Blocking Timer Variables ---
// Use millis() for timing to avoid blocking main program execution
unsigned long prevLoopMillis = 0;           // Last time sensor readings were taken
const long loopInterval = 5000;             // Sensor reading interval: 5 seconds (5000 milliseconds)
unsigned long prevThingsBoardMillis = 0;    // Last time data was sent to ThingsBoard
const long thingsBoardInterval = 10000;     // ThingsBoard transmission interval: 10 seconds
unsigned long prevBaselineMillis = 0;       // Last time SGP30 baseline was updated
const long baselineInterval = 3600000;      // Baseline update interval: 1 hour (3600000 milliseconds)

// --- ThingsBoard Connection Status ---
bool thingsBoardConnected = false;   // Flag to track MQTT connection status to ThingsBoard

// --- FORWARD DECLARATIONS ---
// Declare functions before main code to enable calling them from anywhere
void printAllData();                                          // Function to display all sensor readings on serial monitor
void logDataToSD();                                          // Function to append sensor data to CSV file on SD card
void connectToWiFi();                                        // Function to establish WiFi connection with retry logic
void connectToThingsBoard();                                 // Function to establish MQTT connection to ThingsBoard
void sendTelemetryData();                                    // Function to format and send sensor data as JSON to cloud
void checkThingsBoardConnection();                           // Function to monitor and maintain MQTT connection
uint32_t getAbsoluteHumidity(float temperature, float humidity); // Function to calculate absolute humidity for SGP30 compensation

void setup() {
  Serial.begin(115200);                    // Initialize serial communication at 115200 baud rate for debugging
  while (!Serial);                        // Wait for serial port to be available (important for native USB)
  Serial.println("Gas Sensor System Initializing (ESP32 #2)..."); // Print startup message

  // --- TIMEZONE SETUP ---
  setenv("TZ", "IST-5:30", 1);           // Set timezone environment variable to Indian Standard Time (UTC+5:30)
  tzset();                               // Apply timezone settings to system time functions

  // --- 1. Initialize I2C Bus and SGP30 ---
  Wire.begin(21, 22);                    // Initialize primary I2C bus: SDA=GPIO21, SCL=GPIO22 (ESP32 default pins)
  if (!sgp.begin()) {                    // Attempt to initialize SGP30 sensor on I2C bus
    Serial.println("ERROR: Failed to find SGP30 sensor!"); // Print error if sensor not detected
  } else {
    Serial.println("SGP30 Found.");     // Confirm successful sensor detection
    
    // Read SGP30 Serial ID and Feature Set
    Serial.print("SGP30 Serial ID: 0x"); // Display unique sensor serial number for identification
    Serial.print(sgp.serialnumber[0], HEX); // Print first part of serial number in hexadecimal
    Serial.print(sgp.serialnumber[1], HEX); // Print second part of serial number in hexadecimal
    Serial.println(sgp.serialnumber[2], HEX); // Print third part of serial number in hexadecimal
    
    // Perform soft reset
    Serial.println("Performing SGP30 soft reset..."); // Soft reset ensures clean sensor state
    // Note: Soft reset is handled internally by the library during begin()
    
    // Set initial baseline values to kickstart the sensor
    // These are typical baseline values - the sensor will adapt over time
    Serial.println("Setting initial SGP30 baseline values...");
    sgp.setIAQBaseline(0x8E68, 0x8F41); // Set known good baseline values to accelerate stabilization
    
    Serial.println("SGP30 baseline set. Starting warmup period...");
    Serial.println("Note: TVOC readings may take 12-24 hours to fully stabilize"); // Inform about stabilization time
    
    // Take some initial readings to establish baseline
    for (int i = 0; i < 15; i++) {      // Take 15 initial measurements for warmup
      if (sgp.IAQmeasure()) {           // Attempt to take measurement from SGP30
        Serial.print("Warmup reading "); Serial.print(i+1); 
        Serial.print("/15: TVOC = "); Serial.print(sgp.TVOC);    // Display TVOC reading in ppb
        Serial.print(" ppb, eCO2 = "); Serial.print(sgp.eCO2); Serial.println(" ppm"); // Display eCO2 reading in ppm
        
        // If we start getting non-zero TVOC readings, we can break early
        if (sgp.TVOC > 0) {            // Check if sensor is producing meaningful readings
          Serial.println("TVOC readings detected! Baseline is establishing.");
        }
      }
      delay(1000);                     // Wait 1 second between warmup measurements
    }
    Serial.println("SGP30 warmup complete!"); // Confirm completion of warmup sequence
  }

  // Initialize AHT10 on alternate I2C pins
  Wire1.begin(16, 17);                 // Initialize secondary I2C bus: SDA=GPIO16, SCL=GPIO17 (avoids pin conflicts)
  if (!aht10.begin(&Wire1)) {          // Attempt to initialize AHT10 on secondary I2C bus
    Serial.println("ERROR: Failed to find AHT10 sensor!"); // Print error if sensor not detected
  } else {
    Serial.println("AHT10 Found.");   // Confirm successful sensor detection
  }

  // --- 2. Initialize Software Clock ---
  Serial.println("Setting software clock from compile time..."); // Use compilation timestamp as time reference
  const char* compileTimeStr = __DATE__ " " __TIME__; // Get compile date and time as string
  struct tm tm;                        // Create time structure for parsing

  if (strptime(compileTimeStr, "%b %d %Y %H:%M:%S", &tm) != NULL) { // Parse compile time string
    startTimeUnix = mktime(&tm);       // Convert to Unix timestamp
    startMillis = millis();            // Record current system uptime
    Serial.print("Software clock start time set to: ");
    Serial.println(compileTimeStr);    // Confirm time setting
  } else {
    Serial.println("ERROR: Failed to parse compile time!"); // Handle parsing failure
  }

  // --- 3. Initialize Analog Pins ---
  pinMode(MQ135_PIN, INPUT);           // Configure GPIO35 as analog input for MQ135 sensor
  pinMode(MICS2714_PIN, INPUT);        // Configure GPIO34 as analog input for MiCS2714 sensor
  Serial.println("MQ-135 pin (GPIO 35) and MICS-2714 pin (GPIO 34) set to INPUT.");

  // --- 4. Initialize SD Card (SPI) ---
  if (!SD.begin(SD_CS_PIN)) {          // Attempt to initialize SD card with chip select pin 5
    Serial.println("ERROR: SD Card initialization failed!"); // Handle SD card failure
  } else {
    Serial.println("SD Card Initialized."); // Confirm SD card is ready
    File dataFile = SD.open("/gas_datalog.csv", FILE_APPEND); // Open CSV file in append mode
    if (dataFile) {                    // Check if file opened successfully
      if (dataFile.size() == 0) {      // Check if file is empty (new file)
        Serial.println("Gas data file is empty. Writing new CSV header...");
        // Write CSV header row with column names for data organization
        dataFile.println("Timestamp,TVOC,eCO2,MQ135_Raw,MQ135_Resistance,MQ135_PPM,MICS2714_Raw,MICS2714_PPM,Temperature,Humidity,Device_ID");
      }
      dataFile.close();               // Close file to ensure data is written
    }
  }

  // --- 5. Connect to WiFi ---
  connectToWiFi();                    // Establish WiFi connection using configured credentials

  // --- 6. Setup ThingsBoard MQTT ---
  client.setServer(tb_server, tb_port); // Configure MQTT client with server address and port
  client.setBufferSize(1024);         // Increase MQTT buffer size to handle larger JSON payloads
  client.setKeepAlive(60);            // Set MQTT keep-alive interval to 60 seconds
  connectToThingsBoard();             // Establish connection to ThingsBoard platform

  Serial.println("\n--- Gas Sensor Setup Complete. Starting Main Loop ---"); // Confirm initialization complete
}

void loop() {
  // Task 1: Check ThingsBoard Connection
  checkThingsBoardConnection();       // Monitor and maintain MQTT connection to cloud platform

  // Task 2: Run all blocking reads and logging on a 5-second timer
  unsigned long currentMillis = millis(); // Get current system uptime for timing comparisons
  if (currentMillis - prevLoopMillis >= loopInterval) { // Check if 5 seconds have elapsed
    prevLoopMillis = currentMillis;   // Update timer for next interval

    // --- A. Get I2C Sensor Data (SGP30) ---
    if (!sgp.IAQmeasure()) {          // Attempt to read TVOC and eCO2 from SGP30
      Serial.println("SGP30 measurement failed!"); // Handle measurement failure
    } else {
      tvoc = sgp.TVOC;                // Store TVOC reading in global variable
      eCO2_sgp = sgp.eCO2;           // Store eCO2 reading in global variable
    }

    // --- A2. Get AHT10 Data ---
    sensors_event_t humidity, temp;  // Create event structures for sensor data
    if (aht10.getEvent(&humidity, &temp)) { // Attempt to read temperature and humidity
      temperature_aht10 = temp.temperature;      // Store temperature in global variable
      humidity_aht10 = humidity.relative_humidity; // Store humidity in global variable
      // Check for valid readings (AHT10 should not return exactly 0.0 for both)
      if (temperature_aht10 == 0.0 && humidity_aht10 == 0.0) {
        Serial.println("Warning: AHT10 returned zero values - possible sensor issue");
      }
      
      // Set humidity compensation for SGP30 if we have valid readings
      if (temperature_aht10 > 0.0 && humidity_aht10 > 0.0) {
        uint32_t absolute_humidity = getAbsoluteHumidity(temperature_aht10, humidity_aht10); // Calculate absolute humidity
        sgp.setHumidity(absolute_humidity); // Apply humidity compensation to improve SGP30 accuracy
      }
    } else {
      Serial.println("ERROR: AHT10 reading failed! Check I2C connections (SDA=16, SCL=17)");
      temperature_aht10 = 0.0;        // Set default values on failure
      humidity_aht10 = 0.0;
    }

    // --- B. Calculate CURRENT Ticking Time ---
    unsigned long elapsedSeconds = (millis() - startMillis) / 1000; // Calculate seconds since startup
    time_t currentTimeUnix = startTimeUnix + elapsedSeconds; // Add elapsed time to start time
    struct tm* timeinfo = localtime(&currentTimeUnix); // Convert to local time structure
    strftime(timeBuffer, 30, "%m/%d/%Y %H:%M:%S", timeinfo); // Format time as readable string

    // --- C. Get Analog Sensor Data (MQ-135 and MICS-2714) ---
    rawMQ135 = analogRead(MQ135_PIN);     // Read raw ADC value from MQ135 sensor (0-4095)
    rawMICS2714 = analogRead(MICS2714_PIN); // Read raw ADC value from MiCS2714 sensor (0-4095)
    float vout_mq135 = rawMQ135 * (MQ_VCC / ADC_MAX); // Convert ADC reading to voltage

    if (vout_mq135 == 0) {               // Handle zero voltage case (sensor disconnected)
      resistance_mq135 = 0;              // Set resistance to zero
      gas_mq135_ppm = 0;                 // Set concentration to zero
    } else {
      // Calculate sensor resistance using voltage divider formula: Rs = RL * (Vin/Vout - 1)
      resistance_mq135 = MQ_RLOAD_KOHMS * (MQ_VCC / vout_mq135 - 1.0);
      // Calculate gas concentration using power law: PPM = A * (Rs/R0)^B
      gas_mq135_ppm = MQ135_A * pow((resistance_mq135 / MQ135_R0), MQ135_B);
    }

    float vout_mics = rawMICS2714 * (MICS_VCC / ADC_MAX); // Convert MiCS2714 ADC reading to voltage
    gas_mics_ppm = MICS_A * pow(vout_mics, MICS_B); // Calculate gas concentration using power law

    // --- D. Print All Data ---
    printAllData();                     // Display all sensor readings on serial monitor

    // --- E. Log to SD Card ---
    logDataToSD();                      // Append current readings to CSV file on SD card
  }

  // Task 3: Send data to ThingsBoard every 10 seconds
  if (currentMillis - prevThingsBoardMillis >= thingsBoardInterval && thingsBoardConnected) {
    prevThingsBoardMillis = currentMillis; // Update timer for next transmission
    sendTelemetryData();               // Format and send sensor data to cloud platform
  }
  
  // Task 4: Update SGP30 baseline every hour for long-term stability
  if (currentMillis - prevBaselineMillis >= baselineInterval) {
    prevBaselineMillis = currentMillis; // Update timer for next baseline check
    uint16_t eCO2_base, TVOC_base;     // Variables to store baseline values
    if (sgp.getIAQBaseline(&eCO2_base, &TVOC_base)) { // Retrieve current baseline values
      Serial.print("Current SGP30 baseline values: eCO2: 0x");
      Serial.print(eCO2_base, HEX);    // Display eCO2 baseline in hexadecimal
      Serial.print(", TVOC: 0x");
      Serial.println(TVOC_base, HEX);  // Display TVOC baseline in hexadecimal
      Serial.println("Baseline values retrieved successfully!");
    } else {
      Serial.println("Failed to retrieve SGP30 baseline values");
    }
  }
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi");  // Display connection attempt message
  WiFi.begin(ssid, password);          // Start WiFi connection with configured credentials

  int attempts = 0;                    // Counter for connection attempts
  while (WiFi.status() != WL_CONNECTED && attempts < 20) { // Try up to 20 attempts
    delay(500);                        // Wait 500ms between attempts
    Serial.print(".");                 // Print dot for each attempt
    attempts++;                        // Increment attempt counter
  }

  if (WiFi.status() == WL_CONNECTED) { // Check if connection successful
    Serial.println();                  // New line after dots
    Serial.println("WiFi connected!"); // Confirm successful connection
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());    // Display assigned IP address
  } else {
    Serial.println();
    Serial.println("Failed to connect to WiFi!"); // Handle connection failure
  }
}

void connectToThingsBoard() {
  Serial.print("Connecting to ThingsBoard..."); // Display connection attempt message
  
  // Ensure WiFi is connected first
  if (WiFi.status() != WL_CONNECTED) { // Check WiFi status before attempting MQTT
    Serial.println("WiFi not connected! Cannot connect to ThingsBoard.");
    thingsBoardConnected = false;      // Set connection flag to false
    return;                            // Exit function early
  }

  int attempts = 0;                    // Counter for connection attempts
  while (!client.connected() && attempts < 3) { // Try up to 3 attempts
    attempts++;                        // Increment attempt counter
    Serial.print("Attempt ");
    Serial.print(attempts);
    Serial.print("/3: ");
    
    if (client.connect(device_id, tb_token, NULL)) { // Attempt MQTT connection with device credentials
      Serial.println("Connected to ThingsBoard!"); // Confirm successful connection
      thingsBoardConnected = true;     // Set connection flag to true
      break;                           // Exit loop on success
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());    // Display MQTT client state code
      Serial.print(" (");
      
      // Decode MQTT error codes for troubleshooting
      switch(client.state()) {
        case -4: Serial.print("Connection timeout"); break;    // Network timeout
        case -3: Serial.print("Connection lost"); break;       // Network disconnection
        case -2: Serial.print("Connect failed"); break;        // Network failure
        case -1: Serial.print("Disconnected"); break;          // Clean disconnection
        case 1: Serial.print("Bad protocol"); break;           // Unsupported MQTT version
        case 2: Serial.print("Bad client ID"); break;          // Invalid client identifier
        case 3: Serial.print("Unavailable"); break;            // Server unavailable
        case 4: Serial.print("Bad credentials"); break;        // Authentication failure
        case 5: Serial.print("Unauthorized"); break;           // Authorization failure
        default: Serial.print("Unknown error");                // Unrecognized error
      }
      Serial.println(")");
      
      thingsBoardConnected = false;    // Set connection flag to false
      
      if (attempts < 3) {              // If more attempts remaining
        Serial.println("Retrying in 2 seconds...");
        delay(2000);                   // Wait before retry
      }
    }
  }
  
  if (!thingsBoardConnected) {         // Handle final failure
    Serial.println("Failed to connect to ThingsBoard after 3 attempts.");
  }
}

void checkThingsBoardConnection() {
  if (!client.connected() && WiFi.status() == WL_CONNECTED) { // Check if MQTT disconnected but WiFi still connected
    thingsBoardConnected = false;      // Update connection status
    Serial.println("ThingsBoard connection lost. Attempting to reconnect...");
    connectToThingsBoard();            // Attempt to reestablish connection
  }
  client.loop();                       // Process incoming MQTT messages and maintain connection
}

void sendTelemetryData() {
  // Check connection status
  if (!client.connected()) {           // Verify MQTT client is connected
    Serial.println("MQTT client not connected. Attempting to reconnect...");
    thingsBoardConnected = false;      // Update connection status
    connectToThingsBoard();            // Attempt reconnection
    return;                            // Exit function early
  }

  if (!thingsBoardConnected) {         // Double-check connection flag
    Serial.println("Not connected to ThingsBoard. Cannot send data.");
    return;                            // Exit function early
  }

  // Create JSON document with adequate size
  StaticJsonDocument<800> telemetry;  // Allocate JSON document with 800 bytes capacity

  // Populate JSON document with sensor data
  telemetry["timestamp"] = String(timeBuffer);        // Add formatted timestamp
  telemetry["device_id"] = device_id;                 // Add device identifier
  telemetry["tvoc"] = tvoc;                          // Add TVOC reading in ppb
  telemetry["eco2"] = eCO2_sgp;                      // Add eCO2 reading in ppm
  telemetry["mq135_raw"] = rawMQ135;                 // Add raw MQ135 ADC value
  telemetry["mq135_resistance"] = resistance_mq135;  // Add calculated resistance in kΩ
  telemetry["mq135_ppm"] = gas_mq135_ppm;           // Add calculated gas concentration
  telemetry["mics2714_raw"] = rawMICS2714;          // Add raw MiCS2714 ADC value
  telemetry["mics2714_ppm"] = gas_mics_ppm;         // Add calculated gas concentration
  telemetry["temperature"] = temperature_aht10;     // Add temperature reading in °C
  telemetry["humidity"] = humidity_aht10;           // Add humidity reading in %

  // Calculate air quality classification based on TVOC levels
  String gasQuality = "Good";          // Default classification
  if (tvoc > 2200) gasQuality = "Unhealthy";        // High pollution level
  else if (tvoc > 660) gasQuality = "Moderate";     // Moderate pollution level
  telemetry["gas_quality"] = gasQuality;             // Add classification to JSON

  // Calculate CO2 level classification based on eCO2 readings
  String co2Level = "Normal";          // Default classification
  if (eCO2_sgp > 1000) co2Level = "Poor";           // High CO2 level
  else if (eCO2_sgp > 800) co2Level = "Moderate";   // Moderate CO2 level
  telemetry["co2_level"] = co2Level;                 // Add classification to JSON

  // Create payload with larger buffer
  char payload[800];                   // Character array to hold JSON string
  size_t payloadSize = serializeJson(telemetry, payload); // Convert JSON to string and get size
  
  Serial.print("Payload size: ");
  Serial.print(payloadSize);
  Serial.println(" bytes");           // Display payload size for monitoring
  
  // Debug: Print the payload
  Serial.println("Sending payload:");
  Serial.println(payload);            // Display JSON payload for debugging

  // Publish with error checking
  bool publishResult = client.publish("v1/devices/me/telemetry", payload); // Send data to ThingsBoard
  
  if (publishResult) {                // Check if publish was successful
    Serial.println("✓ Gas Telemetry successfully sent to ThingsBoard!");
  } else {
    Serial.println("✗ Failed to send gas telemetry to ThingsBoard");
    Serial.print("MQTT Client State: ");
    Serial.println(client.state());   // Display client state for debugging
    Serial.print("WiFi Status: ");
    Serial.println(WiFi.status());    // Display WiFi status for debugging
    
    // Try to reconnect if there's an issue
    if (!client.connected()) {        // Check if connection was lost
      Serial.println("MQTT connection lost. Will attempt reconnect on next cycle.");
      thingsBoardConnected = false;   // Update connection status
    }
  }
}

void printAllData() {
  Serial.println("=== GAS SENSOR DATA (ESP32 #2) ===");      // Header for data display
  Serial.print("Timestamp: "); Serial.println(timeBuffer);    // Display current timestamp
  Serial.println("-----------------------------------");        // Separator line
  Serial.print("VOC Index (SGP30): "); Serial.print(tvoc); Serial.println(" ppb");           // Display TVOC in ppb
  Serial.print("eCO2 (SGP30): "); Serial.print(eCO2_sgp); Serial.println(" ppm");           // Display eCO2 in ppm
  Serial.print("Air Quality (MQ-135): "); Serial.print(rawMQ135); Serial.println(" (RAW ADC)");  // Display raw ADC
  Serial.print("MQ-135 Resistance (Rs): "); Serial.print(resistance_mq135); Serial.println(" kOhms"); // Display resistance
  Serial.print("MQ-135 Gas Concentration: "); Serial.print(gas_mq135_ppm); Serial.println(" ppm");    // Display concentration
  Serial.print("MICS-2714 (Raw ADC): "); Serial.print(rawMICS2714); Serial.print(", Approx. PPM: "); Serial.println(gas_mics_ppm); // Display MiCS data
  Serial.print("Temperature (AHT10): "); Serial.print(temperature_aht10); Serial.println(" °C");       // Display temperature
  Serial.print("Humidity (AHT10): "); Serial.print(humidity_aht10); Serial.println(" %");             // Display humidity
  Serial.print("WiFi Status: "); Serial.println(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected"); // Display WiFi status
  Serial.print("ThingsBoard Status: "); Serial.println(thingsBoardConnected ? "Connected" : "Disconnected");   // Display MQTT status
  Serial.println("===================================\n");      // Footer and spacing
}

void logDataToSD() {
  String dataString = "";              // Initialize empty string for CSV data
  dataString += String(timeBuffer);    // Add timestamp
  dataString += ",";                   // CSV delimiter
  dataString += String(tvoc);          // Add TVOC reading
  dataString += ",";                   // CSV delimiter
  dataString += String(eCO2_sgp);      // Add eCO2 reading
  dataString += ",";                   // CSV delimiter
  dataString += String(rawMQ135);      // Add raw MQ135 value
  dataString += ",";                   // CSV delimiter
  dataString += String(resistance_mq135); // Add calculated resistance
  dataString += ",";                   // CSV delimiter
  dataString += String(gas_mq135_ppm); // Add calculated concentration
  dataString += ",";                   // CSV delimiter
  dataString += String(rawMICS2714);   // Add raw MiCS2714 value
  dataString += ",";                   // CSV delimiter
  dataString += String(gas_mics_ppm);  // Add calculated concentration
  dataString += ",";                   // CSV delimiter
  dataString += String(temperature_aht10); // Add temperature reading
  dataString += ",";                   // CSV delimiter
  dataString += String(humidity_aht10); // Add humidity reading
  dataString += ",";                   // CSV delimiter
  dataString += device_id;             // Add device identifier (last column, no comma)

  File dataFile = SD.open("/gas_datalog.csv", FILE_APPEND); // Open CSV file in append mode

  if (dataFile) {                      // Check if file opened successfully
    dataFile.println(dataString);     // Write data string as new line
    dataFile.close();                 // Close file to ensure data is saved
    Serial.println("Gas data logged to SD card."); // Confirm successful logging
  } else {
    Serial.println("ERROR: Failed to open gas_datalog.csv on SD card."); // Handle file error
  }
}

uint32_t getAbsoluteHumidity(float temperature, float humidity) {
  // Calculate absolute humidity for SGP30 humidity compensation
  // This improves sensor accuracy by accounting for water vapor effects on gas measurements
  // Formula from Sensirion SGP30 Driver Integration Guide
  // Absolute humidity = mass of water vapor per unit volume of air [g/m³]
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // Convert to [mg/m^3] and scale to integer
  return absoluteHumidityScaled;       // Return scaled absolute humidity for SGP30 compensation
}
