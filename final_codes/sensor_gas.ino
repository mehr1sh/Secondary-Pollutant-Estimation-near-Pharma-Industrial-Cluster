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
const char* ssid = "realme";// Replace with your WiFi SSID
const char* password = "avaditya";// Replace with your WiFi password


const char* tb_server = "demo.thingsboard.io";// ThingsBoard server
const int tb_port = 1883;// MQTT port
const char* tb_token = "ZnCXWNQanEGsKVJviiOy";// Replace with your gas sensor device token
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
unsigned long prevBaselineMillis = 0;
const long baselineInterval = 3600000; // Update baseline every hour (3600000 ms)


// --- ThingsBoard Connection Status ---
bool thingsBoardConnected = false;


// --- FORWARD DECLARATIONS ---
void printAllData();
void logDataToSD();
void connectToWiFi();
void connectToThingsBoard();
void sendTelemetryData();
void checkThingsBoardConnection();
uint32_t getAbsoluteHumidity(float temperature, float humidity);


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
    
    // Read SGP30 Serial ID and Feature Set
    Serial.print("SGP30 Serial ID: 0x");
    Serial.print(sgp.serialnumber[0], HEX);
    Serial.print(sgp.serialnumber[1], HEX);
    Serial.println(sgp.serialnumber[2], HEX);
    
    // Perform soft reset
    Serial.println("Performing SGP30 soft reset...");
    // Note: Soft reset is handled internally by the library during begin()
    
    // Set initial baseline values to kickstart the sensor
    // These are typical baseline values - the sensor will adapt over time
    Serial.println("Setting initial SGP30 baseline values...");
    sgp.setIAQBaseline(0x8E68, 0x8F41); // Typical baseline values
    
    Serial.println("SGP30 baseline set. Starting warmup period...");
    Serial.println("Note: TVOC readings may take 12-24 hours to fully stabilize");
    
    // Take some initial readings to establish baseline
    for (int i = 0; i < 15; i++) {
      if (sgp.IAQmeasure()) {
        Serial.print("Warmup reading "); Serial.print(i+1); 
        Serial.print("/15: TVOC = "); Serial.print(sgp.TVOC);
        Serial.print(" ppb, eCO2 = "); Serial.print(sgp.eCO2); Serial.println(" ppm");
        
        // If we start getting non-zero TVOC readings, we can break early
        if (sgp.TVOC > 0) {
          Serial.println("TVOC readings detected! Baseline is establishing.");
        }
      }
      delay(1000);
    }
    Serial.println("SGP30 warmup complete!");
  }

  // Initialize AHT10 on alternate I2C pins
  Wire1.begin(16, 17); // SDA=16, SCL=17 for AHT10 (avoiding conflict with SD CS pin 5)
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
  client.setBufferSize(1024); // Increase MQTT buffer size for larger payloads
  client.setKeepAlive(60); // Set keep alive to 60 seconds
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
      // Check for valid readings (AHT10 should not return exactly 0.0 for both)
      if (temperature_aht10 == 0.0 && humidity_aht10 == 0.0) {
        Serial.println("Warning: AHT10 returned zero values - possible sensor issue");
      }
      
      // Set humidity compensation for SGP30 if we have valid readings
      if (temperature_aht10 > 0.0 && humidity_aht10 > 0.0) {
        uint32_t absolute_humidity = getAbsoluteHumidity(temperature_aht10, humidity_aht10);
        sgp.setHumidity(absolute_humidity);
      }
    } else {
      Serial.println("ERROR: AHT10 reading failed! Check I2C connections (SDA=16, SCL=17)");
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
  
  // Task 4: Update SGP30 baseline every hour for long-term stability
  if (currentMillis - prevBaselineMillis >= baselineInterval) {
    prevBaselineMillis = currentMillis;
    uint16_t eCO2_base, TVOC_base;
    if (sgp.getIAQBaseline(&eCO2_base, &TVOC_base)) {
      Serial.print("Current SGP30 baseline values: eCO2: 0x");
      Serial.print(eCO2_base, HEX);
      Serial.print(", TVOC: 0x");
      Serial.println(TVOC_base, HEX);
      Serial.println("Baseline values retrieved successfully!");
    } else {
      Serial.println("Failed to retrieve SGP30 baseline values");
    }
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
  
  // Ensure WiFi is connected first
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected! Cannot connect to ThingsBoard.");
    thingsBoardConnected = false;
    return;
  }

  int attempts = 0;
  while (!client.connected() && attempts < 3) {
    attempts++;
    Serial.print("Attempt ");
    Serial.print(attempts);
    Serial.print("/3: ");
    
    if (client.connect(device_id, tb_token, NULL)) {
      Serial.println("Connected to ThingsBoard!");
      thingsBoardConnected = true;
      break;
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.print(" (");
      
      // Decode MQTT error codes
      switch(client.state()) {
        case -4: Serial.print("Connection timeout"); break;
        case -3: Serial.print("Connection lost"); break;
        case -2: Serial.print("Connect failed"); break;
        case -1: Serial.print("Disconnected"); break;
        case 1: Serial.print("Bad protocol"); break;
        case 2: Serial.print("Bad client ID"); break;
        case 3: Serial.print("Unavailable"); break;
        case 4: Serial.print("Bad credentials"); break;
        case 5: Serial.print("Unauthorized"); break;
        default: Serial.print("Unknown error");
      }
      Serial.println(")");
      
      thingsBoardConnected = false;
      
      if (attempts < 3) {
        Serial.println("Retrying in 2 seconds...");
        delay(2000);
      }
    }
  }
  
  if (!thingsBoardConnected) {
    Serial.println("Failed to connect to ThingsBoard after 3 attempts.");
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
  // Check connection status
  if (!client.connected()) {
    Serial.println("MQTT client not connected. Attempting to reconnect...");
    thingsBoardConnected = false;
    connectToThingsBoard();
    return;
  }

  if (!thingsBoardConnected) {
    Serial.println("Not connected to ThingsBoard. Cannot send data.");
    return;
  }

  // Create JSON document with adequate size
  StaticJsonDocument<800> telemetry;

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

  // Create payload with larger buffer
  char payload[800];
  size_t payloadSize = serializeJson(telemetry, payload);
  
  Serial.print("Payload size: ");
  Serial.print(payloadSize);
  Serial.println(" bytes");
  
  // Debug: Print the payload
  Serial.println("Sending payload:");
  Serial.println(payload);

  // Publish with error checking
  bool publishResult = client.publish("v1/devices/me/telemetry", payload);
  
  if (publishResult) {
    Serial.println("✓ Gas Telemetry successfully sent to ThingsBoard!");
  } else {
    Serial.println("✗ Failed to send gas telemetry to ThingsBoard");
    Serial.print("MQTT Client State: ");
    Serial.println(client.state());
    Serial.print("WiFi Status: ");
    Serial.println(WiFi.status());
    
    // Try to reconnect if there's an issue
    if (!client.connected()) {
      Serial.println("MQTT connection lost. Will attempt reconnect on next cycle.");
      thingsBoardConnected = false;
    }
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

uint32_t getAbsoluteHumidity(float temperature, float humidity) {
  // Calculate absolute humidity for SGP30 humidity compensation
  // approximation formula from Sensirion SGP30 Driver Integration Guide
  // https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/9_Gas_Sensors/Sensirion_Gas_Sensors_SGP30_Driver-Integration-Guide_HW_I2C.pdf
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
  return absoluteHumidityScaled;
}