#include <Wire.h>
#include "Adafruit_AHTX0.h"

Adafruit_AHTX0 aht10 = Adafruit_AHTX0();

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("AHT10 Sensor Test");

  if (!aht10.begin()) {
    Serial.println("Failed to find AHT10 sensor!");
    while (1) delay(10);
  }

  Serial.println("AHT10 Found!");
}

void loop() {
  sensors_event_t humidity, temp;

  if (aht10.getEvent(&humidity, &temp)) {
    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    Serial.println(" °C");

    Serial.print("Humidity: ");
    Serial.print(humidity.relative_humidity);
    Serial.println(" %");
  } else {
    Serial.println("Failed to read data from AHT10 sensor");
  }

  delay(1000);
}
