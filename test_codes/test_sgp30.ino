#include <Wire.h>
#include <Adafruit_SGP30.h>
Adafruit_SGP30 sgp;

void setup() {
  Serial.begin(9600);
  if (!sgp.begin()) {
    Serial.println("SGP30 not found!");
    while (1) delay(10);
  }
}

void loop() {
  if (sgp.IAQmeasure()) {
    Serial.print("TVOC: ");
    Serial.print(sgp.TVOC);
    Serial.print(" ppb, eCO2: ");
    Serial.print(sgp.eCO2);
    Serial.println(" ppm");
  }
  delay(1000);
}
