void setup() {
  Serial.begin(9600);  // Monitor
  Serial1.begin(9600); // SDS011 UART (use Serial1)
}

void loop() {
  if (Serial1.available() >= 10) {
    byte buf[10];
    Serial1.readBytes(buf, 10);
    if (buf[0] == 0xAA && buf[9] == 0xAB) {
      int pm25 = buf[2] + buf[3]*256;
      int pm10 = buf[4] + buf[5]*256;
      Serial.print("SDS011 PM2.5: ");
      Serial.print(pm25 / 10.0);
      Serial.print(" ug/m3, PM10: ");
      Serial.print(pm10 / 10.0);
      Serial.println(" ug/m3");
    }
  }
  delay(1000);
}
