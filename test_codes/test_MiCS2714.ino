int micsPin = A0;  // Connect sensor analog output to Arduino analog pin A0

void setup() {
  Serial.begin(9600);
  pinMode(micsPin, INPUT);
}

void loop() {
  int sensorValue = analogRead(micsPin);
  Serial.print("MICS2714-V10 Sensor Value: ");
  Serial.println(sensorValue);
  delay(1000);
}
