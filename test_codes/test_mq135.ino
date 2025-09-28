int mq135Pin = A0;

void setup() {
  Serial.begin(9600);
  pinMode(mq135Pin, INPUT);
}

void loop() {
  int value = analogRead(mq135Pin);
  Serial.print("MQ135 Value: ");
  Serial.println(value);
  delay(1000);
}
