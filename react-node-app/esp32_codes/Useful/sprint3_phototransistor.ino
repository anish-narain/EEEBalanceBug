const int lightSensorPin = A0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  
  int lightSensorReading = 0;
  lightSensorReading = analogRead(lightSensorPin);
  Serial.println(lightSensorReading);
  delay(100);

}