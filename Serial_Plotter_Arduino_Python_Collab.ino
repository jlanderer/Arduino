void setup() {
Serial.begin(9600);
pinMode(4, OUTPUT); // Double check ports & baud rate please!!!!!!
}

void loop() {
digitalWrite(4, HIGH);
int volts = analogRead(4);
Serial.println(volts);
delay(200);
}
