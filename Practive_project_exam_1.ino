void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600);
}
//This program activates an LED at different frequencies based on the amount of light read by a photoresistor.

void loop() {
  int photoResistor = analogRead(A0);
  Serial.println(photoResistor);
  if (photoResistor > 550)  // 550 is the highest reading I got
    digitalWrite(13, LOW);
  else if (photoResistor < 550 && photoResistor > 498)
    Blink(1000);
  else if (photoResistor < 498 && photoResistor > 446)
    Blink(2000);
  else if (photoResistor < 446 && photoResistor > 394)
    Blink(3000);
  else if (photoResistor < 394 && photoResistor > 342)
    Blink(4000);
  else if (photoResistor < 342 && photoResistor > 290)    // In order to make 10 different frequencies, I took the total range and divided it by 10
    Blink(5000);
  else if (photoResistor < 290 && photoResistor > 238)
    Blink(6000);
  else if (photoResistor < 238 && photoResistor > 186)
    Blink(7000);
  else if (photoResistor < 186 && photoResistor > 134)
    Blink(8000);
  else if (photoResistor < 134 && photoResistor > 82)
    Blink(9000);
else if (photoResistor < 30)   // 30 is the lowest reading I got
Blink(0);
}
void Blink (int delayTime) { // This method makes the loop only run after the delay. Therefore, a photoResistor reading is only taken after the delay.
  digitalWrite (13, HIGH);
  delay(delayTime); // since the delay time is variable, there is a variable
  digitalWrite (13, LOW);
  delay(delayTime);
}
