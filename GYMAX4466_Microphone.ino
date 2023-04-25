// Jacob Landerer 1/5/23
// This program utilizes an arduino microphone sensor. OUT on chip should go to analog input,
// Other 2 pins are self explanatory. Voltage range is 2.4-5.5, so either 3.3 or 5 are safe from any arduino

int noise_Level = 0;
void setup() {
Serial.begin(9600);
}

void loop() {
noise_Level = analogRead(0);
Serial.println(noise_Level);
}
