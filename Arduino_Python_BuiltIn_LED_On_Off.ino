//  This program allows for serial communication between the Arduino IDE and python
//  turns on and off a built in LED with input from python. No setup on breadboard needed
//  Jacob Landerer  7/27/22

String InBytes;  // To hold user commands from python monitor
void setup() {
Serial.begin(9600);
pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if(Serial.available() > 0){ // This translates to 'If there is something written in the serial comm'
  InBytes = Serial.readStringUntil('\n'); //read it
    if(InBytes == "on"){
      digitalWrite(LED_BUILTIN,HIGH);   // if 'ON', turn LED on and tell the user it's on
      Serial.write("LED is on");
    }
    else if (InBytes == "off"){
      digitalWrite(LED_BUILTIN,LOW);  //  if 'OFF', turn LED off and tell the user it's off
      Serial.write("LED is off");
    }
    else {
      Serial.write("invalid input");
    }
  }
}
