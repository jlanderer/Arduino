// Moves a servo varying amounts based on distance detected by an ultrasonic detector

const int trigPin = 11;
const int echoPin = 12;
#include <Servo.h>       
Servo MyServo;   
const int servoPin = 9; 
void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);   // outputs pulses of electricity
  pinMode(echoPin, INPUT);    // measures the duration of pulses from the distance sensor
  MyServo.attach(servoPin);
}

void loop() {
  int distance = getDistance();
  if (distance <= 5) {
    MoveServo(5);
  }
  else if (distance > 5 && distance <= 15) {
    MoveServo(500);
  }
  else {// If the distance is greater than 15
    MoveServo(5000);
  }
}
float getDistance()
{
  float echoTime;                   // stores the time it takes for a ping to bounce off an object
  float calculatedDistance;         // stores the distance calculated based on echo time

  //send out an ultrasonic pulse that's 10ms long
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  echoTime = pulseIn(echoPin, HIGH);      // Set total time equal to how long it takes for the
                                          // pulse to bounce back to the sensor

  calculatedDistance = echoTime / 148.0;  // distance of the object that reflected the pulse
  // half the bounce time (because bounce time is from sensor to object and back to sensor)
  // multiplied by the speed of sound because distance = speed / time

  return calculatedDistance;              // send back the distance that was calculated
}

void MoveServo (int delaySpeed){
  // Only moves 180 degrees
  MyServo.write(0);
  delay(delaySpeed);
  MyServo.write(20);
  delay(delaySpeed);
  MyServo.write(40);
  delay(delaySpeed);
  MyServo.write(60);
  delay(delaySpeed);
  MyServo.write(80);
  delay(delaySpeed);
  MyServo.write(100);
  delay(delaySpeed);
  MyServo.write(120);
  delay(delaySpeed);
  MyServo.write(140);
  delay(delaySpeed);
  MyServo.write(160);
  delay(delaySpeed);
  MyServo.write(180);
  delay(delaySpeed);
  MyServo.write(160);
  delay(delaySpeed);
  delay(delaySpeed);
  MyServo.write(140);
  delay(delaySpeed);
  MyServo.write(120);
  delay(delaySpeed);
  MyServo.write(100);
  delay(delaySpeed);
  MyServo.write(80);
  delay(delaySpeed);
  MyServo.write(60);
  delay(delaySpeed);
  MyServo.write(40);
  delay(delaySpeed);
  MyServo.write(20);
  delay(delaySpeed);
  MyServo.write(0);
  }
