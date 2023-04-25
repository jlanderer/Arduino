//Orange PWM
//Red +5 V
//brown ground

#include <Servo.h>
Servo servo;
void setup() {
  // put your setup code here, to run once:
pinMode(3,OUTPUT);
servo.attach(3);

}
void loop() {
  // put your main code here, to run repeatedly:
servo.write(15);
delay(500);
servo.write(15);
delay(500);
servo.write(30);
delay(500);
servo.write(45);
delay(500);
servo.write(60);
delay(500);
servo.write(75);
delay(500);
servo.write(90);
delay(500);
servo.write(105);
delay(500);
servo.write(120);
delay(500);
servo.write(135);
delay(500);
servo.write(150);
delay(500);
servo.write(165);
delay(500);
servo.write(180);
delay(500);
servo.write(195);
delay(500);
servo.write(210);
delay(500);
servo.write(225);
delay(500);
servo.write(240);
delay(500);
servo.write(255);
delay(500);
servo.write(270);
delay(500);
servo.write(285);
delay(500);
servo.write(300);
delay(500);
servo.write(315);
delay(500);
servo.write(330);
delay(500);
servo.write(345);
delay(500);
servo.write(360);
delay(500);
}
