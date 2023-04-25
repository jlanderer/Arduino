// This code works on a Step motor. Motor's 5-pin connector to MC10 chip. 4 female-male wires from MCU10 to arduino pins set to output below. +5 V & G from arduino to male pins on MCU10
// Jacob Landerer
// Some point in Summer '22 (Rev. 1/3/23)
#include <Stepper.h>
int prev = 0;
int aread = 0;
int ratio = 0;
Stepper stepper(2048,9,10,11,12);  // instance
void setup() {

Serial.begin(9600);
stepper.setSpeed(12);
}
void loop() {
stepper.step(2048);
delay(500);
}
