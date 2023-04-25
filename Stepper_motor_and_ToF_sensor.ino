// printed measurements are in mm
// Please use 3.3 V please
// 200 steps/rotation (I thought 2048)
#include <Wire.h>
#include <VL53L0X.h>
VL53L0X sensor;
#include <Stepper.h>
Stepper stepper(2048,9,10,11,12);  // instance
#define LONG_RANGE // For long range capability
//#define HIGH_SPEED
//#define HIGH_ACCURACY // choose and define
int new_dist; int prev_dist = 0;
void setup() {
  Serial.begin(9600);
  Wire.begin();
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

#if defined LONG_RANGE   // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif
#if defined HIGH_SPEED // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY   // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
#endif
stepper.setSpeed(10);
}

void loop() {
  new_dist = (int)(sensor.readRangeSingleMillimeters()*(2048.00/8190.00));
  Serial.println(new_dist);
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  int diff = new_dist-prev_dist;
  stepper.step(diff);
  prev_dist = new_dist;
  delay(500);
}
