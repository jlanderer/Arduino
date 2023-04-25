// printed measurements are in mm
// Please use 3.3 V please

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

//#define LONG_RANGE // For long range capability
//#define HIGH_SPEED
#define HIGH_ACCURACY // choose and define

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
}

void loop() {
  Serial.println(sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
}
