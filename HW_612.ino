//  The HW-612 has a 'wake-on-motion' application (6.1 in datasheet) that I want to use, as well as programmable low-pass filter
//  which I can't seem to find anything on other than their existence.
//  I just implemented a software filter. Now I just need a sensor that works

//  3.3 V PLEASE

// From 'GetMagOffset'
#include <MPU9250_asukiaaa.h>
#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22    //???
#endif
MPU9250_asukiaaa MPU9250;
uint8_t sensorId;
float aX, aY, aZ, gX, gY, gZ, mX, mY, mZ;
float y_km1 = 0;
float u_km1 = 0;
float T = 0.05;
float omega_c = 20;
float y_k;
bool result;  //  library has int result, but the only checks are binary (T/F) and I like saving space
void setup() {
  Serial.begin(115200); 
  while(!Serial);
  Serial.println("started");

  #ifdef _ESP32_HAL_I2C_H_ // For ESP32
  Wire.begin(SDA_PIN, SCL_PIN); // SDA, SCL
  #else
  Wire.begin();
  #endif

  MPU9250.setWire(&Wire);
  // If cannot read sensor
  while (MPU9250.readId(&sensorId) != 0) {
    Serial.println("Cannot find device to read sensorId");
    delay(2000);
  }
  MPU9250.beginAccel();
  MPU9250.beginGyro();
  MPU9250.beginMag();
  MPU9250.magXOffset = -68;
  MPU9250.magYOffset = -27;
  MPU9250.magZOffset = 184;
}

void loop() {
//delay(T);
// Update readings

// The library I grabbed from has gyro and accel switched
result = MPU9250.accelUpdate();
if (!result) {
  gX = MPU9250.accelX();
  gY = MPU9250.accelY(); 
  gZ = MPU9250.accelZ();
}

result = MPU9250.magUpdate();
if (!result) {
  mX = MPU9250.magX();
  mY = MPU9250.magY();
  mZ = MPU9250.magZ();
}
  
//  Low-pass filter function calls
//gX = filter(gX);
//gY = filter(gY);
//gZ = filter(gZ);
//aX = filter(aX);
//aY = filter(aY);
//aZ = filter(aZ);
//mX = filter(mX);
//mY = filter(mY);
//mZ = filter(mZ);

// Finally, Serial print filtered values so that VPython can grab them. 
// Maybe compute angles here instead of computating them in VPython
// filter the signals so that the visual would not shake as much as unfiltered performance 

Serial.print(gX);Serial.print(",");Serial.print(gY);Serial.print(",");Serial.println(gZ);
//Serial.print(aX);Serial.print(",");Serial.print(aY);Serial.print(",");Serial.print(aZ);
//Serial.print(mX);Serial.print(",");Serial.print(mY);Serial.print(",");Serial.println(mZ);
}

float filter(float value_in){
y_k = (1-omega_c*T)*y_km1 + omega_c*T*u_km1;
y_km1 = y_k;
u_km1 = value_in; 
return y_k;
}