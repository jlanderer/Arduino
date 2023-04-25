// This program gets data from a GY-91 gyroscope & accelerometer, 
// and sends it to the serial monitor. Depending on the accompanying programs, the data should be intercepted
// and manipulated by a python program. Connect SCL and SDA pins on GY and ADC (or no ADC, doesn't rly matter)
// As well as power pins (eithr 3.3V or 5V )
// Gyro: [-1 1],  Accel: [-800 800]
i = 0;
#include <MPU9250_asukiaaa.h> <-- relevant library
//#include <math.h>
#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 4
#define SCL_PIN 5
#endif
uint8_t sensorId; 
MPU9250_asukiaaa mySensor;
float aX,aY,aZ,gX,gY,gZ,;
float omega_C = 10^4;
int T = 5;
int result;

void setup() {
  Serial.begin(9600);
  while(!Serial);

#ifdef _ESP32_HAL_I2C_H_ // For ESP32
  Wire.begin(SDA_PIN, SCL_PIN);
  mySensor.setWire(&Wire);
#endif

  mySensor.beginAccel();
  mySensor.beginGyro();
}

void loop() { 
  delay(T) //ms
  i++;
  result = mySensor.readId(&sensorId);

  result = mySensor.gyroUpdate();
  if (result == 0) {
    aX(i) = mySensor.gyroX();
    aY(i) = mySensor.gyroY();
    aZ(i) = mySensor.gyroZ();
}                                //  Specific functions for getting gyro and accle data are switched in the library I use
  result = mySensor.accelUpdate();
  if (result == 0) {
    gX(i) = mySensor.accelX();
    gY(i) = mySensor.accelY();
    gZ(i) = mySensor.accelZ();   
}

// Let's do some quick maths on the signals using a low-pass filter:

gX = filter()


  Serial.print(aX);Serial.print(" , ");Serial.print(aY);Serial.print(" , ");Serial.print(aZ);Serial.print(" , ");
  Serial.print(gX);Serial.print(" , ");Serial.print(gY);Serial.print(" , ");Serial.println(gZ);
}

float filter(y_km1,u_km1){
  y_k = (1-omega_c*T)*y_km1+omega_C*T*u_m1;
}