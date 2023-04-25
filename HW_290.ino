// Jacob Landerer Summer '22
// HW-290 is MPU-6050 (accel & gyro) and BMP-280 (pressure)

# include <BMx280I2C.h>            
# define I2C_ADDRESS 0x76
BMx280I2C bmx280(I2C_ADDRESS);
// THIS SENSOR IS ALSO AN ACCELEROMETER, GYRO & MAGNETOMETER
// HW-290 is the same thing as GY-87/86
// Although the compass does not want to cooperate
// USE 3.3 V PLEASE UNLESS YOU WANNA RELEASE THE MAGIC SMOKE
// Needs to be 115200 baud
// BMx280MI bmp;

void setup() {
  Serial.begin(115200);
  if (!bmx280.begin()) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  while (1) {}
  }
}
void loop() {
  Serial.print(bmx280.readTemperature());Serial.print(","); Serial.println(bmx280.readPressure());
  delay(250);
}
//Deceivingly simple
