// Jacob Landerer 1/5/23
// Noise & ToF take 3.3 V
// HW-612 takes 5 V

// Include necessary libraries and definitions
# include <Stepper.h>
Stepper stepper(5,9,10,11,12);
# include <Arduino.h>
# include <BMx280I2C.h>            
# define I2C_ADDRESS 0x76  // for this bmp
BMx280I2C bmx280(I2C_ADDRESS);    
# include <MPU9250_asukiaaa.h>             
MPU9250_asukiaaa magAccel; 
# define CALIB_SEC 20
# include <VL53L0X.h>            
# include <Wire.h>
VL53L0X ToF;
# define LONG_RANGE 
# define HIGH_SPEED        
//# define HIGH_ACCURACY 

// Initialize necessary vars
double ToF_height;
double noise_Level;
double temp;
double pressure_64;
uint8_t sensorId;
double aX, aY, aZ, gX, gY, gZ;
double a_resultant;
bool status;

// 1. Include libs, initialize vars ifdefine functions
// 2. Initialize sensors, notify if fail
// 3. Begin serial comms @ desired baud, init sensors
// 4. Begin taking readings, maybe do stuff based on values like a real controls engineer
// 5. HMI?? LEDs?? OLED status display?
// 6. bool Status and enum cause for change in status?

/******************SETUP************************************/
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Wire.begin();
  status = 1;
  pinMode(4,OUTPUT);  // For red and green status LEDs
  pinMode(7,OUTPUT);
  
  // Check BMP-280 Sensor status
  if (!bmx280.begin()){
    Serial.println("begin() failed. check your BMx280 Interface and I2C Address.");
    while (1);
  }
  //reset BMP-280 to default parameters.
  bmx280.resetToDefaults();
  bmx280.writeOversamplingPressure(BMx280MI::OSRS_P_x16);
  bmx280.writeOversamplingTemperature(BMx280MI::OSRS_T_x16);
  
 //ToF sensor check
  ToF.setTimeout(500);
  if (!ToF.init())
  {
    Serial.println("Failed to detect and initialize ToF sensor!");
    while (1) {}
  }
  #if defined LONG_RANGE   // lower the return signal rate limit (default is 0.25 MCPS)
  ToF.setSignalRateLimit(0.1);  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  ToF.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  ToF.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  #endif
  #if defined HIGH_SPEED // reduce timing budget to 20 ms (default is about 33 ms)
  ToF.setMeasurementTimingBudget(20000);
  #elif defined HIGH_ACCURACY   // increase timing budget to 200 ms
  ToF.setMeasurementTimingBudget(200000);
  #endif
  
  // mag&Accel sensor check
  magAccel.setWire(&Wire);
  while (magAccel.readId(&sensorId) != 0) {
    Serial.println("Cannot find device to read sensorId");
  }
  magAccel.beginAccel();
  magAccel.beginGyro();
}
/***********LOOP*********************/
void loop() {
  uint8_t sensorId;
  int result = magAccel.readId(&sensorId);
  status = 1;
  ToF_height = ToF.readRangeSingleMillimeters();
  
  result = magAccel.accelUpdate();
  if (result == 0) {
    gX = magAccel.accelX();
    gY = magAccel.accelY();
    gZ = magAccel.accelZ();
}
  result = magAccel.gyroUpdate();
  if (result == 0) {
    aX = magAccel.gyroX();      // Graphically isolating these vars, varying them one by one showed they are mixed up in the library,
    aY = magAccel.gyroY();      // So I'm switching them to make the measurements accurate
    aZ = magAccel.gyroZ();
}
  //start a BMP-280 measurement
  if (!bmx280.measure())
  {
    Serial.println("could not start bmp measurement, is a measurement already running?");
    return;
  }
  //wait for the measurement to finish
  do
  {
  
  } while (!bmx280.hasValue());
  //pressure = bmx280.getPressure()/101325.0; // kPa --> atm
  pressure_64 = bmx280.getPressure64(); // kPa --> atm
  temp = bmx280.getTemperature();  // C
  if (isnan(pressure_64))
    pressure_64 = 0;
  if (isnan(temp))
  temp = 0;

  //noise from microphone
  noise_Level = analogRead(0);

  if(abs(gX) >1 ||abs(gY)>1||abs(gZ)>1)  // Conditional
  status = 0;

  // Update LEDs as per status
  if(status){
  digitalWrite(7, HIGH);
  digitalWrite(4, LOW);
  }
  else {
  digitalWrite(4,HIGH);
  digitalWrite(7, LOW);;
  }
  
  //conversions for ease of plotting
  a_resultant = sqrt(pow(aX,2)+pow(aY,2)+pow(aZ,2));
  noise_Level = noise_Level/500.00;
  pressure_64 = pressure_64/101325;
  ToF_height = ToF_height/1000;
  
  // One Big Print statement
  Serial.print(noise_Level);Serial.print(" , ");Serial.print(ToF_height);
  Serial.print(" , ");Serial.print(pressure_64);Serial.print(" , ");
  Serial.print(temp);Serial.print(" , ");Serial.print(a_resultant);
  Serial.print(" , ");Serial.print(gX);Serial.print(" , ");
  Serial.print(gY);Serial.print(" , ");Serial.println(gZ);
}
