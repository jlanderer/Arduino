// Keywords:
// Dead reckoning
// Calibration at rest
// 3-2-1 Euler Angles & Rotational Kinematics
// Difference equation low-pass filter
// I2C Serial Coomunication Protocol

//  Process:
//  0. Calibrate gyro to negate drift & init sensor
//  1. Get raw data
//  2. Low-pass filter for mag and accel
//  3. determine heading via each parameter, complimentary filter & respective weight
//  4. my old friend Mr.Serial.Print()
//  5. Profit
//  6. delay

#include <Wire.h>
#include <MPU9250_asukiaaa.h>

// Declare objects and vars
MPU9250_asukiaaa imu; 
float True_heading[3] = {0.0,0.0,0.0}; // pitch(x), then roll(y), then yaw(z)
float a_filt[3],m_filt[3];
float a_km1[3] = {0.0,0.0,0.0};
float m_km1[3] = {0.0,0.0,0.0};
float a_filt_km1[3] = {0.0,0.0,0.0};
float m_filt_km1[3] = {0.0,0.0,0.0};
float omega_c = 40.0; // Cutoff frequency for filtering purposes
float alpha = 0.98; // For comp filter (weighing)
float bias;
void setup() {
  Serial.begin(115200);
  Wire.begin();
  imu.setWire(&Wire);
  imu.beginGyro(GYRO_FULL_SCALE_2000_DPS);
  imu.beginAccel(ACC_FULL_SCALE_16_G);
  imu.beginMag(MAG_MODE_SINGLE);
  imu.magXOffset = -58;
  imu.magYOffset = -11; // For most accurate determination from magnetometer
  imu.magZOffset = 119;

  // debias to get rid of steady-state error
  int i = 0; 
  unsigned long start = millis(); 
  float bias_x[1000];

  while (millis()-start < 5000) {
    bias_x[i] = imu.gyroX();
    i++;
  }
  
  for (i=0;i<1000;i++){
    bias += bias_x[i];
  }
  bias /= 1000;
}

void loop() {
  float T = 10.0; // Sampling delay in ms
  // Get data
  imu.gyroUpdate();
  imu.accelUpdate();
  imu.magUpdate();
  float a[3] = {imu.accelX(), imu.accelY(), imu.accelZ()};
  float g[3] = {(imu.gyroX() - bias) * T/1000,(imu.gyroY() - bias) * T/1000,(imu.gyroZ() - bias) * T/1000};
  float m[3] = {imu.magX(), imu.magY(), imu.magZ};

  // accelerometer low-pass filter
  a_filt[0] = (1-omega_c*T/1000)*a_km1[0] + omega_c*T*a_km1[0];
  a_filt_km1[0] = a_filt[0];
  a_km1[0] = a[0];

  a_filt[1] = (1-omega_c*T/1000)*a_km1[1] + omega_c*T*a_km1[1];
  a_filt_km1[1] = a_filt[1];
  a_km1[1] = a[1];

  a_filt[2] = (1-omega_c*T/1000)*a_km1[2] + omega_c*T*a_km1[2];
  a_filt_km1[2] = a_filt[2];
  a_km1[2] = a[2];

  // Magnetometer low-pass filter
  m_filt[0] = (1-omega_c*T/1000)*m_km1[0] + omega_c*T*m_km1[0];
  m_filt_km1[1] = m_filt[0];
  m_km1[0] = m[0];

  m_filt[1] = (1-omega_c*T/1000)*m_km1[1] + omega_c*T*m_km1[1];
  m_filt_km1[1] = m_filt[1];
  m_km1[1] = m[1];

  m_filt[2] = (1-omega_c*T/1000)*m_km1[2] + omega_c*T*m_km1[2];
  m_filt_km1[2] = m_filt[2];
  m_km1[2] = m[2];

  // Rotation Differential Kinematics

  // Roll & pitch from accelerometer
  float roll_accel = atan2(a_filt[1],a_filt[2]);// * (180/PI);
  float pitch_accel = atan2(-a_filt[1],sqrt(a_filt[0]*a_filt[2]*a_filt[2]));// * (180/PI);

  // Yaw from magnetometer
  float yaw_mag = atan2(-m_filt[1],m_filt[0]);// * (180/PI);
  //if (yaw_mag < 0)
  //yaw_mag += 360;

  // Complimetary filter and weighing

  // True_heading[1] = alpha * gyro_roll + (1- alpha) * roll_accel;
  // True_heading[0] = alpha * gyro_pitch + (1 - alpha) * pitch_accel;
  // True_heading[2] = alpha * gyro_yaw + (1 - alpha) * yaw_mag;


    
  // This works
  True_heading[0] += g[0];True_heading[1] += g[1]; True_heading[2] += g[2];

  // This does not
  //True_heading[0] += gyro_pitch;True_heading[1] += gyro_roll;True_heading[2] += gyro_yaw;

  Serial.print(True_heading[0]);Serial.print(",");
  Serial.print(True_heading[1]);Serial.print(",");
  Serial.println(True_heading[2]);
  delay(T);
}