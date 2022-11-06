#include <IBusBM.h>
#include "SoftPWM.h"
#include <Wire.h>


// GLOBAL VARIABLES FOR READING IBUSX ---------------------------------
IBusBM ibusRc;
HardwareSerial& ibusRcSerial = Serial1;
// --------------------------------------------------------------------
 
// GLOBAL VARIABLES FOR MOTORS ----------------------------------------
int frontLeft  = 11;
int frontRight = 13;
int rearLeft   =  5;
int rearRight  = 12; 
// --------------------------------------------------------------------

// GLOBAL VARIABLES FOR MPU ----------------------------------------
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

// --------------------------------------------------------------------

void setup() {
  // IBUS SETUP ------------------------------------------------------
  ibusRc.begin(ibusRcSerial);
  // ------------------------------------------------------------------


  // PWM SETUP -------------------------------------------------------
  SoftPWMBegin();
  SoftPWMSet(frontLeft,  0);
  SoftPWMSet(frontRight, 0);
  SoftPWMSet(rearLeft ,  0);
  SoftPWMSet(rearRight,  0);
  // ------------------------------------------------------------------


  // MPU SETUP --------------------------------------------------------
    Serial.begin(19200);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  /*
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
  */
  // Call this function if you need to get the IMU error values for your module
  calculate_IMU_error();
  delay(20);

  // ------------------------------------------------------------------
}

// READS DATA FROM THE IBUS -------------------------------------------
int readChannel(byte channelInput, int minLimit, int  maxLimit, int defaultValue){
  uint16_t ch = ibusRc.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}
// --------------------------------------------------------------------
// selfLevel takes pitch and roll
float selfLevel(int motor, float pitch_angle, float roll_angle, int pitch, int roll){
  float multiplier = 1.0;
  float response = 0.1;
  
  // For now, only do anything if pitch and roll are zero
  if (pitch < 10 && pitch > -10 && roll < 10 && roll > -10){

    // Level the pitch
    // Front needs power
    if(pitch_angle > 2.5 && (motor == frontLeft || motor == frontRight)){
      multiplier += response;
    }
    // Rear needs power
    else if(pitch_angle < -2.5 && (motor == rearLeft || motor == rearRight)){
      multiplier += response;
    }

    if(pitch_angle < -2.5 && (motor == frontLeft || motor == frontRight)){
      multiplier -= response;
    }
    else if (pitch_angle > 2.5 && (motor == rearLeft || motor == rearRight)){
      multiplier -= response;
    }

    // Level the roll
    // Left needs power
    if (roll_angle < -2.5 && (motor == frontLeft || motor == rearLeft)){
      multiplier += response;
    }
    // Right needs power 
    else if (roll_angle > 2.5 && (motor == frontRight || motor == rearRight)){
      multiplier += response;
    } 

    if(roll_angle < -2.5 && (motor == frontRight || motor == rearRight)){
      multiplier -= response;
    }
    else if(roll_angle > 2.5 && (motor == frontRight || motor == rearRight)){
      multiplier -= response;
    }
  }

  return multiplier;
}

// pulseWidth takes motor, throttle, roll, pitch, yaw
// We need to stop the pulsewidth from passing a certain value if the angle from the mpu is a certain angle
int pulseWidth(int motor, int throttle, int roll, int pitch, int yaw){
  // Throttle is 0-80

  // Roll, pitch, yaw are -100 to 100

  float response = 0.005;

  float multiplier = 1.0;

  // Calculate roll
  // Roll right, power left
  if (roll > 0 && (motor == rearLeft || motor == frontLeft)){
    multiplier += (response * roll);
  }
  // Roll left, power right
  else if (roll < 0 && (motor == rearRight || motor == frontRight)){
    multiplier += (response * (-roll));
  }

  // Calculate pitch
  // Pitch forward, power rear
  if (pitch > 0 && (motor == rearRight || motor == rearLeft)){
    multiplier += (response * pitch);
  }
  // Pitch back, power front
  else if (pitch < 0 && (motor == frontLeft || motor == frontRight)){
    multiplier += (response * (-pitch));
  }

  // Calculate yaw
  // Yaw Counterclockwise
  if(yaw > 0 && (motor == frontLeft || motor == rearRight)){
    multiplier += (response * yaw);
  }
  // Yaw Clockwise
  if(yaw < 0 && (motor == frontRight || motor == rearLeft)){
    multiplier += (response * (-yaw));
  }

  // Combine calculations to create multiplier

  return (int)(multiplier * throttle);
}

void loop() {
  //READ DATA FROM IBUS AND SET PWM ----------------------------------
  
  //CHANNEL 2 THROTTLE, CHANNEL 3 YAW, CHANNEL 0 ROLL, CHANNEL 1 PITCH 
  int throttle = readChannel(2, 0, 80, 0);
  int yawCtrl = readChannel(3, -100, 100, 0);
  int rollCtrl = readChannel(0, -100, 100, 0);
  int pitchCtrl = readChannel(1, -100, 100, 0);
  
  //Serial.println(value);
  // Calculate the pulse widths
  int frontLeftPW = pulseWidth(frontLeft, throttle, rollCtrl, pitchCtrl, yawCtrl) * selfLevel(frontLeft, pitch, roll, pitchCtrl, rollCtrl);
  int rearLeftPW = pulseWidth(rearLeft, throttle, rollCtrl, pitchCtrl, yawCtrl) * selfLevel(rearLeft, pitch, roll, pitchCtrl, rollCtrl);
  int frontRightPW = pulseWidth(frontRight, throttle, rollCtrl, pitchCtrl, yawCtrl) * selfLevel(frontRight, pitch, roll, pitchCtrl, rollCtrl);
  int rearRightPW = pulseWidth(rearRight, throttle, rollCtrl, pitchCtrl, yawCtrl) * selfLevel(rearRight, pitch, roll, pitchCtrl, rollCtrl);

  // Update signals
  SoftPWMSetPercent(frontLeft, frontLeftPW);
  SoftPWMSetPercent(rearLeft, rearLeftPW);
  SoftPWMSetPercent(frontRight, frontRightPW);
  SoftPWMSetPercent(rearRight, rearRightPW);
  // ------------------------------------------------------------------

// READ DATA FROM MPU -----------------------------------------------
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX + 0.56; // GyroErrorX ~(-0.56)
  GyroY = GyroY - 2; // GyroErrorY ~(2)
  GyroZ = GyroZ + 0.79; // GyroErrorZ ~ (-0.8)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = 0.96 * gyroAngleX + 0.04 * accAngleX;
  gyroAngleY = 0.96 * gyroAngleY + 0.04 * accAngleY;  
  yaw =  yaw + GyroZ * elapsedTime -50;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = gyroAngleX;
  pitch = gyroAngleY;
  
  // Print the values on the serial monitor
  Serial.print(roll);
  Serial.print("/");
  Serial.print(pitch);
  Serial.print("/");
  Serial.println(yaw);
  }


// ------------------------------------------------------------------



void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 2000) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 2000;
  AccErrorY = AccErrorY / 2000;
  c = 0;
  // Read gyro values 200 times
  while (c < 2000) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 2000;
  GyroErrorY = GyroErrorY / 2000;
  GyroErrorZ = GyroErrorZ / 2000;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}
