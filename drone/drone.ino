#include <IBusBM.h>
#include "SoftPWM.h"

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

  // Combine calculations to create multiplier

  return (int)(multiplier * throttle);
}

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


  // MPU SETUP

  // ------------------------------------------------------------------
}

// READS DATA FROM THE IBUS -------------------------------------------
int readChannel(byte channelInput, int minLimit, int  maxLimit, int defaultValue){
  uint16_t ch = ibusRc.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}
// --------------------------------------------------------------------


void loop() {
  //READ DATA FROM IBUS AND SET PWM ----------------------------------
  //CHANNEL 2 THROTTLE, CHANNEL 3 YAW, CHANNEL 0 ROLL, CHANNEL 1 PITCH 
  int throttle = readChannel(2, 0, 80, 0);
  int yaw = readChannel(3, -100, 100, 0);
  int roll = readChannel(0, -100, 100, 0);
  int pitch = readChannel(1, -100, 100, 0);
  
  //Serial.println(value);

  // Calculate the pulse widths
  int frontLeftPW = pulseWidth(frontLeft, throttle, roll, pitch, yaw);
  int rearLeftPW = pulseWidth(rearLeft, throttle, roll, pitch, yaw);
  int frontRightPW = pulseWidth(frontRight, throttle, roll, pitch, yaw);
  int rearRightPW = pulseWidth(rearRight, throttle, roll, pitch, yaw);

  // Update signals
  SoftPWMSetPercent(frontLeft, frontLeftPW);
  SoftPWMSetPercent(rearLeft, rearLeftPW);
  SoftPWMSetPercent(frontRight, frontRightPW);
  SoftPWMSetPercent(rearRight, rearRightPW);
  // ------------------------------------------------------------------

}