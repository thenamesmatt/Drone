#include <IBusBM.h>
#include "SoftPWM.h"

//GLOBAL VARIABLES FOR READING IBUSX ---------------------------------
IBusBM ibusRc;
HardwareSerial& ibusRcSerial = Serial1;
//--------------------------------------------------------------------
 
//GLOBAL VARIABLES FOR MOTORS ----------------------------------------
int frontLeft  = 11;
int frontRight = 13;
int rearLeft   =  5;
int rearRight  = 12; 
//--------------------------------------------------------------------

void setup() {
  // IBUS SETUP --------------------------------------------------------
  ibusRc.begin(ibusRcSerial);
  //--------------------------------------------------------------------


  // PWM SETUP ---------------------------------------------------------
  SoftPWMBegin();
  SoftPWMSet(frontLeft,  0);
  SoftPWMSet(frontRight, 0);
  SoftPWMSet(rearLeft ,  0);
  SoftPWMSet(rearRight,  0);
  //--------------------------------------------------------------------


  //MPU SETUP

  //--------------------------------------------------------------------


}

//READS DATA FROM THE IBUS -------------------------------------------
int readChannel(byte channelInput, int minLimit, int  maxLimit, int defaultValue){
  uint16_t ch = ibusRc.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}
//--------------------------------------------------------------------


void loop() {
  //READ DATA FROM IBUS AND SET PWM --------------------------------------
  //CHANNLE 2 THROTTLE, CHANNEL 3 YAW, CHANNEL 0 ROLL, CHANNEL 1 PITCH 
  int value = readChannel(2, 0, 100, 0);
  Serial.println(value);
  SoftPWMSetPercent(frontLeft,  value);
  SoftPWMSetPercent(rearLeft,   value);
  SoftPWMSetPercent(frontRight, value);
  SoftPWMSetPercent(rearRight , value);
  //--------------------------------------------------------------------


}