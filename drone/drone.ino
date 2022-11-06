#include <IBusBM.h>
#include "SoftPWM.h"
#include <Wire.h>

// Citation: http://www.brokking.net/imu.html, an open source IMU for drones, was used in this project to calculate the angles from the MPU 6050
// All sections that use this IMU code are labeled

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

// Global variables for angle calculations - from http://www.brokking.net/imu.html
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
int lcd_loop_counter;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;

// --------------------------------------------------------------------

// selfLevel takes pitch and roll
float selfLevel(int motor, float pitch_angle, float roll_angle, int pitch, int roll){
  float multiplier = 1.0;
  float response = 0.02;
  
  // For now, only do anything if pitch and roll are zero
  if (pitch == roll == 0){

    // Level the pitch
    // Front needs power
    if(pitch_angle > 1 && (motor == frontLeft || motor == frontRight)){
      multiplier += response;
    }
    // Rear needs power
    else if(pitch_angle < -1 && (motor == rearLeft || motor == rearRight)){
      multiplier += response;
    }

    // Level the roll
    // Left needs power
    if (roll_angle < -1 && (motor == frontLeft || motor == rearLeft)){
      multiplier += response;
    }
    // Right needs power 
    else if (roll_angle > 1 && (motor == frontRight || motor == frontLeft)){
      multiplier += response;
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

// Setup MPU 6050 Registers - from http://www.brokking.net/imu.html
void setup_mpu_6050_registers(){
  // INTERRUPT!!!
  attachInterrupt(digitalPinToInterrupt(9), read_mpu_6050_data, FALLING);

  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}

// Read MPU Data - from http://www.brokking.net/imu.html
void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable
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


  // MPU SETUP --------------------------------------------------------
  setup_mpu_6050_registers();

  Serial.begin(115200);

  // MPU Calibration from http://www.brokking.net/imu.html
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;   


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
  // // CALCULATE ANGLES - from http://www.brokking.net/imu.html
  // read_mpu_6050_data();                                                //Read the raw acc and gyro data from the MPU-6050
  // Interrupt to read mpu data
  digitalWrite(digitalPinToInterrupt(9), LOW);
  digitalWrite(digitalPinToInterrupt(9), HIGH);

  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
  
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_y * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_x * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_x/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_y/acc_total_vector)* -57.296;       //Calculate the roll angle
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value

  // END CALCULATE ANGLES

  //READ DATA FROM IBUS AND SET PWM ----------------------------------
  
  //CHANNEL 2 THROTTLE, CHANNEL 3 YAW, CHANNEL 0 ROLL, CHANNEL 1 PITCH 
  int throttle = readChannel(2, 0, 80, 0);
  int yaw = readChannel(3, -100, 100, 0);
  int roll = readChannel(0, -100, 100, 0);
  int pitch = readChannel(1, -100, 100, 0);

  // Calculate the pulse widths
  int frontLeftPW = pulseWidth(frontLeft, throttle, roll, pitch, yaw) * selfLevel(frontLeft, angle_pitch_output, angle_roll_output, pitch, roll);
  int rearLeftPW = pulseWidth(rearLeft, throttle, roll, pitch, yaw) * selfLevel(rearLeft, angle_pitch_output, angle_roll_output, pitch, roll);
  int frontRightPW = pulseWidth(frontRight, throttle, roll, pitch, yaw) * selfLevel(frontRight, angle_pitch_output, angle_roll_output, pitch, roll);
  int rearRightPW = pulseWidth(rearRight, throttle, roll, pitch, yaw) * selfLevel(rearRight, angle_pitch_output, angle_roll_output, pitch, roll);

  //Serial.print(frontLeftPW);



  // Update signals
  SoftPWMSetPercent(frontLeft, frontLeftPW);
  SoftPWMSetPercent(rearLeft, rearLeftPW);
  SoftPWMSetPercent(frontRight, frontRightPW);
  SoftPWMSetPercent(rearRight, rearRightPW);
  // ------------------------------------------------------------------

// PRINT DATA FROM MPU -----------------------------------------------
  Serial.print("Roll: ");
  Serial.print(angle_roll_output);
  Serial.println("");
  Serial.print("Pitch: ");
  Serial.print(angle_pitch_output);
  Serial.println("");
  Serial.println("");

  

// ------------------------------------------------------------------

}