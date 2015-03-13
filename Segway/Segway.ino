#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>              //Adafruit motoshield library
#include "utility/Adafruit_PWMServoDriver.h"   //Required for motorshield use

//Define Variables
double Setpoint, Input, Output;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,0.1,10,0, DIRECT);

//Adafruit motorshield
Adafruit_MotorShield motorShield = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = motorShield.getMotor(3);
Adafruit_DCMotor *rightMotor = motorShield.getMotor(4);

//Constant
const int MPU=0x68;    //I2C address of the MPU

void setup()
{
  //PID
  Input = AcX;
  Setpoint = 127;
  myPID.SetMode(AUTOMATIC); //turn the PID on
  
  //MPU Communications
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); //Power managment 1 register (PWR_MGMT_1)
  Wire.write(0);    //Wakes up the MPU
  Wire.endTransmission(true);
  
  //Adafruit motors
  motorShield.begin();
  
  Serial.begin(9600);
}

void loop()
{
   get_gyro();
   Input = AcX;
   myPID.Compute();
   move(Output); 
}

//Function reads registers in MPU and assigned them to variables.
//AcX, Y, and Z can be seen as "position"
//GyX, Y, and Z are the speed in which the segway is falling
//Refer to the MPU6050 code found online.
void get_gyro()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14, true); //Requesting a total of 14 registers.
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  

}

//Motor movement fucntion
void move(int output)
{
  //Check direction of output
  //if output is negative (less than zero) Robot is falling forward.
  //GO MOTORS FORWARD.
  //if output is positive (bigger than zero) Robot is falling backwards.
  //GO MOTORS BACKWARDS.

  leftMotor->setSpeed(abs(output));
  rightMotor->setSpeed(abs(output));

  if(output > 0){
    leftMotor->run(BACKWARD);
    rightMotor->run(BACKWARD);
  } else if(output < 0) {
    leftMotor->run(FORWARD);
    rightMotor->run(FORWARD);
  }
}
