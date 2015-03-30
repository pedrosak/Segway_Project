#include <PID_v1.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>              //Adafruit motoshield library
#include "utility/Adafruit_PWMServoDriver.h"   //Required for motorshield use
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>


//Define Variables
double Setpoint, Input, Output;
sensors_event_t accel, mag, gyro, temp;
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,1,0,0, DIRECT);

//Adafruit motorshield
Adafruit_MotorShield motorShield = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = motorShield.getMotor(2);
Adafruit_DCMotor *rightMotor = motorShield.getMotor(4);

/* Assign a unique base ID for this sensor */   
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000

void setup()
{
 while (!Serial);  // wait for flora/leonardo
 
 Serial.begin(9600);
 Serial.println(F("LSM9DS0 9DOF Sensor Test")); Serial.println("");
 
 /* Initialise the sensor */
 if(!lsm.begin())
 {
  /* There was a problem detecting the LSM9DS0 ... check your connections */
  Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
  while(1);
}
Serial.println(F("Found LSM9DS0 9DOF"));

/* Display some basic information on this sensor */
displaySensorDetails();

/* Setup the sensor gain and integration time */
configureSensor();

/* We're ready to go! */
Serial.println("");

//PID
Input = fabs((accel.acceleration.y) * ((gyro.gyro.x)/10));
Setpoint = 0.3;
myPID.SetMode(AUTOMATIC); //turn the PID on

//Adafruit motors
motorShield.begin();
}

void loop()
{
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  Input = fabs((accel.acceleration.y) * ((gyro.gyro.x)/10));
  //Serial.print(Input); Serial.print(" "); Serial.println(Output);
  myPID.Compute();
  move(Output, accel.acceleration.z);
}

void displaySensorDetails(void)
{
  sensor_t accel, mag, gyro, temp;

  lsm.getSensor(&accel, &mag, &gyro, &temp);

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(accel.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(accel.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(accel.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(accel.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(accel.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(accel.resolution); Serial.println(F(" m/s^2"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(mag.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(mag.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(mag.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(mag.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(mag.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(mag.resolution); Serial.println(F(" uT"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(gyro.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(gyro.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(gyro.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(gyro.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(gyro.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(gyro.resolution); Serial.println(F(" rad/s"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(temp.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(temp.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(temp.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(temp.max_value); Serial.println(F(" C"));
  Serial.print  (F("Min Value:    ")); Serial.print(temp.min_value); Serial.println(F(" C"));
  Serial.print  (F("Resolution:   ")); Serial.print(temp.resolution); Serial.println(F(" C"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
}

void configureSensor(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}



//Motor movement fucntion
void move(double output, int dir)
{
  //Serial.println(output);
  leftMotor->setSpeed(fabs(output));
  rightMotor->setSpeed(fabs(output));

  if( dir < 0.0)
  {
    leftMotor->run(BACKWARD);
    rightMotor->run(BACKWARD);

    Serial.print(accel.acceleration.z); Serial.print(" "); Serial.println("Backwards");
  } 
  else if(dir > 0.0) 
  {
    leftMotor->run(FORWARD);
    rightMotor->run(FORWARD);
    Serial.print(accel.acceleration.z); Serial.print(" "); Serial.println("Forwards");
  }
  else if (dir == -0.0)
  {
    // leftMotor->run(RELEASE);
    // rightMotor->run(RELEASE);
    // Serial.println("BACKWARD and dir == -0.0");
    leftMotor->setSpeed(0);
    rightMotor->setSpeed(0);
  }
}
