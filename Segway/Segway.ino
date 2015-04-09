#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>              //Adafruit motoshield library
#include "utility/Adafruit_PWMServoDriver.h"   //Required for motorshield use
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>


//Define Variables
double Setpoint, Input, Output;
sensors_event_t accel, mag, gyro, temp; //Specify the links and initial tuning parameters

//Adafruit motorshield
Adafruit_MotorShield motorShield = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = motorShield.getMotor(2);
Adafruit_DCMotor *rightMotor = motorShield.getMotor(3);

/* Assign a unique base ID for this sensor */   
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
////
//PI
double last_error_one = 0;
double last_error_two = 0;

double error_one = 0;
double output_one = 0;
double integrated_error_one = 0;
double pTerm_one = 0, iTerm_one = 0, dTerm_one = 0;

double error_two = 0;
double output_two = 0;
double integrated_error_two = 0;
double pTerm_two = 0, iTerm_two = 0, dTerm_two = 0;


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
//Adafruit motors
motorShield.begin();

}

void loop()
{
  currentMillis = millis();
  lsm.getEvent(&accel, &mag, &gyro, &temp);

  double magG = sqrt(pow((float)accel.acceleration.y, 2) + pow((float)accel.acceleration.z, 2));
  double angle = (((float)accel.acceleration.y)*10)/magG;

  if(angle > 6.0 | angle < -6.0) {
    while(1) {
      leftMotor->run(RELEASE);
      rightMotor->run(RELEASE);
      Serial.println(" I fell, Help me up.");
    }
  }
  
  error_one = angle - 0.29;

  pTerm_one = (5*error_one);
  integrated_error_one += error_one;
  iTerm_one = constrain((0.32*integrated_error_one), -20,20);
  dTerm_one = 9.8*(error_one - last_error_one);
  last_error_one = error_one;
  output_one = constrain(4*(pTerm_one + iTerm_one + dTerm_one), -255, 255);

  // error_two = (output_one + gyro.gyro.x) - 0.0;
  // pTerm_two = 3.2 * error_two;
  // dTerm_two = 0 * (error_two - last_error_two);
  // last_error_two = error_two;
  // output_two = constrain(1*(pTerm_two + iTerm_two + dTerm_two), -255, 255);

  move(output_one);


  //Serial.print(angle); Serial.print(" ");  Serial.print(gyro.gyro.x);Serial.print(" ");  Serial.print(gyro.gyro.y);Serial.print(" "); Serial.println(gyro.gyro.z);
  //Serial.print(" "); Serial.print (magG); Serial.print(" "); Serial.println(output);
  //Serial.print(" "); Serial.print(pTerm);  Serial.print(" "); Serial.print(iTerm); Serial.print(" "); Serial.println(dTerm);
  Serial.println(output_two);


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
void move(double output)
{
  //Serial.println(output);
  leftMotor->setSpeed(abs(output));
  rightMotor->setSpeed(abs(output));

  if(output < 0.0)
  {
    leftMotor->run(FORWARD);
    rightMotor->run(FORWARD);
    //Serial.print(accel.acceleration.z); Serial.print(" "); Serial.println("Backwards");
  } 
  else if(output > 0.0) 
  {
    leftMotor->run(BACKWARD);
    rightMotor->run(BACKWARD);
    //Serial.print(accel.acceleration.z); Serial.print(" "); Serial.println("Forwards");
  }
}


