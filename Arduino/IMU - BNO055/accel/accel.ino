// IMU Libs
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


// IMU Vars
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float accel_x = 0, vel_x = 0, pos_x = 0, prev_accel_x = 0, prev_vel_x = 0, prev_pos_x = 0, avg_accel_x = 0, avg_vel_x = 0, avg_pos_x = 0;
float sampleTime = 0.1;
unsigned long currTime = 0, prevTime = 0;
 
void setup(void) 
{
  Serial.begin(9600);

  // IMU Setup
  Serial.println("Orientation Sensor Test"); 
  Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
  
  bno.setExtCrystalUse(true);
}
 
void loop(void) 
{
  currTime = millis();
  sampleTime = (currTime - prevTime)/1000.0;
  prevTime = currTime;

  // IMU Data
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  accel_x = -1*accel.x();
  avg_accel_x = (accel_x + prev_accel_x) / 2;

  vel_x = prev_vel_x + (avg_accel_x * sampleTime);
  avg_vel_x = (vel_x + prev_vel_x) / 2;

  pos_x = prev_pos_x + (avg_vel_x * sampleTime);
  avg_pos_x = (pos_x + prev_pos_x) / 2;

  prev_accel_x = accel_x;
  prev_vel_x = vel_x;
  prev_pos_x = pos_x;

//  /* Display the floating point data */
//  Serial.print("Accel X: ");
//  Serial.print(avg_accel_x);
//  Serial.print(" , Vel X: ");
//  Serial.print(avg_vel_x);
//  Serial.print(" , Pos X: ");
//  Serial.print(avg_pos_x);
  Serial.print(" , Sample Time: ");
  Serial.println(sampleTime);
}
