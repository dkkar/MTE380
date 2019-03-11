// IMU Libs
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Motor Definitoions
// Right Motors
#define in1 4
#define in2 5
#define en1 6

// Left Motors
#define in3 7
#define in4 8
#define en2 9

// Motor Vars
int pwm_r = 140;
int pwm_l = 160;

// IMU Vars
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float angle_x = 0;
 
void setup(void) 
{
  Serial.begin(9600);

  // Motor Setup
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);

  // Initial rotation direction (forwards)
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  // IMU Setup
  Serial.println("Rotation Test");
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

/*
 * bool dir:  0 is clockwise, 1 is counter-clockwise
 */
void turnAngle(bool dir, unsigned int target_angle)
{
  // Did this so I can make angle negative but no one is allowed to input target_angle to be negative
  int angle = target_angle;
  
  // Calibration
  angle = angle - 5;  // Accounts for the momentum to stop
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  angle_x = euler.x();
  
  // Setting Direction
  if (dir == 0)
  {
    angle = angle_x + angle;
    // Check for > 359
    if (angle > 359) {angle = angle - 360;}
    
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
      
    analogWrite(en1, pwm_r);
    analogWrite(en2, pwm_l);

    // For example, from 350 to 80 when going positive direction, go from 350 to 360, then 360=0, then 0 to 80 later
    while (angle_x - angle > 0)
    {
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      angle_x = euler.x();

      Serial.print("Target: ");
      Serial.print(angle);
      Serial.print("   ");
      Serial.print("Angle (case 1): ");
      Serial.println(angle_x);
    }
  
    while (angle_x < angle)
    {
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      angle_x = euler.x();

      Serial.print("Target: ");
      Serial.print(angle);
      Serial.print("   ");
      Serial.print("Angle (case 2): ");
      Serial.println(angle_x);
    }
  }

  else if (dir == 1)
  {
    angle = angle_x - angle;
    // Check for < 0
    if (angle < 0) {angle = angle + 360;}
    
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    analogWrite(en1, pwm_r);
    analogWrite(en2, pwm_l);

    // For example, from 80 to 350 when going negative direction, go from 80 to 0, then 0=360, then 360 to 350 later
    while (angle_x - angle < 0)
    {
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      angle_x = euler.x();
      
      Serial.print("Target: ");
      Serial.print(angle);
      Serial.print("   ");
      Serial.print("Angle (case 1): ");
      Serial.println(angle_x);
    }
  
    while (angle_x > angle)
    {
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      angle_x = euler.x();
      
      Serial.print("Target: ");
      Serial.print(angle);
      Serial.print("   ");
      Serial.print("Angle (case 2): ");
      Serial.println(angle_x);
    }
  }

  analogWrite(en1, 0);
  analogWrite(en2, 0);

}
 
void loop(void)
{
  delay(500);
  Serial.println("Test 1");
  turnAngle(1, 45);
  delay(500);
  Serial.println("Test 2");
  turnAngle(0, 90);
  delay(500);
  Serial.println("Test 3");
  turnAngle(1, 45);
  
  while (true);
}
