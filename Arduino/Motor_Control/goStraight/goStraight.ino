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

#define Kp 4
#define Ki 2
#define Kd 0.01

// Motor Vars
int pwm_r = 190;
int pwm_l = 190;

// IMU Vars
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float angle_x = 0, prev_angle_x = 0, target_angle_x = 0;
float accel_x = 0, vel_x = 0, pos_x = 0, prev_accel_x = 0, prev_vel_x = 0, prev_pos_x = 0, avg_accel_x = 0, avg_vel_x = 0, avg_pos_x = 0;
float error = 0, errorSum = 0;
float sampleTime = 0.1;
unsigned long currTime = 0, prevTime = 0;
 
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

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  target_angle_x = euler.x();
}


void goStraight(int counter)
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  for (int i = 0; i < counter; i++)
  {
    
  }
  
}

void loop(void) 
{
  currTime = millis();
  sampleTime = (currTime - prevTime)/1000.0;
  prevTime = currTime;
  
  // Motor Movement
  analogWrite(en1,pwm_r);
  analogWrite(en2,pwm_l);

  // IMU Data
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  angle_x = euler.x();

  accel_x = -1*accel.x();
  avg_accel_x = (accel_x + prev_accel_x) / 2;

  vel_x = prev_vel_x + (avg_accel_x * sampleTime);
  avg_vel_x = (vel_x + prev_vel_x) / 2;

  pos_x = prev_pos_x + (avg_vel_x * sampleTime);
  avg_pos_x = (pos_x + prev_pos_x) / 2;
  
  // Note: Clockwise is +ive
  error = angle_x - target_angle_x;

  // Condition if angle shifts from 0 to -359...
  if (error > 300) {error = error - 360;}
  if (error < -300) {error = error + 360;}

  errorSum = errorSum + error;
  errorSum = constrain(errorSum, -300, 300);

//  pwm_r = pwm_r + ((Kp*error) + (Ki*errorSum*sampleTime) - (Kd*(angle_x - prev_angle_x)/sampleTime));
//  pwm_l = pwm_l - ((Kp*error) + (Ki*errorSum*sampleTime) - (Kd*(angle_x - prev_angle_x)/sampleTime));

  pwm_r = pwm_r + ((Kp*error) + (Ki*errorSum*sampleTime) - (Kd*(angle_x - prev_angle_x)/sampleTime));
  pwm_l = pwm_l - ((Kp*error) + (Ki*errorSum*sampleTime) - (Kd*(angle_x - prev_angle_x)/sampleTime));


  prev_angle_x = angle_x;
  prev_accel_x = accel_x;
  prev_vel_x = vel_x;
  prev_pos_x = pos_x;

  pwm_r = constrain(pwm_r, -255, 255);
  pwm_l = constrain(pwm_l, -255, 255);

  if (avg_pos_x > 0.3) {while(true);}
  
  /* Display the floating point data */
//  Serial.print("X: ");
//  Serial.print(angle_x);
//  Serial.print(" , Error: ");
//  Serial.print(error);
//  Serial.print(" , PWM Right: ");
//  Serial.print(pwm_r);
//  Serial.print(" , PWM Left: ");
//  Serial.print(pwm_l);
//  Serial.print(" , Accel X: ");
//  Serial.print(avg_accel_x);
//  Serial.print(" , Vel X: ");
//  Serial.print(avg_vel_x);
//  Serial.print(" , Pos X: ");
//  Serial.println(avg_pos_x);
//  Serial.print(" , Sample Time: ");
//  Serial.println(sampleTime);
  delay(50);
}
