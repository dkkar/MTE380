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

// Ultrasonic
#define trigPin 11                                  
#define echoPin1 12                                  
#define echoPin2 10

// IMU Vars
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float target_angle_x = 0;


// US Vars
long duration, distance, UltraSensor1, UltraSensor2; 
char data;
bool object_detected = 1;
String SerialData="";
int sampled_distance = 0;

void setup(void) {
  Serial.begin(9600);

  // Motor Setup
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT); 
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);

  // Initial direction (forwards)
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  
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

void stopMotors()
{
  analogWrite(en1, 0);
  analogWrite(en2, 0);
}

/*
 *  bool dir 0 goes forwards, 1 goes backwards
 */
void goStraight(int dir, int counter)
{
  float angle_x = 0;
  float prev_angle_x = 0;
  float error = 0, errorSum = 0;
  float sampleTime = 0.05;
  unsigned long currTime = 0, prevTime = 0;

  int pwm_r = 190;
  int pwm_l = 190;

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  target_angle_x = euler.x();
  
  if (dir == 0)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else if (dir == 1)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }


  for (int i = 0; i < counter; i++)
  {
    currTime = millis();
    sampleTime = (currTime - prevTime)/1000.0;
    prevTime = currTime;
    
    // Motor Movement
    analogWrite(en1,pwm_r);
    analogWrite(en2,pwm_l);
  
    // IMU Data
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
    angle_x = euler.x();
    
    // Note: Clockwise is +ive
    error = angle_x - target_angle_x;
  
    // Condition if angle shifts from 0 to -359...
    if (error > 300) {
      error = error - 360;
    }
    
    if (error < -300) {error = error + 360;}
  
    errorSum = errorSum + error;
    errorSum = constrain(errorSum, -300, 300);

    if (dir == 0)
    {
      pwm_r = pwm_r + ((Kp*error) + (Ki*errorSum*sampleTime) - (Kd*(angle_x - prev_angle_x)/sampleTime));
      pwm_l = pwm_l - ((Kp*error) + (Ki*errorSum*sampleTime) - (Kd*(angle_x - prev_angle_x)/sampleTime));
    }
    else if (dir == 1)
    {
      pwm_r = pwm_r - ((Kp*error) + (Ki*errorSum*sampleTime) - (Kd*(angle_x - prev_angle_x)/sampleTime));
      pwm_l = pwm_l + ((Kp*error) + (Ki*errorSum*sampleTime) - (Kd*(angle_x - prev_angle_x)/sampleTime));
    }
  
    prev_angle_x = angle_x;
  
    pwm_r = constrain(pwm_r, -255, 255);
    pwm_l = constrain(pwm_l, -255, 255);

    delay(50);
  }
  
  stopMotors();
  
}

/*
 * bool dir:  0 is clockwise, 1 is counter-clockwise
 */
void turnAngle(bool dir, unsigned int target_angle)
{
  float angle_x = 0;
  int pwm_r = 250;
  int pwm_l = 250;
  
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

void SonarSensor(int trigPinSensor,int echoPinSensor) {
  pinMode(trigPinSensor, OUTPUT);
  pinMode(echoPinSensor, INPUT);   
 
  digitalWrite(trigPinSensor, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinSensor, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinSensor, LOW);

  duration = pulseIn(echoPinSensor, HIGH);
  distance= (duration/2) / 29.1;  
  delay(250);   
}

void ScanSurroundings() {
                
  UltraSensor1 = DistanceSample(trigPin, echoPin1);   
  delay(2);                  
  UltraSensor2 = DistanceSample(trigPin, echoPin2);                     


  Serial.print("distance measured by the first sensor: ");
  Serial.print(UltraSensor1);
  Serial.println(" cm");
  
  Serial.print("distance measured by the second sensor: ");
  Serial.print(UltraSensor2);
  Serial.println(" cm");
  Serial.println("---------------------------------------------------------------------------------------------------------");
}

int DistanceSample(int trigPinSensor, int echoPinSensor) {
  sampled_distance = 0;
  int readings[3];
  for (int i=0; i < 3; i++) {
    SonarSensor(trigPinSensor, echoPinSensor);
    readings[i] = distance;
  }
  for (int i=0; i < 3; i++) {
    sampled_distance = sampled_distance + readings[i]; 
  }
  return sampled_distance/3;
}

void loop(void) {
  //delay(500);
  
  // Move forward 5 sec
//  goStraight(0, 100);
//  delay(500);
//  goStraight(1, 100);
//  delay(500);
//  turnAngle(0, 90);
//  delay(500);

    ScanSurroundings();
//    goStraight(0, 500);
//    delay(500);
//    goStraight(0, 100);
//    delay(500);
    //goStraight(0, 1);
 
  while (object_detected == 1 && UltraSensor1 > 10 && UltraSensor2 > 10) {
     ScanSurroundings();
//     if (UltraSensor1 <= 50 || UltraSensor2 <= 50) {
//      object_detected = 1;
//    } else {
//      object_detected = 0;
//    }

    if (UltraSensor1 > 10 || UltraSensor2 > 10) {
      ScanSurroundings();
      goStraight(0, 10);
      delay(500);
      turnAngle(0, 10);
      
    } else {
      while (UltraSensor1 - UltraSensor2 >= 10) {
        ScanSurroundings();
        turnAngle(0, 10);
        delay(500);
      }
      while (UltraSensor2 - UltraSensor1 >= 10) {
        ScanSurroundings();
        turnAngle(1, 10);
        delay(500);
      }
    }
  }

  object_detected = 0;
}
