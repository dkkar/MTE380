#include <Servo.h>

// defines pins numbers
const int trigPin = 9;
const int echoPin = 10;

// defines variables
long duration;
int distance;

Servo myservo;
int pos = 10;
int counter = 0;

void setup() 
{
Serial.println("START");

pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPin, INPUT); // Sets the echoPin as an Input
Serial.begin(9600); // Starts the serial communication
myservo.attach(8);
}

void loop() 
{

myservo.write(pos);
delay(1000);

while(counter < 100)
{
  counter++;
  // in steps of 1 degree
  myservo.write(pos);              // tell servo to go to position in variable 'pos'
  
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration*0.034/2;
  // Prints the distance on the Serial Monitor
  Serial.print(pos-10);
  Serial.print(",");
  Serial.println(distance);
}

counter = 0;
pos += 1;

if (pos == 101)
{
  while(true); 
}
}
