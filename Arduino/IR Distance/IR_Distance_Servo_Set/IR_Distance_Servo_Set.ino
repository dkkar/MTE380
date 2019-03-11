#include <Servo.h>

#define sensor A0                           // Sharp IR GP2Y0A41SK0F (4-30cm, analog)

Servo myservo;
int pos = 90;
int counter = 0;
float volts = 0;
int distance = 0;

void setup() {
  Serial.begin(9600);                       // start the serial port
  Serial.println("START");

  myservo.attach(8); 
  myservo.write(pos);
  delay(1000);
}

void loop() {
  volts = analogRead(sensor)*0.0048828125;  // value from sensor * (5/1024)
  distance = 13*pow(volts, -1);             // worked out from datasheet graph
  delay(100); // slow down serial port 

  // Prints the distance on the Serial Monitor
  Serial.print(pos-90);
  Serial.print(",");
  if (distance <= 30){
    Serial.println(distance);               // print the distance
  }
  else {
    Serial.println(-1);
  }
}
