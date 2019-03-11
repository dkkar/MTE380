const int pingPin1 = 7; // Trigger Pin of Ultrasonic Sensor
const int echoPin1 = 6; // Echo Pin of Ultrasonic Sensor
const int pingPin2 = 5; // Trigger Pin of Ultrasonic Sensor
const int echoPin2 = 4; // Echo Pin of Ultrasonic Sensor

void setup() {
   Serial.begin(9600); // Starting Serial Terminal
}

void loop() {
   long duration1, duration2, inches, cm1, cm2;
   
   pinMode(pingPin1, OUTPUT);
   digitalWrite(pingPin1, LOW);
   pinMode(pingPin2, OUTPUT);
   digitalWrite(pingPin2, LOW);
   delayMicroseconds(2);
   
   digitalWrite(pingPin1, HIGH);
   digitalWrite(pingPin2, HIGH);
   delayMicroseconds(10);
   
   digitalWrite(pingPin1, LOW);
   digitalWrite(pingPin2, LOW);
   pinMode(echoPin1, INPUT);
   pinMode(echoPin2, INPUT);
   
   duration1 = pulseIn(echoPin1, HIGH);
   duration2 = pulseIn(echoPin2, HIGH);
   //inches = microsecondsToInches(duration);
   cm1 = microsecondsToCentimeters(duration1);
   cm2 = microsecondsToCentimeters(duration2);
   
   Serial.print(cm1);
   Serial.print("cm1");
   Serial.print(cm2);
   Serial.print("cm2");
   Serial.println();
   delay(100);
}

long microsecondsToInches(long microseconds) {
   return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}
