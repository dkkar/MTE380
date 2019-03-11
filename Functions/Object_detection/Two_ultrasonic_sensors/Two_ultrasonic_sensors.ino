#define trigPin1 2                                  
#define echoPin1 3                                  
#define LED_first_ping 22                            

#define trigPin2 4
#define echoPin2 5
#define LED_second_ping 24

long duration, distance, UltraSensor1, UltraSensor2; 

char data;
String SerialData="";

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  // setup pins first sensor
  pinMode(trigPin1, OUTPUT);                      
  pinMode(echoPin1, INPUT);                       
  
  //setup pins second sensor
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

   Serial.begin(9600);
  
}


void loop() {
  SonarSensor(trigPin1, echoPin1);              
  UltraSensor1 = distance;                     
  SonarSensor(trigPin2,echoPin2);              
  UltraSensor2 = distance;                     

  while(Serial.available()){
    delay(10);
    data=Serial.read();
    SerialData+=data;
  }


  Serial.print("distance measured by the first sensor: ");
  Serial.print(UltraSensor1);
  Serial.println(" cm");
  
  Serial.print("distance measured by the second sensor: ");
  Serial.print(UltraSensor2);
  Serial.println(" cm");
  Serial.println("---------------------------------------------------------------------------------------------------------");
  delay(500);

  if (UltraSensor1 <= 20 || UltraSensor2 <=20) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);
  } else {
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (LOW is the voltage level)
  }
 
}

void SonarSensor(int trigPinSensor,int echoPinSensor) {
 
  digitalWrite(trigPinSensor, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinSensor, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinSensor, LOW);

  duration = pulseIn(echoPinSensor, HIGH);
  distance= (duration/2) / 29.1;  
}
