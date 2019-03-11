#define trigPin 2                                  
#define echoPin1 3                                  
#define echoPin2 4

long duration, distance, UltraSensor1, UltraSensor2; 

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  // setup pins first sensor
//  pinMode(trigPin1, OUTPUT);                      
//  pinMode(echoPin1, INPUT);                       
//  
//  //setup pins second sensor
//  pinMode(trigPin2, OUTPUT);
//  pinMode(echoPin2, INPUT);

   Serial.begin(9600);
  
}


void loop() {
  SonarSensor(trigPin, echoPin1);              
  UltraSensor1 = distance;                     
  SonarSensor(trigPin,echoPin2);              
  UltraSensor2 = distance;                     


  Serial.print("distance measured by the first sensor: ");
  Serial.print(UltraSensor1);
  Serial.println(" cm");
  
  Serial.print("distance measured by the second sensor: ");
  Serial.print(UltraSensor2);
  Serial.println(" cm");
  Serial.println("---------------------------------------------------------------------------------------------------------");
  
  delay(500); 
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
  delay(1000); 
}
