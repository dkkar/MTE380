#define FLAME1 53 
#define FLAME2 51 
#define FLAME3 50 
#define FLAME4 52 
#define FAN 49

void setup() {
  Serial.begin(9600);
  pinMode(FLAME1, INPUT);
  pinMode(FLAME2, INPUT);
  pinMode(FLAME3, INPUT);
  pinMode(FLAME4, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(FAN, OUTPUT);

  delay(5000);
}

void loop() {

  int fire1 = digitalRead(FLAME1);
  int fire2 = digitalRead(FLAME2);
  int fire3 = digitalRead(FLAME3);
  int fire4 = digitalRead(FLAME4);


  if( fire1 == LOW || fire2 == LOW || fire3 == LOW || fire4 == LOW)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(FAN, HIGH);
    Serial.println("Fire");

    delay(200);
    
  }else{
    Serial.println("Nothing");
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(FAN, LOW);
  }

  delay(200);
}
