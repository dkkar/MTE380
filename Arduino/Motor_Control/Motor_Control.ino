
#define in1 4
#define in2 5
#define en1 6

#define in3 7
#define in4 8
#define en2 9

int val = 190;

void setup() {
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
}

void loop() {
  analogWrite(en1,val);
  analogWrite(en2,val);

  delay(1000);
}
