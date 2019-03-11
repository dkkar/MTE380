#include <TimeLib.h>
bool object_detected = 1;
int commands{3]{2];


void setup() {
  // put your setup code here, to run once:

}

int time() {
  hour = hour();
  minute = minute();
  second = second();

  time = hour*60*60 + minute*60 + second;
  return time;
}

void loop() {
  if (UltraSensor1 < 30 || UltraSensor1 < 30) {
    object_detected = 1;
  } else {
    object_detected = 0;
  }
  
  if (object_detected == 1) {
    if (UltraSensor1 > 5 || UltraSensor1 > 5) {
      time_start = start();
      //DRIVE FORWARD
      time_end = start();

     commands[0][0] = 1;
     commands[0][1] = time_end - time_start;

      delay(500);
    }
    if (UltraSensor1 - UltraSensor2 >= 5) {
      time_start = start();
      //TURN CW
      time_end = start();

     commands[1][0] = 1;
     commands[1][1] = time_end - time_start;
    }
    if (UltraSensor2 - UltraSensor1 >= 5) {
      time_start = start();
      //TURN CCW
      time_end = start();

     commands[2][0] = 0;
     commands[2][1] = time_end - time_start;
    }
  }
  

}
