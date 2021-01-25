
#include <Servo.h>

Servo servo;

void setup() {
  // put your setup code here, to run once:
  servo.attach(9);
  servo.write(90);
}

void loop() {
  // put your main code here, to run repeatedly:
  /*
  int pos;
  for(pos = 0; pos < 150; pos += 1) { // goes from 0 degrees to 180 degrees, 1 degree steps
    servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for(pos = 150; pos>=1; pos-=1) {   // goes from 180 degrees to 0 degrees
    servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  */
}
