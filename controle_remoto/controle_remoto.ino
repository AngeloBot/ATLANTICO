#include <Servo.h>
Servo servo;
int pos;
int sinal;

int min_PWM =1980;
int max_PWM =997;


void setup() {

Serial.begin(9600);
servo.attach(5);
pos = 90;
servo.write(pos);

}

void loop() {
  
sinal = pulseIn(3,HIGH);
pos = map(sinal,max_PWM,min_PWM,140, 50);
servo.write(pos);
Serial.println(pos);
delay(100);

}
