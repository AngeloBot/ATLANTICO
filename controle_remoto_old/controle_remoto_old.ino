#include <Servo.h>

int pinS = 22;    // CHANGE 22 TO THE NUMBER OF PIN YOU ARE USING!!!!   Digital pins connected to the radio receiver ports
int pin2 = 30;
//Make sure that pin1 is hooked up to the throttle port on the receiver (port 2 on 9x)
//and that pin2 is hooked up to the rudder port on the receiver (port 4 on 9x)

int maxval = 1870; //PUT YOUR MAX VALUE HERE
int minval = 1050; // PUT YOUR MIN VALUE HERE

//variaveis para margem de posição neutra
int neutralh = 1420;
int neutrall = 1500;

Servo servo;

#define range(servosh, servol);


void setup() {
  
  Serial.begin(9600);           // set up Serial monitor at 9600 bps

  pinMode(pinS, INPUT); //Setting up RC input pins

  Servo.attach(pinS);
}

void loop() {

  int Value = pulseIn(pin1, HIGH); //Getting the pulse in value from pin 1 and defining pin1 as direction

  Serial.println(speedValue);
  //This can be printed for debugging


 

    if (Value >= neutrall && Value <= neutralh) {     //If the sticks are Neutral, then stay still

      servo.write(0);    //releases motors
      
      delay(50);            //checks again every 50 micro Seconds
    }


    if (Value > neutralh) {

      
      
     
    }


    
}
