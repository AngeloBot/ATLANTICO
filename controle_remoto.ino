#include <Servo.h>;

const int pinS=22;  // CHANGE 22 TO THE NUMBER OF PIN YOU ARE USING!!!!   Digital pins connected to the radio receiver ports
const int pin2=30;
//Make sure that pin1 is hooked up to the throttle port on the receiver (port 2 on 9x)
//and that pin2 is hooked up to the rudder port on the receiver (port 4 on 9x)

const int maxval=1870;
const int minval=1050;

//variaveis para margem de posição neutra
const int margemn=20;

//constantes para valores extremantes da ampltude de operação do servo
const int servoh=100;
const int servol=-100;


int neutralh = (((maxval-minval)/2) + margemn);
int neutrall = (((maxval-minval)/2) - margemn);
Servo servo;
int valor; 
float delta;

float range(servoh, servol, valor, maxval, minval){ 
  delta = (valor-minval)/maxval;
  ouput = delta*servoh+servol;
  return output;
}

void setup() {
  
  Serial.begin(9600);           // set up Serial monitor at 9600 bps

  pinMode(pinS, INPUT); //Setting up RC input pins

  Servo.attach(pinS);
}
void loop() {

  int Value = pulseIn(pin1, HIGH); //Getting the pulse in value from pin 1 and defining pin1 as direction

  //Serial.println(Value);
  //This can be printed for debugging


 

    if (Value >= neutrall && Value <= neutralh) {     //If the sticks are Neutral, then stay still

      servo.write(0);    //releases motors
      
      delay(50);            //checks again every 50 micro Seconds
    }


    if (Value > neutralh or Valeu < neutrall) {
      servo.write(range(servoh, servol, Value, maxval, minval));

      delay(50);
      
      
     
    }


    
}
