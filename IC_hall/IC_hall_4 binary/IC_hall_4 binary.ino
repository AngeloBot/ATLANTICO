int statusLEFT = 0;
int statusRIGHT = 0;
int statusFLEFT = 0;
int statusFRIGHT = 0;

//pin para input proveniente sensores
int pinLEFT = A2;
int pinRIGHT = A1; 
int pinFLEFT = A3;
int pinFRIGHT = A0;

//pin para output nos leds
int LpinLEFT = 4;
int LpinRIGHT = 7;
int LpinFLEFT = 2;
int LpinFRIGHT = 8;

int status_Hall =B0;
int maxHallSignal=10;

void setup() {
  
  Serial.begin(9600);
  
  //RIGHT
  pinMode(pinRIGHT, INPUT);
  pinMode(LpinRIGHT, OUTPUT);
  //LEFT
  pinMode(pinLEFT, INPUT);
  pinMode(LpinLEFT, OUTPUT);
  //FLEFT
  pinMode(pinFLEFT, INPUT);
  pinMode(LpinFLEFT, OUTPUT);
  //FRIGHT
  pinMode(pinFRIGHT, INPUT);
  pinMode(LpinFRIGHT, OUTPUT);
}

void loop() {
  statusRIGHT = analogRead(pinRIGHT);
  statusLEFT = analogRead(pinLEFT);
  statusFRIGHT = analogRead(pinFRIGHT);
  statusFLEFT = analogRead(pinFLEFT);

  if ( statusFRIGHT < maxHallSignal){
    status_Hall |= (1<<3);
    }
  if ( statusRIGHT < maxHallSignal){
    status_Hall |= (1<<2);
    }
  if ( statusFLEFT < maxHallSignal){
    status_Hall |= (1<<1);
    }
  if ( statusLEFT < maxHallSignal){
    status_Hall |= (1<<0);
    }
  }