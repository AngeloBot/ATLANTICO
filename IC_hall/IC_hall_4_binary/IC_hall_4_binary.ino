int statusLEFT = 0;
int statusRIGHT = 0;
int statusFLEFT = 0;
int statusFRIGHT = 0;

//pin para input proveniente sensores
int pinLEFT = A1;
int pinRIGHT = A2; 
int pinFLEFT = A0;
int pinFRIGHT = A3;

//pin para output nos leds
int LpinLEFT = 7;
int LpinRIGHT = 4;
int LpinFLEFT = 8;
int LpinFRIGHT = 2;

int status_Hall =B0;
int maxHallSignal=15;

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
    status_Hall |= 1;
    digitalWrite(LpinFRIGHT, HIGH);
    }
  else{

    digitalWrite(LpinFRIGHT, LOW);
    status_Hall &= ~1;
    }
    
    if ( statusRIGHT < maxHallSignal){
    status_Hall |= (1<<1);
    digitalWrite(LpinRIGHT, HIGH);
    }
  else{

    digitalWrite(LpinRIGHT, LOW);
    status_Hall &= ~(1<<1);
    }
    
    if ( statusLEFT < maxHallSignal){
    status_Hall |= (1<<2);
    digitalWrite(LpinLEFT, HIGH);
    }
  else{
    status_Hall &= ~(1<<2);
    digitalWrite(LpinLEFT, LOW);
    }
    if ( statusFLEFT < maxHallSignal){
    status_Hall |= (1<<3);
    digitalWrite(LpinFLEFT, HIGH);
    }
  else{

    digitalWrite(LpinFLEFT, LOW);
    status_Hall &= ~(1<<3);
    }

    Serial.print(statusFLEFT);Serial.print("\t");Serial.print(statusLEFT);Serial.print("\t");Serial.print(statusRIGHT);Serial.print("\t");Serial.println(statusFRIGHT);
    //Serial.println(status_Hall);
  }
