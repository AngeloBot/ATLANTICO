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
  
//RIGHT
  if ( statusRIGHT < 10){
    
    digitalWrite(LpinRIGHT, HIGH);
    }
  else{

    digitalWrite(LpinRIGHT, LOW);
    }
  Serial.print("RIGHT");
  Serial.println(statusRIGHT);
    
//LEFT  
  if ( statusLEFT < 10){
    
    digitalWrite(LpinLEFT, HIGH);
    }
  else{

    digitalWrite(LpinLEFT, LOW);
    }
  Serial.print("LEFT");
  Serial.println(statusLEFT);
  
//FRIGHT
  if ( statusFRIGHT < 10){
    
    digitalWrite(LpinFRIGHT, HIGH);
    }
  else{

    digitalWrite(LpinFRIGHT, LOW);
    }
  Serial.print("FRIGHT");
  Serial.println(statusFRIGHT);

  //FLEFT
  if ( statusFLEFT < 10){
    
    digitalWrite(LpinFLEFT, HIGH);
    }
  else{

    digitalWrite(LpinFLEFT, LOW);
    }
  Serial.print("FLEFT");
  Serial.println(statusFLEFT);
}
