int statusLEFT = 0;
int statusRIGHT = 0;
int statusMID = 0;

//pin para input proveniente sensores
int pinLEFT = 3;
int pinRIGHT = 5; 
int pinMID = 6;

//pin para output nos leds
int LpinLEFT = 2;
int LpinRIGHT = 4;
int LpinMID = 7;

void setup() {
  
  Serial.begin(9600);
  
  //RIGHT
  pinMode(pinRIGHT, INPUT);
  pinMode(LpinRIGHT, OUTPUT);
  //LEFT
  pinMode(pinLEFT, INPUT);
  pinMode(LpinLEFT, OUTPUT);
  //MID
  pinMode(pinMID, INPUT);
  pinMode(LpinMID, OUTPUT);
}

void loop() {
  statusRIGHT = digitalRead(pinRIGHT);
  statusLEFT = digitalRead(pinLEFT);
  statusMID = digitalRead(pinMID);
  
//RIGHT
  if ( statusRIGHT == 1){
    
    digitalWrite(LpinRIGHT, HIGH);
    }
  else{

    digitalWrite(LpinRIGHT, LOW);
    }
  Serial.print("RIGHT");
  Serial.println(statusRIGHT);
    
//LEFT  
  if ( statusLEFT == 1){
    
    digitalWrite(LpinLEFT, HIGH);
    }
  else{

    digitalWrite(LpinLEFT, LOW);
    }
  Serial.print("LEFT");
  Serial.println(statusLEFT);
  
//MID
  if ( statusMID == 1){
    
    digitalWrite(LpinMID, HIGH);
    }
  else{

    digitalWrite(LpinMID, LOW);
    }
  Serial.print("MID");
  Serial.println(statusMID);
}
