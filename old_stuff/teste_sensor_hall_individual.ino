int statusS = 0;

//pin para input proveniente sensores
int pin= 3;

//pin para output nos leds
int Lpin= 2;

void setup() {

  Serial.begin(9600);
  
  //LEFT
  pinMode(pin, INPUT);
  pinMode(Lpin, OUTPUT);

}

void loop() {
  statusS = digitalRead(pin);
  //LEFT  
  if ( statusS == 1){
    
    digitalWrite(Lpin, HIGH);
    }
  else{

    digitalWrite(Lpin, LOW);
    }
  Serial.println(statusS);
}
