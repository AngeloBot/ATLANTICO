int pass = 0;

// status instant.
int statusFR;
int statusR;
int statusL;
int statusFL;

//pin FAR RIGHT(amarelo)
int pinFR = 10;

//pin RIGHT(verde)
int pinR= 3;

//pin LEFT(amarelo)
int pinL= 5;

//pin FAR LEFT(verde)
int pinFL = 9;

void setup() {

  Serial.begin(9600);
  
  pinMode(pinR, INPUT);
  pinMode(pinL, INPUT);
  pinMode(pinFR, INPUT);
  pinMode(pinFL, INPUT);


}

void loop() {

  pass = 0;
  
  statusR = digitalRead(pinR);
  statusL = digitalRead(pinL);
  statusFR = digitalRead(pinFR);
  statusFL = digitalRead(pinFL);
  
  if( statusR == 0){ 
      if (statusFR == 0){
        Serial.println("Far Right");
        pass = 1;
      }
      else{
        Serial.println("Right");
        pass = 1;
      }
  }
  if( statusL == 1 ){
    if (statusFL == 1){
      Serial.println("Far Left");
      pass = 1;
    }
    else{
      Serial.println("Left");
      pass = 1;
    }
  }
  if (pass == 0){
    Serial.println("Middle");
  }
}
