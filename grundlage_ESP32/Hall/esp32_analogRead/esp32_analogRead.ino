const int pin = 2;
const int pinLED = 15;

// variable for storing the potentiometer value
int potValue = 0;

void setup() {
  Serial.begin(115200);
  pinMode(pinLED, OUTPUT);
  pinMode(pin,INPUT);

}

void loop() {
  // Reading potentiometer value
  potValue = analogRead(pin);
  Serial.println(potValue);
  
  if(potValue>0){
    
    digitalWrite(pinLED,HIGH);
  }
  else{
    digitalWrite(pinLED,LOW);  
  }
}
