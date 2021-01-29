

//int LED_BUILTIN = 13;

void setup() {
pinMode (LED_BUILTIN, OUTPUT);
Serial.begin(9600);
}

void loop() {
digitalWrite(LED_BUILTIN, HIGH);
Serial.println("ON");
delay(1000);
digitalWrite(LED_BUILTIN, LOW);
Serial.println("OFF");
delay(1000);


}
