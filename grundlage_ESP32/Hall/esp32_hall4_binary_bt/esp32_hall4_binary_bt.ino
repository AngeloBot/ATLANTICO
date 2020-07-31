int statusLEFT = 0;
int statusRIGHT = 0;
int statusFLEFT = 0;
int statusFRIGHT = 0;

//pin para input proveniente sensores
int pinLEFT = 34;
int pinRIGHT = 35;
int pinFLEFT = 32;
int pinFRIGHT = 33;

//pin para output nos leds
int LpinLEFT = 27;
int LpinRIGHT = 25;
int LpinFLEFT = 12;
int LpinFRIGHT = 4;

int status_Hall =B0;
int maxHallSignal=10;

//configuração do bluetooth
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

void setup() {
  
    Serial.begin(115200);

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

    //iniciar bluetooth definindo nome do dispositivo
    SerialBT.begin("ESP32_veleiro_autonomo");

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
    SerialBT.print(statusFLEFT);SerialBT.print("\t");SerialBT.print(statusLEFT);SerialBT.print("\t");SerialBT.print(statusRIGHT);SerialBT.print("\t");SerialBT.println(statusFRIGHT);
    //Serial.println(status_Hall);
  }
