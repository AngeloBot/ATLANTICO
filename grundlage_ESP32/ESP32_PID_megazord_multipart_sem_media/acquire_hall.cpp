
#include <Arduino.h>
#include 'def_system.h'
#include 'acquire_hall.h'

void acquire_hall(void){
    statusRIGHT = analogRead(pinRIGHT);
    statusLEFT = analogRead(pinLEFT);
    statusFRIGHT = analogRead(pinFRIGHT);
    statusFLEFT = analogRead(pinFLEFT);

    if ( statusFRIGHT < maxHallSignal){
        status_Hall |= 1;
    }
    else{
        status_Hall &= ~1;
    }
    if ( statusRIGHT < maxHallSignal){
        status_Hall |= (1<<1);
    }
    else{
        status_Hall &= ~(1<<1);
    }
    if ( statusLEFT < maxHallSignal){
        status_Hall |= (1<<2);
    }
    else{
        status_Hall &= ~(1<<2);
    }
    if ( statusFLEFT < maxHallSignal){
        status_Hall |= (1<<3);
    }
    else{
        status_Hall &= ~(1<<3);
    }

    Serial.print("HALL=");
    switch(status_Hall){
      
      case 0:
      Serial.println("0000");
      break;
      
      case 1:
      Serial.println("0001");
      break;

      
      case 2:
      Serial.println("0010");
      break;

      
      case 3:
      Serial.println("0011");
      break;

      
      case 4:
      Serial.println("0100");
      break;

      
      case 6:
      Serial.println("0110");
      break;

      case 8:
      Serial.println("1000");
      break;

      case 12:
      Serial.println("1100");
      break;

    }
    //Serial.print("HALL=");Serial.println(status_Hall);
    flag_hall--;

}