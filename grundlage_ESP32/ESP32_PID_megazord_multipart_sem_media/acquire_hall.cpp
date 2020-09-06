
#include <Arduino.h>
#include "def_system.h"
#include "acquire_hall.h"

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
    flag_hall--;
}
