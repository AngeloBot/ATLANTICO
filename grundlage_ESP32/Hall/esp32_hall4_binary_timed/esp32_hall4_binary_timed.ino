/* Copyright (c) 2017 pcbreflux. All Rights Reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>. *
 * 
 */
hw_timer_t * timer = NULL;

volatile int state=0;
int LED_BUILTIN = 2;

int statusLEFT = 0;
int statusRIGHT = 0;
int statusFLEFT = 0;
int statusFRIGHT = 0;

//pin para input proveniente sensores
int pinLEFT = 14;
int pinRIGHT = 26; 
int pinFLEFT = 13;
int pinFRIGHT = 15;

//pin para output nos leds
int LpinLEFT = 27;
int LpinRIGHT = 25;
int LpinFLEFT = 12;
int LpinFRIGHT = 4;

int status_Hall =B0;
int maxHallSignal=10;

volatile int counter=0;

void IRAM_ATTR onTimer(){
  

    if(state==0){
        digitalWrite(LED_BUILTIN, HIGH);
        }
    else{
        digitalWrite(LED_BUILTIN, LOW);
        }

    state=~state;
    statusRIGHT = analogRead(pinRIGHT);
    statusLEFT = analogRead(pinLEFT);
    statusFRIGHT = analogRead(pinFRIGHT);
    statusFLEFT = analogRead(pinFLEFT);

    counter++;

}

void setup() {
    Serial.begin(115200);
    pinMode (LED_BUILTIN, OUTPUT);

    Serial.println("start timer ");
    timer = timerBegin(0, 80, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
    timerAttachInterrupt(timer, &onTimer, true); // edge (not level) triggered 
    timerAlarmWrite(timer, 100000, true); // 100000 * 1 us = 0.1 s (10Hz), autoreload true
    timerAlarmEnable(timer); // enable
    
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
  // nope nothing here
  
  if(counter>0){

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
    
    Serial.println(status_Hall);
    counter--;
  }  
}
