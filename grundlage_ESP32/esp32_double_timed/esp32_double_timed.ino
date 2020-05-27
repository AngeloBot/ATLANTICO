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
hw_timer_t * timer0 = NULL;
hw_timer_t * timer1 = NULL;

volatile int state=0;
int LED_BUILTIN = 2;
int LED=4;

volatile int counter=0;

void IRAM_ATTR onTimer0(){
  

  if(state==0){
    digitalWrite(LED_BUILTIN, HIGH);
    }
  else{
    digitalWrite(LED_BUILTIN, LOW);
    }

  state=~state;

}

void IRAM_ATTR onTimer1(){
  

    if(counter==0){
        digitalWrite(LED, HIGH);
        }
    else{
        digitalWrite(LED, LOW);
        }

    counter=~counter;

}

void setup() {
    Serial.begin(115200);
    pinMode (LED_BUILTIN, OUTPUT);
    pinMode (LED, OUTPUT);

    Serial.println("start timers ");
    timer0 = timerBegin(0, 80, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
    timerAttachInterrupt(timer0, &onTimer0, true); // edge (not level) triggered 
    timerAlarmWrite(timer0, 1000000, true); // 1000000 * 1 us = 1 s (1Hz), autoreload true
    timerAlarmEnable(timer0); // enable

    timer1 = timerBegin(1, 800, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 800 -> 10000 ns = 10 us, countUp
    timerAttachInterrupt(timer1, &onTimer1, true); // edge (not level) triggered 
    timerAlarmWrite(timer1, 500000, true); // 500000 * 10 us = 5 s (0.2Hz), autoreload true
    timerAlarmEnable(timer1); // enable

}

void loop() {
  // nope nothing here
    
  Serial.println("something...");
}
