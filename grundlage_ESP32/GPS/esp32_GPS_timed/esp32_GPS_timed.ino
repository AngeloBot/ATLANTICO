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

#include <TinyGPS++.h> 
#include <HardwareSerial.h> 
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);//RX1 e TX1


hw_timer_t * timer = NULL;
int state=0;
int LED_BUILTIN = 2;
volatile int gps_counter=0;

void IRAM_ATTR onTimer(){
  Serial.println(String("onTimer() ")+String(millis()));
  if(state==0){
    digitalWrite(LED_BUILTIN, HIGH);
    }
  else{
    digitalWrite(LED_BUILTIN, LOW);
    }
  state=~state;
  gps_counter++;
}

void setup() {
  Serial.begin(115200);
  pinMode (LED_BUILTIN, OUTPUT);

  Serial.println("start timer ");
  timer = timerBegin(0, 800, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 10000 ns = 10 us, countUp
  timerAttachInterrupt(timer, &onTimer, true); // edge (not level) triggered 
  timerAlarmWrite(timer, 1000000, true); // 1000000 * 10 us = 10 s, autoreload true
  timerAlarmEnable(timer); // enable

  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);//RX1 e TX1
}

void loop() {
  // nope nothing here
  if(gps_counter>0) {

    static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;  
    /*
    * Rohdaten von Serieller Verbndung zum GPS-Modul
    * einlesen. Die Daten werden mittels TinyGPS++ verarbeitet
    * Die Daten werden bewusst erst nach der Zuweisung der Variablen
    * gelesen, damit wir noch im nachfolgenden vereinfacht 
    * Berechnungen anstellen können.
    */
    while (SerialGPS.available() > 0) {
      gps.encode(SerialGPS.read());
    }

    /*
    * Diverse Berechnungen von Maximum und Minimum-Werten und zurückgelegter Distanz
    * Diese werden aber erst gemacht, wenn mindestens ein Fix mit 4 Satelliten vorhanden
    * ist, allenfalls wäre die Genauigkeit nicht gegeben und es würden falsche
    * Werte berechnet werden.
    */
    if (gps.satellites.value() > 4) {
      
      Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
      Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
      Serial.print("Sats=");  Serial.println(gps.satellites.value());
        unsigned long distanceKmToLondon =
      (unsigned long)TinyGPSPlus::distanceBetween(
        gps.location.lat(),
        gps.location.lng(),
        LONDON_LAT, 
        LONDON_LON) / 1000;
    Serial.println(distanceKmToLondon);

    double courseToLondon =
      TinyGPSPlus::courseTo(
        gps.location.lat(),
        gps.location.lng(),
        LONDON_LAT, 
        LONDON_LON);

    Serial.println(courseToLondon);
    gps_counter--;
    }
  }
}
