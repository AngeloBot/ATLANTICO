#include <TinyGPS++.h>
#include <HardwareSerial.h>

TinyGPSPlus gps;
HardwareSerial SerialGPS(1);

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
int LpinFRIGHT = 2;

int status_Hall =B0;
int maxHallSignal=10;

hw_timer_t * timer = NULL;
int state=0;
int LED_BUILTIN = 2;
int num_satelite;
double act_lat;
double act_lon;

unsigned long distanceKmToLondon;
double courseToLondon;

void acquire_GPS() {

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
    
    act_lat=gps.location.lat();
    act_lon=gps.location.lng();
    num_satelite=gps.satellites.value();
   
    distanceKmToLondon =
    (unsigned long)TinyGPSPlus::distanceBetween(
      act_lat,
      act_lon,
      LONDON_LAT, 
      LONDON_LON) / 1000;
 
  courseToLondon =
    TinyGPSPlus::courseTo(
      act_lat,
      act_lon,
      LONDON_LAT,
      LONDON_LON);
  }

}

void acquire_Hall() {
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

  }


void IRAM_ATTR onTimer(){
  if(state==0){
    digitalWrite(LED_BUILTIN, HIGH);
    }
  else{
    digitalWrite(LED_BUILTIN, LOW);
    }
  state=~state;

  acquire_GPS();
  acquire_Hall();

  
}

void setup() {
  
  Serial.begin(115200);
  pinMode (LED_BUILTIN, OUTPUT);
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);
  
  Serial.println("start timer ");
  timer = timerBegin(0, 80, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer, &onTimer, true); // edge (not level) triggered 
  timerAlarmWrite(timer, 250000, true); // 1000000 * 1 us = 1 s, autoreload true
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
  
  Serial.print("LAT=");  Serial.println(act_lat, 6);
  Serial.print("LONG="); Serial.println(act_lon, 6);
  Serial.print("Sats=");  Serial.println(num_satelite);
  Serial.println(distanceKmToLondon);
  Serial.println(courseToLondon);
  Serial.print("HALL="); Serial.println(status_Hall);
    
}
