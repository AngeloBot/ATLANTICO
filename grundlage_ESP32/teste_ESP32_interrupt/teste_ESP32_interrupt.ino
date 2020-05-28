#include <Wire.h>
#include <HMC5883L.h>

#include <TinyGPS++.h> 
#include <HardwareSerial.h> 


HMC5883L compass;
float rumo_real;

TinyGPSPlus gps;
HardwareSerial SerialGPS(1);//RX1 e TX1

double courseToLondon;
unsigned long distanceKmToLondon;
double lat_boat;
double lon_boat;
static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

hw_timer_t * timer0 = NULL; //hall
hw_timer_t * timer1 = NULL; //gps
hw_timer_t * timer2 = NULL; //buss

volatile int flag_hall=0;
volatile int flag_gps=0;
volatile int flag_buss=0;

volatile int counter0=0; //hall
volatile int counter1=0; //gps
volatile int counter2=0; //buss

int LED_Hall = 2;
int LED_GPS=4;
int LED_Buss=5;

//variáveis para gravar leitura analógica dos pins dos sensores hall
int statusLEFT = 0;
int statusRIGHT = 0;
int statusFLEFT = 0;
int statusFRIGHT = 0;
//pin para input proveniente sensores
int pinLEFT = 14;
int pinRIGHT = 26; 
int pinFLEFT = 13;
int pinFRIGHT = 15;

//variável que resume estado de todos os hall
int status_Hall =B0;
//variável a ser calibrada para valor máximo q atinge a leitura analógica do sensor hall quando ele detecta campo magnético
int maxHallSignal=10;

void IRAM_ATTR onTimer0(){
  
    if(counter0==0){
        digitalWrite(LED_Hall, HIGH);
        }
    else{
        digitalWrite(LED_Hall, LOW);
        }

    counter0=~counter0;
    flag_hall++;

}

void IRAM_ATTR onTimer1(){
  

    if(counter1==0){
        digitalWrite(LED_GPS, HIGH);
        }
    else{
        digitalWrite(LED_GPS, LOW);
        }

    counter1=~counter1;
    flag_gps++;

}

void IRAM_ATTR onTimer2(){
  

    if(counter2==0){
        digitalWrite(LED_Buss, HIGH);
        }
    else{
        digitalWrite(LED_Buss, LOW);
        }

    counter2=~counter2;
    flag_buss++;

}

void setup() {
    Serial.begin(115200);
    pinMode (LED_Hall, OUTPUT);
    pinMode (LED_GPS, OUTPUT);
    pinMode (LED_Buss, OUTPUT);

    SerialGPS.begin(9600, SERIAL_8N1, 16, 17);
    
    Serial.println("start timers ");
    timer0 = timerBegin(0, 8000, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 8000 -> 100000 ns = 100 us, countUp
    timerAttachInterrupt(timer0, &onTimer0, true); // edge (not level) triggered 
    timerAlarmWrite(timer0, 1000, true); // 1000 * 100 us = 0.1 s (10 Hz), autoreload true
    timerAlarmEnable(timer0); // enable

    timer1 = timerBegin(1, 8000, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 8000 -> 100000 ns = 100 us, countUp
    timerAttachInterrupt(timer1, &onTimer1, true); // edge (not level) triggered 
    timerAlarmWrite(timer1, 100000, true); // 100000 * 100 us = 10 s (0.1Hz), autoreload true
    timerAlarmEnable(timer1); // enable

    timer2 = timerBegin(2, 8000, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 8000 -> 100000 ns = 100 us, countUp
    timerAttachInterrupt(timer2, &onTimer2, true); // edge (not level) triggered 
    timerAlarmWrite(timer2, 2500, true); // 2500 * 100 us = 0.25 s (4Hz), autoreload true
    timerAlarmEnable(timer2); // enable

    if (!compass.begin()){
      Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    }
    // Set measurement range
    compass.setRange(HMC5883L_RANGE_1_3GA);

    // Set measurement mode
    compass.setMeasurementMode(HMC5883L_CONTINOUS);

    // Set data rate
    compass.setDataRate(HMC5883L_DATARATE_30HZ);

    // Set number of samples averaged
    compass.setSamples(HMC5883L_SAMPLES_8);

    // Set calibration offset. See HMC5883L_calibration.ino
    compass.setOffset(99, -41);
}

void loop() {
    if (flag_hall>0) {

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
        
        Serial.println(status_Hall);
        flag_hall--;  
        }
    
    if(flag_buss>0){
        Vector norm = compass.readNormalize();

        // Calculate heading
        rumo_real = atan2(norm.YAxis, norm.XAxis);
        // Set declination angle on your location and fix heading
        // You can find your declination on: http://magnetic-declination.com/
        // (+) Positive or (-) for negative
        // For Bytom / Poland declination angle is 4'26E (positive)
        // For Barueri / Brazil declination angle is 21'22W(negative)
        // For Santo Amaro / Brazil declination angle is 21'23W(negative)
        // For Taboao da Serra / Brazil declination angle is 21'25W(negative)
        // Formula: (deg + (min / 60.0)) / (180 / M_PI);
        float declinationAngle = (-21.0 - (25.0 / 60.0)) / (180 / M_PI);
        rumo_real += declinationAngle;
        
        // Correct for heading < 0deg and heading > 360deg
        if (rumo_real < 0){
            rumo_real += 2 * PI;
        }
        if (rumo_real > 2 * PI){
            rumo_real -= 2 * PI;
        }
        // Converter para graus e guardar na variável apropriada
        rumo_real = rumo_real * 180/M_PI;
        flag_buss--;
        Serial.println(rumo_real);
    }

    if(flag_gps>0) {

        while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
        
        Serial.print("GPS= ");Serial.println(gps.satellites.value());
        
        if (gps.satellites.value() > 4) {
        
            lat_boat=gps.location.lat();
            lon_boat=gps.location.lng();
            Serial.print("LAT=");  Serial.println(lat_boat, 6);
            Serial.print("LONG="); Serial.println(lon_boat, 6);
            

            distanceKmToLondon =(unsigned long)TinyGPSPlus::distanceBetween(lat_boat,lon_boat,LONDON_LAT,LONDON_LON) / 1000;
            Serial.println(distanceKmToLondon);

            courseToLondon =TinyGPSPlus::courseTo(lat_boat,lon_boat,LONDON_LAT,LONDON_LON);
            Serial.println(courseToLondon);
            
        }
        flag_gps--;
    }
}
