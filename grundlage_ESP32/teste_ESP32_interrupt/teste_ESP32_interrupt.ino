#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <HMC5883L.h>

hw_timer_t * timer0 = NULL;
hw_timer_t * timer1 = NULL;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile int hall_buss_counter=0;

int LED_BUILTIN=2;

volatile int flag=B0;

volatile int statusLEFT = 0;
volatile int statusRIGHT = 0;
volatile int statusFLEFT = 0;
volatile int statusFRIGHT = 0;

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

volatile int status_Hall =B0;
int maxHallSignal=10;

HMC5883L compass; //SDA=>21 e SCL=>22
volatile float rumo_real;

TinyGPSPlus gps;
volatile int gps_counter=0;

HardwareSerial SerialGPS(1); //usar portas RX2 e TX2 (GPIO16, GPIO17)
volatile unsigned long distanceKmToLondon;
volatile unsigned long courseToLondon;
volatile unsigned long lat_boat;
volatile unsigned long lon_boat;

void acquire_GPS(){

    static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;  
    int count=0;
    
    while (SerialGPS.available() > 0 || count<5) {
        gps.encode(SerialGPS.read());
        count++;
    }
    
    if (gps.satellites.value() > 4) {

    lat_boat=gps.location.lat();
    lon_boat=gps.location.lng();
    distanceKmToLondon =(unsigned long)TinyGPSPlus::distanceBetween(lat_boat,lon_boat,LONDON_LAT, LONDON_LON) / 1000;
    courseToLondon =TinyGPSPlus::courseTo(lat_boat,lon_boat,LONDON_LAT, LONDON_LON);
    }
}
void acquire_Hall(){

    statusRIGHT = analogRead(pinRIGHT);
    statusLEFT = analogRead(pinLEFT);
    statusFRIGHT = analogRead(pinFRIGHT);
    statusFLEFT = analogRead(pinFLEFT);

    if ( statusFRIGHT < maxHallSignal){status_Hall |= 1;}else{status_Hall &= ~1;}
    if ( statusRIGHT < maxHallSignal){status_Hall |= (1<<1);}else{status_Hall &= ~(1<<1);}
    if ( statusLEFT < maxHallSignal){status_Hall |= (1<<2);}else{status_Hall &= ~(1<<2);}
    if ( statusFLEFT < maxHallSignal){status_Hall |= (1<<3);}else{status_Hall &= ~(1<<3);}
}

void acquire_Buss(){

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
    if (rumo_real < 0){rumo_real += 2 * PI;}
    if (rumo_real > 2 * PI){rumo_real -= 2 * PI;}
    // Converter para graus e guardar na variável apropriada
    rumo_real = rumo_real * 180/M_PI;
}

void IRAM_ATTR onTimer0(){

    hall_buss_counter++;   
}

void IRAM_ATTR onTimer1(){

   gps_counter++;
}
void setup(){

    Serial.begin(115200);
    pinMode (LED_BUILTIN, OUTPUT);
    SerialGPS.begin(9600, SERIAL_8N1, 16, 17); //setup da comunicação serial entre ESP32 e GPS

    Serial.println("start timer ");
    timer0 = timerBegin(0, 20000, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 20000 -> 250000 ns = 250 us, countUp
    timerAttachInterrupt(timer0, &onTimer0, true); // edge (not level) triggered 
    timerAlarmWrite(timer0, 1000, true); // 1000 * 250 us = 0.25 s (4Hz), autoreload true
    timerAlarmEnable(timer0); // enable

    timer1 = timerBegin(1, 40000, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 40000 -> 500000 ns = 500 us, countUp
    timerAttachInterrupt(timer1, &onTimer1, true); // edge (not level) triggered 
    timerAlarmWrite(timer1, 20000, true); // 20000 * 500 us = 10 s (0.1Hz), autoreload true
    timerAlarmEnable(timer1); // enable

    Serial.println("Initialize HMC5883L");
    while (!compass.begin()){
        Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
        delay(500);
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

void loop(){

    if(gps_counter>0){
      acquire_GPS();
      gps_counter--;  
    }
    if(hall_buss_counter>0){
      acquire_Buss();
      acquire_Hall();
      hall_buss_counter--;
    }
    //HALL
    if ( statusFRIGHT < maxHallSignal){digitalWrite(LpinFRIGHT, HIGH);}else{digitalWrite(LpinFRIGHT, LOW);}
    if ( statusRIGHT < maxHallSignal){digitalWrite(LpinRIGHT, HIGH);}else{digitalWrite(LpinRIGHT, LOW);}
    if ( statusLEFT < maxHallSignal){digitalWrite(LpinLEFT, HIGH);}else{digitalWrite(LpinLEFT, LOW);}
    if ( statusFLEFT < maxHallSignal){digitalWrite(LpinFLEFT, HIGH);}else{digitalWrite(LpinFLEFT, LOW);}
    Serial.print("HALL=");Serial.println(status_Hall);

    //GPS
    Serial.print("LAT=");  Serial.println(lat_boat, 6);
    Serial.print("LONG="); Serial.println(lon_boat, 6);
    Serial.print("DIST="); Serial.println(distanceKmToLondon);
    Serial.print("COURSE="); Serial.println(courseToLondon);

    //BUSS
     Serial.print("BUSS=");Serial.println(rumo_real);
}
