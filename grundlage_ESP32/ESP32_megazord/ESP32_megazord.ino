#include <Wire.h>
#include <HMC5883L.h>

#include <TinyGPS++.h> 
#include <HardwareSerial.h> 

//erro de rumo
int zero_margin_rumo=10; //poço do zero de rumo
int lambda_rumo= 10; //poço de erro em relação ao rumo ideal
int lambda_jib=5; //poço de erro de rumo em relação ao rumo ideal ao dar jibe

//variáveis para transição para o estado 13
double deltat = 10000; //em ms
double timer_13_i=0;
double timer_13_f=0;
int flag_13=0;

HMC5883L compass;

//variáveis bússola================================================================
float rumo_real;
float erro_rumo=0;
//=================================================================================

TinyGPSPlus gps;
HardwareSerial SerialGPS(1);//RX1 e TX1

//variáveis GPS====================================================================
float rumo_ideal = 0;
float lat_barco, long_barco;
float lat_waypoint, long_waypoint;
float desvio_waypoint;

//lista de waypoints com respectivos desvios magnéticos
float lat_long_desvio_waypoint[6] = {-23.578426, -46.744687, -21.32, -23580636, -46.743040, -21.32};
volatile int waypoint_count = 0;

//=================================================================================

//variáveis para setup dos timers==================================================
hw_timer_t * timer0 = NULL; //hall
hw_timer_t * timer1 = NULL; //gps
hw_timer_t * timer2 = NULL; //buss

volatile int flag_hall=0;
volatile int flag_gps=0;
volatile int flag_buss=0;

volatile int counter0=0; //hall
volatile int counter1=0; //gps
volatile int counter2=0; //buss
//=================================================================================

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
int status_Hall =B0000;
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

void acquire_hall(){
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

void acquire_GPS(){
    
    double dist_waypoint;

    while (SerialGPS.available() > 0) {
        gps.encode(SerialGPS.read());
    }
    
    Serial.print("GPS= ");Serial.println(gps.satellites.value());
    
    if (gps.satellites.value() > 4) {
    
        lat_boat=gps.location.lat();
        lon_boat=gps.location.lng();
        Serial.print("LAT=");  Serial.println(lat_boat, 6);
        Serial.print("LONG="); Serial.println(lon_boat, 6);
        

        dist_waypoint=(unsigned long)TinyGPSPlus::distanceBetween(lat_barco,long_barco,lat_waypoint,long_barco) / 1000;
        Serial.print("Dist ");Serial.println(dist_waypoint);

        if (waypoint_radius > ){
            waypoint_count += 1;
            lat_waypoint=lat_long_desvio_waypoint[waypoint_count*3];
            long_waypoint=lat_long_desvio_waypoint[waypoint_count*3+1];
            desvio_waypoint=lat_long_desvio_waypoint[waypoint_count*3+2];
         }
        
    }
    flag_gps--;

}
void acquire_buss(){

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
    erro_rumo=rumo_ideal-rumo_real;

    flag_buss--;
    Serial.print("rumo real: ");Serial.println(rumo_real);

}


void setup() {
    Serial.begin(115200);
    pinMode (LED_Hall, OUTPUT);
    pinMode (LED_GPS, OUTPUT);
    pinMode (LED_Buss, OUTPUT);

    SerialGPS.begin(9600, SERIAL_8N1, 16, 17);
    
    //variáveis de estado
    current_state = 0;
    previous_state = 0;
    waypoint_count = 0;

    lat_waypoint=lat_long_desvio_waypoint[0];
    long_waypoint=lat_long_desvio_waypoint[1];
    desvio_waypoint=lat_long_desvio_waypoint[2];

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

    while (!compass.begin()){
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
        acquire_Hall();    
    }
    if(flag_buss>0){
        acquire_buss();
    }
    if(flag_gps>0) {
        acquire_GPS();
    }

    switch (current_state){
  
        case 0: //à favor

            //manter leme

            if(status_Hall==B1000 || status_Hall==B1100 || status_Hall==B0110){
                current_state=3; //ir para contra por BB
                break;

            }
            else if(status_Hall==B0001 || status_Hall==B0011 || status_Hall==B0010){
                current_state=4; //ir para contra por BE
                break;
            }
            else if(erro_rumo > lambda_rumo){
                current_state=1; //ir para ajeitar BB
                break;
            }
            else if(erro_rumo < (-1)*lambda_rumo){
                current_state=4; //ir para ajeitar BE
            }
            break;

        case 1: //ajeitar BB---------------------------------------------------------------------------------------------------

            //leme p/ BB

            if(erro_rumo < lambda_rumo){
                current_state=0; //ir para à favor
            }
            break;

        case 2: //ajeitar BE---------------------------------------------------------------------------------------------------

            //manter leme

            if(erro_rumo > (-1)*lambda_rumo){
                current_state=0; //ir para à favor
            }
            break;

        case 3: //contra por BB------------------------------------------------------------------------------------------------
        
            //manter leme

            if(status_Hall==B0100 || status_Hall==B0110){
                flag_13=1;
                current_state=7; //ir para arribar_1 BB
                break;

            }
            else if(abs(erro_rumo)<(90+lambda_jibe) && abs(erro_rumo)>(90-lambda_jibe)){
                flag_13=1;
                current_state=11; //ir para Jib BB
                break;
            }
            else if(erro_rumo > 0 && status_Hall==B1000){//rumo_ideal está para BB e pode orçar
                current_state=9; //ir para orçar BB
                break;
            }
            else if(erro_rumo < 0) {//rumo_ideal está para BE
                current_state=5; //ir para arribar_2 BB
            }
            break;

        case 4: //contra por BB------------------------------------------------------------------------------------------------
        
            //manter leme

            if(status_Hall==B0010 || status_Hall==B0110){
                flag_13=1;
                current_state=8; //ir para arribar_1 BE
                break;
            }
            else if(abs(erro_rumo)<(90+lambda_jibe) && abs(erro_rumo)>(90-lambda_jibe)){
                flag_13=1;
                current_state=12; //ir para Jib BE
                break;
            }
            else if(erro_rumo < 0 && status_Hall==B0001){//rumo_ideal está para BE e pode orçar
                current_state=10; //ir para orçar BE
                break;
            }
            else if(erro_rumo > 0) {//rumo_ideal está para BB
                current_state=6; //ir para arribar_2 BE
            }
            break;

        case 5: //arribar_2 BB-------------------------------------------------------------------------------------------------
        
            //leme p/ BE

            if(status_Hall==B0000 || (erro_rumo<(zero_margin_rumo) && erro_rumo>(-1)*zero_margin_rumo)) {
                current_state=3; //ir para contra por BB
            }
            break;

        case 6: //arribar_2 BE-------------------------------------------------------------------------------------------------
        
            //leme p/ BB

            if(status_Hall==B0000 || (erro_rumo<(zero_margin_rumo) && erro_rumo>(-1)*zero_margin_rumo)) {
                current_state=4; //ir para contra por BE
            }
            break;

        case 7: //arribar_1 BB-------------------------------------------------------------------------------------------------

            if (flag_13==1){
                timer_13=milis();
                flag_13=0;
            }
            //leme p/ BE

            if(status_Hall==B0000 || status_Hall==B1000 || status_Hall==B1100 ) {
                current_state=3; //ir para contra por BB
            }
            else if(milis()-timer_13 > deltat ){

                flag_13=1;
                current_state=13; //ir para espera
            }
            break;
            
        
        case 8: //arribar_1 BE-------------------------------------------------------------------------------------------------

            if (flag_13==1){
                timer_13=milis();
                flag_13=0;
            }
            //leme p/ BB

            if(status_Hall==B0000 || status_Hall==B0001 || status_Hall==B0011 ) {
                current_state=4; //ir para contra por BE
            }

            else if(milis()-timer_13 > deltat ){

                flag_13=1;
                current_state=13; //ir para espera
            }
            break;

        case 9: //orçar BB-----------------------------------------------------------------------------------------------------
        
            //leme p/ BB

            if(status_Hall==B1100 || (erro_rumo<(zero_margin_rumo) && erro_rumo>(-1)*zero_margin_rumo)) {
                current_state=3; //ir para contra por BB
            }
            break;

        case 10: //orçar BE----------------------------------------------------------------------------------------------------
        
            //leme p/ BE

            if(status_Hall==B0011 || (erro_rumo<(zero_margin_rumo) && erro_rumo>(-1)*zero_margin_rumo)) {
                current_state=4; //ir para contra por BE
            }
            break;

        case 11: //Jibe BB------------------------------------------------------------------------------------------------------

            if (flag_13==1){
                timer_13=milis();
                flag_13=0;
            }
            //leme p/ BE

            if(erro_rumo<(zero_margin_rumo) && erro_rumo>(-1)*zero_margin_rumo){//<<<<<<<<<<<<<adicionar condição de 10s
                current_state=4; //ir para contra por BE
            }

            else if(milis()-timer_13 > deltat ){

                flag_13=1;
                current_state=13; //ir para espera
            }
            break;

        case 12: //Jibe BE---------------------------------------------------------------------------------------------------

            if (flag_13==1){
                timer_13=milis();
                flag_13=0;
            }
            //leme p/ BB

            if(erro_rumo<(zero_margin_rumo) && erro_rumo>(-1)*zero_margin_rumo) {
                current_state=3; //ir para contra por BB
            }

            else if(milis()-timer_13 > deltat ){

                flag_13=1;
                current_state=13; //ir para espera
            }

            break;

        case 13: //espera------------------------------------------------------------------------------------------------------
        
        //leme ao centro
            if (flag_13==1)
            {
                timer_13=milis();
                flag_13=0;
            }


            if(milis()-timer_13 > deltat ){
                current_state=0; //ir para à favor
            }
        break;
}
