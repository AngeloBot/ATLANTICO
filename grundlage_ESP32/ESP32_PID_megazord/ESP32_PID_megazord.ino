#include <Wire.h>
#include <HMC5883L.h>

#include <TinyGPS++.h> 
#include <HardwareSerial.h> 

#include <ESP32_Servo.h>

//configuração servo
Servo servo;
int pos=0;
#define pos_zero 90 //define posição zero
#define servoPin 18 //define pin do servo
#define leme_min 60 //max de -30 graus
#define leme_max 120 //max de 30 graus

//PID
double SOMAE=0;
#define Kp 1.2
#define Ki 0.5
#define Kd 0.5
float timer_PID=0;
float E=0;
float v_yaw=0;
float ultimo_rumo=0;
int cte=0;

//variáveis de estado da máquina
int current_state = 0;
int previous_state = 0;

//erro de rumo
#define lambda_rumo 5 //poço de erro em relação ao rumo ideal
#define lambda_jibe 5 //poço de erro de rumo em relação ao rumo ideal ao dar jibe
#define ang_jibe 110 //diferença entre rumo real e ideal para realizar o jibe
int rumo_margin_flag=0; //1-> rumo está dentro da margem de erro estipulada | 0->não está dentro

//variáveis para transição para o estado 13
double deltat = 10000; //em ms
double timer_13=0;
int flag_13=0;

HMC5883L compass;

//variáveis bússola================================================================
float rumo_real;
float erro_rumo=0;
#define BUSS_X_OFFSET 262
#define BUSS_Y_OFFSET -171
//=================================================================================

TinyGPSPlus gps;
HardwareSerial SerialGPS(1);//RX1 e TX1

//variáveis GPS====================================================================

double rumo_ideal = 0;
//double rumo_idel_save=0;
//double rumo_ideal = 170;
float lat_barco,long_barco;
//float lat_barco=-23.554703;
//float long_barco=-46.877897;

float lat_waypoint, long_waypoint;
float desvio_waypoint;
#define waypoint_radius 100 //em metros

//lista de waypoints com respectivos desvios magnéticos
float lat_long_desvio_waypoint[6] = {-23.556035, -46.877682, -21.24, -23.554985, -46.876259, -21.24};
int waypoint_count = 0;

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

#define LED_Hall 2
#define LED_GPS 4
#define LED_Buss 5

//variáveis para gravar leitura analógica dos pins dos sensores hall
int statusLEFT = 0;
int statusRIGHT = 0;
int statusFLEFT = 0;
int statusFRIGHT = 0;
//pin para input proveniente sensores
#define pinLEFT 14
#define pinRIGHT 26 
#define pinFLEFT 13
#define pinFRIGHT 15

//variável que resume estado de todos os hall
int status_Hall =B0000;
//variável a ser calibrada para valor máximo q atinge a leitura analógica do sensor hall quando ele detecta campo magnético
#define maxHallSignal 10

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

void acquire_GPS(){
    
    double dist_waypoint;

    while (SerialGPS.available() > 0) {
        gps.encode(SerialGPS.read());
    }
    
    Serial.print("GPS= ");Serial.println(gps.satellites.value());
    

    if (gps.satellites.value() > 4) {
    
        lat_barco=gps.location.lat();
        long_barco=gps.location.lng();
        Serial.print("LAT=");  Serial.println(lat_barco, 6);
        Serial.print("LONG="); Serial.println(long_barco, 6);
        

        dist_waypoint=(unsigned long)TinyGPSPlus::distanceBetween(lat_barco,long_barco,lat_waypoint,long_barco); //m
        
        Serial.print("Dist ");Serial.println(dist_waypoint);

        rumo_ideal =(double)TinyGPSPlus::courseTo(lat_barco,long_barco,lat_waypoint,long_waypoint);
        Serial.print("Course ");Serial.println(rumo_ideal);

        if (waypoint_radius > dist_waypoint){
            waypoint_count += 1;
            lat_waypoint=lat_long_desvio_waypoint[waypoint_count*3];
            long_waypoint=lat_long_desvio_waypoint[waypoint_count*3+1];
            desvio_waypoint=lat_long_desvio_waypoint[waypoint_count*3+2];
            acquire_GPS();
         }
        
    }

    flag_gps--;

}
void acquire_buss(){

    //salvar rumo anterior para calculo da velocidade angular do barco
    ultimo_rumo=rumo_real;

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
    rumo_real = rumo_real * 180/M_PI-90;
    
    if(current_state==3 || current_state==4){
        calc_erro_rumo(rumo_real);
    }
    else{
        calc_erro_rumo(rumo_ideal);
    }

    flag_buss--;
    Serial.print("rumo real: ");Serial.println(rumo_real);
    Serial.print("erro rumo: ");Serial.println(erro_rumo);

}

void calc_erro_rumo(double rumo){
    
    erro_rumo=rumo_real-(rumo+cte);

    //ajeitar sinal para menor mudança de rumo
    if (((erro_rumo) < 0 && (erro_rumo) >= -180)) {
        erro_rumo = erro_rumo;
    }
    else if(((erro_rumo) > 0 && (erro_rumo) <= 180)){
        erro_rumo= erro_rumo;
    }
    else if (((erro_rumo) < 360  && (erro_rumo) > 180) ) {
        erro_rumo = -abs(abs(erro_rumo) - 360);
    }
    else if(((erro_rumo) > -360 && (erro_rumo) < -180)){
        erro_rumo = abs(abs(erro_rumo) - 360);
    }
    if(abs(erro_rumo)<lambda_rumo){
        rumo_margin_flag=1;
    }
    else{
        rumo_margin_flag=0;
    }
}

int leme_ok(int pos_leme){

    if(pos_leme<leme_min){
        return leme_min;
    }
    else if(pos_leme>leme_max){
        return leme_max;
    }
    else{
        return pos_leme;
    }
}

int calc_PID(float E){

    int now=millis();
    int heap=now-timer_PID;
    timer_PID=now;
    
    SOMAE+=E*heap/1000;

    //velocidade angular em rad/s
    v_yaw=(rumo_real-ultimo_rumo)*PI*1000/(180*heap);

    return leme_ok(pos_zero+Kp*E+Ki*SOMAE-Kd*v_yaw);
}
int calc_PD(float E){

    int now=millis();
    int heap=now-timer_PID;
    timer_PID=now;
    
    //velocidade angular em rad/s
    v_yaw=(rumo_real-ultimo_rumo)*PI*1000/(180*heap);

    return leme_ok(pos_zero+Kp*E-Kd*v_yaw);
}


void setup() {
    Serial.begin(115200);
    pinMode (LED_Hall, OUTPUT);
    pinMode (LED_GPS, OUTPUT);
    pinMode (LED_Buss, OUTPUT);

    SerialGPS.begin(9600, SERIAL_8N1, 16, 17);
    
    servo.attach(servoPin);
    servo.write(pos_zero);

    //variáveis de estado
    current_state = 0;
    //previous_state = 0;
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
    compass.setOffset(BUSS_X_OFFSET, BUSS_Y_OFFSET);

    //rotina de aquisicao inicial
    acquire_hall();
    acquire_GPS();
    acquire_buss();
    timer_PID=millis();

}

void loop() {
    
    Serial.print("STATE= "); Serial.println(current_state);
    delay(250);
    
    
    if (flag_hall>0) {
        acquire_hall();        
    }
    
    if(flag_gps>0) {
        acquire_GPS();
    }

    if(flag_buss>0){

        acquire_buss();

        //mover servo apenas quando lê a bússola
        switch (current_state){
    
            case 0: //à favor-----------------------------------------------------------------------------------------------------
                

                pos=calc_PID(erro_rumo);
                servo.write(pos);

                if(status_Hall==B1000 || status_Hall==B1100 || status_Hall==B0110 || status_Hall==B0100){
                    previous_state = current_state;
                    current_state=3; //ir para contra por BB 
                }
                else if(status_Hall==B0001 || status_Hall==B0011 || status_Hall==B0010){
                    previous_state = current_state;
                    current_state=4; //ir para contra por BE
                }
                else if(erro_rumo < -lambda_rumo && status_Hall==B0000){
                    previous_state = current_state;
                    current_state=1; //ir para ajeitar BB
                }
                else if(erro_rumo > lambda_rumo && status_Hall==B0000){
                    previous_state = current_state;
                    current_state=2; //ir para ajeitar BE
                }
                previous_state = current_state;
                break;

            case 1: //ajeitar BB---------------------------------------------------------------------------------------------------

                pos=calc_PID(erro_rumo);
                servo.write(pos);

                if( abs(erro_rumo) < lambda_rumo || status_Hall!=B0000){
                    previous_state = current_state;
                    current_state=0; //ir para à favor
                }
                previous_state = current_state;
                break;

            case 2: //ajeitar BE---------------------------------------------------------------------------------------------------

                pos=calc_PID(erro_rumo);
                servo.write(pos);

                if( abs(erro_rumo) < lambda_rumo || status_Hall!=B0000){
                    previous_state = current_state;
                    current_state=0; //ir para à favor
                }
                previous_state = current_state;
                break;

            case 3: //contra por BB------------------------------------------------------------------------------------------------
                if(previous_state!=3){
                    switch(previous_state){
                        case 0:
                            if(rumo_margin_flag==0){
                                cte=-5;
                            }
                            else{
                                cte=0
                            }
                            break;

                        case 5:
                            cte=0;
                            break;
                        case 7:
                            cte=5;
                            break;
                        case 9:
                            if(rumo_margin_flag==0){
                                cte=-5;
                            }
                            else{
                                cte=0
                            }
                            break;
                    }                   
                }
                calc_erro_rumo(rumo_real);
                pos=calc_PID(erro_rumo);
                if(previous_state==12){
                    pos=0;
                }
                servo.write(pos);

                if(status_Hall==B0100 || status_Hall==B0110){
                    flag_13=1;
                    previous_state = current_state;
                    current_state=7; //ir para arribar_1 BB
                }
                else if(abs(erro_rumo)<(ang_jibe+lambda_jibe) && abs(erro_rumo)>(90-lambda_jibe) && status_Hall==B1100){
                    flag_13=1;
                    previous_state = current_state;
                    current_state=11; //ir para Jib BB
                }
                else if(erro_rumo < - lambda_rumo && status_Hall==B1000){//rumo_ideal está para BB e pode orçar
                    previous_state = current_state;
                    current_state=9; //ir para orçar BB
                }
                else if(erro_rumo > lambda_rumo) {//rumo_ideal está para BE
                    previous_state = current_state;
                    current_state=5; //ir para arribar_2 BB
                }

                else if(status_Hall==B0000){
                    previous_state = current_state;
                    current_state=0; //ir para à favor
                }

                else if(status_Hall==B0001 || status_Hall==B0011 || status_Hall==B0010){
                    previous_state = current_state;
                    current_state=4; //ir para contra por BE
                }
                previous_state = current_state;
                break;

            case 4: //contra por BB------------------------------------------------------------------------------------------------
            
                if(previous_state!=4){
                    switch(previous_state){
                        case 0:
                            if(rumo_margin_flag==0){
                                cte=5;
                            }
                            else{
                                cte=0
                            }
                            break;

                        case 6:
                            cte=0;
                            break;
                        case 8:
                            cte=-5;
                            break;
                        case 10:
                            if(rumo_margin_flag==0){
                                cte=5;
                            }
                            else{
                                cte=0
                            }
                            break;
                    }                   
                }
                calc_erro_rumo(rumo_real);
                pos=calc_PID(erro_rumo);
                if(previous_state==11){
                    pos=0;
                }
                servo.write(pos);

                if(status_Hall==B0010 || status_Hall==B0110){
                    flag_13=1;
                    previous_state = current_state;
                    current_state=8; //ir para arribar_1 BE   
                }
                else if(abs(erro_rumo)<(ang_jibe+lambda_jibe) && abs(erro_rumo)>(90-lambda_jibe) && status_Hall==B0011){
                    flag_13=1;
                    previous_state = current_state;
                    current_state=12; //ir para Jib BE        
                }
                else if(erro_rumo > lambda_rumo && status_Hall==B0001){//rumo_ideal está para BE e pode orçar
                    previous_state = current_state;
                    current_state=10; //ir para orçar BE    
                }
                else if(erro_rumo < -lambda_rumo) {//rumo_ideal está para BB
                    previous_state = current_state;
                    current_state=6; //ir para arribar_2 BE
                }
                
                else if(status_Hall==B1000 || status_Hall==B1100 || status_Hall==B0100){
                    previous_state = current_state;
                    current_state=3; //ir para contra por BB
                }

                else if(status_Hall==B0000){
                    previous_state = current_state;
                    current_state=0; //ir para à favor
                }
                previous_state = current_state;
                break;

            case 5: //arribar_2 BB-------------------------------------------------------------------------------------------------
            
                pos=calc_PID(erro_rumo);
                servo.write(pos);

                if((status_Hall!=B1000 && status_Hall!=B1100) || abs(erro_rumo)<lambda_rumo || erro_rumo<-lambda_rumo) {
                    previous_state = current_state;
                    current_state=3; //ir para contra por BB
                }
                previous_state = current_state;
                break;

            case 6: //arribar_2 BE-------------------------------------------------------------------------------------------------
            
                pos=calc_PID(erro_rumo);
                servo.write(pos);

                if((status_Hall!=B0011 && status_Hall!=B0001)  || abs(erro_rumo)<lambda_rumo || erro_rumo>lambda_rumo) {
                    previous_state = current_state;
                    current_state=4; //ir para contra por BE
                }
                previous_state = current_state;
                break;

            case 7: //arribar_1 BB-------------------------------------------------------------------------------------------------

                if (flag_13==1){
                    timer_13=millis();
                    flag_13=0;
                }
                
                switch(status_Hall){
                    case 4:
                        E=15;
                        break;
                    case 6:
                        E=40;
                        break;
                    default:
                        E=0;
                        break;
                }
                SOMAE=0;
                pos=calc_PD(E);
                servo.write(pos);

                if(status_Hall!=B0100 && status_Hall!=B0110 ) {
                    previous_state = current_state;
                    current_state=3; //ir para contra por BB
                }
                else if(millis()-timer_13 > deltat ){=
                    flag_13=1;
                    previous_state = current_state;
                    current_state=13; //ir para espera
                }
                previous_state = current_state;
                break;
                
            
            case 8: //arribar_1 BE-------------------------------------------------------------------------------------------------

                if (flag_13==1){
                    timer_13=millis();
                    flag_13=0;
                }

                switch(status_Hall){
                    case 2:
                        E=-15;
                        break;
                    case 6:
                        E=-40;
                        break;
                    default:
                        E=0;
                        break;
                }
                SOMAE=0;
                pos=calc_PD(E);
                servo.write(pos);

                if(status_Hall!=B0110 && status_Hall!=B0010 ){
                    previous_state = current_state;
                    current_state=4; //ir para contra por BE
                }

                else if(millis()-timer_13 > deltat ){
                    flag_13=1;
                    previous_state = current_state;
                    current_state=13; //ir para espera
                }
                previous_state = current_state;
                break;

            case 9: //orçar BB-----------------------------------------------------------------------------------------------------
            
                SOMAE=0;
                pos=calc_PD(-10);
                servo.write(pos);

                if((status_Hall!=B1000)|| abs(erro_rumo)<lambda_rumo || erro_rumo>lambda_rumo) {
                    previous_state = current_state;
                    current_state=3; //ir para contra por BB
                }
                previous_state = current_state;
                break;

            case 10: //orçar BE----------------------------------------------------------------------------------------------------
            
                SOMAE=0;
                pos=calc_PD(10);
                servo.write(pos);

                if((status_Hall!=B0001) || abs(erro_rumo)<lambda_rumo || erro_rumo<-lambda_rumo) {
                    previous_state = current_state;
                    current_state=4; //ir para contra por BE
                }
                previous_state = current_state;
                break;

            case 11: //Jibe BB------------------------------------------------------------------------------------------------------

                if (flag_13==1){
                    timer_13=millis();
                    flag_13=0;
                }
                
                SOMAE=0;
                servo.write(leme_max);

                if(abs(erro_rumo)<lambda_rumo || (status_Hall!=B1100 && status_Hall!=B1000 && status_Hall!=B0000 && status_Hall!=B0100)){
                    previous_state = current_state;
                    current_state=4; //ir para contra por BE
                }

                else if(millis()-timer_13 > deltat ){
                    flag_13=1;
                    previous_state = current_state;
                    current_state=13; //ir para espera
                }
                previous_state = current_state;
                break;

            case 12: //Jibe BE------------------------------------------------------------------------------------------------------

                if (flag_13==1){
                    timer_13=millis();
                    flag_13=0;
                }

                SOMAE=0;
                servo.write(leme_min);

                if(abs(erro_rumo)<lambda_rumo || (status_Hall!=B0011 && status_Hall!=B0001 && status_Hall!=B0000 && status_Hall!=B0010)) {
                    previous_state = current_state;
                    current_state=3; //ir para contra por BB
                }

                else if(millis()-timer_13 > deltat ){
                    flag_13=1;
                    previous_state = current_state;
                    current_state=13; //ir para espera
                }
                previous_state = current_state;
                break;

            case 13: //espera------------------------------------------------------------------------------------------------------
            
                //leme ao centro
                servo.write(90);

                if (flag_13==1){
                    timer_13=millis();
                    flag_13=0;
                }
                if(millis()-timer_13 > deltat ){
                    previous_state = current_state;
                    current_state=0; //ir para à favor
                }
                previous_state = current_state;
                break;
        }
    }
}
