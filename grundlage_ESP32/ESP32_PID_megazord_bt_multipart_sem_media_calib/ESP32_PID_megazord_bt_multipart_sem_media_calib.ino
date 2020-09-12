
#include <Wire.h>
#include <HMC5883L.h>

#include <TinyGPS++.h> 
#include <HardwareSerial.h> 

#include <ESP32_Servo.h>

#include "buss_tools.h"
#include "acquire_GPS.h"
#include "acquire_hall.h"
#include "controller_tools.h"
#include "supp_tools.h"
#include "def_system.h"
#include "bt_report.h"
#include "serial_report.h"

//configuração do bluetooth
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

//configuração servo
Servo servo;
int pos=0;
int nova_pos=0;
int time_servo=0;

//PID
double SOMAE=0;
int timer_PID=0;
float E=0;
float v_yaw=0;
float ultimo_rumo=0;
int cte=0;

//variáveis de estado da máquina
int current_state = 0;
int previous_state = 0;
int command_flag=3; //0=> ~report+~move | 1=> ~report+move | 2=> report+~move | 3=> report+move || report=>bit+ significativo || move=>bit- significativo
int time_report=0;

//erro de rumo
int rumo_margin_flag=0; //1-> rumo está dentro da margem de erro estipulada | 0->não está dentro

//variáveis para transição para o estado 13
double deltat = 10000; //em ms
double timer_13=0;
int flag_13=0;

HMC5883L compass;

//variáveis bússola================================================================
float rumo_real;
float erro_rumo=0;
int get_buss=0;
float sum_buss=0;
float sum_erro_buss=0;
int amostra_buss=0;
//=================================================================================

TinyGPSPlus gps;
HardwareSerial SerialGPS(1);//RX2 e TX2

//variáveis GPS====================================================================

float rumo_ideal = 0;
float rumo_ideal_PID=0;
//double rumo_idel_save=0;
//double rumo_ideal = 170;
double lat_barco,long_barco;
//float lat_barco=-23.554703;
//float long_barco=-46.877897;

double lat_waypoint, long_waypoint;
float desvio_waypoint;
unsigned long dist_waypoint;

//lista de waypoints com respectivos desvios magnéticos
//float lat_long_desvio_waypoint[6] = {-23.556035, -46.877682, -21.42, -23.554985, -46.876259, -21.42};
//quintal
double lat_long_desvio_waypoint[6] = {-23.554781, -46.877785,-21.42, -23.554695, -46.877906,-21.42};
//guarapiranga
//double lat_long_desvio_waypoint[6] = {-23.694047, -46.730670,-21.45, -23.695619, -46.728572, -21.45, -23.695314, -46.732949,-21.45};
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

//variáveis para gravar leitura analógica dos pins dos sensores hall
int statusLEFT = 0;
int statusRIGHT = 0;
int statusFLEFT = 0;
int statusFRIGHT = 0;

//variável que resume estado de todos os hall
int status_Hall =B0000;

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

    SerialGPS.begin(9600, SERIAL_8N1,3, 1);
    
    servo.attach(servoPin);
    servo.write(pos_zero);

    //variáveis de estado
    current_state = 0;
    //previous_state = 0;
    waypoint_count = 0;

    lat_waypoint=lat_long_desvio_waypoint[0];
    long_waypoint=lat_long_desvio_waypoint[1];
    desvio_waypoint=lat_long_desvio_waypoint[2];

    //Serial.println("start timers ");
    timer0 = timerBegin(0, 8000, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 8000 -> 100000 ns = 100 us, countUp
    timerAttachInterrupt(timer0, &onTimer0, true); // edge (not level) triggered 
    timerAlarmWrite(timer0, 1000, true); // 1000 * 100 us = 0.1 s (10 Hz), autoreload true
    timerAlarmEnable(timer0); // enable

    timer1 = timerBegin(1, 8000, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 8000 -> 100000 ns = 100 us, countUp
    timerAttachInterrupt(timer1, &onTimer1, true); // edge (not level) triggered 
    timerAlarmWrite(timer1, 50000, true); // 50000 * 100 us = 5 s (0.2Hz), autoreload true
    timerAlarmEnable(timer1); // enable

    timer2 = timerBegin(2, 8000, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 8000 -> 100000 ns = 100 us, countUp
    timerAttachInterrupt(timer2, &onTimer2, true); // edge (not level) triggered 
    timerAlarmWrite(timer2, 500, true); // 500 * 100 us = 0.05 s (20Hz), autoreload true
    timerAlarmEnable(timer2); // enable

    while (!compass.begin()){
      Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
        //SerialBT.println("Could not find a valid HMC5883L sensor, check wiring!");
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

    //iniciar bluetooth definindo nome do dispositivo
    SerialBT.begin("ESP32test");
    
    //rotina de aquisicao inicial
    acquire_hall();
    acquire_GPS();
    acquire_buss();
    timer_PID=millis();
    time_servo=timer_PID;
    time_report=time_servo;

    float sum_rumo=0;

}

void loop() {

    char command_reading;

    if((command_flag == 2 || command_flag == 3) && (millis() - time_report > TIME_REPORT) ){
      bt_alt_report();  
    }
    
    if (SerialBT.available()){
    
        command_reading=SerialBT.read();
        
        switch(command_reading){
            case 'b': //iniciar calibração da bússola
                SerialBT.println("Calibrating...");
                calib_buss();
                break;
            
            case 'c': // ligar/desligar comunicação;
                SerialBT.println("Changing communication...");
                command_flag^=(1<<1); 
                break;
                
            case 'g': //gps info
                GPS_info_bt();
                break;
                
            case 'm': //ligar/desligar movimento
                SerialBT.println("Changing movement...");
                command_flag ^=1;
                break;
                   
            case 'r': //reiniciar
                SerialBT.println("Restarting...");
                delay(3000);
                ESP.restart();
                break;

            case 'w': //show waypoint
                SerialBT.print("Current waypoint= ");SerialBT.println(waypoint_count);
                break;
        }
    }
    delay(100);
    
    if(abs(SOMAE)>=100){
      SOMAE=0;  
    }

    
    
    if (flag_hall>0) {
        acquire_hall();
        GPS_info_bt();        
    }
    
    if(flag_gps>0) {
        acquire_GPS();
    }

    if(flag_buss>0){

        acquire_buss();
            
    }

    if((command_flag == 1 || command_flag == 3) && (millis()-time_servo > TIME_SERVO)){
        
        //mover servo apenas quando lê a bússola
        switch (current_state){
    
            case 0: //à favor-----------------------------------------------------------------------------------------------------

                pos=calc_PID(erro_rumo);
                servo.write(pos);
                //move_servo(calc_PID(erro_rumo));
                
                if(status_Hall==B1000 || status_Hall==B1100 || status_Hall==B0110 || status_Hall==B0100){
                    previous_state = current_state;
                    current_state=3; //ir para contra por BB 
                }
                else if(status_Hall==B0001 || status_Hall==B0011 || status_Hall==B0010){
                    previous_state = current_state;
                    current_state=4; //ir para contra por BE
                }

                else{
                previous_state = current_state;
                }
                
                break;

            case 3: //contra por BB------------------------------------------------------------------------------------------------
                
                if(previous_state!=3){
                    switch(previous_state){
                        case 0:
                            if(rumo_margin_flag==0){
                                cte=-constante;
                            }
                            else{
                                cte=0;
                            }
                            break;
                        case 4:
                            cte=constante;
                            break;
                        case 5:
                            cte=0;
                            break;
                        case 7:
                            cte=constante;
                            break;
                        case 9:
                            if(rumo_margin_flag==0){
                                cte=-constante;
                            }
                            else{
                                cte=0;
                            }
                            break;
                    }
                    rumo_ideal_PID=rumo_real+cte;                   
                }
                E=calc_erro_rumo(rumo_ideal_PID);
                Serial.print("E= "); Serial.println(E);
                pos=calc_PID(E);
                if(previous_state==12){
                    pos=0;
                }
                servo.write(pos);
                if(status_Hall==B0100 || status_Hall==B0110){
                    flag_13=1;
                    previous_state = current_state;
                    current_state=7; //ir para arribar_1 BB
                    cte=0;
                }
                else if(erro_rumo < -ang_jibe && abs(erro_rumo)>(90-lambda_jibe) && status_Hall==B1100){
                    flag_13=1;
                    previous_state = current_state;
                    current_state=11; //ir para Jib BB
                    cte=0;
                }
                else if(erro_rumo < - lambda_rumo && status_Hall==B1000){//rumo_ideal está para BB e pode orçar
                    previous_state = current_state;
                    current_state=9; //ir para orçar BB
                    cte=0;
                }
                else if(erro_rumo > lambda_rumo) {//rumo_ideal está para BE
                    previous_state = current_state;
                    current_state=5; //ir para arribar_2 BB
                    cte=0;
                }

                else if(status_Hall==B0000){
                    previous_state = current_state;
                    current_state=0; //ir para à favor
                    cte=0;
                }

                else if(status_Hall==B0001 || status_Hall==B0011 || status_Hall==B0010){
                    previous_state = current_state;
                    current_state=4; //ir para contra por BE
                    cte=0;
                }
                else{
                previous_state = current_state;
                }
                break;

            case 4: //contra por BB------------------------------------------------------------------------------------------------
            
                if(previous_state!=4){
                    switch(previous_state){
                        case 0:
                            if(rumo_margin_flag==0){
                                cte=constante;
                            }
                            else{
                                cte=0;
                            }
                            break;
                        case 3:
                            cte=-constante;
                            break;
                        case 6:
                            cte=0;
                            break;
                        case 8:
                            cte=-constante;
                            break;
                        case 10:
                            if(rumo_margin_flag==0){
                                cte=constante;
                            }
                            else{
                                cte=0;
                            }
                            break;
                    }
                    rumo_ideal_PID=rumo_real+cte;                   
                }
                E=calc_erro_rumo(rumo_ideal_PID);
                Serial.print("E= "); Serial.println(E);
                pos=calc_PID(E);
                if(previous_state==11){
                    pos=0;
                }
                servo.write(pos);

                if(status_Hall==B0010 || status_Hall==B0110){
                    flag_13=1;
                    previous_state = current_state;
                    current_state=8; //ir para arribar_1 BE
                    cte=0;   
                }
                else if(erro_rumo > ang_jibe && abs(erro_rumo)>(90-lambda_jibe) && status_Hall==B0011){
                    flag_13=1;
                    previous_state = current_state;
                    current_state=12; //ir para Jib BE
                    cte=0;        
                }
                else if(erro_rumo > lambda_rumo && status_Hall==B0001){//rumo_ideal está para BE e pode orçar
                    previous_state = current_state;
                    current_state=10; //ir para orçar BE
                    cte=0;    
                }
                else if(erro_rumo < -lambda_rumo) {//rumo_ideal está para BB
                    previous_state = current_state;
                    current_state=6; //ir para arribar_2 BE
                    cte=0;
                }
                
                else if(status_Hall==B1000 || status_Hall==B1100 || status_Hall==B0100){
                    previous_state = current_state;
                    current_state=3; //ir para contra por BB
                    cte=0;
                }

                else if(status_Hall==B0000){
                    previous_state = current_state;
                    current_state=0; //ir para à favor
                    cte=0;
                }
                else{
                previous_state = current_state;
                }
                break;

            case 5: //arribar_2 BB-------------------------------------------------------------------------------------------------
            
                pos=calc_PID(erro_rumo);
                servo.write(pos);

                if((status_Hall!=B1000 && status_Hall!=B1100) || abs(erro_rumo)<lambda_rumo || erro_rumo<-lambda_rumo) {
                    previous_state = current_state;
                    current_state=3; //ir para contra por BB
                }
                else{
                previous_state = current_state;
                }
                break;

            case 6: //arribar_2 BE-------------------------------------------------------------------------------------------------
            
                pos=calc_PID(erro_rumo);
                servo.write(pos);

                if((status_Hall!=B0011 && status_Hall!=B0001)  || abs(erro_rumo)<lambda_rumo || erro_rumo>lambda_rumo) {
                    previous_state = current_state;
                    current_state=4; //ir para contra por BE
                }
                else{
                previous_state = current_state;
                }
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
                else if(millis()-timer_13 > deltat ){
                    flag_13=1;
                    previous_state = current_state;
                    current_state=13; //ir para espera
                }
                else{
                previous_state = current_state;
                }
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
                else{
                previous_state = current_state;
                }
                break;

            case 9: //orçar BB-----------------------------------------------------------------------------------------------------
            
                SOMAE=0;
                pos=calc_PD(-10);
                servo.write(pos);

                if((status_Hall!=B1000)|| abs(erro_rumo)<lambda_rumo || erro_rumo>lambda_rumo) {
                    previous_state = current_state;
                    current_state=3; //ir para contra por BB
                }
                else{
                previous_state = current_state;
                }
                break;

            case 10: //orçar BE----------------------------------------------------------------------------------------------------
            
                SOMAE=0;
                pos=calc_PD(10);
                servo.write(pos);

                if((status_Hall!=B0001) || abs(erro_rumo)<lambda_rumo || erro_rumo<-lambda_rumo) {
                    previous_state = current_state;
                    current_state=4; //ir para contra por BE
                }
                else{
                previous_state = current_state;
                }
                break;

            case 11: //Jibe BB------------------------------------------------------------------------------------------------------

                if (flag_13==1){
                    timer_13=millis();
                    flag_13=0;
                }
                
                SOMAE=0;
                servo.write(leme_max);

                if(abs(erro_rumo)<lambda_jibe || (status_Hall!=B1100 && status_Hall!=B1000 && status_Hall!=B0000 && status_Hall!=B0100)){
                    previous_state = current_state;
                    current_state=4; //ir para contra por BE
                }

                else if(millis()-timer_13 > deltat ){
                    flag_13=1;
                    previous_state = current_state;
                    current_state=13; //ir para espera
                }
                else{
                previous_state = current_state;
                }
                break;

            case 12: //Jibe BE------------------------------------------------------------------------------------------------------

                if (flag_13==1){
                    timer_13=millis();
                    flag_13=0;
                }

                SOMAE=0;
                servo.write(leme_min);

                if(abs(erro_rumo)<lambda_jibe || (status_Hall!=B0011 && status_Hall!=B0001 && status_Hall!=B0000 && status_Hall!=B0010)) {
                    previous_state = current_state;
                    current_state=3; //ir para contra por BB
                }

                else if(millis()-timer_13 > deltat ){
                    flag_13=1;
                    previous_state = current_state;
                    current_state=13; //ir para espera
                }
                else{
                previous_state = current_state;
                }
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
                else{
                previous_state = current_state;
                }
                break;
        }
    }
}
