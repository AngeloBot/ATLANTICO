#include <SoftwareSerial.h>
#include <Wire.h>
#include <HMC5883L.h>
#include <TinyGPS.h>
#include <Servo.h>

HMC5883L compass;

Servo servo; 


//variáveis servo
int pos;
int pinServo=11;
int pos_zero=90;

//o sentido crescente do servo é sentido anti-horário, portanto, para virar o barco para BB, deve-se diminuir o ângulo e vice-versa
int acres_BB=-30;
int acres_BE=30;

SoftwareSerial serial1(6, 7); // RX, TX conectar invertido com as portas do GPS
TinyGPS gps1;

//variáveis de estado da máquina
volatile int current_state = 0;
volatile int previous_state = 0;
bool stable; //variável para prender processo em loop

//parâmetros de erro

//erro de rumo
//convencionando que erros de rumo <0 significam que o waypoint está à BB
//convencionando que erros de rumo >0 significam que o waypoint está à BE
int zero_margin_rumo=10;
int lambda_rumo= 10;
int lambda_jibe=5;
volatile float erro_rumo=0;

//variáveis bússola
volatile float rumo_real;


//lista de waypoints com respectivos desvios magnéticos
float lat_lon_desvio_waypoint[] = {-23.578426, -46.744687, -21.32, -23580636, -46.743040, -21.32};
volatile int waypoint_count = 0;

//variáveis HALL=============================
volatile int status_Hall = B0;

//Nível lógico máximo para representar ativação do hall
int maxHallSignal = 10;

//pin para input proveniente sensores
int pinLEFT = A1;
int pinRIGHT = A2; 
int pinFLEFT = A0;
int pinFRIGHT = A3;

//pin para output nos leds
int LpinLEFT = 7;
int LpinRIGHT = 4;
int LpinFLEFT = 8;
int LpinFRIGHT = 2;

//variáveis para o GPS
volatile float rumo_ideal = 0;
volatile float lat_barco, long_barco;
volatile float lat_waypoint, long_waypoint;
//unsigned long sentido;

//variáveis dos Interrupts

const uint8_t t_load = 0;
const uint8_t t0_cicles = 255; //para prescale=1024 com amostragem ~61.2745Hz
volatile int contador_gps =0;


void acquire_GPS() {
  //receber dados de posição do GPS e definir o rumo ideal
  bool recebido = false;
  static unsigned long delayPrint;

  while (serial1.available()) {
     char cIn = serial1.read();
     recebido = (gps1.encode(cIn) || recebido);  //Verifica até receber o primeiro sinal dos satelites
  }

  if (recebido) {

     unsigned long idadeInfo;
     gps1.f_get_position(&latitude, &longitude, &idadeInfo);   //O método f_get_position é mais indicado para retornar as coordenadas em variáveis float, para não precisar fazer nenhum cálculo    

     //sentido = gps1.course();
     
     rumo_ideal = gps1.course_to(lat_barco, long_barco, lat_waypoint, long_waypoint);
    
    if (waypoint_radius > gps1.distance_between(at_barco, long_barco, lat_waypoint, long_waypoint)){
      waypoint_count += 1;
      lat_waypoint=lat_lon_desvio_waypoint[waypoint_count*3];
      long_waypoint=lat_lon_desvio_waypoint[waypoint_count*3+1];
      desvio_waypoint=lat_lon_desvio_waypoint[waypoint_count*3+2];
      
    }

  }
}

void acquire_Hall(){

  int statusRIGHT = analogRead(pinRIGHT);
  int statusLEFT = analogRead(pinLEFT);
  int statusFRIGHT = analogRead(pinFRIGHT);
  int statusFLEFT = analogRead(pinFLEFT);

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


float acquire_buss(){
  //lê a bússola e guarda o valor

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
  float declinationAngle = (-21.0 - (34.0 / 60.0)) / (180 / M_PI);
  rumo_real += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (rumo_real < 0)
  {
    rumo_real += 2 * PI;
  }

  if (rumo_real > 2 * PI)
  {
    rumo_real -= 2 * PI;
  }

  // Converter para graus e guardar na variável apropriada
  rumo_real = rumo_real * 180/M_PI;
  float delta_rumo = rumo_real - rumo_ideal;
  
  if (((delta_rumo) < 0 && (delta_rumo) >= -180) || ((delta_rumo) > 0 && (delta_rumo) <= 180)) {
        erro_rumo = delta_rumo;
       }
     
  else if (((delta_rumo) < 360  && (delta_rumo) > 180) || ((delta_rumo) > -360 && (delta_rumo) < -180)) {
       erro_rumo = abs(abs(delta_rumo) - 360);
       }

  else if ((delta_rumo <= zero_margin_rumo && delta_rumo >= -zero_margin_rumo) || (delta_rumo <= -350) || (delta_rumo >= 350)){
       erro_rumo = 0;
      }

}

void setup() {

  //config servo

  servo.attach(pinServo);
  servo.write(pos_zero);


  //Configuração dos Interrupts
  //os timers usados serão o 0, temos frequencia de amostragem de ~80Hz (T=0.1248s)

  //Reset Timer0 Control Reg A
  TCCR0A = 0; 
  //Set to prescaler of 1024 of timer 0
  TCCR0B |= (1<<CS12);
  TCCR0B &= ~(1<<CS11);
  TCCR0B |= (1<<CS10);

  //Reset Timer0 and set compare value
  TCNT0 = t_load;
  OCR0A = t0_cicles; 
  //Enable Timer0 compare interrupt
  TIMSK0 = (1<<OCIE0A); //buss
  //Enable global interrupts
  sci();

  //variáveis de estado
  current_state = 0;
  previous_state = 0;
  waypoint_count = 0;

  lat_waypoint=lat_lon_desvio_waypoint[0];
  long_waypoint=lat_lon_desvio_waypoint[1];
  desvio_waypoint=lat_lon_desvio_waypoint[2];
  
  Serial.println("Esperando por Dados do Modulo...");
  serial1.begin(9600);
  Serial.begin(9600);
  
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

  // Initialize Initialize HMC5883L
  Serial.println("Initialize HMC5883L");
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }

  // Set measurement range : (resolução) [realce]
  // +/- 0.88 Ga: (0.73mG) [1370] HMC5883L_RANGE_0_88GA
  // +/- 1.30 Ga: (0.92mG) [1090] HMC5883L_RANGE_1_3GA (default)
  // +/- 1.90 Ga: (1.22mG) [820] HMC5883L_RANGE_1_9GA
  // +/- 2.50 Ga: (1.52mG) [660] HMC5883L_RANGE_2_5GA
  // +/- 4.00 Ga: (2.27mG) [440] HMC5883L_RANGE_4GA
  // +/- 4.70 Ga: (2.56mG) [390] HMC5883L_RANGE_4_7GA
  // +/- 5.60 Ga: (3.03mG) [330] HMC5883L_RANGE_5_6GA
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  // Soneca:           HMC5883L_IDLE
  // Medição Única:    HMC5883L_SINGLE
  // Medição Contínua: HMC5883L_CONTINOUS (default)
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  //  0.75Hz: HMC5883L_DATARATE_0_75HZ
  //  1.50Hz: HMC5883L_DATARATE_1_5HZ
  //  3.00Hz: HMC5883L_DATARATE_3HZ
  //  7.50Hz: HMC5883L_DATARATE_7_50HZ
  // 15.00Hz: HMC5883L_DATARATE_15HZ (default)
  // 30.00Hz: HMC5883L_DATARATE_30HZ
  // 75.00Hz: HMC5883L_DATARATE_75HZ
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  // 1 sample:  HMC5883L_SAMPLES_1 (default)
  // 2 samples: HMC5883L_SAMPLES_2
  // 4 samples: HMC5883L_SAMPLES_4
  // 8 samples: HMC5883L_SAMPLES_8
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(116, -47);
  
}

void loop() {

  switch (current_state){
  
    case 0: //à favor-------------------------------------------------------------------------------------------------------

      pos=pos_zero
      servo.write(pos);

      if(status_Hall==B1000 || status_Hall==B1100 || status_Hall==B0110 || status_Hall==B0100){
        current_state=3; //ir para contra por BB
        break;

      }
      else if(status_Hall==B0001 || status_Hall==B0011 || status_Hall==B0010){
        current_state=4; //ir para contra por BE
        break;
      }
      else if(erro_rumo > 0 && status_Hall==B0000){
        current_state=1; //ir para ajeitar BB
        break;
      }
      else if(erro_rumo < 0 && status_Hall==B0000){
        current_state=2; //ir para ajeitar BE
      }
      break;

    case 1: //ajeitar BB---------------------------------------------------------------------------------------------------

      //leme p/ BB
      pos+=acres_BB;
      servo.write(pos);

      if(erro_rumo == 0 || status_Hall != B0){
        current_state=0; //ir para à favor
      }
      break;

    case 2: //ajeitar BE---------------------------------------------------------------------------------------------------

      //leme p/ BE
      pos+=acres_BE;
      servo.write(pos);

      if(erro_rumo == 0 || status_Hall != B0){
        current_state=0; //ir para à favor
      }
      break;

    case 3: //contra por BB------------------------------------------------------------------------------------------------
      
      //manter leme
      pos=pos_zero;
      servo.write(pos);
      
      if(status_Hall==B0100 || status_Hall==B0110){
        current_state=7; //ir para arribar_1 BB
        break;

      }
      else if(abs(erro_rumo)<(90+lambda_jibe) && abs(erro_rumo)>(90-lambda_jibe) && status_Hall==B1100){
        current_state=11; //ir para Jibe BB
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

      else if(status_Hall==B0001 || status_Hall==B0011 || status_Hall==B0010) {
        current_state=4; //ir para contra por BE
      }
      break;

      else if(status_Hall==B0000){
        current_state=0; //ir para à favor

      }

    case 4: //contra por BE------------------------------------------------------------------------------------------------
    
      //manter leme
      pos=pos_zero;
      servo.write(pos);

      if(status_Hall==B0010 || status_Hall==B0110){
        current_state=8; //ir para arribar_1 BE
        break;
      }
      else if(abs(erro_rumo)<(90+lambda_jibe) && abs(erro_rumo)>(90-lambda_jibe)&& status_Hall==B0011){
        current_state=12; //ir para Jibe BE
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
      else if(status_Hall==B1000 || status_Hall==B1100 || status_Hall==B0100) {
        current_state=3; //ir para contra por BB
      }
      break;

      else if(status_Hall==B000){
        current_state=0; //ir para à favor

      }
    case 5: //arribar_2 BB-------------------------------------------------------------------------------------------------
    
      //leme p/ BE

      if(status_Hall==B1000 || status_Hall==B1100 || erro_rumo==0) {
        current_state=3; //ir para contra por BB
      }
      break;

    case 6: //arribar_2 BE-------------------------------------------------------------------------------------------------
    
      //leme p/ BB

      if(status_Hall==B0001 || status_Hall==B0011 || erro_rumo==0) {
        current_state=4; //ir para contra por BE
      }
      break;

    case 7: //arribar_1 BB-------------------------------------------------------------------------------------------------
    
      //leme p/ BE

      if(status_Hall==B0110 || status_Hall==B0100) {
        current_state=3; //ir para contra por BB
      }
      break;
    
      //ADICIONAR CONDIÇÃO DE TIMER PARA O IR PARA O ESTADO 13

    case 8: //arribar_1 BE-------------------------------------------------------------------------------------------------
    
      //leme p/ BB

      if(status_Hall==B0110 || status_Hall==B0010) {
        current_state=4; //ir para contra por BE
      }
      break;

      //ADICIONAR CONDIÇÃO DE TIMER PARA O IR PARA O ESTADO 13

    case 9: //orçar BB-----------------------------------------------------------------------------------------------------
    
      //leme p/ BB

      if(!(status_Hall==B0000 || status_Hall==B1000 || status_Hall==B0010) || erro_rumo==0) {
        current_state=3; //ir para contra por BB
      }
      break;

    case 10: //orçar BE----------------------------------------------------------------------------------------------------
    
      //leme p/ BE
      pos=pos_centro;
      servo.write(pos);

      if(!(status_Hall==B0000 || status_Hall==B0100 || status_Hall==B0001) || erro_rumo==0 {
        current_state=4; //ir para contra por BE
      }
      break;

    case 11: //Jibe BB------------------------------------------------------------------------------------------------------
    
      //leme p/ BE
      pos=pos_centro+acres_BE;
      servo.write(pos);

      if(erro_rumo==0 || status_Hall==B0011 || status_Hall==B0001){
        current_state=4; //ir para contra por BE
      }
      break;

      //ADICIONAR CONDIÇÃO DE TIMER PARA O IR PARA O ESTADO 13

    case 12: //Jibe BE----------------------------------------------------------------------------------------------------
    
      //leme p/ BB
      pos=pos_centro+acres_BB;
      servo.write(pos);

      if(erro_rumo==0 || status_Hall==B1000 || status_Hall==B1100) {
        current_state=3; //ir para contra por BB
      }
      break;

      //ADICIONAR CONDIÇÃO DE TIMER PARA O IR PARA O ESTADO 13

    case 13: //espera------------------------------------------------------------------------------------------------------
    
      //leme ao centro
      pos=pos_centro;
      servo.write(pos);
      //ADICIONAR CONDIÇÃO DE TIMER PARA O IR PARA O ESTADO 0
      break;

ISR(TIMER0_COMPA_vct){
  acquire_Hall();
  acquire_buss();
  contador_gps+=1;
  if (contador_gps == 1200){
    acquire_GPS();
    contador_gps=0;
  } 
}
