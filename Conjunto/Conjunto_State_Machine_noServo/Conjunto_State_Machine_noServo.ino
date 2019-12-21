#include <SoftwareSerial.h>
#include <Wire.h>
#include <HMC5883L.h>
#include <TinyGPS.h>

HMC5883L compass;


SoftwareSerial serial1(6, 7); // RX, TX conectar invertido com as portas do GPS
TinyGPS gps1;

//parâmetros de erro

//erro de rumo
int zero_margin_rumo=10;
int lambda_rumo= 15;

//variáveis bússola
float rumo_real;


//lista de waypoints com respectivos desvios magnéticos
float lat2_lon2_desvio[] = {-23.578426, -46.744687, -21.32, -23580636, -46.743040, -21.32};


//variáveis HALL
int statusLEFT = 0;
int statusRIGHT = 0;
int statusFLEFT = 0;
int statusFRIGHT = 0;
int maxHallSignal = 10;

int array_Hall[] = {0,0,0,0};


//pin para input proveniente sensores
int pinLEFT = A1;
int pinRIGHT = A2; 
int pinFLEFT = A0;
int pinFRIGHT = A3;

//pin para output nos leds
int LpinLEFT = 5;
int LpinRIGHT = 4;
int LpinFLEFT = 8;
int LpinFRIGHT = 2;

//variáveis para o GPS
float rumo_ideal = 0;
float latitude, longitude;
float lat2, long2;
unsigned long sentido;

void lerGPS() {
  //receber dados de posição do GPS e definir o rumo ideal
  bool recebido = false;
  static unsigned long delayPrint;

  while (serial1.available()) {
     char cIn = serial1.read();
     recebido = (gps1.encode(cIn) || recebido);  //Verifica até receber o primeiro sinal dos satelites
  }

  if ( (recebido) && ((millis() - delayPrint) > 1000) ) {  //Mostra apenas após receber o primeiro sinal. Após o primeiro sinal, mostra a cada segundo.
     delayPrint = millis();
     
     Serial.println("----------------------------------------");
     
     //Latitude e Longitude
     //long latitude, longitude; 
     
     
     unsigned long idadeInfo;
     gps1.f_get_position(&latitude, &longitude, &idadeInfo);   //O método f_get_position é mais indicado para retornar as coordenadas em variáveis float, para não precisar fazer nenhum cálculo    

     if (latitude != TinyGPS::GPS_INVALID_F_ANGLE) {
        //Serial.print("Latitude: ");
        //Serial.println(latitude, 6);  //Mostra a latitude com a precisão de 6 dígitos decimais
     }

     if (longitude != TinyGPS::GPS_INVALID_F_ANGLE) {
        //Serial.print("Longitude: ");
        //Serial.println(longitude, 6);  //Mostra a longitude com a precisão de 6 dígitos decimais
     }

     if ( (latitude != TinyGPS::GPS_INVALID_F_ANGLE) && (longitude != TinyGPS::GPS_INVALID_F_ANGLE) ) {
        Serial.print("Link para Google Maps:   https://maps.google.com/maps/?&z=10&q=");
        Serial.print(latitude,6);
        Serial.print(",");
        Serial.println(longitude, 6);           
     }

     if (idadeInfo != TinyGPS::GPS_INVALID_AGE) {
        //Serial.print("Idade da Informacao (ms): ");
        //Serial.println(idadeInfo);
     }



     //sentito (em graus)
     sentido = gps1.course();

     Serial.print("Sentido (grau): ");
     Serial.println(float(sentido), 2);
     

     //satelites e precisão
     unsigned short satelites;
     unsigned long precisao;
     satelites = gps1.satellites();
     precisao =  gps1.hdop();

     if (satelites != TinyGPS::GPS_INVALID_SATELLITES) {
        //Serial.print("Satelites: ");
        //Serial.println(satelites);
     }

     if (precisao != TinyGPS::GPS_INVALID_HDOP) {
        //Serial.print("Precisao (centesimos de segundo): ");
        //Serial.println(precisao);
     }
     //float distancia_entre;
     //distancia_entre = gps1.distance_between(lat1, long1, lat2, long2);

     
     rumo_ideal = gps1.course_to(latitude, longitude, lat2, long2);
  }
}

void lerHALL(){
  statusRIGHT = analogRead(pinRIGHT);
  statusLEFT = analogRead(pinLEFT);
  statusFRIGHT = analogRead(pinFRIGHT);
  statusFLEFT = analogRead(pinFLEFT);


  if ( statusFRIGHT < maxHallSignal){

    array_Hall[3]=1;
    }
  else if ( statusFLEFT < maxHallSignal) {
    array_Hall[0]=1;
    }

  else if ( statusLEFT < maxHallSignal) {
    array_Hall[1]=1;
    }

  else if ( statusRIGHT < maxHallSignal) {
    array_Hall[2]=1;
    }
  
  }


float acquire_buss(){
    Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
  float headingDegrees = heading * 180/M_PI; 

  //adicionar parte que guarda na lista a informação q é lidaS

}

bool check_rumo(){
  
  // Output
  //Serial.print(" Degress = ");
  //Serial.print(headingDegrees);
  //Serial.println();

  float delta_rumo = rumo_real - rumo_ideal;
  
  if (((delta_rumo) < 0 && (delta_rumo) >= -180) || ((delta_rumo) > 0 && (delta_rumo) <= 180)) {
        erro_rumo = abs(delta_rumo);
       }
     
  else if (((delta_rumo) < 360  && (delta_rumo) > 180) || ((delta_rumo) > -360 && (delta_rumo) < -180)) {
        erro_rumo = abs(abs(delta_rumo) - 360);
       }

  else if ((delta_rumo <= zero_margin_rumo && delta_rumo >= -zero_margin_rumo) || (delta_rumo <= -350) || (delta_rumo >= 350)){
       erro_rumo = 0;
      }

  if (abs(erro_rumo) > 80 && abs(erro_rumo) < 90 ){

    return 0; //dar bordo
    }

  else if (abs(erro_rumo) > lamda_rumo){

    return 1; //precisa ajeitar rumo
    }
  
  else{
    return 2; //não é necessário ajustar rumo
    }
  }

void setup() {


  int current_state = 1;
  int previous_state = 0;
  int waypoint_count = 0;

  lat2=lat2_lon2_desvio[0];
  lon2=lat2_lon2_desvio[1];
  desvio2=lat2_lon2_desvio[2];
  
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

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(95, -3);
  
}

void loop() {

switch (current_state){
  
  case 0: //troca_waypoint_BE

  
  waypoint_count = waypoint_count + 1;

  

  delay(1000);

  current_state=1; //ir para manter_rumo_BE
  previous_state=0;
  
  break;

  
  case 1: //manter_rumo_BE

  //colocar servo no centro

  bool stable = true;
  while stable {

    //pode inserir um timer pra não deixar o código preso nesse loop

    int hall_state = lerHall();

    if ( hall_state = {0,0,1,0}){
      current_state=2; //ir para arribar_BE
      previous_state=1;
      
      }

    else if ( hall_state = {0,0,0,1}){
      current_state=2; //ir para orçar_BE
      previous_state=1;
      }

    int check_rumo=check_rumo();
    
    else if (check_rumo==1){
      current_state=4; //ir para ajeitar_rumo_BE
      previous_state=1;
      }

    else if (check_rumo==0){
      current_state=5; //ir para cambar_BE
      previous_state=1;
      }
    
    }
  
  break;

  
  case 2: //arribar_BE
  break;

  
  case 3: //orçar_BE
  break;

  
  case 4: //ajeitar_rumo_BE
  break;

  
  case 5: //cambar_BE
  break;

  case 6: //
  break;
  
  }
  


     
  
}
