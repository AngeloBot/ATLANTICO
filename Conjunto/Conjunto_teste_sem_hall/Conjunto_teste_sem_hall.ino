#include <SoftwareSerial.h>
#include <Wire.h>
#include <HMC5883L.h>
#include <TinyGPS.h>

HMC5883L compass;


SoftwareSerial serial1(6, 7); // RX, TX
TinyGPS gps1;

int statusLEFT = 0;
int statusRIGHT = 0;
int statusFLEFT = 0;
int statusFRIGHT = 0;

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

void setup() {

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
  compass.setOffset(108, 39);
  
}

void loop() {

//#############################################################Hall

//#############################################################Bússola

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

  // Output
  Serial.print(" Heading = ");
  Serial.print(heading);
  Serial.print(" Degress = ");
  Serial.print(headingDegrees);
  Serial.println();

//#############################################################GPS
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
     float latitude, longitude; //As variaveis podem ser float, para não precisar fazer nenhum cálculo
     
     unsigned long idadeInfo;
     gps1.f_get_position(&latitude, &longitude, &idadeInfo);   //O método f_get_position é mais indicado para retornar as coordenadas em variáveis float, para não precisar fazer nenhum cálculo    

     if (latitude != TinyGPS::GPS_INVALID_F_ANGLE) {
        Serial.print("Latitude: ");
        Serial.println(latitude, 6);  //Mostra a latitude com a precisão de 6 dígitos decimais
     }

     if (longitude != TinyGPS::GPS_INVALID_F_ANGLE) {
        Serial.print("Longitude: ");
        Serial.println(longitude, 6);  //Mostra a longitude com a precisão de 6 dígitos decimais
     }

     if ( (latitude != TinyGPS::GPS_INVALID_F_ANGLE) && (longitude != TinyGPS::GPS_INVALID_F_ANGLE) ) {
        Serial.print("Link para Google Maps:   https://maps.google.com/maps/?&z=10&q=");
        Serial.print(latitude,6);
        Serial.print(",");
        Serial.println(longitude, 6);           
     }

     if (idadeInfo != TinyGPS::GPS_INVALID_AGE) {
        Serial.print("Idade da Informacao (ms): ");
        Serial.println(idadeInfo);
     }



     //sentito (em centesima de graus)
     unsigned long sentido;
     sentido = gps1.course();

     Serial.print("Sentido (grau): ");
     Serial.println(float(sentido) / 100, 2);


     //satelites e precisão
     unsigned short satelites;
     unsigned long precisao;
     satelites = gps1.satellites();
     precisao =  gps1.hdop();

     if (satelites != TinyGPS::GPS_INVALID_SATELLITES) {
        Serial.print("Satelites: ");
        Serial.println(satelites);
     }

     if (precisao != TinyGPS::GPS_INVALID_HDOP) {
        Serial.print("Precisao (centesimos de segundo): ");
        Serial.println(precisao);
     }


     //float distancia_entre;
     //distancia_entre = gps1.distance_between(lat1, long1, lat2, long2);

     //float sentido_para;
     //sentido_para = gps1.course_to(lat1, long1, lat2, long2);
  }
}
