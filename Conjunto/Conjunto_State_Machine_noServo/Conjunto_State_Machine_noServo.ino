#include <SoftwareSerial.h>
#include <Wire.h>
#include <HMC5883L.h>
#include <TinyGPS.h>

HMC5883L compass;


SoftwareSerial serial1(6, 7); // RX, TX conectar invertido com as portas do GPS
TinyGPS gps1;

//variáveis de estado da máquina
int current_state = 1;
int previous_state = 0;
int wind; //0 para bombordo, 1 para boreste
bool stable; //variável para prender processo em loop

//parâmetros de erro

//erro de rumo
int zero_margin_rumo=10;
int lambda_rumo= 15;
float erro_rumo=0;

//variáveis bússola
float rumo_real;


//lista de waypoints com respectivos desvios magnéticos
float lat2_lon2_desvio[] = {-23.578426, -46.744687, -21.32, -23580636, -46.743040, -21.32};
int waypoint_count = 0;

//variáveis HALL=============================
int status_Hall[4]={0,0,0,0};

//Nível lógico máximo para representar ativação do hall
int maxHallSignal = 10;

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

//variáveis dos Interrupts

const uint8_t t_load = 0;
const uint8_t t0_cicles = 195; //oara prescale=1024 com amostragem ~80Hz
const uint16_t contador_gps =0;


void acquire_GPS() {
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

void acquire_Hall(){

  int statusRIGHT = analogRead(pinRIGHT);
  int statusLEFT = analogRead(pinLEFT);
  int statusFRIGHT = analogRead(pinFRIGHT);
  int statusFLEFT = analogRead(pinFLEFT);

  status_Hall[0]=0;
  status_Hall[1]=0;
  status_Hall[2]=0;
  status_Hall[3]=0;

  if ( statusFRIGHT < maxHallSignal){
    status_Hall[3]=1;
    }
  if ( statusRIGHT < maxHallSignal){
    status_Hall[2]=1;
    }
  if ( statusFLEFT < maxHallSignal){
    status_Hall[0]=1;
    }
  if ( statusLEFT < maxHallSignal){
    status_Hall[1]=1; 
    }}
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
  // For Liberdade, São Paulo / Brazil declination angle is 21'34W(negative)
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

}

int check_rumo(){
  //de acordo com a última leitura da bússola, verifica o desvio do rumo real

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

  if (abs(delta_rumo) > 80 && abs(delta_rumo) < 90 ){
    current_state=5; //ir para cambar
    return false;
    }

  else if (abs(erro_rumo) > lamda_rumo){
    current_state=4; //ir para ajeitar_rumo
    return false;
    }
  
  else{
    return true;
    }
  }

void setup() {

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
  current_state = 1;
  previous_state = 0;
  waypoint_count = 0;

  //settar direção inicial do vento
  //wind= B0; //se bombordo
  wind= B1; //se boreste

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
  compass.setOffset(95, -3);
  
}

void loop() {

switch (current_state){
  
  case 0: //troca_waypoint

  
  waypoint_count = waypoint_count + 1;
  lat2=lat2_lon2_desvio[waypoint_count*3];
  lon2=lat2_lon2_desvio[waypoint_count*3+1];
  desvio2=lat2_lon2_desvio[waypoint_count*3+2];

  current_state=1; //ir para manter_rumo_BE
  previous_state=0;
  
  break;

  
  case 1: //manter_rumo
  //manter servo parado na posição
  stable = true;
  //pode inserir um timer pra não deixar o código preso nesse loop
  while (stable) {

    switch (wind){
    
    case 0: //vento de bombordo
    if ( status_Hall == {0,1,0,0}){
      current_state=2; //ir para arribar
      stable = false;
      
      }

    else if ( status_Hall == {1,0,0,0}){
      current_state=2; //ir para orçar
      stable = false;
      }
    break;

    case 1: //vento de boreste
    if ( status_Hall == {0,0,1,0}){
      current_state=2; //ir para arribar
      stable = false;
      
      }

    else if ( status_Hall == {0,0,0,1}){
      current_state=2; //ir para orçar
      stable = false;
      }
    break;
  }
    stable=check_rumo();

    }
  
  previous_state=1;
  break;

  
  case 2: //arribar
  stable = false;

  //talvez colocar algum timer pra tirar o código desse loop.
  while (!stable){
    

    switch (wind){
    
    case 0: //vento de bombordo
    if (status_Hall != {1,1,0,0}){
    //mover leme para bombordo
      }
    
    else{
      stable=true;
    }
    
    }
    break;

    case 1: //vento de boreste
    if (status_Hall != {0,0,1,1}){
    //mover leme para boreste
      }
    
    else{
      stable=true;
    }
    break;

    }
  previous_state=2;
  break;
  
  case 3: //orçar
  stable = false;
  //talvez colocar algum timer pra tirar o código desse loop.
  while (!stable){
    

    switch (wind){
    
    case 0: //vento de bombordo
    if (status_Hall != {1,1,0,0}){
    //mover leme para boreste
    
      }
    
    else{
      stable=true;
    }
    
    }
    break;

    case 1: //vento de boreste
    if (status_Hall != {0,0,1,1}){
    //mover leme para bombordo
    
      }
    
    else{
      stable=true;
    }
    break;

    }
  current_state=1; //voltar para manter_rumo
  previous_state=3;
  break;

  
  case 4: //ajeitar_rumo
  

  previous_state=4;
  break;

  
  case 5: //cambar

  bool stable = false;

  while (!stable){
    

    //mover incisivamente leme para o lado oposto do vento

    stable=check_rumo();
    //função_move_servo(delta_rumo,case_rumo)
    
  }


  wind = ~wind; //inverte de onde vem o vento
  current_state=1; //retorna para manter_rumo
  previous_state=5;
  break;
  
  }
   
}

ISR(TIMER0_COMPA_vct){
  acquire_Hall();
  acquire_buss();
  contador_gps+=1;
  if contador_gps == 
}
