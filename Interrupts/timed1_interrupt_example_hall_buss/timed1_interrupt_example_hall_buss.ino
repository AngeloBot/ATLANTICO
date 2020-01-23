#include <Wire.h>
#include <HMC5883L.h>
HMC5883L compass;

//Pins
const int led_pin = PB5; //pin 13 interno

//Counter and compare values
const uint16_t t1_load =0;
const uint16_t t1_comp = 31250;

//parâmetros de erro

//erro de rumo
int zero_margin_rumo=10;
int lambda_rumo= 15;
float erro_rumo=0;

//variáveis bússola
float rumo_real;


//variáveis HALL=============================
int status_Hall[4]={0,0,0,0};

//Nível lógico máximo para representar ativação do hall
int maxHallSignal = 10;

//pin para input proveniente sensores
int pinLEFT = A1;
int pinRIGHT = A2; 
int pinFLEFT = A0;
int pinFRIGHT = A3;

void setup() {

    //Set LED pin to be output
    DDRB |= (1<< led_pin);

    //Reset Timer1 Control Reg A
    TCCR1A = 0;

    //Set to prescaler of 256
    TCCR1B |= (1<<CS12);
    TCCR1B &= ~(1<<CS11);
    TCCR1B &= ~(1<<CS10);

    //Reset Timer1 and set compare value
    TCNT1 = t1_load;
    OCR1A = t1_comp;

    //Enable Timer1 compare interrupt
    TIMSK1 = (1<<OCIE1A);

    //Enable global interrupts
    sei();

    //Serial.println("Esperando por Dados do Modulo...");
    Serial.begin(9600);
    
    // Initialize Initialize HMC5883L
    //Serial.println("Initialize HMC5883L");
    while (!compass.begin())
    {
        Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
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
    compass.setOffset(118, -48);

}

void loop(){
    delay(500);
    Serial.print("HALL ");
    Serial.print(status_Hall[0]);
    Serial.print(" ");
    Serial.print(status_Hall[1]);
    Serial.print(" ");
    Serial.print(status_Hall[2]);
    Serial.print(" ");
    Serial.println(status_Hall[3]);
    Serial.print("BUSS ");
    Serial.println(rumo_real);
}

ISR(TIMER1_COMPA_vect){

    TCNT1 =t1_load;
    PORTB ^= (1<<led_pin);
    
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
        }
    

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
  
  
}
