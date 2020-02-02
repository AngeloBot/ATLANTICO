#include <SoftwareSerial.h>

//variáveis HALL=============================
int status_Hall[4]={0,0,0,0};

//Nível lógico máximo para representar ativação do hall
int maxHallSignal = 10;

//pin para input proveniente sensores
int pinLEFT = A1;
int pinRIGHT = A2; 
int pinFLEFT = A0;
int pinFRIGHT = A3;

//variáveis dos Interrupts

const uint8_t t_load = 0;
const uint8_t t0_cicles = 195; //para prescale=1024 com amostragem ~80Hz

void setup() {

  //Configuração dos Interrupts
  //os timers usados serão o 0, temos frequencia de amostragem de ~80Hz (T=0.1248s)

  //Reset Timer0 Control Reg A
  TCCR0A = 0; 
  //Set to prescaler of 1024 of timer 0
  TCCR0B |= (1<<CS02);
  TCCR0B &= ~(1<<CS01);
  TCCR0B |= (1<<CS00);

  //Reset Timer0 and set compare value
  TCNT0 = t_load;
  OCR0A = t0_cicles; 
  //Enable Timer0 compare interrupt
  TIMSK0 = (1<<OCIE0A);
  
  //Enable global interrupts
  sei();

  //Serial.println("Esperando por Dados do Modulo...");
  Serial.begin(9600);

}

void loop() {
  Serial.print("HALL ");
  Serial.print(status_Hall[0]);
  Serial.print(" ");
  Serial.print(status_Hall[1]);
  Serial.print(" ");
  Serial.print(status_Hall[2]);
  Serial.print(" ");
  Serial.println(status_Hall[3]);
}
ISR(TIMER0_COMPA_vect){

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
  
}
