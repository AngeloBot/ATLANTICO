//Pins
const int led_pin = PB5; //pin 13 interno

//Counter and compare values
const uint16_t t1_load =0;
const uint16_t t1_comp = 31250;


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
int LpinLEFT = 7;
int LpinRIGHT = 4;
int LpinFLEFT = 8;
int LpinFRIGHT = 2;

void setup() {

    Serial.begin(9600);
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


}

void loop(){ 
    //RIGHT
  if ( status_Hall[2] == 1){
    
    digitalWrite(LpinRIGHT, HIGH);
    }
  else{

    digitalWrite(LpinRIGHT, LOW);
    }    
//LEFT  
  if ( status_Hall[1] == 1){
    
    digitalWrite(LpinLEFT, HIGH);
    }
  else{

    digitalWrite(LpinLEFT, LOW);
    }  
//FRIGHT
  if ( status_Hall[3] ==1){
    
    digitalWrite(LpinFRIGHT, HIGH);
    }
  else{

    digitalWrite(LpinFRIGHT, LOW);
    }
  //FLEFT
  if ( status_Hall[0] == 1){
    
    digitalWrite(LpinFLEFT, HIGH);
    }
  else{

    digitalWrite(LpinFLEFT, LOW);
    }
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
    Serial.println("in");
}
