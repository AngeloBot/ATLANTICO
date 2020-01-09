#include <avr/io.h> 
#include <avr/interrupt.h> 
 
// Set up input capture mode
// Set up timers
// Set up port for output
 
unsigned int Period;
 
void setup(void) 
{ 
  Serial.begin(9600); 
  DDRB |= 0x01; 
  
  Init(); 
  
  Serial.println("Init finished"); 
} 
 void loop(void)
 {
   while(1)
   {
     Serial.print(" ");
   }
 } 
 
void Init(void) 
{ 
    //init Port B0 
    DDRB &= ~0x01;   // make ICP1 pin i/p 
    //init timer registers
    TCCR1A = 0x00;   // Normal Mode
    TCCR1B = 0x05; // prescalar 1024
    TIMSK1 = 0x21;  //-Interrupt enable 
    // Timer overflow enable 
    TCNT1 = 0; 
    //Clear flags in Timer flag reg 1
    TIFR1  = 0x21;              //0x21; 
} 
 
// ISR for falling edge
ISR(TIMER1_CAPT_vect) 
{ 
  if((PINB & (1<<PORTB0)) == 0)       // capture falling edge 
  { 
    Period = ICR1; 
    TCNT1 = 0;               //Reset timer 1
    TCCR1A = 0x00;
    PORTB = PORTB^0x20;       // toggle PB5 pin 
  } 
}