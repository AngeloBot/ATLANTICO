#include <EEPROM.h>

//sketch para carregar valor dados na eeprom
//int -> 4 bytes
int offX= 171;
int offY= -189;


int X=0;
int Y=0; 

void divide_number(){
  
  
  }

void setup() {
  
  int offX_H = (offX >> 16);
  int offX_L = offX & (int) pow(2,17)-1;
  int offY_H = (offY >> 16);
  int offY_L = offY & (int) pow(2,17)-1;
  Serial.begin(115200);

  Serial.println(offX,BIN);
  Serial.println(offY,BIN);
  
  Serial.println(offX_H,BIN);
  Serial.println(offX_L,BIN);
  
  Serial.println(offY_H,BIN);
  Serial.println(offY_L,BIN);
  
  X=pow(2,17)*offX_H+offX_L;
  Y=pow(2,17)*offY_H+offY_L;
  Serial.println(X);
  Serial.println(Y);

  /*
  EEPROM.begin(EEPROM_SIZE);

  //escrever
  //EEPROM.write(address, value);
  EEPROM.write(0x01,1);
  //offX
  EEPROM.write(0x02,(offX >> 8));
  EEPROM.write(0x03,offX &= );
  //offY
  EEPROM.write(0x04,(offY >> 8));
  EEPROM.write(0x05,offY &= pow(2,9)-1);

  //salvar
  EEPROM.commit();
  */
}

void loop() {

  if (Serial.available()){
    
    }
}
