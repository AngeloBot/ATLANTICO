#include <EEPROM.h>

//sketch para carregar valor dados na eeprom

int offX= 171;
int offY= -189;


int X=0;
int Y=0; 

void setup() {
  
  int offX_H = (offX >> 8);
  int offX_L = offX & pow(2,9)-1;
  int offY_H = (offY >> 8);
  int offY_L = offY & pow(2,9)-1;
  Serial.begin(115200);

  X=pow(2,8)*offX_H+offX_L;
  Y=pow(2,8)*offY_H+offY_L;
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
  
  delay(1000);
}
