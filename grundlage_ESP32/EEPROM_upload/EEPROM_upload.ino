#include <EEPROM.h>

//sketch para carregar valor dados na eeprom
//int -> 4 bytes
int offX= 171;
int offY= -189;


int X=0;
int Y=0;

void multibyte_write(int num_byte, int value , int address){
    //byte mais significativo é gravado no endereço dado de argumento e os
    //bytes subsequentes (em ordem decrescente de significância)
    //nos próximos endereços em ordem crescente
    
    int value_piece=0;

    for (i=0;i<num_byte;i++){
        
        value_piece= ((value >> 8*(num_byte-i-1)) & ((int) (pow(2,num_byte*8+1)-1)<<8)) 
        EEPROM.write(address+i,value_piece);
        Serial.println(value_piece,BIN);
    }

}

int multibyte_read(int num_byte, int address){
    //assume que o primeiro byte mais signigicativo esta armazenado no endereço
    //do argumento da função e os bytes subsequentes estão nos endereços seguintes
    //em ordem crescente

    int value=0;

    for (i=0;i<num_byte;i++){
        
        value=value + pow(2,num_byte-i-1)*EEPROM.read(address+i);
    }
    Serial.println(value);

    return value;
}

void setup() {
  /*
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
  */

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
  Serial.begin(115200);

  multibyte_write(4,offX,0x01);
  multibyte_write(4,offY,0x05);

  Serial.print((int) multibyte_read(4,0x01));
  Serial.print((int) multibyte_read(4,0x05));
}

void loop() {

  delay(1000);
}
