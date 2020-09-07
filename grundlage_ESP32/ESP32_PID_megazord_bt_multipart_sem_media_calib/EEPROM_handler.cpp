#include "def_system.h"
#include "EEPROM_handler.h"
#include <EEPROM.h>
#include <Arduino.h>

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