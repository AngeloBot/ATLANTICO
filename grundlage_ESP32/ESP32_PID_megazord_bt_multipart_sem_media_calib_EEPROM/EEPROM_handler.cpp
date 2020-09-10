#include "def_system.h"
#include "EEPROM_handler.h"
#include <EEPROM.h>
#include <Arduino.h>

void multibyte_write(int num_byte, int num_value , int address){
    //byte mais significativo é gravado no endereço dado de argumento e os
    //bytes subsequentes (em ordem decrescente de significância)
    //nos próximos endereços em ordem crescente
    
    int num_value_piece=0;
    int cleaner=pow(2,8)-1;
    //Serial.println(cleaner,BIN);
    //Serial.println(num_value,BIN);
    //Serial.println("===================");
    
    for (int i=0;i<num_byte;i++){
        
        num_value_piece= (num_value >> 8*(num_byte-i-1));
        //Serial.println(num_value_piece,BIN);
        num_value_piece &= ((int) cleaner);
        
        //Serial.println(num_value_piece,BIN);
        //Serial.println("===================");
        
        EEPROM.write(address+i,num_value_piece);
    }
    
}

int multibyte_read(int num_byte, int address){
    //assume que o primeiro byte mais signigicativo esta armazenado no endereço
    //do argumento da função e os bytes subsequentes estão nos endereços seguintes
    //em ordem crescente

    int read_value=EEPROM.read(address);
    //Serial.println(read_value,BIN);
    int num_value=pow(2,(num_byte-1)*8);
    //Serial.println(num_value,BIN);
    num_value=num_value*read_value;

    for (int i=1;i<num_byte;i++){
            
        int read_value=EEPROM.read(address+i);
        //Serial.println(read_value,BIN);
        num_value=num_value + pow(2,(num_byte-1-i)*8)*read_value;
        //Serial.println(num_value,BIN);
    }
    //Serial.println(num_value);

    return num_value;
}