
#include <Arduino.h>
#include "def_system.h"
#include "serial_report.h"

void serial_report(){
    
    Serial.println("=======================");
    Serial.print("STATE= "); Serial.println(current_state);
    Serial.print("LAST STATE= "); Serial.println(previous_state);
    Serial.print("POS= "); Serial.println(pos);
    Serial.print("SOMAE= "); Serial.println(SOMAE);
    Serial.print("CTE= "); Serial.println(cte);
    Serial.print("V_ang= "); Serial.println(v_yaw);
    Serial.print("rumo= "); Serial.println(rumo_real);
}

void serial_alt_report(){
    
    Serial.println("=======================");
    Serial.print("STATE= "); Serial.println(current_state);
    //Serial.print("LAST STATE= "); Serial.println(previous_state);
    Serial.print("POS= "); Serial.println(pos);
    serial_hall_print();
    //Serial.print("SOMAE= "); Serial.println(SOMAE);
    //Serial.print("CTE= "); Serial.println(cte);
    //Serial.print("V_ang= "); Serial.println(v_yaw);
    Serial.print("rumo= "); Serial.println(rumo_real);
}

void serial_hall_print(void){

    Serial.print("HALL=");
    switch(status_Hall){
      
      case 0:
      Serial.println("0000");
      break;
      
      case 1:
      Serial.println("0001");
      break;
      
      case 2:
      Serial.println("0010");
      break;

      case 3:
      Serial.println("0011");
      break;
   
      case 4:
      Serial.println("0100");
      break;

      case 6:
      Serial.println("0110");
      break;

      case 8:
      Serial.println("1000");
      break;

      case 12:
      Serial.println("1100");
      break;

    }
    //Serial.print("HALL=");Serial.println(status_Hall);
}