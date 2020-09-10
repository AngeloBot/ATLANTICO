

#include <Arduino.h>
#include "def_system.h"
#include "bt_report.h"
#include "BluetoothSerial.h"

void bt_report(void){
    
    SerialBT.println("=======================");
    SerialBT.print("STATE= "); SerialBT.println(current_state);
    SerialBT.print("LAST STATE= "); SerialBT.println(previous_state);
    SerialBT.print("POS= "); SerialBT.println(pos);
    SerialBT.print("SOMAE= "); SerialBT.println(SOMAE);
    SerialBT.print("CTE= "); SerialBT.println(cte);
    SerialBT.print("V_ang= "); SerialBT.println(v_yaw);
    SerialBT.print("rumo= "); SerialBT.println(rumo_real);
}

void bt_alt_report(void){
    
    SerialBT.println("=======================");
    SerialBT.print("STATE= "); SerialBT.println(current_state);
    //SerialBT.print("LAST STATE= "); SerialBT.println(previous_state);
    SerialBT.print("POS= "); SerialBT.println(pos);
    bt_hall_print();
    //SerialBT.print("SOMAE= "); SerialBT.println(SOMAE);
    //SerialBT.print("CTE= "); SerialBT.println(cte);
    //SerialBT.print("V_ang= "); SerialBT.println(v_yaw);
    SerialBT.print("rumo= "); SerialBT.println(rumo_real);
}

void bt_hall_print(void){

    SerialBT.print("HALL=");
    switch(status_Hall){
      
      case 0:
      SerialBT.println("0000");
      break;
      
      case 1:
      SerialBT.println("0001");
      break;

      case 2:
      SerialBT.println("0010");
      break;

      case 3:
      SerialBT.println("0011");
      break;
      
      case 4:
      SerialBT.println("0100");
      break;

      case 6:
      SerialBT.println("0110");
      break;

      case 8:
      SerialBT.println("1000");
      break;

      case 12:
      SerialBT.println("1100");
      break;
    }
    //Serial.print("HALL=");Serial.println(status_Hall);

}
