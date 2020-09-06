

#include <Arduino.h>
#include "def_system.h"
#include "bt_print.h"
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
    //SerialBT.print("SOMAE= "); SerialBT.println(SOMAE);
    //SerialBT.print("CTE= "); SerialBT.println(cte);
    //SerialBT.print("V_ang= "); SerialBT.println(v_yaw);
    SerialBT.print("rumo= "); SerialBT.println(rumo_real);
}