
#include <Arduino.h>
#include "def_system.h"
#include "serial_print.h"

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