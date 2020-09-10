#ifndef BT_PRINT_H_INCLUDED
#define BT_PRINT_H_INCLUDED

#include "BluetoothSerial.h"

extern BluetoothSerial SerialBT;
extern int current_state;
extern int previous_state;
extern int pos;
extern double SOMAE;
extern int cte;
extern float v_yaw;
extern float rumo_real;
extern int status_Hall;

void bt_report(void);
void bt_alt_report(void);
void bt_hall_print(void);

#endif //BT_PRINT_H_INCLUDED