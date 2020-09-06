#ifndef SERIAL_PRINT_H_INCLUDED
#define SERIAL_PRINT_H_INCLUDED

#include <Arduino.h>

extern int current_state;
extern int previous_state;
extern int pos;
extern double SOMAE;
extern int cte;
extern float v_yaw;
extern float rumo_real;

void serial_report(void);

#endif //SERIAL_PRINT_H_INCLUDED