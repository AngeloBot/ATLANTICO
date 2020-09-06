#ifndef SUPP_TOOLS_H_INCLUDED
#define SUPP_TOOLS_H_INCLUDED

#include <ESP32_Servo.h>

extern Servo servo;
extern float rumo_real;
extern int rumo_margin_flag;
extern int pos;

double calc_erro_rumo(double);
int leme_ok(int);
void move_servo(int);
#endif // SUPP_TOOLS_H_INCLUDED
