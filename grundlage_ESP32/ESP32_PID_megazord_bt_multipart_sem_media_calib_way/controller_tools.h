#ifndef CONTROLLER_TOOLS_H_INCLUDED
#define CONTROLLER_TOOLS_H_INCLUDED
 
extern int timer_PID;
extern double SOMAE;
extern float v_yaw;
extern float ultimo_rumo;
extern float rumo_real;

int calc_PID(float);
int calc_PD(float);
#endif // CONTROLLER_TOOLS_H_INCLUDED
