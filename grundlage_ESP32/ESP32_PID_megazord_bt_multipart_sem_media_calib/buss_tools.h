#ifndef BUSS_TOOLS_H_INCLUDED
#define BUSS_TOOLS_H_INCLUDED

#include <HMC5883L.h>

extern float erro_rumo;
extern float rumo_real;
extern double rumo_ideal;
extern float desvio_waypoint;
extern volatile int flag_buss;
extern float ultimo_rumo;
extern HMC5883L compass;

void acquire_buss(void);
#endif // BUSS_TOOLS_INCLUDE
