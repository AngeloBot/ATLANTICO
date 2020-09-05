#ifndef ACQUIRE_BUSS_H_INCLUDED
#define ACQUIRE_BUSS_H_INCLUDED

#include <HMC5883L.h>

extern float erro_rumo;
extern float rumo_real;
extern double rumo_ideal;
extern float desvio_waypoint;
extern int flag_buss;
extern HMC5883L compass;

void acquire_hall(void);
#endif // ACQUIRE_HALL_INCLUDE