#ifndef ACQUIRE_HALL_H_INCLUDED
#define ACQUIRE_HALL_H_INCLUDED

extern int status_Hall;
extern int statusRIGHT;
extern int statusFRIGHT;
extern int statusLEFT;
extern int statusFLEFT;
extern volatile int flag_hall;

void acquire_hall(void);
#endif // ACQUIRE_HALL_INCLUDE
