#ifndef ACQUIRE_GPS_H_INCLUDED
#define ACQUIRE_GPS_H_INCLUDED

#include <Arduino.h>
#include <TinyGPS++.h> 
#include <HardwareSerial.h> 

extern double lat_long_desvio_waypoint[6];
extern int flag_gps;
extern HardwareSerial SerialGPS;
extern TinyGPSPlus gps;
extern int waypoint_count;
extern double lat_barco;
extern double long_barco;
extern double lat_waypoint;
extern double long_waypoint;
extern float desvio_waypoint;
extern double rumo_ideal;

void acquire_GPS(void);
#endif // ACQUIRE_HALL_INCLUDE