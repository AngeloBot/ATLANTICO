
#include <HardwareSerial.h>
#include <Arduino.h>
#include <TinyGPS++.h>
#include "def_system.h"
#include "acquire_GPS.h"

#include "BluetoothSerial.h"

void acquire_GPS(void){

    while (SerialGPS.available() > 0) {
        gps.encode(SerialGPS.read());
    }
    
    SerialBT.print("GPS= ");SerialBT.println(gps.satellites.value());

    if (gps.satellites.value() >= 4) {
    
        lat_barco=gps.location.lat();
        long_barco=gps.location.lng();
        //Serial.print("LAT=");  Serial.println(lat_barco, 6);
        //Serial.print("LONG="); Serial.println(long_barco, 6);
        

        dist_waypoint=(unsigned long)TinyGPSPlus::distanceBetween(lat_barco,long_barco,lat_waypoint,long_waypoint); //m
        
        //Serial.print("Dist ");Serial.println(dist_waypoint);

        rumo_ideal =(double)TinyGPSPlus::courseTo(lat_barco,long_barco,lat_waypoint,long_waypoint);
        //Serial.print("Course ");Serial.println(rumo_ideal);

        if (waypoint_radius > dist_waypoint){
            waypoint_count += 1;
            if(waypoint_count==waypoint_num-1){
                waypoint_count=0;
            }
            lat_waypoint=lat_long_desvio_waypoint[waypoint_count*3];
            long_waypoint=lat_long_desvio_waypoint[waypoint_count*3+1];
            desvio_waypoint=lat_long_desvio_waypoint[waypoint_count*3+2];
            acquire_GPS();
         }
        
    }

    flag_gps--;

}

void GPS_info_bt(void){
  
    SerialBT.print("LAT=");  SerialBT.println(lat_barco, 6);
    SerialBT.print("LONG="); SerialBT.println(long_barco, 6);
    SerialBT.print("Course= "); SerialBT.println(rumo_ideal);
    SerialBT.print("Dist= "); SerialBT.println(dist_waypoint);
}
