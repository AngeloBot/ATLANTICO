#include <Arduino.h>
#include 'def_system.h'
#include 'controller_tools.h'

int calc_PID(float E){

    int now=millis();
    int heap=now-timer_PID;
    timer_PID=now;
    
    SOMAE+=E*heap/1000;

    //velocidade angular em grau/s
    v_yaw=(rumo_real-ultimo_rumo)*1000/heap;

    return map(leme_ok(Kp*E+Ki*SOMAE-Kd*v_yaw),leme_min,leme_max,servo_min,servo_max);
}

int calc_PD(float E){

    int now=millis();
    int heap=now-timer_PID;
    timer_PID=now;
    
    //velocidade angular em grau/s
    v_yaw=(rumo_real-ultimo_rumo)*1000/heap;

    return map(leme_ok(Kp*E-Kd*v_yaw),leme_min,leme_max,servo_min,servo_max);
}