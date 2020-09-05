#include <ESP32_Servo.h>
#include 'def_system.h'
#include 'supp_tools.h'

double calc_erro_rumo(double rumo){
    
    float erro=rumo-rumo_real;

    //ajeitar sinal para menor mudança de rumo
    if (abs(erro) <= 180 ) {
    erro = erro;
    }

    else if(((erro) > 0 && abs(erro) > 180)){
    erro -=360;
    }
    
    else if(((erro) < 0 && abs(erro) > 180)){
    erro += 360;
    }
    
    if(abs(erro) < lambda_rumo){
        rumo_margin_flag=1;
    }
    else{
        rumo_margin_flag=0;
    }
    return erro;
}

int leme_ok(int pos_leme){

    if(pos_leme<leme_min){
        return leme_min;
    }
    else if(pos_leme>leme_max){
        return leme_max;
    }
    else{
        return pos_leme;
    }
}

void move_servo(int nova_pos){

    if(nova_pos-pos > delta_servo){
        pos=+delta_servo;
    }
    else if(nova_pos-pos < -delta_servo){
        pos-=delta_servo;
    }
    else{
        pos=nova_pos;
    }
    servo.write(pos);
}