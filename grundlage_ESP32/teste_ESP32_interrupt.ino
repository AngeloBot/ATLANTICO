hw_timer_t*timer = NULL;

volatile int flag=B0;

void setup(){


}

void loop(){



}

void IRAM_ATTR onTimer(){

    if(flag==0){

        digitalWrite(2,HIGH);
    }

    else{

        digitalWrite(2,LOW);
    }

    flag=~flag;

}