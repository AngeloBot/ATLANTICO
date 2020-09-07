#include <HardwareSerial.h>
#include <Arduino.h>
#include <HMC5883L.h>
#include "def_system.h"
#include "buss_tools.h"
#include "supp_tools.h"
#include "BluetoothSerial.h"

void acquire_buss(){
    int done=0;

    ultimo_rumo=rumo_real;
    Vector norm = compass.readNormalize();
    // Calculate heading
    rumo_real = atan2(norm.YAxis, -norm.XAxis);
    // Set declination angle on your location and fix heading
    // You can find your declination on: http://magnetic-declination.com/
    // (+) Positive or (-) for negative
    // For Bytom / Poland declination angle is 4'26E (positive)
    // For Barueri / Brazil declination angle is 21'22W(negative)
    // For Santo Amaro / Brazil declination angle is 21'23W(negative)
    // For Taboao da Serra / Brazil declination angle is 21'25W(negative)
    // Formula: (deg + (min / 60.0)) / (180 / M_PI);

    //rumo_real_int=-rumo_real_int;
    rumo_real-=PI;
    rumo_real += desvio_waypoint*(PI/180);
    //SerialBT.print("rumo real antes: ");SerialBT.println(rumo_real);
    
    // Correct for heading < 0deg and heading > 360deg
    while (done!=1){
      if (rumo_real < 0){
          rumo_real += 2 * PI;
          //SerialBT.print("rumo real menor: ");SerialBT.println(rumo_real);
      }
      else if (rumo_real > 2 * PI){
          rumo_real -= 2 * PI;
          //SerialBT.print("rumo real maior: ");SerialBT.println(rumo_real);
      }
      else{
        done=1;
        }
    }
    // Converter para graus e guardar na variável apropriada
    rumo_real = rumo_real * 180/PI;
    //rumo_real_int = rumo_real_int * 180/PI;
    
    erro_rumo=calc_erro_rumo(rumo_ideal);
    
    flag_buss--;
    
    //Serial.print("rumo real: ");Serial.println(rumo_real);
    //Serial.print("erro rumo: ");Serial.println(erro_rumo);

}

void calib_buss(void){

    char command_reading=0;
    int calib_flag=0;
    int minX = 0;
    int maxX = 0;
    int minY = 0;
    int maxY = 0;
    int offX = 0;
    int offY = 0;
    
    digitalWrite(LED_Hall, HIGH);
    
    //desabilitar timers
    timerAlarmDisable(timer0); // enable
    timerAlarmDisable(timer1); // enable
    timerAlarmDisable(timer2); // enable

    //calibrar bússola
    while(calib_flag==0){

        if (SerialBT.available()){
            command_reading=(char) SerialBT.read();
            switch(command_reading){
                case 'f': //finalizar calibração
                    calib_flag=1;
                    break;

                case 'c': //cancelar calibração
                    calib_flag=2;
                    break;
            }       
            
        }

        Vector mag = compass.readRaw();
    
        // Determine Min / Max values
        if (mag.XAxis < minX) minX = mag.XAxis;
        if (mag.XAxis > maxX) maxX = mag.XAxis;
        if (mag.YAxis < minY) minY = mag.YAxis;
        if (mag.YAxis > maxY) maxY = mag.YAxis;
    
        // Calculate offsets
        offX = (maxX + minX)/2;
        offY = (maxY + minY)/2;

        //SerialBT.print("X\t");SerialBT.println(offX);
        //SerialBT.print("\tY\t");SerialBT.println(offY);
    }

    if(calib_flag==2){
        digitalWrite(LED_Hall, LOW);
        compass.setOffset(offX, offY);
    }

    //habilitar timers
    timerAlarmEnable(timer0); // enable
    timerAlarmEnable(timer1); // enable
    timerAlarmEnable(timer2); // enable
}
