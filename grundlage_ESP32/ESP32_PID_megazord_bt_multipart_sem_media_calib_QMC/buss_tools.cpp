#include <HardwareSerial.h>
#include <Arduino.h>
#include <QMC5883LCompass.h>
#include "def_system.h"
#include "buss_tools.h"
#include "supp_tools.h"
#include "BluetoothSerial.h"

void acquire_buss(){
    int done=0;

    ultimo_rumo=rumo_real;
    compass.read();
    // Calculate heading
    rumo_real = atan2(compass.getY(), -compass.getX());
    // Set declination angle on your location and fix heading
    // You can find your declination on: http://magnetic-declination.com/
    // (+) Positive or (-) for negative
    // For Bytom / Poland declination angle is 4'26E (positive)
    // For Barueri / Brazil declination angle is 21'22W(negative)
    // For Santo Amaro / Brazil declination angle is 21'23W(negative)
    // For Taboao da Serra / Brazil declination angle is 21'25W(negative)
    // Formula: (deg + (min / 60.0)) / (180 / M_PI);

    //rumo_real_int=-rumo_real_int;
    rumo_real-=0;
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

    int calibrationData[3][2];
    //bool changed = false;
    //bool done = false;
    //int t = 0;
    //int c = 0;
    int x, y, z;
    
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
                case 'b': //finalizar calibração
                    calib_flag=1;
                    break;

                case 'c': //cancelar calibração
                    calib_flag=2;
                    break;
            }       
            
        }

        // Read compass values
        compass.read();

        // Return XYZ readings
        x = compass.getX();
        y = compass.getY();
        z = compass.getZ();

        //changed = false;

        if(x < calibrationData[0][0]) {
            calibrationData[0][0] = x;
            //changed = true;
        }
        if(x > calibrationData[0][1]) {
            calibrationData[0][1] = x;
            //changed = true;
        }

        if(y < calibrationData[1][0]) {
            calibrationData[1][0] = y;
            //changed = true;
        }
        if(y > calibrationData[1][1]) {
            calibrationData[1][1] = y;
            //changed = true;
        }

        if(z < calibrationData[2][0]) {
            calibrationData[2][0] = z;
            //changed = true;
        }
        if(z > calibrationData[2][1]) {
            calibrationData[2][1] = z;
            //changed = true;
        }
    }

    if(calib_flag==1){
        //SerialBT.println("====================");
        //SerialBT.print("X ");SerialBT.print(offX);
        //SerialBT.print(" Y ");SerialBT.println(offY);
        compass.setCalibration(calibrationData[0][0],calibrationData[0][1],calibrationData[1][0],calibrationData[1][1],calibrationData[2][0],calibrationData[2][1]);
        delay(3000);
        digitalWrite(LED_Hall, LOW); 
    }

    //habilitar timers
    timerAlarmEnable(timer0); // enable
    timerAlarmEnable(timer1); // enable
    timerAlarmEnable(timer2); // enable
    
}
