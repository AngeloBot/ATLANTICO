#include <HardwareSerial.h>
#include <Arduino.h>
#include <HMC5883L.h>
#include 'def_system.h'
#include 'acquire_buss.h'
#include 'supp_tools.h'

void acquire_buss(){
    int done=0;

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