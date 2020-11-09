//This example code is in the Public Domain (or CC0 licensed, at your option.)
//By Evandro Copercini - 2018
//
//This example creates a bridge between Serial and Classical Bluetooth (SPP)
//and also demonstrate that SerialBT have the same functionalities of a normal Serial

#include <Wire.h>
#include <HMC5883L.h>
#include "BluetoothSerial.h"


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

HMC5883L compass;
BluetoothSerial SerialBT;

void acquire_buss(){
    int done=0;
    float rumo_real;

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
    
    SerialBT.print(millis());
    SerialBT.print("    ");
    SerialBT.println(rumo_real)

}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
  acquire_buss();
}
