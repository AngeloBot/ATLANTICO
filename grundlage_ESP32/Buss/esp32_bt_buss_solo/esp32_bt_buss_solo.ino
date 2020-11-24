#include <Wire.h>
#include <HMC5883L.h>

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;


#include <ESP32_Servo.h>

Servo myservo;  // create servo object to control a servo
                // 16 servo objects can be created on the ESP32

int pos = 90;    // variable to store the servo position
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
int servoPin = 18;

HMC5883L compass;

#define BUSS_X_OFFSET 175
#define BUSS_Y_OFFSET -75

//desvio magnetico embu das artes
//float desvio_mag = -38-51/60;

//desvio magnetico jandira
float desvio_mag = -38-45/60; 

void setup() {
  
  Serial.begin(115200);

  myservo.attach(servoPin);

  myservo.write(pos);
  while (!compass.begin()){
      Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
      //digitalWrite(LED_BUILTIN,HIGH);
      //SerialBT.println("Could not find a valid HMC5883L sensor, check wiring!");
    }

  //digitalWrite(LED_BUILTIN,LOW);
  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(BUSS_X_OFFSET, BUSS_Y_OFFSET);
  
  
  
  SerialBT.begin("ESP32test"); //Bluetooth device name
  //Serial.println("The device started, now you can pair it with bluetooth!");

  
}

void loop() {

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
  rumo_real-=PI/2;
  rumo_real += desvio_mag*(PI/180);
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

  rumo_real = rumo_real * 180/PI;

  
  SerialBT.println(rumo_real);
  Serial.println(rumo_real);
  delay(125);
}
