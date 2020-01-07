#include <HMC5883L.h>

HMC5883L compass;

float rumo_real =0;

void setup(){

    Serial.begin(9600);
}

void loop(){
  
  //lê a bússola e guarda o valor

  Vector norm = compass.readNormalize();

  // Calculate heading
  rumo_real = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // For Barueri / Brazil declination angle is 21'22W(negative)
  // For Santo Amaro / Brazil declination angle is 21'23W(negative)
  // For Taboao da Serra / Brazil declination angle is 21'25W(negative)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (-21.0 - (34.0 / 60.0)) / (180 / M_PI);
  rumo_real += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (rumo_real < 0)
  {
    rumo_real += 2 * PI;
  }

  if (rumo_real > 2 * PI)
  {
    rumo_real -= 2 * PI;
  }

  // Converter para graus e guardar na variável apropriada
  rumo_real = rumo_real * 180/M_PI; 

  Serial.print(rumo_real);
  

}