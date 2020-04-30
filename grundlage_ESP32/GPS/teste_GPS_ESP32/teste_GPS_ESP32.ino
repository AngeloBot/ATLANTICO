#include <TinyGPS++.h>

TinyGPSPlus gps;

void setup(){
  
  Serial.begin(9600);
  }

void loop(){
  
  while (Serial.available() > 0){
    
    if(gps.encode(Serial.read())){
      
      displayInfo();
      }
    }
  }

void displayInfo(){
  
  Serial.print(F("Location: "));

  if (gps.location.isValid()){
    
    Serial.print(gps.location.lat(),8);
    Serial.print(F(","));
    Serial.print(gps.location.lng(),8);
    }

  else{

    Serial.print(F("INVALID"));
    }
  Serial.println();
  }
