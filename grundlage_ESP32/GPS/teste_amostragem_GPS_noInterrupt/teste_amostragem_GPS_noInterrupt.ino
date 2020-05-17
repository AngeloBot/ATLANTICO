#include <TinyGPS++.h>

TinyGPSPlus gps;

static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

void setup(){
  
  Serial.begin(115200);
  }

void loop(){
  
  while (Serial.available() > 0){
    
    if(gps.encode(Serial.read())){
      
      displayInfo();
      }
    }
  }

void displayInfo(){
  
  double lat;
  double lng;
  double courseToWaypoint;
  unsigned long distanceToWaypoint;
  
  Serial.print(F("Location: "));

  if (gps.location.isValid()){
    
    lat=gps.location.lat();
    lng=gps.location.lng();

    
    Serial.print(lat,8);
    Serial.print(F(","));
    Serial.print(lng,8);

    courseToWaypoint =
    TinyGPSPlus::courseTo(
      lat,
      lng,
      LONDON_LAT, 
      LONDON_LON);

    Serial.print("Rumo ideal: ");
    Serial.print(courseToWaypoint);
    Serial.print(" ");

    distanceToWaypoint =
    (unsigned long)TinyGPSPlus::distanceBetween(
      lat,
      lng,
      LONDON_LAT, 
      LONDON_LON); //em metros

    Serial.print("Distância: ");
    Serial.print(distanceToWaypoint);
    

    }

  else{

    Serial.print(F("INVALID"));
    }
  Serial.println();
  }
