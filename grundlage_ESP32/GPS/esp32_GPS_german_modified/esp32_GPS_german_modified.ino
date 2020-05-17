/*
   (c) 2019 by shopofthings.ch
   Beispiel zum Verbinden des GPS Modules mit
   einem ESP32 Modul
   Pinverbindung:
   ESP 3.3V > GPS VCC
   ESP GND  > GPS GND
   ESP RX   > GPS TX
   ESP TX   > GPS RX
   Achtung: RX wird mit TX bzw. TX mit RX verbunden!
   Das GPS Modul hat eine blaue LED. Diese blinkt, wenn Satelliten erreichbar sind. Sind
   keine Satelliten im Sichtfeld, bleibt sie dunkel. Ein Zustand, wo sie immer leuchtet 
   gibt es nicht.
  Troubleshooting:
  - Arduino IDE gibt bei Upload Fehler aus: Boot Taste vor dem Upload gedrückt halten, bis Upload beginnt.
  - 
*/

#include <TinyGPS++.h> // Library über http://arduiniana.org/libraries/tinygpsplus/ downloaden und installieren
#include <HardwareSerial.h> // sollte bereits mit Arduino IDE installiert sein

/*
   Als nächstes wird das Programmobjekt für das GPS erstellt, wir nennen
   die Variable die darauf zeigt einfach "gps"
*/
TinyGPSPlus gps;

/*
   Nun müssen wir eine serielle Verbindung zum GPS Modul erstellen
   ESP32 unterstützt bis zu 3 Hardware Serielle Verbndungen. Deshalb
   müssen wir auch nicht die Softwareserial-Library verwenden.
   Die Zahl in der Klammer ist die uart_nr. Damit werden die drei möglichen
   Verbindungen unterschieden. Für ESP32 kann dieser Wert also 0, 1 oder 2 sein
*/
HardwareSerial SerialGPS(1);



void setup() {

  // Serial ist die Ausgabe im Serial Monitor
  Serial.begin(115200);

  /*
      Die Verbindung mit dem GPS Modul. Wir
      void begin(unsigned long baud, uint32_t config=SERIAL_8N1, int8_t rxPin=-1, int8_t txPin=-1, bool invert=false, unsigned long timeout_ms = 20000UL);
      baud: baudrate gemäss Spzifikation des GPS Moduls, in diesem Fall 9600
      config: default Wert
      rxPin: ein RX-Pin z.B. 16
      txPin: ein RX-Pin z.B. 17
  */
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);

}
void loop() {

  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;  
  /*
   * Rohdaten von Serieller Verbndung zum GPS-Modul
   * einlesen. Die Daten werden mittels TinyGPS++ verarbeitet
   * Die Daten werden bewusst erst nach der Zuweisung der Variablen
   * gelesen, damit wir noch im nachfolgenden vereinfacht 
   * Berechnungen anstellen können.
   */
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }

  /*
   * Diverse Berechnungen von Maximum und Minimum-Werten und zurückgelegter Distanz
   * Diese werden aber erst gemacht, wenn mindestens ein Fix mit 4 Satelliten vorhanden
   * ist, allenfalls wäre die Genauigkeit nicht gegeben und es würden falsche
   * Werte berechnet werden.
   */
  if (gps.satellites.value() > 4) {
    
    Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
    Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
    Serial.print("Sats=");  Serial.println(gps.satellites.value());
      unsigned long distanceKmToLondon =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON) / 1000;
  Serial.println(distanceKmToLondon);

  double courseToLondon =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON);

  Serial.println(courseToLondon);

  }


}
