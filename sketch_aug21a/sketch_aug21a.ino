//configuração do bluetooth
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

void setup() {
  // put your setup code here, to run once:
  //iniciar bluetooth definindo nome do dispositivo
   SerialBT.begin("ESP32_veleiro_autonomo");
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(500);
  SerialBT.println("Yes, im here");
}
