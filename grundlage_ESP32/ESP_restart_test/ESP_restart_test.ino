#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

void setup() {

    SerialBT.begin("ESP32test"); //Bluetooth device name
    SerialBT.println("I WOKE UP JUST NOW");
}

void loop() {

    char command='b';

    if (SerialBT.available()) {
        command = SerialBT.read();
        switch(command){
            case 'r':

                ESP.restart();
                break;
            default:
                SerialBT.println("I AM AWAKE");
                break;
        }
    }
    delay(20);
}