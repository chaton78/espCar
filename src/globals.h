#ifndef _GLOBALS_H_
#define _GLOBALS_H_
#include "BluetoothSerial.h"
#include <WiFi.h>
#define HOST_NAME "esp32gps" // PLEASE CHANGE IT
extern const char apn[];
extern const char user[];
extern const char pass[];
extern const char server[];
extern const char resource[];
extern BluetoothSerial SerialBT;
extern WiFiUDP udp;
extern hw_timer_t *timer;
#define SerialAT Serial1
#define SerialGPS Serial2

#endif