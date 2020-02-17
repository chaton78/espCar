#include "globals.h"
#include "config.h"
const char apn[] = APN;
const char user[] = "";
const char pass[] = "";
const char server[] = "158.69.201.42";
const char resource[] = "/TinyGSM/logo.txt";
BluetoothSerial SerialBT;
WiFiUDP udp;
hw_timer_t *timer = NULL;