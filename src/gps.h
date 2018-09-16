#ifndef _GPS_H_
#define _GPS_H_
#include "Arduino.h"
#include "globals.h"
#include "config.h"





#define MAX_SIZE_SEND 1460 // Maximum size of packet (limit of TCP/IP)







void initGPS();
void handleGPS();
void displayInfo();
#endif