#include "gps.hpp"
#include "globals.h"
#include "Arduino.h"
#include "debug.hpp"
#include <TinyGPS++.h>

TinyGPSPlus gps;

void initGPS()
{
    SerialGPS.begin(115200, SERIAL_8N1, 35, 34);
}

void handleGPS() {
    while (Serial.available())
    {
        SerialGPS.write(Serial.read());
    }
    while (SerialGPS.available())
    {
        int byte = SerialGPS.read();
        if (gps.encode(byte))
        {
            //gpsRead = true;
            //SerialGPS.flush();
        }

        Serial.write(byte);
    }
    displayInfo();
}

void displayInfo()
{
    Debug.print(F("Location: "));
    if (gps.location.isValid())
    {
        Debug.print(gps.location.lat(), 6);
        Debug.print(F(","));
        Debug.print(gps.location.lng(), 6);
    }
    else
    {
        Debug.print(F("INVALID"));
    }

    Debug.print(F("  Date/Time: "));
    if (gps.date.isValid())
    {
        Debug.print(gps.date.month());
        Debug.print(F("/"));
        Debug.print(gps.date.day());
        Debug.print(F("/"));
        Debug.print(gps.date.year());
    }
    else
    {
        Debug.print(F("INVALID"));
    }

    Debug.print(F(" "));
    if (gps.time.isValid())
    {
        if (gps.time.hour() < 10)
            Debug.print(F("0"));
        Debug.print(gps.time.hour());
        Debug.print(F(":"));
        if (gps.time.minute() < 10)
            Debug.print(F("0"));
        Debug.print(gps.time.minute());
        Debug.print(F(":"));
        if (gps.time.second() < 10)
            Debug.print(F("0"));
        Debug.print(gps.time.second());
        Debug.print(F("."));
        if (gps.time.centisecond() < 10)
            Debug.print(F("0"));
        Debug.print(gps.time.centisecond());
    }
    else
    {
        Debug.print(F("INVALID"));
    }

    Debug.println();
}