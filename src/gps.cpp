#include "gps.h"
#include "globals.h"
#include "Arduino.h"
#include "debug.h"
#include "geohash.h"
#include <TinyGPS++.h>
#include <WiFi.h>

TinyGPSPlus gps;
GeoHash hasher(9);
WiFiServer gpsServer(GPS_PORT);
WiFiClient gpsClient;
String _bufferSend = "";      // Buffer to send data to telnet
uint16_t _sizeBufferSend = 0; // Size of it
uint32_t _lastTimeSend = 0;   // Last time command send data
void initGPS()
{
    SerialGPS.begin(115200, SERIAL_8N1, 33, 35);
    gpsServer.begin(GPS_PORT);
    gpsServer.setNoDelay(true);
    _bufferSend.reserve(MAX_SIZE_SEND);
}

void handleGPS() {
    if (gpsServer.hasClient())
    {


        if (gpsClient && gpsClient.connected())
        {

            // Verify if the IP is same than actual conection

            WiFiClient newClient;
            newClient = gpsServer.available();
            String ip = newClient.remoteIP().toString();

            if (ip == gpsClient.remoteIP().toString())
            {
                // Reconnect
                gpsClient.stop();
                gpsClient = newClient;
            }
            else
            {
                // Disconnect (not allow more than one connection)
                newClient.stop();
            }
        }
        else
        {
            // New TCP client
            gpsClient = gpsServer.available();
        }

        if (gpsClient)
        {
            gpsClient.setNoDelay(true); // More faster
            gpsClient.flush();          // clear input buffer, else you get strange characters
        }

        // Client buffering - send data in intervals to avoid delays or if its is too big
        _bufferSend = "";
        _sizeBufferSend = 0;
        _lastTimeSend = millis();


        // Empty buffer in

        delay(100);

        while (gpsClient.available())
        {
            gpsClient.read();
        }
    }

    // Is client connected ? (to reduce overhead in active)
    bool _connected = (gpsClient && gpsClient.connected());

     while (SerialGPS.available() && _sizeBufferSend< MAX_SIZE_SEND)
    {
        int byte = SerialGPS.read();
        gps.encode(byte);
        _bufferSend.concat((char)byte);
        _sizeBufferSend++;
    }

    // Client buffering - send data in intervals to avoid delays or if its is too big

    if (_connected)
    {
        gpsClient.print(_bufferSend);
    }
    else
    {
        Serial.print(_bufferSend);
    }
    
    _bufferSend = "";
    _sizeBufferSend = 0;
    _lastTimeSend = millis();
    

    while (!_connected && Serial.available())
    {
        SerialGPS.write(Serial.read());
    }
    
    while (_connected && gpsClient.available())
    {
        SerialGPS.write(gpsClient.read());
    }
}

void displayInfo()
{
    Debug.print(F("Location: "));
    if (gps.location.isValid())
    {
        Debug.print(gps.location.lat(), 6);
        Debug.print(F(","));
        Debug.print(gps.location.lng(), 6);
        Debug.printf(" GeoHash: %s", hasher.encode(gps.location.lat(), gps.location.lng()));
        Debug.print(F(","));
        Debug.print(gps.speed.kmph(),1);
        Debug.print(F(","));
        Debug.print(gps.hdop.hdop(), 1);
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