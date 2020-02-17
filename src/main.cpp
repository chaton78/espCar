/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 by ThingPulse, Daniel Eichhorn
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * ThingPulse invests considerable time and money to develop these open source libraries.
 * Please support us by buying our products (and not the clones) from
 * https://thingpulse.com
 *
 */
#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 512

#include <TinyGsmClient.h>

#include <Arduino.h>

#include "BluetoothSerial.h"
#include "esp_bt.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
// Include the correct display library
// For a connection via I2C using Wire include

#include "debug.h"
#include "gps.h"
#include "screen.h"
#include "globals.h"
#include "wifi_button.h"
// Include the UI lib

//Adafruit_BME280 bme; // I2C

uint32_t mLastTime = 0;
uint32_t mTimeSeconds = 0;

// Buildin Led ON ?

boolean mLedON = true;

//HardwareSerial Serial1(1);
//HardwareSerial Serial2(2);

StreamDebugger debugger(SerialAT, Debug);
TinyGsm modem(debugger);
TinyGsmClient client(modem);

const int port = 80;

volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;

void IRAM_ATTR onTimer()
{
    // Increment the counter and set the time of ISR
    portENTER_CRITICAL_ISR(&timerMux);
    isrCounter++;
    lastIsrAt = millis();
    portEXIT_CRITICAL_ISR(&timerMux);
    // Give a semaphore that we can check in the loop
    xSemaphoreGiveFromISR(timerSemaphore, NULL);
    // It is safe to use digitalRead/Write here if you want to toggle an output
}

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    if (event == ESP_SPP_SRV_OPEN_EVT)
    {
        //Debug.println("Client Connected");
    }
    else if (event == ESP_SPP_DATA_IND_EVT)
    {
        esp_log_buffer_hex("", param->data_ind.data, param->data_ind.len);

        for (int i = 0; i < param->data_ind.len; i++)
        {
            //Debug.write(param->data_ind.data[i]);
            SerialAT.write(param->data_ind.data[i]);
        }
    }
}

bool done = false;
void modemTask( void * pvParameters ){
    while(true){
    String modemInfo = modem.getModemInfo();
    Serial.print("Modem: ");
    Serial.println(modemInfo);
    while (!done && modem.restart())
    {
        //done = true;
        Serial.print(F("Waiting for network..."));
        if (!modem.waitForNetwork())
        {
            Serial.println(" fail");
            delay(10000);
            return;
        }
        Serial.println(" OK");
        Serial.print(F("Connecting to "));
        Serial.print(apn);
        if (!modem.gprsConnect(apn, user, pass))
        {
            Serial.println(" fail");
            delay(10000);
            return;
        }
        Serial.println(" OK");
        Serial.println(modem.getLocalIP());

            Serial.print(F("Connecting to "));
        Serial.print(server);
        if (!client.connect(server, port))
        {
            Serial.println(" fail");
            delay(10000);
            return;
        }
        Serial.println(" OK");
        // Make a HTTP GET request:
        client.print(String("GET ") + resource + " HTTP/1.0\r\n");
        client.print(String("Host: ") + server + "\r\n");
        client.print("Connection: close\r\n\r\n");

        unsigned long timeout = millis();
        while (client.connected() && millis() - timeout < 10000L)
        {
            // Print available data
            while (client.available())
            {
                char c = client.read();
                Serial.print(c);
                timeout = millis();
            }
        }
        Serial.println();

        // Shutdown

        client.stop();
        Serial.println(F("Server disconnected"));

        modem.gprsDisconnect();
        Serial.println(F("GPRS disconnected"));
        done = true;
    }
    delay(1000);
    }

    /*if (SerialAT.available())
    {
        byte c = SerialAT.read();
        Serial.write(c);
        SerialBT.write(c);
    }
    if (SerialBT.available())
    {
        byte c= SerialBT.read();
        Serial.write(c);
        SerialAT.write(c);
    }*/
    delay(20);
}
void setup()
{
    //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
    Serial.begin(115200, SERIAL_8N1);
    
   
    SerialAT.begin(115200, SERIAL_8N1, 17, 16, false);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
    SerialBT.begin(BT_NAME);

    SerialBT.register_callback(callback);
    //SerialGPS.begin(115200, SERIAL_8N1, 33, 35);
    /* WiFiManager wifiManager;
    //
    //resetSettings();
    wifiManager.setConfigPortalTimeout(180);
    wifiManager.setMinimumSignalQuality(10);
    wifiManager.autoConnect("BMW328xi");
*/
    initScreen();

    //bme.begin(0x76);

    initWifi();
    // initGPS();
    Serial.print("Initializing modem...");
    if (!modem.restart())
    {
        Serial.println(F(" [fail]"));
        delay(10);
    }
    modem.enableGPS();
 
    // xTaskCreate(
     //               modemTask,   /* Function to implement the task */
     //               "modem", /* Name of the task */
     //               10000,      /* Stack size in words */
//              NULL,       /* Task input parameter */
      //              0,          /* Priority of the task */
        //            NULL); 
 


}





void loop()
{


    //handleGPS();
    handleWifiButton();
    //delay(20);
    //String gps_raw = modem.getGPSraw();
    //Serial.print(gps_raw);
    
    /*// read from port 0, send to port 1:
    if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE)
    {
        uint32_t isrCount = 0, isrTime = 0;
        // Read the interrupt count and time
        portENTER_CRITICAL(&timerMux);
        isrCount = isrCounter;
        isrTime = lastIsrAt;
        portEXIT_CRITICAL(&timerMux);
        // Print it
        Debug.print("onTimer no. ");
        Debug.print(isrCount);
        Debug.print(" at ");
        Debug.print(isrTime);
        Debug.println(" ms");
        //bme.takeForcedMeasurement(); // has no effect in normal mode
        //printValues();
        displayInfo();
        if (WiFi.status() == WL_CONNECTED)
        {
            udp.beginPacket(SERVER_IP, UDP_PORT);
            udp.print(geoHash());
            udp.endPacket();
        }
    }
 
    //bool gpsRead = false;
    // while (SerialGPS.available() > 0 && !gpsRead)
    */
    while (SerialAT.available())
    {
        SerialBT.write(SerialAT.read());
    }
    /*if (SerialBT.available())
    {
        SerialAT.write(SerialBT.read());
    }*/
    //delay(200);
}
