// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Sketch shows how to switch between WiFi and BlueTooth or use both
// Button is attached between GPIO 0 and GND and modes are switched with each press

#include "wifi_button.h"
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "screen.h"
#include "config.h"
#include "debug.h"
#include "gps.h"
enum
{
    STEP_BTON,
    STEP_BTOFF,
    STEP_STA,
    STEP_AP,
    STEP_AP_STA,
    STEP_OFF,
    STEP_BT_STA,
    STEP_END
};
void connect()
{
    int ledState = 0;
    Serial.println("Connecting Wifi");
    WiFi.begin(STA_SSID, STA_PASS);
    while (WiFi.status() != WL_CONNECTED)
    {
         digitalWrite(LEDPIN, ledState);
    ledState = (ledState + 1) % 2; // Flip ledState
    delay(500);
    Serial.print(".");
    }
    //udp.begin(UDP_PORT);
    Serial.println();
    Serial.println("WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}
static uint32_t step = STEP_BTON;
void onButton()
{

    switch (step)
    {
    case STEP_BTON: //BT Only
        printCenterString("** Starting BT");
        if (!SerialBT.begin(BT_NAME))
        {
            Serial.println("An error occurred initializing Bluetooth");
            printCenterString("** Error BT");
        }
        break;
    case STEP_BTOFF: //All Off
        printCenterString("** Stopping BT");
        SerialBT.end();
        break;
    case STEP_STA: //STA Only
        printCenterString("** Starting STA");
        connect();
        break;
    case STEP_AP: //AP Only
        printCenterString("** Stopping STA");
        WiFi.mode(WIFI_AP);
        printCenterString("** Starting AP");
        WiFi.softAP(AP_SSID);
        break;
    case STEP_AP_STA: //AP+STA
        printCenterString("** Starting AP_STA");
        connect();
        break;
    case STEP_OFF: //All Off
        printCenterString("** Stopping WiFi");
        WiFi.mode(WIFI_OFF);
        break;
    case STEP_BT_STA: //BT+STA
        printCenterString("*Start STA+BT");
        connect();
        if (!SerialBT.begin(BT_NAME))
        {
            Serial.println("An error occurred initializing Bluetooth");
        }
        break;
    case STEP_END: //All Off
        printCenterString("** Stopping WiFi+BT");
        WiFi.mode(WIFI_OFF);
        SerialBT.end();
        break;
    default:
        break;
    }
    if (step == STEP_END)
    {
        step = STEP_BTON;
    }
    else
    {
        step++;
    }
    //little debounce
    delay(100);
}

void WiFiEvent(WiFiEvent_t event)
{
    switch (event)
    {
    case SYSTEM_EVENT_AP_START:
        printCenterString("AP Started");
        WiFi.softAPsetHostname(AP_SSID);
        break;
    case SYSTEM_EVENT_AP_STOP:
        printCenterString("AP Stopped");
        break;
    case SYSTEM_EVENT_STA_START:
        printCenterString("STA Started");
        WiFi.setHostname(AP_SSID);
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        printCenterString("STA Connected");
        WiFi.enableIpV6();
        break;
    case SYSTEM_EVENT_AP_STA_GOT_IP6:
        printCenterString("STA IPv6: " + WiFi.localIPv6().toString());
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        printCenterString("STA IPv4: " + WiFi.localIP().toString());
        Serial.print("Start OTA");
       
        ArduinoOTA.onStart([]() {
            printCenterString(" OTA Update ");
        });

        ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
            display.drawProgressBar(4, 32, 120, 8, progress / (total / 100));
            display.display();
        });

        ArduinoOTA.onEnd([]() {
            printCenterString("Restart");
        });
        initDebug();
        initGPSServer();
        ArduinoOTA.begin();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        printCenterString("STA Disconnected");
        break;
    case SYSTEM_EVENT_STA_STOP:
        printCenterString("STA Stopped");
        ArduinoOTA.end();
        break;
    default:
        break;
    }
}

void initWifi()
{
    pinMode(0, INPUT_PULLUP);
    WiFi.onEvent(WiFiEvent);
    onButton();
}

void handleWifiButton()
{
    static uint8_t lastPinState = 1;
    uint8_t pinState = digitalRead(0);
    if (!pinState && lastPinState)
    {
        onButton();
    }
    lastPinState = pinState;
    if (WiFi.status() == WL_CONNECTED)
    {
        //Serial.print("Handling OTA");
        ArduinoOTA.handle();
        handleDebug();
    }
}