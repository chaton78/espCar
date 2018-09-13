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
#define TINY_GSM_MODEM_A6
#define TINY_GSM_RX_BUFFER 512
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <TinyGsmClient.h>

#include <Arduino.h>
#include <TimeLib.h>
#include <ArduinoOTA.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Include the correct display library
// For a connection via I2C using Wire include
#include <Wire.h>    // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"`
#include "debug.hpp"
#include "gps.hpp"
#include "globals.h"
// Include the UI lib
#include "OLEDDisplayUi.h"

// Include custom images
#include "images.h"



// Use the corresponding display class:

// Initialize the OLED display using SPI
// D5 -> CLK
// D7 -> MOSI (DOUT)
// D0 -> RES
// D2 -> DC
// D8 -> CS
// SSD1306Spi        display(D0, D2, D8);
// or
// SH1106Spi         display(D0, D2);

// Initialize the OLED display using brzo_i2c
// D3 -> SDA
// D5 -> SCL
// SSD1306Brzo display(0x3c, D3, D5);
// or
// SH1106Brzo  display(0x3c, D3, D5);

// Initialize the OLED display using Wire library
SSD1306 display(0x3c, 5, 4);
// SH1106 display(0x3c, D3, D5);

OLEDDisplayUi ui(&display);
Adafruit_BME280 bme; // I2C


uint32_t mLastTime = 0;
uint32_t mTimeSeconds = 0;

// Buildin Led ON ?

boolean mLedON = true;
int screenW = 128;
int screenH = 64;
int clockCenterX = screenW / 2;
int clockCenterY = ((screenH - 16) / 2) + 16; // top yellow part is 16 px height
int clockRadius = 23;

// utility function for digital clock display: prints leading 0
String twoDigits(int digits)
{
    if (digits < 10)
    {
        String i = '0' + String(digits);
        return i;
    }
    else
    {
        return String(digits);
    }
}

void clockOverlay(OLEDDisplay *display, OLEDDisplayUiState *state)
{
}

void analogClockFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
    //  ui.disableIndicator();

    // Draw the clock face
    //  display->drawCircle(clockCenterX + x, clockCenterY + y, clockRadius);
    display->drawCircle(clockCenterX + x, clockCenterY + y, 2);
    //
    //hour ticks
    for (int z = 0; z < 360; z = z + 30)
    {
        //Begin at 0° and stop at 360°
        float angle = z;
        angle = (angle / 57.29577951); //Convert degrees to radians
        int x2 = (clockCenterX + (sin(angle) * clockRadius));
        int y2 = (clockCenterY - (cos(angle) * clockRadius));
        int x3 = (clockCenterX + (sin(angle) * (clockRadius - (clockRadius / 8))));
        int y3 = (clockCenterY - (cos(angle) * (clockRadius - (clockRadius / 8))));
        display->drawLine(x2 + x, y2 + y, x3 + x, y3 + y);
    }

    // display second hand
    float angle = second() * 6;
    angle = (angle / 57.29577951); //Convert degrees to radians
    int x3 = (clockCenterX + (sin(angle) * (clockRadius - (clockRadius / 5))));
    int y3 = (clockCenterY - (cos(angle) * (clockRadius - (clockRadius / 5))));
    display->drawLine(clockCenterX + x, clockCenterY + y, x3 + x, y3 + y);
    //
    // display minute hand
    angle = minute() * 6;
    angle = (angle / 57.29577951); //Convert degrees to radians
    x3 = (clockCenterX + (sin(angle) * (clockRadius - (clockRadius / 4))));
    y3 = (clockCenterY - (cos(angle) * (clockRadius - (clockRadius / 4))));
    display->drawLine(clockCenterX + x, clockCenterY + y, x3 + x, y3 + y);
    //
    // display hour hand
    angle = hour() * 30 + int((minute() / 12) * 6);
    angle = (angle / 57.29577951); //Convert degrees to radians
    x3 = (clockCenterX + (sin(angle) * (clockRadius - (clockRadius / 2))));
    y3 = (clockCenterY - (cos(angle) * (clockRadius - (clockRadius / 2))));
    display->drawLine(clockCenterX + x, clockCenterY + y, x3 + x, y3 + y);
}

void digitalClockFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
    String timenow = String(hour()) + ":" + twoDigits(minute()) + ":" + twoDigits(second());
    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->setFont(ArialMT_Plain_24);
    display->drawString(clockCenterX + x, clockCenterY + y, timenow);
}

// This array keeps function pointers to all frames
// frames are the single views that slide in
FrameCallback frames[] = {analogClockFrame, digitalClockFrame};

// how many frames are there?
int frameCount = 2;

// Overlays are statically drawn on top of a frame eg. a clock
OverlayCallback overlays[] = {clockOverlay};
int overlaysCount = 1;
//HardwareSerial Serial1(1);
//HardwareSerial Serial2(2);


StreamDebugger debugger(SerialAT, Debug);
TinyGsm modem(debugger);
TinyGsmClient client(modem);


const int port = 80;

hw_timer_t *timer = NULL;
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

void setup()
{
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
    Serial.begin(115200, SERIAL_8N1);
    Serial.println();
    SerialAT.begin(115200, SERIAL_8N1, 16, 17, false);

    delay(0);

    WiFi.begin(ssid, password);

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(10);
    }
    
    // The ESP is capable of rendering 60fps in 80Mhz mode
    // but that won't give you much time for anything else
    // run it in 160Mhz mode or just set it to 30 fps
    ui.setTargetFPS(30);

    // Customize the active and inactive symbol
    ui.setActiveSymbol(activeSymbol);
    ui.setInactiveSymbol(inactiveSymbol);

    // You can change this to
    // TOP, LEFT, BOTTOM, RIGHT
    ui.setIndicatorPosition(TOP);

    // Defines where the first frame is located in the bar.
    ui.setIndicatorDirection(LEFT_RIGHT);

    // You can change the transition that is used
    // SLIDE_LEFT, SLIDE_RIGHT, SLIDE_UP, SLIDE_DOWN
    ui.setFrameAnimation(SLIDE_LEFT);

    // Add frames
    ui.setFrames(frames, frameCount);

    // Add overlays
    ui.setOverlays(overlays, overlaysCount);

    // Initialising the UI will init the display too.
    ui.init();

    display.flipScreenVertically();

    unsigned long secsSinceStart = millis();
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSinceStart - seventyYears * SECS_PER_HOUR;
    setTime(epoch);
    display.init();
    display.flipScreenVertically();
    display.setContrast(255);

    ArduinoOTA.begin();
    ArduinoOTA.onStart([]() {
        timerAlarmDisable(timer);
        display.clear();
        display.setFont(ArialMT_Plain_10);
        display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
        display.drawString(display.getWidth() / 2, display.getHeight() / 2 - 10, "OTA Update");
        display.display();
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        display.drawProgressBar(4, 32, 120, 8, progress / (total / 100));
        display.display();
    });

    ArduinoOTA.onEnd([]() {
        display.clear();
        display.setFont(ArialMT_Plain_10);
        display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
        display.drawString(display.getWidth() / 2, display.getHeight() / 2, "Restart");
        display.display();
    });

    // Align text vertical/horizontal center
    display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
    display.setFont(ArialMT_Plain_10);
    display.drawString(display.getWidth() / 2, display.getHeight() / 2, "Ready for OTA:\n" + WiFi.localIP().toString());
    display.display();
    bme.begin(0x76);

   /* Serial.print("Initializing modem...");
    if (!modem.restart())
    {
        Serial.println(F(" [fail]"));
        Serial.println(F("************************"));
        Serial.println(F(" Is your modem connected properly?"));
        Serial.println(F(" Is your serial speed (baud rate) correct?"));
        Serial.println(F(" Is your modem powered on?"));
        Serial.println(F(" Do you use a good, stable power source?"));
        Serial.println(F(" Try useing File -> Examples -> TinyGSM -> tools -> AT_Debug to find correct configuration"));
        Serial.println(F("************************"));
        delay(10000);
        return;
    }
    Serial.println(F(" [OK]"));

    String modemInfo = modem.getModemInfo();
    Serial.print("Modem: ");
    Serial.println(modemInfo);*/

    initDebug();
    initGPS();
    // Create semaphore to inform us when the timer has fired
    timerSemaphore = xSemaphoreCreateBinary();

    // Use 1st timer of 4 (counted from zero).
    // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
    // info).
    timer = timerBegin(0, 80, true);

    // Attach onTimer function to our timer.
    timerAttachInterrupt(timer, &onTimer, true);

    // Set alarm to call onTimer function every second (value in microseconds).
    // Repeat the alarm (third parameter)
    timerAlarmWrite(timer, 5000000, true);

    // Start an alarm
    timerAlarmEnable(timer);
}
#define SEALEVELPRESSURE_HPA (1013.25)
void printValues()
{
    display.drawString(10, 10, "Temperature = " + String(bme.readTemperature()));
    display.drawString(10, 20, "test2");
    display.display();
    DEBUG("Temperature = %f *C\n", bme.readTemperature());
    DEBUG("Pressure = %f  hPa\n", bme.readPressure() / 100.0F);
    DEBUG("Approx. Altitude = %f m\n", bme.readAltitude(SEALEVELPRESSURE_HPA));
    DEBUG("Humidity = %f %%\n", bme.readHumidity());
}
bool done = false;
void loop()
{
    ArduinoOTA.handle();
    handleDebug();



    // read from port 0, send to port 1:
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
        bme.takeForcedMeasurement(); // has no effect in normal mode
        printValues();
        handleGPS();
    }

    //bool gpsRead = false;
   // while (SerialGPS.available() > 0 && !gpsRead)
        
            

   
    /*
    if (!done)
    {
        Debug.print(F("Waiting for network..."));
        if (!modem.waitForNetwork())
        {
            Debug.println(" fail");
            delay(10000);
            return;
        }
        Debug.println(" OK");
        Debug.print(F("Connecting to "));
        Debug.print(apn);
        if (!modem.gprsConnect(apn, user, pass))
        {
            Debug.println(" fail");
            delay(10000);
            return;
        }
        Debug.println(" OK");

        Debug.print(F("Connecting to "));
        Debug.print(server);
        if (!client.connect(server, port))
        {
            Debug.println(" fail");
            delay(10000);
            return;
        }
        Debug.println(" OK");
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
                Debug.print(c);
                timeout = millis();
            }
        }
        Debug.println();

        // Shutdown

        client.stop();
        Debug.println(F("Server disconnected"));

        modem.gprsDisconnect();
        Debug.println(F("GPRS disconnected"));
        done = true;
    }*/
    delay(500);
}
