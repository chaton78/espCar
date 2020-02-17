#include "screen.h"
#include "OLEDDisplayUi.h"
#include <Wire.h> // Only needed for Arduino 1.6.5 and earlier
#include <TimeLib.h>
// Include custom images
#include "images.h"

// Initialize the OLED display using Wire library
SSD1306 display(0x3c, 5, 4);
// SH1106 display(0x3c, D3, D5);
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

OLEDDisplayUi ui(&display);

int screenW = 128;
int screenH = 64;
// utility function for digital clock display: prints leading 0

void initScreen()
{
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

    // Initialising the UI will init the display too.
    ui.init();
    display.flipScreenVertically();
    display.init();
    display.flipScreenVertically();
    display.setContrast(255);
}
void printCenterString(String text)
{
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
    display.drawString(display.getWidth() / 2, display.getHeight() / 2, text);
    display.display();
}
void handleScreen();