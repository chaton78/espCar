#ifndef _SCREEN_H
#define _SCREEN_H
#include "Arduino.h"
#include "globals.h"
#include "config.h"
#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"`

extern SSD1306 display;
void initScreen();
void printCenterString(String text);
#endif