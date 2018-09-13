#ifndef _GLOBALS_H_
#define _GLOBALS_H_
#define HOST_NAME "esp32gps" // PLEASE CHANGE IT
extern const char *ssid;
extern const char *password;
extern const char apn[];
extern const char user[];
extern const char pass[];
extern const char server[];
extern const char resource[];
#define SerialAT Serial1
#define SerialGPS Serial2
#endif