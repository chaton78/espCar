#include "debug.h"
#include "globals.h"
RemoteDebug Debug;

void initDebug() {

    Debug.begin(HOST_NAME, 23, RemoteDebug::DEBUG); // Initiaze the telnet server

    Debug.setResetCmdEnabled(true); // Enable the reset command

    //Debug.showDebugLevel(false); // To not show debug levels
    Debug.showTime(false);     // To show time
    Debug.showProfiler(false); // To show profiler - time between messages of Debug
    // Good to "begin ...." and "end ...." messages

    Debug.showColors(true); // Colors
}

void handleDebug() { Debug.handle(); }