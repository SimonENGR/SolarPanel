#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <SolarPosition.h>

// --- SHARED STATE VARIABLES (Declarations) ---
// "extern" tells the compiler: "These exist in main.cpp, don't create duplicates."

extern volatile bool isWifiConnected;      // True after BLE Prov success
extern volatile bool isSystemInitialized;  // True after GPS Sync
extern volatile bool isManualOverride;

extern volatile double currentLat;
extern volatile double currentLon;
extern volatile double currentAzimuth;
extern volatile double currentElevation;

// The Solar Calculator Object (Shared so WebServer can reset it, and SolarTask can use it)
extern SolarPosition* sunPosition;

#endif