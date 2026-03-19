#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <SolarPosition.h>

// --- TRACKING MODE CONSTANTS ---
#define MODE_AUTOMATIC   0
#define MODE_SEMI_AUTO   1
#define MODE_MANUAL      2

// --- SHARED STATE VARIABLES (Declarations) ---
// "extern" tells the compiler: "These exist in main.cpp, don't create
// duplicates."

extern volatile bool isWifiConnected;     // True after BLE Prov success
extern volatile bool isSystemInitialized; // True after GPS Sync
extern volatile int  trackingMode;        // 0=AUTO, 1=SEMI_AUTO, 2=MANUAL
extern volatile bool forceTrackingUpdate;

// --- WEATHER OVERRIDE STATE ---
extern volatile int  weatherOverrideAngle;        // -1 = no override, else target angle
extern volatile bool weatherPendingConfirmation;   // Semi-auto: waiting for user
extern String currentWeatherCondition;             // "Clear", "Clouds", "Rain", "Snow"

extern volatile double currentLat;
extern volatile double currentLon;
extern volatile double currentAzimuth;
extern volatile double currentElevation;

// The Solar Calculator Object (Shared so WebServer can reset it, and SolarTask
// can use it)
extern SolarPosition *sunPosition;

#endif