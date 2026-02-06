#ifndef SOLAR_WEB_SERVER_H
#define SOLAR_WEB_SERVER_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include "Globals.h" // Access to isSystemInitialized, currentLat, etc.

class SolarWebServer {
    private:
        AsyncWebServer server;
        const char* ssid;
        const char* password;
        
        void setupRoutes();

    public:
        SolarWebServer(const char* ssid, const char* password);
        void begin();
};

#endif