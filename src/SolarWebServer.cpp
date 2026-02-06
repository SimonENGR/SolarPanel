#include "SolarWebServer.h"

// Constructor
SolarWebServer::SolarWebServer(const char* ssid, const char* password) 
    : server(80), ssid(ssid), password(password) {}

void SolarWebServer::begin() {
        
    Serial.print("Web Server Starting on IP: ");
    Serial.println(WiFi.localIP());

    // 2. Start mDNS
    if (MDNS.begin("esp32-solar")) {
        Serial.println("mDNS responder started");
    }

    // 3. Setup Routes & Start
    setupRoutes();
    server.begin();
}

void SolarWebServer::setupRoutes() {
    
    // --- STATUS ENDPOINT ---
    server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
        StaticJsonDocument<200> doc;
        
        if (!isSystemInitialized) {
            doc["status"] = "WAITING_FOR_SYNC";
        } else {
            doc["status"] = isManualOverride ? "MANUAL" : "AUTO";
        }
        
        doc["override"] = isManualOverride;
        doc["azimuth"] = currentAzimuth;
        doc["elevation"] = currentElevation;
        doc["lat"] = currentLat;
        doc["lon"] = currentLon;
        
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });

    // --- MODE CONTROL ---
    server.on("/mode", HTTP_GET, [](AsyncWebServerRequest *request){
        if (!isSystemInitialized) {
            request->send(403, "text/plain", "Error: System not synced yet!");
            return;
        }
        if (request->hasParam("manual")) {
            int val = request->getParam("manual")->value().toInt();
            isManualOverride = (val == 1);
            request->send(200, "text/plain", isManualOverride ? "Manual Mode" : "Auto Mode");
        } else {
            request->send(400, "text/plain", "Missing 'manual' parameter");
        }
    });

    // --- DATA UPDATE (GATEKEEPER UNLOCK) ---
    AsyncCallbackJsonWebHandler *handler = new AsyncCallbackJsonWebHandler("/update", 
        [](AsyncWebServerRequest *request, JsonVariant &json) {
            
            JsonObject jsonObj = json.as<JsonObject>();

            if (jsonObj.containsKey("lat") && jsonObj.containsKey("lon")) {
                double newLat = jsonObj["lat"];
                double newLon = jsonObj["lon"];
                // long phoneTime = jsonObj["timestamp"]; // Usage optional depending on NTP strategy

                // 1. Update Global Coords
                currentLat = newLat;
                currentLon = newLon;
                
                // 2. Re-initialize Solar Calculator (Deleting old pointer if exists)
                if (sunPosition) delete sunPosition;
                sunPosition = new SolarPosition(currentLat, currentLon);

                // 3. Unlock System
                isSystemInitialized = true; 

                // 4. Send Receipt
                StaticJsonDocument<200> responseDoc;
                responseDoc["message"] = "Sync Complete. Tracking Started.";
                responseDoc["received_lat"] = currentLat;
                responseDoc["received_lon"] = currentLon;
                
                String responseText;
                serializeJson(responseDoc, responseText);
                request->send(200, "application/json", responseText);

                Serial.println(">>> SYSTEM UNLOCKED: GPS Received.");

            } else {
                request->send(400, "application/json", "{\"message\":\"Invalid JSON\"}");
            }
    });
    server.addHandler(handler);
}