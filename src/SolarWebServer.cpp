#include "SolarWebServer.h"
#include "MotorDriver.h" 

extern MotorDriver motorSystem;

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
    
    // --- STATUS ---
    server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
        StaticJsonDocument<200> doc;
        doc["status"] = isManualOverride ? "MANUAL" : (isSystemInitialized ? "AUTO" : "WAITING");
        doc["override"] = isManualOverride;
        doc["azimuth"] = currentAzimuth;
        doc["elevation"] = currentElevation;
        
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });

    // --- MOTOR CONTROL (RESTORED) ---
    server.on("/motor", HTTP_GET, [](AsyncWebServerRequest *request){
        if (request->hasParam("type") && request->hasParam("dir")) {
            String type = request->getParam("type")->value();
            int dir = request->getParam("dir")->value().toInt();

            // Force Manual Mode when App takes control
            isManualOverride = true; 

            if (type == "clean") {
                motorSystem.setCleaningMotor(dir, 255);
                request->send(200, "text/plain", "Clean Motor OK");
            } else if (type == "tilt") {
                motorSystem.setTiltMotor(dir);
                request->send(200, "text/plain", "Tilt Motor OK");
            } else if (type == "all" && dir == 0) {
                motorSystem.stopAll();
                request->send(200, "text/plain", "Emergency Stop");
            } else {
                request->send(400, "text/plain", "Unknown Type");
            }
        } else {
            request->send(400, "text/plain", "Missing Params");
        }
    });

    // --- MODE ---
    server.on("/mode", HTTP_GET, [](AsyncWebServerRequest *request){
        if (request->hasParam("manual")) {
            int val = request->getParam("manual")->value().toInt();
            isManualOverride = (val == 1);
            if (!isManualOverride) motorSystem.stopAll(); // Safety stop when switching to Auto
            request->send(200, "text/plain", isManualOverride ? "Manual Mode" : "Auto Mode");
        } else {
            request->send(400, "text/plain", "Missing 'manual' param");
        }
    });

    // --- UPDATE ---
    AsyncCallbackJsonWebHandler *handler = new AsyncCallbackJsonWebHandler("/update", 
        [](AsyncWebServerRequest *request, JsonVariant &json) {
            JsonObject jsonObj = json.as<JsonObject>();
            if (jsonObj.containsKey("lat") && jsonObj.containsKey("lon")) {
                currentLat = jsonObj["lat"];
                currentLon = jsonObj["lon"];
                
                if (sunPosition) delete sunPosition;
                sunPosition = new SolarPosition(currentLat, currentLon);
                isSystemInitialized = true; 

                request->send(200, "application/json", "{\"message\":\"Sync Complete\"}");
            } else {
                request->send(400, "application/json", "{\"message\":\"Invalid JSON\"}");
            }
    });
    server.addHandler(handler);
}