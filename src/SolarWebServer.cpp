#include "SolarWebServer.h"
#include "MotorDriver.h"

// Allows the web server to access the global motor instance defined in main.cpp
extern MotorDriver motorSystem;

// Constructor
SolarWebServer::SolarWebServer(const char *ssid, const char *password)
    : server(80), ssid(ssid), password(password) {}

void SolarWebServer::begin() {

  Serial.print("Web Server Starting on IP: ");
  Serial.println(WiFi.localIP());

  // 2. Start mDNS
  if (MDNS.begin("esp32-solar")) {
    Serial.println("mDNS responder started");
    // Add service for discovery
    MDNS.addService("http", "tcp", 80);
  }

  // 3. Setup Routes & Start
  setupRoutes();
  server.begin();
}

void SolarWebServer::setupRoutes() {

  // --- DISCOVERY ENDPOINT (NEW!) ---
  // This allows the phone to check if ESP32 is ready without BLE
  server.on("/discover", HTTP_GET, [](AsyncWebServerRequest *request) {
    // Enable CORS for any origin
    AsyncWebServerResponse *response = request->beginResponse(
        200, "application/json",
        "{\"status\":\"ready\",\"device\":\"ESP32-Solar-Online\",\"ip\":\"" +
            WiFi.localIP().toString() + "\"}");

    response->addHeader("Access-Control-Allow-Origin", "*");
    request->send(response);

    Serial.println("[WEB] Discovery request received from: " +
                   request->client()->remoteIP().toString());
  });

  // --- STATUS ENDPOINT ---
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    StaticJsonDocument<300> doc;

    // Determine status string
    if (isManualOverride) {
      doc["status"] = "MANUAL";
    } else if (isSystemInitialized) {
      doc["status"] = "AUTO";
    } else {
      doc["status"] = "WAITING";
    }

    doc["override"] = isManualOverride;
    doc["azimuth"] = currentAzimuth;
    doc["elevation"] = currentElevation;
    doc["tilt_angle"] = motorSystem.getAngleDegrees();
    doc["encoder_pos"] = motorSystem.getEncoderPosition();
    doc["limit_triggered"] = motorSystem.isLimitTriggered();

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  // --- ENCODER ENDPOINT ---
  // /encoder              → returns current angle & position
  // /encoder?action=reset → resets encoder to 0°
  server.on("/encoder", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("action")) {
      String action = request->getParam("action")->value();
      if (action == "reset") {
        motorSystem.resetEncoder();
        request->send(200, "application/json",
                      "{\"message\":\"Encoder reset to 0\",\"angle\":0}");
        return;
      }
    }

    StaticJsonDocument<128> doc;
    doc["angle"] = motorSystem.getAngleDegrees();
    doc["position"] = motorSystem.getEncoderPosition();
    doc["ppr"] = 3600;

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  // --- MOTOR CONTROL ENDPOINT ---
  // Called by Android App: /motor?type=clean&dir=1
  server.on("/motor", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("type") && request->hasParam("dir")) {
      String type = request->getParam("type")->value();
      int dir = request->getParam("dir")->value().toInt();

      // CRITICAL: Force Manual Mode when App takes control
      // This prevents the auto-tracker from fighting the user
      isManualOverride = true;

      if (type == "clean") {
        // Direction: 1 (Fwd), -1 (Rev), 0 (Stop)
        motorSystem.setCleaningMotor(dir, 255);
        request->send(200, "application/json",
                      "{\"message\":\"Clean Motor OK\"}");
      } else if (type == "tilt") {
        // Direction: 1 (Up), -1 (Down), 0 (Stop)
        motorSystem.setTiltMotor(dir);
        request->send(200, "application/json",
                      "{\"message\":\"Tilt Motor OK\"}");
      } else if (type == "all" && dir == 0) {
        motorSystem.stopAll();
        request->send(200, "application/json",
                      "{\"message\":\"Emergency Stop\"}");
      } else {
        request->send(400, "application/json",
                      "{\"message\":\"Unknown Type\"}");
      }
    } else {
      request->send(400, "application/json",
                    "{\"message\":\"Missing Params\"}");
    }
  });

  // --- MODE CONTROL ---
  server.on("/mode", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("manual")) {
      int val = request->getParam("manual")->value().toInt();
      isManualOverride = (val == 1);

      // Safety: Stop motors if switching back to Auto to prevent runaway
      if (!isManualOverride) {
        motorSystem.stopAll();
      }

      request->send(200, "application/json",
                    isManualOverride ? "{\"message\":\"Manual Mode\"}"
                                     : "{\"message\":\"Auto Mode\"}");
    } else {
      request->send(400, "application/json",
                    "{\"message\":\"Missing manual param\"}");
    }
  });

  // --- DATA UPDATE (GPS SYNC) ---
  AsyncCallbackJsonWebHandler *handler = new AsyncCallbackJsonWebHandler(
      "/update", [](AsyncWebServerRequest *request, JsonVariant &json) {
        JsonObject jsonObj = json.as<JsonObject>();

        if (jsonObj.containsKey("lat") && jsonObj.containsKey("lon")) {
          currentLat = jsonObj["lat"];
          currentLon = jsonObj["lon"];

          // Re-initialize Solar Calculator
          if (sunPosition)
            delete sunPosition;
          sunPosition = new SolarPosition(currentLat, currentLon);

          // Unlock System
          isSystemInitialized = true;

          request->send(200, "application/json",
                        "{\"message\":\"Sync Complete\"}");
        } else {
          request->send(400, "application/json",
                        "{\"message\":\"Invalid JSON\"}");
        }
      });
  server.addHandler(handler);
}
