/**
 * @file SolarWebServer.cpp
 * @brief Serves the HTTP API linking the Android App to the ESP32.
 */

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
    doc["wiper_moving"]    = motorSystem.isWiperMoving();
    doc["wiper_stalled"]   = motorSystem.isWiperStalled();

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  // --- ENCODER ENDPOINT ---
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
    doc["cpr"] = 14400;

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  // --- ANGLE CONTROL ENDPOINT ---
  // /angle?target=5.3   → move to 5.3°
  // /angle?delta=0.1    → nudge by +0.1°
  // /angle?delta=-1.0   → nudge by -1.0°
  // /angle              → returns current angle + moving status
  server.on("/angle", HTTP_GET, [](AsyncWebServerRequest *request) {
    // CRITICAL: Force Manual Mode when App explicitly commands a new angle
    isManualOverride = true;

    if (request->hasParam("target")) {
      float target = request->getParam("target")->value().toFloat();
      motorSystem.moveToAngle(target);

      StaticJsonDocument<128> doc;
      doc["message"] = "Moving to target";
      doc["target"] = target;
      doc["current"] = motorSystem.getAngleDegrees();
      String response;
      serializeJson(doc, response);
      request->send(200, "application/json", response);
      return;
    }

    if (request->hasParam("delta")) {
      float delta = request->getParam("delta")->value().toFloat();
      motorSystem.moveByAngle(delta);

      StaticJsonDocument<128> doc;
      doc["message"] = "Nudging";
      doc["delta"] = delta;
      doc["current"] = motorSystem.getAngleDegrees();
      String response;
      serializeJson(doc, response);
      request->send(200, "application/json", response);
      return;
    }

    // No params = status query
    StaticJsonDocument<128> doc;
    doc["angle"] = motorSystem.getAngleDegrees();
    doc["position"] = motorSystem.getEncoderPosition();
    doc["moving"] = motorSystem.isMoving();
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  // --- WIPER CONTROL ENDPOINT ---
  // /wiper?clean=1  → trigger full clean cycle (down → up to rest)
  // /wiper          → returns current wiper status
  server.on("/wiper", HTTP_GET, [](AsyncWebServerRequest *request) {
    isManualOverride = true;

    if (request->hasParam("clean")) {
      motorSystem.initiateFullCleanCycle();

      StaticJsonDocument<128> doc;
      doc["message"]      = "Clean cycle started";
      doc["wiper_moving"] = motorSystem.isWiperMoving();
      String response;
      serializeJson(doc, response);
      request->send(200, "application/json", response);
      return;
    }

    // No params — return current wiper status
    StaticJsonDocument<128> doc;
    doc["wiper_moving"] = motorSystem.isWiperMoving();
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

      // Safety: Stop motors when switching modes to prevent runaway
      motorSystem.stopAll();

      if (!isManualOverride) {
        forceTrackingUpdate = true; // Force an immediate sun sync
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