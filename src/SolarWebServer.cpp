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
    StaticJsonDocument<512> doc;

    // Mode string for display
    if (trackingMode == MODE_MANUAL) {
      doc["status"] = "MANUAL";
    } else if (trackingMode == MODE_SEMI_AUTO) {
      doc["status"] = "SEMI_AUTO";
    } else if (isSystemInitialized) {
      doc["status"] = "AUTO";
    } else {
      doc["status"] = "WAITING";
    }

    doc["override"] = (trackingMode == MODE_MANUAL);
    doc["mode"] = trackingMode; // 0=AUTO, 1=SEMI, 2=MANUAL
    doc["azimuth"] = currentAzimuth;
    doc["elevation"] = currentElevation;
    doc["tilt_angle"] = motorSystem.getAngleDegrees();
    doc["encoder_pos"] = motorSystem.getEncoderPosition();
    doc["limit_triggered"] = motorSystem.isLimitTriggered();
    doc["wiper_moving"]    = motorSystem.isWiperMoving();
    doc["wiper_stalled"]   = motorSystem.isWiperStalled();
    doc["weather_condition"] = currentWeatherCondition;
    doc["weather_override"] = weatherOverrideAngle;
    doc["weather_pending"] = weatherPendingConfirmation;

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
    trackingMode = MODE_MANUAL;

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
    trackingMode = MODE_MANUAL;

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
      trackingMode = MODE_MANUAL;

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
    if (request->hasParam("mode")) {
      int val = request->getParam("mode")->value().toInt();
      trackingMode = constrain(val, 0, 2);

      // Safety: Stop motors when switching modes to prevent runaway
      motorSystem.stopAll();

      if (trackingMode != MODE_MANUAL) {
        forceTrackingUpdate = true; // Force an immediate sun sync
      }

      // Clear weather pending when mode changes
      weatherPendingConfirmation = false;

      const char* modeNames[] = {"Auto", "Semi-Auto", "Manual"};
      String msg = "{\"message\":\"" + String(modeNames[trackingMode]) + " Mode\"}";
      request->send(200, "application/json", msg);

    // Legacy support: also accept ?manual=0|1
    } else if (request->hasParam("manual")) {
      int val = request->getParam("manual")->value().toInt();
      trackingMode = (val == 1) ? MODE_MANUAL : MODE_AUTOMATIC;
      motorSystem.stopAll();
      if (trackingMode != MODE_MANUAL) forceTrackingUpdate = true;
      weatherPendingConfirmation = false;
      request->send(200, "application/json",
                    trackingMode == MODE_MANUAL ? "{\"message\":\"Manual Mode\"}"
                                               : "{\"message\":\"Auto Mode\"}");
    } else {
      request->send(400, "application/json",
                    "{\"message\":\"Missing mode param\"}");
    }
  });

  // --- WEATHER CONTROL (Semi-Auto Confirm + Manual Override) ---
  server.on("/weather", HTTP_GET, [](AsyncWebServerRequest *request) {
    // Semi-auto confirmation
    if (request->hasParam("confirm")) {
      int confirm = request->getParam("confirm")->value().toInt();
      if (confirm == 1) {
        // User approved the weather override
        weatherPendingConfirmation = false;
        forceTrackingUpdate = true;
        Serial.println("[WEATHER] User confirmed weather override");
        request->send(200, "application/json",
                      "{\"message\":\"Weather override confirmed\"}");
      } else {
        // User rejected — clear the override
        weatherPendingConfirmation = false;
        weatherOverrideAngle = -1;
        forceTrackingUpdate = true;
        Serial.println("[WEATHER] User rejected weather override");
        request->send(200, "application/json",
                      "{\"message\":\"Weather override rejected\"}");
      }
      return;
    }

    // Manual weather override (from app buttons)
    if (request->hasParam("condition")) {
      String cond = request->getParam("condition")->value();
      int angle = -1;
      if (cond == "overcast")  angle = 90;
      else if (cond == "rain") angle = 60;
      else if (cond == "snow") angle = 15;
      else if (cond == "wind") angle = 90;
      else if (cond == "clear") angle = -1;

      weatherOverrideAngle = angle;
      weatherPendingConfirmation = false;
      currentWeatherCondition = cond;

      // Immediately move to the target angle
      if (angle >= 0) {
        motorSystem.moveToAngle((float)angle);
        Serial.printf("[WEATHER] Button pressed: %s → moving to %d°\n", cond.c_str(), angle);
      } else {
        // Clear: resume sun tracking
        forceTrackingUpdate = true;
        Serial.println("[WEATHER] Button pressed: clear → resuming sun tracking");
      }

      String msg = "{\"message\":\"Weather set: " + cond + "\",\"angle\":" + String(angle) + "}";
      request->send(200, "application/json", msg);
      return;
    }

    // No params — return current weather state
    StaticJsonDocument<256> doc;
    doc["condition"] = currentWeatherCondition;
    doc["override_angle"] = weatherOverrideAngle;
    doc["pending"] = weatherPendingConfirmation;
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
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