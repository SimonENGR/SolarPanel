/**
 * @file main.cpp
 * @brief Main entry point for the ESP32 Solar Tracker Firmware.
 * 
 * Orchestrates FreeRTOS tasks (Network, Solar Math, Motor Control), handles 
 * system initialization, and manages the primary hardware loop.
 */

#include "BleProvisioningManager.h"
#include "Globals.h"
#include "MotorDriver.h"
#include "SensorInputManager.h"
#include "SolarWebServer.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <HTTPClient.h>
#include <NTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ======================================================================
// GLOBALS
// ======================================================================
volatile bool isWifiConnected = false;
volatile bool isSystemInitialized = false;
volatile int  trackingMode = MODE_AUTOMATIC; // 0=AUTO, 1=SEMI, 2=MANUAL
volatile bool forceTrackingUpdate = false;
volatile int  weatherOverrideAngle = -1;      // -1 = no override
volatile bool weatherPendingConfirmation = false;
String currentWeatherCondition = "Unknown";
volatile double currentLat = 0.0;
volatile double currentLon = 0.0;
volatile double currentAzimuth = 0;
volatile double currentElevation = 0;
SolarPosition *sunPosition = nullptr;

// OpenWeatherMap API key (same as Android app)
const char* OWM_API_KEY = "a00650a3a9459e3365cebef437c717e9";

// ======================================================================
// PIN DEFINITIONS
// ======================================================================
#define PIN_CLEAN_R    32   // Cleaning motor IBT-2 Right
#define PIN_CLEAN_L    33   // Cleaning motor IBT-2 Left
#define PIN_TILT_ENA   21   // Stepper enable (LOW = enabled)
#define PIN_TILT_STEP  22   // Stepper pulse (PUL+)
#define PIN_TILT_DIR   23   // Stepper direction (DIR+)
#define PIN_ENC_A      18   // Tilt encoder A+ (Grey wire)
#define PIN_ENC_B      19   // Tilt encoder B+ (Green wire)
#define PIN_LIMIT      15   // Tilt limit switch (home / 0°)
#define PIN_CURRENT    34   // Solar current sensor (placeholder)
#define PIN_IR_1       13   // IR sensor 1 (placeholder)
#define PIN_IR_2       14   // IR sensor 2 (placeholder)
// --- Wiper / Cleaning motor ---
// IBT-2 drive pins reuse PIN_CLEAN_R / PIN_CLEAN_L above (no change)
// Limit switches only — no encoder needed
// TODO: assign real GPIO numbers once wired
#define PIN_WIPER_LIMIT_BOTTOM  16   // Bottom limit switch — gray wire signal
#define PIN_WIPER_LIMIT_TOP      4   // Top    limit switch — gray wire signal

// ======================================================================
// MODULE INSTANTIATION
// ======================================================================
MotorDriver motorSystem(PIN_CLEAN_R, PIN_CLEAN_L, PIN_TILT_ENA, PIN_TILT_STEP,
                        PIN_TILT_DIR, PIN_ENC_A, PIN_ENC_B, PIN_LIMIT,
                        PIN_WIPER_LIMIT_BOTTOM, PIN_WIPER_LIMIT_TOP);

SolarWebServer webSystem(NULL, NULL);
SensorInputManager sensorSystem(PIN_CURRENT, PIN_IR_1, PIN_IR_2);
BleProvisioningManager bleManager;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000);

unsigned long lastCleaningTime = 0;
const unsigned long CLEANING_COOLDOWN = 60000;

// TASKS
TaskHandle_t NetworkTask;
TaskHandle_t SolarTask;
TaskHandle_t MotorTask;

// NETWORK TASK
void networkTaskCode(void *parameter) {
  timeClient.begin();
  while (true) {
    if (isWifiConnected) {
      timeClient.update();
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// SOLAR TASK
void solarTaskCode(void *parameter) {
  while (!isSystemInitialized) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  while (true) {
    if (sunPosition != nullptr) {
      time_t now = timeClient.getEpochTime();
      currentAzimuth = sunPosition->getSolarAzimuth(now);
      currentElevation = sunPosition->getSolarElevation(now);
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

// ======================================================================
// WEATHER POLLING
// ======================================================================

/**
 * @brief Maps an OWM weather condition string to a panel override angle.
 * @param condition  The weather[0].main string from OWM (e.g. "Clouds", "Rain")
 * @param cloudPct   Cloud coverage percentage (0-100) from clouds.all
 * @return -1 if no override needed, else the target angle.
 */
int mapWeatherToAngle(const String &condition, int cloudPct) {
  if (condition == "Clouds") {
    // Only override for heavy overcast (>80%), not spotty clouds
    if (cloudPct > 80) return 90;
    return -1; // Partial clouds → keep sun tracking
  }
  if (condition == "Rain")         return 60;  // Rain → steep runoff
  if (condition == "Drizzle")      return 60;
  if (condition == "Thunderstorm") return 60;
  if (condition == "Snow")         return 15;  // Snow → near-vertical
  return -1; // Clear, Mist, Haze, etc. → no override
}

/**
 * @brief Fetches current weather from OpenWeatherMap and updates globals.
 *        Called every 10 minutes from motorTaskCode.
 */
void pollWeather() {
  if (!isWifiConnected || currentLat == 0.0) return;

  HTTPClient http;
  String url = "http://api.openweathermap.org/data/2.5/weather?lat="
               + String(currentLat, 4) + "&lon=" + String(currentLon, 4)
               + "&appid=" + String(OWM_API_KEY);

  http.begin(url);
  http.setTimeout(10000);
  int httpCode = http.GET();

  if (httpCode == 200) {
    String payload = http.getString();
    StaticJsonDocument<1024> doc;
    DeserializationError err = deserializeJson(doc, payload);

    if (!err) {
      String condition = doc["weather"][0]["main"].as<String>();
      int cloudPct = doc["clouds"]["all"] | 0; // Cloud coverage 0-100%
      int newOverride = mapWeatherToAngle(condition, cloudPct);

      Serial.printf("[WEATHER] Condition: %s, Clouds: %d%% → Override: %d°\n",
                    condition.c_str(), cloudPct, newOverride);

      currentWeatherCondition = condition;

      // Only update if weather has changed
      if (newOverride != weatherOverrideAngle) {
        if (trackingMode == MODE_AUTOMATIC) {
          // AUTO: Apply immediately
          weatherOverrideAngle = newOverride;
          forceTrackingUpdate = true;
        } else if (trackingMode == MODE_SEMI_AUTO) {
          // SEMI-AUTO: Set pending, wait for user confirmation
          weatherOverrideAngle = newOverride;
          if (newOverride >= 0) {
            weatherPendingConfirmation = true;
            Serial.println("[WEATHER] Semi-auto: waiting for user confirmation");
          } else {
            // Clearing override doesn't need confirmation
            weatherPendingConfirmation = false;
            forceTrackingUpdate = true;
          }
        }
        // MANUAL: Do nothing, user controls everything
      }
    } else {
      Serial.printf("[WEATHER] JSON parse error: %s\n", err.c_str());
    }
  } else {
    Serial.printf("[WEATHER] HTTP error: %d\n", httpCode);
  }

  http.end();
}

// ======================================================================
// MOTOR LOGIC TASK
// ======================================================================
void motorTaskCode(void *parameter) {
  motorSystem.begin();
  sensorSystem.begin();

  // CRITICAL: Home to limit switch FIRST on startup (before app connects)
  Serial.println("[STARTUP] Initiating homing sequence...");
  motorSystem.homeToZero();  // Blocking: drives motor to limit switch directly
  Serial.println("[STARTUP] Homing procedure finished.");

  // Wait for System Init (app must sync lat/lon before tracking begins)
  while (!isSystemInitialized) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  while (true) {
    // 1. Update Sensors
    sensorSystem.update();

    // 2. Logic Decision Tree
    if (trackingMode == MODE_MANUAL) {
      // In Manual Mode, we do nothing here.
      // The WebServer sets the state, the void loop() drives the motor.
      vTaskDelay(
          100 /
          portTICK_PERIOD_MS); // CRITICAL: Yield CPU to prevent starving loop()
    } else {
      // Auto / Semi-Auto Mode Logic
      unsigned long now = millis();
/*
      // Sensor-triggered cleaning (both Auto and Semi-Auto)
      if ((sensorSystem.isCurrentBelowThreshold() ||
           sensorSystem.areIRSensorsReflected()) &&
          (now - lastCleaningTime > CLEANING_COOLDOWN)) {
        Serial.println("[AUTO] Triggering Maintenance...");
        motorSystem.initiateFullCleanCycle();
        lastCleaningTime = millis();
      }
*/
      // Auto-Tracking + Weather Logic
      if (sunPosition != nullptr) {
        static unsigned long lastMoveTime = 0;
        if (forceTrackingUpdate || now - lastMoveTime > 600000 ||
            lastMoveTime == 0) {
          forceTrackingUpdate = false;

          // Poll weather on the same 10-min interval
          pollWeather();

          float targetAngle = 0.0;

          // Weather override takes priority (AUTO mode applies immediately)
          if (weatherOverrideAngle >= 0 && trackingMode == MODE_AUTOMATIC) {
            targetAngle = (float)weatherOverrideAngle;
            Serial.printf("[AUTO] Weather override active: %d°\n",
                          weatherOverrideAngle);
          } else if (weatherOverrideAngle >= 0 &&
                     trackingMode == MODE_SEMI_AUTO &&
                     !weatherPendingConfirmation) {
            // Semi-auto: only apply after user confirmed
            targetAngle = (float)weatherOverrideAngle;
            Serial.printf("[SEMI] Weather confirmed, applying: %d°\n",
                          weatherOverrideAngle);
          } else if (currentElevation > 0.0) {
            // Daytime: target sun elevation (no weather override)
            targetAngle = (float)currentElevation;
            if (targetAngle > 85.0f)
              targetAngle = 85.0f; // Soft cap near horizontal
          } else {
            // Nighttime: return to vertical bounds (0 degrees)
            targetAngle = 0.0f;
          }

          // Fetch current tilt to apply deadband
          float currentAngle = motorSystem.getAngleDegrees();

          // Deadband: Only move if difference is > 2.0 degrees
          if (abs(currentAngle - targetAngle) > 2.0f) {
            Serial.printf(
                "[TRACK] Adjusting panel: %.1f° -> target: %.1f°\n",
                currentAngle, targetAngle);
            motorSystem.moveToAngle(targetAngle);
            lastMoveTime = now;
          }
        }
      }

      // Run this task at a moderate speed (10Hz is plenty for decision making)
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}

// --- HELPER: Attempt WiFi Connection ---
bool attemptWifiConnection(String ssid, String password, int maxRetries = 20) {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.begin(ssid.c_str(), password.c_str());

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < maxRetries) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ WiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    return true;
  } else {
    Serial.println("\n✗ WiFi Connection Failed");
    return false;
  }
}

// --- SETUP ---
void setup() {
  // Disable brownout detector (needed for buck converter power supplies)
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(9600);

  // ====================================================================
  // BLE HARDWARE DIAGNOSTIC
  // ====================================================================
  Serial.println("\n--- BLE Diagnostic ---");
  BLEDevice::init(""); // Temporary init to test hardware
  if (BLEDevice::getInitialized()) {
    Serial.println("[BLE] ✓ Bluetooth initialized successfully");
    Serial.print("[BLE] MAC Address: ");
    Serial.println(BLEDevice::getAddress().toString().c_str());
  } else {
    Serial.println("[BLE] ✗ Bluetooth FAILED to initialize!");
  }
  BLEDevice::deinit(false);
  Serial.println("--- End BLE Diagnostic ---\n");

  // ====================================================================
  // SMART STARTUP LOGIC
  // ====================================================================

  // STEP 1: Try to load saved WiFi credentials
  bool hasSavedCredentials = bleManager.loadSavedCredentials();

  if (hasSavedCredentials) {
    Serial.println("\n>>> ATTEMPTING AUTO-CONNECT WITH SAVED CREDENTIALS <<<");

    String savedSSID = bleManager.getSSID();
    String savedPass = bleManager.getPassword();

    // Try to connect with saved credentials
    isWifiConnected = attemptWifiConnection(savedSSID, savedPass, 20);

    if (isWifiConnected) {
      // SUCCESS! WiFi connected automatically
      Serial.println(">>> AUTO-CONNECT SUCCESS <<<");

      // Start BLE in "status broadcast" mode
      bleManager.beginStatusBroadcast();
      bleManager.sendStatus("READY:" + WiFi.localIP().toString());
      Serial.println("[BLE] Bluetooth staying active for app communication");

    } else {
      // FAILED! Saved credentials are invalid (wrong password or network
      // gone)
      Serial.println(">>> AUTO-CONNECT FAILED - CLEARING OLD CREDENTIALS <<<");
      bleManager.clearSavedCredentials();
      hasSavedCredentials = false; // Fall through to provisioning
    }
  }

  // STEP 2: If no saved credentials OR auto-connect failed, enter
  // provisioning mode
  if (!isWifiConnected) {
    Serial.println("\n>>> ENTERING BLE PROVISIONING MODE <<<");
    bleManager.begin(); // Full provisioning mode

    // Wait for phone to send credentials
    while (!isWifiConnected) {
      delay(500); // Wait for credentials

      if (bleManager.hasCredentials()) {
        String ssid = bleManager.getSSID();
        String pass = bleManager.getPassword();

        Serial.println("\n>>> CREDENTIALS RECEIVED VIA BLE <<<");
        bleManager.sendStatus("CONNECTING...");

        // Attempt connection
        isWifiConnected = attemptWifiConnection(ssid, pass, 20);

        if (isWifiConnected) {
          // Notify phone of success
          bleManager.sendStatus("IP:" + WiFi.localIP().toString());
          delay(4000);       // Give phone time to receive
          bleManager.stop(); // Stop BLE, switch to WiFi
        } else {
          // Notify phone of failure
          bleManager.sendStatus("FAILED");
          bleManager.clearSavedCredentials(); // Clear bad credentials
        }
      }

      delay(100); // Small delay in provisioning loop
    }
  }

  // ====================================================================
  // PHASE 2: SYSTEM STARTUP (After WiFi is connected)
  // ====================================================================
  Serial.println("\n>>> PHASE 2: SYSTEM STARTUP <<<");

  webSystem.begin();

  // Start FreeRTOS Tasks
  xTaskCreatePinnedToCore(networkTaskCode, "NetTask", 4096, NULL, 1,
                          &NetworkTask, 0);
  xTaskCreatePinnedToCore(solarTaskCode, "SunTask", 4096, NULL, 1, &SolarTask,
                          1);
  xTaskCreatePinnedToCore(motorTaskCode, "MotTask", 4096, NULL, 2, &MotorTask,
                          1);

  Serial.println(">>> SYSTEM READY <<<\n");
}

// --- MAIN LOOP ---
void loop() {
  // This runs as fast as the CPU allows (~200kHz+)
  // giving us a smooth stepper pulse train
  motorSystem.tick();
  // Add this debug command
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'r' || cmd == 'R') {
      Serial.println("\n>>> CLEARING WIFI CREDENTIALS <<<");
      bleManager.clearSavedCredentials();
      Serial.println(">>> RESTARTING ESP32 <<<");
      delay(1000);
      ESP.restart();
    }

    if (cmd == 'e' || cmd == 'E') {
      // Encoder diagnostic: print pin states rapidly
      Serial.println("\n--- ENCODER DIAGNOSTIC ---");
      Serial.printf("Pin A (GPIO %d), Pin B (GPIO %d)\n", PIN_ENC_A, PIN_ENC_B);
      Serial.printf("Current position: %ld\n",
                    motorSystem.getEncoderPosition());
      Serial.printf("Current angle: %.2f°\n", motorSystem.getAngleDegrees());
      Serial.println("Reading pins 20 times (turn motor shaft slowly):");
      for (int i = 0; i < 20; i++) {
        Serial.printf("  A=%d  B=%d  pos=%ld\n", digitalRead(PIN_ENC_A),
                      digitalRead(PIN_ENC_B), motorSystem.getEncoderPosition());
        delay(200);
      }
      Serial.println("--- END DIAGNOSTIC ---\n");
    }
  }
}