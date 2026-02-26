#include "BleProvisioningManager.h"
#include "Globals.h"
#include "MotorDriver.h"
#include "SensorInputManager.h"
#include "SolarWebServer.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"
#include <Arduino.h>
#include <BLEDevice.h>
#include <NTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// --- GLOBALS ---
volatile bool isWifiConnected = false;
volatile bool isSystemInitialized = false;
volatile bool isManualOverride = false;
volatile double currentLat = 0.0;
volatile double currentLon = 0.0;
volatile double currentAzimuth = 0;
volatile double currentElevation = 0;
SolarPosition *sunPosition = nullptr;

// --- PIN DEFINITIONS ---
#define PIN_CLEAN_R 32   // Permanent
#define PIN_CLEAN_L 33   // Permanent
#define PIN_TILT_ENA 21  // Enable pin (LOW = enabled)
#define PIN_TILT_STEP 22 // Pulse pin (PUL+)
#define PIN_TILT_DIR 23  // Direction pin (DIR+)
#define PIN_ENC_A 18     // Encoder A+ (Grey wire)
#define PIN_ENC_B 19     // Encoder B+ (Green wire)
#define PIN_LIMIT 15     // Limit switch (home position)
#define PIN_CURRENT 34   // Placeholder value
#define PIN_IR_1 13      // Placeholder value
#define PIN_IR_2 14      // Placeholder value

// --- MODULE INSTANTIATION ---
MotorDriver motorSystem(PIN_CLEAN_R, PIN_CLEAN_L, PIN_TILT_ENA, PIN_TILT_STEP,
                        PIN_TILT_DIR, PIN_ENC_A, PIN_ENC_B, PIN_LIMIT);

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

// MOTOR LOGIC TASK
void motorTaskCode(void *parameter) {
  motorSystem.begin();
  sensorSystem.begin();

  // Wait for System Init
  while (!isSystemInitialized) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  while (true) {
    // 1. Update Sensors
    sensorSystem.update();

    // 2. Logic Decision Tree
    if (isManualOverride) {
      // In Manual Mode, we do nothing here.
      // The WebServer sets the state, the void loop() drives the motor.
    } else {
      // Auto Mode Logic
      unsigned long now = millis();
      if ((sensorSystem.isCurrentBelowThreshold() ||
           sensorSystem.areIRSensorsReflected()) &&
          (now - lastCleaningTime > CLEANING_COOLDOWN)) {
        Serial.println("[AUTO] Triggering Maintenance...");
        motorSystem.initiateCleaningCycle();
        lastCleaningTime = millis();
      }

      // FUTURE: Auto-Tracking logic would go here
      // Example: if (elevation > x) motorSystem.setTiltMotor(1);
    }

    // Run this task at a moderate speed (10Hz is plenty for decision making)
    vTaskDelay(100 / portTICK_PERIOD_MS);
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

  long startWait = millis();
  while (!Serial && (millis() - startWait < 3000)) {
    delay(10);
  }
  delay(2000); // Extra delay for external power supply stabilization

  Serial.println("\n\n>>> ESP32 SOLAR TRACKER STARTING <<<");
  motorSystem.begin();

  // ====================================================================
  // BLE HARDWARE CHECK
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
      // FAILED! Saved credentials are invalid (wrong password or network gone)
      Serial.println(">>> AUTO-CONNECT FAILED - CLEARING OLD CREDENTIALS <<<");
      bleManager.clearSavedCredentials();
      hasSavedCredentials = false; // Fall through to provisioning
    }
  }

  // STEP 2: If no saved credentials OR auto-connect failed, enter provisioning
  // mode
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
