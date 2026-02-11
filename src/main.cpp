#include <Arduino.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <WiFi.h>
#include "Globals.h"       
#include "MotorDriver.h"   
#include "SolarWebServer.h"
#include "SensorInputManager.h" 
#include "BleProvisioningManager.h" 

// --- GLOBALS ---
volatile bool isWifiConnected = false;
volatile bool isSystemInitialized = false; 
volatile bool isManualOverride = false;
volatile double currentLat = 0.0;
volatile double currentLon = 0.0;
volatile double currentAzimuth = 0;   
volatile double currentElevation = 0; 
SolarPosition* sunPosition = nullptr;

// --- PIN DEFINITIONS ---
#define PIN_LED 15       // Permanent
#define PIN_CLEAN_R 12   // Permanent
#define PIN_CLEAN_L 14   // Permanent
#define PIN_TILT_STEP 23 // Permanent
#define PIN_TILT_DIR  22 // Permanent
#define PIN_ENC_A     2  // Permanent
#define PIN_ENC_B     4  // Permanent
#define PIN_CURRENT 34   // Placeholder value
#define PIN_IR_1 13      // Placeholder value
#define PIN_IR_2 27      // Placeholder value

// --- MODULE INSTANTIATION ---
MotorDriver motorSystem(
    PIN_LED, 
    PIN_CLEAN_R, PIN_CLEAN_L, 
    PIN_TILT_STEP, PIN_TILT_DIR, 
    PIN_ENC_A, PIN_ENC_B
);

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
void networkTaskCode(void * parameter) {
  timeClient.begin();
  while(true) {
    if (isWifiConnected) { 
        timeClient.update(); 
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// SOLAR TASK
void solarTaskCode(void * parameter) {
  while (!isSystemInitialized) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  while(true) {
    if (sunPosition != nullptr) {
      time_t now = timeClient.getEpochTime();
      currentAzimuth = sunPosition->getSolarAzimuth(now);
      currentElevation = sunPosition->getSolarElevation(now);
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

// MOTOR LOGIC TASK
void motorTaskCode(void * parameter) {
  motorSystem.begin();
  sensorSystem.begin();

  // Wait for System Init
  while (!isSystemInitialized) {
    motorSystem.signalWaiting(); 
    vTaskDelay(1000 / portTICK_PERIOD_MS); 
  }

  while(true) {
    // 1. Update Sensors
    sensorSystem.update(); 
    
    // 2. Logic Decision Tree
    if (isManualOverride) {
        // In Manual Mode, we do nothing here. 
        // The WebServer sets the state, the void loop() drives the motor.
        motorSystem.signalManual();
    } 
    else {
        // Auto Mode Logic
        unsigned long now = millis();
        if ( (sensorSystem.isCurrentBelowThreshold() || sensorSystem.areIRSensorsReflected()) 
             && (now - lastCleaningTime > CLEANING_COOLDOWN) ) 
        {
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

// --- SETUP ---
void setup() {
  Serial.begin(9600);
  
  long startWait = millis();
  while (!Serial && (millis() - startWait < 3000)) { delay(10); }
  delay(1000); 

  Serial.println(">>> SERIAL PORT CONNECTED SUCCESSFULLY <<<");
  motorSystem.begin();
  
  // PHASE 1: BLE PROVISIONING
  Serial.println(">>> PHASE 1: ENTERING BLE PROVISIONING <<<");
  bleManager.begin();
  
  while (!isWifiConnected) {
    motorSystem.signalWaiting(); 

    if (bleManager.hasCredentials()) {
        String ssid = bleManager.getSSID();
        String pass = bleManager.getPassword();
        
        Serial.print("Attempting WiFi Connection to: "); Serial.println(ssid);
        bleManager.sendStatus("CONNECTING...");
        
        WiFi.begin(ssid.c_str(), pass.c_str());
        
        int retry = 0;
        while (WiFi.status() != WL_CONNECTED && retry < 20) {
            delay(500); Serial.print("."); retry++;
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nWiFi Success!");
            bleManager.sendStatus("IP:" + WiFi.localIP().toString()); 
            delay(2000); 
            isWifiConnected = true;
        } else {
            Serial.println("\nWiFi Failed.");
            bleManager.sendStatus("FAILED");
        }
    }
  }

  // PHASE 2: SYSTEM STARTUP
  Serial.println(">>> PHASE 2: SYSTEM STARTUP <<<");
  bleManager.stop();
  webSystem.begin();

  xTaskCreatePinnedToCore(networkTaskCode, "NetTask", 4096, NULL, 1, &NetworkTask, 0);
  xTaskCreatePinnedToCore(solarTaskCode,   "SunTask", 4096, NULL, 1, &SolarTask, 1);
  xTaskCreatePinnedToCore(motorTaskCode,   "MotTask", 4096, NULL, 2, &MotorTask, 1);
}

// --- MAIN LOOP ---
void loop() { 
    // This runs as fast as the CPU allows (~200kHz+)
    // giving us a smooth stepper pulse train
    motorSystem.tick();
}
