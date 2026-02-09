#include <Arduino.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <WiFi.h>
#include "Globals.h"       // Shared variables
#include "MotorDriver.h"   // Hardware Logic
#include "SolarWebServer.h"// Network Logic
#include "SensorInputManager.h" // Sensor Logic
#include "BleProvisioningManager.h" // Enables Bluetooth Connection

// --- 3.1.1 GLOBAL VARIABLE CREATION ---
volatile bool isWifiConnected = false;
volatile bool isSystemInitialized = false; 
volatile bool isManualOverride = false;
volatile double currentLat = 0.0;
volatile double currentLon = 0.0;
volatile double currentAzimuth = 0;   
volatile double currentElevation = 0; 
SolarPosition* sunPosition = nullptr;

// --- 3.1.2 PIN DEFINITIONS ---

// IMPORTANT: Moved LED to Pin 15 because Pin 2 is needed for the Encoder
#define PIN_LED 15  

// Cleaning Motor (Keep 12/14 as requested)
#define PIN_CLEAN_R 12   
#define PIN_CLEAN_L 14   

// Tilt Motor (Stepper + Encoder) - New Pins
#define PIN_TILT_STEP 23
#define PIN_TILT_DIR  22
#define PIN_ENC_A     2   // Was LED, now Encoder A
#define PIN_ENC_B     4

// Sensors
#define PIN_CURRENT 34   
#define PIN_IR_1 13      
#define PIN_IR_2 27      

// --- MODULE INSTANTIATION ---

// Updated Constructor with all new pins
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

// Cleaning Timer
unsigned long lastCleaningTime = 0;
const unsigned long CLEANING_COOLDOWN = 60000; 

// --- RTOS TASK HANDLES ---
TaskHandle_t NetworkTask;
TaskHandle_t SolarTask;
TaskHandle_t MotorTask;

// --- TASK DEFINITIONS ---

// Network Task
void networkTaskCode(void * parameter) {
  timeClient.begin();
  while(true) {
    if (isWifiConnected) { 
        timeClient.update(); 
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// Solar Task
void solarTaskCode(void * parameter) {
  while (!isSystemInitialized) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  while(true) {
    if (sunPosition != nullptr) {
      time_t now = timeClient.getEpochTime();
      currentAzimuth = sunPosition->getSolarAzimuth(now);
      currentElevation = sunPosition->getSolarElevation(now);
      // Serial.printf("[SUN] Azimuth: %.2f | Elevation: %.2f\n", currentAzimuth, currentElevation);
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

// --- REPLACED MOTOR TASK CODE ---
void motorTaskCode(void * parameter) {
  motorSystem.begin();
  sensorSystem.begin();

  // Gatekeeper: Wait for System Init
  while (!isSystemInitialized) {
    motorSystem.signalWaiting(); 
    vTaskDelay(1000 / portTICK_PERIOD_MS); 
  }

  // Normal Operation
  while(true) {
    
    // 1. Update Sensors
    sensorSystem.update(); 
    
    // 2. Check Motor State
    if (isManualOverride) {
        motorSystem.signalManual();
        
        // --- CRITICAL CHANGE FOR SMOOTHNESS ---
        // If the tilt motor is active, we step immediately and SKIP the vTaskDelay.
        // This creates a continuous, uninterrupted pulse stream.
        if (motorSystem.getTiltState() != 0) {
             motorSystem.update(); 
             // We do NOT call vTaskDelay here. The loop repeats instantly.
             // This effectively "hijacks" the task to focus 100% on the motor.
        } 
        else {
             // If we are NOT moving, we wait to save CPU resources.
             vTaskDelay(1 / portTICK_PERIOD_MS); 
        }

    } else {
        // Auto Mode Logic (Keep existing)
        unsigned long now = millis();
        if ( (sensorSystem.isCurrentBelowThreshold() || sensorSystem.areIRSensorsReflected()) 
             && (now - lastCleaningTime > CLEANING_COOLDOWN) ) 
        {
            Serial.println("[AUTO] Triggering Maintenance...");
            motorSystem.initiateCleaningCycle(); 
            lastCleaningTime = millis();         
        }
        // In auto mode, we are mostly idle, so we keep the delay.
        vTaskDelay(100 / portTICK_PERIOD_MS); 
    }
  }
}

// --- SETUP & LOOP ---
void setup() {
  Serial.begin(9600);
  
  long startWait = millis();
  while (!Serial && (millis() - startWait < 3000)) {
     delay(10);
  }
  delay(1000); 

  Serial.println(">>> SERIAL PORT CONNECTED SUCCESSFULLY <<<");
  motorSystem.begin();
  
  // --- PHASE 1: BLE PROVISIONING ---
  Serial.println(">>> PHASE 1: ENTERING BLE PROVISIONING <<<");
  bleManager.begin();
  
  while (!isWifiConnected) {
    motorSystem.signalWaiting(); 

    if (bleManager.hasCredentials()) {
        String ssid = bleManager.getSSID();
        String pass = bleManager.getPassword();
        
        Serial.print("Attempting WiFi Connection to: ");
        Serial.println(ssid);
        
        bleManager.sendStatus("CONNECTING...");
        WiFi.begin(ssid.c_str(), pass.c_str());
        
        int retry = 0;
        while (WiFi.status() != WL_CONNECTED && retry < 20) {
            delay(500);
            Serial.print(".");
            retry++;
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nWiFi Success!");
            String ipAddress = WiFi.localIP().toString();
            bleManager.sendStatus("IP:" + ipAddress); 
            delay(2000); 
            isWifiConnected = true;
        } else {
            Serial.println("\nWiFi Failed. Waiting for new credentials...");
            bleManager.sendStatus("FAILED");
        }
    }
  }

  // --- PHASE 2: SYSTEM STARTUP ---
  Serial.println(">>> PHASE 2: WIFI CONNECTED. STARTING SYSTEM. <<<");
  
  bleManager.stop();
  webSystem.begin();

  xTaskCreatePinnedToCore(networkTaskCode, "NetTask", 4096, NULL, 1, &NetworkTask, 0);
  xTaskCreatePinnedToCore(solarTaskCode,   "SunTask", 4096, NULL, 1, &SolarTask, 1);
  xTaskCreatePinnedToCore(motorTaskCode,   "MotTask", 4096, NULL, 2, &MotorTask, 1);
}

void loop() { vTaskDelete(NULL); }