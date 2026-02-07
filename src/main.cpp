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
// Memory allocation for state flags
volatile bool isWifiConnected = false;
volatile bool isSystemInitialized = false; // [Ref 3.1.1] The Loop Protection Flag
volatile bool isManualOverride = false;
volatile double currentLat = 0.0;
volatile double currentLon = 0.0;
volatile double currentAzimuth = 0;   
volatile double currentElevation = 0; 
SolarPosition* sunPosition = nullptr;

// --- 3.1.2 MODULE INSTANTIATION ---
#define PIN_LED 2

// Cleaning Motor (IBT-2) - CONFIRMED & WORKING
#define PIN_CLEAN_R 12   // RPWM (Signal Column)
#define PIN_CLEAN_L 14   // LPWM (Signal Column)

// Tilt Motor (Placeholder)
#define PIN_TILT_UP 26   // Changed to safe generic GPIO
#define PIN_TILT_DOWN 25 // Changed to safe generic GPIO

// Sensors - FIXED CONFLICTS
#define PIN_CURRENT 34   // CHANGED: Pin 10 causes crash. Pin 34 is Input-Only ADC.
#define PIN_IR_1 13      // OK
#define PIN_IR_2 27      // CHANGED: Pin 14 was conflicting with Motor LPWM.

MotorDriver motorSystem(PIN_LED, PIN_CLEAN_R, PIN_CLEAN_L, PIN_TILT_UP, PIN_TILT_DOWN);// [Ref 3.1.2] Constructor: LED on Pin 2
SolarWebServer webSystem(NULL, NULL); // [Ref 3.1.2] Constructor: WiFi credentials
SensorInputManager sensorSystem(PIN_CURRENT, PIN_IR_1, PIN_IR_2);
BleProvisioningManager bleManager;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000); 

// Cleaning Timer
unsigned long lastCleaningTime = 0;
const unsigned long CLEANING_COOLDOWN = 60000; // 1 Minute cooldown between cleans

// --- RTOS TASK HANDLES ---
TaskHandle_t NetworkTask;
TaskHandle_t SolarTask;
TaskHandle_t MotorTask;

// --- TASK DEFINITIONS (PHASE 3: GATEKEEPER STATE) ---

// [Ref 3.3.1] Network Task (Core 0)
// Keeps NTP time synced independently of the other tasks.
void networkTaskCode(void * parameter) {
  timeClient.begin();
  while(true) {
    if (isWifiConnected) { // Only update if WiFi is ready
        timeClient.update(); 
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// [Ref 3.3.2] Solar Task (Core 1)
// Calculates Sun Position (Business Logic)
void solarTaskCode(void * parameter) {
  
  // [Ref 3.3.2] Gatekeeper Lock: BLOCKED
  // Sits in this loop until isSystemInitialized becomes true.
  while (!isSystemInitialized) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  // [Ref 3.5.1] Normal Operation: UNLOCKED
  while(true) {
    if (sunPosition != nullptr) {
      time_t now = timeClient.getEpochTime();
      currentAzimuth = sunPosition->getSolarAzimuth(now);
      currentElevation = sunPosition->getSolarElevation(now);
      Serial.printf("[SUN] Azimuth: %.2f | Elevation: %.2f\n", currentAzimuth, currentElevation);
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

// [Ref 3.3.3] Motor Task (Core 1)
// Visual Feedback & Motor Control (Hardware)
void motorTaskCode(void * parameter) {
  motorSystem.begin();
  sensorSystem.begin();
  // [Ref 3.3.3] Gatekeeper Lock: VISUALLY ACTIVE
  // Signals "Triple Blip" pattern to indicate WAITING state.
  while (!isSystemInitialized) {
    motorSystem.signalWaiting(); 
    vTaskDelay(1000 / portTICK_PERIOD_MS); 
  }

  // [Ref 3.5.2] Normal Operation: UNLOCKED
  while(true) {
    // READ SENSORS EVERY LOOP
    sensorSystem.update(); 
// Auto-cleaning logic (Only if NOT in manual mode)
    if (!isManualOverride) {
        unsigned long now = millis();
        if ( (sensorSystem.isCurrentBelowThreshold() || sensorSystem.areIRSensorsReflected()) 
             && (now - lastCleaningTime > CLEANING_COOLDOWN) ) 
        {
            Serial.println("[AUTO] Triggering Maintenance...");
            motorSystem.initiateCleaningCycle(); 
            lastCleaningTime = millis();         
        }
        motorSystem.signalTracking(); 
    } else {
        motorSystem.signalManual();
        // In Manual mode, we do nothing here. 
        // The WebServer directly calls motorSystem.setCleaningMotor(...)
    }
    
    vTaskDelay(100 / portTICK_PERIOD_MS); 
  }
}

// --- PHASE 2: THE SETUP SEQUENCE ---
void setup() {
  Serial.begin(9600);
  // 2. CRITICAL: Wait for USB to connect (Max 3 seconds)
  // This ensures the Serial Monitor is ready before we print anything.
  long startWait = millis();
  while (!Serial && (millis() - startWait < 3000)) {
     delay(10);
  }
  delay(1000); // Extra safety second

  Serial.println(">>> SERIAL PORT CONNECTED SUCCESSFULLY <<<");
  motorSystem.begin();
  
  // ============================================================
  // PHASE 1: BLUETOOTH PROVISIONING LOOP
  // ============================================================
  Serial.println(">>> PHASE 1: ENTERING BLE PROVISIONING <<<");
  bleManager.begin();
  
  // Block setup until WiFi is successfully connected
  while (!isWifiConnected) {
    
    // Visual Feedback: Slow Blink "Blue Waiting"
    motorSystem.signalWaiting(); // Or create a new specific BLE pattern

    // Check if Phone sent credentials
    if (bleManager.hasCredentials()) {
        String ssid = bleManager.getSSID();
        String pass = bleManager.getPassword();
        
        Serial.print("Attempting WiFi Connection to: ");
        Serial.println(ssid);
        
        bleManager.sendStatus("CONNECTING...");
        
        WiFi.begin(ssid.c_str(), pass.c_str());
        
        // Wait up to 10 seconds for connection
        int retry = 0;
        while (WiFi.status() != WL_CONNECTED && retry < 20) {
            delay(500);
            Serial.print(".");
            retry++;
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nWiFi Success!");
            
            // Send the actual IP address back to the phone
            String ipAddress = WiFi.localIP().toString();
            bleManager.sendStatus("IP:" + ipAddress); 
            // ---------------------
            delay(2000); // Give phone time to read success
            isWifiConnected = true;
        } else {
            Serial.println("\nWiFi Failed. Waiting for new credentials...");
            bleManager.sendStatus("FAILED");
            // Loop continues, user must re-enter pass in App
        }
    }
  }

  // ============================================================
  // PHASE 2: SYSTEM STARTUP
  // ============================================================
  Serial.println(">>> PHASE 2: WIFI CONNECTED. STARTING SYSTEM. <<<");
  
  // 1. Shutdown BLE to save memory/power
  bleManager.stop();

  // 2. Start Web Server (Now that we have IP)
  webSystem.begin();

  // 3. Launch RTOS Tasks (Gatekeeper Phase)
  xTaskCreatePinnedToCore(networkTaskCode, "NetTask", 4096, NULL, 1, &NetworkTask, 0);
  xTaskCreatePinnedToCore(solarTaskCode,   "SunTask", 4096, NULL, 1, &SolarTask, 1);
  xTaskCreatePinnedToCore(motorTaskCode,   "MotTask", 4096, NULL, 2, &MotorTask, 1);
}

void loop() { vTaskDelete(NULL); }