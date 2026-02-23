#ifndef BLE_PROVISIONING_MANAGER_H
#define BLE_PROVISIONING_MANAGER_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// UUIDs (Must match Android App)
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_SSID_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_PASS_UUID      "82544256-1d18-4066-976d-1d6836932486"
#define CHAR_STATUS_UUID    "e97c992c-559d-48d6-96b0-754784411135"

class BleProvisioningManager {
private:
    BLEServer *pServer;
    BLEService *pService;
    BLECharacteristic *pSsidCharacteristic;
    BLECharacteristic *pPassCharacteristic;
    BLECharacteristic *pStatusCharacteristic;

    bool deviceConnected;
    bool credentialsReceived;
    bool bleActive;
    
    String receivedSSID;
    String receivedPass;

    // Internal helper to save credentials to NVS
    void saveCredentials(String ssid, String password);

public:
    BleProvisioningManager();

    // Start BLE in full provisioning mode (for first-time setup)
    void begin();
    
    // Start BLE in status-only broadcast mode (when WiFi already configured)
    void beginStatusBroadcast();
    
    void update();
    void stop();

    // Load previously saved WiFi credentials from flash
    bool loadSavedCredentials();
    
    // Clear saved credentials (useful for factory reset)
    void clearSavedCredentials();

    // Status Getters
    bool isClientConnected();
    bool hasCredentials();
    String getSSID();
    String getPassword();

    // Send status update to phone via BLE
    void sendStatus(String status);

    // Called by BLE callbacks
    void setCredentials(String ssid, String pass);
    void setDeviceConnected(bool connected);
};

#endif
