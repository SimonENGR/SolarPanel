#ifndef BLE_PROVISIONING_MANAGER_H
#define BLE_PROVISIONING_MANAGER_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// UUIDs for the Service and Characteristics
// (Generated random UUIDs - standard for BLE)
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_SSID_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_PASS_UUID      "82544256-1d18-4066-976d-1d6836932486"
#define CHAR_STATUS_UUID    "e97c992c-559d-48d6-96b0-754784411135"

class BleProvisioningManager {
    private:
        BLEServer* pServer;
        BLEService* pService;
        BLECharacteristic* pSsidCharacteristic;
        BLECharacteristic* pPassCharacteristic;
        BLECharacteristic* pStatusCharacteristic;
        
        bool deviceConnected;
        String receivedSSID;
        String receivedPass;
        bool credentialsReceived;

    public:
        BleProvisioningManager();
        void begin();
        void update(); // Call in loop
        
        bool isClientConnected();
        bool hasCredentials();
        String getSSID();
        String getPassword();
        
        void sendStatus(String status); // Send "Connected" or "Failed" to phone
        void stop(); // Shut down BLE after WiFi works
        
        // Callback helpers
        void setCredentials(String ssid, String pass);
        void setDeviceConnected(bool connected);
};

#endif