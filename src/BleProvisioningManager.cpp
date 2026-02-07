#include "BleProvisioningManager.h"
#include <BLESecurity.h>  

// --- BLE SERVER CALLBACKS ---
class ServerCallbacks: public BLEServerCallbacks {
    BleProvisioningManager* manager;
public:
    ServerCallbacks(BleProvisioningManager* m) { manager = m; }
    void onConnect(BLEServer* pServer) { manager->setDeviceConnected(true); }
    void onDisconnect(BLEServer* pServer) { 
        manager->setDeviceConnected(false); 
        // Restart advertising so others can connect
        pServer->getAdvertising()->start();
    }
};

class DataCallbacks: public BLECharacteristicCallbacks {
    BleProvisioningManager* manager;
    bool isSSID;
public:
    DataCallbacks(BleProvisioningManager* m, bool isSsidField) { 
        manager = m; 
        isSSID = isSsidField;
    }
    void onWrite(BLECharacteristic *pCharacteristic) {
        String value = pCharacteristic->getValue().c_str();
        if (value.length() > 0) {
            String currentSSID = manager->getSSID();
            String currentPass = manager->getPassword();
            
            if (isSSID) manager->setCredentials(value, currentPass);
            else        manager->setCredentials(currentSSID, value);
        }
    }
};

// --- IMPLEMENTATION ---

BleProvisioningManager::BleProvisioningManager() {
    deviceConnected = false;
    credentialsReceived = false;
}

void BleProvisioningManager::begin() {
    BLEDevice::init("ESP32-Solar-Prov"); // Name shown on Phone
    BLESecurity *pSecurity = new BLESecurity();
    
    pSecurity->setCapability(ESP_IO_CAP_NONE); // No buttons/display, just connect
    pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
    pSecurity->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
    // ---------------------------

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks(this));

    pService = pServer->createService(SERVICE_UUID);

    // SSID Characteristic (Write only)
    pSsidCharacteristic = pService->createCharacteristic(
                                CHAR_SSID_UUID,
                                BLECharacteristic::PROPERTY_WRITE
                            );
    pSsidCharacteristic->setCallbacks(new DataCallbacks(this, true));

    // Password Characteristic (Write only)
    pPassCharacteristic = pService->createCharacteristic(
                                CHAR_PASS_UUID,
                                BLECharacteristic::PROPERTY_WRITE
                            );
    pPassCharacteristic->setCallbacks(new DataCallbacks(this, false));

    // Status Characteristic (Read/Notify - Phone reads this)
    pStatusCharacteristic = pService->createCharacteristic(
                                CHAR_STATUS_UUID,
                                BLECharacteristic::PROPERTY_READ |
                                BLECharacteristic::PROPERTY_NOTIFY
                            );

    pService->start();
    
    // Start Advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); 
    BLEDevice::startAdvertising();
    Serial.println("BLE Provisioning Started. Waiting for Phone...");
}

void BleProvisioningManager::update() {
    // Maintenance if needed
}

void BleProvisioningManager::stop() {
    BLEDevice::deinit(true); // Turn off radio to save power
    Serial.println("BLE Stopped.");
}

bool BleProvisioningManager::isClientConnected() { return deviceConnected; }
bool BleProvisioningManager::hasCredentials() { return credentialsReceived; }
String BleProvisioningManager::getSSID() { return receivedSSID; }
String BleProvisioningManager::getPassword() { return receivedPass; }

void BleProvisioningManager::sendStatus(String status) {
    pStatusCharacteristic->setValue(status.c_str());
    pStatusCharacteristic->notify();
}

void BleProvisioningManager::setCredentials(String ssid, String pass) {
    receivedSSID = ssid;
    receivedPass = pass;
    if (receivedSSID.length() > 0 && receivedPass.length() > 0) {
        // Simple check: if both fields have text, we assume we are ready to try
        credentialsReceived = true;
    }
}

void BleProvisioningManager::setDeviceConnected(bool connected) {
    deviceConnected = connected;
}