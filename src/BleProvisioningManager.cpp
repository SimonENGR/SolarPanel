#include "BleProvisioningManager.h"
#include <BLESecurity.h>
#include <Preferences.h>

// Global Preferences object for NVS storage
Preferences wifiPrefs;

// --- BLE SERVER CALLBACKS ---
class ServerCallbacks: public BLEServerCallbacks {
    BleProvisioningManager* manager;
public:
    ServerCallbacks(BleProvisioningManager* m) { manager = m; }
    void onConnect(BLEServer* pServer) { 
        Serial.println("[BLE] Client connected");
        manager->setDeviceConnected(true); 
    }
    void onDisconnect(BLEServer* pServer) { 
        Serial.println("[BLE] Client disconnected");
        manager->setDeviceConnected(false); 
        // Restart advertising so others can connect
        pServer->getAdvertising()->start();
        Serial.println("[BLE] Advertising restarted");
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
            
            if (isSSID) {
                Serial.println("[BLE] Received SSID: " + value);
                manager->setCredentials(value, currentPass);
            } else {
                Serial.println("[BLE] Received Password: ****");
                manager->setCredentials(currentSSID, value);
            }
        }
    }
};

// --- IMPLEMENTATION ---

BleProvisioningManager::BleProvisioningManager() {
    deviceConnected = false;
    credentialsReceived = false;
    bleActive = false;
    pServer = nullptr;
    pService = nullptr;
    pSsidCharacteristic = nullptr;
    pPassCharacteristic = nullptr;
    pStatusCharacteristic = nullptr;
}

bool BleProvisioningManager::loadSavedCredentials() {
    wifiPrefs.begin("wifi", true); // Open in read-only mode
    
    String savedSSID = wifiPrefs.getString("ssid", "");
    String savedPass = wifiPrefs.getString("password", "");
    
    wifiPrefs.end();
    
    if (savedSSID.length() > 0 && savedPass.length() > 0) {
        receivedSSID = savedSSID;
        receivedPass = savedPass;
        credentialsReceived = true;
        
        Serial.println(">>> LOADED SAVED WiFi CREDENTIALS <<<");
        Serial.print("SSID: "); Serial.println(savedSSID);
        return true;
    }
    
    Serial.println(">>> NO SAVED WiFi CREDENTIALS FOUND <<<");
    return false;
}

void BleProvisioningManager::saveCredentials(String ssid, String password) {
    wifiPrefs.begin("wifi", false); // Open in read-write mode
    
    wifiPrefs.putString("ssid", ssid);
    wifiPrefs.putString("password", password);
    
    wifiPrefs.end();
    
    Serial.println(">>> WiFi CREDENTIALS SAVED TO FLASH <<<");
}

void BleProvisioningManager::clearSavedCredentials() {
    wifiPrefs.begin("wifi", false);
    wifiPrefs.clear();
    wifiPrefs.end();
    
    receivedSSID = "";
    receivedPass = "";
    credentialsReceived = false;
    
    Serial.println(">>> SAVED WiFi CREDENTIALS CLEARED <<<");
}

void BleProvisioningManager::begin() {
    Serial.println("[BLE] Initializing in PROVISIONING mode...");
    
    BLEDevice::init("ESP32-Solar-Prov"); // Name shown on Phone
    BLESecurity *pSecurity = new BLESecurity();
    
    pSecurity->setCapability(ESP_IO_CAP_NONE);
    pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
    pSecurity->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

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
    
    // Add descriptor for notifications
    BLEDescriptor *pDescriptor = new BLEDescriptor((uint16_t)0x2902);
    pStatusCharacteristic->addDescriptor(pDescriptor);

    pService->start();
    
    // Start Advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); 
    BLEDevice::startAdvertising();
    
    bleActive = true;
    Serial.println("[BLE] PROVISIONING MODE ACTIVE - Device: ESP32-Solar-Prov");
}

void BleProvisioningManager::beginStatusBroadcast() {
    Serial.println("[BLE] Initializing in STATUS BROADCAST mode...");
    
    // Start BLE in "status-only" mode - no provisioning needed
    BLEDevice::init("ESP32-Solar-Online");
    
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks(this));

    pService = pServer->createService(SERVICE_UUID);

    // Only create the Status Characteristic
    pStatusCharacteristic = pService->createCharacteristic(
                                CHAR_STATUS_UUID,
                                BLECharacteristic::PROPERTY_READ |
                                BLECharacteristic::PROPERTY_NOTIFY
                            );
    
    // Add descriptor for notifications
    BLEDescriptor *pDescriptor = new BLEDescriptor((uint16_t)0x2902);
    pStatusCharacteristic->addDescriptor(pDescriptor);

    pService->start();
    
    // Start Advertising with different name
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); 
    BLEDevice::startAdvertising();
    
    bleActive = true;
    Serial.println("[BLE] STATUS BROADCAST ACTIVE - Device: ESP32-Solar-Online");
}

void BleProvisioningManager::update() {
    // Maintenance if needed
}

void BleProvisioningManager::stop() {
    if (bleActive) {
        BLEDevice::deinit(true); // Turn off radio to save power
        bleActive = false;
        Serial.println("[BLE] Stopped.");
    }
}

bool BleProvisioningManager::isClientConnected() { return deviceConnected; }
bool BleProvisioningManager::hasCredentials() { return credentialsReceived; }
String BleProvisioningManager::getSSID() { return receivedSSID; }
String BleProvisioningManager::getPassword() { return receivedPass; }

void BleProvisioningManager::sendStatus(String status) {
    if (pStatusCharacteristic != nullptr) {
        Serial.print("[BLE] Sending status: ");
        Serial.println(status);
        
        pStatusCharacteristic->setValue(status.c_str());
        pStatusCharacteristic->notify();
        
        Serial.println("[BLE] Status sent");
    } else {
        Serial.println("[BLE] ERROR: Cannot send status - characteristic is null");
    }
}

void BleProvisioningManager::setCredentials(String ssid, String pass) {
    receivedSSID = ssid;
    receivedPass = pass;
    if (receivedSSID.length() > 0 && receivedPass.length() > 0) {
        credentialsReceived = true;
        saveCredentials(ssid, pass);
    }
}

void BleProvisioningManager::setDeviceConnected(bool connected) {
    deviceConnected = connected;
}
