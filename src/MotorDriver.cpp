#include "MotorDriver.h"

MotorDriver::MotorDriver(int pinNumber) {
    this->pin = pinNumber;
}

void MotorDriver::begin() {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

void MotorDriver::signalWaiting() {
    // Distinct Pattern: 3 short blips
    for(int i=0; i<3; i++) {
        digitalWrite(pin, HIGH); delay(50);
        digitalWrite(pin, LOW);  delay(50);
    }
}

void MotorDriver::signalTracking() {
    // Heartbeat: Slow Pulse
    digitalWrite(pin, HIGH); delay(1000);
    digitalWrite(pin, LOW);  delay(1000);
}

void MotorDriver::signalManual() {
    // Alert: Fast Blink
    digitalWrite(pin, HIGH); delay(100);
    digitalWrite(pin, LOW);  delay(100);
}

void MotorDriver::initiateCleaningCycle() {
    // PLACEHOLDER: Visual simulation of a cleaning cycle
    // In the future, this will drive the wiper motor forward and back.
    
    // Pattern: "Wipe Right" (Long Blink) -> "Wipe Left" (Long Blink)
    Serial.println(">>> CLEANING CYCLE STARTED <<<");
    
    for(int i=0; i<3; i++) {
        // Swipe Out
        digitalWrite(pin, HIGH); 
        delay(500); // Motor moving...
        
        // Swipe Back
        digitalWrite(pin, LOW);  
        delay(500); // Motor returning...
    }
    
    Serial.println(">>> CLEANING CYCLE COMPLETE <<<");
}