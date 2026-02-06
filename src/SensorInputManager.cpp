#include "SensorInputManager.h"

SensorInputManager::SensorInputManager(int currentPin, int ir1Pin, int ir2Pin) {
    this->pinCurrent = currentPin;
    this->pinIR1 = ir1Pin;
    this->pinIR2 = ir2Pin;
}

void SensorInputManager::begin() {
    pinMode(pinIR1, INPUT);
    pinMode(pinIR2, INPUT);
    // INA219 would be initialized here (I2C) in the future
    pinMode(pinCurrent, INPUT); 
}

void SensorInputManager::update() {
    // 1. Read IR Sensors
    // Standard IR sensors usually go LOW when they see an reflection (obstacle)
    ir1Blocked = (digitalRead(pinIR1) == LOW);
    ir2Blocked = (digitalRead(pinIR2) == LOW);

    // 2. Read Solar Current (Placeholder for INA219 I2C read)
    // We simulate a value here. In reality, you'd use ina219.getCurrent_mA();
    int rawAdc = analogRead(pinCurrent); 
    solarCurrentAmps = (float)rawAdc * (3.3 / 4095.0); // Dummy conversion
}

float SensorInputManager::getMotorCurrent() {
    return lastCurrentReading;
}

bool SensorInputManager::isIR1Triggered() {
    return ir1State; // Assuming HIGH means triggered
}

bool SensorInputManager::isIR2Triggered() {
    return ir2State;
}

bool SensorInputManager::isCurrentBelowThreshold() {
    // If we are generating LESS power than expected, panel might be dirty
    return (solarCurrentAmps < THRESHOLD_SOLAR_CURRENT);
}

bool SensorInputManager::areIRSensorsReflected() {
    // Logic: If sensors suddenly give a "close" reading (Blocked),
    // it means there is debris on the panel physically reflecting the beam.
    // If BOTH see something, it's definitely debris.
    if (ir1Blocked && ir2Blocked) {
        return true; 
    }
    return false;
}

bool SensorInputManager::isSafeToMove() {
    // Safety: Don't move if we are actively detecting debris (collision risk?)
    // Or maybe we allow it. For now, we assume safe unless stalled.
    return true; 
}