#include "MotorDriver.h"

MotorDriver::MotorDriver(int ledPin, int cleanR, int cleanL, int tiltUp, int tiltDown) {
    this->statusLedPin = ledPin;
    this->cleanRPin = cleanR;
    this->cleanLPin = cleanL;
    this->tiltUpPin = tiltUp;
    this->tiltDownPin = tiltDown;
}

void MotorDriver::begin() {
    // 1. Setup Status LED
    pinMode(statusLedPin, OUTPUT);
    digitalWrite(statusLedPin, LOW);

    // 2. Setup Cleaning Motor (PWM for IBT-2)
    // Using ledcSetup (Safe for Core v2.x and v3.x compat)
    ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES);
    ledcSetup(PWM_CH_L, PWM_FREQ, PWM_RES);
    
    ledcAttachPin(cleanRPin, PWM_CH_R);
    ledcAttachPin(cleanLPin, PWM_CH_L);

    // Initialize to 0
    ledcWrite(PWM_CH_R, 0);
    ledcWrite(PWM_CH_L, 0);

    // 3. Setup Tilt Motor (Digital Mode for now)
    pinMode(tiltUpPin, OUTPUT);
    pinMode(tiltDownPin, OUTPUT);
    digitalWrite(tiltUpPin, LOW);
    digitalWrite(tiltDownPin, LOW);
}

void MotorDriver::signalWaiting() {
    for(int i=0; i<3; i++) {
        digitalWrite(statusLedPin, HIGH); delay(50);
        digitalWrite(statusLedPin, LOW);  delay(50);
    }
}

void MotorDriver::signalTracking() {
    digitalWrite(statusLedPin, HIGH); delay(1000);
    digitalWrite(statusLedPin, LOW);  delay(1000);
}

void MotorDriver::signalManual() {
    digitalWrite(statusLedPin, HIGH); delay(100);
    digitalWrite(statusLedPin, LOW);  delay(100);
}

void MotorDriver::setCleaningMotor(int direction, int speed) {
    if (direction == 1) { // Forward
        ledcWrite(PWM_CH_L, 0);
        ledcWrite(PWM_CH_R, speed);
        Serial.println("[MOTOR] Cleaning: FWD");
    } 
    else if (direction == -1) { // Reverse
        ledcWrite(PWM_CH_R, 0);
        ledcWrite(PWM_CH_L, speed);
        Serial.println("[MOTOR] Cleaning: REV");
    } 
    else { // Stop
        ledcWrite(PWM_CH_R, 0);
        ledcWrite(PWM_CH_L, 0);
        Serial.println("[MOTOR] Cleaning: STOP");
    }
}

void MotorDriver::setTiltMotor(int direction) {
    // Simple Digital Logic (Relay / H-Bridge style)
    if (direction == 1) { // UP
        digitalWrite(tiltDownPin, LOW);
        digitalWrite(tiltUpPin, HIGH);
        Serial.println("[MOTOR] Tilt: UP");
    } 
    else if (direction == -1) { // DOWN
        digitalWrite(tiltUpPin, LOW);
        digitalWrite(tiltDownPin, HIGH);
        Serial.println("[MOTOR] Tilt: DOWN");
    } 
    else { // STOP
        digitalWrite(tiltUpPin, LOW);
        digitalWrite(tiltDownPin, LOW);
        Serial.println("[MOTOR] Tilt: STOP");
    }
}

void MotorDriver::stopAll() {
    setCleaningMotor(0);
    setTiltMotor(0);
    Serial.println("[MOTOR] EMERGENCY STOP ALL");
}

void MotorDriver::initiateCleaningCycle() {
    // Automated demo cycle
    Serial.println(">>> CLEANING CYCLE STARTED <<<");
    setCleaningMotor(1, 200); // Fwd Speed 200
    delay(2000);
    setCleaningMotor(0);      // Stop
    delay(500);
    setCleaningMotor(-1, 200); // Rev Speed 200
    delay(2000);
    setCleaningMotor(0);       // Stop
    Serial.println(">>> CLEANING CYCLE COMPLETE <<<");
}