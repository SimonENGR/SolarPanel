#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

class MotorDriver {
    private:
        int statusLedPin;
        // --- Cleaning Motor (IBT-2 PWM) ---
        int cleanRPin;
        int cleanLPin;
        // --- Tilt Motor (Placeholder / Relays) ---
        int tiltUpPin;
        int tiltDownPin;

    // PWM Settings
    const int PWM_FREQ = 5000;
    const int PWM_RES = 8;
    const int PWM_CH_R = 0;
    const int PWM_CH_L = 1;
    public:
    // Constructor
    MotorDriver(int ledPin, int cleanR, int cleanL, int tiltUp, int tiltDown);        
    // Setup
    void begin();
    // Visual Signals
    void signalWaiting();
    void signalTracking();
    void signalManual();

    // --- Movement Logic ---
    // Direction: 1 = Forward/Up, -1 = Backward/Down, 0 = Stop
    void setCleaningMotor(int direction, int speed = 255);
    void setTiltMotor(int direction);
    void stopAll();

    // Legacy Automated Cycle
    void initiateCleaningCycle();
    // void moveTo(double azimuth, double elevation);
        
};

#endif