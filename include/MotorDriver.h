#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

class MotorDriver {
    private:
        int pin;
        
    public:
        // Constructor
        MotorDriver(int pinNumber);
        
        // Setup
        void begin();
        
        // Visual Feedback Patterns
        void signalWaiting();  // Triple Blink (Waiting for Sync)
        void signalTracking(); // Slow Pulse (Normal Operation)
        void signalManual();   // Fast Blink (Manual Override)
        
        // Motor Methods
        void initiateCleaningCycle();
        // void moveTo(double azimuth, double elevation);
        
};

#endif