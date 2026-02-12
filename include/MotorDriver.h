#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

class MotorDriver {
    private:
        int statusLedPin;

        // --- Cleaning Motor (IBT-2 PWM) ---
        int cleanRPin;
        int cleanLPin;

        // --- Tilt Motor (Stepper + Encoder) ---
        int pinStep;
        int pinDir;
        int pinEncA;
        int pinEncB;

        // State Tracking for Stepper
        // 0 = Idle, 1 = Moving Up, -1 = Moving Down
        volatile int tiltState; 
        
        // Tuning for speed (microseconds)
        const int STEP_DELAY = 800;

        // PWM Settings
        const int PWM_FREQ = 5000;
        const int PWM_RES = 8;
        const int PWM_CH_R = 0;
        const int PWM_CH_L = 1;

    public:
        // Constructor
        MotorDriver(int ledPin, int cleanR, int cleanL, int stepPin, int dirPin, int encA, int encB);        
        
        // Setup
        void begin();

        // Main Loop: MUST be called repeatedly in void loop()
        // This handles the physical stepping logic
        void tick(); 
        
        // Helper function (Legacy support if needed, otherwise maps to tick)
        void update(); 

        // --- Movement Logic ---
        // Direction: 1 = Forward, -1 = Backward, 0 = Stop
        // Default speed is 255 if not specified
        void setCleaningMotor(int direction, int speed = 255);
        
        // Direction: 1 = Up (Step Fwd), -1 = Down (Step Rev), 0 = Stop
        void setTiltMotor(int direction);
        
        void stopAll();

        // --- Visual Signals ---
        void signalWaiting();
        void signalTracking();
        void signalManual();
        int getTiltState();

        // --- Encoder Logic ---
        long getEncoderPosition();
        void resetEncoder();

        // Legacy Automated Cycle
        void initiateCleaningCycle();
};

#endif