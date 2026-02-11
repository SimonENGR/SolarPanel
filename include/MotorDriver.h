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
        volatile int tiltState; // 0=Stop, 1=Fwd, -1=Rev
        // Tuning for speed (microseconds)
        const int STEP_DELAY = 800;

    
    // Tuning for speed (microseconds)
    const int STEP_DELAY = 800;
        // PWM Settings
        const int PWM_FREQ = 5000;
        const int PWM_RES = 8;
        const int PWM_CH_R = 0;
        const int PWM_CH_L = 1;

    public:
        // Updated Constructor to include Stepper & Encoder Pins
        MotorDriver(int ledPin, int cleanR, int cleanL, int stepPin, int dirPin, int encA, int encB);        
        
        // Setup
        void begin();

        // Main Loop: MUST be called repeatedly in void loop() or RTOS task
        // This handles the physical stepping logic
        void update();
        void setTiltMotor(int direction);
        void setCleaningMotor(int direction, int speed);
        void stopAll();
        // Visual Signals
        void signalWaiting();
        void signalTracking();
        void signalManual();
        int getTiltState();
        // --- Movement Logic ---
        // Direction: 1 = Forward, -1 = Backward, 0 = Stop
        void setCleaningMotor(int direction, int speed = 255);
        
        // Direction: 1 = Up (Step Fwd), -1 = Down (Step Rev), 0 = Stop
        void setTiltMotor(int direction);
        void tick(); // Replaces "update()"
        void stopAll();

        // --- Encoder Logic ---
        long getEncoderPosition();
        void resetEncoder();

        // Legacy Automated Cycle
        void initiateCleaningCycle();
};

#endif
