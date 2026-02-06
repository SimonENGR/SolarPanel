#ifndef SENSOR_INPUT_MANAGER_H
#define SENSOR_INPUT_MANAGER_H

#include <Arduino.h>
#define THRESHOLD_SOLAR_CURRENT 0.5 // Amps (Example: If below 0.5A, panel is dirty) THIS VALUE WILL NEED TO BE UPDATED

class SensorInputManager {
    private:
        int pinCurrent;
        int pinIR1;
        int pinIR2;

        // Raw Data Storage
        float lastCurrentReading;
        bool ir1State;
        bool ir2State;

        float solarCurrentAmps;
        bool ir1Blocked;
        bool ir2Blocked;

    public:
        // Constructor: Accepts the 3 pins
        SensorInputManager(int currentPin, int ir1Pin, int ir2Pin);

        // Initialization (pinMode setup)
        void begin();

        // The "Listener" - Call this inside your loop
        void update();

        // Getters - So other tasks can read the data safely
        float getMotorCurrent(); // Returns Amps
        bool isIR1Triggered();   // Returns true/false
        bool isIR2Triggered();   // Returns true/false

        // --- Life Cycle Functions ---
        // 1. Efficiency Check
        bool isCurrentBelowThreshold(); 
        
        // 2. Debris Check (The "Arm" Logic)
        // Returns true if sensors detect an object on the panel
        bool areIRSensorsReflected();

        // Safety Check
        bool isSafeToMove();     // Returns false if current too high or limits hit
};

#endif