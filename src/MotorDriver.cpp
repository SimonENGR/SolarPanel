#include "MotorDriver.h"

// --- GLOBALS FOR ENCODER INTERRUPT ---
// These must exist outside the class to work with attachInterrupt
volatile long encoderPos = 0;
int globalEncBPin = 0; // Used to pass the B pin number to the ISR

// Interrupt Service Routine
void IRAM_ATTR isr_updateEncoder() {
    // Read the B pin to determine direction (Quadrature encoding)
    if (digitalRead(globalEncBPin) == LOW) {
        encoderPos++;
    } else {
        encoderPos--;
    }
}

// --- CONSTRUCTOR ---
MotorDriver::MotorDriver(int ledPin, int cleanR, int cleanL, int stepPin, int dirPin, int encA, int encB) {
    this->statusLedPin = ledPin;
    
    // Cleaning Motor Pins
    this->cleanRPin = cleanR;
    this->cleanLPin = cleanL;
    
    // Tilt Motor (Stepper) Pins
    this->pinStep = stepPin;
    this->pinDir = dirPin;
    this->pinEncA = encA;
    this->pinEncB = encB;
    
    this->tiltState = 0; // Default to stopped
}

void MotorDriver::begin() {
    // 1. Setup Status LED
    pinMode(statusLedPin, OUTPUT);
    digitalWrite(statusLedPin, LOW);

    // 2. Setup Cleaning Motor (PWM for IBT-2) - PRESERVED EXACTLY
    ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES);
    ledcSetup(PWM_CH_L, PWM_FREQ, PWM_RES);
    
    ledcAttachPin(cleanRPin, PWM_CH_R);
    ledcAttachPin(cleanLPin, PWM_CH_L);

    // Initialize to 0
    ledcWrite(PWM_CH_R, 0);
    ledcWrite(PWM_CH_L, 0);

    // 3. Setup Tilt Motor (Stepper + Encoder)
    pinMode(pinStep, OUTPUT);
    pinMode(pinDir, OUTPUT);
    
    // Encoder Pins (Input Pullup is safer for open-collector encoders)
    pinMode(pinEncA, INPUT_PULLUP);
    pinMode(pinEncB, INPUT_PULLUP);
    
    // Store the B pin in the global variable so the ISR can read it
    globalEncBPin = pinEncB; 

    // Attach Interrupt to Pin A
    attachInterrupt(digitalPinToInterrupt(pinEncA), isr_updateEncoder, RISING);
}
int MotorDriver::getTiltState() {
    return tiltState;
}
// --- MAIN LOOP UPDATE (New) ---
// You must call motorSystem.update() in your main loop!
void MotorDriver::update() {
    if (tiltState != 0) {
        // 1. Set Direction
        digitalWrite(pinDir, (tiltState == 1) ? HIGH : LOW);

        // 2. Step Fast (Adjust these numbers for speed)
        // 400 microseconds = faster than before
        // 200 microseconds = even faster
        digitalWrite(pinStep, HIGH);
        delayMicroseconds(800); 
        digitalWrite(pinStep, LOW);
        delayMicroseconds(800);
    }
}

// --- VISUAL SIGNALS (Unchanged) ---
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

// --- CLEANING MOTOR LOGIC (Unchanged) ---
void MotorDriver::setCleaningMotor(int direction, int speed) {
    if (direction == 1) { // Forward
        ledcWrite(PWM_CH_L, 0);
        ledcWrite(PWM_CH_R, speed);
        // Serial.println("[MOTOR] Cleaning: FWD");
    } 
    else if (direction == -1) { // Reverse
        ledcWrite(PWM_CH_R, 0);
        ledcWrite(PWM_CH_L, speed);
        // Serial.println("[MOTOR] Cleaning: REV");
    } 
    else { // Stop
        ledcWrite(PWM_CH_R, 0);
        ledcWrite(PWM_CH_L, 0);
        // Serial.println("[MOTOR] Cleaning: STOP");
    }
}

// --- COMMAND FUNCTIONS (Called by WebServer or Auto-Logic) ---
void MotorDriver::setTiltMotor(int direction) {
    // We strictly update the STATE here. No physical movement code.
    tiltState = direction; 
    
    if (direction == 1) {
        digitalWrite(pinDir, HIGH); // Set direction pin once
        Serial.println("[MOTOR] Tilt State: UP");
    }
    else if (direction == -1) {
        digitalWrite(pinDir, LOW);  // Set direction pin once
        Serial.println("[MOTOR] Tilt State: DOWN");
    }
    else {
        Serial.println("[MOTOR] Tilt State: STOP");
    }
}

void MotorDriver::stopAll() {
    setCleaningMotor(0,0);
    setTiltMotor(0);
    Serial.println("[MOTOR] EMERGENCY STOP ALL");
}
// --- ACTUATION LOOP (Called by void loop()) ---
void MotorDriver::tick() {
    // Only step if the state is active
    if (tiltState != 0) {
        digitalWrite(pinStep, HIGH);
        delayMicroseconds(STEP_DELAY); 
        digitalWrite(pinStep, LOW);
        delayMicroseconds(STEP_DELAY);
    }
}

// --- ENCODER FUNCTIONS (New) ---
long MotorDriver::getEncoderPosition() {
    long pos;
    noInterrupts(); // Pause interrupts for atomic read
    pos = encoderPos;
    interrupts();
    return pos;
}

void MotorDriver::resetEncoder() {
    noInterrupts();
    encoderPos = 0;
    interrupts();
    Serial.println("[MOTOR] Encoder Reset to 0");
}

void MotorDriver::initiateCleaningCycle() {
    // Automated demo cycle
    Serial.println(">>> CLEANING CYCLE STARTED <<<");
    //setCleaningMotor(1, 200); // Fwd Speed 200
    delay(2000);
    //setCleaningMotor(0);      // Stop
    delay(500);
    //setCleaningMotor(-1, 200); // Rev Speed 200
    delay(2000);
    //setCleaningMotor(0);       // Stop
    Serial.println(">>> CLEANING CYCLE COMPLETE <<<");
}