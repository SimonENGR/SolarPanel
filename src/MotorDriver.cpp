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
MotorDriver::MotorDriver(int cleanR, int cleanL, int enaPin, int stepPin,
                         int dirPin, int encA, int encB, int limitPin) {
  // Cleaning Motor Pins
  this->cleanRPin = cleanR;
  this->cleanLPin = cleanL;

  // Tilt Motor (Stepper) Pins
  this->pinEna = enaPin;
  this->pinStep = stepPin;
  this->pinDir = dirPin;
  this->pinEncA = encA;
  this->pinEncB = encB;

  // Limit Switch
  this->pinLimitSwitch = limitPin;
  this->limitTriggered = false;

  this->tiltState = 0; // Default to stopped
}

void MotorDriver::begin() {
  // 1. Setup Cleaning Motor (PWM for IBT-2)
  ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_L, PWM_FREQ, PWM_RES);

  ledcAttachPin(cleanRPin, PWM_CH_R);
  ledcAttachPin(cleanLPin, PWM_CH_L);

  // Initialize to 0
  ledcWrite(PWM_CH_R, 0);
  ledcWrite(PWM_CH_L, 0);

  // 2. Setup Tilt Motor (Stepper + Encoder)
  pinMode(pinEna, OUTPUT);
  digitalWrite(pinEna, HIGH); // Start DISABLED (HIGH = disabled, saves power)
  Serial.println("[MOTOR] Stepper driver disabled (power saving)");

  pinMode(pinStep, OUTPUT);
  pinMode(pinDir, OUTPUT);

  // Encoder Pins (pullup prevents noise when disconnected;
  // encoder's push-pull output overrides pullup when connected)
  pinMode(pinEncA, INPUT_PULLUP);
  pinMode(pinEncB, INPUT_PULLUP);

  // Store the B pin in the global variable so the ISR can read it
  globalEncBPin = pinEncB;

  // Attach Interrupt to Pin A
  attachInterrupt(digitalPinToInterrupt(pinEncA), isr_updateEncoder, RISING);

  // 3. Setup Limit Switch (NO wiring: switch connects pin to GND when
  // triggered)
  pinMode(pinLimitSwitch, INPUT_PULLUP);
  delay(10); // Let pullup stabilize after boot
  // Read initial state so boot-time LOW (GPIO15 strapping pin) doesn't
  // false-trigger
  limitTriggered = (digitalRead(pinLimitSwitch) == LOW);
  Serial.println("[MOTOR] Limit switch initialized on GPIO " +
                 String(pinLimitSwitch) +
                 (limitTriggered ? " (currently pressed)" : " (open)"));
}

int MotorDriver::getTiltState() { return tiltState; }

// --- MAIN LOOP UPDATE ---
void MotorDriver::update() {
  if (tiltState != 0) {
    digitalWrite(pinDir, (tiltState == 1) ? HIGH : LOW);
    digitalWrite(pinStep, HIGH);
    delayMicroseconds(800);
    digitalWrite(pinStep, LOW);
    delayMicroseconds(800);
  }
}

// --- CLEANING MOTOR LOGIC ---
void MotorDriver::setCleaningMotor(int direction, int speed) {
  if (direction == 1) { // Forward
    ledcWrite(PWM_CH_L, 0);
    ledcWrite(PWM_CH_R, speed);
  } else if (direction == -1) { // Reverse
    ledcWrite(PWM_CH_R, 0);
    ledcWrite(PWM_CH_L, speed);
  } else { // Stop
    ledcWrite(PWM_CH_R, 0);
    ledcWrite(PWM_CH_L, 0);
  }
}

// --- COMMAND FUNCTIONS ---
void MotorDriver::setTiltMotor(int direction) {
  tiltState = direction;

  if (direction == 1) {
    digitalWrite(pinEna, LOW); // Enable driver
    digitalWrite(pinDir, HIGH);
    Serial.println("[MOTOR] Tilt State: UP (driver enabled)");
  } else if (direction == -1) {
    digitalWrite(pinEna, LOW); // Enable driver
    digitalWrite(pinDir, LOW);
    Serial.println("[MOTOR] Tilt State: DOWN (driver enabled)");
  } else {
    digitalWrite(pinEna, HIGH); // Disable driver to save power
    Serial.println("[MOTOR] Tilt State: STOP (driver disabled)");
  }
}

void MotorDriver::stopAll() {
  setCleaningMotor(0, 0);
  setTiltMotor(0); // This also disables the driver
  Serial.println("[MOTOR] EMERGENCY STOP ALL");
}

// --- ACTUATION LOOP (Called by void loop()) ---
void MotorDriver::tick() {
  // Check limit switch FIRST (LOW = triggered with INPUT_PULLUP + NO wiring)
  if (digitalRead(pinLimitSwitch) == LOW && !limitTriggered) {
    // Limit switch just triggered!
    limitTriggered = true;
    tiltState = 0;              // Stop motor immediately
    digitalWrite(pinEna, HIGH); // Disable driver
    resetEncoder();             // Set angle to 0°
    Serial.println("[LIMIT] *** HOME POSITION REACHED - Motor stopped, angle "
                   "reset to 0° ***");
  } else if (digitalRead(pinLimitSwitch) == HIGH) {
    // Switch released - clear the flag
    limitTriggered = false;
  }

  // Only step if the state is active
  if (tiltState != 0) {
    digitalWrite(pinStep, HIGH);
    delayMicroseconds(STEP_DELAY);
    digitalWrite(pinStep, LOW);
    delayMicroseconds(STEP_DELAY);
  }
}

// --- ENCODER FUNCTIONS ---
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
  Serial.println("[ENCODER] Reset to 0° (position 0)");
}

float MotorDriver::getAngleDegrees() {
  long pos = getEncoderPosition();
  // 3600 PPR, 1:10 worm gear
  // Motor angle = pos * 360 / 3600 = pos * 0.1°
  // Panel angle = motor angle / 10 = pos * 0.01°
  return (pos * 360.0f) / (ENCODER_PPR * GEAR_RATIO);
}

// --- LIMIT SWITCH ---
bool MotorDriver::isLimitTriggered() { return limitTriggered; }

void MotorDriver::initiateCleaningCycle() {
  Serial.println(">>> CLEANING CYCLE STARTED <<<");
  delay(2000);
  delay(500);
  delay(2000);
  Serial.println(">>> CLEANING CYCLE COMPLETE <<<");
}