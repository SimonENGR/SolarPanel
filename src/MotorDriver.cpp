#include "MotorDriver.h"

// --- GLOBALS FOR ENCODER INTERRUPT ---
volatile long encoderPos = 0;
int globalEncAPin = 0;
int globalEncBPin = 0;
volatile int *globalTiltState = nullptr;
volatile unsigned long lastEdgeTime = 0; // For debounce

// Single ISR on RISING of pin A, read B for direction (1x mode)
// Motor-gated + time-debounced to reject step pulse EMI
void IRAM_ATTR isr_encoderA() {
  if (globalTiltState == nullptr || *globalTiltState == 0)
    return;

  // Debounce: reject edges within 1ms of last valid edge (step EMI filter)
  unsigned long now = micros();
  if (now - lastEdgeTime < 1000)
    return;
  lastEdgeTime = now;

  // On RISING of A: B LOW = forward, B HIGH = backward
  if (digitalRead(globalEncBPin) == LOW) {
    encoderPos++;
  } else {
    encoderPos--;
  }
}

// --- CONSTRUCTOR ---
MotorDriver::MotorDriver(int cleanR, int cleanL, int enaPin, int stepPin,
                         int dirPin, int encA, int encB, int limitPin) {
  this->cleanRPin = cleanR;
  this->cleanLPin = cleanL;
  this->pinEna = enaPin;
  this->pinStep = stepPin;
  this->pinDir = dirPin;
  this->pinEncA = encA;
  this->pinEncB = encB;
  // --- Limit Switch ---
  this->pinLimitSwitch = limitPin;
  this->limitTriggered = false;
  this->limitDebounce = 0; // Debounce counter for EMI filtering
  this->tiltState = 0;
  this->targetPos = 0;
  this->isPositioning = false;
}

void MotorDriver::begin() {
  // 1. Setup Cleaning Motor (PWM for IBT-2)
  ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_L, PWM_FREQ, PWM_RES);
  ledcAttachPin(cleanRPin, PWM_CH_R);
  ledcAttachPin(cleanLPin, PWM_CH_L);
  ledcWrite(PWM_CH_R, 0);
  ledcWrite(PWM_CH_L, 0);

  // 2. Setup Tilt Motor (Stepper + Encoder)
  pinMode(pinEna, OUTPUT);
  digitalWrite(pinEna, HIGH); // Start DISABLED
  Serial.println("[MOTOR] Stepper driver disabled (power saving)");

  pinMode(pinStep, OUTPUT);
  pinMode(pinDir, OUTPUT);

  // Encoder Pins (pullup prevents noise when disconnected)
  pinMode(pinEncA, INPUT_PULLUP);
  pinMode(pinEncB, INPUT_PULLUP);

  // Store pin numbers for ISR
  globalEncAPin = pinEncA;
  globalEncBPin = pinEncB;
  globalTiltState = &tiltState;

  // 1x mode: single interrupt on RISING of pin A, read B for direction
  attachInterrupt(digitalPinToInterrupt(pinEncA), isr_encoderA, RISING);
  Serial.println("[ENCODER] 1x decoding active (3600 CPR, motor-gated)");

  // 3. Setup Limit Switch
  pinMode(pinLimitSwitch, INPUT_PULLUP);
  delay(10);
  limitTriggered = (digitalRead(pinLimitSwitch) == LOW);
  Serial.println("[MOTOR] Limit switch on GPIO " + String(pinLimitSwitch) +
                 (limitTriggered ? " (pressed)" : " (open)"));
}

int MotorDriver::getTiltState() { return tiltState; }

// --- MAIN LOOP UPDATE (Legacy) ---
void MotorDriver::update() {
  if (tiltState != 0) {
    digitalWrite(pinDir, (tiltState == 1) ? HIGH : LOW);
    digitalWrite(pinStep, HIGH);
    delayMicroseconds(800);
    digitalWrite(pinStep, LOW);
    delayMicroseconds(800);
  }
}

// --- CLEANING MOTOR ---
void MotorDriver::setCleaningMotor(int direction, int speed) {
  if (direction == 1) {
    ledcWrite(PWM_CH_L, 0);
    ledcWrite(PWM_CH_R, speed);
  } else if (direction == -1) {
    ledcWrite(PWM_CH_R, 0);
    ledcWrite(PWM_CH_L, speed);
  } else {
    ledcWrite(PWM_CH_R, 0);
    ledcWrite(PWM_CH_L, 0);
  }
}

// --- TILT MOTOR COMMANDS ---
void MotorDriver::setTiltMotor(int direction) {
  // Cancel any precision move when manual control is used
  isPositioning = false;
  tiltState = direction;

  if (direction == 1) {
    digitalWrite(pinEna, LOW);
    digitalWrite(pinDir, LOW); // LOW = UP (physical direction)
    Serial.println("[MOTOR] Tilt: UP (manual)");
  } else if (direction == -1) {
    digitalWrite(pinEna, LOW);
    digitalWrite(pinDir, HIGH); // HIGH = DOWN (physical direction)
    Serial.println("[MOTOR] Tilt: DOWN (manual)");
  } else {
    digitalWrite(pinEna, HIGH);
    Serial.printf("[MOTOR] Tilt: STOP at %.2f°\n", getAngleDegrees());
  }
}

void MotorDriver::stopAll() {
  isPositioning = false;
  setCleaningMotor(0, 0);
  setTiltMotor(0);
  Serial.println("[MOTOR] EMERGENCY STOP ALL");
}

// --- PRECISION POSITIONING ---
void MotorDriver::moveToAngle(float degrees) {
  // Convert panel degrees to encoder counts
  // panel_angle = (counts * 360) / (CPR * GEAR_RATIO)
  // counts = panel_angle * CPR * GEAR_RATIO / 360
  targetPos = (long)(degrees * ENCODER_CPR * GEAR_RATIO / 360.0f);
  isPositioning = true;

  long currentPos = getEncoderPosition();
  int dir = (targetPos > currentPos) ? 1 : -1;

  digitalWrite(pinEna, LOW); // Enable driver
  digitalWrite(pinDir, (dir == 1) ? HIGH : LOW);
  tiltState = dir;

  Serial.printf("[POSITION] Moving to %.2f° (target pos: %ld, current: %ld)\n",
                degrees, targetPos, currentPos);
}

void MotorDriver::moveByAngle(float deltaDeg) {
  float current = getAngleDegrees();
  float target = current + deltaDeg;
  Serial.printf("[POSITION] Nudge %.2f° → target %.2f°\n", deltaDeg, target);
  moveToAngle(target);
}

bool MotorDriver::isMoving() { return isPositioning; }

// --- ACTUATION LOOP (Called by void loop()) ---
void MotorDriver::tick() {
  // 1. Check limit switch with debouncing (5 consecutive LOW reads = triggered)
  if (digitalRead(pinLimitSwitch) == LOW) {
    limitDebounce++;
    if (limitDebounce >= 5 && !limitTriggered) {
      limitTriggered = true;
      tiltState = 0;
      isPositioning = false;
      digitalWrite(pinEna, HIGH);
      resetEncoder();
      Serial.println("[LIMIT] *** HOME - Motor stopped, angle = 0° ***");
    }
  } else {
    limitDebounce = 0;
    limitTriggered = false;
  }

  // 2. Check precision positioning target
  if (isPositioning) {
    long currentPos = getEncoderPosition();
    long error = targetPos - currentPos;

    if (abs(error) <= 2) {
      // Close enough — stop
      tiltState = 0;
      isPositioning = false;
      digitalWrite(pinEna, HIGH);
      Serial.printf("[POSITION] Reached target at %.2f° (pos: %ld)\n",
                    getAngleDegrees(), currentPos);
      return;
    }

    // Ensure direction is still correct (in case of overshoot)
    int dir = (error > 0) ? 1 : -1;
    if (tiltState != dir) {
      tiltState = dir;
      digitalWrite(pinDir, (dir == 1) ? HIGH : LOW);
    }
  }

  // 3. Step if motor is active
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
  noInterrupts();
  pos = encoderPos;
  interrupts();
  return pos;
}

void MotorDriver::resetEncoder() {
  noInterrupts();
  encoderPos = 0;
  interrupts();
  Serial.println("[ENCODER] Reset to 0°");
}

float MotorDriver::getAngleDegrees() {
  long pos = getEncoderPosition();
  // Quadrature: 14400 CPR, 1:10 gear
  return (pos * 360.0f) / (ENCODER_CPR * GEAR_RATIO);
}

bool MotorDriver::isLimitTriggered() { return limitTriggered; }

void MotorDriver::initiateCleaningCycle() {
  Serial.println(">>> CLEANING CYCLE STARTED <<<");
  delay(2000);
  delay(500);
  delay(2000);
  Serial.println(">>> CLEANING CYCLE COMPLETE <<<");
}