#include "MotorDriver.h"
#include "driver/pcnt.h"

// --- HARDWARE PULSE COUNTER (PCNT) ---
// Uses ESP32 dedicated PCNT peripheral for reliable encoder counting.
// 1x mode: counts RISING edges on channel A, B determines direction.
// Hardware glitch filter rejects EMI — no software debounce needed.
#define ENCODER_PCNT_UNIT PCNT_UNIT_0

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

  // Encoder: ESP32 hardware pulse counter (PCNT)
  // Pull-ups for signal integrity when encoder disconnected
  pinMode(pinEncA, INPUT_PULLUP);
  pinMode(pinEncB, INPUT_PULLUP);

  // Channel 0: Count on A, Control on B
  pcnt_config_t pcnt_config_a = {};
  pcnt_config_a.pulse_gpio_num = pinEncA;
  pcnt_config_a.ctrl_gpio_num = pinEncB;
  pcnt_config_a.unit = ENCODER_PCNT_UNIT;
  pcnt_config_a.channel = PCNT_CHANNEL_0;
  pcnt_config_a.pos_mode = PCNT_COUNT_DEC;
  pcnt_config_a.neg_mode = PCNT_COUNT_INC;
  pcnt_config_a.lctrl_mode = PCNT_MODE_REVERSE;
  pcnt_config_a.hctrl_mode = PCNT_MODE_KEEP;
  pcnt_config_a.counter_h_lim = 32767;
  pcnt_config_a.counter_l_lim = -32768;
  pcnt_unit_config(&pcnt_config_a);

  // Channel 1: Count on B, Control on A
  pcnt_config_t pcnt_config_b = {};
  pcnt_config_b.pulse_gpio_num = pinEncB;
  pcnt_config_b.ctrl_gpio_num = pinEncA;
  pcnt_config_b.unit = ENCODER_PCNT_UNIT;
  pcnt_config_b.channel = PCNT_CHANNEL_1;
  pcnt_config_b.pos_mode = PCNT_COUNT_INC;
  pcnt_config_b.neg_mode = PCNT_COUNT_DEC;
  pcnt_config_b.lctrl_mode = PCNT_MODE_REVERSE;
  pcnt_config_b.hctrl_mode = PCNT_MODE_KEEP;
  pcnt_config_b.counter_h_lim = 32767;
  pcnt_config_b.counter_l_lim = -32768;
  pcnt_unit_config(&pcnt_config_b);

  // Hardware glitch filter: 25 APB cycles = 312ns at 80 MHz
  pcnt_set_filter_value(ENCODER_PCNT_UNIT, 25);
  pcnt_filter_enable(ENCODER_PCNT_UNIT);

  pcnt_counter_pause(ENCODER_PCNT_UNIT);
  pcnt_counter_clear(ENCODER_PCNT_UNIT);
  pcnt_counter_resume(ENCODER_PCNT_UNIT);
  Serial.println(
      "[ENCODER] Hardware PCNT 4x Quadrature active (14400 CPR, 312ns filter)");

  // 3. Setup Limit Switch
  pinMode(pinLimitSwitch, INPUT_PULLUP);
  delay(10);
  limitTriggered = (digitalRead(pinLimitSwitch) == LOW);
  Serial.println("[MOTOR] Limit switch on GPIO " + String(pinLimitSwitch) +
                 (limitTriggered ? " (pressed)" : " (open)"));
}

int MotorDriver::getTiltState() { return tiltState; }

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
    tiltState = 0; // IMPORTANT: clear tiltState when setting to 0 so tick()
                   // stops pulsing
    Serial.printf("[MOTOR] Tilt: STOP at %.2f°\n", getAngleDegrees());
  }
}

void MotorDriver::stopAll() {
  isPositioning = false;
  tiltState = 0; // IMPORTANT: clear tiltState
  setCleaningMotor(0, 0);
  setTiltMotor(0);
  Serial.println("[MOTOR] EMERGENCY STOP ALL");
}

// --- PRECISION POSITIONING ---
void MotorDriver::moveToAngle(float degrees) {
  // Convert panel degrees to encoder counts
  // panel_angle = (counts * 360) / (CPR * GEAR_RATIO)
  // counts = panel_angle * CPR * GEAR_RATIO / 360
  targetPos = (long)(degrees * (ENCODER_CPR * 4) * GEAR_RATIO / 360.0f);
  isPositioning = true;

  long currentPos = getEncoderPosition();
  int dir = (targetPos > currentPos) ? 1 : -1;

  digitalWrite(pinEna, LOW); // Enable driver
  digitalWrite(pinDir, (dir == 1) ? LOW : HIGH);
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
  // 0. Poll PCNT to accumulate 16-bit hardware diffs into 32-bit software
  getEncoderPosition();

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

    if (abs(error) <= 5) {
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
      digitalWrite(pinDir, (dir == 1) ? LOW : HIGH);
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

// --- ENCODER FUNCTIONS (Hardware PCNT) ---
double cheated_pcnt_accum = 0.0;
int16_t last_pcnt = 0;
portMUX_TYPE encoder_mux = portMUX_INITIALIZER_UNLOCKED;

long MotorDriver::getEncoderPosition() {
  int16_t count;
  pcnt_get_counter_value(ENCODER_PCNT_UNIT, &count);

  portENTER_CRITICAL_ISR(&encoder_mux);
  // Handle 16-bit hardware overflow
  int16_t diff = count - last_pcnt;
  if (diff > 16383)
    diff -= 32768;
  else if (diff < -16384)
    diff += 32768;

  // The actual hardware wiring counts backwards, so we invert the diff FIRST
  double true_diff = (double)(-diff);

  // Apply Asymmetric Mathematical "Cheat" for mechanical backlash
  if (true_diff > 0) {
    // UP Correction: 86 raw degrees == 90 visual degrees
    true_diff *= (90.0 / 86.0);
  } else if (true_diff < 0) {
    // DOWN Correction: 64 raw degrees == 90 visual degrees
    // (64 raw degrees of motor movement drops the panel 90 degrees)
    true_diff *= (90.0 / 64.0);
  }

  cheated_pcnt_accum += true_diff;
  last_pcnt = count;
  long current_pos = (long)cheated_pcnt_accum;
  portEXIT_CRITICAL_ISR(&encoder_mux);

  return current_pos;
}

void MotorDriver::resetEncoder() {
  portENTER_CRITICAL_ISR(&encoder_mux);
  pcnt_counter_pause(ENCODER_PCNT_UNIT);
  pcnt_counter_clear(ENCODER_PCNT_UNIT);
  cheated_pcnt_accum = 0.0;
  last_pcnt = 0;
  pcnt_counter_resume(ENCODER_PCNT_UNIT);
  portEXIT_CRITICAL_ISR(&encoder_mux);

  Serial.println("[ENCODER] Reset to 0°");
}

float MotorDriver::getAngleDegrees() {
  long pos = getEncoderPosition();
  // 4x Quadrature mode: 14400 CPR, 1:10 gear
  return (pos * 360.0f) / ((ENCODER_CPR * 4) * GEAR_RATIO);
}

bool MotorDriver::isLimitTriggered() { return limitTriggered; }

void MotorDriver::initiateCleaningCycle() {
  Serial.println(">>> CLEANING CYCLE STARTED <<<");
  delay(2000);
  delay(500);
  delay(2000);
  Serial.println(">>> CLEANING CYCLE COMPLETE <<<");
}