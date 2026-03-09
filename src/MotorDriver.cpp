#include "MotorDriver.h"
#include "driver/pcnt.h"

// --- TILT ENCODER uses PCNT_UNIT_0 only ---
// Wiper encoder has been removed — position is determined by limit switches.
#define ENCODER_PCNT_UNIT PCNT_UNIT_0

// ======================================================================
// CONSTRUCTOR
// ======================================================================
MotorDriver::MotorDriver(int cleanR, int cleanL,
                         int enaPin, int stepPin, int dirPin,
                         int encA, int encB, int limitPin,
                         int limitBottomPin, int limitTopPin) {
  this->cleanRPin      = cleanR;
  this->cleanLPin      = cleanL;
  this->pinEna         = enaPin;
  this->pinStep        = stepPin;
  this->pinDir         = dirPin;
  this->pinEncA        = encA;
  this->pinEncB        = encB;
  this->pinLimitSwitch = limitPin;
  this->pinLimitBottom = limitBottomPin;
  this->pinLimitTop    = limitTopPin;

  // Tilt state
  this->limitTriggered  = false;
  this->limitDebounce   = 0;
  this->tiltState       = 0;
  this->targetPos       = 0;
  this->isPositioning   = false;

  // Wiper state
  this->bottomLimitTriggered = false;
  this->topLimitTriggered    = false;
  this->bottomDebounce       = 0;
  this->topDebounce          = 0;
  this->cleanCycleState      = CLEAN_IDLE;
  this->wiperPhaseStartMs    = 0;
  this->wiperReliefStartMs   = 0;
}

// ======================================================================
// BEGIN
// ======================================================================
void MotorDriver::begin() {
  // ----------------------------------------------------------------
  // 1. Cleaning Motor PWM (IBT-2) — unchanged
  // ----------------------------------------------------------------
  ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_L, PWM_FREQ, PWM_RES);
  ledcAttachPin(cleanRPin, PWM_CH_R);
  ledcAttachPin(cleanLPin, PWM_CH_L);
  ledcWrite(PWM_CH_R, 0);
  ledcWrite(PWM_CH_L, 0);

  // ----------------------------------------------------------------
  // 2. Tilt Stepper + Encoder — unchanged
  // ----------------------------------------------------------------
  pinMode(pinEna, OUTPUT);
  digitalWrite(pinEna, HIGH); // Start DISABLED
  Serial.println("[MOTOR] Stepper driver disabled (power saving)");

  pinMode(pinStep, OUTPUT);
  pinMode(pinDir,  OUTPUT);

  pinMode(pinEncA, INPUT_PULLUP);
  pinMode(pinEncB, INPUT_PULLUP);

  // Tilt encoder — PCNT_UNIT_0, Channel 0
  pcnt_config_t pcnt_config_a = {};
  pcnt_config_a.pulse_gpio_num = pinEncA;
  pcnt_config_a.ctrl_gpio_num  = pinEncB;
  pcnt_config_a.unit           = ENCODER_PCNT_UNIT;
  pcnt_config_a.channel        = PCNT_CHANNEL_0;
  pcnt_config_a.pos_mode       = PCNT_COUNT_DEC;
  pcnt_config_a.neg_mode       = PCNT_COUNT_INC;
  pcnt_config_a.lctrl_mode     = PCNT_MODE_REVERSE;
  pcnt_config_a.hctrl_mode     = PCNT_MODE_KEEP;
  pcnt_config_a.counter_h_lim  = 32767;
  pcnt_config_a.counter_l_lim  = -32768;
  pcnt_unit_config(&pcnt_config_a);

  // Tilt encoder — PCNT_UNIT_0, Channel 1
  pcnt_config_t pcnt_config_b = {};
  pcnt_config_b.pulse_gpio_num = pinEncB;
  pcnt_config_b.ctrl_gpio_num  = pinEncA;
  pcnt_config_b.unit           = ENCODER_PCNT_UNIT;
  pcnt_config_b.channel        = PCNT_CHANNEL_1;
  pcnt_config_b.pos_mode       = PCNT_COUNT_INC;
  pcnt_config_b.neg_mode       = PCNT_COUNT_DEC;
  pcnt_config_b.lctrl_mode     = PCNT_MODE_REVERSE;
  pcnt_config_b.hctrl_mode     = PCNT_MODE_KEEP;
  pcnt_config_b.counter_h_lim  = 32767;
  pcnt_config_b.counter_l_lim  = -32768;
  pcnt_unit_config(&pcnt_config_b);

  pcnt_set_filter_value(ENCODER_PCNT_UNIT, 25);
  pcnt_filter_enable(ENCODER_PCNT_UNIT);
  pcnt_counter_pause(ENCODER_PCNT_UNIT);
  pcnt_counter_clear(ENCODER_PCNT_UNIT);
  pcnt_counter_resume(ENCODER_PCNT_UNIT);
  Serial.println("[ENCODER] Tilt PCNT active (PCNT_UNIT_0, 4x Quadrature)");

  // ----------------------------------------------------------------
  // 3. Tilt limit switch — unchanged
  // ----------------------------------------------------------------
  pinMode(pinLimitSwitch, INPUT_PULLUP);
  delay(10);
  limitTriggered = (digitalRead(pinLimitSwitch) == LOW);
  Serial.println("[MOTOR] Tilt limit switch on GPIO " + String(pinLimitSwitch) +
                 (limitTriggered ? " (pressed)" : " (open)"));

  // ----------------------------------------------------------------
  // 4. Wiper limit switches (read-only — no encoder needed)
  // ----------------------------------------------------------------
  pinMode(pinLimitBottom, INPUT_PULLUP);
  pinMode(pinLimitTop,    INPUT_PULLUP);
  delay(10);

  bool atTop    = (digitalRead(pinLimitTop)    == LOW);
  bool atBottom = (digitalRead(pinLimitBottom) == LOW);

  Serial.println("[WIPER] Bottom limit switch on GPIO " + String(pinLimitBottom) +
                 (atBottom ? " (pressed)" : " (open)"));
  Serial.println("[WIPER] Top    limit switch on GPIO " + String(pinLimitTop) +
                 (atTop    ? " (pressed)" : " (open)"));

  if (atTop) {
    Serial.println("[WIPER] Boot: wiper confirmed at rest (top). Ready.");
  } else if (atBottom) {
    Serial.println("[WIPER] Boot: wiper at bottom — driving UP to rest position.");
    wiperEnterState(CLEAN_GOING_UP);
    setCleaningMotor(1, 255); // UP
  } else {
    Serial.println("[WIPER] Boot: wiper position unknown — driving UP to rest position.");
    wiperEnterState(CLEAN_GOING_UP);
    setCleaningMotor(1, 255); // UP — will stop when top switch fires
  }
}

// ======================================================================
// TILT MOTOR METHODS — completely unchanged
// ======================================================================

int MotorDriver::getTiltState() { return tiltState; }

void MotorDriver::setCleaningMotor(int direction, int speed) {
  // direction  1 → RPWM (cleanRPin) → wiper UP   toward top    limit
  // direction -1 → LPWM (cleanLPin) → wiper DOWN toward bottom limit
  // direction  0 → stop
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

void MotorDriver::setTiltMotor(int direction) {
  isPositioning = false;
  tiltState     = direction;

  if (direction == 1) {
    digitalWrite(pinEna, LOW);
    digitalWrite(pinDir, LOW);
    Serial.println("[MOTOR] Tilt: UP (manual)");
  } else if (direction == -1) {
    digitalWrite(pinEna, LOW);
    digitalWrite(pinDir, HIGH);
    Serial.println("[MOTOR] Tilt: DOWN (manual)");
  } else {
    digitalWrite(pinEna, HIGH);
    tiltState = 0;
    Serial.printf("[MOTOR] Tilt: STOP at %.2f°\n", getAngleDegrees());
  }
}

void MotorDriver::stopAll() {
  // Stop tilt
  isPositioning = false;
  tiltState     = 0;
  setTiltMotor(0);

  // Stop wiper — abort any running cycle
  cleanCycleState = CLEAN_IDLE;
  setCleaningMotor(0, 0);

  Serial.println("[MOTOR] EMERGENCY STOP ALL");
}

void MotorDriver::moveToAngle(float degrees) {
  targetPos     = (long)(degrees * (ENCODER_CPR * 4) * GEAR_RATIO / 360.0f);
  isPositioning = true;

  long currentPos = getEncoderPosition();
  int  dir        = (targetPos > currentPos) ? 1 : -1;

  digitalWrite(pinEna, LOW);
  digitalWrite(pinDir, (dir == 1) ? LOW : HIGH);
  tiltState = dir;

  Serial.printf("[POSITION] Moving to %.2f° (target: %ld, current: %ld)\n",
                degrees, targetPos, currentPos);
}

void MotorDriver::moveByAngle(float deltaDeg) {
  float current = getAngleDegrees();
  float target  = current + deltaDeg;
  Serial.printf("[POSITION] Nudge %.2f° → target %.2f°\n", deltaDeg, target);
  moveToAngle(target);
}

bool MotorDriver::isMoving() { return isPositioning; }

// ======================================================================
// WIPER CLEAN CYCLE
// ======================================================================

/**
 * @brief Helper to transition the wiper state machine and reset stall timers.
 * @param newState The CleanCycleState to enter.
 */
void MotorDriver::wiperEnterState(CleanCycleState newState) {
  cleanCycleState    = newState;
  wiperPhaseStartMs  = millis();
  wiperReliefStartMs = millis();
}

/**
 * @brief Initiates a full automated wipe down-and-up cycle.
 */
void MotorDriver::initiateFullCleanCycle() {
  if (cleanCycleState != CLEAN_IDLE) {
    Serial.println("[WIPER] Clean cycle already in progress — ignoring.");
    return;
  }
  if (digitalRead(pinLimitTop) != LOW) {
    Serial.println("[WIPER] WARNING: Top limit not active at cycle start. Proceeding anyway.");
  }

  wiperEnterState(CLEAN_GOING_DOWN);
  setCleaningMotor(-1, 255); // DOWN
  Serial.println("[WIPER] >>> CLEAN CYCLE STARTED — Phase 1: Going DOWN <<<");
}

// ======================================================================
// TICK — called from void loop() as fast as possible
// ======================================================================
void MotorDriver::tick() {
  // ----------------------------------------------------------------
  // 0. Poll tilt encoder
  // ----------------------------------------------------------------
  getEncoderPosition();

  // ----------------------------------------------------------------
  // 1. Tilt limit switch debounce — unchanged
  // ----------------------------------------------------------------
  if (digitalRead(pinLimitSwitch) == LOW) {
    limitDebounce++;
    if (limitDebounce >= 5 && !limitTriggered) {
      limitTriggered = true;
      tiltState      = 0;
      isPositioning  = false;
      digitalWrite(pinEna, HIGH);
      resetEncoder();
      Serial.println("[LIMIT] *** TILT HOME — Motor stopped, angle = 0° ***");
    }
  } else {
    limitDebounce  = 0;
    limitTriggered = false;
  }

  // ----------------------------------------------------------------
  // 2. Tilt precision positioning — unchanged
  // ----------------------------------------------------------------
  if (isPositioning) {
    long currentPos = getEncoderPosition();
    long error      = targetPos - currentPos;

    if (abs(error) <= 5) {
      tiltState     = 0;
      isPositioning = false;
      digitalWrite(pinEna, HIGH);
      Serial.printf("[POSITION] Reached target at %.2f° (pos: %ld)\n",
                    getAngleDegrees(), currentPos);
    } else {
      int dir = (error > 0) ? 1 : -1;
      if (tiltState != dir) {
        tiltState = dir;
        digitalWrite(pinDir, (dir == 1) ? LOW : HIGH);
      }
    }
  }

  // ----------------------------------------------------------------
  // 3. Tilt stepper pulse — unchanged
  // ----------------------------------------------------------------
  if (tiltState != 0) {
    digitalWrite(pinStep, HIGH);
    delayMicroseconds(STEP_DELAY);
    digitalWrite(pinStep, LOW);
    delayMicroseconds(STEP_DELAY);
  }

  // ----------------------------------------------------------------
  // 4. Wiper state machine
  // ----------------------------------------------------------------
  unsigned long now = millis();

  switch (cleanCycleState) {

    case CLEAN_IDLE:
      // Nothing to do — motor already off
      break;

    // ----------------------------------------------------------
    // PHASE 1: Travelling DOWN toward bottom limit switch
    // ----------------------------------------------------------
    case CLEAN_GOING_DOWN: {
      // --- Stall detection ---
      if (now - wiperPhaseStartMs > WIPER_STALL_TIMEOUT_MS) {
        setCleaningMotor(0, 0);
        wiperEnterState(CLEAN_STALLED);
        Serial.println("[WIPER] *** STALL DETECTED going DOWN — cycle aborted ***");
        break;
      }

      // --- Bottom limit switch debounce ---
      if (digitalRead(pinLimitBottom) == LOW) {
        bottomDebounce++;
        if (bottomDebounce >= 5 && !bottomLimitTriggered) {
          bottomLimitTriggered = true;
          Serial.println("[WIPER] Bottom limit switch triggered");

          // Stop, then apply brief UP relief pulse
          setCleaningMotor(0, 0);
          delay(10); // tiny settle before reversing
          setCleaningMotor(1, 180); // UP at reduced power — relief only
          wiperEnterState(CLEAN_RELIEF_AT_BOTTOM);
          Serial.println("[WIPER] Relief pulse UP started");
        }
      } else {
        bottomDebounce       = 0;
        bottomLimitTriggered = false;
      }
      break;
    }

    // ----------------------------------------------------------
    // RELIEF PULSE after bottom switch — brief UP burst
    // ----------------------------------------------------------
    case CLEAN_RELIEF_AT_BOTTOM: {
      if (now - wiperReliefStartMs >= WIPER_RELIEF_PULSE_MS) {
        setCleaningMotor(0, 0);
        delay(30); // brief pause before starting upward stroke
        setCleaningMotor(1, 255); // UP full speed
        wiperEnterState(CLEAN_GOING_UP);
        Serial.println("[WIPER] Phase 2: Going UP to rest position");
      }
      break;
    }

    // ----------------------------------------------------------
    // PHASE 2: Travelling UP toward top limit switch (rest)
    // ----------------------------------------------------------
    case CLEAN_GOING_UP: {
      // --- Stall detection ---
      if (now - wiperPhaseStartMs > WIPER_STALL_TIMEOUT_MS) {
        setCleaningMotor(0, 0);
        wiperEnterState(CLEAN_STALLED);
        Serial.println("[WIPER] *** STALL DETECTED going UP — cycle aborted ***");
        break;
      }

      // --- Top limit switch debounce ---
      if (digitalRead(pinLimitTop) == LOW) {
        topDebounce++;
        if (topDebounce >= 5 && !topLimitTriggered) {
          topLimitTriggered = true;
          Serial.println("[WIPER] Top limit switch triggered");

          // Stop, then apply brief DOWN relief pulse
          setCleaningMotor(0, 0);
          delay(10);
          setCleaningMotor(-1, 180); // DOWN at reduced power — relief only
          wiperEnterState(CLEAN_RELIEF_AT_TOP);
          Serial.println("[WIPER] Relief pulse DOWN started");
        }
      } else {
        topDebounce       = 0;
        topLimitTriggered = false;
      }
      break;
    }

    // ----------------------------------------------------------
    // RELIEF PULSE after top switch — brief DOWN burst, then idle
    // ----------------------------------------------------------
    case CLEAN_RELIEF_AT_TOP: {
      if (now - wiperReliefStartMs >= WIPER_RELIEF_PULSE_MS) {
        setCleaningMotor(0, 0);
        wiperEnterState(CLEAN_IDLE);
        // Clear debounce flags so switches are fresh for next cycle
        bottomLimitTriggered = false;
        topLimitTriggered    = false;
        bottomDebounce       = 0;
        topDebounce          = 0;
        Serial.println("[WIPER] >>> CLEAN CYCLE COMPLETE. Wiper resting at top. <<<");
      }
      break;
    }

    // ----------------------------------------------------------
    // STALLED — motor is off, waiting for manual intervention
    // ----------------------------------------------------------
    case CLEAN_STALLED:
      // Motor is already off. The app's Emergency Stop or a new
      // cycle request (after clearing the obstruction) will reset this.
      // stopAll() sets cleanCycleState = CLEAN_IDLE.
      break;
  }
}

// ======================================================================
// TILT ENCODER — completely unchanged
// ======================================================================
double       cheated_pcnt_accum = 0.0;
int16_t      last_pcnt          = 0;
portMUX_TYPE encoder_mux        = portMUX_INITIALIZER_UNLOCKED;

long MotorDriver::getEncoderPosition() {
  int16_t count;
  pcnt_get_counter_value(ENCODER_PCNT_UNIT, &count);

  portENTER_CRITICAL_ISR(&encoder_mux);
  int16_t diff = count - last_pcnt;
  if (diff > 16383)       diff -= 32768;
  else if (diff < -16384) diff += 32768;

  double true_diff = (double)(-diff);

  if (true_diff > 0) {
    true_diff *= (90.0 / 86.0);
  } else if (true_diff < 0) {
    true_diff *= (90.0 / 64.0);
  }

  cheated_pcnt_accum += true_diff;
  last_pcnt           = count;
  long current_pos    = (long)cheated_pcnt_accum;
  portEXIT_CRITICAL_ISR(&encoder_mux);

  return current_pos;
}

void MotorDriver::resetEncoder() {
  portENTER_CRITICAL_ISR(&encoder_mux);
  pcnt_counter_pause(ENCODER_PCNT_UNIT);
  pcnt_counter_clear(ENCODER_PCNT_UNIT);
  cheated_pcnt_accum = 0.0;
  last_pcnt          = 0;
  pcnt_counter_resume(ENCODER_PCNT_UNIT);
  portEXIT_CRITICAL_ISR(&encoder_mux);
  Serial.println("[ENCODER] Tilt encoder reset to 0°");
}

float MotorDriver::getAngleDegrees() {
  long pos = getEncoderPosition();
  return (pos * 360.0f) / ((ENCODER_CPR * 4) * GEAR_RATIO);
}

bool MotorDriver::isLimitTriggered() { return limitTriggered; }