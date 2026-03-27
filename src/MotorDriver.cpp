#include "MotorDriver.h"
#include "driver/pcnt.h"

// --- TILT ENCODER uses PCNT_UNIT_0 only ---
// Wiper encoder has been removed — position is determined by limit switches.
#define ENCODER_PCNT_UNIT PCNT_UNIT_0

// ======================================================================
// CONSTRUCTOR
// ======================================================================
MotorDriver::MotorDriver(int cleanR, int cleanL, int cleanEnR, int cleanEnL,
                         int enaPin, int stepPin, int dirPin,
                         int encA, int encB, int limitPin,
                         int limitBottomPin, int limitTopPin) {
  this->cleanRPin      = cleanR;
  this->cleanLPin      = cleanL;
  this->cleanEnRPin    = cleanEnR;
  this->cleanEnLPin    = cleanEnL;
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
  this->isHomed         = false;
  this->isHoming        = false;
  this->currentStepDelay = STEP_DELAY_MAX;

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

  pinMode(cleanEnRPin, OUTPUT);
  pinMode(cleanEnLPin, OUTPUT);
  digitalWrite(cleanEnRPin, LOW);
  digitalWrite(cleanEnLPin, LOW);

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
  limitTriggered = (digitalRead(pinLimitSwitch) == HIGH);
  Serial.println("[MOTOR] Tilt limit switch on GPIO " + String(pinLimitSwitch) +
                 (limitTriggered ? " (pressed/HIGH)" : " (open/LOW)"));

  // ----------------------------------------------------------------
  // 4. Wiper limit switches (read-only — no encoder needed)
  // ----------------------------------------------------------------
  pinMode(pinLimitBottom, INPUT_PULLUP);
  pinMode(pinLimitTop,    INPUT_PULLUP);
  delay(10);

  bool atTop    = (digitalRead(pinLimitTop)    == HIGH);
  bool atBottom = (digitalRead(pinLimitBottom) == HIGH);

  Serial.println("[WIPER] Bottom limit switch on GPIO " + String(pinLimitBottom) +
                 (atBottom ? " (pressed)" : " (open)"));
  Serial.println("[WIPER] Top    limit switch on GPIO " + String(pinLimitTop) +
                 (atTop    ? " (pressed)" : " (open)"));

 if (atBottom) {
    Serial.println("[WIPER] Boot: wiper confirmed at rest (bottom). Ready.");
  } else if (atTop) {
    Serial.println("[WIPER] Boot: wiper at top — waiting for app command.");
  } else {
    Serial.println("[WIPER] Boot: wiper position unknown — waiting for app command.");
  }
  
  // Force IDLE state on boot regardless of limit switch positions
  cleanCycleState = CLEAN_IDLE; 
  setCleaningMotor(0, 0);
}

// ======================================================================
// HOMING PROCEDURE
// ======================================================================

/**
 * @brief BLOCKING homing: drives the motor directly toward the limit switch.
 *        Does NOT depend on tick() or the positioning system.
 *        Steps the motor, reads the switch, and stops when triggered.
 *        Safe to call before isSystemInitialized.
 */
void MotorDriver::homeToZero() {
  Serial.printf("[HOME] Limit switch pin %d initial read: %s\n",
                pinLimitSwitch,
                digitalRead(pinLimitSwitch) == HIGH ? "HIGH (triggered)" : "LOW (open)");

  // If limit switch is already pressed, just reset encoder
  if (digitalRead(pinLimitSwitch) == HIGH) {
    resetEncoder();
    isHomed  = true;
    isHoming = false;
    limitTriggered = true;
    Serial.println("[HOME] Already at limit switch - encoder zeroed.");
    return;
  }

  Serial.println("[HOME] Driving motor toward limit switch (blocking)...");
  isHoming = true;
  isPositioning = false;

  // Enable motor, direction toward limit switch
  // (pinDir LOW = DOWN / toward 0 degrees / toward limit switch)
  digitalWrite(pinEna, LOW);
  digitalWrite(pinDir, LOW);
  tiltState = 0; // Keep at 0 so tick() in loop doesn't double-step us

  unsigned long startMs = millis();
  unsigned long lastPrintMs = 0;
  const unsigned long TIMEOUT_MS = 150000; // 150s — accounts for 1:50 gear ratio
  int debounceCount = 0;
  long steps = 0;

  while (millis() - startMs < TIMEOUT_MS) {
    // Generate one step pulse (slower than normal for reliable detection)
    digitalWrite(pinStep, HIGH);
    delayMicroseconds(STEP_DELAY * 2);  // Slower stepping during homing
    digitalWrite(pinStep, LOW);
    delayMicroseconds(STEP_DELAY * 2);
    steps++;

    // Check limit switch (HIGH = pressed)
    if (digitalRead(pinLimitSwitch) == HIGH) {
      debounceCount++;
      if (debounceCount >= 10) {
        // CONFIRMED: limit switch triggered
        tiltState = 0;
        digitalWrite(pinEna, HIGH);
        resetEncoder();
        isHomed = true;
        isHoming = false;
        limitTriggered = true;
        Serial.printf("[HOME] HOMED after %ld steps - encoder zeroed\n", steps);
        return;
      }
    } else {
      debounceCount = 0;
    }

    // Yield to watchdog every 500 steps (~1s at this speed)
    if (steps % 500 == 0) {
      vTaskDelay(1 / portTICK_PERIOD_MS);
      unsigned long now = millis();
      if (now - lastPrintMs >= 2000) {
        Serial.printf("[HOME] ...steps=%ld, switch=%s, elapsed=%lums\n",
                      steps,
                      digitalRead(pinLimitSwitch) == HIGH ? "HIGH" : "LOW",
                      now - startMs);
        lastPrintMs = now;
      }
    }
  }

  // TIMEOUT: stop motor
  tiltState = 0;
  digitalWrite(pinEna, HIGH);
  isHoming = false;
  Serial.printf("[HOME] TIMEOUT after %ld steps - switch never triggered!\n", steps);
}

bool MotorDriver::isHomingComplete() { return isHomed; }

// ======================================================================
// TILT MOTOR METHODS — completely unchanged
// ======================================================================

int MotorDriver::getTiltState() { return tiltState; }

void MotorDriver::setCleaningMotor(int direction, int speed) {
  // direction  1 → RPWM (cleanRPin) → wiper UP   toward top    limit
  // direction -1 → LPWM (cleanLPin) → wiper DOWN toward bottom limit
  // direction  0 → stop
  if (direction == 1) {
    digitalWrite(cleanEnRPin, HIGH);
    digitalWrite(cleanEnLPin, HIGH);
    ledcWrite(PWM_CH_L, 0);
    ledcWrite(PWM_CH_R, speed);
  } else if (direction == -1) {
    digitalWrite(cleanEnRPin, HIGH);
    digitalWrite(cleanEnLPin, HIGH);
    ledcWrite(PWM_CH_R, 0);
    ledcWrite(PWM_CH_L, speed);
  } else {
    digitalWrite(cleanEnRPin, LOW);
    digitalWrite(cleanEnLPin, LOW);
    ledcWrite(PWM_CH_R, 0);
    ledcWrite(PWM_CH_L, 0);
  }
}

void MotorDriver::setTiltMotor(int direction) {
  isPositioning = false;
  tiltState     = direction;

  if (direction == 1) {
    digitalWrite(pinEna, LOW);
    digitalWrite(pinDir, HIGH); // HIGH = UP
    Serial.println("[MOTOR] Tilt: UP (manual)");
  } else if (direction == -1) {
    digitalWrite(pinEna, LOW);
    digitalWrite(pinDir, LOW);  // LOW = DOWN
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
  // SAFETY: Clamp to -1.5 minimum (Limit switch is at -1.4)
  if (degrees < -1.5f) degrees = -1.5f;

  targetPos     = (long)(degrees * (ENCODER_CPR * 4) * GEAR_RATIO / 360.0f);
  isPositioning = true;

  long currentPos = getEncoderPosition();
  int  dir        = (targetPos > currentPos) ? 1 : -1;

  digitalWrite(pinEna, LOW);
  // dir == 1 (UP) requires pinDir = HIGH. dir == -1 (DOWN) requires pinDir = LOW.
  digitalWrite(pinDir, (dir == 1) ? HIGH : LOW);
  tiltState = dir;

  Serial.printf("[POSITION] Moving to %.2f deg (target: %ld, current: %ld)\n",
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
  if (digitalRead(pinLimitTop) != HIGH) {
    Serial.println("[WIPER] WARNING: Top limit not active at cycle start. Proceeding anyway.");
  }

  // --- NEW SAFETY CHECK ---
  // Check if the wiper is already resting on the bottom limit switch
  if (digitalRead(pinLimitBottom) == HIGH) {
    Serial.println("[WIPER] >>> CLEAN CYCLE STARTED — Already at bottom limit! Skipping Phase 1. <<<");
    
    // Jump straight to the pumping phase. 
    // The tick() function will wait for the pump duration, then automatically start going UP.
    wiperEnterState(CLEAN_PUMPING);
    
  } else {
    // Normal sequence: Wiper is somewhere in the middle or top, drive DOWN first
    Serial.println("[WIPER] >>> CLEAN CYCLE STARTED — Phase 1: Going DOWN <<<");
    wiperEnterState(CLEAN_GOING_DOWN);
    setCleaningMotor(-1, 155); // DOWN
  }
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
  // 1. Tilt limit switch debounce — HIGH = triggered (NC switch config)
  // ----------------------------------------------------------------
  if (digitalRead(pinLimitSwitch) == HIGH) {
    limitDebounce++;
    if (limitDebounce >= 5 && !limitTriggered) {
      limitTriggered = true;
      tiltState      = 0;
      isPositioning  = false;
      isHoming       = false;
      isHomed        = true;
      digitalWrite(pinEna, HIGH);
      resetEncoder();
      Serial.println("[LIMIT] *** TILT HOME — Motor stopped, encoder zeroed, isHomed=true ***");
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
        // dir == 1 (UP) requires pinDir = HIGH. dir == -1 (DOWN) requires pinDir = LOW.
        digitalWrite(pinDir, (dir == 1) ? HIGH : LOW);
      }
    }
  }

  // ----------------------------------------------------------------
  // 3. Tilt stepper pulse
  // ----------------------------------------------------------------
  if (tiltState != 0) {
    digitalWrite(pinStep, HIGH);
    delayMicroseconds(currentStepDelay);
    digitalWrite(pinStep, LOW);
    delayMicroseconds(currentStepDelay);

    // Dynamic speed profiling (Ramping)
    if (isPositioning) {
      long currentPos = getEncoderPosition();
      long error      = targetPos - currentPos;

      if (abs(error) < 800) {
        // Decelerate when close to target
        if (currentStepDelay < STEP_DELAY_MAX) {
          currentStepDelay += 20;
          if (currentStepDelay > STEP_DELAY_MAX) currentStepDelay = STEP_DELAY_MAX;
        }
      } else {
        // Accelerate up to max speed
        if (currentStepDelay > STEP_DELAY) {
          currentStepDelay -= 10;
          if (currentStepDelay < STEP_DELAY) currentStepDelay = STEP_DELAY;
        }
      }
    } else {
      // Manual mode (no target): Just accelerate
      if (currentStepDelay > STEP_DELAY) {
        currentStepDelay -= 10;
        if (currentStepDelay < STEP_DELAY) currentStepDelay = STEP_DELAY;
      }
    }
  } else {
    // Re-arm delay curve when stopped
    currentStepDelay = STEP_DELAY_MAX;
  }

  // ----------------------------------------------------------------
  // 4. Wiper state machine
  //    Sequence: GOING_DOWN → PUMPING → GOING_UP → GOING_DOWN_TO_REST → IDLE
  //    NOTE: Switches read HIGH when triggered (NO wiring with gray wire)
  // ----------------------------------------------------------------
  unsigned long now = millis();

  switch (cleanCycleState) {

    case CLEAN_IDLE:
      // Nothing to do — motor already off
      break;

    // ----------------------------------------------------------
    // PHASE 1: Travel DOWN toward bottom limit switch
    // ----------------------------------------------------------
    case CLEAN_GOING_DOWN: {
      if (now - wiperPhaseStartMs > WIPER_STALL_TIMEOUT_MS) {
        setCleaningMotor(0, 0);
        wiperEnterState(CLEAN_STALLED);
        Serial.println("[WIPER] *** STALL DETECTED going DOWN — cycle aborted ***");
        break;
      }

      if (digitalRead(pinLimitBottom) == HIGH) {
        bottomDebounce++;
        if (bottomDebounce >= 5 && !bottomLimitTriggered) {
          bottomLimitTriggered = true;
          Serial.println("[WIPER] Bottom limit switch triggered");
          setCleaningMotor(0, 0);

          // TODO: Activate cleaning pump here once GPIO is wired.
          //   Example: digitalWrite(PIN_PUMP, HIGH);

          wiperEnterState(CLEAN_PUMPING);
          Serial.println("[WIPER] Phase 2: Pump ON — waiting " + String(WIPER_PUMP_DURATION_MS) + "ms");
        }
      } else {
        bottomDebounce       = 0;
        bottomLimitTriggered = false;
      }
      break;
    }

    // ----------------------------------------------------------
    // PHASE 2: Pump running at bottom — wait for duration then go up
    // ----------------------------------------------------------
    case CLEAN_PUMPING: {
      if (now - wiperPhaseStartMs >= WIPER_PUMP_DURATION_MS) {
        // TODO: Deactivate pump here once GPIO is wired.
        //   Example: digitalWrite(PIN_PUMP, LOW);

        Serial.println("[WIPER] Pump OFF. Phase 3: Going UP");
        setCleaningMotor(1, 255); // UP full speed
        wiperEnterState(CLEAN_GOING_UP);
      }
      break;
    }

    // ----------------------------------------------------------
    // PHASE 3: Travel UP toward top limit switch
    // ----------------------------------------------------------
    case CLEAN_GOING_UP: {
      if (now - wiperPhaseStartMs > WIPER_STALL_TIMEOUT_MS) {
        setCleaningMotor(0, 0);
        wiperEnterState(CLEAN_STALLED);
        Serial.println("[WIPER] *** STALL DETECTED going UP — cycle aborted ***");
        break;
      }

      if (digitalRead(pinLimitTop) == HIGH) {
        topDebounce++;
        if (topDebounce >= 5 && !topLimitTriggered) {
          topLimitTriggered = true;
          Serial.println("[WIPER] Top limit switch triggered");
          setCleaningMotor(0, 0);
          wiperEnterState(CLEAN_WAIT_AT_TOP);
          Serial.println("[WIPER] Phase 3.5: Pausing at top for 3 seconds");
        }
      } else {
        topDebounce       = 0;
        topLimitTriggered = false;
      }
      break;
    }

    // ----------------------------------------------------------
    // PHASE 3.5: Pause at top for 3 seconds
    // ----------------------------------------------------------
    case CLEAN_WAIT_AT_TOP: {
      if (now - wiperPhaseStartMs >= 3000) {
        Serial.println("[WIPER] Pause complete. Phase 4: Going DOWN to rest position (slowly for 3s)");
        setCleaningMotor(-1, 55); // DOWN at reduced speed (55) for the first 3 seconds
        wiperEnterState(CLEAN_GOING_DOWN_TO_REST);
      }
      break;
    }

    // ----------------------------------------------------------
    // PHASE 4: Travel DOWN back to rest position at bottom
    // ----------------------------------------------------------
    case CLEAN_GOING_DOWN_TO_REST: {
      if (now - wiperPhaseStartMs > WIPER_STALL_TIMEOUT_MS) {
        setCleaningMotor(0, 0);
        wiperEnterState(CLEAN_STALLED);
        Serial.println("[WIPER] *** STALL DETECTED going DOWN to rest — cycle aborted ***");
        break;
      }

      // Speed control: 55 for first 3 seconds, then 155
      if (now - wiperPhaseStartMs <= 3000) {
        setCleaningMotor(-1, 55);
      } else {
        setCleaningMotor(-1, 155);
      }

      if (digitalRead(pinLimitBottom) == HIGH) {
        bottomDebounce++;
        if (bottomDebounce >= 5 && !bottomLimitTriggered) {
          bottomLimitTriggered = true;
          setCleaningMotor(0, 0);

          // Reset all debounce flags for the next cycle
          bottomLimitTriggered = false;
          topLimitTriggered    = false;
          bottomDebounce       = 0;
          topDebounce          = 0;

          wiperEnterState(CLEAN_IDLE);
          Serial.println("[WIPER] >>> CLEAN CYCLE COMPLETE. Wiper resting at bottom. <<<");
        }
      } else {
        bottomDebounce       = 0;
        bottomLimitTriggered = false;
      }
      break;
    }

    // ----------------------------------------------------------
    // STALLED — motor is off, waiting for manual intervention
    // ----------------------------------------------------------
    case CLEAN_STALLED:
      // Motor is already off. Emergency Stop or a new cycle request
      // (after clearing the obstruction) will reset via stopAll().
      break;
  }
}

// ======================================================================
// TILT ENCODER
// ======================================================================
// 1 degree = 2000 counts. Limit switch home position is -1.4 degrees.
double       cheated_pcnt_accum = -2800.0;
int16_t      last_pcnt          = 0;
portMUX_TYPE encoder_mux        = portMUX_INITIALIZER_UNLOCKED;

long MotorDriver::getEncoderPosition() {
  int16_t count;
  pcnt_get_counter_value(ENCODER_PCNT_UNIT, &count);

  portENTER_CRITICAL_ISR(&encoder_mux);
  int16_t diff = count - last_pcnt;
  if (diff > 16383)       diff -= 32768;
  else if (diff < -16384) diff += 32768;

  double true_diff = (double)(diff);
  
  // Asymmetric calibration: going UP vs going DOWN
  if (true_diff > 0) {
    // Final tweak: moving to 1.094 balances the very top of the arc to hit 90 dead-on.
    cheated_pcnt_accum += true_diff * 1.084;
  } else {
    // Adjusted DOWN multiplier to perfectly balance the 85° return path (from 89.62 targetting 5.0 flat)
    // New multiplier: 1.362 * (84.62 target / 86.20 actual) = 1.337
    cheated_pcnt_accum += true_diff * 1.345;
  }
  last_pcnt           = count;
  long current_pos    = (long)cheated_pcnt_accum;
  portEXIT_CRITICAL_ISR(&encoder_mux);

  return current_pos;
}

void MotorDriver::resetEncoder() {
  portENTER_CRITICAL_ISR(&encoder_mux);
  pcnt_counter_pause(ENCODER_PCNT_UNIT);
  pcnt_counter_clear(ENCODER_PCNT_UNIT);
  cheated_pcnt_accum = -2800.0; // 1 degree = 2000 counts. -1.4 degrees = -2800 counts
  last_pcnt          = 0;
  pcnt_counter_resume(ENCODER_PCNT_UNIT);
  portEXIT_CRITICAL_ISR(&encoder_mux);
  Serial.println("[ENCODER] Tilt encoder reset to -1.4°");
}

float MotorDriver::getAngleDegrees() {
  long pos = getEncoderPosition();
  return (pos * 360.0f) / ((ENCODER_CPR * 4) * GEAR_RATIO);
}

bool MotorDriver::isLimitTriggered() { return limitTriggered; }