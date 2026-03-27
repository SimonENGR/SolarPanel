#pragma once
#include <Arduino.h>

// --- PWM Config (Cleaning Motor IBT-2) ---
#define PWM_CH_R  0
#define PWM_CH_L  1
#define PWM_FREQ  1000
#define PWM_RES   8

// --- Tilt Stepper Config ---
#define STEP_DELAY   1000  // target speed delay (microseconds between step pulses)
#define STEP_DELAY_MAX 4000  // start speed delay (slower, for ramping)
#define ENCODER_CPR  3600  // encoder counts per revolution (before 4x quadrature)
#define GEAR_RATIO   50    // 1:50 worm gear


// --- Wiper Safety Config ---
// Maximum time (ms) allowed for a single stroke before declaring a stall.
// Measure your actual full-stroke time and set this to ~2x that value.
#define WIPER_STALL_TIMEOUT_MS   20000UL

// Duration (ms) the cleaning pump runs after the wiper reaches the bottom.
// TODO: Tune this value once the pump is wired and tested.
#define WIPER_PUMP_DURATION_MS   3000UL

// --- Wiper Clean Cycle State Machine ---
// Wiper rests at BOTTOM (bottom limit switch pressed).
// Switches read HIGH when triggered (NO wiring with gray wire).
// Cycle: DOWN → bottom switch → PUMP → UP → top switch → DOWN to rest → IDLE
enum CleanCycleState {
  CLEAN_IDLE,
  CLEAN_GOING_DOWN,          // Phase 1: driving down toward bottom limit switch
  CLEAN_PUMPING,             // Phase 2: wiper stopped at bottom, pump running
  CLEAN_GOING_UP,            // Phase 3: driving up toward top limit switch
  CLEAN_WAIT_AT_TOP,         // Phase 3.5: 3 second pause at top limit switch
  CLEAN_GOING_DOWN_TO_REST,  // Phase 4: driving back down to rest at bottom
  CLEAN_STALLED              // Stall detected — awaiting manual intervention
};

class MotorDriver {
public:
  // ----------------------------------------------------------------
  // Constructor
  // cleanR       = IBT-2 RPWM → wiper UP   (toward top limit switch)
  // cleanL       = IBT-2 LPWM → wiper DOWN (toward bottom limit switch)
  // limitBottomPin = bottom limit switch, LOW when triggered (end of stroke)
  // limitTopPin    = top    limit switch, LOW when triggered (rest position)
  // No wiper encoder pins — position is determined solely by limit switches
  // ----------------------------------------------------------------
  MotorDriver(int cleanR, int cleanL, int cleanEnR, int cleanEnL,
              int enaPin, int stepPin, int dirPin,
              int encA, int encB, int limitPin,
              int limitBottomPin, int limitTopPin);

  void begin();

  // ----------------------------------------------------------------
  // Tilt Motor — unchanged public interface
  // ----------------------------------------------------------------
  void  setTiltMotor(int direction);
  void  moveToAngle(float degrees);
  void  moveByAngle(float deltaDeg);
  bool  isMoving();
  int   getTiltState();
  void  stopAll();

  // Homing: drive toward limit switch, reset encoder as 0°
  void  homeToZero();           // blocking until limit switch hit
  bool  isHomingComplete();     // has the system been homed at least once?

  long  getEncoderPosition();
  void  resetEncoder();
  float getAngleDegrees();
  bool  isLimitTriggered();

  // ----------------------------------------------------------------
  // Cleaning / Wiper Motor
  // ----------------------------------------------------------------
  void  setCleaningMotor(int direction, int speed); // raw IBT-2 control
  void  initiateFullCleanCycle();                   // start autonomous cycle

  bool  isWiperMoving()    { return cleanCycleState != CLEAN_IDLE && cleanCycleState != CLEAN_STALLED; }
  bool  isWiperStalled()   { return cleanCycleState == CLEAN_STALLED; }
  bool  isTopLimitHit()    { return topLimitTriggered; }
  bool  isBottomLimitHit() { return bottomLimitTriggered; }

  // ----------------------------------------------------------------
  // Main loop driver
  // ----------------------------------------------------------------
  void tick();

private:
  // --- Tilt motor pins ---
  int pinEna, pinStep, pinDir;
  int pinEncA, pinEncB;
  int pinLimitSwitch;

  // --- Cleaning motor pins ---
  int cleanRPin, cleanLPin;
  int cleanEnRPin, cleanEnLPin;

  // --- Wiper limit switch pins (read-only inputs) ---
  int pinLimitBottom;   // bottom of stroke
  int pinLimitTop;      // top of stroke (rest position)

  // --- Tilt state ---
  volatile int  tiltState;
  volatile long targetPos;
  volatile bool isPositioning;
  volatile bool limitTriggered;
  int           limitDebounce;
  volatile bool isHomed;        // has been homed at least once
  volatile bool isHoming;       // currently in homing procedure
  unsigned int  currentStepDelay; // speed control (acceleration profile)

  // --- Wiper state ---
  volatile bool            bottomLimitTriggered;
  volatile bool            topLimitTriggered;
  int                      bottomDebounce;
  int                      topDebounce;
  volatile CleanCycleState cleanCycleState;

  // Stall detection: timestamp when the current stroke phase started
  unsigned long            wiperPhaseStartMs;

  // Relief pulse: timestamp when the relief pulse started
  unsigned long            wiperReliefStartMs;

  // Internal helper: transition to a new state and reset the stall timer
  void wiperEnterState(CleanCycleState newState);
};