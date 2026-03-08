#pragma once
#include <Arduino.h>

// --- PWM Config (Cleaning Motor IBT-2) ---
#define PWM_CH_R  0
#define PWM_CH_L  1
#define PWM_FREQ  1000
#define PWM_RES   8

// --- Tilt Stepper Config ---
#define STEP_DELAY   500   // microseconds between step pulses
#define ENCODER_CPR  3600  // encoder counts per revolution (before 4x quadrature)
#define GEAR_RATIO   10    // 1:10 gearbox

// --- Wiper Safety Config ---
// Maximum time (ms) allowed for a single stroke before declaring a stall.
// Measure your actual full-stroke time and set this to ~2x that value.
// Example: if bottom→top takes ~4 s, set 8000 ms.
#define WIPER_STALL_TIMEOUT_MS   8000UL

// Duration (ms) of the brief reverse relief pulse applied after a limit switch
// fires. Just enough to relieve mechanical pressure — not a full reversal.
#define WIPER_RELIEF_PULSE_MS    80UL

// --- Wiper Clean Cycle State Machine ---
// Wiper rests at TOP (top limit switch pressed).
// Cycle: DOWN until bottom switch → relief pulse UP → UP until top switch →
//        relief pulse DOWN → stop (rest).
enum CleanCycleState {
  CLEAN_IDLE,
  CLEAN_GOING_DOWN,        // Phase 1: driving down toward bottom limit switch
  CLEAN_RELIEF_AT_BOTTOM,  // Brief upward relief pulse after bottom fires
  CLEAN_GOING_UP,          // Phase 2: driving up back to top limit switch
  CLEAN_RELIEF_AT_TOP,     // Brief downward relief pulse after top fires
  CLEAN_STALLED            // Stall detected — awaiting manual intervention
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
  MotorDriver(int cleanR, int cleanL,
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

  long  getEncoderPosition();
  void  resetEncoder();
  float getAngleDegrees();
  bool  isLimitTriggered();

  // ----------------------------------------------------------------
  // Cleaning / Wiper Motor
  // ----------------------------------------------------------------
  void  setCleaningMotor(int direction, int speed); // raw IBT-2 control
  void  initiateFullCleanCycle();                   // start autonomous cycle
  void  initiateCleaningCycle();                    // legacy shim

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

  // --- Wiper limit switch pins (read-only inputs) ---
  int pinLimitBottom;   // bottom of stroke
  int pinLimitTop;      // top of stroke (rest position)

  // --- Tilt state ---
  volatile int  tiltState;
  volatile long targetPos;
  volatile bool isPositioning;
  volatile bool limitTriggered;
  int           limitDebounce;

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