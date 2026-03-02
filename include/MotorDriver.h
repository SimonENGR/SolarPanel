#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

class MotorDriver {
private:
  // --- Cleaning Motor (IBT-2 PWM) ---
  int cleanRPin;
  int cleanLPin;

  // --- Tilt Motor (Stepper + Encoder) ---
  int pinEna;
  int pinStep;
  int pinDir;
  int pinEncA;
  int pinEncB;

  // --- Limit Switch ---
  int pinLimitSwitch;
  bool limitTriggered;
  int limitDebounce;

  // State Tracking for Stepper
  // 0 = Idle, 1 = Moving Up, -1 = Moving Down
  volatile int tiltState;

  // --- Precision Positioning ---
  volatile long targetPos;     // Target encoder position for moveToAngle
  volatile bool isPositioning; // True when executing a precision move

  // Tuning for speed (microseconds)
  const int STEP_DELAY = 2500;

  // Encoder Resolution (1x mode: RISING on A only)
  static const int ENCODER_PPR = 3600;
  static const int ENCODER_CPR = ENCODER_PPR; // 3600 counts/rev in 1x mode
  static constexpr float GEAR_RATIO = 10.0f;  // 1:10 worm gear (motor:panel)

  // PWM Settings
  const int PWM_FREQ = 5000;
  const int PWM_RES = 8;
  const int PWM_CH_R = 0;
  const int PWM_CH_L = 1;

public:
  // Constructor
  MotorDriver(int cleanR, int cleanL, int enaPin, int stepPin, int dirPin,
              int encA, int encB, int limitPin);

  // Setup
  void begin();

  // Main Loop: MUST be called repeatedly in void loop()
  void tick();

  // Helper function (Legacy support)
  void update();

  // --- Movement Logic ---
  void setCleaningMotor(int direction, int speed = 255);
  void setTiltMotor(int direction);
  void stopAll();
  int getTiltState();

  // --- Precision Positioning ---
  void moveToAngle(float degrees);  // Absolute: go to this angle
  void moveByAngle(float deltaDeg); // Relative: move by ±degrees
  bool isMoving();                  // True if precision move in progress

  // --- Encoder Logic ---
  long getEncoderPosition();
  void resetEncoder();
  float getAngleDegrees();

  // --- Limit Switch ---
  bool isLimitTriggered();

  // Legacy Automated Cycle
  void initiateCleaningCycle();
};

#endif