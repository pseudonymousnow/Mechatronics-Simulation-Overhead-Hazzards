#pragma once

/*
  WinchControl.h

  This library owns the reusable winch subsystem behavior:
  - hardware setup for the winch motor, pawl servo, and home switch
  - encoder-based winch position tracking
  - raw and filtered speed estimation
  - the internal finite state machine (FSM) for raise, lower, jog, and homing

  Test sketches should stay focused on:
  - creating the hardware objects
  - providing the configuration values through the helper setters below
  - requesting actions
  - printing status / handling serial commands
*/

#pragma region Includes

#include <Arduino.h>
#include <Servo.h>

#include "DualG2HighPowerMotorShield.h"

#pragma endregion

#pragma region Config_Structs

/*
  Store pointers to the hardware objects that are created by the sketch.

  The library does not create these objects itself because Arduino hardware
  drivers are usually constructed as globals in the top-level sketch.
*/
struct WinchHardwareBindings {
  DualG2HighPowerMotorShield24v14 *motorShield;
  Servo *pawlServo;
};

/*
  Store the winch-specific pin assignments and motor orientation.

  flipMotorDirection should match the existing Pololu shield setup where a
  positive M2 command means "raise".
*/
struct WinchHardwareConfig {
  uint8_t pawlServoPin;
  uint8_t homeSwitchPin;
  bool flipMotorDirection;
};

/*
  Pawl servo positions and timing values.

  lockPosition and openPosition are written directly to the Arduino Servo
  object, so they should be standard servo angles in degrees.
*/
struct WinchPawlConfig {
  uint8_t lockPosition;
  uint8_t openPosition;
  uint32_t lockDelayMs;
  uint32_t unlockDelayMs;
};

/*
  Encoder and drum calibration values for the winch.

  The encoder counts are converted into inches of cable travel using the drum
  radius. The mux/sensor fields describe how to reach the AS5600 on the I2C
  bus.
*/
struct WinchEncoderConfig {
  uint8_t muxChannel;
  float drumRadiusIn;
  int countsPerRevolution;
  int wrapThreshold;
  int noiseThreshold;
  uint8_t muxAddress;
  uint8_t sensorAddress;
  uint8_t rawAngleRegister;
};

/*
  Motor command and jog behavior settings.

  These values were previously hard-coded in winch_test.cpp. They now live in
  setup code so a test sketch can tune them without editing the library.
*/
struct WinchMotionConfig {
  int raiseSpeedCmd;
  int lowerSpeedCmd;
  int homingSlowSpeedCmd;
  int maxMotorCommand;
  int jogUpMinCommand;
  float defaultLowerTargetIn;
  float jogUpTargetSpeedInPerS;
  float jogUpReliefDistanceIn;
  float jogUpMaxTravelIn;
  float jogUpCmdRatePerSec;
  float jogUpKpSpeed;
  uint32_t jogUpReliefDebounceMs;
};

/*
  Filter settings for the exponential moving average (EMA) speed estimate.

  minSamplePeriodMs limits how often the filtered speed updates.
  timeConstantSeconds controls how aggressively the filter smooths the speed.
*/
struct WinchSpeedFilterConfig {
  uint32_t minSamplePeriodMs;
  float timeConstantSeconds;
};

/*
  Homing-specific travel limits.

  slowdownThresholdIn is an absolute winch position at which the homing move
  transitions from the normal raise speed to the slower approach speed.
*/
struct WinchHomingConfig {
  float backoffDistanceIn;
  float maxSearchTravelIn;
  float slowdownThresholdIn;
  float maxOvershootIn;
};

#pragma endregion

#pragma region Action_Enums

/*
  Public winch actions that external code can request.
*/
enum WinchActionType : uint8_t {
  WINCH_ACTION_RAISE = 0,
  WINCH_ACTION_LOWER,
  WINCH_ACTION_HOMING,
  WINCH_ACTION_MANUAL_LOWER
};

#pragma endregion

#pragma region Internal_State_Enums

/*
  Internal sub-steps used by the non-blocking winch FSM.

  External code can read these for debugging, but it should not need to manage
  them directly.
*/
enum WinchStep : uint8_t {
  WINCH_STEP_IDLE = 0,
  WINCH_STEP_JOG_UP,
  WINCH_STEP_UNLOCK,
  WINCH_STEP_MOVING,
  WINCH_STEP_HOMING_SLOW,
  WINCH_STEP_HOME_SWITCH_HIT,
  WINCH_STEP_BRAKE_AND_LOCK
};

#pragma endregion

#pragma region Status_Structs

/*
  Snapshot of the most recent winch state.

  This lets the sketch print or inspect the subsystem without needing access to
  the library's internal file-scope variables.
*/
struct WinchStatus {
  WinchActionType requestedAction;
  WinchStep currentStep;
  float positionIn;
  float speedInPerS;
  float filteredSpeedInPerS;
  float homePositionIn;
  float targetDepthIn;
  float jogStartPositionIn;
  bool actionComplete;
  bool actionActive;
  bool homeSwitchPressed;
  bool motorFaultActive;
  bool encoderReadSucceeded;
  int appliedMotorCommand;
};

#pragma endregion

#pragma region Public_Function_Declarations

/*
  Supply the hardware objects that the library should operate on.
*/
void setWinchHardwareBindings(DualG2HighPowerMotorShield24v14 *motorShield,
                              Servo *pawlServo);

/*
  Store the pin assignments and motor direction settings used by begin.
*/
void setWinchHardwarePins(uint8_t pawlServoPin,
                          uint8_t homeSwitchPin,
                          bool flipMotorDirection);

/*
  Store the pawl servo angles and the lock/unlock delays.
*/
void setWinchPawlParameters(uint8_t lockPosition,
                            uint8_t openPosition,
                            uint32_t lockDelayMs,
                            uint32_t unlockDelayMs);

/*
  Store all encoder and drum conversion parameters in one place.
*/
void setWinchEncoderParameters(uint8_t muxChannel,
                               float drumRadiusIn,
                               bool invertDirection,
                               int countsPerRevolution,
                               int wrapThreshold,
                               int noiseThreshold,
                               uint8_t muxAddress,
                               uint8_t sensorAddress,
                               uint8_t rawAngleRegister);

/*
  Store the raise/lower motor commands and jog-unlock tuning values.
*/
void setWinchMotionParameters(int raiseSpeedCmd,
                              int lowerSpeedCmd,
                              int homingSlowSpeedCmd,
                              int maxMotorCommand,
                              int jogUpMinCommand,
                              float defaultLowerTargetIn,
                              float jogUpTargetSpeedInPerS,
                              float jogUpReliefDistanceIn,
                              float jogUpMaxTravelIn,
                              float jogUpCmdRatePerSec,
                              float jogUpKpSpeed,
                              uint32_t jogUpReliefDebounceMs);

/*
  Store the EMA filter parameters used for the filtered speed estimate.
*/
void setWinchSpeedFilterParameters(uint32_t minSamplePeriodMs,
                                   float timeConstantSeconds);

/*
  Store the travel limits used by the homing routine.
*/
void setWinchHomingParameters(float backoffDistanceIn,
                              float maxSearchTravelIn,
                              float slowdownThresholdIn,
                              float maxOvershootIn);

/*
  Update only the default lower target.

  This is helpful when a test sketch wants to keep all other motion parameters
  unchanged while trying different lowering depths.
*/
void setWinchDefaultLowerTarget(float targetDepthIn);

/*
  Initialize the winch hardware and zero the current encoder position.

  initialZeroPositionIn is the physical position value that should be reported
  immediately after startup.
*/
bool beginWinchControl(float initialZeroPositionIn = 0.0f);

/*
  Refresh sensors, check for motor faults, and advance the internal FSM.

  Call this once per loop so the winch subsystem can continue progressing.
*/
void updateWinchControl();

/*
  Reset the continuous encoder tracking so the current position becomes a known
  reference value.
*/
bool resetWinchEncoderToCurrentPosition(float newZeroInches = 0.0f);

/*
  Start one of the public winch actions.

  Lowering uses the currently stored default lower target. Use
  startWinchLowerToDepth() when a different target is needed for one move.
*/
bool startWinchAction(WinchActionType action);

/*
  Start a lower-to-depth action using a one-off target value.
*/
bool startWinchLowerToDepth(float targetDepthIn);

/*
  Start the same jog-up/unlock/lower sequence that the legacy test sketch used
  for the 'j' serial command.
*/
bool startJogUpForUnlock();

/*
  Stop the winch immediately, optionally re-locking the pawl first.
*/
void finishWinchAction(bool shouldLock = true);

// Return whether the most recent requested action has finished.
bool isWinchActionComplete();

// Return true whenever the internal FSM is in any step other than IDLE.
bool isWinchBusy();

// Return whether the Pololu motor shield has reported an M2 fault.
bool hasWinchFault();

// Return the current home-switch state after pull-up inversion.
bool isWinchHomeSwitchPressed();

// Return the latest encoder-based winch position in inches.
float getWinchPosition();

// Return the latest unfiltered speed estimate in inches per second.
float getWinchSpeed();

// Return the latest filtered speed estimate in inches per second.
float getWinchSpeedFiltered();

// Return the most recent target depth used by a lower action.
float getWinchTargetDepth();

// Return the most recent home reference value.
float getWinchHomePosition();

// Return the most recent M2 motor command that the library applied.
int getWinchAppliedMotorCommand();

// Return the full status snapshot for logging or debugging.
WinchStatus getWinchStatus();

// Human-readable names for logging.
const char *getWinchActionName(WinchActionType action);
const char *getWinchStepName(WinchStep step);

#pragma endregion
