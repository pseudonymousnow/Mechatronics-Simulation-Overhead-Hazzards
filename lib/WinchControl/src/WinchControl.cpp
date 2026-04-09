#include <WinchControl.h>

/*
  WinchControl.cpp

  This file keeps the full reusable winch implementation in one place.
  External code requests actions such as "lower", "raise", or "home".
  The library handles the step-by-step details internally:
  - pawl servo timing
  - encoder conversion to inches
  - speed estimation
  - homing limits
  - motor commands
*/

#pragma region Includes

#include <EncoderMux.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>

#pragma endregion

namespace {

constexpr float kWinchMoveToPositionBandIn = 0.05f;

#pragma region Private_Config_Storage

/*
  All configuration is stored at file scope so the library keeps a simple
  function-based interface like DriveControl.

  The sketch fills these in through the setter helpers during setup.
*/
WinchHardwareBindings winchHardware = {NULL, NULL};
WinchHardwareConfig winchHardwareConfig = {0xFF, 0xFF, true};
WinchPawlConfig winchPawlConfig = {0, 0, 0, 0};
WinchMotionConfig winchMotionConfig = {0, 0, 0, 0, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0};
WinchSpeedFilterConfig winchSpeedFilterConfig = {0, 0.0f};
WinchHomingConfig winchHomingConfig = {0.0f, 0.0f, 0.0f, 0.0f};
float winchDrumRadiusIn = 0.0f;
bool winchEncoderDirectionInverted = false;

EncoderMuxConfig winchEncoderConfig = {};
EncoderMuxState winchEncoderState = {};

#pragma endregion

#pragma region Private_Runtime_State

WinchActionType requestedAction = WINCH_ACTION_LOWER;
WinchStep currentStep = WINCH_STEP_IDLE;

float winchPositionIn = 0.0f;
float winchHomePositionIn = 0.0f;
float winchSpeedInPerS = 0.0f;
float winchFilteredSpeedInPerS = 0.0f;
float winchPreviousPositionIn = 0.0f;
float winchFilterPreviousPositionIn = 0.0f;
float targetDepthIn = 0.0f;
float targetPositionIn = 0.0f;

float jogStartPositionIn = 0.0f;
float homingStartPositionIn = 0.0f;
float homeSwitchTripPositionIn = 0.0f;
float homeSlowdownPositionIn = 0.0f;

uint32_t actionTimestampMs = 0;
uint32_t lastSpeedSampleMs = 0;
uint32_t lastFilterSampleMs = 0;
uint32_t jogReliefDetectedMs = 0;
uint32_t lastJogControllerMs = 0;

bool winchActionComplete = false;
bool winchMotorFaultActive = false;
bool winchEncoderReadSucceeded = false;
bool winchSpeedFilterInitialized = false;
bool targetPositionActive = false;

int appliedMotorCommand = 0;

#pragma endregion

#pragma region Private_Helper_Functions

bool isWinchConfigured() {
  return winchHardware.motorShield != NULL &&
         winchHardware.pawlServo != NULL &&
         winchHardwareConfig.pawlServoPin != 0xFF &&
         winchHardwareConfig.homeSwitchPin != 0xFF &&
         winchEncoderConfig.countsPerRevolution > 0 &&
         fabsf(winchDrumRadiusIn) > 0.0f;
}

float getWinchDrumCircumferenceIn() {
  return 2.0f * PI * fabsf(winchDrumRadiusIn);
}

float convertWinchCountsToInches(long encoderCounts) {
  const float circumferenceIn = getWinchDrumCircumferenceIn();

  if (winchEncoderConfig.countsPerRevolution <= 0 || circumferenceIn <= 0.0f) {
    return 0.0f;
  }

  const float signedEncoderCounts = winchEncoderDirectionInverted
                                        ? -(float)encoderCounts
                                        : (float)encoderCounts;
  const float shaftRevolutions =
      signedEncoderCounts / (float)winchEncoderConfig.countsPerRevolution;
  return shaftRevolutions * circumferenceIn;
}

long convertWinchInchesToCounts(float positionIn) {
  const float circumferenceIn = getWinchDrumCircumferenceIn();

  if (winchEncoderConfig.countsPerRevolution <= 0 || circumferenceIn <= 0.0f) {
    return 0;
  }

  const float signedPositionIn =
      winchEncoderDirectionInverted ? -positionIn : positionIn;

  return lroundf((signedPositionIn / circumferenceIn) *
                 (float)winchEncoderConfig.countsPerRevolution);
}

void resetSpeedEstimatorState(float seedPositionIn) {
  winchPositionIn = seedPositionIn;
  winchPreviousPositionIn = seedPositionIn;
  winchFilterPreviousPositionIn = seedPositionIn;
  winchSpeedInPerS = 0.0f;
  winchFilteredSpeedInPerS = 0.0f;
  lastSpeedSampleMs = millis();
  lastFilterSampleMs = 0;
  winchSpeedFilterInitialized = false;
}

void writePawlServo(uint8_t servoPosition) {
  if (winchHardware.pawlServo != NULL) {
    winchHardware.pawlServo->write(servoPosition);
  }
}

void clearTargetPositionStop() {
  targetPositionIn = winchPositionIn;
  targetPositionActive = false;
}

bool readHomeSwitchPressedInternal() {
  if (winchHardwareConfig.homeSwitchPin == 0xFF) {
    return false;
  }

  return digitalRead(winchHardwareConfig.homeSwitchPin) == LOW;
}

int clampMotorCommand(int motorCommand) {
  return constrain(motorCommand,
                   -winchMotionConfig.maxMotorCommand,
                   winchMotionConfig.maxMotorCommand);
}

void applyMotorCommand(int motorCommand) {
  appliedMotorCommand = clampMotorCommand(motorCommand);

  if (winchHardware.motorShield != NULL) {
    winchHardware.motorShield->setM2Speed(appliedMotorCommand);
  }
}

float clampFloat(float value, float low, float high) {
  return (value < low) ? low : ((value > high) ? high : value);
}

int rampToSpeedCommand(float desiredSpeedInPerS,
                       float measuredSpeedInPerS,
                       float dtSeconds,
                       float commandRatePerSecond,
                       float proportionalGain) {
  /*
    This is the same ramp-style speed helper that existed in winch_test.cpp.
    It aims for a desired speed using a small proportional term while also
    limiting how quickly the motor command can change between loop iterations.
  */
  static float commandOut = 0.0f;

  const float speedError = desiredSpeedInPerS - measuredSpeedInPerS;
  float commandTarget = proportionalGain * speedError;
  commandTarget =
      clampFloat(commandTarget,
                 -(float)winchMotionConfig.maxMotorCommand,
                 (float)winchMotionConfig.maxMotorCommand);

  const float maxCommandStep = commandRatePerSecond * dtSeconds;
  float deltaCommand = commandTarget - commandOut;
  deltaCommand = clampFloat(deltaCommand, -maxCommandStep, maxCommandStep);
  commandOut += deltaCommand;

  if (fabsf(commandTarget) < 2.0f && fabsf(commandOut) < 2.0f) {
    commandOut = 0.0f;
  }

  return (int)commandOut;
}

void resetRampToSpeedState() {
  /*
    The ramp helper stores its output in a function-static variable.
    Calling it with zero demand and a large dt is a simple way to return that
    state to zero whenever a new action starts or the winch is stopped.
  */
  (void)rampToSpeedCommand(0.0f,
                           0.0f,
                           1.0f,
                           (winchMotionConfig.jogUpCmdRatePerSec > 0.0f)
                               ? winchMotionConfig.jogUpCmdRatePerSec
                               : 1.0f,
                           0.0f);
}

void updateWinchMeasurements() {
  const long encoderCounts = updateEncoderCounts(winchEncoderState, winchEncoderConfig);
  winchEncoderReadSucceeded = winchEncoderState.lastReadSucceeded;

  winchPositionIn = convertWinchCountsToInches(encoderCounts);

  const uint32_t nowMs = millis();

  if (lastSpeedSampleMs == 0) {
    lastSpeedSampleMs = nowMs;
    winchPreviousPositionIn = winchPositionIn;
    winchSpeedInPerS = 0.0f;
  } else {
    const uint32_t dtMs = nowMs - lastSpeedSampleMs;
    if (dtMs > 0U) {
      const float dtSeconds = ((float)dtMs) / 1000.0f;
      winchSpeedInPerS = (winchPositionIn - winchPreviousPositionIn) / dtSeconds;
      lastSpeedSampleMs = nowMs;
      winchPreviousPositionIn = winchPositionIn;
    }
  }

  if (!winchSpeedFilterInitialized) {
    lastFilterSampleMs = nowMs;
    winchFilterPreviousPositionIn = winchPositionIn;
    winchFilteredSpeedInPerS = 0.0f;
    winchSpeedFilterInitialized = true;
    return;
  }

  const uint32_t filterDtMs = nowMs - lastFilterSampleMs;
  if (filterDtMs < winchSpeedFilterConfig.minSamplePeriodMs) {
    return;
  }

  const float filterDtSeconds = ((float)filterDtMs) / 1000.0f;
  const float rawSpeedInPerS =
      (winchPositionIn - winchFilterPreviousPositionIn) / filterDtSeconds;

  lastFilterSampleMs = nowMs;
  winchFilterPreviousPositionIn = winchPositionIn;

  const float tauSeconds = winchSpeedFilterConfig.timeConstantSeconds;
  if (tauSeconds <= 0.0f) {
    winchFilteredSpeedInPerS = rawSpeedInPerS;
    return;
  }

  const float alpha = filterDtSeconds / (tauSeconds + filterDtSeconds);
  winchFilteredSpeedInPerS +=
      alpha * (rawSpeedInPerS - winchFilteredSpeedInPerS);
}

void startConfiguredAction(WinchActionType action,
                          bool useTargetPosition = false,
                          float requestedTargetPositionIn = 0.0f) {
  requestedAction = action;
  winchActionComplete = false;
  winchMotorFaultActive = false;
  targetPositionIn = requestedTargetPositionIn;
  targetPositionActive = useTargetPosition;

  actionTimestampMs = millis();
  jogStartPositionIn = winchPositionIn;
  homingStartPositionIn = winchPositionIn;
  homeSwitchTripPositionIn = winchPositionIn;
  homeSlowdownPositionIn = winchHomingConfig.slowdownThresholdIn;
  jogReliefDetectedMs = 0;
  lastJogControllerMs = millis();

  resetRampToSpeedState();

  if (action == WINCH_ACTION_LOWER ||
      action == WINCH_ACTION_MANUAL_LOWER) {
    // Both lowering modes should relieve pawl load before opening it.
    currentStep = WINCH_STEP_JOG_UP;
  } else {
    currentStep = WINCH_STEP_MOVING;
  }
}

void completeActionImmediately() {
  clearTargetPositionStop();
  currentStep = WINCH_STEP_IDLE;
  winchActionComplete = true;
}

void runWinchStateMachine() {
  switch (currentStep) {
    case WINCH_STEP_IDLE:
      applyMotorCommand(0);
      break;

    case WINCH_STEP_JOG_UP: {
      const uint32_t nowMs = millis();
      float dtSeconds = ((float)(nowMs - lastJogControllerMs)) / 1000.0f;
      if (dtSeconds <= 0.0f) {
        dtSeconds = 0.001f;
      }
      lastJogControllerMs = nowMs;

      int jogMotorCommand =
          rampToSpeedCommand(winchMotionConfig.jogUpTargetSpeedInPerS,
                             winchFilteredSpeedInPerS,
                             dtSeconds,
                             winchMotionConfig.jogUpCmdRatePerSec,
                             winchMotionConfig.jogUpKpSpeed);

      if (jogMotorCommand > 0 &&
          jogMotorCommand < winchMotionConfig.jogUpMinCommand) {
        jogMotorCommand = winchMotionConfig.jogUpMinCommand;
      }

      applyMotorCommand(jogMotorCommand);

      const float jogTravelIn = winchPositionIn - jogStartPositionIn;
      const bool reliefEvidence =
          jogTravelIn >= winchMotionConfig.jogUpReliefDistanceIn;

      if (reliefEvidence) {
        if (jogReliefDetectedMs == 0U) {
          jogReliefDetectedMs = nowMs;
        }

        if ((nowMs - jogReliefDetectedMs) >=
            winchMotionConfig.jogUpReliefDebounceMs) {
          writePawlServo(winchPawlConfig.openPosition);
          actionTimestampMs = nowMs;
          currentStep = WINCH_STEP_UNLOCK;
        }
      } else {
        jogReliefDetectedMs = 0;
      }

      if (jogTravelIn >= winchMotionConfig.jogUpMaxTravelIn) {
        writePawlServo(winchPawlConfig.openPosition);
        actionTimestampMs = nowMs;
        currentStep = WINCH_STEP_UNLOCK;
      }
      break;
    }

    case WINCH_STEP_UNLOCK:
      writePawlServo(winchPawlConfig.openPosition);
      if ((millis() - actionTimestampMs) >= winchPawlConfig.unlockDelayMs) {
        applyMotorCommand(0);
        currentStep = WINCH_STEP_MOVING;
        actionTimestampMs = millis();
      }
      break;

    case WINCH_STEP_MOVING:
      if (requestedAction == WINCH_ACTION_LOWER) {
        if (targetPositionActive && winchPositionIn <= targetPositionIn) {
          applyMotorCommand(0);
          currentStep = WINCH_STEP_BRAKE_AND_LOCK;
        } else {
          applyMotorCommand(winchMotionConfig.lowerSpeedCmd);
        }
      } else if (requestedAction == WINCH_ACTION_RAISE) {
        if (readHomeSwitchPressedInternal()) {
          applyMotorCommand(0);
          completeActionImmediately();
        } else if (targetPositionActive &&
                   winchPositionIn >= targetPositionIn) {
          applyMotorCommand(0);
          currentStep = WINCH_STEP_BRAKE_AND_LOCK;
        } else {
          applyMotorCommand(winchMotionConfig.raiseSpeedCmd);
        }
      } else if (requestedAction == WINCH_ACTION_MANUAL_LOWER) {
        writePawlServo(winchPawlConfig.openPosition);
        applyMotorCommand(winchMotionConfig.lowerSpeedCmd);
      } else if (requestedAction == WINCH_ACTION_HOMING) {
        const float homingTravelIn = winchPositionIn - homingStartPositionIn;

        if (readHomeSwitchPressedInternal()) {
          applyMotorCommand(0);
          homeSwitchTripPositionIn = winchPositionIn;
          writePawlServo(winchPawlConfig.openPosition);
          actionTimestampMs = millis();
          currentStep = WINCH_STEP_HOME_SWITCH_HIT;
        } else if (winchPositionIn >= homeSlowdownPositionIn) {
          currentStep = WINCH_STEP_HOMING_SLOW;
        } else if (homingTravelIn >= winchHomingConfig.maxSearchTravelIn ||
                   winchPositionIn > winchHomingConfig.maxOvershootIn) {
          applyMotorCommand(0);
          currentStep = WINCH_STEP_BRAKE_AND_LOCK;
          Serial.println("Homing aborted: exceeded homing safety limit.");
        } else {
          applyMotorCommand(winchMotionConfig.raiseSpeedCmd);
        }
      }
      break;

    case WINCH_STEP_HOMING_SLOW:
      if (readHomeSwitchPressedInternal()) {
        applyMotorCommand(0);
        homeSwitchTripPositionIn = winchPositionIn;
        writePawlServo(winchPawlConfig.openPosition);
        actionTimestampMs = millis();
        currentStep = WINCH_STEP_HOME_SWITCH_HIT;
      } else if ((winchPositionIn - homingStartPositionIn) >=
                     winchHomingConfig.maxSearchTravelIn ||
                 winchPositionIn > winchHomingConfig.maxOvershootIn) {
        applyMotorCommand(0);
        currentStep = WINCH_STEP_BRAKE_AND_LOCK;
        Serial.println("Homing aborted: exceeded homing safety limit.");
      } else {
        applyMotorCommand(winchMotionConfig.homingSlowSpeedCmd);
      }
      break;

    case WINCH_STEP_HOME_SWITCH_HIT:
      writePawlServo(winchPawlConfig.openPosition);

      if ((millis() - actionTimestampMs) < winchPawlConfig.unlockDelayMs) {
        applyMotorCommand(0);
      } else {
        applyMotorCommand(winchMotionConfig.lowerSpeedCmd);
      }

      if (winchPositionIn <=
          (homeSwitchTripPositionIn - winchHomingConfig.backoffDistanceIn)) {
        applyMotorCommand(0);
        resetWinchEncoderToCurrentPosition(0.0f);
        currentStep = WINCH_STEP_BRAKE_AND_LOCK;
      } else if (winchPositionIn >=
                 (homeSwitchTripPositionIn + winchHomingConfig.maxOvershootIn)) {
        applyMotorCommand(0);
        currentStep = WINCH_STEP_BRAKE_AND_LOCK;
        Serial.println("Homing aborted: overshot home switch trigger point.");
      }
      break;

    case WINCH_STEP_BRAKE_AND_LOCK:
      /*
        Keep the original behavior from winch_test.cpp:
        stop the motor, command the pawl closed, wait for the mechanism to seat,
        then declare the action complete.
      */
      applyMotorCommand(0);
      writePawlServo(winchPawlConfig.lockPosition);
      delay(winchPawlConfig.lockDelayMs);
      completeActionImmediately();
      Serial.println("Finished!");
      break;
  }
}

#pragma endregion

}  // namespace

#pragma region Configuration_Functions

void setWinchHardwareBindings(DualG2HighPowerMotorShield *motorShield,
                              Servo *pawlServo) {
  winchHardware.motorShield = motorShield;
  winchHardware.pawlServo = pawlServo;
}

void setWinchHardwarePins(uint8_t pawlServoPin,
                          uint8_t homeSwitchPin,
                          bool flipMotorDirection) {
  winchHardwareConfig.pawlServoPin = pawlServoPin;
  winchHardwareConfig.homeSwitchPin = homeSwitchPin;
  winchHardwareConfig.flipMotorDirection = flipMotorDirection;
}

void setWinchPawlParameters(uint8_t lockPosition,
                            uint8_t openPosition,
                            uint32_t lockDelayMs,
                            uint32_t unlockDelayMs) {
  winchPawlConfig.lockPosition = lockPosition;
  winchPawlConfig.openPosition = openPosition;
  winchPawlConfig.lockDelayMs = lockDelayMs;
  winchPawlConfig.unlockDelayMs = unlockDelayMs;
}

void setWinchEncoderParameters(uint8_t muxChannel,
                               float drumRadiusIn,
                               bool invertDirection,
                               int countsPerRevolution,
                               int wrapThreshold,
                               int noiseThreshold,
                               uint8_t muxAddress,
                               uint8_t sensorAddress,
                               uint8_t rawAngleRegister) {
  winchEncoderConfig.wireBus = &Wire;
  winchEncoderConfig.muxAddress = muxAddress;
  winchEncoderConfig.sensorAddress = sensorAddress;
  winchEncoderConfig.rawAngleRegister = rawAngleRegister;
  winchEncoderConfig.muxChannel = muxChannel;
  winchEncoderDirectionInverted = invertDirection;
  winchEncoderConfig.countsPerRevolution = countsPerRevolution;
  winchEncoderConfig.wrapThreshold = wrapThreshold;
  winchEncoderConfig.noiseThreshold = noiseThreshold;
  winchDrumRadiusIn = drumRadiusIn;

  resetEncoderState(winchEncoderState);
  winchEncoderReadSucceeded = false;
}

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
                              uint32_t jogUpReliefDebounceMs) {
  winchMotionConfig.raiseSpeedCmd = raiseSpeedCmd;
  winchMotionConfig.lowerSpeedCmd = lowerSpeedCmd;
  winchMotionConfig.homingSlowSpeedCmd = homingSlowSpeedCmd;
  winchMotionConfig.maxMotorCommand = abs(maxMotorCommand);
  winchMotionConfig.jogUpMinCommand = abs(jogUpMinCommand);
  winchMotionConfig.defaultLowerTargetIn = defaultLowerTargetIn;
  winchMotionConfig.jogUpTargetSpeedInPerS = jogUpTargetSpeedInPerS;
  winchMotionConfig.jogUpReliefDistanceIn = jogUpReliefDistanceIn;
  winchMotionConfig.jogUpMaxTravelIn = jogUpMaxTravelIn;
  winchMotionConfig.jogUpCmdRatePerSec = jogUpCmdRatePerSec;
  winchMotionConfig.jogUpKpSpeed = jogUpKpSpeed;
  winchMotionConfig.jogUpReliefDebounceMs = jogUpReliefDebounceMs;
}

void setWinchSpeedFilterParameters(uint32_t minSamplePeriodMs,
                                   float timeConstantSeconds) {
  winchSpeedFilterConfig.minSamplePeriodMs = minSamplePeriodMs;
  winchSpeedFilterConfig.timeConstantSeconds = timeConstantSeconds;
}

void setWinchHomingParameters(float backoffDistanceIn,
                              float maxSearchTravelIn,
                              float slowdownThresholdIn,
                              float maxOvershootIn) {
  winchHomingConfig.backoffDistanceIn = backoffDistanceIn;
  winchHomingConfig.maxSearchTravelIn = maxSearchTravelIn;
  winchHomingConfig.slowdownThresholdIn = slowdownThresholdIn;
  winchHomingConfig.maxOvershootIn = maxOvershootIn;
}

void setWinchDefaultLowerTarget(float targetDepthInInches) {
  winchMotionConfig.defaultLowerTargetIn = targetDepthInInches;
}

#pragma endregion

#pragma region Public_Action_And_Update_Functions

bool beginWinchControl(float initialZeroPositionIn) {
  if (!isWinchConfigured()) {
    return false;
  }

  Wire.begin();

  winchHardware.motorShield->init();
  winchHardware.motorShield->enableDrivers();
  winchHardware.motorShield->flipM2(winchHardwareConfig.flipMotorDirection);

  pinMode(winchHardwareConfig.homeSwitchPin, INPUT_PULLUP);
  winchHardware.pawlServo->attach(winchHardwareConfig.pawlServoPin);
  writePawlServo(winchPawlConfig.lockPosition);

  requestedAction = WINCH_ACTION_LOWER;
  currentStep = WINCH_STEP_IDLE;
  targetDepthIn = winchMotionConfig.defaultLowerTargetIn;
  targetPositionIn = initialZeroPositionIn;
  jogStartPositionIn = initialZeroPositionIn;
  homingStartPositionIn = initialZeroPositionIn;
  homeSwitchTripPositionIn = initialZeroPositionIn;
  homeSlowdownPositionIn = winchHomingConfig.slowdownThresholdIn;
  actionTimestampMs = millis();
  jogReliefDetectedMs = 0;
  winchActionComplete = false;
  winchMotorFaultActive = false;
  appliedMotorCommand = 0;
  targetPositionActive = false;

  const bool resetSucceeded =
      resetWinchEncoderToCurrentPosition(initialZeroPositionIn);
  applyMotorCommand(0);
  return resetSucceeded;
}

void updateWinchControl() {
  if (!isWinchConfigured()) {
    return;
  }

  updateWinchMeasurements();

  if (winchHardware.motorShield->getM2Fault()) {
    if (!winchMotorFaultActive) {
      Serial.println("Winch motor fault detected. Motor stopped.");
    }
    winchMotorFaultActive = true;
    applyMotorCommand(0);
    return;
  }

  winchMotorFaultActive = false;

  runWinchStateMachine();
}

bool resetWinchEncoderToCurrentPosition(float newZeroInches) {
  if (winchEncoderConfig.countsPerRevolution <= 0) {
    return false;
  }

  const bool zeroSucceeded =
      zeroEncoderToCurrentRaw(winchEncoderState,
                              winchEncoderConfig,
                              convertWinchInchesToCounts(newZeroInches));

  winchEncoderReadSucceeded = zeroSucceeded;
  winchHomePositionIn = newZeroInches;
  resetSpeedEstimatorState(newZeroInches);

  if (targetPositionActive) {
    applyMotorCommand(0);
    completeActionImmediately();
  }

  return zeroSucceeded;
}

bool startWinchAction(WinchActionType action) {
  if (!isWinchConfigured()) {
    return false;
  }

  if (action == WINCH_ACTION_LOWER) {
    targetDepthIn = winchMotionConfig.defaultLowerTargetIn;
    startConfiguredAction(action, true, targetDepthIn);
    return true;
  }

  startConfiguredAction(action);
  return true;
}

bool startWinchLowerToDepth(float requestedTargetDepthIn) {
  if (!isWinchConfigured()) {
    return false;
  }

  targetDepthIn = requestedTargetDepthIn;
  startConfiguredAction(WINCH_ACTION_LOWER, true, requestedTargetDepthIn);
  return true;
}

bool winchToPosition(float desiredPositionIn) {
  if (!isWinchConfigured()) {
    return false;
  }

  targetDepthIn = desiredPositionIn;

  const float positionErrorIn = desiredPositionIn - winchPositionIn;
  if (fabsf(positionErrorIn) <= kWinchMoveToPositionBandIn) {
    applyMotorCommand(0);
    resetRampToSpeedState();
    completeActionImmediately();
    return true;
  }

  if (positionErrorIn > 0.0f) {
    startConfiguredAction(WINCH_ACTION_RAISE, true, desiredPositionIn);
  } else {
    startConfiguredAction(WINCH_ACTION_LOWER, true, desiredPositionIn);
  }
  return true;
}

bool startJogUpForUnlock() {
  return startWinchAction(WINCH_ACTION_LOWER);
}

void finishWinchAction(bool shouldLock) {
  applyMotorCommand(0);
  resetRampToSpeedState();
  winchActionComplete = false;
  clearTargetPositionStop();

  if (shouldLock) {
    currentStep = WINCH_STEP_BRAKE_AND_LOCK;
    return;
  }

  completeActionImmediately();
}

#pragma endregion

#pragma region Public_Status_And_Query_Functions

bool isWinchActionComplete() {
  return winchActionComplete;
}

bool isWinchBusy() {
  return currentStep != WINCH_STEP_IDLE;
}

bool hasWinchFault() {
  return winchMotorFaultActive;
}

bool isWinchHomeSwitchPressed() {
  return readHomeSwitchPressedInternal();
}

float getWinchPosition() {
  return winchPositionIn;
}

float getWinchSpeed() {
  return winchSpeedInPerS;
}

float getWinchSpeedFiltered() {
  return winchFilteredSpeedInPerS;
}

float getWinchTargetDepth() {
  return targetPositionActive ? targetPositionIn : targetDepthIn;
}

float getWinchHomePosition() {
  return winchHomePositionIn;
}

int getWinchAppliedMotorCommand() {
  return appliedMotorCommand;
}

WinchStatus getWinchStatus() {
  WinchStatus status;

  status.requestedAction = requestedAction;
  status.currentStep = currentStep;
  status.positionIn = winchPositionIn;
  status.speedInPerS = winchSpeedInPerS;
  status.filteredSpeedInPerS = winchFilteredSpeedInPerS;
  status.homePositionIn = winchHomePositionIn;
  status.targetDepthIn = targetDepthIn;
  status.jogStartPositionIn = jogStartPositionIn;
  status.actionComplete = winchActionComplete;
  status.actionActive = currentStep != WINCH_STEP_IDLE;
  status.homeSwitchPressed = readHomeSwitchPressedInternal();
  status.motorFaultActive = winchMotorFaultActive;
  status.encoderReadSucceeded = winchEncoderReadSucceeded;
  status.appliedMotorCommand = appliedMotorCommand;

  return status;
}

const char *getWinchActionName(WinchActionType action) {
  switch (action) {
    case WINCH_ACTION_RAISE:
      return "RAISE";
    case WINCH_ACTION_LOWER:
      return "LOWER";
    case WINCH_ACTION_HOMING:
      return "HOMING";
    case WINCH_ACTION_MANUAL_LOWER:
      return "MANUAL_LOWER";
  }

  return "UNKNOWN_ACTION";
}

const char *getWinchStepName(WinchStep step) {
  switch (step) {
    case WINCH_STEP_IDLE:
      return "IDLE";
    case WINCH_STEP_JOG_UP:
      return "JOG_UP";
    case WINCH_STEP_UNLOCK:
      return "UNLOCK";
    case WINCH_STEP_MOVING:
      return "MOVING";
    case WINCH_STEP_HOMING_SLOW:
      return "HOMING_SLOW";
    case WINCH_STEP_HOME_SWITCH_HIT:
      return "HOME_SWITCH_HIT";
    case WINCH_STEP_BRAKE_AND_LOCK:
      return "BRAKE_AND_LOCK";
  }

  return "UNKNOWN_STEP";
}

#pragma endregion
