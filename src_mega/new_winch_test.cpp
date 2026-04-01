#include <Arduino.h>
#include <Servo.h>

#include "DualG2HighPowerMotorShield.h"
#include "WinchControl.h"

/*
  new_winch_test.cpp

  This sketch is the reusable-library version of the original winch_test.cpp.
  The goals are:
  - keep the same basic serial-driven winch tests
  - move the actual winch behavior into WinchControl
  - keep the sketch focused on setup, commands, and status printing
*/

#pragma region Hardware_Objects

DualG2HighPowerMotorShield24v14 md;
Servo pawlServo;

#pragma endregion

#pragma region Winch_Test_Configuration

/*
  Pin assignments copied from the original winch_test.cpp.
*/
const uint8_t PAWL_SERVO_PIN = 13;
const uint8_t WINCH_HOME_SWITCH_PIN = 3;

/*
  Servo positions copied from the original winch_test.cpp.
*/
const uint8_t PAWL_SERVO_LOCK_POS = 70;
const uint8_t PAWL_SERVO_OPEN_POS = 130;

/*
  I2C / encoder configuration copied from the original winch_test.cpp.
*/
const uint8_t I2C_MUX_ADDR = 0x70;
const uint8_t AS5600_ADDR = 0x36;
const uint8_t AS5600_RAW_ANGLE_REG = 0x0C;
const uint8_t WINCH_ENCODER_MUX_CH = 7;
const bool FLIP_WINCH_ENCODER_SIGN = true;
const int WINCH_ENCODER_COUNTS_PER_REV = 4096;
const int WINCH_WRAP_THRESHOLD = WINCH_ENCODER_COUNTS_PER_REV / 2;
const int WINCH_NOISE_THRESHOLD = 4;

/*
  Mechanical calibration copied from the original winch_test.cpp.
*/
const float WINCH_DRUM_RADIUS_IN = (1.486f / 2.0f) + (1.0f / 32.0f);

/*
  Motor commands and timing values copied from the original winch_test.cpp.
*/
const bool FLIP_WINCH_MOTOR_DIRECTION = true;
const int WINCH_RAISE_SPEED_CMD = 400;
const int WINCH_LOWER_SPEED_CMD = -200;
const int WINCH_HOMING_SLOW_SPEED_CMD = 140;
const int WINCH_MAX_MOTOR_COMMAND = 400;
const int WINCH_JOG_UP_MIN_COMMAND = 80;
const uint32_t PAWL_LOCK_DELAY_MS = 500;
const uint32_t PAWL_UNLOCK_DELAY_MS = 300;

/*
  Jog-unlock tuning copied from the original winch_test.cpp.
*/
const float JOG_UP_TARGET_SPEED_IN_S = 0.15f;
const float JOG_UP_RELIEF_DISTANCE_IN = 0.03f;
const float JOG_UP_MAX_TRAVEL_IN = 0.50f;
const uint32_t JOG_UP_RELIEF_DEBOUNCE_MS = 120;
const float JOG_UP_CMD_RATE_PER_SEC = 350.0f;
const float JOG_UP_KP_SPEED = 900.0f;

/*
  Homing limits copied from the original winch_test.cpp.
*/
const float HOMING_BACKOFF_DISTANCE_IN = 1.0f;
const float HOMING_MAX_SEARCH_TRAVEL_IN = 60.0f;
const float HOMING_SLOWDOWN_THRESHOLD_IN = -2.0f;
const float HOMING_MAX_OVERSHOOT_IN = 0.25f;

/*
  Filter settings copied from the original winch_test.cpp speed estimator.
*/
const uint32_t WINCH_FILTER_MIN_DT_MS = 10;
const float WINCH_FILTER_TAU_S = 0.05f;

/*
  Default lower target copied from the original winch_test.cpp.
*/
const float DEFAULT_TEST_LOWER_TARGET_IN = -10.0f;

#pragma endregion

#pragma region Function_Prototypes

void configureWinchControl();
void handleTestSerial();
void printWinchStatus();
void printHelp();

#pragma endregion

#pragma region Setup_And_Loop

void setup() {
  Serial.begin(115200);

  configureWinchControl();
  const bool beginSucceeded = beginWinchControl(0.0f);

  printHelp();

  if (!beginSucceeded) {
    Serial.println("WinchControl begin failed. Check hardware bindings and encoder configuration.");
  }
}

void loop() {
  updateWinchControl();
  printWinchStatus();
  handleTestSerial();
}

#pragma endregion

#pragma region Test_Setup_Functions

void configureWinchControl() {
  /*
    All of the values that used to be hard-coded inside the legacy test now
    flow through explicit setup helpers. That keeps WinchControl reusable while
    still making the chosen test configuration easy to see in one place.
  */
  setWinchHardwareBindings(&md, &pawlServo);
  setWinchHardwarePins(PAWL_SERVO_PIN,
                       WINCH_HOME_SWITCH_PIN,
                       FLIP_WINCH_MOTOR_DIRECTION);

  setWinchPawlParameters(PAWL_SERVO_LOCK_POS,
                         PAWL_SERVO_OPEN_POS,
                         PAWL_LOCK_DELAY_MS,
                         PAWL_UNLOCK_DELAY_MS);

  setWinchEncoderParameters(WINCH_ENCODER_MUX_CH,
                            WINCH_DRUM_RADIUS_IN,
                            FLIP_WINCH_ENCODER_SIGN,
                            WINCH_ENCODER_COUNTS_PER_REV,
                            WINCH_WRAP_THRESHOLD,
                            WINCH_NOISE_THRESHOLD,
                            I2C_MUX_ADDR,
                            AS5600_ADDR,
                            AS5600_RAW_ANGLE_REG);

  setWinchMotionParameters(WINCH_RAISE_SPEED_CMD,
                           WINCH_LOWER_SPEED_CMD,
                           WINCH_HOMING_SLOW_SPEED_CMD,
                           WINCH_MAX_MOTOR_COMMAND,
                           WINCH_JOG_UP_MIN_COMMAND,
                           DEFAULT_TEST_LOWER_TARGET_IN,
                           JOG_UP_TARGET_SPEED_IN_S,
                           JOG_UP_RELIEF_DISTANCE_IN,
                           JOG_UP_MAX_TRAVEL_IN,
                           JOG_UP_CMD_RATE_PER_SEC,
                           JOG_UP_KP_SPEED,
                           JOG_UP_RELIEF_DEBOUNCE_MS);

  setWinchSpeedFilterParameters(WINCH_FILTER_MIN_DT_MS, WINCH_FILTER_TAU_S);

  setWinchHomingParameters(HOMING_BACKOFF_DISTANCE_IN,
                           HOMING_MAX_SEARCH_TRAVEL_IN,
                           HOMING_SLOWDOWN_THRESHOLD_IN,
                           HOMING_MAX_OVERSHOOT_IN);
}

#pragma endregion

#pragma region Serial_Test_Interface

void handleTestSerial() {
  if (Serial.available() <= 0) {
    return;
  }

  const char command = (char)Serial.read();

  if (command == 'j') {
    startJogUpForUnlock();
  } else if (command == 'u') {
    startWinchAction(WINCH_ACTION_RAISE);
  } else if (command == 'd') {
    startWinchAction(WINCH_ACTION_MANUAL_LOWER);
  } else if (command == 'x') {
    finishWinchAction(true);
  } else if (command == 'h') {
    startWinchAction(WINCH_ACTION_HOMING);
  } else if (command == 'z') {
    const bool resetOk = resetWinchEncoderToCurrentPosition(0.0f);
    Serial.println(resetOk ? "Winch encoder reset to zero." :
                             "Winch encoder reset failed.");
  } else if (command == '?') {
    printHelp();
  }
}

void printHelp() {
  Serial.println("Winch test commands:");
  Serial.println("  j = jog up, unlock, then lower to the configured target");
  Serial.println("  u = raise continuously");
  Serial.println("  d = manual lower continuously with pawl open");
  Serial.println("  h = run the homing routine");
  Serial.println("  x = stop and lock");
  Serial.println("  z = zero encoder at the current position");
  Serial.println("  ? = print this help again");
}

#pragma endregion

#pragma region Status_Printing

void printWinchStatus() {
  const WinchStatus status = getWinchStatus();

  Serial.print("Pos: ");
  Serial.print(status.positionIn);
  Serial.print(" | Speed: ");
  Serial.print(status.speedInPerS);
  Serial.print(", ");
  Serial.print(status.filteredSpeedInPerS);
  Serial.print(" | Step: ");
  Serial.print(getWinchStepName(status.currentStep));
  Serial.print(" | Action: ");
  Serial.print(getWinchActionName(status.requestedAction));
  Serial.print(" | Cmd: ");
  Serial.print(status.appliedMotorCommand);

  if (status.homeSwitchPressed) {
    Serial.print(" | HOME");
  }

  if (status.actionComplete) {
    Serial.print(" | COMPLETE");
  }

  if (!status.encoderReadSucceeded) {
    Serial.print(" | ENCODER_READ_FAIL");
  }

  if (status.motorFaultActive) {
    Serial.print(" | MOTOR_FAULT");
  }

  Serial.println();
}

#pragma endregion
