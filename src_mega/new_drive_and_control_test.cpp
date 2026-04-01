#include <Arduino.h>
#include <Encoder.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "DriveControl.h"
#include "EncoderMux.h"
#include "DualG2HighPowerMotorShield.h"

// ===== Hardware configuration =====
DualG2HighPowerMotorShield24v14 md;
Encoder driveEncoder(19, 18);

const bool FLIP_DRIVE_MOTOR = false;

// ===== Calibration values copied from drive_and_control_test.cpp =====
const float DRIVE_ENCODER_COUNTS_PER_MOTOR_REV = 1200.0f;
const float DRIVE_GEAR_RATIO = 0.5f;
const float DRIVE_WHEEL_RADIUS_IN = 0.6f;

const uint8_t REAL_POSITION_MUX_CH = 5;
const float REAL_POSITION_WHEEL_DIAMETER_IN = 0.875f;
const int AS5600_COUNTS_PER_REV = 4096;
const int AS5600_WRAP_THRESH = AS5600_COUNTS_PER_REV / 2;
const int AS5600_NOISE_THRESH = 1;

// ===== Runtime / control settings =====
const int MAX_MOTOR_CMD = 400;
const uint32_t CONTROL_INTERVAL_MS = 20;
const uint32_t PRINT_INTERVAL_MS = 200;
const float TARGET_TOLERANCE_IN = 0.15f;
const float STOP_VELOCITY_TOLERANCE_IN_S = 0.20f;
const uint32_t TARGET_SETTLE_TIME_MS = 250;

// The new trajectory helper uses one acceleration limit for both accel/decel.
float steadyCruiseSpeedInPerS = 12.0f;
float trajectoryMaxAccelInPerS2 = 18.0f;

// The new ramp controller is position based, so the test sketch exposes PID
// tuning directly instead of the old velocity-loop terms.
float rampKp = 24.0f;
float rampKi = 10.0f;
float rampKd = 0.0f;
float rampIntegralWindowIn = 4.0f;

const float REAL_POSITION_FIR_COEFFS[] = {1.0f};

enum ControlMode : uint8_t {
  MODE_IDLE = 0,
  MODE_MANUAL,
  MODE_TRAJECTORY
};

enum StreamMode : uint8_t {
  STREAM_AUTO = 0,
  STREAM_ON,
  STREAM_OFF
};

struct DriveSnapshot {
  long driveCounts;
  long realCounts;
  float drivePosIn;
  float realPosRawIn;
  float realPosFirIn;
  float driveVelInPerS;
  float realVelInPerS;
};

DriveSnapshot currentState = {0, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
DriveSnapshot previousState = currentState;

ControlMode controlMode = MODE_IDLE;
StreamMode streamMode = STREAM_AUTO;
TrajectoryProfile activeTrajectory = {};

int manualMotorCmd = 0;
int appliedMotorCmd = 0;
float activeDesiredPositionIn = 0.0f;
float targetPositionIn = 0.0f;

uint32_t trajectoryStartMs = 0;
uint32_t lastControlMs = 0;
uint32_t lastPrintMs = 0;
uint32_t targetSettledSinceMs = 0;

char serialBuf[96];
uint8_t serialIdx = 0;

// ===== Prototypes =====
void configureDriveControl();
void applyRampSettings();
bool zeroAllSensors(float newZeroPositionIn = 0.0f);
void refreshSensors(float dtSeconds);

float getDriveEncoderDistanceIn();
float getSlipDistanceIn();

void applyMotorCommand(int speedCmd);
void stopDrive();
void startManualMode(int speedCmd);
bool startTrajectoryMove(float targetIn);
void updateControl();

bool withinTargetSettleWindow();
bool targetReached();
bool shouldPrintStatusStream();

void handleSerialInput();
void processCommand(char *cmd);
void printHelp();
void printStatus();
void printTrajectorySummary();

const char *getControlModeName(ControlMode mode);
const char *getStreamModeName(StreamMode mode);

void configureDriveControl() {
  EncoderMuxConfig realPositionConfig = makeDefaultAs5600MuxConfig(REAL_POSITION_MUX_CH);
  realPositionConfig.countsPerRevolution = AS5600_COUNTS_PER_REV;
  realPositionConfig.wrapThreshold = AS5600_WRAP_THRESH;
  realPositionConfig.noiseThreshold = AS5600_NOISE_THRESH;

  setDriveRealPositionSensorParameters(realPositionConfig.muxChannel,
                                       REAL_POSITION_WHEEL_DIAMETER_IN * 0.5f,
                                       realPositionConfig.countsPerRevolution,
                                       realPositionConfig.wrapThreshold,
                                       realPositionConfig.noiseThreshold,
                                       realPositionConfig.muxAddress,
                                       realPositionConfig.sensorAddress,
                                       realPositionConfig.rawAngleRegister);

  setDriveRealPositionFirParameters(REAL_POSITION_FIR_COEFFS,
                                    (uint8_t)(sizeof(REAL_POSITION_FIR_COEFFS) /
                                              sizeof(REAL_POSITION_FIR_COEFFS[0])));
  applyRampSettings();
}

void applyRampSettings() {
  setRampControlGains(rampKp, rampKi, rampKd);
  setRampIntegralWindow(rampIntegralWindowIn);
}

bool zeroAllSensors(float newZeroPositionIn) {
  applyMotorCommand(0);
  controlMode = MODE_IDLE;
  manualMotorCmd = 0;
  activeTrajectory = {};

  driveEncoder.write(0);
  resetRampController();

  const bool sensorResetOk = resetDriveRealPositionSensor(newZeroPositionIn);

  currentState.driveCounts = 0;
  currentState.realCounts = getDriveRealPositionCounts();
  currentState.drivePosIn = 0.0f;
  currentState.realPosRawIn = getDriveRealPosition();
  currentState.realPosFirIn = getDriveRealPositionFir();
  currentState.driveVelInPerS = 0.0f;
  currentState.realVelInPerS = 0.0f;
  previousState = currentState;

  activeDesiredPositionIn = currentState.realPosFirIn;
  targetPositionIn = currentState.realPosFirIn;
  targetSettledSinceMs = 0;
  trajectoryStartMs = millis();

  return sensorResetOk;
}

float getDriveEncoderDistanceIn() {
  const long counts = driveEncoder.read();
  const float wheelRevs =
      ((float)counts) / (DRIVE_GEAR_RATIO * DRIVE_ENCODER_COUNTS_PER_MOTOR_REV);
  return wheelRevs * (2.0f * PI * DRIVE_WHEEL_RADIUS_IN);
}

float getSlipDistanceIn() {
  return currentState.drivePosIn - currentState.realPosFirIn;
}

void refreshSensors(float dtSeconds) {
  previousState = currentState;

  updateDriveRealPositionFromSensor();

  currentState.driveCounts = driveEncoder.read();
  currentState.realCounts = getDriveRealPositionCounts();
  currentState.drivePosIn = getDriveEncoderDistanceIn();
  currentState.realPosRawIn = getDriveRealPosition();
  currentState.realPosFirIn = getDriveRealPositionFir();

  if (dtSeconds <= 0.0f) {
    currentState.driveVelInPerS = 0.0f;
    currentState.realVelInPerS = 0.0f;
    return;
  }

  currentState.driveVelInPerS =
      (currentState.drivePosIn - previousState.drivePosIn) / dtSeconds;
  currentState.realVelInPerS =
      (currentState.realPosFirIn - previousState.realPosFirIn) / dtSeconds;
}

void applyMotorCommand(int speedCmd) {
  appliedMotorCmd = constrain(speedCmd, -MAX_MOTOR_CMD, MAX_MOTOR_CMD);
  md.setM1Speed(appliedMotorCmd);
}

void stopDrive() {
  controlMode = MODE_IDLE;
  manualMotorCmd = 0;
  activeTrajectory = {};
  resetRampController();
  activeDesiredPositionIn = currentState.realPosFirIn;
  targetPositionIn = currentState.realPosFirIn;
  targetSettledSinceMs = 0;
  applyMotorCommand(0);
}

void startManualMode(int speedCmd) {
  controlMode = MODE_MANUAL;
  manualMotorCmd = constrain(speedCmd, -MAX_MOTOR_CMD, MAX_MOTOR_CMD);
  activeTrajectory = {};
  resetRampController();
  activeDesiredPositionIn = currentState.realPosFirIn;
  targetPositionIn = currentState.realPosFirIn;
  targetSettledSinceMs = 0;
  applyMotorCommand(manualMotorCmd);
}

bool startTrajectoryMove(float targetIn) {
  activeTrajectory = buildTrajectoryProfile(trajectoryMaxAccelInPerS2,
                                           steadyCruiseSpeedInPerS,
                                           currentState.realPosFirIn,
                                           targetIn);
  if (!activeTrajectory.isValid) {
    activeTrajectory = {};
    return false;
  }

  controlMode = MODE_TRAJECTORY;
  targetPositionIn = targetIn;
  activeDesiredPositionIn = currentState.realPosFirIn;
  trajectoryStartMs = millis();
  targetSettledSinceMs = 0;

  resetRampController();
  applyRampSettings();
  return true;
}

bool withinTargetSettleWindow() {
  const float positionError = targetPositionIn - currentState.realPosFirIn;
  return fabsf(positionError) <= TARGET_TOLERANCE_IN &&
         fabsf(currentState.realVelInPerS) <= STOP_VELOCITY_TOLERANCE_IN_S;
}

bool targetReached() {
  if (!withinTargetSettleWindow()) {
    targetSettledSinceMs = 0;
    return false;
  }

  const uint32_t nowMs = millis();
  if (targetSettledSinceMs == 0) {
    targetSettledSinceMs = nowMs;
    return false;
  }

  return (nowMs - targetSettledSinceMs) >= TARGET_SETTLE_TIME_MS;
}

void updateControl() {
  if (controlMode == MODE_IDLE) {
    applyMotorCommand(0);
    return;
  }

  if (controlMode == MODE_MANUAL) {
    applyMotorCommand(manualMotorCmd);
    return;
  }

  const float elapsedTimeSeconds =
      (float)(millis() - trajectoryStartMs) / 1000.0f;

  activeDesiredPositionIn =
      getTrajectoryPositionAtElapsedTime(activeTrajectory, elapsedTimeSeconds);

  const int motorCmd = rampControl(activeDesiredPositionIn,
                                   currentState.realPosFirIn);
  applyMotorCommand(motorCmd);

  if (isTrajectoryFinished(activeTrajectory, elapsedTimeSeconds) && targetReached()) {
    stopDrive();
    Serial.println("Target reached.");
  }
}

bool shouldPrintStatusStream() {
  if (streamMode == STREAM_ON) {
    return true;
  }

  if (streamMode == STREAM_OFF) {
    return false;
  }

  return (controlMode != MODE_IDLE) || (appliedMotorCmd != 0);
}

const char *getControlModeName(ControlMode mode) {
  switch (mode) {
    case MODE_IDLE:
      return "IDLE";
    case MODE_MANUAL:
      return "MANUAL";
    case MODE_TRAJECTORY:
      return "TRAJ";
    default:
      return "?";
  }
}

const char *getStreamModeName(StreamMode mode) {
  switch (mode) {
    case STREAM_AUTO:
      return "AUTO";
    case STREAM_ON:
      return "ON";
    case STREAM_OFF:
      return "OFF";
    default:
      return "?";
  }
}

void printHelp() {
  Serial.println("New drive + control test");
  Serial.println("Commands:");
  Serial.println("  h                   -> print this help");
  Serial.println("  z [pos_in]          -> stop and zero sensors at the current location");
  Serial.println("  x                   -> stop motor and exit control mode");
  Serial.println("  p                   -> print one status line");
  Serial.println("  stream [on|off|auto]-> set periodic status streaming mode");
  Serial.println("  say <text>          -> print any text you want back to serial");
  Serial.println("  m <cmd>             -> manual motor command (-400 to 400)");
  Serial.println("  g <inches>          -> move to absolute rail position using trajectory + PID");
  Serial.println("  profile             -> print the active trajectory summary");
  Serial.println("  ss <ft/s>           -> set steady-state cruise speed");
  Serial.println("  accel <in/s^2>      -> set trajectory acceleration limit");
  Serial.println("  kp <value>          -> set ramp controller proportional gain");
  Serial.println("  ki <value>          -> set ramp controller integral gain");
  Serial.println("  kd <value>          -> set ramp controller derivative gain");
  Serial.println("  iwin <inches>       -> set integral accumulation window");
  Serial.println("Status units: pos=in, vel=in/s, accel=in/s^2, cruise=ft/s");
}

void printTrajectorySummary() {
  if (!activeTrajectory.isValid) {
    Serial.println("No valid trajectory is active.");
    return;
  }

  Serial.print("Profile | Start: ");
  Serial.print(activeTrajectory.startPosition, 3);
  Serial.print(" in | Stop: ");
  Serial.print(activeTrajectory.stopPosition, 3);
  Serial.print(" in | Peak: ");
  Serial.print(activeTrajectory.peakSpeed, 3);
  Serial.print(" in/s | TotalTime: ");
  Serial.print(activeTrajectory.totalDurationSeconds, 3);
  Serial.print(" s | Shape: ");
  Serial.println(activeTrajectory.reachesSteadyStateSpeed ? "TRAPEZOID" : "TRIANGLE");
}

void printStatus() {
  const float slipIn = getSlipDistanceIn();

  Serial.print("Mode: ");
  Serial.print(getControlModeName(controlMode));
  Serial.print(" | Stream: ");
  Serial.print(getStreamModeName(streamMode));
  Serial.print(" | MotorCmd: ");
  Serial.print(appliedMotorCmd);
  Serial.print(" | TargetPos(in): ");
  Serial.print(targetPositionIn, 3);
  Serial.print(" | DesiredPos(in): ");
  Serial.print(activeDesiredPositionIn, 3);
  Serial.print(" | Cruise(ft/s): ");
  Serial.print(steadyCruiseSpeedInPerS / 12.0f, 3);
  Serial.print(" | Accel(in/s^2): ");
  Serial.print(trajectoryMaxAccelInPerS2, 3);
  Serial.print(" | PID(Kp/Ki/Kd): ");
  Serial.print(rampKp, 3);
  Serial.print("/");
  Serial.print(rampKi, 3);
  Serial.print("/");
  Serial.print(rampKd, 3);
  Serial.print(" | IWin(in): ");
  Serial.print(rampIntegralWindowIn, 3);
  Serial.print(" | RealRaw(in): ");
  Serial.print(currentState.realPosRawIn, 3);
  Serial.print(" | RealFir(in): ");
  Serial.print(currentState.realPosFirIn, 3);
  Serial.print(" | DrivePos(in): ");
  Serial.print(currentState.drivePosIn, 3);
  Serial.print(" | RealVel(in/s): ");
  Serial.print(currentState.realVelInPerS, 3);
  Serial.print(" | DriveVel(in/s): ");
  Serial.print(currentState.driveVelInPerS, 3);
  Serial.print(" | Slip(in): ");
  Serial.print(slipIn, 3);
  Serial.print(" | DriveCnt: ");
  Serial.print(currentState.driveCounts);
  Serial.print(" | RealCnt: ");
  Serial.print(currentState.realCounts);
  Serial.print(" | Sensor: ");
  Serial.print(didDriveRealPositionReadSucceed() ? "OK" : "FAIL");

  if (withinTargetSettleWindow()) {
    Serial.print(" | SETTLING");
  }

  Serial.println();
}

void processCommand(char *cmd) {
  if (cmd[0] == '\0') {
    return;
  }

  char *messageText = strchr(cmd, ' ');
  if (messageText != NULL) {
    while (*messageText == ' ') {
      ++messageText;
    }
  }
  char *verb = strtok(cmd, " ");
  if (verb == NULL) {
    return;
  }

  if (strcmp(verb, "h") == 0) {
    printHelp();
    return;
  }

  if (strcmp(verb, "z") == 0) {
    char *arg = strtok(NULL, " ");
    const float newZeroPositionIn = (arg == NULL) ? 0.0f : (float)atof(arg);
    const bool sensorResetOk = zeroAllSensors(newZeroPositionIn);

    Serial.print("Sensors reset");
    Serial.print(sensorResetOk ? "." : ", but the real-position sensor did not ack.");
    Serial.print(" New zero = ");
    Serial.print(newZeroPositionIn, 3);
    Serial.println(" in");
    return;
  }

  if (strcmp(verb, "x") == 0) {
    stopDrive();
    Serial.println("Drive stopped.");
    return;
  }

  if (strcmp(verb, "p") == 0) {
    printStatus();
    return;
  }

  if (strcmp(verb, "stream") == 0) {
    char *arg = strtok(NULL, " ");
    if (arg == NULL) {
      Serial.print("Streaming mode = ");
      Serial.println(getStreamModeName(streamMode));
      return;
    }

    if (strcmp(arg, "on") == 0) {
      streamMode = STREAM_ON;
    } else if (strcmp(arg, "off") == 0) {
      streamMode = STREAM_OFF;
    } else if (strcmp(arg, "auto") == 0) {
      streamMode = STREAM_AUTO;
    } else {
      Serial.println("Usage: stream [on|off|auto]");
      return;
    }

    Serial.print("Streaming mode set to ");
    Serial.println(getStreamModeName(streamMode));
    if (streamMode != STREAM_OFF) {
      printStatus();
    }
    return;
  }

  if (strcmp(verb, "say") == 0) {
    if (messageText == NULL || *messageText == '\0') {
      Serial.println("Usage: say <text>");
      return;
    }

    Serial.println(messageText);
    return;
  }

  if (strcmp(verb, "m") == 0) {
    char *arg = strtok(NULL, " ");
    if (arg == NULL) {
      Serial.println("Usage: m <cmd>");
      return;
    }

    startManualMode(atoi(arg));
    Serial.print("Manual motor command set to ");
    Serial.println(manualMotorCmd);
    return;
  }

  if (strcmp(verb, "g") == 0) {
    char *arg = strtok(NULL, " ");
    if (arg == NULL) {
      Serial.println("Usage: g <inches>");
      return;
    }

    const float requestedTargetIn = (float)atof(arg);
    if (!startTrajectoryMove(requestedTargetIn)) {
      Serial.println("Unable to build a valid trajectory. Check accel and cruise speed.");
      return;
    }

    Serial.print("Moving to ");
    Serial.print(targetPositionIn, 3);
    Serial.println(" in");
    printTrajectorySummary();
    return;
  }

  if (strcmp(verb, "profile") == 0) {
    printTrajectorySummary();
    return;
  }

  if (strcmp(verb, "ss") == 0) {
    char *arg = strtok(NULL, " ");
    if (arg == NULL) {
      Serial.println("Usage: ss <ft/s>");
      return;
    }

    steadyCruiseSpeedInPerS = max(0.1f, (float)atof(arg) * 12.0f);
    Serial.print("steadyCruiseSpeed = ");
    Serial.print(steadyCruiseSpeedInPerS / 12.0f, 3);
    Serial.println(" ft/s");
    return;
  }

  if (strcmp(verb, "accel") == 0) {
    char *arg = strtok(NULL, " ");
    if (arg == NULL) {
      Serial.println("Usage: accel <in/s^2>");
      return;
    }

    trajectoryMaxAccelInPerS2 = max(0.1f, (float)atof(arg));
    Serial.print("trajectoryMaxAccelInPerS2 = ");
    Serial.println(trajectoryMaxAccelInPerS2, 3);
    return;
  }

  if (strcmp(verb, "kp") == 0) {
    char *arg = strtok(NULL, " ");
    if (arg == NULL) {
      Serial.println("Usage: kp <value>");
      return;
    }

    rampKp = (float)atof(arg);
    applyRampSettings();
    Serial.print("rampKp = ");
    Serial.println(rampKp, 4);
    return;
  }

  if (strcmp(verb, "ki") == 0) {
    char *arg = strtok(NULL, " ");
    if (arg == NULL) {
      Serial.println("Usage: ki <value>");
      return;
    }

    rampKi = (float)atof(arg);
    applyRampSettings();
    Serial.print("rampKi = ");
    Serial.println(rampKi, 4);
    return;
  }

  if (strcmp(verb, "kd") == 0) {
    char *arg = strtok(NULL, " ");
    if (arg == NULL) {
      Serial.println("Usage: kd <value>");
      return;
    }

    rampKd = (float)atof(arg);
    applyRampSettings();
    Serial.print("rampKd = ");
    Serial.println(rampKd, 4);
    return;
  }

  if (strcmp(verb, "iwin") == 0) {
    char *arg = strtok(NULL, " ");
    if (arg == NULL) {
      Serial.println("Usage: iwin <inches>");
      return;
    }

    rampIntegralWindowIn = max(0.0f, (float)atof(arg));
    applyRampSettings();
    Serial.print("rampIntegralWindowIn = ");
    Serial.println(rampIntegralWindowIn, 4);
    return;
  }

  Serial.println("Unknown command. Enter h for help.");
}

void handleSerialInput() {
  while (Serial.available() > 0) {
    const char c = Serial.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      serialBuf[serialIdx] = '\0';
      processCommand(serialBuf);
      serialIdx = 0;
      continue;
    }

    if (serialIdx < sizeof(serialBuf) - 1) {
      serialBuf[serialIdx++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  md.init();
  md.enableDrivers();
  md.flipM1(FLIP_DRIVE_MOTOR);

  configureDriveControl();
  zeroAllSensors();

  lastControlMs = millis();
  lastPrintMs = millis();

  printHelp();
  printStatus();
}

void loop() {
  handleSerialInput();

  if (md.getM1Fault()) {
    stopDrive();
    Serial.println("Motor fault on M1. Drive stopped.");
    delay(50);
    return;
  }

  const uint32_t nowMs = millis();

  if (nowMs - lastControlMs >= CONTROL_INTERVAL_MS) {
    const float dtSeconds = (float)(nowMs - lastControlMs) / 1000.0f;
    lastControlMs = nowMs;

    refreshSensors(dtSeconds);

    if (controlMode == MODE_TRAJECTORY && !didDriveRealPositionReadSucceed()) {
      stopDrive();
      Serial.println("Real-position sensor read failed. Trajectory stopped.");
    } else {
      updateControl();
    }
  }

  if (nowMs - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = nowMs;
    if (shouldPrintStatusStream()) {
      printStatus();
    }
  }
}
