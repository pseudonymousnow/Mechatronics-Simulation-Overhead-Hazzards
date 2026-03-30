#include <Arduino.h>
#include <Encoder.h>
#include <Wire.h>
#include <math.h>
#include "DualG2HighPowerMotorShield.h"

// ===== Hardware configuration =====
DualG2HighPowerMotorShield24v14 md;
Encoder driveEncoder(19, 18);

const uint8_t I2C_MUX_ADDR = 0x70;
const uint8_t AS5600_ADDR = 0x36;
const uint8_t AS5600_RAW_ANGLE_REG = 0x0C;
const uint8_t REAL_POSITION_MUX_CH = 5;

const bool FLIP_DRIVE_MOTOR = false;

// ===== Calibration constants =====
const float DRIVE_ENCODER_COUNTS_PER_MOTOR_REV = 1200.0f;
const float DRIVE_GEAR_RATIO = 0.5f;
const float DRIVE_WHEEL_RADIUS_IN = 0.6f;

const float REAL_POSITION_WHEEL_DIAMETER_IN = 0.875f;
const float REAL_POSITION_WHEEL_CIRCUMFERENCE_IN = PI * REAL_POSITION_WHEEL_DIAMETER_IN;
const int AS5600_COUNTS_PER_REV = 4096;
const int AS5600_WRAP_THRESH = AS5600_COUNTS_PER_REV / 2;
const int AS5600_NOISE_THRESH = 1;

const int MAX_MOTOR_CMD = 400;
const float CONTROL_INTERVAL_S = 0.02f;
const uint32_t CONTROL_INTERVAL_MS = 20;
const uint32_t PRINT_INTERVAL_MS = 200;
const float TARGET_TOLERANCE_IN = 0.15f;
const float STOP_VELOCITY_TOLERANCE_IN_S = 0.20f;
const float MAX_CRUISE_SPEED_IN_PER_S = 24.0f;  // 2 ft/s max requirement
const uint32_t TARGET_SETTLE_TIME_MS = 250;

// ===== Controller tuning =====
// Motion profile: hold a requested cruise speed, then slow down near the target.
float steadyCruiseSpeedInPerS = 12.0f;   // 1 ft/s default
float rampUpAccelInPerS2 = 18.0f;        // acceleration limit to reduce startup slip
float slowdownDecelInPerS2 = 20.0f;      // used to compute braking speed from remaining distance
float finalApproachWindowIn = 2.0f;      // start extra-slow final approach inside this distance
float finalApproachSpeedInPerS = 2.0f;   // low speed near the stop to avoid exciting the suspended load
float stopDampingGain = 0.35f;           // subtract a bit of measured chassis velocity near target

// Inner loop: wheel speed -> motor command.
float kVelP = 24.0f;
float kVelI = 10.0f;
float velIntegralLimit = 250.0f;

// Slip monitor: compare wheel estimate against real rail estimate.
float slipWarnThresholdIn = 0.75f;
float slipSlowThresholdIn = 1.50f;
float slipSlowScale = 0.55f;

enum ControlMode : uint8_t {
  MODE_IDLE = 0,
  MODE_MANUAL,
  MODE_POSITION_HOLD
};

struct DriveSnapshot {
  float drivePosIn;
  float realPosIn;
  float driveVelInPerS;
  float realVelInPerS;
};

// ===== Runtime state =====
long realPosTotalCounts = 0;
int realPosLastRaw = 0;

ControlMode controlMode = MODE_IDLE;
int manualMotorCmd = 0;
int appliedMotorCmd = 0;
float targetPositionIn = 0.0f;
float targetWheelSpeedInPerS = 0.0f;
float velocityIntegral = 0.0f;

DriveSnapshot currentState = {0.0f, 0.0f, 0.0f, 0.0f};
DriveSnapshot previousState = {0.0f, 0.0f, 0.0f, 0.0f};

uint32_t lastControlMs = 0;
uint32_t lastPrintMs = 0;
uint32_t targetSettledSinceMs = 0;
bool forceStatusStreaming = false;

char serialBuf[64];
uint8_t serialIdx = 0;

// ===== Prototypes =====
bool selectMuxChannel(uint8_t channel);
int readAs5600RawAngle(uint8_t muxChannel);
long updateRealPositionCounts();
void refreshSensors(float dtSeconds);
void resetDriveState();

float getDriveEncoderDistanceIn();
float getRealPositionDistanceIn();
float getSlipDistanceIn();

void applyMotorCommand(int speedCmd);
void stopDrive();
void startManualMode(int speedCmd);
void startPositionMove(float targetIn);
float computeProfiledTargetSpeed(float positionErrorIn, float dtSeconds);
void updateControl(float dtSeconds);
bool targetReached();
bool withinTargetSettleWindow();

void handleSerialInput();
void processCommand(char *cmd);
void printHelp();
void printStatus();
bool shouldPrintStatusStream();

bool selectMuxChannel(uint8_t channel) {
  if (channel > 7) return false;

  Wire.beginTransmission(I2C_MUX_ADDR);
  Wire.write(1 << channel);
  return Wire.endTransmission() == 0;
}

int readAs5600RawAngle(uint8_t muxChannel) {
  if (!selectMuxChannel(muxChannel)) return -1;

  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_RAW_ANGLE_REG);
  if (Wire.endTransmission(false) != 0) return -1;

  const uint8_t requestedBytes = 2;
  uint8_t received = Wire.requestFrom((int)AS5600_ADDR, (int)requestedBytes);
  if (received != requestedBytes) return -1;

  int rawAngle = ((int)Wire.read() << 8) | Wire.read();
  return rawAngle & 0x0FFF;
}

long updateRealPositionCounts() {
  int currentRaw = readAs5600RawAngle(REAL_POSITION_MUX_CH);
  if (currentRaw < 0) return realPosTotalCounts;

  int delta = currentRaw - realPosLastRaw;

  if (delta > AS5600_WRAP_THRESH) {
    delta -= AS5600_COUNTS_PER_REV;
  } else if (delta < -AS5600_WRAP_THRESH) {
    delta += AS5600_COUNTS_PER_REV;
  }

  if (abs(delta) >= AS5600_NOISE_THRESH) {
    realPosTotalCounts += delta;
    realPosLastRaw = currentRaw;
  }

  return realPosTotalCounts;
}

float getDriveEncoderDistanceIn() {
  long counts = driveEncoder.read();
  float wheelRevs = ((float)counts) / (DRIVE_GEAR_RATIO * DRIVE_ENCODER_COUNTS_PER_MOTOR_REV);
  return wheelRevs * (2.0f * PI * DRIVE_WHEEL_RADIUS_IN);
}

float getRealPositionDistanceIn() {
  return ((float)realPosTotalCounts / (float)AS5600_COUNTS_PER_REV) * REAL_POSITION_WHEEL_CIRCUMFERENCE_IN;
}

float getSlipDistanceIn() {
  return currentState.drivePosIn - currentState.realPosIn;
}

void refreshSensors(float dtSeconds) {
  updateRealPositionCounts();

  previousState = currentState;
  currentState.drivePosIn = getDriveEncoderDistanceIn();
  currentState.realPosIn = getRealPositionDistanceIn();

  if (dtSeconds <= 0.0f) {
    currentState.driveVelInPerS = 0.0f;
    currentState.realVelInPerS = 0.0f;
    return;
  }

  currentState.driveVelInPerS = (currentState.drivePosIn - previousState.drivePosIn) / dtSeconds;
  currentState.realVelInPerS = (currentState.realPosIn - previousState.realPosIn) / dtSeconds;
}

void applyMotorCommand(int speedCmd) {
  appliedMotorCmd = constrain(speedCmd, -MAX_MOTOR_CMD, MAX_MOTOR_CMD);
  md.setM1Speed(appliedMotorCmd);
}

void stopDrive() {
  controlMode = MODE_IDLE;
  manualMotorCmd = 0;
  targetWheelSpeedInPerS = 0.0f;
  velocityIntegral = 0.0f;
  targetSettledSinceMs = 0;
  applyMotorCommand(0);
}

void resetDriveState() {
  applyMotorCommand(0);
  driveEncoder.write(0);
  realPosTotalCounts = 0;

  int raw = readAs5600RawAngle(REAL_POSITION_MUX_CH);
  realPosLastRaw = (raw >= 0) ? raw : 0;

  velocityIntegral = 0.0f;
  targetWheelSpeedInPerS = 0.0f;
  targetSettledSinceMs = 0;
  currentState = {0.0f, 0.0f, 0.0f, 0.0f};
  previousState = currentState;
}

void startManualMode(int speedCmd) {
  controlMode = MODE_MANUAL;
  manualMotorCmd = constrain(speedCmd, -MAX_MOTOR_CMD, MAX_MOTOR_CMD);
  velocityIntegral = 0.0f;
  targetWheelSpeedInPerS = 0.0f;
  targetSettledSinceMs = 0;
  applyMotorCommand(manualMotorCmd);
}

void startPositionMove(float targetIn) {
  controlMode = MODE_POSITION_HOLD;
  targetPositionIn = targetIn;
  velocityIntegral = 0.0f;
  targetWheelSpeedInPerS = 0.0f;
  targetSettledSinceMs = 0;
}

bool withinTargetSettleWindow() {
  float positionError = targetPositionIn - currentState.realPosIn;
  return fabs(positionError) <= TARGET_TOLERANCE_IN &&
         fabs(currentState.realVelInPerS) <= STOP_VELOCITY_TOLERANCE_IN_S;
}

bool targetReached() {
  if (!withinTargetSettleWindow()) {
    targetSettledSinceMs = 0;
    return false;
  }

  uint32_t nowMs = millis();
  if (targetSettledSinceMs == 0) {
    targetSettledSinceMs = nowMs;
    return false;
  }

  return (nowMs - targetSettledSinceMs) >= TARGET_SETTLE_TIME_MS;
}

float computeProfiledTargetSpeed(float positionErrorIn, float dtSeconds) {
  if (fabs(positionErrorIn) <= TARGET_TOLERANCE_IN) return 0.0f;

  float requestedCruise = constrain(steadyCruiseSpeedInPerS, 0.0f, MAX_CRUISE_SPEED_IN_PER_S);
  float brakingSpeed = sqrtf(2.0f * slowdownDecelInPerS2 * fabs(positionErrorIn));
  float desiredSpeed = min(requestedCruise, brakingSpeed);

  if (fabs(positionErrorIn) <= finalApproachWindowIn) {
    desiredSpeed = min(desiredSpeed, finalApproachSpeedInPerS);
  }

  desiredSpeed = (positionErrorIn >= 0.0f) ? desiredSpeed : -desiredSpeed;

  if (fabs(positionErrorIn) <= finalApproachWindowIn) {
    desiredSpeed -= stopDampingGain * currentState.realVelInPerS;
  }

  if (dtSeconds <= 0.0f) return desiredSpeed;

  float maxDeltaSpeed = max(0.01f, rampUpAccelInPerS2 * dtSeconds);
  float speedDelta = desiredSpeed - targetWheelSpeedInPerS;
  speedDelta = constrain(speedDelta, -maxDeltaSpeed, maxDeltaSpeed);
  float profiledSpeed = targetWheelSpeedInPerS + speedDelta;

  return profiledSpeed;
}

void updateControl(float dtSeconds) {
  if (controlMode == MODE_IDLE) {
    applyMotorCommand(0);
    return;
  }

  if (controlMode == MODE_MANUAL) {
    applyMotorCommand(manualMotorCmd);
    return;
  }

  float positionError = targetPositionIn - currentState.realPosIn;
  targetWheelSpeedInPerS = computeProfiledTargetSpeed(positionError, dtSeconds);

  if (targetReached()) {
    stopDrive();
    Serial.println("Target reached.");
    return;
  }

  float slipMagnitude = fabs(getSlipDistanceIn());
  if (slipMagnitude >= slipSlowThresholdIn) {
    targetWheelSpeedInPerS *= slipSlowScale;
  }

  float velocityError = targetWheelSpeedInPerS - currentState.driveVelInPerS;
  bool inFinalApproach = fabs(positionError) <= finalApproachWindowIn;
  if (!inFinalApproach) {
    velocityIntegral += velocityError * dtSeconds;
    velocityIntegral = constrain(velocityIntegral, -velIntegralLimit, velIntegralLimit);
  } else {
    velocityIntegral *= 0.85f;
  }

  float cmd = (kVelP * velocityError) + (kVelI * velocityIntegral);

  if (fabs(targetWheelSpeedInPerS) < 0.05f) {
    velocityIntegral = 0.0f;
  }

  applyMotorCommand((int)lroundf(cmd));
}

void printHelp() {
  Serial.println("Drive + control test");
  Serial.println("Commands:");
  Serial.println("  h              -> print this help");
  Serial.println("  z              -> stop and zero both encoders at the current position");
  Serial.println("  x              -> stop motor and exit control mode");
  Serial.println("  p              -> print one status line");
  Serial.println("  stream [on|off]-> force periodic status printing on or off");
  Serial.println("  m <cmd>        -> manual motor command (-400 to 400)");
  Serial.println("  g <inches>     -> move to rail position using cruise + slowdown profile");
  Serial.println("  ss <ft/s>      -> set requested steady speed (max 2.0 ft/s)");
  Serial.println("  accel <in/s^2> -> set ramp-up acceleration limit");
  Serial.println("  decel <in/s^2> -> set slowdown deceleration");
  Serial.println("  approach <window_in> <speed_in_s> -> set final approach slowdown");
  Serial.println("  damp <gain>    -> set stop damping gain from measured real velocity");
  Serial.println("  kvp <value>    -> set inner velocity P gain");
  Serial.println("  kvi <value>    -> set inner velocity I gain");
  Serial.println("  slip <warn> <slow> -> set slip warning / slowdown thresholds in inches");
  Serial.println("Status units: pos=in, vel=in/s, accel=in/s^2, cruise=ft/s");
}

bool shouldPrintStatusStream() {
  return forceStatusStreaming || (appliedMotorCmd != 0);
}

void printStatus() {
  float slipIn = getSlipDistanceIn();

  Serial.print("Mode: ");
  Serial.print((int)controlMode);
  Serial.print(" | MotorCmd: ");
  Serial.print(appliedMotorCmd);
  Serial.print(" | TargetPos(in): ");
  Serial.print(targetPositionIn, 2);
  Serial.print(" | Cruise(ft/s): ");
  Serial.print(steadyCruiseSpeedInPerS / 12.0f, 3);
  Serial.print(" | Accel(in/s^2): ");
  Serial.print(rampUpAccelInPerS2, 2);
  Serial.print(" | FinalWin(in): ");
  Serial.print(finalApproachWindowIn, 2);
  Serial.print(" | FinalVel(in/s): ");
  Serial.print(finalApproachSpeedInPerS, 2);
  Serial.print(" | RealPos(in): ");
  Serial.print(currentState.realPosIn, 3);
  Serial.print(" | DrivePos(in): ");
  Serial.print(currentState.drivePosIn, 3);
  Serial.print(" | RealVel(in/s): ");
  Serial.print(currentState.realVelInPerS, 3);
  Serial.print(" | DriveVel(in/s): ");
  Serial.print(currentState.driveVelInPerS, 3);
  Serial.print(" | TargetVel(in/s): ");
  Serial.print(targetWheelSpeedInPerS, 3);
  Serial.print(" | Slip(in): ");
  Serial.print(slipIn, 3);
  if (withinTargetSettleWindow()) {
    Serial.print(" | SETTLING");
  }
  if (fabs(slipIn) >= slipWarnThresholdIn) {
    Serial.print(" | SLIP");
  }
  Serial.println();
}

void processCommand(char *cmd) {
  if (cmd[0] == '\0') return;

  char *verb = strtok(cmd, " ");
  if (verb == nullptr) return;

  if (strcmp(verb, "h") == 0) {
    printHelp();
    return;
  }

  if (strcmp(verb, "z") == 0) {
    stopDrive();
    resetDriveState();
    Serial.println("Encoders reset.");
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
    char *arg = strtok(nullptr, " ");
    if (arg == nullptr) {
      forceStatusStreaming = !forceStatusStreaming;
    } else if (strcmp(arg, "on") == 0) {
      forceStatusStreaming = true;
    } else if (strcmp(arg, "off") == 0) {
      forceStatusStreaming = false;
    } else {
      Serial.println("Usage: stream [on|off]");
      return;
    }

    Serial.print("Periodic status streaming ");
    Serial.println(forceStatusStreaming ? "ON" : "AUTO");
    if (forceStatusStreaming) {
      printStatus();
    }
    return;
  }

  if (strcmp(verb, "m") == 0) {
    char *arg = strtok(nullptr, " ");
    if (arg == nullptr) {
      Serial.println("Usage: m <cmd>");
      return;
    }

    startManualMode(atoi(arg));
    Serial.print("Manual motor command set to ");
    Serial.println(manualMotorCmd);
    return;
  }

  if (strcmp(verb, "g") == 0) {
    char *arg = strtok(nullptr, " ");
    if (arg == nullptr) {
      Serial.println("Usage: g <inches>");
      return;
    }

    startPositionMove((float)atof(arg));
    Serial.print("Position target set to ");
    Serial.print(targetPositionIn, 2);
    Serial.println(" in");
    return;
  }

  if (strcmp(verb, "ss") == 0) {
    char *arg = strtok(nullptr, " ");
    if (arg == nullptr) {
      Serial.println("Usage: ss <ft/s>");
      return;
    }

    float requestedFtPerS = (float)atof(arg);
    steadyCruiseSpeedInPerS = constrain(requestedFtPerS * 12.0f, 0.0f, MAX_CRUISE_SPEED_IN_PER_S);
    Serial.print("steadyCruiseSpeed = ");
    Serial.print(steadyCruiseSpeedInPerS / 12.0f, 3);
    Serial.println(" ft/s");
    return;
  }

  if (strcmp(verb, "accel") == 0) {
    char *arg = strtok(nullptr, " ");
    if (arg == nullptr) {
      Serial.println("Usage: accel <in/s^2>");
      return;
    }

    rampUpAccelInPerS2 = max(1.0f, (float)atof(arg));
    Serial.print("rampUpAccelInPerS2 = ");
    Serial.println(rampUpAccelInPerS2, 3);
    return;
  }

  if (strcmp(verb, "decel") == 0) {
    char *arg = strtok(nullptr, " ");
    if (arg == nullptr) {
      Serial.println("Usage: decel <in/s^2>");
      return;
    }

    slowdownDecelInPerS2 = max(1.0f, (float)atof(arg));
    Serial.print("slowdownDecelInPerS2 = ");
    Serial.println(slowdownDecelInPerS2, 3);
    return;
  }

  if (strcmp(verb, "approach") == 0) {
    char *windowArg = strtok(nullptr, " ");
    char *speedArg = strtok(nullptr, " ");
    if (windowArg == nullptr || speedArg == nullptr) {
      Serial.println("Usage: approach <window_in> <speed_in_s>");
      return;
    }

    finalApproachWindowIn = max(TARGET_TOLERANCE_IN * 2.0f, (float)atof(windowArg));
    finalApproachSpeedInPerS = max(0.1f, (float)atof(speedArg));
    Serial.print("Final approach window/speed = ");
    Serial.print(finalApproachWindowIn, 3);
    Serial.print(" in / ");
    Serial.print(finalApproachSpeedInPerS, 3);
    Serial.println(" in/s");
    return;
  }

  if (strcmp(verb, "damp") == 0) {
    char *arg = strtok(nullptr, " ");
    if (arg == nullptr) {
      Serial.println("Usage: damp <gain>");
      return;
    }

    stopDampingGain = max(0.0f, (float)atof(arg));
    Serial.print("stopDampingGain = ");
    Serial.println(stopDampingGain, 4);
    return;
  }

  if (strcmp(verb, "kvp") == 0) {
    char *arg = strtok(nullptr, " ");
    if (arg == nullptr) {
      Serial.println("Usage: kvp <value>");
      return;
    }

    kVelP = (float)atof(arg);
    Serial.print("kVelP = ");
    Serial.println(kVelP, 4);
    return;
  }

  if (strcmp(verb, "kvi") == 0) {
    char *arg = strtok(nullptr, " ");
    if (arg == nullptr) {
      Serial.println("Usage: kvi <value>");
      return;
    }

    kVelI = (float)atof(arg);
    Serial.print("kVelI = ");
    Serial.println(kVelI, 4);
    return;
  }

  if (strcmp(verb, "slip") == 0) {
    char *warnArg = strtok(nullptr, " ");
    char *slowArg = strtok(nullptr, " ");
    if (warnArg == nullptr || slowArg == nullptr) {
      Serial.println("Usage: slip <warn> <slow>");
      return;
    }

    slipWarnThresholdIn = (float)atof(warnArg);
    slipSlowThresholdIn = (float)atof(slowArg);
    Serial.print("Slip warn/slow = ");
    Serial.print(slipWarnThresholdIn, 3);
    Serial.print(" / ");
    Serial.println(slipSlowThresholdIn, 3);
    return;
  }

  Serial.println("Unknown command. Enter h for help.");
}

void handleSerialInput() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\r') continue;

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

  resetDriveState();
  lastControlMs = millis();
  lastPrintMs = millis();

  printHelp();
}

void loop() {
  uint32_t nowMs = millis();
  handleSerialInput();

  if (md.getM1Fault()) {
    stopDrive();
    Serial.println("Motor fault on M1. Drive stopped.");
    delay(50);
    return;
  }

  if (nowMs - lastControlMs >= CONTROL_INTERVAL_MS) {
    float dtSeconds = (float)(nowMs - lastControlMs) / 1000.0f;
    lastControlMs = nowMs;

    refreshSensors(dtSeconds);
    updateControl(dtSeconds);
  }

  if (nowMs - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = nowMs;
    if (shouldPrintStatusStream()) {
      printStatus();
    }
  }
}
