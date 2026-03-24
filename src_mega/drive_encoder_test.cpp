#include <Arduino.h>
#include <Encoder.h>
#include <Wire.h>
#include "DualG2HighPowerMotorShield.h"

// ===== Hardware configuration =====
DualG2HighPowerMotorShield24v14 md;

const uint8_t DRIVE_ENCODER_PIN_A = 19;
const uint8_t DRIVE_ENCODER_PIN_B = 18;

const uint8_t I2C_MUX_ADDR = 0x70;
const uint8_t AS5600_ADDR = 0x36;
const uint8_t AS5600_RAW_ANGLE_REG = 0x0C;
const uint8_t REAL_POSITION_MUX_CH = 5;

// Set this if positive speed commands drive the wrong direction.
const bool FLIP_DRIVE_MOTOR = false;

// ===== Calibration constants =====
const float DRIVE_ENCODER_COUNTS_PER_MOTOR_REV = 1200.0f; // From proof_of_concept.cpp
const float DRIVE_GEAR_RATIO = 0.5f;                      // From proof_of_concept.cpp
const float DRIVE_WHEEL_RADIUS_IN = 0.6f;                 // From proof_of_concept.cpp

const float REAL_POSITION_WHEEL_DIAMETER_IN = 0.875f;
const float REAL_POSITION_WHEEL_CIRCUMFERENCE_IN = PI * REAL_POSITION_WHEEL_DIAMETER_IN;
const int AS5600_COUNTS_PER_REV = 4096;
const int AS5600_WRAP_THRESH = AS5600_COUNTS_PER_REV / 2;
const int AS5600_NOISE_THRESH = 1;

const uint32_t PRINT_INTERVAL_MS = 200;

// ===== Runtime state =====
Encoder driveEncoder(DRIVE_ENCODER_PIN_A, DRIVE_ENCODER_PIN_B);

long realPosTotalCounts = 0;
int realPosLastRaw = 0;

int commandedSpeed = 0;
uint32_t lastPrintMs = 0;

char serialBuf[32];
uint8_t serialIdx = 0;

// ===== Prototypes =====
bool selectMuxChannel(uint8_t channel);
int readAs5600RawAngle(uint8_t muxChannel);
long updateRealPositionCounts();
void resetAllEncoders();
float getDriveEncoderDistanceIn();
float getRealPositionDistanceIn();
float getAbsoluteErrorIn();
float getPercentError(float referenceDistanceIn, float measuredDistanceIn);
void applyMotorCommand(int speedCmd);
void printStatus();
void printHelp();
void handleSerialInput();
void processCommand(const char *cmd);

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

void resetAllEncoders() {
  driveEncoder.write(0);
  realPosTotalCounts = 0;

  int raw = readAs5600RawAngle(REAL_POSITION_MUX_CH);
  realPosLastRaw = (raw >= 0) ? raw : 0;
}

float getDriveEncoderDistanceIn() {
  long counts = driveEncoder.read();
  float wheelRevs = ((float)counts) / (DRIVE_GEAR_RATIO * DRIVE_ENCODER_COUNTS_PER_MOTOR_REV);
  return wheelRevs * (2.0f * PI * DRIVE_WHEEL_RADIUS_IN);
}

float getRealPositionDistanceIn() {
  return ((float)realPosTotalCounts / (float)AS5600_COUNTS_PER_REV) * REAL_POSITION_WHEEL_CIRCUMFERENCE_IN;
}

float getAbsoluteErrorIn() {
  return getDriveEncoderDistanceIn() - getRealPositionDistanceIn();
}

float getPercentError(float referenceDistanceIn, float measuredDistanceIn) {
  if (fabs(referenceDistanceIn) < 0.01f) return 0.0f;
  return ((measuredDistanceIn - referenceDistanceIn) / referenceDistanceIn) * 100.0f;
}

void applyMotorCommand(int speedCmd) {
  speedCmd = constrain(speedCmd, -400, 400);
  commandedSpeed = speedCmd;
  md.setM1Speed(commandedSpeed);
}

void printStatus() {
  float driveDistanceIn = getDriveEncoderDistanceIn();
  float realDistanceIn = getRealPositionDistanceIn();
  float errorIn = driveDistanceIn - realDistanceIn;
  float percentError = getPercentError(realDistanceIn, driveDistanceIn);

  Serial.print("Cmd: ");
  Serial.print(commandedSpeed);
  Serial.print(" | Drive Counts: ");
  Serial.print(driveEncoder.read());
  Serial.print(" | Real Counts: ");
  Serial.print(realPosTotalCounts);
  Serial.print(" | Drive Dist (in): ");
  Serial.print(driveDistanceIn, 3);
  Serial.print(" | Real Dist (in): ");
  Serial.print(realDistanceIn, 3);
  Serial.print(" | Error (in): ");
  Serial.print(errorIn, 3);
  Serial.print(" | Error (%): ");
  Serial.println(percentError, 2);
}

void printHelp() {
  Serial.println("Drive encoder accuracy test");
  Serial.println("Commands:");
  Serial.println("  h   -> help");
  Serial.println("  z   -> zero both encoders");
  Serial.println("  x   -> stop motor");
  Serial.println("  p   -> print one status line");
  Serial.println("  any integer from -400 to 400 -> set drive motor speed");
}

void processCommand(const char *cmd) {
  if (cmd[0] == '\0') return;

  if (strcmp(cmd, "h") == 0) {
    printHelp();
    return;
  }

  if (strcmp(cmd, "z") == 0) {
    applyMotorCommand(0);
    resetAllEncoders();
    Serial.println("Encoders reset.");
    return;
  }

  if (strcmp(cmd, "x") == 0) {
    applyMotorCommand(0);
    Serial.println("Motor stopped.");
    return;
  }

  if (strcmp(cmd, "p") == 0) {
    printStatus();
    return;
  }

  int speedCmd = atoi(cmd);
  if (speedCmd < -400 || speedCmd > 400) {
    Serial.println("Invalid command. Use h, z, s, p, or a speed from -400 to 400.");
    return;
  }

  applyMotorCommand(speedCmd);
  Serial.print("Set motor speed to ");
  Serial.println(commandedSpeed);
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
  md.flipM2(FLIP_DRIVE_MOTOR);
  applyMotorCommand(0);

  resetAllEncoders();
  printHelp();
}

void loop() {
  updateRealPositionCounts();
  handleSerialInput();

  if (md.getM2Fault()) {
    applyMotorCommand(0);
    Serial.println("Drive motor fault detected. Motor stopped.");
    delay(250);
    return;
  }

  uint32_t nowMs = millis();
  if (nowMs - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = nowMs;
    printStatus();
  }
}
