#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>

/*
  Direct winch encoder I2C test

  Reads the winch AS5600 with Wire directly, tracks the unwrapped count across
  rollover, and converts that count to winch position using the same calibration
  values as new_winch_test.cpp / WinchControl.
*/

const uint8_t AS5600_ADDR = 0x36;
const uint8_t AS5600_RAW_ANGLE_REG = 0x0C;

const uint32_t WIRE_TIMEOUT_US = 25000;
const bool FLIP_WINCH_ENCODER_SIGN = true;
const int WINCH_ENCODER_COUNTS_PER_REV = 4096;
const int WINCH_WRAP_THRESHOLD = WINCH_ENCODER_COUNTS_PER_REV / 2;
const int WINCH_NOISE_THRESHOLD = 4;
const float WINCH_DRUM_RADIUS_IN = (1.502f / 2.0f) + (1.0f / 32.0f);

const uint32_t PRINT_INTERVAL_MS = 200;

long winchTotalCounts = 0;
int winchLastRawCount = 0;
int winchLatestRawCount = 0;
int winchLastDeltaCounts = 0;
bool winchHasValidReference = false;
bool winchLastReadSucceeded = false;
uint32_t lastPrintMs = 0;

int readWinchRawAngle();
long updateWinchCounts();
void resetWinchEncoderToCurrentPosition(float newZeroInches = 0.0f);
float convertWinchCountsToInches(long encoderCounts);
long convertWinchInchesToCounts(float positionIn);
void printStatus();
void printHelp();
void handleSerial();

int readWinchRawAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_RAW_ANGLE_REG);
  if (Wire.endTransmission(false) != 0) {
    return -1;
  }

  const uint8_t requestedBytes = 2;
  const uint8_t receivedBytes =
      Wire.requestFrom((int)AS5600_ADDR, (int)requestedBytes);

  if (receivedBytes != requestedBytes) {
    return -1;
  }

  const int rawAngle = ((int)Wire.read() << 8) | Wire.read();
  return rawAngle & 0x0FFF;
}

long updateWinchCounts() {
  const int currentRawCount = readWinchRawAngle();

  if (currentRawCount < 0) {
    winchLastReadSucceeded = false;
    winchLastDeltaCounts = 0;
    return winchTotalCounts;
  }

  winchLatestRawCount = currentRawCount;
  winchLastReadSucceeded = true;

  if (!winchHasValidReference) {
    winchLastRawCount = currentRawCount;
    winchLastDeltaCounts = 0;
    winchHasValidReference = true;
    return winchTotalCounts;
  }

  int deltaCounts = currentRawCount - winchLastRawCount;

  if (deltaCounts > WINCH_WRAP_THRESHOLD) {
    deltaCounts -= WINCH_ENCODER_COUNTS_PER_REV;
  } else if (deltaCounts < -WINCH_WRAP_THRESHOLD) {
    deltaCounts += WINCH_ENCODER_COUNTS_PER_REV;
  }

  winchLastDeltaCounts = deltaCounts;

  if (abs(deltaCounts) >= WINCH_NOISE_THRESHOLD) {
    winchTotalCounts += deltaCounts;
    winchLastRawCount = currentRawCount;
  }

  return winchTotalCounts;
}

void resetWinchEncoderToCurrentPosition(float newZeroInches) {
  winchTotalCounts = convertWinchInchesToCounts(newZeroInches);
  winchLastRawCount = 0;
  winchLatestRawCount = 0;
  winchLastDeltaCounts = 0;
  winchHasValidReference = false;
  winchLastReadSucceeded = false;

  const int currentRawCount = readWinchRawAngle();
  if (currentRawCount >= 0) {
    winchLastRawCount = currentRawCount;
    winchLatestRawCount = currentRawCount;
    winchLastReadSucceeded = true;
    winchHasValidReference = true;
  }
}

float convertWinchCountsToInches(long encoderCounts) {
  const float circumferenceIn = 2.0f * PI * fabsf(WINCH_DRUM_RADIUS_IN);

  if (WINCH_ENCODER_COUNTS_PER_REV <= 0 || circumferenceIn <= 0.0f) {
    return 0.0f;
  }

  const float signedEncoderCounts = FLIP_WINCH_ENCODER_SIGN
                                        ? -(float)encoderCounts
                                        : (float)encoderCounts;
  const float shaftRevolutions =
      signedEncoderCounts / (float)WINCH_ENCODER_COUNTS_PER_REV;

  return shaftRevolutions * circumferenceIn;
}

long convertWinchInchesToCounts(float positionIn) {
  const float circumferenceIn = 2.0f * PI * fabsf(WINCH_DRUM_RADIUS_IN);

  if (WINCH_ENCODER_COUNTS_PER_REV <= 0 || circumferenceIn <= 0.0f) {
    return 0;
  }

  const float signedPositionIn =
      FLIP_WINCH_ENCODER_SIGN ? -positionIn : positionIn;

  return lroundf((signedPositionIn / circumferenceIn) *
                 (float)WINCH_ENCODER_COUNTS_PER_REV);
}

void printStatus() {
  Serial.print("Raw: ");
  Serial.print(winchLatestRawCount);
  Serial.print(" | Delta: ");
  Serial.print(winchLastDeltaCounts);
  Serial.print(" | Counts: ");
  Serial.print(winchTotalCounts);
  Serial.print(" | Position (in): ");
  Serial.print(convertWinchCountsToInches(winchTotalCounts), 4);

  if (!winchLastReadSucceeded) {
    Serial.print(" | ENCODER_READ_FAIL");
  }

  Serial.print(" | Direct I2C");

  Serial.println();
}

void printHelp() {
  Serial.println("Direct winch encoder I2C test");
  Serial.println("Read path: direct AS5600 I2C. The multiplexer is bypassed.");
  Serial.println("Commands:");
  Serial.println("  h = print this help");
  Serial.println("  p = print one status line");
  Serial.println("  z = zero the current encoder position");
}

void handleSerial() {
  while (Serial.available() > 0) {
    const char command = (char)Serial.read();

    if (command == '\r' || command == '\n') {
      continue;
    }

    if (command == 'h' || command == '?') {
      printHelp();
    } else if (command == 'p') {
      printStatus();
    } else if (command == 'z') {
      resetWinchEncoderToCurrentPosition(0.0f);
      Serial.println(winchLastReadSucceeded
                         ? "Winch encoder reset to zero."
                         : "Winch encoder reset failed.");
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setWireTimeout(WIRE_TIMEOUT_US, true);

  printHelp();

  resetWinchEncoderToCurrentPosition(0.0f);

  if (!winchLastReadSucceeded) {
    Serial.println("Initial encoder read failed. Check AS5600 wiring/address.");
  }
}

void loop() {
  updateWinchCounts();
  handleSerial();

  const uint32_t nowMs = millis();
  if ((nowMs - lastPrintMs) >= PRINT_INTERVAL_MS) {
    lastPrintMs = nowMs;
    printStatus();
  }
}
