#include <EncoderMux.h>

/*
  EncoderMux.cpp

  This file contains the shared implementation for:
  - selecting a TCA9548A mux channel
  - reading the AS5600 RAW ANGLE register
  - unwrapping the 12-bit encoder count across rollover points
  - maintaining a continuous running count for higher-level code

  DriveControl and WinchControl should treat this as a low-level utility layer.
  They can ask it for stable counts, then convert those counts into inches,
  depth, velocity, or homing state using subsystem-specific rules.
*/

#pragma region Includes

#include <stdlib.h>

#pragma endregion

namespace {

/*
  These helpers sanitize config values so the library behaves predictably even
  if a caller forgets to fill in every field.

  The defaults match the hardware usage already present in the project:
  - TCA9548A mux address 0x70
  - AS5600 encoder address 0x36
  - RAW ANGLE register 0x0C
  - 4096 counts per revolution for the 12-bit AS5600
*/
TwoWire *getWireBus(const EncoderMuxConfig &config) {
  return (config.wireBus != NULL) ? config.wireBus : &Wire;
}

int getCountsPerRevolution(const EncoderMuxConfig &config) {
  return (config.countsPerRevolution > 0) ? config.countsPerRevolution : 4096;
}

int getWrapThreshold(const EncoderMuxConfig &config) {
  const int countsPerRevolution = getCountsPerRevolution(config);

  if (config.wrapThreshold > 0 && config.wrapThreshold < countsPerRevolution) {
    return config.wrapThreshold;
  }

  return countsPerRevolution / 2;
}

int getNoiseThreshold(const EncoderMuxConfig &config) {
  return (config.noiseThreshold >= 0) ? config.noiseThreshold : 0;
}

uint8_t getMuxAddress(const EncoderMuxConfig &config) {
  return (config.muxAddress != 0U) ? config.muxAddress : 0x70;
}

uint8_t getSensorAddress(const EncoderMuxConfig &config) {
  return (config.sensorAddress != 0U) ? config.sensorAddress : 0x36;
}

uint8_t getRawAngleRegister(const EncoderMuxConfig &config) {
  return (config.rawAngleRegister != 0U) ? config.rawAngleRegister : 0x0C;
}

}  // namespace

#pragma region Config_Helpers

EncoderMuxConfig makeDefaultAs5600MuxConfig(uint8_t muxChannel) {
  EncoderMuxConfig config;

  config.wireBus = &Wire;
  config.muxAddress = 0x70;
  config.sensorAddress = 0x36;
  config.rawAngleRegister = 0x0C;
  config.muxChannel = muxChannel;
  config.countsPerRevolution = 4096;
  config.wrapThreshold = config.countsPerRevolution / 2;
  config.noiseThreshold = 1;

  return config;
}

#pragma endregion

#pragma region Reset_And_State_Functions

void resetEncoderState(EncoderMuxState &state, long newZeroCounts) {
  /*
    Resetting clears all runtime history so the next successful encoder read
    becomes the new reference point for unwrap calculations.

    newZeroCounts lets the caller decide what "zero" means. Most code will pass
    0, but a subsystem could also choose to align the encoder to another known
    count if needed later.
  */
  state.totalCounts = newZeroCounts;
  state.lastRawCount = 0;
  state.latestRawCount = 0;
  state.lastDeltaCounts = 0;
  state.hasValidReference = false;
  state.lastReadSucceeded = false;
}

bool initializeEncoderState(EncoderMuxState &state, const EncoderMuxConfig &config) {
  const int currentRawCount = readAs5600RawAngle(config);

  if (currentRawCount < 0) {
    state.lastReadSucceeded = false;
    return false;
  }

  /*
    The first successful read is not treated as movement. It is only used to
    establish the starting point that later readings will be compared against.
  */
  state.lastRawCount = currentRawCount;
  state.latestRawCount = currentRawCount;
  state.lastDeltaCounts = 0;
  state.hasValidReference = true;
  state.lastReadSucceeded = true;
  return true;
}

bool zeroEncoderToCurrentRaw(EncoderMuxState &state,
                             const EncoderMuxConfig &config,
                             long newZeroCounts) {
  /*
    Start by clearing the software count. If the live encoder read succeeds we
    also capture a matching raw reference immediately. If it fails, the state is
    still safely reset and will recover on a later successful update.
  */
  resetEncoderState(state, newZeroCounts);
  return initializeEncoderState(state, config);
}

#pragma endregion

#pragma region Low_Level_I2C_Functions

bool selectMuxChannel(const EncoderMuxConfig &config) {
  /*
    The TCA9548A uses one control byte where each bit represents a channel.
    Writing 1 << channel turns on exactly one downstream path.
  */
  if (config.muxChannel > 7U) {
    return false;
  }

  TwoWire *wireBus = getWireBus(config);

  wireBus->beginTransmission(getMuxAddress(config));
  wireBus->write((uint8_t)(1U << config.muxChannel));
  return wireBus->endTransmission() == 0;
}

int readAs5600RawAngle(const EncoderMuxConfig &config) {
  if (!selectMuxChannel(config)) {
    return -1;
  }

  TwoWire *wireBus = getWireBus(config);
  const uint8_t sensorAddress = getSensorAddress(config);
  const uint8_t rawAngleRegister = getRawAngleRegister(config);

  /*
    First tell the AS5600 which register we want to read. The repeated-start
    form of endTransmission(false) keeps control of the bus so the subsequent
    requestFrom call can immediately fetch the data bytes.
  */
  wireBus->beginTransmission(sensorAddress);
  wireBus->write(rawAngleRegister);
  if (wireBus->endTransmission(false) != 0) {
    return -1;
  }

  const uint8_t requestedBytes = 2;
  const uint8_t receivedBytes =
      wireBus->requestFrom((int)sensorAddress, (int)requestedBytes);

  if (receivedBytes != requestedBytes) {
    return -1;
  }

  /*
    The AS5600 returns a 12-bit angle split across two bytes.
    - first byte: high bits
    - second byte: low bits

    The final mask keeps only the lower 12 bits so the return value is always
    in the expected 0-4095 range.
  */
  const int rawAngle = ((int)wireBus->read() << 8) | wireBus->read();
  return rawAngle & 0x0FFF;
}

#pragma endregion

#pragma region Continuous_Count_Update_Functions

int unwrapEncoderDelta(const EncoderMuxConfig &config,
                       int currentRawCount,
                       int previousRawCount) {
  int deltaCounts = currentRawCount - previousRawCount;

  /*
    Without this correction, moving across the 0/4095 rollover would look like
    a huge jump in the wrong direction.

    Example forward wrap:
    - previous = 4090
    - current = 5
    - direct delta = -4085
    - corrected delta = +11
  */
  if (deltaCounts > getWrapThreshold(config)) {
    deltaCounts -= getCountsPerRevolution(config);
  } else if (deltaCounts < -getWrapThreshold(config)) {
    deltaCounts += getCountsPerRevolution(config);
  }

  return deltaCounts;
}

long updateEncoderCounts(EncoderMuxState &state, const EncoderMuxConfig &config) {
  const int currentRawCount = readAs5600RawAngle(config);

  if (currentRawCount < 0) {
    /*
      Failed reads should not corrupt position tracking. The safest option is to
      leave the accumulated total unchanged and simply report the failure.
    */
    state.lastReadSucceeded = false;
    state.lastDeltaCounts = 0;
    return state.totalCounts;
  }

  state.latestRawCount = currentRawCount;
  state.lastReadSucceeded = true;

  /*
    If the caller did not explicitly initialize the encoder yet, the first good
    read becomes the reference point and no movement is reported on that cycle.
    This makes the update helper safe to call immediately after reset.
  */
  if (!state.hasValidReference) {
    state.lastRawCount = currentRawCount;
    state.lastDeltaCounts = 0;
    state.hasValidReference = true;
    return state.totalCounts;
  }

  const int deltaCounts =
      unwrapEncoderDelta(config, currentRawCount, state.lastRawCount);
  state.lastDeltaCounts = deltaCounts;

  /*
    The noise gate rejects tiny changes that are likely caused by sensor jitter.
    If a delta is rejected, lastRawCount is intentionally left unchanged so
    several small movements can still build into one accepted larger movement.
  */
  if (abs(deltaCounts) >= getNoiseThreshold(config)) {
    state.totalCounts += deltaCounts;
    state.lastRawCount = currentRawCount;
  }

  return state.totalCounts;
}

#pragma endregion
