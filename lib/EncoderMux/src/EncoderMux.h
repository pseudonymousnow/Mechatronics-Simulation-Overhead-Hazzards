#pragma once

/*
  EncoderMux.h

  This library holds the shared low-level code for reading AS5600 magnetic
  encoders through a TCA9548A I2C multiplexer.

  The goal is to keep the "hardware conversation" in one reusable place:
  - select the correct mux channel
  - read the encoder's 12-bit raw angle value
  - unwrap the encoder count across rollover points
  - maintain a continuous running count over many revolutions

  Higher-level libraries such as DriveControl and WinchControl should build on
  top of this by adding their own mechanical conversion math, homing logic,
  speed estimation, and control behavior.
*/

#pragma region Includes

#include <Arduino.h>
#include <Wire.h>

#pragma endregion

#pragma region Config_Structs

/*
  EncoderMuxConfig describes one AS5600 encoder connected through the I2C mux.

  A caller will usually create one of these per sensor. For example:
  - one config for the drive real-position encoder on mux channel 5
  - one config for the winch encoder on mux channel 7

  The default helper below fills in the common AS5600/TCA9548A values so the
  caller only needs to change fields that are different for a specific sensor.
*/
struct EncoderMuxConfig {
  // Pointer to the I2C bus used to talk to the mux and sensor.
  TwoWire *wireBus;

  // I2C address of the TCA9548A multiplexer.
  uint8_t muxAddress;

  // I2C address of the AS5600 sensor.
  uint8_t sensorAddress;

  // Register address for the AS5600 RAW ANGLE high byte.
  uint8_t rawAngleRegister;

  // Which mux channel this encoder is wired to. Valid range is 0 through 7.
  uint8_t muxChannel;

  // Number of encoder counts in one full revolution.
  int countsPerRevolution;

  /*
    If the raw count jumps by more than this amount between reads, treat it as
    a rollover event and unwrap it back into the expected small step.
  */
  int wrapThreshold;

  /*
    Ignore very small deltas below this threshold to reduce jitter from sensor
    noise. A value of 1 means "keep any change of 1 count or larger."
  */
  int noiseThreshold;
};

/*
  Build a config with the standard AS5600 + TCA9548A defaults used elsewhere
  in this project.

  The caller can take the returned struct and then override only the fields it
  cares about, such as mux channel or noise threshold.
*/
EncoderMuxConfig makeDefaultAs5600MuxConfig(uint8_t muxChannel);

#pragma endregion

#pragma region State_Structs

/*
  EncoderMuxState stores runtime information for one continuously tracked
  encoder.

  The AS5600 only gives a 0-4095 reading for the current shaft angle. This
  state struct keeps enough history to turn that limited reading into a running
  total that can increase or decrease across many full rotations.
*/
struct EncoderMuxState {
  // Running unwrapped encoder count accumulated over time.
  long totalCounts;

  // Last raw encoder reading that was accepted as the reference position.
  int lastRawCount;

  // Most recent raw reading, even if it was rejected by the noise gate.
  int latestRawCount;

  // Most recent delta after wrap handling, before any mechanical conversion.
  int lastDeltaCounts;

  // True once the state has captured an initial raw reading to compare against.
  bool hasValidReference;

  // True if the most recent I2C read completed successfully.
  bool lastReadSucceeded;
};

#pragma endregion

#pragma region Helper_Function_Declarations

/*
  Reset the runtime state back to a known software zero.

  This does not read the encoder. After this call, the next successful update
  will capture a fresh raw reference automatically.
*/
void resetEncoderState(EncoderMuxState &state, long newZeroCounts = 0);

/*
  Read the current raw angle once and store it as the reference point for
  future unwrap calculations.

  This is useful during setup or when a subsystem wants to zero itself against
  the current physical position.
*/
bool initializeEncoderState(EncoderMuxState &state, const EncoderMuxConfig &config);

/*
  Reset the running total and immediately zero the state against the encoder's
  current raw angle reading.

  Returns false if the encoder read fails. In that failure case the total count
  is still reset, but the state waits for a later successful update to capture
  a valid raw reference.
*/
bool zeroEncoderToCurrentRaw(EncoderMuxState &state,
                             const EncoderMuxConfig &config,
                             long newZeroCounts = 0);

/*
  Select the mux channel stored in the config.

  This is broken out as a public helper because some higher-level code may want
  to force channel selection before performing additional device-specific work.
*/
bool selectMuxChannel(const EncoderMuxConfig &config);

/*
  Read the AS5600 raw angle from the encoder described by the config.

  On success this returns a value in the range 0 to 4095.
  On failure this returns -1.
*/
int readAs5600RawAngle(const EncoderMuxConfig &config);

/*
  Apply wrap handling to the difference between two raw readings.

  Example:
  - previous = 4090
  - current = 5
  The direct subtraction gives -4085, but the real motion was only +11 counts.
  This helper converts that large wrapped jump back into the expected small
  signed motion.
*/
int unwrapEncoderDelta(const EncoderMuxConfig &config,
                       int currentRawCount,
                       int previousRawCount);

/*
  Read the encoder, unwrap the new delta, apply the noise filter, and update
  the running total.

  Safe behavior on read failure:
  - totalCounts is left unchanged
  - lastReadSucceeded becomes false
*/
long updateEncoderCounts(EncoderMuxState &state, const EncoderMuxConfig &config);

#pragma endregion
