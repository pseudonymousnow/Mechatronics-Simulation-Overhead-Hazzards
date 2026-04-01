#include <DriveControl.h>
#include <Arduino.h>
#include <EncoderMux.h>
#include <math.h>

/*
  DriveControl.cpp

  Put the reusable drive sensing and control implementation here.

  This file should eventually include:
  - Drive encoder conversion to linear distance
  - Real-position sensor conversion using the independent rail-position measurement
  - Velocity estimation for drive and real motion
  - Slip estimation between wheel-estimated travel and measured rail travel
  - Motor command helpers and drive stop/reset logic
  - Motion profiling for target approach, slowdown, and settling
  - Closed-loop control logic for position and/or velocity control

  Keep the test-only serial interface and top-level node decisions outside this file.
*/

namespace {

// Convert a positive path length into an absolute position along the move.
float getAbsolutePositionAlongProfile(const TrajectoryProfile &profile, float pathDistance) {
  return profile.startPosition + (pathDistance * profile.direction);
}

#pragma region Drive Real Position

/*
  Drive real-position sensor state.

  This is the drive-system-specific layer built on top of EncoderMux. The
  EncoderMux library knows how to talk to the muxed AS5600 hardware and track
  continuous counts. The DriveControl file adds the mechanical meaning:
  converting those counts into inches along the rail.
*/
EncoderMuxConfig driveRealPositionEncoderConfig = makeDefaultAs5600MuxConfig(0);
EncoderMuxState driveRealPositionEncoderState = {};

// Mechanical calibration for the passive measurement wheel.
float driveRealPositionWheelRadiusIn = 0.0f;
float driveRealPositionWheelCircumferenceIn = 0.0f;

/*
  This offset lets the current physical sensor position map to any desired
  reported position value after a reset. For example, the current location can
  be defined as 0.0 in, 12.5 in, or any other known coordinate.
*/
float driveRealPositionZeroOffsetIn = 0.0f;

// Cached position results from the most recent sensor update.
float driveRealPositionIn = 0.0f;
float driveRealPositionFirIn = 0.0f;

/*
  FIR filter configuration and history.

  FIR stands for Finite Impulse Response. The filter output is a weighted sum
  of recent position samples. A tap is one coefficient/history pair in that
  weighted sum.
*/
const uint8_t DRIVE_REAL_POSITION_FIR_MAX_TAPS = 8;
float driveRealPositionFirCoefficients[DRIVE_REAL_POSITION_FIR_MAX_TAPS] = {1.0f};
float driveRealPositionFirSamples[DRIVE_REAL_POSITION_FIR_MAX_TAPS] = {0.0f};
uint8_t driveRealPositionFirTapCount = 1;
bool driveRealPositionFirHistoryPrimed = false;

/*
  Convert continuous encoder counts into linear rail position.

  This helper is kept private because it depends on the drive system's
  mechanical calibration values stored in this file.
*/
float convertDriveRealPositionCountsToInches(long encoderCounts) {
  const int countsPerRevolution =
      (driveRealPositionEncoderConfig.countsPerRevolution > 0)
          ? driveRealPositionEncoderConfig.countsPerRevolution
          : 4096;

  if (countsPerRevolution <= 0) {
    return driveRealPositionZeroOffsetIn;
  }

  const float wheelRevolutions =
      ((float)encoderCounts) / (float)countsPerRevolution;
  return driveRealPositionZeroOffsetIn +
         (wheelRevolutions * driveRealPositionWheelCircumferenceIn);
}

/*
  Clear the FIR sample history and optionally seed it with a known position.

  Seeding all history slots with the same starting position prevents the filter
  from producing a misleading transient drop toward zero on the first few
  updates after a reset or parameter change.
*/
void resetDriveRealPositionFirHistory(float seedPositionIn) {
  for (uint8_t i = 0; i < DRIVE_REAL_POSITION_FIR_MAX_TAPS; ++i) {
    driveRealPositionFirSamples[i] = seedPositionIn;
  }

  driveRealPositionFirHistoryPrimed = true;
  driveRealPositionFirIn = seedPositionIn;
}

/*
  Insert one new raw position sample into the FIR history buffer.

  Index 0 is always the newest sample. Older samples are shifted toward the end
  of the array so the coefficients line up in time order.
*/
float applyDriveRealPositionFirFilter(float newPositionIn) {
  if (!driveRealPositionFirHistoryPrimed) {
    resetDriveRealPositionFirHistory(newPositionIn);
  }

  for (int i = driveRealPositionFirTapCount - 1; i > 0; --i) {
    driveRealPositionFirSamples[i] = driveRealPositionFirSamples[i - 1];
  }
  driveRealPositionFirSamples[0] = newPositionIn;

  float filteredPositionIn = 0.0f;
  for (uint8_t i = 0; i < driveRealPositionFirTapCount; ++i) {
    filteredPositionIn +=
        driveRealPositionFirCoefficients[i] * driveRealPositionFirSamples[i];
  }

  driveRealPositionFirIn = filteredPositionIn;
  return driveRealPositionFirIn;
}

#pragma endregion

/*
  Ramp controller state lives at file scope because the controller needs to
  remember values from one update call to the next.

  These are kept inside the anonymous namespace so they stay private to this
  file until the DriveControl library grows into a larger class-based design.
*/

// Controller gains. These all default to zero until the caller sets them.
float rampKp = 0.0f;
float rampKi = 0.0f;
float rampKd = 0.0f;

// Only accumulate integral error while the position error is inside this window.
float rampIntegralWindow = 0.0f;

// PID state carried between control updates.
float rampError = 0.0f;
float prevRampError = 0.0f;
float rampIntError = 0.0f;

// Individual PID term values are stored separately to aid debugging and tuning.
float rampPTerm = 0.0f;
float rampITerm = 0.0f;
float rampDError_dt = 0.0f;
float rampDTerm = 0.0f;

// Output motor command from the PID controller.
int rampMtrCmd = 0;

// Timing values used to compute elapsed time between PID updates.
float T = 0.0f;
float T0 = 0.0f;
float TOld = 0.0f;
bool rampTimingInitialized = false;

}  // namespace

#pragma region Drive_Real_Position_Sensor

/*
  Store all drive real-position sensor calibration in one place.

  This mirrors the values that were previously hard-coded in the drive test:
  mux channel, AS5600 count behavior, and the passive wheel radius used to turn
  encoder revolutions into inches of travel.
*/
void setDriveRealPositionSensorParameters(uint8_t muxChannel,
                                          float wheelRadiusIn,
                                          int countsPerRevolution,
                                          int wrapThreshold,
                                          int noiseThreshold,
                                          uint8_t muxAddress,
                                          uint8_t sensorAddress,
                                          uint8_t rawAngleRegister) {
  driveRealPositionEncoderConfig.wireBus = &Wire;
  driveRealPositionEncoderConfig.muxAddress = muxAddress;
  driveRealPositionEncoderConfig.sensorAddress = sensorAddress;
  driveRealPositionEncoderConfig.rawAngleRegister = rawAngleRegister;
  driveRealPositionEncoderConfig.muxChannel = muxChannel;
  driveRealPositionEncoderConfig.countsPerRevolution = countsPerRevolution;
  driveRealPositionEncoderConfig.wrapThreshold = wrapThreshold;
  driveRealPositionEncoderConfig.noiseThreshold = noiseThreshold;

  driveRealPositionWheelRadiusIn = fabsf(wheelRadiusIn);
  driveRealPositionWheelCircumferenceIn = 2.0f * PI * driveRealPositionWheelRadiusIn;

  /*
    Changing calibration invalidates any previously tracked position history, so
    both the encoder state and filter state are reset to a clean baseline.
  */
  resetEncoderState(driveRealPositionEncoderState);
  driveRealPositionZeroOffsetIn = 0.0f;
  driveRealPositionIn = 0.0f;
  resetDriveRealPositionFirHistory(0.0f);
}

/*
  Copy the caller's FIR coefficients into the internal filter state.

  The library stores its own copy so the caller does not need to manage the
  lifetime of the original array after configuration.
*/
void setDriveRealPositionFirParameters(const float *coefficients,
                                       uint8_t coefficientCount) {
  /*
    Invalid FIR input falls back to a pass-through filter:
    output = latest raw position

    This keeps the API safe even if setup code passes a null pointer or an
    empty coefficient list by mistake.
  */
  if (coefficients == NULL || coefficientCount == 0) {
    driveRealPositionFirCoefficients[0] = 1.0f;
    driveRealPositionFirTapCount = 1;
    resetDriveRealPositionFirHistory(driveRealPositionIn);
    return;
  }

  if (coefficientCount > DRIVE_REAL_POSITION_FIR_MAX_TAPS) {
    coefficientCount = DRIVE_REAL_POSITION_FIR_MAX_TAPS;
  }

  for (uint8_t i = 0; i < coefficientCount; ++i) {
    driveRealPositionFirCoefficients[i] = coefficients[i];
  }

  for (uint8_t i = coefficientCount; i < DRIVE_REAL_POSITION_FIR_MAX_TAPS; ++i) {
    driveRealPositionFirCoefficients[i] = 0.0f;
  }

  driveRealPositionFirTapCount = coefficientCount;
  resetDriveRealPositionFirHistory(driveRealPositionIn);
}

/*
  Zero the drive real-position tracker against the encoder's current raw angle.

  Using the current raw angle as the reference avoids an artificial jump on the
  next update. The caller chooses what physical position value should correspond
  to that current location.
*/
bool resetDriveRealPositionSensor(float newZeroPositionIn) {
  driveRealPositionZeroOffsetIn = newZeroPositionIn;
  return resetDriveRealPositionCounts(0);
}

/*
  Reset the continuous encoder count total used by the real-position sensor.

  This helper is intentionally narrower than resetDriveRealPositionSensor().
  It focuses on encoder counts only:
  - clear the accumulated unwrapped count total
  - capture the encoder's current raw reading as the new unwrap reference
  - refresh the cached raw/FIR positions so they match the new count value

  The existing position offset is preserved. That means callers can zero the
  underlying encoder counts without losing whatever physical coordinate system
  the drive subsystem is currently using.
*/
bool resetDriveRealPositionCounts(long newZeroCounts) {
  const bool resetSucceeded =
      zeroEncoderToCurrentRaw(driveRealPositionEncoderState,
                              driveRealPositionEncoderConfig,
                              newZeroCounts);

  /*
    Whether the live read succeeds or fails, the EncoderMux helper leaves the
    state in a safe reset condition:
    - totalCounts is set to newZeroCounts
    - a new raw reference is captured if the hardware read succeeds
    - otherwise the next successful update will establish that reference

    Recomputing the cached positions here keeps all public getters consistent
    immediately after the reset call.
  */
  driveRealPositionIn =
      convertDriveRealPositionCountsToInches(driveRealPositionEncoderState.totalCounts);
  resetDriveRealPositionFirHistory(driveRealPositionIn);

  return resetSucceeded;
}

/*
  Read the encoder once, convert the updated count into inches, and refresh the
  FIR-filtered position using the same sample.

  This function is the main "update sensor state" entry point. Call it once per
  control loop, then use the getter helpers to access the cached raw and
  filtered positions without triggering extra I2C transactions.
*/
float updateDriveRealPositionFromSensor() {
  updateEncoderCounts(driveRealPositionEncoderState, driveRealPositionEncoderConfig);

  driveRealPositionIn =
      convertDriveRealPositionCountsToInches(driveRealPositionEncoderState.totalCounts);

  /*
    Even when the read fails, totalCounts stays unchanged inside EncoderMux, so
    re-applying the FIR filter simply reuses the last stable position sample.
  */
  applyDriveRealPositionFirFilter(driveRealPositionIn);
  return driveRealPositionIn;
}

float getDriveRealPosition() {
  return driveRealPositionIn;
}

float getDriveRealPositionFir() {
  return driveRealPositionFirIn;
}

long getDriveRealPositionCounts() {
  return driveRealPositionEncoderState.totalCounts;
}

bool didDriveRealPositionReadSucceed() {
  return driveRealPositionEncoderState.lastReadSucceeded;
}

#pragma endregion

#pragma region PID Controller

int rampControl(float rampPosDes, float rampPosReal){
  /*
    The controller operates on elapsed time since the first PID update, not on
    raw Arduino uptime. The first call initializes the reference time.

    rampPosReal is provided by the caller so this controller stays independent
    from the code that owns the real-position sensor hardware.
  */
  if (!rampTimingInitialized) {
    T0 = micros() / 1000000.0f;
    T = 0.0f;
    TOld = 0.0f;
    rampTimingInitialized = true;
  }

  T = micros()/1000000.0 - T0;
  
  float deltaPTime = T - TOld;

  rampError = rampPosDes - rampPosReal;
  rampPTerm = rampKp*rampError;

  /*
    Integral accumulation is only allowed inside the configured error window.
    This reduces the chance of the integral term building up while the system
    is still far away from the target.

    The Ki == 0 guard prevents division-by-zero in the clamp calculation while
    the controller is still using its default zero gains.
  */
  if (fabsf(rampError) < rampIntegralWindow && rampKi != 0.0f){
    rampIntError = rampIntError + rampError*deltaPTime;
    rampIntError = constrain(rampIntError, -400.0f/fabsf(rampKi), 400.0f/fabsf(rampKi));
    rampITerm = rampIntError*rampKi;
  } else {
    rampITerm = 0.0f;
  }

  /*
    Derivative uses the change in error over time. If two updates happen at the
    same timestamp, treat the derivative as zero to avoid dividing by zero.
  */
  if (deltaPTime > 0.0f) {
    rampDError_dt = (rampError - prevRampError) / deltaPTime;
  } else {
    rampDError_dt = 0.0f;
  }
  rampDTerm = rampKd * rampDError_dt;

  rampMtrCmd = constrain(rampPTerm + rampITerm + rampDTerm, -400, 400);

  TOld = T;
  prevRampError = rampError;

  return rampMtrCmd;

}

#pragma endregion

#pragma region Helpers

/*
  Set the ramp controller gains.

  This helper only updates the tuning values. It does not reset the controller
  history, which allows gain changes during tuning without erasing the stored
  error terms unless a separate reset helper is added later.
*/
void setRampControlGains(float kp, float ki, float kd) {
  rampKp = kp;
  rampKi = ki;
  rampKd = kd;
}

/*
  Update only the proportional gain.

  This is primarily a tuning convenience helper for testing, where the caller
  may want to adjust one gain at a time without re-sending the others.
*/
void setRampControlKp(float kp) {
  rampKp = kp;
}

/*
  Update only the integral gain.

  Keeping Ki configurable on its own makes it easy to temporarily disable or
  re-enable integral action while testing the controller.
*/
void setRampControlKi(float ki) {
  rampKi = ki;
}

/*
  Update only the derivative gain.

  This helper supports derivative tuning without changing the proportional or
  integral terms.
*/
void setRampControlKd(float kd) {
  rampKd = kd;
}

/*
  Set the allowable error window for integral accumulation.

  Only the magnitude of the window matters, so the stored value is forced to be
  non-negative even if a negative number is provided by mistake.
*/
void setRampIntegralWindow(float integralWindow) {
  rampIntegralWindow = fabsf(integralWindow);
}

/*
  Reset all stored controller history that depends on previous updates.

  This is useful before starting a new move or whenever the caller wants to
  discard old integral, derivative, and timing history. The gains and integral
  window are intentionally left alone because they are controller settings, not
  transient runtime state.
*/
void resetRampController() {
  rampError = 0.0f;
  prevRampError = 0.0f;
  rampIntError = 0.0f;

  rampPTerm = 0.0f;
  rampITerm = 0.0f;
  rampDError_dt = 0.0f;
  rampDTerm = 0.0f;

  rampMtrCmd = 0;

  T = 0.0f;
  T0 = 0.0f;
  TOld = 0.0f;
  rampTimingInitialized = false;
}

#pragma endregion

#pragma region Trajectory Builder

/*
  Build the motion profile for a move in absolute space.

  Example:
  - startPosition = 12 in
  - stopPosition = 36 in

  The generated profile will begin at 12 in, move forward 24 in, and every
  sampled position will also be returned in absolute rail coordinates.
*/
TrajectoryProfile buildTrajectoryProfile(float maxAcceleration,
                                         float steadyStateSpeed,
                                         float startPosition,
                                         float stopPosition) {
  TrajectoryProfile profile = {};

  // Store the original caller inputs so the finished profile is self-contained.
  profile.maxAcceleration = fabsf(maxAcceleration);
  profile.steadyStateSpeed = fabsf(steadyStateSpeed);
  profile.startPosition = startPosition;
  profile.stopPosition = stopPosition;

  const float requestedDisplacement = stopPosition - startPosition;
  profile.direction = (requestedDisplacement < 0.0f) ? -1.0f : 1.0f;
  profile.totalDistance = fabsf(requestedDisplacement);

  // If the robot is already at the target, this is a valid zero-length move.
  if (profile.totalDistance == 0.0f) {
    profile.isValid = true;
    return profile;
  }

  // Acceleration and speed limits must both be positive to build a usable move.
  if (profile.maxAcceleration <= 0.0f || profile.steadyStateSpeed <= 0.0f) {
    return profile;
  }

  // Distance needed to accelerate from rest up to steadyStateSpeed.
  const float accelDistanceToCruise =
      (profile.steadyStateSpeed * profile.steadyStateSpeed) / (2.0f * profile.maxAcceleration);

  // Total distance needed to accelerate to cruise and later decelerate to rest.
  const float fullSpeedDistance = 2.0f * accelDistanceToCruise;

  profile.isValid = true;

  /*
    If the move is too short to ever reach the requested steady-state speed,
    switch to a triangular profile:
    - accelerate at maxAcceleration
    - immediately decelerate at maxAcceleration

    The peak speed is chosen so the robot still stops exactly at stopPosition.
  */
  if (profile.totalDistance <= fullSpeedDistance) {
    profile.reachesSteadyStateSpeed = false;
    profile.peakSpeed = sqrtf(profile.totalDistance * profile.maxAcceleration);
    profile.accelDurationSeconds = profile.peakSpeed / profile.maxAcceleration;
    profile.cruiseDurationSeconds = 0.0f;
    profile.decelDurationSeconds = profile.accelDurationSeconds;
    profile.totalDurationSeconds =
        profile.accelDurationSeconds + profile.decelDurationSeconds;
    profile.accelDistance = profile.totalDistance * 0.5f;
    profile.cruiseDistance = 0.0f;
    profile.decelStartDistance = profile.accelDistance;
    return profile;
  }

  // Otherwise, this is the normal trapezoidal case with a true cruise segment.
  profile.reachesSteadyStateSpeed = true;
  profile.peakSpeed = profile.steadyStateSpeed;
  profile.accelDurationSeconds = profile.steadyStateSpeed / profile.maxAcceleration;
  profile.decelDurationSeconds = profile.accelDurationSeconds;
  profile.accelDistance = accelDistanceToCruise;
  profile.cruiseDistance = profile.totalDistance - fullSpeedDistance;
  profile.cruiseDurationSeconds = profile.cruiseDistance / profile.steadyStateSpeed;
  profile.decelStartDistance = profile.accelDistance + profile.cruiseDistance;
  profile.totalDurationSeconds = profile.accelDurationSeconds +
                                 profile.cruiseDurationSeconds +
                                 profile.decelDurationSeconds;

  return profile;
}

/*
  Return the desired absolute position for the requested elapsed time.

  The trajectory is split into three time regions:
  1. Acceleration from rest
  2. Constant-speed cruise, when the move is long enough
  3. Deceleration back to rest
*/
float getTrajectoryPositionAtElapsedTime(const TrajectoryProfile &profile,
                                         float elapsedTimeSeconds) {
  // Invalid profiles are treated as "hold where you started" so callers fail safe.
  if (!profile.isValid) {
    return profile.startPosition;
  }

  // Zero-length trajectories begin and end at the same absolute position.
  if (profile.totalDistance == 0.0f) {
    return profile.stopPosition;
  }

  /*
    The caller is expected to pass time elapsed since the trajectory started.
    Negative time should not occur in normal use, but returning the start
    position makes this function safe against accidental bad inputs.
  */
  if (elapsedTimeSeconds <= 0.0f) {
    return profile.startPosition;
  }

  // Once the profile has ended, keep returning the final stop position.
  if (elapsedTimeSeconds >= profile.totalDurationSeconds) {
    return profile.stopPosition;
  }

  // Acceleration phase: x = 0.5 * a * t^2
  if (elapsedTimeSeconds <= profile.accelDurationSeconds) {
    const float pathDistance =
        0.5f * profile.maxAcceleration * elapsedTimeSeconds * elapsedTimeSeconds;
    return getAbsolutePositionAlongProfile(profile, pathDistance);
  }

  // Cruise phase: distance already covered during acceleration + v * dt
  if (elapsedTimeSeconds <=
      (profile.accelDurationSeconds + profile.cruiseDurationSeconds)) {
    const float cruiseElapsedSeconds =
        elapsedTimeSeconds - profile.accelDurationSeconds;
    const float pathDistance =
        profile.accelDistance + (profile.peakSpeed * cruiseElapsedSeconds);
    return getAbsolutePositionAlongProfile(profile, pathDistance);
  }

  // Deceleration phase: continue from the decel start point and subtract braking distance.
  const float decelElapsedSeconds = elapsedTimeSeconds -
                                    profile.accelDurationSeconds -
                                    profile.cruiseDurationSeconds;
  const float pathDistance = profile.decelStartDistance +
                             (profile.peakSpeed * decelElapsedSeconds) -
                             (0.5f * profile.maxAcceleration *
                              decelElapsedSeconds * decelElapsedSeconds);
  return getAbsolutePositionAlongProfile(profile, pathDistance);
}

// Simple helper for callers that only need the total planned move time.
float getTrajectoryDuration(const TrajectoryProfile &profile) {
  return profile.totalDurationSeconds;
}

// Invalid profiles are never considered complete because they were never usable.
bool isTrajectoryFinished(const TrajectoryProfile &profile, float elapsedTimeSeconds) {
  if (!profile.isValid) {
    return false;
  }

  return elapsedTimeSeconds >= profile.totalDurationSeconds;
}

#pragma endregion
