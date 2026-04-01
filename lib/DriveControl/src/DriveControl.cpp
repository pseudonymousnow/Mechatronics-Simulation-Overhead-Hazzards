#include <DriveControl.h>
#include <Arduino.h>
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
