#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
  DriveControl.h

  Put the reusable drive sensing and control interface here.

  This file should eventually include:
  - Config structs for drive motor behavior, encoder calibration, real-position calibration, and controller tuning
  - Enums for drive/control modes such as idle, manual, and closed-loop position control
  - Status structs/classes for drive position, real position, velocities, slip, target state, and applied motor command
  - Public function/class declarations for begin, reset, update, stop, manual drive commands, and move-to-position commands
  - Getter/query declarations for current drive status and target-complete checks

  This file should NOT include:
  - Test-sketch serial command parsing
  - Node FSM transitions
  - XBee communication logic

  This library should answer:
  - How do we read the drive-related sensors and command the robot to move along the rail?
*/

/*
  TrajectoryProfile stores the complete motion plan for a single move.

  All positions should be in the same "real position" units used by the rest
  of the control system, for example inches along the rail.

  All time values stored in this struct are durations in seconds, not raw
  Arduino timestamps. Raw timestamps from millis() or micros() should stay in
  the caller, which can convert them into an elapsed time before sampling the
  trajectory.
*/
struct TrajectoryProfile {
  // User-provided trajectory limits.
  float maxAcceleration;
  float steadyStateSpeed;
  float startPosition;
  float stopPosition;

  // Derived geometric properties of the move.
  float direction;
  float totalDistance;
  float peakSpeed;

  // Duration of each phase of the move, in seconds.
  float accelDurationSeconds;
  float cruiseDurationSeconds;
  float decelDurationSeconds;
  float totalDurationSeconds;

  // Derived distances for each phase of the move.
  float accelDistance;
  float cruiseDistance;
  float decelStartDistance;

  // Metadata that makes it easy for callers to inspect the profile.
  bool reachesSteadyStateSpeed;
  bool isValid;
};

/*
  Configure the drive system's independent real-position sensor.

  This helper groups the drive-specific encoder and mechanical calibration in
  one place so setup code can describe the full sensor arrangement with a
  single call.

  wheelRadiusIn should be the radius of the passive measurement wheel that is
  driven by the rail, not the powered drive wheel.
*/
void setDriveRealPositionSensorParameters(uint8_t muxChannel,
                                          float wheelRadiusIn,
                                          int countsPerRevolution,
                                          int wrapThreshold,
                                          int noiseThreshold,
                                          uint8_t muxAddress = 0x70,
                                          uint8_t sensorAddress = 0x36,
                                          uint8_t rawAngleRegister = 0x0C);

/*
  Configure the FIR filter used for the real-position signal.

  FIR stands for Finite Impulse Response. In practice this means the filtered
  position is a weighted combination of the most recent position samples.

  coefficientCount must be greater than zero and no larger than the internal
  library limit. The coefficients are copied into the library, so the caller
  does not need to keep the original array alive after this function returns.
*/
void setDriveRealPositionFirParameters(const float *coefficients,
                                       uint8_t coefficientCount);

/*
  Reset the drive real-position tracker so the current sensor position becomes
  a known reference point.

  newZeroPositionIn is the physical position value that should be reported
  immediately after the reset. Passing 0.0f creates a normal zeroing action.
*/
bool resetDriveRealPositionSensor(float newZeroPositionIn = 0.0f);

/*
  Reset only the tracked continuous encoder counts used by the drive real-
  position sensor.

  This is a lower-level helper than resetDriveRealPositionSensor(). It keeps
  the existing position offset/calibration, but clears the accumulated count
  total and re-anchors the unwrap logic to the encoder's current raw reading.

  newZeroCounts is the count value that should be stored immediately after the
  reset. Passing 0 performs a standard encoder zero.
*/
bool resetDriveRealPositionCounts(long newZeroCounts = 0);

/*
  Read the sensor once and update the stored raw and FIR-filtered positions.

  The returned value is the unfiltered real position derived directly from the
  sensor. The corresponding filtered value can be read afterward with
  getDriveRealPositionFir().
*/
float updateDriveRealPositionFromSensor();

// Return the most recently computed unfiltered real position.
float getDriveRealPosition();

// Return the most recently computed FIR-filtered real position.
float getDriveRealPositionFir();

// Return the most recent continuous encoder count used for real position.
long getDriveRealPositionCounts();

// Return whether the most recent real-position sensor read succeeded.
bool didDriveRealPositionReadSucceed();

/*
  Compute the ramp controller motor command for a requested position.

  rampPosDes is the desired position for the current control update.
  rampPosReal is the measured real position supplied by the caller.

  The measured position is passed in as a function argument so sensor handling
  can remain in a separate library. This function returns the motor command
  after applying the P, I, and D terms.
*/
int rampControl(float rampPosDes, float rampPosReal);

/*
  Update the ramp controller gains.

  All gains default to 0.0f until this helper is called. Keeping the gains in a
  dedicated helper makes it easier for setup code to configure the controller
  from one place.
*/
void setRampControlGains(float kp, float ki, float kd);

/*
  Update the proportional gain only.

  A single-gain helper is useful during tuning because it allows test code to
  change just one term without needing to resend the other two values.
*/
void setRampControlKp(float kp);

/*
  Update the integral gain only.

  Keeping this separate from the combined gain setter makes it easier to enable
  or disable integral action during testing.
*/
void setRampControlKi(float ki);

/*
  Update the derivative gain only.

  This helper supports quick derivative tuning without disturbing the other
  controller gains.
*/
void setRampControlKd(float kd);

/*
  Set the size of the error window in which integral accumulation is allowed.

  The controller only grows the integral term when the absolute position error
  is smaller than this window. This helps reduce integral windup when the robot
  is still far from the target.
*/
void setRampIntegralWindow(float integralWindow);

/*
  Reset the stored ramp controller state.

  This clears the accumulated controller history so the next call to
  rampControl starts fresh. The gains are left unchanged so tuning values do
  not need to be reapplied after every reset.
*/
void resetRampController();

/*
  Build a trapezoidal or triangular position trajectory in absolute space.

  The move begins at startPosition, accelerates at maxAcceleration, cruises at
  steadyStateSpeed when possible, and then decelerates at maxAcceleration so it
  stops exactly at stopPosition.
*/
TrajectoryProfile buildTrajectoryProfile(float maxAcceleration,
                                         float steadyStateSpeed,
                                         float startPosition,
                                         float stopPosition);

/*
  Sample the trajectory using elapsed time since the trajectory started.

  elapsedTimeSeconds should be the amount of time that has passed since this
  specific move began, not the board uptime from millis() or micros().
  Elapsed times after the end of the profile return stopPosition.
*/
float getTrajectoryPositionAtElapsedTime(const TrajectoryProfile &profile,
                                         float elapsedTimeSeconds);

// Return the full planned move duration in seconds.
float getTrajectoryDuration(const TrajectoryProfile &profile);

/*
  Return true once the elapsed time since the trajectory start is at or beyond
  the end of the move.
*/
bool isTrajectoryFinished(const TrajectoryProfile &profile, float elapsedTimeSeconds);
