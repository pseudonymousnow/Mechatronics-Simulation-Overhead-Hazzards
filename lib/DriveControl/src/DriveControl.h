#pragma once

#include <stdbool.h>

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
