#pragma once

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

// Shared drive and control declarations go here.
