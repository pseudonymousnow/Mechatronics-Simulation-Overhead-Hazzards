#pragma once

/*
  RobotProtocol.h

  Put the shared communication language for manager <-> node here.

  This file should eventually include:
  - Packet type constants for run, go, estop, ready, ack, finish, battery, and winch commands
  - Shared structs used by both manager and node, such as run command data and finish report data
  - Function declarations for parsing incoming payload strings
  - Function declarations for formatting outgoing payload strings
  - Shared CSV / tokenization helper declarations if you want one common implementation

  This file should NOT include:
  - XBee polling loops
  - Manager state machine logic
  - Node state machine logic
  - Direct hardware control
*/

#pragma region Includes

#include <Arduino.h>

#pragma endregion

#pragma region Constants

// Shared protocol constants and packet identifiers go here.

#pragma endregion

#pragma region Shared_Structs

// Shared packet, run command, and finish report structs go here.

#pragma endregion

#pragma region Parse_Function_Declarations

// Shared parsing function declarations go here.

#pragma endregion

#pragma region Format_Function_Declarations

// Shared payload formatting function declarations go here.

#pragma endregion

#pragma region Utility_Function_Declarations

// Shared sanitize / CSV helper declarations go here if you want one copy for both boards.

#pragma endregion
