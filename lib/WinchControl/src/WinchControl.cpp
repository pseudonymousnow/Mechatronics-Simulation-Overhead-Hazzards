#include <WinchControl.h>

/*
  WinchControl.cpp

  Put the full reusable winch action implementation here.

  This file should eventually include:
  - The internal non-blocking action logic for:
      * rehome
      * lower to depth
      * hoist to height
      * prepare for traverse
  - Pawl unlock / lock sequencing
  - Jog-up relief logic before unlocking when needed
  - Home switch handling
  - Winch encoder position and filtered speed updates
  - Completion and fault reporting

  node_main.cpp should decide WHEN a winch action is needed.
  WinchControl.cpp should decide HOW that requested action is executed step by step.

  Keep top-level node FSM transitions out of this file, but keep the internal winch action state machine here.
*/

#pragma region Includes

// Add any extra implementation-only includes here.

#pragma endregion

#pragma region Hardware_Helper_Functions

// Pawl servo, home switch, and motor helper implementations go here.

#pragma endregion

#pragma region Sensor_Update_Functions

// Winch position and speed update implementations go here.

#pragma endregion

#pragma region Action_Request_Functions

// Start-action and reset-action implementations go here.

#pragma endregion

#pragma region Internal_Action_State_Machine

// The internal non-blocking winch action logic goes here.

#pragma endregion

#pragma region Public_Status_And_Query_Functions

// Public completion, fault, and state query implementations go here.

#pragma endregion
