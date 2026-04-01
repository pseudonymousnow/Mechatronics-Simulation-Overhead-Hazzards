#pragma once

/*
  WinchControl.h

  Put the full reusable winch action subsystem here.

  Based on your intended architecture, this library should own:
  - The internal winch action state machine
  - All logic for what the winch needs to do after node_main requests an action
  - Homing, lowering, hoisting, and prepare-for-traverse behavior
  - Pawl servo control helpers
  - Winch encoder position and speed tracking

  node_main.cpp should only need to do things like:
  - request a winch action
  - call winchAction() / update() while in the states that require winch motion
  - check whether the requested action is complete

  This file should eventually include:
  - Config structs for motor, pawl servo, home switch, encoder mux channel, and tuning constants
  - Enums for requested winch actions and internal sub-steps
  - Status structs/classes for current position, speed, action, and completion flags
  - Public function/class declarations for begin, start action, update action, stop, read position, and completion checks
*/

#pragma region Includes

#include <Arduino.h>
#include <Servo.h>

#pragma endregion

#pragma region Config_Structs

// Winch hardware and tuning configuration structs go here.

#pragma endregion

#pragma region Action_Enums

// Requested winch action enums go here.

#pragma endregion

#pragma region Internal_State_Enums

// Internal non-blocking winch sub-step enums go here.

#pragma endregion

#pragma region Status_Structs

// Winch status and measurement structs go here.

#pragma endregion

#pragma region Class_Declarations

// Winch control class declarations go here.

#pragma endregion

#pragma region Public_Function_Declarations

// Public action request, update, stop, read position, and completion declarations go here.

#pragma endregion
