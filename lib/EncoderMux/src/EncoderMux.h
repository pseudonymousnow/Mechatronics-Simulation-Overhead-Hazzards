#pragma once

/*
  EncoderMux.h

  Put the reusable low-level TCA9548A + AS5600 access code here.

  This file should eventually include:
  - Config structs for mux address, sensor address, channel, counts per revolution, and noise threshold
  - A small state struct/class for tracked encoder state such as last raw count and total unwrapped counts
  - Function or class declarations for:
      * selecting a mux channel
      * reading a raw AS5600 angle
      * updating continuous counts with wrap handling
      * resetting encoder state to a known zero

  This file should NOT contain:
  - Winch depth conversion logic
  - Drive wheel distance conversion logic
  - Motor commands or subsystem state machines
*/

#pragma region Includes

#include <Arduino.h>
#include <Wire.h>

#pragma endregion

#pragma region Config_Structs

// Encoder mux and AS5600 configuration structs go here.

#pragma endregion

#pragma region State_Structs

// Continuous encoder state structs/classes go here.

#pragma endregion

#pragma region Class_Declarations

// Reusable muxed-encoder classes or function declarations go here.

#pragma endregion

#pragma region Helper_Function_Declarations

// Low-level helper declarations go here.

#pragma endregion
