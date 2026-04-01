#include <EncoderMux.h>

/*
  EncoderMux.cpp

  Put the reusable low-level mux and AS5600 implementations here.

  This file should eventually include:
  - Wire transactions for mux channel selection
  - Raw AS5600 register reads
  - Wraparound handling for 12-bit angle values
  - Continuous count accumulation logic
  - Safe behavior when a read fails

  Keep drive-specific and winch-specific conversion math out of this file.
*/

#pragma region Includes

// Add any extra implementation-only includes here.

#pragma endregion

#pragma region Low_Level_I2C_Functions

// Mux select and raw AS5600 read implementations go here.

#pragma endregion

#pragma region Continuous_Count_Update_Functions

// Wrap handling and count accumulation logic go here.

#pragma endregion

#pragma region Reset_And_State_Functions

// Encoder zero/reset helpers go here.

#pragma endregion
