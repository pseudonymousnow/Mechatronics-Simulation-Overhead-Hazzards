#include <DriveControl.h>

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

// Shared drive and control definitions go here.
#pragma region PID Controller

void rampControl(float rampPosDes, int maxmin){
    if (maxmin = 0){
        return;
    }
}

T = micros()/1000000.0 = T0; //T0 needs to be a globably defined initial time variable

float deltaPTime = T - TOldDrive; //TOldDrive needs to be globably defined last timestep

rampError = rampPosDes - rampPosReal; //all need to be globally defined
rampPTerm = rampKp*rampError; //again, globaly defined.

if (abs(rampError) < rampIntegralWindow){
    rampIntError = rampIntError + rampError*deltaPTime;
    contrain(rampIntError, -float(maxmin)/rampKi, float(maxmin)/rampKi);
    rampITerm = rampIntError*rampKi;
}

rampDError_dt = (rampError - prevRampError) / deltaPTime;
rampDTerm = rampKd*rampDError_dt;

rampMtrCmd = contrain(rampPTerm + rampITerm + rampDTerm, -maxmin, maxmin)

#pragma endregion



#pragma region Trajectory Builder






#pragma endregion


