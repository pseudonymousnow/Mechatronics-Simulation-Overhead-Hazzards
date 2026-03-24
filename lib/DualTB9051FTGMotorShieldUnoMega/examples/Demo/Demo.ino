#include "DualTB9051FTGMotorShieldUnoMega.h"

// If using the default pins, use the following line:
DualTB9051FTGMotorShieldUnoMega md;

float f = 0.25;             //frequency in Hz
unsigned long t = 0;        //current time

void setup() {
  // put your setup code here, to run once:
  
  md.init();
  md.enableDrivers();
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  double t = micros() / 1000000.0; //current time
  int M = 400 * sin(2 * PI * f * t); //Sinusoid motor voltage command
  // Turn on Motor 1 for 4 seconds /////////////////////////////////////////
  if (0.0 < t && t < 4.0) {
    md.setM1Speed(M);
  }
  else {
    md.setM1Speed(0);
  }
  // Turn on Motor 2 for 4 seconds /////////////////////////////////////////
  if (4.0 < t && t < 8.0) {
    md.setM2Speed(M);
  }
  else {
    md.setM2Speed(0);
  }

}
