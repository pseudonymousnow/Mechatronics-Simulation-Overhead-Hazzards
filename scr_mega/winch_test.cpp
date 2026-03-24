#include <Arduino.h>
#include "DualG2HighPowerMotorShield.h"
#include <Servo.h> //figure out which one this needs to be. may be using wrong library in vscode

// 1. Setup the Motor Shield
  DualG2HighPowerMotorShield24v14 md;

// 2. Setup the Winch Parts
  Servo pawlServo;
  int PAWL_SERVO_PIN = 13;

// Variables Defined in Dylan's Guide
  String winchAction = "IDLE";
  float depthTarget = 0;
  String winchNextState = "";
  bool winchComplete = false;

// New Variables+++

  float drum_radius = (1.5/2) + (1/16);                  // Inches adding width of cable !CHECK!
  const int Winch_ENC_PIN = A11;              // Delete if you use Quinton's library
  const int Winch_ENC_COUNTS = 1024;          // Delete if you use Quinton's library
  const int WRAP_THRESH = Winch_ENC_COUNTS/2; // 512
  const int NOISE_THRESH = 4;                 // counts
  float winchHome = 0;
  float actual_winchPos = 0;
  float t_stamp = 0;                         //Timestamp used for non-blocking delays
  float winchSpeed = 0;
  float old_winchPos = 0;
  float t_winch_old = 0;

  int Winch_lastPos = 0;                      // Delete if you use Quinton's library
  long Winch_totalSteps = 0;                  // Delete if you use Quinton's library

// Raise and Lower Speeds
  int raise_speed = 400;                      // Positive is up! Use change md.flipM2(true) to false if positive is down.
  int lower_speed = -200;                     // Negative should be down.

// 3. Setup AS5600 as analog sensor
// MagneticSensorAnalog sensor(Winch_ENC_PIN, 0, 1023);

// Simplified Steps for the Winch
  enum WinchSteps {IDLE, JOG_UP, UNLOCK, MOVING, BRAKE_AND_LOCK };
  WinchSteps currentStep = IDLE;

void setup() {
  Serial.begin(115200);
  md.init();           // Starts the motor shield
  md.enableDrivers();  // Turns the power on
  md.flipM2(true);
  pawlServo.attach(PAWL_SERVO_PIN);
  pawlServo.write(70);  // Start Locked
  // pawlServo.write(130);  // Start Unlocked
  Winch_lastPos = analogRead(Winch_ENC_PIN);
  winchHome = getEncoderDepth();        // !CHECK! TEMPORARY: USE TIL WE HAVE A HOMING SWITCH. SETS HOME TO INITIAL START POSITION.
}

void loop() {

  // Actual Encoder Depth 
  actual_winchPos = getEncoderDepth();
  winchSpeed = getWinchSpeedFromPos(actual_winchPos);
  float winchSpeedFiltered = getWinchSpeedFromPosFiltered(actual_winchPos);

  // IMPORTANT: KEEP IN CODE. CHECKS FOR M2 FAULTS
  if (md.getM2Fault()) {
    Serial.println("Motor Fault! WTF");
    md.setM2Speed(0);  // Emergency Brake
    return;
  }
// Main Loop
  runWinchLogic();
  Serial.print("Pos: ");
  Serial.print(actual_winchPos);
  Serial.print(" | Speed: ");
  Serial.print(winchSpeed);
  Serial.print(", ");
  Serial.print(winchSpeedFiltered);
  Serial.print(" | State: ");
  Serial.println(currentStep);
  // getEncoderDepth();

  // Important: Emergency stop command for testing
  // if (Serial.available() > 0) {
  // char c = Serial.read();
  // if (c == 'x') currentStep = BRAKE_AND_LOCK;        // PRESS X FOR EMERGENCY STOP AND LOCK
  // md.setM2Speed(0); // Kill motor instantly
  // Serial.println("!!! EMERGENCY STOP TRIGGERED !!!");
  // }
}
//Winch speed & Pos functions
  float getEncoderDepth() {
    int currentPos = analogRead(Winch_ENC_PIN);

    int delta = currentPos - Winch_lastPos;

    // unwrap
    if (delta > WRAP_THRESH) {
      delta -= Winch_ENC_COUNTS;
    } else if (delta < -WRAP_THRESH) {
      delta += Winch_ENC_COUNTS;
    }

    // noise gate
    int absDelta = abs(delta);
    if (absDelta > 2 /*&& absDelta < 300 /*NOISE_THRESH*/) { //SET BACK TO NOISE_THRESH (4) !CHECK!
      Serial.print("Above Noise Thresh: ");
      Serial.println(delta);
      Winch_totalSteps += delta;
      Winch_lastPos = currentPos;
    }

    // Convert measurement to linear travel distance
    const float circumference = 2.0f * PI * drum_radius;
    float inches = ( (float)Winch_totalSteps / (float)Winch_ENC_COUNTS ) * circumference;

    return inches;
  }

float getWinchSpeedFromPos(float currentPosInches){ 
  float t_new = millis(); 
  winchSpeed = ((old_winchPos - currentPosInches)*1000.0)/(t_winch_old - t_new); 
  t_winch_old = t_new; 
  old_winchPos = currentPosInches; 
  return winchSpeed; //speed is in in/s 
}

float getWinchSpeedFromPosFiltered(float currentPosInches) {
  // --- speed estimator state ---
  static float lastPos = 0.0f;
  static uint32_t lastMs = 0;

  // --- EMA filter state ---
  static float v_filt = 0.0f;
  static bool filtInit = false;

  // --- hard-coded tunables ---
  const uint32_t MIN_DT_MS = 10;  // update speed at most at 50 Hz (reduces dt=0/1ms noise) !CHECK!
  const float TAU_S = 0.05f;      // EMA time constant (0.05–0.20 typical)

  uint32_t nowMs = millis();

  // Initialize on first run
  if (lastMs == 0) {
    lastMs = nowMs;
    lastPos = currentPosInches;
    v_filt = 0.0f;
    filtInit = true;
    return 0.0f;
  }

  uint32_t dtMs = nowMs - lastMs;

  // If called too quickly, return last filtered value
  if (dtMs < MIN_DT_MS) {
    return v_filt;
  }

  float dt_s = dtMs / 1000.0f;

  // Raw speed (in/s)
  float v_raw = (currentPosInches - lastPos) / dt_s;

  // Update estimator state
  lastMs = nowMs;
  lastPos = currentPosInches;

  // EMA filter
  if (!filtInit) {
    v_filt = v_raw;
    filtInit = true;
  } else {
    float alpha = dt_s / (TAU_S + dt_s);  // stable even if dt varies
    v_filt += alpha * (v_raw - v_filt);
  }

  return v_filt;
}


//Winch Control Functions
  static const int MAX_CMD = 400;

  // Simple clamp helpers
  static inline float clampf(float x, float lo, float hi) { return (x < lo) ? lo : (x > hi) ? hi : x; }
  static inline int clampi(int x, int lo, int hi) { return (x < lo) ? lo : (x > hi) ? hi : x; }

  // Sign helper
  static inline int sgn(float x) { return (x > 0) - (x < 0); }

  // ===== USER-SUPPLIED SENSORS (stubs; replace with your real ones) =====
  float getWinchPosition();  // unfiltered getEncoderDepth

  // ==============================
  // 1) Speed Ramp Controller
  // - Slowly ramps motor command until desired speed is met.
  // - Optional: can be used for "open-loop" speed matching with just a small P term.
  // ==============================
  int winchRampToSpeedCmd(float desiredSpeed,
                          float measuredSpeed,
                          float dt_s,
                          float cmdRatePerSec,   // how fast command may change (PWM units per second), e.g. 200
                          float kP_speed)        // small proportional gain, e.g. 10..60 depending on speed units
  {
    // Proportional "requested" command based on speed error
    float speedErr = desiredSpeed - measuredSpeed;
    float cmdTarget = kP_speed * speedErr;

    // Clamp target
    cmdTarget = clampf(cmdTarget, -MAX_CMD, +MAX_CMD);

    // Rate-limit to "slowly ramp"
    static float cmdOut = 0.0f;
    float maxStep = cmdRatePerSec * dt_s;            // max change this update
    float delta = cmdTarget - cmdOut;
    delta = clampf(delta, -maxStep, +maxStep);
    cmdOut += delta;

    // Deadband near zero to prevent chatter
    if (fabs(cmdOut) < 2.0f) cmdOut = 0.0f;

    return (int)cmdOut;
  }

void runWinchLogic() {
  switch (currentStep) {

    case IDLE:
      // Do nothing until the Manager sends a command
      // Default Case is idle. Add individual testing commands BELOW:

      // // Testing Code:
      // if (Serial.available() > 0) {
      // char c = Serial.read();
      // if (c == 'u') md.setM2Speed(100);
      // else if (c == 'd') md.setM2Speed(-100);
      // else if (c == 'x') md.setM2Speed(0);
      // }

      if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'j') {
        currentStep = JOG_UP;
        t_stamp = millis();
      }
      else if (c == 'u') {md.setM2Speed(200);}
      else if (c == 'd') {md.setM2Speed(-100);}
      else if (c == 'x') { md.setM2Speed(0);}
      }

      // //Testing Code:


      break;

    case JOG_UP:
      // Move up slightly to take pressure off the lock !CHECK! maybe change to encoder based
      md.setM2Speed(100);  // setM2Speed (Positive is UP)
      if ((millis() - t_stamp) >= 200){
          currentStep = UNLOCK;
          t_stamp = millis();
      }
      
      break;

    case UNLOCK:
      pawlServo.write(130);  // Disengage servo pawl
      if (millis() - t_stamp >= 500){
        currentStep = MOVING;
        t_stamp = millis();
      }
      break;

    case MOVING:
      winchAction = "lower"; //!CHECK! hardcoded for now
      depthTarget = -10.0; //!CHECK! hardcoded for now
      if (winchAction == "lower") {
        md.setM2Speed(lower_speed);
        if (actual_winchPos <= depthTarget) currentStep = BRAKE_AND_LOCK;
      }
      if (winchAction == "raise") {
        md.setM2Speed(raise_speed);
        if (actual_winchPos >= winchHome) currentStep = BRAKE_AND_LOCK; 
      }
      break;

    case BRAKE_AND_LOCK:
      md.setM2Speed(0);    // KEYWORD: This triggers the BRAKE
      pawlServo.write(70);  // Re-engage the mechanical lock
      delay(500);
      winchComplete = true;
      currentStep = IDLE;
      Serial.println("Finished!");
      break;
  }
}