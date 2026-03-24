#include <Arduino.h>
#include "DualG2HighPowerMotorShield.h"
#include <Servo.h> 
#include <Wire.h>

// 1. Setup the Motor Shield
  DualG2HighPowerMotorShield24v14 md;

// 2. Setup the Winch Parts
  Servo pawlServo;
  int PAWL_SERVO_PIN = 13;
  const int PAWL_SERVO_LOCK_POS = 70;
  const int PAWL_SERVO_OPEN_POS = 130;

// Variables Defined in Dylan's Guide
  String winchAction = "IDLE";
  float depthTarget = 0;
  String winchNextState = "";
  bool winchComplete = false;

// New Variables+++

  float drum_radius = (1.5/2) + (1/16) + (1/32);       // Inches adding width of cable !CHECK!
  const uint8_t I2C_MUX_ADDR = 0x70;          // Adafruit TCA9548A default address
  const uint8_t AS5600_ADDR = 0x36;           // Magnetic encoder I2C address
  const uint8_t AS5600_RAW_ANGLE_REG = 0x0C;  // RAW ANGLE high byte register
  const uint8_t WINCH_ENCODER_MUX_CH = 7;     // Wired on mux channel 7
  const uint8_t ROBOT_ENCODER_MUX_CH = 5;     // Wired on mux channel 5
  const int Winch_ENC_COUNTS = 4096;          // AS5600 is 12-bit
  const int WRAP_THRESH = Winch_ENC_COUNTS/2; // 2048 counts for the AS5600
  const int NOISE_THRESH = 4;                 // counts !CHECK!
  float winchHome = 0;
  float actual_winchPos = 0;
  float t_stamp = 0;                         //Timestamp used for non-blocking delays
  float winchSpeed = 0;
  float winchSpeedFiltered = 0;
  float old_winchPos = 0;
  float t_winch_old = 0;
  float jogStartPos = 0;
  uint32_t jogReliefDetectedMs = 0;
  uint32_t winchControlLastMs = 0;

  int Winch_lastPos = 0;                      // Delete if you use Quinton's library
  long Winch_totalSteps = 0;                  // Delete if you use Quinton's library

// Raise and Lower Speeds
  int raise_speed = 400;                      // Positive is up! Use change md.flipM2(true) to false if positive is down.
  int lower_speed = -200;                     // Negative should be down.
  const float JOG_UP_TARGET_SPEED_IN_S = 0.15f;
  const float JOG_UP_RELIEF_SPEED_IN_S = 0.05f;
  const float JOG_UP_RELIEF_DISTANCE_IN = 0.03f;
  const float JOG_UP_MAX_TRAVEL_IN = 0.50f; // half an inch of allowable travel, rehome position should be an inch below plate !CHECK!
  const uint32_t JOG_UP_RELIEF_DEBOUNCE_MS = 120;
  const float JOG_UP_CMD_RATE_PER_SEC = 350.0f;
  const float JOG_UP_KP_SPEED = 900.0f;
  const int JOG_UP_MIN_CMD = 35;

// 3. Setup AS5600 encoder over the TCA9548A I2C mux
// Winch Sub-Steps
  enum WinchSteps {IDLE, JOG_UP, UNLOCK, MOVING, BRAKE_AND_LOCK };
  WinchSteps currentStep = IDLE;

bool selectMuxChannel(uint8_t channel);
int readAs5600RawAngle(uint8_t muxChannel);
int readWinchEncoderCounts();
int readRobotEncoderCounts();

// Function prototypes
#pragma region Function Prototypes
float getEncoderDepth();
float getWinchSpeedFromPos(float currentPosInches);
float getWinchSpeedFromPosFiltered(float currentPosInches);
int winchRampToSpeedCmd(float desiredSpeed,
                        float measuredSpeed,
                        float dt_s,
                        float cmdRatePerSec,
                        float kP_speed);
void runWinchLogic();
#pragma endregion


void setup() {
  #pragma region Setup
  Serial.begin(115200);
  Wire.begin();
  md.init();           // Starts the motor shield
  md.enableDrivers();  // Turns the power on
  md.flipM2(true);
  pawlServo.attach(PAWL_SERVO_PIN);
  pawlServo.write(PAWL_SERVO_LOCK_POS);  // Start Locked
  // pawlServo.write(130);  // Start Unlocked
  Winch_lastPos = readWinchEncoderCounts();
  winchHome = getEncoderDepth();        // !CHECK! TEMPORARY: USE TIL WE HAVE A HOMING SWITCH. SETS HOME TO INITIAL START POSITION.
  winchControlLastMs = millis();
  #pragma endregion
}

void loop() {
  #pragma region Main Loop

  // Actual Encoder Depth 
  actual_winchPos = getEncoderDepth();
  winchSpeed = getWinchSpeedFromPos(actual_winchPos);
  winchSpeedFiltered = getWinchSpeedFromPosFiltered(actual_winchPos);

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

  // Testing code outside of idle case
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'j') {
        currentStep = JOG_UP;
        t_stamp = millis();
        jogStartPos = actual_winchPos;
        jogReliefDetectedMs = 0;
      }
      else if (c == 'u' && currentStep == IDLE) {md.setM2Speed(200);}
      else if (c == 'd'&& currentStep == IDLE) {md.setM2Speed(-100);}
      else if (c == 'x') { 
        md.setM2Speed(0);
        currentStep = BRAKE_AND_LOCK;
      }
    }
    //End testing code
    #pragma endregion
}
//Winch speed & Pos functions
  #pragma region WinchSpeed&PosFunctions
  bool selectMuxChannel(uint8_t channel) {
    if (channel > 7) return false;

    Wire.beginTransmission(I2C_MUX_ADDR);
    Wire.write(1 << channel);
    return (Wire.endTransmission() == 0);
  }

  int readAs5600RawAngle(uint8_t muxChannel) {
    if (!selectMuxChannel(muxChannel)) {
      Serial.print("Mux channel select failed: ");
      Serial.println(muxChannel);
      return -1;
    }

    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_RAW_ANGLE_REG);
    if (Wire.endTransmission(false) != 0) {
      Serial.print("AS5600 register select failed on channel ");
      Serial.println(muxChannel);
      return -1;
    }

    const uint8_t requestedBytes = 2;
    uint8_t received = Wire.requestFrom((int)AS5600_ADDR, (int)requestedBytes);
    if (received != requestedBytes) {
      Serial.print("AS5600 read failed on channel ");
      Serial.println(muxChannel);
      return -1;
    }

    int rawAngle = ((int)Wire.read() << 8) | Wire.read();
    return rawAngle & 0x0FFF;
  }

  int readWinchEncoderCounts() {
    return readAs5600RawAngle(WINCH_ENCODER_MUX_CH);
  }

  int readRobotEncoderCounts() {
    return readAs5600RawAngle(ROBOT_ENCODER_MUX_CH);
  }

  float getEncoderDepth() {
    int currentPos = readWinchEncoderCounts();
    if (currentPos < 0) {
      return ((float)Winch_totalSteps / (float)Winch_ENC_COUNTS) * (2.0f * PI * drum_radius);
    }

    int delta = currentPos - Winch_lastPos;

    // unwrap
    if (delta > WRAP_THRESH) {
      delta -= Winch_ENC_COUNTS;
    } else if (delta < -WRAP_THRESH) {
      delta += Winch_ENC_COUNTS;
    }

    // noise gate
    int absDelta = abs(delta);
    if (absDelta > NOISE_THRESH) { // !CHECK! keep an upper bound here if noise spikes show up
      // Serial.print("Noise Thresh Triggered: ");
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
  #pragma endregion 

// Winch Control Functions 
  #pragma region WinchControlFunctions
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

  #pragma endregion

void runWinchLogic() {
  #pragma region runWinchLogic Function
  switch (currentStep) {

    case IDLE:
      // Do nothing until the Manager sends a command
      // Default Case is idle. Add individual testing commands BELOW:

      //Testing Code:
      if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'j') {
        currentStep = JOG_UP;
        t_stamp = millis();
        jogStartPos = actual_winchPos;
        jogReliefDetectedMs = 0;
      }
      else if (c == 'u') {md.setM2Speed(200);}
      else if (c == 'd') {md.setM2Speed(-100);}
      else if (c == 'x') { md.setM2Speed(0);}
      }
      //End Testing Code


      break;

    case JOG_UP:
      // Nudge upward at the lowest command that still produces real motion.
      {
        uint32_t nowMs = millis();
        float dt_s = (nowMs - winchControlLastMs) / 1000.0f;
        if (dt_s <= 0.0f) dt_s = 0.001f;
        winchControlLastMs = nowMs;

        int jogCmd = winchRampToSpeedCmd(JOG_UP_TARGET_SPEED_IN_S,
                                         winchSpeedFiltered,
                                         dt_s,
                                         JOG_UP_CMD_RATE_PER_SEC,
                                         JOG_UP_KP_SPEED);

        if (jogCmd > 0 && jogCmd < JOG_UP_MIN_CMD) jogCmd = JOG_UP_MIN_CMD;
        md.setM2Speed(jogCmd);

        float jogTravel = actual_winchPos - jogStartPos;
        bool reliefEvidence =
          (jogTravel >= JOG_UP_RELIEF_DISTANCE_IN) &&
          (winchSpeedFiltered >= JOG_UP_RELIEF_SPEED_IN_S);

        if (reliefEvidence) {
          if (jogReliefDetectedMs == 0) jogReliefDetectedMs = nowMs;
          if ((nowMs - jogReliefDetectedMs) >= JOG_UP_RELIEF_DEBOUNCE_MS) {
            pawlServo.write(PAWL_SERVO_OPEN_POS);
            currentStep = UNLOCK;
            t_stamp = nowMs;
          }
        } else {
          jogReliefDetectedMs = 0;
        }

        if (jogTravel >= JOG_UP_MAX_TRAVEL_IN) {
          pawlServo.write(PAWL_SERVO_OPEN_POS);
          md.setM2Speed(0);
          currentStep = UNLOCK;
          t_stamp = nowMs;
        }
      }
      
      break;

    case UNLOCK:
      //Unlock pawl arm with the servo
      pawlServo.write(PAWL_SERVO_OPEN_POS);  // Disengage servo pawl
      if (millis() - t_stamp >= 300){ //!CHECK! change to not hard coded variable
        md.setM2Speed(0);
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
      pawlServo.write(PAWL_SERVO_LOCK_POS);  // Re-engage the mechanical lock
      delay(500);
      winchComplete = true;
      currentStep = IDLE;
      Serial.println("Finished!");
      break;
  }
  #pragma endregion
}
