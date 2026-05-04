/************************************************************
 * Node (Arduino Mega) - Node1 FSM + Comms
 * Mechatronics Simulation of Overhead Hazards
 *
 * Requirements Source:
 *  - RequirementsAndStateTransition.md
 *
 * Hardware:
 *  - Arduino Mega
 *  - XBee on Serial2 (pins RX2=17, TX2=16 on Mega) 
 *
 * Critical Requirement:
 *  - Change NODE_ID in ONE place to convert node1 <-> node2.
 *
 * Wireless Regime:
 *  - Manager <-> Node uses framed packets: < ... >
 *  - Node-to-node emergency stop uses raw: !X!  (NOT framed)
 *
 * Added requirement:
 *  - TEST_MODE state that simulates a complete run handshake for comm testing.
 *    To use: set `currentState = TEST_MODE;` below before upload.
 ************************************************************/

#pragma region Includes/Variable_Declarations/Definitions
/* ========================= Includes ========================= */
  #include <Arduino.h>
  #include <Encoder.h>
  #include <Wire.h>
  #include <math.h>
  #include <string.h>
  #include <stdlib.h>
  #include <DriveControl.h>
  #include <EncoderMux.h>
  #include <DualG2HighPowerMotorShield.h>

/* ========================= Node Identity ========================= */
  #define NODE_ID 1               // <-- CHANGE TO 2 to make this node behave as Node2

/* ========================= Serial Definitions ========================= */
  #define xbeeSerial Serial2      // Mega: Serial2 (TX2=16, RX2=17)  

/* ========================= Protocol ========================= */
  const char START_CMD = 'G';     // <G>
  const char ESTOP_CMD = 'X';     // <X> (nodes also accept <x>)
  const char RUN_S = 'S';
  const char RUN_T = 'T';
  const char RUN_I = 'I';
  const char WINCH_CMD = 'W';     // <W,depth1,depth2>

/* ========================= Buffers ========================= */
  static const uint16_t XBEE_FRAME_MAX = 240;

  static bool inAngleFrame = false;
  static char angleBuf[XBEE_FRAME_MAX];
  static uint16_t angleLen = 0;

  /* Parser for raw node-to-node "!X!" */
  static uint8_t bangState = 0;   // 0=idle, 1=got '!', 2=got '!' + cmd
  static char bangCmd = 0;

/* ========================= State Definitions ========================= */
  enum NodeState : uint8_t {
    /* Added test state */
    TEST_MODE = 0,

    /* Requirements states */
    WAIT_FOR_COMMAND,
    WAIT_FOR_WINCH_ACTION,
    SET_WINCH_TO_COMMAND_HEIGHT,
    SEND_READY_TO_MANAGER,
    WAIT_FOR_GO,
    WAIT_FOR_START_DELAY,
    DRIVING,
    WAIT_LOWER_DELAY,
    WAIT_HOIST_DELAY,
    RESUME_DRIVING_AFTER_HOIST,
    POST_TRAVERSAL_DECISION,
    DRIVE_TO_OPPOSITE,
    SEND_FINISH,
    EMERGENCY_STOP
  };

  NodeState currentState = WAIT_FOR_COMMAND; // <-- set to TEST_MODE for comm simulation testing. WAIT_FOR_COMMAND for normal opperation

/* ========================= Run Parameters ========================= */
  struct RunParams {
    char runType = 0;            // S/T/I

    float speedCmd = 0.0f;
    float winchHeightCmd = 0.0f;

    uint32_t startDelayMs = 0;
    uint32_t lowerDelayMs = 0;
    uint32_t hoistDelayMs = 0;

    float hoistStopPosCmd = 0.0f;     // where robot stops to lower/hoist
    float lowerLoadDepthCmd = 0.0f;   // how far load goes down during stop

    bool lowerAndHoist = false;       // whether to do the mid-rail lower/hoist
    uint8_t startSide = 0;            // desired finish side for this run / start side for the next run
  };

  RunParams run;

/* ========================= Winch Variables ========================= */
  enum WinchAction : uint8_t {
    WINCH_NONE = 0,
    WINCH_LOWER_TO_DEPTH,
    WINCH_HOIST_TO_HEIGHT,
    WINCH_REHOME,
    WINCH_PREPARE_FOR_TRAVERSE
  };

  WinchAction winchAction = WINCH_NONE;
  float winchTarget = 0.0f;           // meaning depends on action (depth/height)
  bool winchComplete = false;
  NodeState winchNextState = WAIT_FOR_COMMAND;

/* ========================= Drive / Side / Flags ========================= */
  enum Side : uint8_t { SIDE_0 = 0, SIDE_1 = 1 };
  Side startSide = SIDE_0;          // desired finish side for this run / start side for the next run
  Side currentSide = SIDE_0;        // actual tracked side of the robot
  Side runPhysicalStartSide = SIDE_0; // actual side the robot was on when the current run began

  bool slowdown = false;
  bool goReceived = false;

  /* Track whether we have already recorded the "finished" timestamp for this run */
  bool finishedTimestampRecorded = false;

  /* Mid-run hoist sequence tracking */
  bool hoistStopReached = false;        // we stopped mid-rail already
  bool lowerDelayTimerRunning = false;
  bool hoistDelayTimerRunning = false;

/* ========================= Timing / Measurements ========================= */
  uint32_t tGoReceivedMs = 0;
  uint32_t tDriveStartedMs = 0;
  uint32_t tHoistStopMs = 0;
  uint32_t tFinishedMs = 0;

  uint32_t startDelayStartMs = 0;
  uint32_t lowerDelayStartMs = 0;
  uint32_t hoistDelayStartMs = 0;

  /* Measured variables (stubs until you implement sensors/encoders) */
  float measuredAvgSpeedCenter = 0.0f;
  float measuredWinchHeightBeforeTranslation = 0.0f;
  float measuredWinchMaxDepth = 0.0f;

/* ========================= Pin Placeholders ========================= */
  const uint8_t PIN_BATT_SENSE = A2;            // !CHECK! battery divider input pin, change to MUX
  const uint8_t PIN_DRIVE_CONTACT_SWITCH = 3;   // shared rail-end contact switch for both sides

/* ========================= Drive Hardware ========================= */
  unsigned char M1nSLEEP = 5;
  unsigned char M1DIR = 7;
  unsigned char M1PWM = 9;
  unsigned char M1nFAULT = 6;
  unsigned char M1CS = A0;
  unsigned char M2nSLEEP = 4;
  unsigned char M2DIR = 8;
  unsigned char M2PWM = 10;
  unsigned char M2nFAULT = 12;
  unsigned char M2CS = A1;

  DualG2HighPowerMotorShield md(M1nSLEEP,
                                M1DIR,
                                M1PWM,
                                M1nFAULT,
                                M1CS,
                                M2nSLEEP,
                                M2DIR,
                                M2PWM,
                                M2nFAULT,
                                M2CS);
  Encoder driveEncoder(19, 18);

/* ========================= Drive Configuration ========================= */
  const bool FLIP_DRIVE_MOTOR = false;

  const float SIDE_0_POSITION_FT = 0.0f;  // !CHECK! test-rail coordinate for side 0
  const float SIDE_1_POSITION_FT = 4.0f;  // !CHECK! test-rail coordinate for side 1

  const float DRIVE_ENCODER_COUNTS_PER_MOTOR_REV = 1200.0f;
  const float DRIVE_GEAR_RATIO = 0.5f;
  const float DRIVE_WHEEL_RADIUS_IN = 0.6f;

  const uint8_t DRIVE_REAL_POSITION_MUX_CH = 0;
  const float REAL_POSITION_WHEEL_DIAMETER_IN = 0.875f;
  const int AS5600_COUNTS_PER_REV = 4096;
  const int AS5600_WRAP_THRESH = AS5600_COUNTS_PER_REV / 2;
  const int AS5600_NOISE_THRESH = 1;

  const int MAX_DRIVE_MOTOR_CMD = 400;
  const uint32_t DRIVE_CONTROL_INTERVAL_MS = 20;
  const uint32_t DRIVE_CONTACT_SWITCH_ARM_DELAY_MS = 800; // !CHECK! ignore shared endstop briefly after drive start
  const float DRIVE_RETURN_END_BUFFER_IN = 0.60f;         // !CHECK! extend post-middle-stop return trajectory slightly to absorb slip
  const float DRIVE_TARGET_TOLERANCE_IN = 0.5f;
  const float DRIVE_STOP_VELOCITY_TOLERANCE_IN_S = 0.20f;
  const uint32_t DRIVE_TARGET_SETTLE_TIME_MS = 750;
  const float DRIVE_TRAJECTORY_MAX_ACCEL_IN_PER_S2 = 9.0f; //!CHECK!
  const float DRIVE_RAMP_KP = 170.0f; //!CHECK!
  const float DRIVE_RAMP_KI = 15.0f; //!CHECK!
  const float DRIVE_RAMP_KD = 20.0f;
  const float DRIVE_RAMP_INTEGRAL_WINDOW_IN = 4.0f;
  const float DRIVE_REAL_POSITION_FIR_COEFFS[] = {1.0f};
  const float DRIVE_CENTER_SAMPLE_HALF_WINDOW_IN = 6.0f; // !CHECK! center-speed averaging window
  const float DRIVE_SLOWDOWN_WINDOW_IN = 12.0f;          // !CHECK! informational slowdown window

/* ========================= Drive Runtime ========================= */
  enum DriveMode : uint8_t {
    DRIVE_MODE_IDLE = 0,
    DRIVE_MODE_TRAJECTORY
  };

  struct DriveSnapshot {
    long driveCounts = 0;
    long realCounts = 0;
    float drivePosIn = 0.0f;
    float realPosRawIn = 0.0f;
    float realPosFirIn = 0.0f;
    float driveVelInPerS = 0.0f;
    float realVelInPerS = 0.0f;
  };

  DriveSnapshot driveSnapshot;
  DriveSnapshot previousDriveSnapshot;

  DriveMode driveMode = DRIVE_MODE_IDLE;
  TrajectoryProfile activeDriveTrajectory = {};
  Side driveTargetSide = SIDE_0;

  int appliedDriveMotorCmd = 0;
  float steadyCruiseSpeedInPerS = 12.0f;
  float activeDesiredDrivePositionIn = 0.0f;
  float driveTargetPositionIn = 0.0f;

  uint32_t driveTrajectoryStartMs = 0;
  uint32_t driveLastControlMs = 0;
  uint32_t driveTargetSettledSinceMs = 0;
  uint32_t driveContactSwitchArmAtMs = 0;

  float centerSpeedSumInPerS = 0.0f;
  uint32_t centerSpeedSampleCount = 0;

  bool driveContactSwitchWasPressed = false;

#pragma endregion

#pragma region Function_Prototypes
/* ========================= Function Prototypes ========================= */
  /* ---- Wireless ---- */
    void checkWireless();
    void handleAngleChar(char c);
    void handleAnglePayload(char *payload);
    void handleBangChar(char c);

    void xbeeSendPayload(const char *payload);
    void sendAckToManager();
    void sendReadyToManager();
    void sendFinishToManager();
    void sendBatteryWarningToManager();
    void broadcastEmergencyStopToOtherNode();

  /* ---- Parsing ---- */
    bool isWs(char c);
    void sanitizeInPlace(char *s);
    uint8_t splitCsv(char *line, char *tokens[], uint8_t maxTokens);
    float feetToInches(float feet);
    float inchesToFeet(float inches);

    bool parseRunCommandPacket(const char *payloadIn);
    bool parseLowerWinchPacket(const char *payloadIn, float &depthOut);

  /* ---- Battery / Global safety ---- */
    bool battery_too_low();
    void enterEmergencyStop();

  /* ---- Winch (stubs you will implement) ---- */
    void winchUpdate();                 // must be non-blocking
    void winchResetVars();
    void winchStartAction(WinchAction action, float target, NodeState nextState);

    void stopWinchMotor();              // stub
    void lockWinchPawl();               // stub
    void unlockWinchPawl();             // stub
    float readWinchPosition();          // stub (encoder->feet or similar)
    void resetWinchEncoder();           // stub
    bool winchAtTarget(float target);   // stub

  /* ---- Drive (stubs you will implement) ---- */
    void driveStartTowardOppositeSide();  // non-blocking start
    void driveStartTowardSide(Side s);    // non-blocking start
    void driveStartTowardSideWithEndBuffer(Side s, float extraEndBufferIn);
    void driveStartTowardRailPositionFeet(float targetFeet, Side targetSideAtEndstop);
    void driveUpdate();                  // non-blocking update loop
    void stopDriveMotors();              // stub

    float robotPositionAlongRail();      // stub (feet or similar)
    bool reachedSlowdownPosition();      // stub
    bool contactSwitchHit();             // stub + side logic
    void resetDriveEncoders();           // stub

  /* ---- Drive helpers ---- */
    void configureDriveControl();
    void applyDriveRampSettings();
    void armDriveContactSwitchDelay();
    bool zeroDriveSensorsToAbsolutePosition(float newZeroPositionIn);
    void refreshDriveSensors(float dtSeconds);
    void applyDriveMotorCommand(int speedCmd);
    bool startDriveTrajectoryToPositionIn(float targetPositionIn, Side targetSideIn);
    float getDriveEncoderDistanceIn();
    float getSidePositionFeet(Side side);
    float getSidePositionInches(Side side);
    float getRailCenterPositionInches();
    float applyDriveEndBufferInches(float baseTargetIn, float extraEndBufferIn);
    bool hasDriveSettledAtTarget();
    bool hasReachedRailPositionFeet(float targetFeet);
    bool isDriveContactSwitchPressed();
    void updateCenterSpeedMeasurement();
    Side getOppositeSide(Side side);
    Side getDriveResetReferenceSide();

  /* ---- Run Logic helpers ---- */
    bool destinationIsStartSide();       // stub decision function
    void resetRunRuntimeFlags();
    void computeMeasuredValues();        // stub

  /* ---- TEST MODE ---- */
    void testModeService();

#pragma endregion

#pragma region Function_Definitions

  #pragma region Wireless_Communication_Functions
/* ========================= Utility ========================= */
  bool isWs(char c) { return (c == ' ' || c == '\t' || c == '\r' || c == '\n'); }
  float feetToInches(float feet) { return feet * 12.0f; }
  float inchesToFeet(float inches) { return inches / 12.0f; }

  void sanitizeInPlace(char *s) {
    // remove '<','>','\r','\n' and trim whitespace
    uint16_t w = 0;
    for (uint16_t r = 0; s[r] != 0; r++) {
      char c = s[r];
      if (c == '<' || c == '>' || c == '\r' || c == '\n') continue;
      s[w++] = c;
    }
    s[w] = 0;

    // trim left
    uint16_t i = 0;
    while (s[i] && isWs(s[i])) i++;
    if (i) {
      uint16_t j = 0;
      while (s[i]) s[j++] = s[i++];
      s[j] = 0;
    }

    // trim right
    int len = (int)strlen(s);
    while (len > 0 && isWs(s[len - 1])) s[--len] = 0;
  }

  uint8_t splitCsv(char *line, char *tokens[], uint8_t maxTokens) {
    sanitizeInPlace(line);
    if (!line || !tokens || maxTokens == 0) return 0;
    if (line[0] == 0) return 0;

    uint8_t n = 0;
    tokens[n++] = line;
    for (char *p = line; *p && n < maxTokens; p++) {
      if (*p == ',') {
        *p = 0;
        sanitizeInPlace(tokens[n - 1]);
        tokens[n++] = p + 1;
      }
    }
    sanitizeInPlace(tokens[n - 1]);
    return n;
  }

/* ========================= Wireless Send ========================= */
  void xbeeSendPayload(const char *payload) {
    xbeeSerial.print('<');
    xbeeSerial.print(payload);
    xbeeSerial.print('>');
    xbeeSerial.print('\n'); // receiver ignores newline
  }

  void sendAckToManager() {
    Serial.println("Sending Ack to manager"); //Debug print out
    char msg[8];
    snprintf(msg, sizeof(msg), "A%d", NODE_ID);
    xbeeSendPayload(msg);
  }

  void sendReadyToManager() {
    Serial.println("Sending Ready to manager"); //Debug print out
    char msg[8];
    snprintf(msg, sizeof(msg), "R%d", NODE_ID);
    xbeeSendPayload(msg);
  }

  void sendBatteryWarningToManager() {
    char msg[8];
    snprintf(msg, sizeof(msg), "B%d", NODE_ID);
    xbeeSendPayload(msg);
  }

  void broadcastEmergencyStopToOtherNode() {
    // Per requirements: raw "!X!" so manager (which parses only <...>) ignores it.
    xbeeSerial.print("!X!");
    xbeeSerial.print('\n');
  }

  void sendFinishToManager() {
    // Finished handshake required to start with: <F#, ...>
    // Token order (manager expects this):
    // F#, avgSpeedCenter, measuredWinchHeight, measuredMaxDepth,
    //     tGoToMoveMs, tGoToHoistStopMs, tGoToFinishedMs,
    //     startDelayCmdMs, lowerDelayCmdMs, hoistDelayCmdMs

    Serial.println("Sending Finish to manager"); //Debug print out

    char msg[XBEE_FRAME_MAX];

    char a[16], b[16], c[16];
    dtostrf(measuredAvgSpeedCenter, 0, 3, a);
    dtostrf(measuredWinchHeightBeforeTranslation, 0, 3, b);
    dtostrf(measuredWinchMaxDepth, 0, 3, c);

    uint32_t tGoToMove = (tDriveStartedMs >= tGoReceivedMs) ? (tDriveStartedMs - tGoReceivedMs) : 0;
    uint32_t tGoToHoistStop = (tHoistStopMs >= tGoReceivedMs) ? (tHoistStopMs - tGoReceivedMs) : 0;
    uint32_t tGoToFinished = (tFinishedMs >= tGoReceivedMs) ? (tFinishedMs - tGoReceivedMs) : 0;

    snprintf(msg, sizeof(msg),
            "F%d,%s,%s,%s,%lu,%lu,%lu,%lu,%lu,%lu",
            NODE_ID,
            a, b, c,
            (unsigned long)tGoToMove,
            (unsigned long)tGoToHoistStop,
            (unsigned long)tGoToFinished,
            (unsigned long)run.startDelayMs,
            (unsigned long)run.lowerDelayMs,
            (unsigned long)run.hoistDelayMs);

    xbeeSendPayload(msg);
  }

/* ========================= Wireless Receive ========================= */
  void checkWireless() {

    while (xbeeSerial.available()) {

      char c = (char)xbeeSerial.read();

      // Handle raw "!X!" parser first
      if (c == '!' || bangState != 0) {
        handleBangChar(c);
        // Still allow angle framing to work even if '!' appears elsewhere,
        // but for this project we treat '!' sequences specially.
        continue;
      }

      // Standard <...> frame parser
      handleAngleChar(c);
    }
  }

  void handleBangChar(char c) {
    // Recognize exactly: ! X !
    // bangState:
    //   0: waiting for '!'
    //   1: got '!' waiting for command
    //   2: got command waiting for ending '!'
    if (bangState == 0) {
      if (c == '!') {
        bangState = 1;
        bangCmd = 0;
      }
      return;
    }

    if (bangState == 1) {
      // store cmd
      bangCmd = c;
      bangState = 2;
      return;
    }

    if (bangState == 2) {
      if (c == '!') {
        // Complete message
        if (bangCmd == 'X' || bangCmd == 'x') {
          enterEmergencyStop();
        }
      }
      // reset regardless
      bangState = 0;
      bangCmd = 0;
    }
  }

  void handleAngleChar(char c) {
    if (!inAngleFrame) {
      if (c == '<') {
        inAngleFrame = true;
        angleLen = 0;
      }
      return;
    }

    if (c == '>') {
      angleBuf[angleLen] = 0;
      inAngleFrame = false;
      handleAnglePayload(angleBuf);
      return;
    }

    if (c == '\r' || c == '\n') return;

    if (angleLen + 1 < XBEE_FRAME_MAX) {
      angleBuf[angleLen++] = c;
    } else {
      // overflow protection
      inAngleFrame = false;
      angleLen = 0;
    }
  }

  void handleAnglePayload(char *payload) {
    sanitizeInPlace(payload);
    if (payload[0] == 0) return;

    /* -------- Global background event: emergency stop received -------- */
    if ((payload[0] == 'X' || payload[0] == 'x') && payload[1] == 0) {
      enterEmergencyStop();
      return;
    }

    /* -------- TEST MODE override -------- */
    if (currentState == TEST_MODE) {
      // testModeService() will read flags set by parsing below, but we keep it simple:
      // In TEST_MODE we still parse run packets and go packets using the same functions,
      // but do not transition into the real FSM.
      // We'll store run params and set goReceived; testModeService() will respond.
    }

    /* -------- Lower winch command: <W, depth1, depth2> -------- */
    if (payload[0] == WINCH_CMD) {
      float d = 0.0f;
      if (parseLowerWinchPacket(payload, d)) {
        if (currentState != TEST_MODE) {
          // Per requirements: store requested lower distance, set winch variables,
          // winchNextState=WAIT_FOR_COMMAND, transition WAIT_FOR_WINCH_ACTION.
          winchStartAction(WINCH_LOWER_TO_DEPTH, d, WAIT_FOR_COMMAND);
          currentState = WAIT_FOR_WINCH_ACTION;
        }
        // In TEST_MODE: no required response; simulation can just log via Serial.
      }
      return;
    }

    /* -------- Run command packet received -------- */
    if (payload[0] == RUN_S || payload[0] == RUN_T || payload[0] == RUN_I) {
      // Only accept a run packet if we are waiting for command, or in TEST_MODE
      if (currentState == WAIT_FOR_COMMAND || currentState == TEST_MODE) {
        if (parseRunCommandPacket(payload)) {
          sendAckToManager(); // <A#> per requirements

          if (currentState != TEST_MODE) {
            // Set winch variables: rehome; winchNextState = SET_WINCH_TO_COMMAND_HEIGHT
            winchStartAction(WINCH_REHOME, 0.0f, SET_WINCH_TO_COMMAND_HEIGHT);
            currentState = WAIT_FOR_WINCH_ACTION;
          }
        }
      }
      return;
    }

    /* -------- Go command: <G> -------- */
    if (payload[0] == START_CMD && payload[1] == 0) {
      goReceived = true;
      return;
    }

    // Ignore everything else
  }

/* ========================= Parsing Packets ========================= */
  bool parseLowerWinchPacket(const char *payloadIn, float &depthOut) {
    // payloadIn is like "W,depth1,depth2"
    char copy[XBEE_FRAME_MAX];
    strncpy(copy, payloadIn, sizeof(copy));
    copy[sizeof(copy) - 1] = 0;

    static const uint8_t MAXTOK = 6;
    char *tok[MAXTOK];
    uint8_t n = splitCsv(copy, tok, MAXTOK);
    if (n < 2) return false;
    if (tok[0][0] != 'W') return false;

    float d1 = (float)atof(tok[1]);
    float d2 = (n >= 3) ? (float)atof(tok[2]) : d1;

    // Per requirements: node1 reads depth1, node2 reads depth2
    depthOut = (NODE_ID == 1) ? d1 : d2;
    return true;
  }

  bool parseRunCommandPacket(const char *payloadIn) {
    // Expected from Manager:
    // S/T:
    //   <S/T,speed,winchHeight,startDelayMs,lowerDelayMs,hoistDelayMs,hoistStopPos,lowerLoadDepth,lowerAndHoist,startSide>
    // I:
    //   <I, s1,h1,sd1,ld1,hd1,sp1,dd1,lah1,ss1,  s2,h2,sd2,ld2,hd2,sp2,dd2,lah2,ss2>
    //

    Serial.println("Parsing run command packet"); //debug print out

    char copy[XBEE_FRAME_MAX];
    strncpy(copy, payloadIn, sizeof(copy));
    copy[sizeof(copy) - 1] = 0;

    static const uint8_t MAXTOK = 24;
    char *tok[MAXTOK];
    uint8_t n = splitCsv(copy, tok, MAXTOK);
    if (n < 2) return false;

    char t = tok[0][0];
    if (!(t == RUN_S || t == RUN_T || t == RUN_I)) return false;

    // Single-run rule: if runType == S and this is node2, ignore entirely.
    if (t == RUN_S && NODE_ID != 1) {
      return false;
    }

    RunParams rp;
    rp.runType = t;

    if (t == RUN_S || t == RUN_T) {
      if (n != 10) return false;

      rp.speedCmd = (float)atof(tok[1]);
      rp.winchHeightCmd = (float)atof(tok[2]);
      rp.startDelayMs = (uint32_t)atol(tok[3]);
      rp.lowerDelayMs = (uint32_t)atol(tok[4]);
      rp.hoistDelayMs = (uint32_t)atol(tok[5]);
      rp.hoistStopPosCmd = (float)atof(tok[6]);
      rp.lowerLoadDepthCmd = (float)atof(tok[7]);
      rp.lowerAndHoist = (atoi(tok[8]) != 0);
      rp.startSide = (uint8_t)atoi(tok[9]);

      run = rp;
      startSide = (run.startSide == 0) ? SIDE_0 : SIDE_1;
      return true;
    }

    // Independent
    if (t == RUN_I) {
      if (n != 19) return false;

      // node1 uses fields 1..9, node2 uses 10..18
      int base = (NODE_ID == 1) ? 1 : 10;

      rp.speedCmd = (float)atof(tok[base + 0]);
      rp.winchHeightCmd = (float)atof(tok[base + 1]);
      rp.startDelayMs = (uint32_t)atol(tok[base + 2]);
      rp.lowerDelayMs = (uint32_t)atol(tok[base + 3]);
      rp.hoistDelayMs = (uint32_t)atol(tok[base + 4]);
      rp.hoistStopPosCmd = (float)atof(tok[base + 5]);
      rp.lowerLoadDepthCmd = (float)atof(tok[base + 6]);
      rp.lowerAndHoist = (atoi(tok[base + 7]) != 0);
      rp.startSide = (uint8_t)atoi(tok[base + 8]);

      run = rp;
      startSide = (run.startSide == 0) ? SIDE_0 : SIDE_1;
      return true;
    }

    return false;
  }

  #pragma endregion

  #pragma region Battery/Emergency_Stop_Functions
  /* ========================= Battery / Emergency Stop ========================= */
    bool battery_too_low() {
      // STUB: implement your battery divider + threshold here.
      // Typical approach:
      //  - read analog PIN_BATT_SENSE
      //  - convert to voltage
      //  - compare to threshold
      //
      // For now: always false (safe default for development).
      (void)PIN_BATT_SENSE;
      return false;
    }

    void enterEmergencyStop() {
      currentState = EMERGENCY_STOP;
      stopDriveMotors();
      stopWinchMotor();
      lockWinchPawl();
    }
  
  #pragma endregion

  #pragma region Winch_Functions
/* ========================= Winch Stubs ========================= */
  void winchStartAction(WinchAction action, float target, NodeState nextState) {
    winchAction = action;
    winchTarget = target;
    winchNextState = nextState;
    winchComplete = false;
  }

  void winchResetVars() {
    winchAction = WINCH_NONE;
    winchTarget = 0.0f;
    winchComplete = false;
  }

  void winchUpdate() {
    // REQUIRED: non-blocking winch state machine that sets winchComplete=true when done.
    //
    // Implementations required by requirements:
    //  - WINCH_REHOME:
    //      run winch upwards until contact switch hit
    //      set winch encoder = 0
    //  - WINCH_LOWER_TO_DEPTH:
    //      run winch up slightly (barely move encoder)
    //      unlock pawl via servo
    //      motor off then run down
    //      stop at target, motor off, lock pawl
    //  - WINCH_HOIST_TO_HEIGHT:
    //      unlock pawl, raise to target, lock pawl
    //  - WINCH_PREPARE_FOR_TRAVERSE:
    //      if destinationIsStartSide()==true && lowerAndHoist==false:
    //         hoist close to top so load can traverse back safely
    //
    // PLACEHOLDER behavior:
    //  - Immediately mark complete so higher-level FSM can be tested.
    //
    // !!! Replace this with real non-blocking winch logic. !!!
    winchComplete = true;
  }

  void stopWinchMotor() {
    // STUB: turn off winch motor driver output
  }

  void lockWinchPawl() {
    // STUB: servo to lock pawl
  }

  void unlockWinchPawl() {
    // STUB: servo to unlock pawl
  }

  float readWinchPosition() {
    // STUB: return current winch position (feet or encoder->feet)
    return 0.0f;
  }

  void resetWinchEncoder() {
    // STUB: set winch encoder counts to 0
  }

  bool winchAtTarget(float target) {
    // STUB: compare readWinchPosition() to target with tolerance
    (void)target;
    return true;
  }

  #pragma endregion

  #pragma region Drive_Functions

/* ========================= Drive Control ========================= */
  Side getOppositeSide(Side side) {
    return (side == SIDE_0) ? SIDE_1 : SIDE_0;
  }

  float getSidePositionFeet(Side side) {
    return (side == SIDE_0) ? SIDE_0_POSITION_FT : SIDE_1_POSITION_FT;
  }

  float getSidePositionInches(Side side) {
    return feetToInches(getSidePositionFeet(side));
  }

  float getRailCenterPositionInches() {
    return 0.5f * (getSidePositionInches(SIDE_0) + getSidePositionInches(SIDE_1));
  }

  float applyDriveEndBufferInches(float baseTargetIn, float extraEndBufferIn) {
    if (extraEndBufferIn <= 0.0f) {
      return baseTargetIn;
    }
    return (baseTargetIn >= driveSnapshot.realPosFirIn)
               ? (baseTargetIn + extraEndBufferIn)
               : (baseTargetIn - extraEndBufferIn);
  }

  bool isDriveContactSwitchPressed() {
    return digitalRead(PIN_DRIVE_CONTACT_SWITCH) == LOW;
  }

  void applyDriveRampSettings() {
    setRampControlGains(DRIVE_RAMP_KP, DRIVE_RAMP_KI, DRIVE_RAMP_KD);
    setRampIntegralWindow(DRIVE_RAMP_INTEGRAL_WINDOW_IN);
  }

  void armDriveContactSwitchDelay() {
    driveContactSwitchArmAtMs = millis() + DRIVE_CONTACT_SWITCH_ARM_DELAY_MS;
  }

  void configureDriveControl() {
    EncoderMuxConfig realPositionConfig = makeDefaultAs5600MuxConfig(DRIVE_REAL_POSITION_MUX_CH);
    realPositionConfig.countsPerRevolution = AS5600_COUNTS_PER_REV;
    realPositionConfig.wrapThreshold = AS5600_WRAP_THRESH;
    realPositionConfig.noiseThreshold = AS5600_NOISE_THRESH;

    setDriveRealPositionSensorParameters(realPositionConfig.muxChannel,
                                         REAL_POSITION_WHEEL_DIAMETER_IN * 0.5f,
                                         realPositionConfig.countsPerRevolution,
                                         realPositionConfig.wrapThreshold,
                                         realPositionConfig.noiseThreshold,
                                         realPositionConfig.muxAddress,
                                         realPositionConfig.sensorAddress,
                                         realPositionConfig.rawAngleRegister);

    setDriveRealPositionFirParameters(DRIVE_REAL_POSITION_FIR_COEFFS,
                                      (uint8_t)(sizeof(DRIVE_REAL_POSITION_FIR_COEFFS) /
                                                sizeof(DRIVE_REAL_POSITION_FIR_COEFFS[0])));
    applyDriveRampSettings();
  }

  bool zeroDriveSensorsToAbsolutePosition(float newZeroPositionIn) {
    applyDriveMotorCommand(0);
    driveMode = DRIVE_MODE_IDLE;
    activeDriveTrajectory = {};

    driveEncoder.write(0);
    resetRampController();

    const bool sensorResetOk = resetDriveRealPositionSensor(newZeroPositionIn);

    driveSnapshot.driveCounts = 0;
    driveSnapshot.realCounts = getDriveRealPositionCounts();
    driveSnapshot.drivePosIn = 0.0f;
    driveSnapshot.realPosRawIn = getDriveRealPosition();
    driveSnapshot.realPosFirIn = getDriveRealPositionFir();
    driveSnapshot.driveVelInPerS = 0.0f;
    driveSnapshot.realVelInPerS = 0.0f;
    previousDriveSnapshot = driveSnapshot;

    activeDesiredDrivePositionIn = driveSnapshot.realPosFirIn;
    driveTargetPositionIn = driveSnapshot.realPosFirIn;
    driveTargetSettledSinceMs = 0;
    driveTrajectoryStartMs = millis();
    driveLastControlMs = millis();
    armDriveContactSwitchDelay();
    driveContactSwitchWasPressed = isDriveContactSwitchPressed();

    return sensorResetOk;
  }

  float getDriveEncoderDistanceIn() {
    const long counts = driveEncoder.read();
    const float wheelRevs =
        ((float)counts) / (DRIVE_GEAR_RATIO * DRIVE_ENCODER_COUNTS_PER_MOTOR_REV);
    return wheelRevs * (2.0f * PI * DRIVE_WHEEL_RADIUS_IN);
  }

  void refreshDriveSensors(float dtSeconds) {
    previousDriveSnapshot = driveSnapshot;

    updateDriveRealPositionFromSensor();

    driveSnapshot.driveCounts = driveEncoder.read();
    driveSnapshot.realCounts = getDriveRealPositionCounts();
    driveSnapshot.drivePosIn = getDriveEncoderDistanceIn();
    driveSnapshot.realPosRawIn = getDriveRealPosition();
    driveSnapshot.realPosFirIn = getDriveRealPositionFir();

    if (dtSeconds <= 0.0f) {
      driveSnapshot.driveVelInPerS = 0.0f;
      driveSnapshot.realVelInPerS = 0.0f;
      return;
    }

    driveSnapshot.driveVelInPerS =
        (driveSnapshot.drivePosIn - previousDriveSnapshot.drivePosIn) / dtSeconds;
    driveSnapshot.realVelInPerS =
        (driveSnapshot.realPosFirIn - previousDriveSnapshot.realPosFirIn) / dtSeconds;
  }

  void applyDriveMotorCommand(int speedCmd) {
    appliedDriveMotorCmd = constrain(speedCmd, -MAX_DRIVE_MOTOR_CMD, MAX_DRIVE_MOTOR_CMD);
    md.setM1Speed(appliedDriveMotorCmd);
  }

  bool startDriveTrajectoryToPositionIn(float targetPositionIn, Side targetSideIn) {
    activeDriveTrajectory = buildTrajectoryProfile(DRIVE_TRAJECTORY_MAX_ACCEL_IN_PER_S2,
                                                   steadyCruiseSpeedInPerS,
                                                   driveSnapshot.realPosFirIn,
                                                   targetPositionIn);
    if (!activeDriveTrajectory.isValid) {
      activeDriveTrajectory = {};
      return false;
    }

    driveMode = DRIVE_MODE_TRAJECTORY;
    driveTargetPositionIn = targetPositionIn;
    driveTargetSide = targetSideIn;
    activeDesiredDrivePositionIn = driveSnapshot.realPosFirIn;
    driveTrajectoryStartMs = millis();
    driveTargetSettledSinceMs = 0;
    armDriveContactSwitchDelay();

    resetRampController();
    applyDriveRampSettings();
    driveLastControlMs = millis();
    return true;
  }

  void updateCenterSpeedMeasurement() {
    const float centerPositionIn = getRailCenterPositionInches();
    if (fabsf(driveSnapshot.realPosFirIn - centerPositionIn) <= DRIVE_CENTER_SAMPLE_HALF_WINDOW_IN) {
      centerSpeedSumInPerS += fabsf(driveSnapshot.realVelInPerS);
      centerSpeedSampleCount++;
    }
  }

  void driveStartTowardOppositeSide() {
    Side dest = getOppositeSide(currentSide);
    driveStartTowardSide(dest);
  }

  void driveStartTowardSide(Side s) {
    driveStartTowardRailPositionFeet(getSidePositionFeet(s), s);
  }

  void driveStartTowardSideWithEndBuffer(Side s, float extraEndBufferIn) {
    steadyCruiseSpeedInPerS = max(0.1f, feetToInches(fabsf(run.speedCmd)));
    armDriveContactSwitchDelay();
    driveContactSwitchWasPressed = isDriveContactSwitchPressed();
    const float bufferedTargetIn =
        applyDriveEndBufferInches(getSidePositionInches(s), extraEndBufferIn);
    if (!startDriveTrajectoryToPositionIn(bufferedTargetIn, s)) {
      Serial.println("Unable to build a buffered drive trajectory. Entering emergency stop.");
      enterEmergencyStop();
    }
  }

  void driveStartTowardRailPositionFeet(float targetFeet, Side targetSideAtEndstop) {
    steadyCruiseSpeedInPerS = max(0.1f, feetToInches(fabsf(run.speedCmd)));
    armDriveContactSwitchDelay();
    driveContactSwitchWasPressed = isDriveContactSwitchPressed();
    if (!startDriveTrajectoryToPositionIn(feetToInches(targetFeet), targetSideAtEndstop)) {
      Serial.println("Unable to build a valid drive trajectory. Entering emergency stop.");
      enterEmergencyStop();
    }
  }

  void driveUpdate() {
    if (driveMode == DRIVE_MODE_IDLE) {
      applyDriveMotorCommand(0);
      return;
    }

    if (md.getM1Fault()) {
      Serial.println("Drive motor fault detected. Entering emergency stop.");
      enterEmergencyStop();
      return;
    }

    const uint32_t nowMs = millis();
    if ((nowMs - driveLastControlMs) < DRIVE_CONTROL_INTERVAL_MS) {
      return;
    }

    const float dtSeconds = (float)(nowMs - driveLastControlMs) / 1000.0f;
    driveLastControlMs = nowMs;

    refreshDriveSensors(dtSeconds);
    if (!didDriveRealPositionReadSucceed()) {
      Serial.println("Real-position sensor read failed. Entering emergency stop.");
      enterEmergencyStop();
      return;
    }

    updateCenterSpeedMeasurement();

    const float elapsedTimeSeconds =
        (float)(millis() - driveTrajectoryStartMs) / 1000.0f;
    activeDesiredDrivePositionIn =
        getTrajectoryPositionAtElapsedTime(activeDriveTrajectory, elapsedTimeSeconds);

    const int motorCmd = rampControl(activeDesiredDrivePositionIn,
                                     driveSnapshot.realPosFirIn);
    applyDriveMotorCommand(motorCmd);

    const float positionErrorIn = driveTargetPositionIn - driveSnapshot.realPosFirIn;
    const bool withinSettleWindow =
        fabsf(positionErrorIn) <= DRIVE_TARGET_TOLERANCE_IN &&
        fabsf(driveSnapshot.realVelInPerS) <= DRIVE_STOP_VELOCITY_TOLERANCE_IN_S;

    if (!withinSettleWindow) {
      driveTargetSettledSinceMs = 0;
    } else if (driveTargetSettledSinceMs == 0) {
      driveTargetSettledSinceMs = nowMs;
    }
  }

  void stopDriveMotors() {
    driveMode = DRIVE_MODE_IDLE;
    activeDriveTrajectory = {};
    resetRampController();
    activeDesiredDrivePositionIn = driveSnapshot.realPosFirIn;
    driveTargetPositionIn = driveSnapshot.realPosFirIn;
    driveTargetSettledSinceMs = 0;
    applyDriveMotorCommand(0);
  }

  float robotPositionAlongRail() {
    return inchesToFeet(driveSnapshot.realPosFirIn);
  }

  bool hasDriveSettledAtTarget() {
    return driveMode == DRIVE_MODE_TRAJECTORY &&
           driveTargetSettledSinceMs != 0 &&
           (millis() - driveTargetSettledSinceMs) >= DRIVE_TARGET_SETTLE_TIME_MS;
  }

  bool reachedSlowdownPosition() {
    if (driveMode != DRIVE_MODE_TRAJECTORY) return false;
    return fabsf(driveTargetPositionIn - driveSnapshot.realPosFirIn) <= DRIVE_SLOWDOWN_WINDOW_IN;
  }

  bool contactSwitchHit() {
    if (millis() < driveContactSwitchArmAtMs) {
      return false;
    }
    const bool pressed = isDriveContactSwitchPressed();
    const bool risingEdge = pressed && !driveContactSwitchWasPressed;
    driveContactSwitchWasPressed = pressed;
    if ((driveMode != DRIVE_MODE_IDLE) && risingEdge) {
      // Rearm the same timer immediately so a follow-up return traversal starts
      // from the same delay-based structure after an endstop hit.
      armDriveContactSwitchDelay();
      return true;
    }
    return false;
  }

  Side getDriveResetReferenceSide() {
    if (currentState == SEND_READY_TO_MANAGER ||
        currentState == WAIT_FOR_GO ||
        currentState == WAIT_FOR_START_DELAY) {
      return currentSide;
    }

    if (currentState == DRIVING ||
        currentState == RESUME_DRIVING_AFTER_HOIST ||
        currentState == DRIVE_TO_OPPOSITE) {
      return driveTargetSide;
    }

    return currentSide;
  }

  void resetDriveEncoders() {
    const Side referenceSide = getDriveResetReferenceSide();
    const bool sensorResetOk =
        zeroDriveSensorsToAbsolutePosition(getSidePositionInches(referenceSide));
    if (!sensorResetOk) {
      Serial.println("Drive sensor reset did not receive a live sensor acknowledgement.");
    }
  }

  bool hasReachedRailPositionFeet(float targetFeet) {
    const float targetIn = feetToInches(targetFeet);
    if (!activeDriveTrajectory.isValid || activeDriveTrajectory.direction >= 0.0f) {
      return driveSnapshot.realPosFirIn >= targetIn;
    }
    return driveSnapshot.realPosFirIn <= targetIn;
  }

  #pragma endregion

  #pragma region Runtime_Functions

/* ========================= Run Helpers ========================= */
  bool destinationIsStartSide() {
    // Interpret the command-side field as "where the robot should finish this
    // run", which is also the side the next run should start from.
    return startSide == runPhysicalStartSide;
  }

  void resetRunRuntimeFlags() {
    slowdown = false;
    goReceived = false;

    finishedTimestampRecorded = false;
    hoistStopReached = false;

    lowerDelayTimerRunning = false;
    hoistDelayTimerRunning = false;

    tGoReceivedMs = 0;
    tDriveStartedMs = 0;
    tHoistStopMs = 0;
    tFinishedMs = 0;

    centerSpeedSumInPerS = 0.0f;
    centerSpeedSampleCount = 0;

    measuredAvgSpeedCenter = 0.0f;
    measuredWinchHeightBeforeTranslation = 0.0f;
    measuredWinchMaxDepth = 0.0f;
  }

  void computeMeasuredValues() {
    // STUB: compute:
    //  - Average robot speed in center
    //  - Measured winch height before translation
    //  - Measured winch maximum depth
    //
    // Keep these fields updated during the run if possible.
    measuredAvgSpeedCenter =
        (centerSpeedSampleCount > 0)
            ? inchesToFeet(centerSpeedSumInPerS / (float)centerSpeedSampleCount)
            : fabsf(run.speedCmd);
    measuredWinchHeightBeforeTranslation = run.winchHeightCmd;
    measuredWinchMaxDepth = run.winchHeightCmd + run.lowerLoadDepthCmd; // placeholder
  }

  #pragma endregion

  #pragma region Test_Mode_Functions

/* ========================= TEST MODE ========================= */
  void testModeService() {
    // TEST_MODE simulates:
    //  - run packet received -> <A#> -> (short delay) <R#> -> wait <G> -> wait startDelay -> <F#, ...>
    // without moving any hardware.
    //
    // This allows validating manager<->node comms and parsing.

    static uint8_t phase = 0;
    static uint32_t phaseStartMs = 0;

    // phase:
    // 0 = idle waiting for run packet (parseRunCommandPacket will set run fields)
    // 1 = after ACK sent, waiting to send READY
    // 2 = READY sent, waiting for GO
    // 3 = GO received, waiting startDelay
    // 4 = running simulated motion, then send FINISH and return to idle

    // If a valid runType is loaded and we are idle, move to phase 1.
    if (phase == 0) {
      if (run.runType == RUN_S || run.runType == RUN_T || run.runType == RUN_I) {
        // We already sent ACK in handleAnglePayload() when packet arrived.
        phase = 1;
        phaseStartMs = millis();
      }
      return;
    }

    if (phase == 1) {
      // simulate winch rehome + lower height completion quickly
      if (millis() - phaseStartMs > 250) {
        sendReadyToManager();
        phase = 2;
        phaseStartMs = millis();
      }
      return;
    }

    if (phase == 2) {
      // waiting GO
      if (goReceived) {
        goReceived = false;
        tGoReceivedMs = millis();
        phase = 3;
        phaseStartMs = millis();
      }
      return;
    }

    if (phase == 3) {
      // wait startDelay (commanded)
      if (millis() - phaseStartMs >= run.startDelayMs) {
        tDriveStartedMs = millis();
        // simulate hoist stop timestamp if lowerAndHoist is true
        tHoistStopMs = run.lowerAndHoist ? (tDriveStartedMs + 500) : tDriveStartedMs;
        // record "finished" timestamp (first traverse end) after some time
        tFinishedMs = tDriveStartedMs + 2000;

        phase = 4;
        phaseStartMs = millis();
      }
      return;
    }

    if (phase == 4) {
      // send finish once
      computeMeasuredValues();
      sendFinishToManager();

      // reset for next test
      run = RunParams();
      resetRunRuntimeFlags();
      phase = 0;
      phaseStartMs = millis();
      return;
    }
  }

  #pragma endregion
#pragma endregion //the function declaration region

#pragma region Setup_Function
/* ========================= Setup / Loop ========================= */
  void setup() {
    Serial.begin(115200);
    xbeeSerial.begin(9600);
    Wire.begin();

    Serial.println("Hello Computer"); //debug print out

    md.init();
    md.enableDrivers();
    md.flipM1(FLIP_DRIVE_MOTOR);

    configureDriveControl();

    pinMode(PIN_BATT_SENSE, INPUT);
    pinMode(PIN_DRIVE_CONTACT_SWITCH, INPUT_PULLUP);

    zeroDriveSensorsToAbsolutePosition(getSidePositionInches(currentSide));
    driveContactSwitchWasPressed = isDriveContactSwitchPressed();

    resetRunRuntimeFlags();
    stopDriveMotors();
    stopWinchMotor();
    lockWinchPawl();
  }
  #pragma endregion

#pragma region Main_Loop
  void loop() {
    /* ---------- Global checks ---------- */
    checkWireless();

    // Global background event (always active): emergency stop command is received
    // is handled immediately in enterEmergencyStop()

    /* ---------- TEST MODE ---------- */
      if (currentState == TEST_MODE) {
        testModeService();
        return;
      }

    /* ---------- State Machine ---------- */
      switch (currentState) {

        /* 1) WAIT_FOR_COMMAND */
          case WAIT_FOR_COMMAND:
            // battery check only while waiting for command (per requirements)
            if (battery_too_low()) {
              // Service:
              //  - stop motors, lock winch
              //  - broadcast emergency stop to other node: !X!
              //  - broadcast battery warning to manager: <B#>
              stopDriveMotors();
              stopWinchMotor();
              lockWinchPawl();
              broadcastEmergencyStopToOtherNode();
              sendBatteryWarningToManager();

              currentState = EMERGENCY_STOP;
              break;
            }
            // All packet handling that transitions out of WAIT_FOR_COMMAND occurs in handleAnglePayload()
            break;

        /* 2) WAIT_FOR_WINCH_ACTION */
          case WAIT_FOR_WINCH_ACTION:
            winchUpdate();
            if (winchComplete) {
              // Service: reset winch variables
              winchResetVars();
              // Transition: winchNextState
              currentState = winchNextState;
            }
            break;

        /* 3) SET_WINCH_TO_COMMAND_HEIGHT */
          case SET_WINCH_TO_COMMAND_HEIGHT:
            // Service: set winch variables (lower to heightCommand), then go to WAIT_FOR_WINCH_ACTION
            winchStartAction(WINCH_LOWER_TO_DEPTH, run.winchHeightCmd, SEND_READY_TO_MANAGER);
            currentState = WAIT_FOR_WINCH_ACTION;
            break;

        /* 4) SEND_READY_TO_MANAGER */
          case SEND_READY_TO_MANAGER:
            // Service: reset encoders, transmit READY <R#>
            resetDriveEncoders();
            // (Optional) capture measured winch height before translation here
            measuredWinchHeightBeforeTranslation = readWinchPosition(); // stub will be 0 until implemented
            sendReadyToManager();
            currentState = WAIT_FOR_GO;
            break;

        /* 5) WAIT_FOR_GO */
          case WAIT_FOR_GO:
            if (goReceived) {
              // Service: record go timestamp, start startDelay timer
              goReceived = false;
              tGoReceivedMs = millis();
              startDelayStartMs = millis();
              currentState = WAIT_FOR_START_DELAY;
            }
            break;

        /* 6) WAIT_FOR_START_DELAY */
          case WAIT_FOR_START_DELAY:
            if ((millis() - startDelayStartMs) >= run.startDelayMs) {
              // Service: record start timestamp, start drive motion (nonblocking)
              tDriveStartedMs = millis();

              // Track the physical side this run actually began from. The
              // command packet's "startSide" is interpreted as the desired
              // finish side for this run / start side for the next run.
              runPhysicalStartSide = currentSide;

              // Every run starts by traversing toward the opposite side from
              // where the robot is currently sitting.
              if (run.lowerAndHoist) {
                driveStartTowardRailPositionFeet(run.hoistStopPosCmd,
                                                 getOppositeSide(runPhysicalStartSide));
              } else {
                driveStartTowardSide(getOppositeSide(runPhysicalStartSide));
              }

              currentState = DRIVING;
            }
            break;

        /* 7) DRIVING */
          case DRIVING:
            // Update drive control loop (nonblocking)
            driveUpdate();

            // Event: lower_load_position reached && lowerAndHoist == TRUE
            if (run.lowerAndHoist && !hoistStopReached) {
              // For middle-stop runs, the first trajectory itself ends at the
              // requested hoist-stop position. Transition only after that
              // trajectory has actually settled at its target.
              if (hasDriveSettledAtTarget()) {
                hoistStopReached = true;

                // Service: record hoist_stop timestamp, stop drive, start lowerDelay timer
                tHoistStopMs = millis();
                stopDriveMotors();

                lowerDelayStartMs = millis();
                lowerDelayTimerRunning = true;

                currentState = WAIT_LOWER_DELAY;
                break;
              }
            }

            // Event: slowdown position reached
            if (!slowdown && reachedSlowdownPosition()) {
              slowdown = true;
              // Transition stays in DRIVING
            }

            // Event: contact_switch_hit
            if (contactSwitchHit()) {
              // Service: record finished timestamp ONLY on first traverse
              if (!finishedTimestampRecorded) {
                tFinishedMs = millis();
                finishedTimestampRecorded = true;
              }

              stopDriveMotors();
              slowdown = false;

              // Reset encoders based on side logic (stub)
              resetDriveEncoders();

              // Shared contact switch is used at both rail ends, so the active
              // trajectory target determines which side we have just reached.
              currentSide = driveTargetSide;

              currentState = POST_TRAVERSAL_DECISION;
              break;
            }

            break;

        /* 8) WAIT_LOWER_DELAY */
          case WAIT_LOWER_DELAY:
            if (lowerDelayTimerRunning && (millis() - lowerDelayStartMs) >= run.lowerDelayMs) {
              lowerDelayTimerRunning = false;

              // Service: set winch variables (lower to lowerLoadDepth), winchNextState=WAIT_HOIST_DELAY
              winchStartAction(WINCH_LOWER_TO_DEPTH, run.lowerLoadDepthCmd, WAIT_HOIST_DELAY);
              currentState = WAIT_FOR_WINCH_ACTION;
            }
            break;

        /* 9) WAIT_HOIST_DELAY */
          case WAIT_HOIST_DELAY:
            // Event: enters state -> start hoistDelay timer if not started
            if (!hoistDelayTimerRunning) {
              hoistDelayTimerRunning = true;
              hoistDelayStartMs = millis();
            }

            // Event: hoistDelay elapsed
            if (hoistDelayTimerRunning && (millis() - hoistDelayStartMs) >= run.hoistDelayMs) {
              hoistDelayTimerRunning = false;

              // Service: set winch vars (hoist to height), winchNextState=RESUME_DRIVING_AFTER_HOIST
              winchStartAction(WINCH_HOIST_TO_HEIGHT, run.winchHeightCmd, RESUME_DRIVING_AFTER_HOIST);
              currentState = WAIT_FOR_WINCH_ACTION;
            }
            break;

        /* 10) RESUME_DRIVING_AFTER_HOIST */
          case RESUME_DRIVING_AFTER_HOIST:
            // Service: determine destinationIsStartSide(); set drive direction accordingly; resume driving
            if (destinationIsStartSide()) {
              driveStartTowardSideWithEndBuffer(runPhysicalStartSide,
                                                DRIVE_RETURN_END_BUFFER_IN);
            } else {
              driveStartTowardSideWithEndBuffer(getOppositeSide(runPhysicalStartSide),
                                                DRIVE_RETURN_END_BUFFER_IN);
            }
            currentState = DRIVING;
            break;

        /* 11) POST_TRAVERSAL_DECISION */
          case POST_TRAVERSAL_DECISION: {
            // Service pseudocode from requirements:
            // if (destinationIsStartSide()==TRUE AND currentSide==startSide) -> Send_finish
            // else if (destinationIsStartSide()==TRUE) -> prepareForTraverse then Drive_to_opposite via winch state
            // else -> Send_finish
            if (destinationIsStartSide() && currentSide == startSide) {
              currentState = SEND_FINISH;
            } else if (destinationIsStartSide()) {
              // prepareForTraverse (especially needed if lowerAndHoist==FALSE)
              winchStartAction(WINCH_PREPARE_FOR_TRAVERSE, 0.0f, DRIVE_TO_OPPOSITE);
              currentState = WAIT_FOR_WINCH_ACTION;
            } else {
              currentState = SEND_FINISH;
            }
          } break;

        /* 12) DRIVE_TO_OPPOSITE */
          case DRIVE_TO_OPPOSITE:
            // Service: set drive variables to drive to the opposite side
            driveStartTowardOppositeSide();
            currentState = DRIVING;
            break;

        /* 13) SEND_FINISH */
          case SEND_FINISH:
            // Service: compute measured values, send FINISH handshake
            computeMeasuredValues();
            sendFinishToManager();

            // Transition: WAIT_FOR_COMMAND
            run = RunParams();
            resetRunRuntimeFlags();
            currentState = WAIT_FOR_COMMAND;
            break;

        /* 14) EMERGENCY_STOP */
          case EMERGENCY_STOP:
            // entered state service: Stop motors, lock winch
            stopDriveMotors();
            stopWinchMotor();
            lockWinchPawl();
            // Transition: EMERGENCY_STOP (stay here)
            break;

          default:
            currentState = WAIT_FOR_COMMAND;
            break;
      }
  }
#pragma endregion
