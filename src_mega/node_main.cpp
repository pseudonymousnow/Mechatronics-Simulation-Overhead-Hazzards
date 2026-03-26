/************************************************************
 * Node (Arduino Mega) - Node1 FSM + Comms
 * Mechatronics Simulation of Overhead Hazards
 *
 * Requirements Source:
 *  - RequirementsAndStateTransition.md
 *
 * Hardware:
 *  - Arduino Mega
 *  - XBee on Serial2 (pins RX2=17, TX2=16 on Mega) !CHECK!
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
  #include <string.h>
  #include <stdlib.h>

/* ========================= Node Identity ========================= */
  #define NODE_ID 1               // <-- CHANGE TO 2 to make this node behave as Node2

/* ========================= Serial Definitions ========================= */
  #define xbeeSerial Serial2      // Mega: Serial2 (TX2=16, RX2=17)  !CHECK! wiring

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

  NodeState currentState = TEST_MODE; // <-- set to TEST_MODE for comm simulation testing. WAIT_FOR_COMMAND for normal opperation

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
    uint8_t startSide = 0;            // 0 or 1
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
  Side startSide = SIDE_0;
  Side currentSide = SIDE_0;

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
  const uint8_t PIN_BATT_SENSE = A0;    // !CHECK! battery divider input pin
  const uint8_t PIN_LIMIT_SIDE0 = 0;    // !CHECK! contact switch for SIDE_0 end
  const uint8_t PIN_LIMIT_SIDE1 = 0;    // !CHECK! contact switch for SIDE_1 end

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
    void driveUpdate();                  // non-blocking update loop
    void stopDriveMotors();              // stub

    float robotPositionAlongRail();      // stub (feet or similar)
    bool reachedSlowdownPosition();      // stub
    bool contactSwitchHit();             // stub + side logic
    void resetDriveEncoders();           // stub

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

/* ========================= Drive Stubs ========================= */
  void driveStartTowardOppositeSide() {
    // STUB: decide opposite based on currentSide, then call driveStartTowardSide()
    Side dest = (currentSide == SIDE_0) ? SIDE_1 : SIDE_0;
    driveStartTowardSide(dest);
  }

  void driveStartTowardSide(Side s) {
    // STUB: set direction pins, enable drive motors, set speed based on run.speedCmd
    (void)s;
  }

  void driveUpdate() {
    // STUB: non-blocking control loop:
    //  - apply slowdown speed if slowdown==true
    //  - use encoder feedback if available
  }

  void stopDriveMotors() {
    // STUB: disable drive motors
  }

  float robotPositionAlongRail() {
    // STUB: return true position along rail (feet)
    return 0.0f;
  }

  bool reachedSlowdownPosition() {
    // STUB: return true when near end; usually depends on currentSide and direction
    return false;
  }

  bool contactSwitchHit() {
    // STUB: must return true when the appropriate endstop for the current travel is hit.
    //
    // You likely need:
    //  - one contact switch for each end (PIN_LIMIT_SIDE0 / PIN_LIMIT_SIDE1)
    //  - logic based on current travel direction to decide which switch to watch
    //
    // For now always false.
    return false;
  }

  void resetDriveEncoders() {
    // STUB: set driveWheel and truePosition encoders to 0
  }

  #pragma endregion

  #pragma region Runtime_Functions

/* ========================= Run Helpers ========================= */
  bool destinationIsStartSide() {
    // STUB: define your experiment logic here.
    // If true:
    //  - after first traverse, the robot will "prepareForTraverse" and return,
    //    then Send_finish occurs when currentSide == startSide.
    //
    // Default: false (finish on the opposite side after first traverse).
    return false;
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
    // For now, populate with command-based placeholders.
    measuredAvgSpeedCenter = run.speedCmd;                 // placeholder
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

    Serial.println("Hello Computer"); //debug print out

    pinMode(PIN_BATT_SENSE, INPUT);
    if (PIN_LIMIT_SIDE0 != 0) pinMode(PIN_LIMIT_SIDE0, INPUT_PULLUP); // !CHECK!
    if (PIN_LIMIT_SIDE1 != 0) pinMode(PIN_LIMIT_SIDE1, INPUT_PULLUP); // !CHECK!

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

              // Set currentSide to startSide at run start
              currentSide = startSide;

              // Start driving toward opposite side
              driveStartTowardOppositeSide();

              currentState = DRIVING;
            }
            break;

        /* 7) DRIVING */
          case DRIVING:
            // Update drive control loop (nonblocking)
            driveUpdate();

            // Event: lower_load_position reached && lowerAndHoist == TRUE
            if (run.lowerAndHoist && !hoistStopReached) {
              // Define "lower_load_position reached" as reaching hoistStopPosCmd
              if (robotPositionAlongRail() >= run.hoistStopPosCmd) { // !CHECK! direction-aware compare
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

              // Update side:
              // You must implement direction + which switch hit; for now toggle
              currentSide = (currentSide == SIDE_0) ? SIDE_1 : SIDE_0; // !CHECK! replace with real side logic

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
            // In this FSM design, destination handling is implemented in POST_TRAVERSAL_DECISION
            // after reaching an endstop, matching the provided transitions.
            driveStartTowardOppositeSide();
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
