/************************************************************
 * Manager (Arduino Uno) - FSM
 * Mechatronics Simulation of Overhead Hazards
 *
 * Requirements Source:
 *  - RequirementsAndStateTransition.md
 *
 * Hardware:
 *  - Arduino Uno
 *  - XBee via SoftwareSerial
 *  - XBEE_RX = 2, XBEE_TX = 3
 *
 * Wireless Regime:
 *  - Framed packets:  < ... >
 *  - Start:           <G>
 *  - Parse ACK:       <A1>, <A2>
 *  - Ready:           <R1>, <R2>
 *  - Finish:          <F#, ...>
 *  - Battery warning: <B1>, <B2>
 *  - E-Stop:          <X> (also sends legacy <x> for compatibility)
 *  - Node-to-node E-Stop flag: !X!  (manager ignores because it is not <...>)
 *
 * User Commands (typed into Serial Monitor):
 *  - Run config:
 *      S,<16 params>,<runs>
 *      T,<16 params>,<runs>
 *      I,<16 params for node1>,<16 params for node2>,<runs>
 *
 *    16 params order (per node):
 *      speedMean, speedVar,
 *      winchHeightMean, winchHeightVar,
 *      startDelayMeanMs, startDelayVarMs,
 *      lowerDelayMeanMs, lowerDelayVarMs,
 *      hoistDelayMeanMs, hoistDelayVarMs,
 *      hoistStopPosMean, hoistStopPosVar,
 *      lowerLoadDepthMean, lowerLoadDepthVar,
 *      chanceRaiseLower(0-100),
 *      startSideWeight(0-100)
 *
 *  - Lower winch command (broadcast):
 *      W,<depth1_ft>,<depth2_ft>
 *
 *  - Batch prompt:
 *      y   (next run)
 *      n   (abort batch)
 *
 *  - Emergency stop (global):
 *      x  or  X
 ************************************************************/

/* ========================= Includes ========================= */
  #include <Arduino.h>
  #include <SoftwareSerial.h>
  #include <string.h>
  #include <stdlib.h>

/* ========================= Pin Definitions ========================= */
  const uint8_t XBEE_RX = 2; // XBee TX -> Arduino D2
  const uint8_t XBEE_TX = 3; // Arduino D3 -> XBee RX
  SoftwareSerial xbeeSerial(XBEE_RX, XBEE_TX);

/* ========================= Protocol ========================= */
  const char START_CMD = 'G';     // <G>
  const char ESTOP_CMD = 'X';     // <X> (also send legacy <x>)

/* ========================= Buffers ========================= */
  static const uint16_t USER_BUF_MAX = 240;
  static const uint16_t XBEE_FRAME_MAX = 240;

  char userBuf[USER_BUF_MAX];
  uint16_t userLen = 0;

  bool inXBeeFrame = false;
  char xbeeFrame[XBEE_FRAME_MAX];
  uint16_t xbeeFrameLen = 0;

/* ========================= State Definitions ========================= */
  enum ManagerState : uint8_t {
    WAIT_FOR_USER_INPUT = 0,
    WAIT_FOR_ACKNOWLEDGEMENT,
    WAIT_FOR_READY,
    WAIT_FOR_FINISHED,
    WAIT_FOR_BATCH_DECISION
  };

  ManagerState currentState = WAIT_FOR_USER_INPUT;

/* ========================= Timing / Thresholds ========================= */
  const uint32_t ACK_TIMEOUT_MS = 7000; // timeThreshold (edit as needed) !CHECK!
  uint32_t ackWaitStartMs = 0;

/* ========================= Run / Command Variables ========================= */
  char runType = 0;                // 'S','T','I'
  bool nextRunQuestion = false;    // used by WAIT_FOR_BATCH_DECISION
  uint16_t runsInBatch = 0;
  uint16_t runsCompleted = 0;

/* ---- Base (mean/variance) parameters from user ---- */
  struct BaseParams {
    float speedMean = 0.0f, speedVar = 0.0f;
    float winchHeightMean = 0.0f, winchHeightVar = 0.0f;

    long startDelayMeanMs = 0, startDelayVarMs = 0;
    long lowerDelayMeanMs = 0, lowerDelayVarMs = 0;
    long hoistDelayMeanMs = 0, hoistDelayVarMs = 0;

    float hoistStopPosMean = 0.0f, hoistStopPosVar = 0.0f;
    float lowerLoadDepthMean = 0.0f, lowerLoadDepthVar = 0.0f;

    uint8_t chanceRaiseLower = 0;   // 0..100
    uint8_t startSideWeight = 50;   // 0..100 (probability of "Side 1")
  };

  BaseParams base1; // node1 base
  BaseParams base2; // node2 base (used only for 'I'; for 'T' we mirror base1)

/* ---- Per-run randomized commands actually sent to nodes ---- */
  struct RunCmd {
    float speedCmd = 0.0f;
    float winchHeightCmd = 0.0f;

    uint32_t startDelayCmdMs = 0;
    uint32_t lowerDelayCmdMs = 0;
    uint32_t hoistDelayCmdMs = 0;

    float hoistStopPosCmd = 0.0f;
    float lowerLoadDepthCmd = 0.0f;

    bool lowerAndHoistCmd = false; // result of chanceRaiseLower draw
    uint8_t startSideCmd = 0;      // 0 or 1, result of startSideWeight draw
  };

  RunCmd cmd1;
  RunCmd cmd2;

/* ---- ACK/READY/FINISHED tracking ---- */
  bool ack1 = false, ack2 = false;
  bool ready1 = false, ready2 = false;
  bool fin1 = false, fin2 = false;

/* ---- Finish handshake storage (manager prints these) ---- */
  struct FinishReport {
    bool valid = false;
    float avgSpeedCenter = 0.0f;
    float measuredWinchHeight = 0.0f;
    float measuredMaxDepth = 0.0f;
    uint32_t tGoToMoveMs = 0;
    uint32_t tGoToHoistStopMs = 0;
    uint32_t tGoToFinishedMs = 0;
    uint32_t startDelayCmdMs = 0;
    uint32_t lowerDelayCmdMs = 0;
    uint32_t hoistDelayCmdMs = 0;
  };

  FinishReport rep1;
  FinishReport rep2;

/* ========================= Function Prototypes ========================= */
  /* ---- Parsing / Utility ---- */
    bool isWs(char c);
    void sanitizeInPlace(char *s);
    uint8_t splitCsv(char *line, char *tokens[], uint8_t maxTokens);

    void printHelp();
    void clearAllStoredConfig();
    void clearPerRunTracking();

    bool parseRunConfigLine(char *line);
    bool parseLowerWinchLine(char *line, float &d1, float &d2);

  /* ---- Randomization ---- */
    float uniformFloat(float lo, float hi);
    long uniformLong(long lo, long hi);
    uint8_t clampU8(int v, int lo, int hi);

    void generatePerRunCommands();

  /* ---- Wireless Send ---- */
    void xbeeSendPayload(const char *payload);
    void xbeeSendCharFramed(char c);
    void broadcastEmergencyStop();

  /* ---- Wireless Receive ---- */
    void handleXBeeChar(char c);
    void handleXBeePayload(char *payload);

  /* ---- User Serial ---- */
    void handleUserChar(char c);
    void handleUserLine(char *line);

  /* ---- State Helpers ---- */
    uint8_t requiredMaskForRunType(char t);   // bit0=node1, bit1=node2
    uint8_t haveMaskAck();
    uint8_t haveMaskReady();
    uint8_t haveMaskFinished();

    void sendRunCommandPacket();
    void sendLowerWinchPacket(float depth1Ft, float depth2Ft);

    void printAckStatusIfAny();
    void printFinishSummaryAndPrompt();

/* ========================= Utility ========================= */
  bool isWs(char c) { return (c == ' ' || c == '\t' || c == '\r' || c == '\n'); }

  void sanitizeInPlace(char *s) {
    // remove framing chars and CR/LF; trim whitespace
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
    if (!line || !tokens || maxTokens == 0) return 0;

    sanitizeInPlace(line);
    if (line[0] == 0) return 0;

    uint8_t n = 0;
    char *p = line;

    while (n < maxTokens && p && *p) {
      // token starts at p
      tokens[n++] = p;

      // find next comma
      char *comma = strchr(p, ',');
      if (!comma) break;

      *comma = 0;           // terminate this token
      sanitizeInPlace(tokens[n - 1]);

      p = comma + 1;        // advance to next token start
    }

    // sanitize last token
    if (n > 0) sanitizeInPlace(tokens[n - 1]);
    return n;
  }

  uint8_t clampU8(int v, int lo, int hi) {
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    return (uint8_t)v;
  }

/* ========================= Help / Reset ========================= */
  void printHelp() {
    Serial.println(F("\n================ Manager Commands ================"));
    Serial.println(F("Lower winch:"));
    Serial.println(F("  W,<depth1_ft>,<depth2_ft>"));
    Serial.println(F("Run config (S/T):"));
    Serial.println(F("  S,<16 params>,<runs>"));
    Serial.println(F("  T,<16 params>,<runs>"));
    Serial.println(F("Run config (I):"));
    Serial.println(F("  I,<16 params node1>,<16 params node2>,<runs>"));
    Serial.println(F("\n16 params per node:"));
    Serial.println(F("  speedMean,speedVar,winchHeightMean,winchHeightVar,"));
    Serial.println(F("  startDelayMeanMs,startDelayVarMs,lowerDelayMeanMs,lowerDelayVarMs,"));
    Serial.println(F("  hoistDelayMeanMs,hoistDelayVarMs,hoistStopPosMean,hoistStopPosVar,"));
    Serial.println(F("  lowerLoadDepthMean,lowerLoadDepthVar,chanceRaiseLower(0-100),startSideWeight(0-100)"));
    Serial.println(F("\nEmergency stop (global): x or X"));
    Serial.println(F("==================================================\n"));
  }

  void clearPerRunTracking() {
    ack1 = ack2 = false;
    ready1 = ready2 = false;
    fin1 = fin2 = false;
    rep1 = FinishReport();
    rep2 = FinishReport();
  }

  void clearAllStoredConfig() {
    runType = 0;
    base1 = BaseParams();
    base2 = BaseParams();

    runsInBatch = 0;
    runsCompleted = 0;
    nextRunQuestion = false;

    cmd1 = RunCmd();
    cmd2 = RunCmd();

    clearPerRunTracking();

    currentState = WAIT_FOR_USER_INPUT;
  }

/* ========================= Randomization ========================= */
  float uniformFloat(float lo, float hi) {
    if (hi < lo) { float t = lo; lo = hi; hi = t; }
    const long SCALE = 10000L;
    long r = random(0, SCALE + 1);
    float u = (float)r / (float)SCALE;
    return lo + u * (hi - lo);
  }

  long uniformLong(long lo, long hi) {
    if (hi < lo) { long t = lo; lo = hi; hi = t; }
    if (hi == lo) return lo;
    return random(lo, hi + 1);
  }

  static uint32_t genDelayMs(long meanMs, long varMs) {
    long lo = meanMs - varMs;
    long hi = meanMs + varMs;
    if (lo < 0) lo = 0;
    if (hi < 0) hi = 0;
    return (uint32_t)uniformLong(lo, hi);
  }

  static float genFloat(float mean, float var) {
    return uniformFloat(mean - var, mean + var);
  }

  static bool drawChance(uint8_t percent) {
    if (percent >= 100) return true;
    if (percent == 0) return false;
    int r = random(0, 100); // 0..99
    return (r < (int)percent);
  }

  static uint8_t drawStartSide(uint8_t weight) {
    // weight=0 => always side 0
    // weight=100 => always side 1
    // weight=50 => 50/50
    if (weight >= 100) return 1;
    if (weight == 0) return 0;
    int r = random(0, 100);
    return (r < (int)weight) ? 1 : 0;
  }

  void generatePerRunCommands() {
    // Node1 always generated from base1
    cmd1.speedCmd = genFloat(base1.speedMean, base1.speedVar);
    cmd1.winchHeightCmd = genFloat(base1.winchHeightMean, base1.winchHeightVar);
    cmd1.startDelayCmdMs = genDelayMs(base1.startDelayMeanMs, base1.startDelayVarMs);
    cmd1.lowerDelayCmdMs = genDelayMs(base1.lowerDelayMeanMs, base1.lowerDelayVarMs);
    cmd1.hoistDelayCmdMs = genDelayMs(base1.hoistDelayMeanMs, base1.hoistDelayVarMs);
    cmd1.hoistStopPosCmd = genFloat(base1.hoistStopPosMean, base1.hoistStopPosVar);
    cmd1.lowerLoadDepthCmd = genFloat(base1.lowerLoadDepthMean, base1.lowerLoadDepthVar);
    cmd1.lowerAndHoistCmd = drawChance(base1.chanceRaiseLower);
    cmd1.startSideCmd = drawStartSide(base1.startSideWeight);

    if (runType == 'I') {
      // Independent: Node2 has its own distribution
      cmd2.speedCmd = genFloat(base2.speedMean, base2.speedVar);
      cmd2.winchHeightCmd = genFloat(base2.winchHeightMean, base2.winchHeightVar);
      cmd2.startDelayCmdMs = genDelayMs(base2.startDelayMeanMs, base2.startDelayVarMs);
      cmd2.lowerDelayCmdMs = genDelayMs(base2.lowerDelayMeanMs, base2.lowerDelayVarMs);
      cmd2.hoistDelayCmdMs = genDelayMs(base2.hoistDelayMeanMs, base2.hoistDelayVarMs);
      cmd2.hoistStopPosCmd = genFloat(base2.hoistStopPosMean, base2.hoistStopPosVar);
      cmd2.lowerLoadDepthCmd = genFloat(base2.lowerLoadDepthMean, base2.lowerLoadDepthVar);
      cmd2.lowerAndHoistCmd = drawChance(base2.chanceRaiseLower);
      cmd2.startSideCmd = drawStartSide(base2.startSideWeight);
    } else if (runType == 'T') {
      // Tied-in: same randomized values & info to both robots (per requirements)
      cmd2 = cmd1;
    } else {
      // Single: node2 unused
      cmd2 = RunCmd();
    }
  }

/* ========================= Wireless Send ========================= */
  void xbeeSendPayload(const char *payload) {
    xbeeSerial.print('<');
    xbeeSerial.print(payload);
    xbeeSerial.print('>');
    xbeeSerial.print('\n'); // newline is fine; receiver ignores it

    Serial.print(F("TX: <"));
    Serial.print(payload);
    Serial.println(F(">"));
  }

  void xbeeSendCharFramed(char c) {
    char msg[2];
    msg[0] = c;
    msg[1] = 0;
    xbeeSendPayload(msg);
  }

  void broadcastEmergencyStop() {
    // Requirements say <X>; existing code used <x>. Send both for compatibility.
    xbeeSendCharFramed('X');
    xbeeSendCharFramed('x');
  }

/* ========================= Required Masks ========================= */
  uint8_t requiredMaskForRunType(char t) {
    // bit0=node1, bit1=node2
    if (t == 'S') return 0b01;
    if (t == 'T') return 0b11;
    if (t == 'I') return 0b11;
    return 0;
  }

  uint8_t haveMaskAck() {
    return (ack1 ? 0b01 : 0) | (ack2 ? 0b10 : 0);
  }
  uint8_t haveMaskReady() {
    return (ready1 ? 0b01 : 0) | (ready2 ? 0b10 : 0);
  }
  uint8_t haveMaskFinished() {
    return (fin1 ? 0b01 : 0) | (fin2 ? 0b10 : 0);
  }

/* ========================= Run / Winch Packets ========================= */
  void sendRunCommandPacket() {
    // Packet formats used by Node code below:
    //
    // S/T:
    //   <T,speed,winchHeight,startDelayMs,lowerDelayMs,hoistDelayMs,hoistStopPos,lowerLoadDepth,lowerAndHoist,startSide>
    //
    // I:
    //   <I, s1,h1,sd1,ld1,hd1,sp1,dd1,lah1,ss1,  s2,h2,sd2,ld2,hd2,sp2,dd2,lah2,ss2>
    //
    char payload[XBEE_FRAME_MAX];
    payload[0] = 0;

    char s1[16], h1[16], sp1[16], dd1[16];
    dtostrf(cmd1.speedCmd,        0, 3, s1);
    dtostrf(cmd1.winchHeightCmd,  0, 3, h1);
    dtostrf(cmd1.hoistStopPosCmd, 0, 3, sp1);
    dtostrf(cmd1.lowerLoadDepthCmd, 0, 3, dd1);

    if (runType == 'I') {
      char s2[16], h2[16], sp2[16], dd2[16];
      dtostrf(cmd2.speedCmd,        0, 3, s2);
      dtostrf(cmd2.winchHeightCmd,  0, 3, h2);
      dtostrf(cmd2.hoistStopPosCmd, 0, 3, sp2);
      dtostrf(cmd2.lowerLoadDepthCmd, 0, 3, dd2);

      snprintf(payload, sizeof(payload),
        "%c,%s,%s,%lu,%lu,%lu,%s,%s,%u,%u,%s,%s,%lu,%lu,%lu,%s,%s,%u,%u",
        runType,
        s1, h1,
        (unsigned long)cmd1.startDelayCmdMs,
        (unsigned long)cmd1.lowerDelayCmdMs,
        (unsigned long)cmd1.hoistDelayCmdMs,
        sp1, dd1,
        (unsigned)(cmd1.lowerAndHoistCmd ? 1 : 0),
        (unsigned)cmd1.startSideCmd,
        s2, h2,
        (unsigned long)cmd2.startDelayCmdMs,
        (unsigned long)cmd2.lowerDelayCmdMs,
        (unsigned long)cmd2.hoistDelayCmdMs,
        sp2, dd2,
        (unsigned)(cmd2.lowerAndHoistCmd ? 1 : 0),
        (unsigned)cmd2.startSideCmd
      );
    } else {
      snprintf(payload, sizeof(payload),
        "%c,%s,%s,%lu,%lu,%lu,%s,%s,%u,%u",
        runType,
        s1, h1,
        (unsigned long)cmd1.startDelayCmdMs,
        (unsigned long)cmd1.lowerDelayCmdMs,
        (unsigned long)cmd1.hoistDelayCmdMs,
        sp1, dd1,
        (unsigned)(cmd1.lowerAndHoistCmd ? 1 : 0),
        (unsigned)cmd1.startSideCmd
      );
    }

    xbeeSendPayload(payload);
  }

  void sendLowerWinchPacket(float depth1Ft, float depth2Ft) {
    char d1[16], d2[16];
    dtostrf(depth1Ft, 0, 3, d1);
    dtostrf(depth2Ft, 0, 3, d2);

    char payload[XBEE_FRAME_MAX];
    snprintf(payload, sizeof(payload), "W,%s,%s", d1, d2);
    xbeeSendPayload(payload);
  }

/* ========================= Finish Printing ========================= */
  void printFinishSummaryAndPrompt() {
    Serial.println(F("\n================ RUN FINISHED ================"));
    Serial.print(F("Run "));
    Serial.print(runsCompleted);
    Serial.print(F(" / "));
    Serial.println(runsInBatch);

    // Node1 always expected for S/T/I? (S expects only node1)
    if (rep1.valid) {
      Serial.println(F("\nNode1 Finish Report:"));
      Serial.print(F("  avgSpeedCenter: ")); Serial.println(rep1.avgSpeedCenter, 3);
      Serial.print(F("  measuredWinchHeight: ")); Serial.println(rep1.measuredWinchHeight, 3);
      Serial.print(F("  measuredMaxDepth: ")); Serial.println(rep1.measuredMaxDepth, 3);
      Serial.print(F("  tGoToMoveMs: ")); Serial.println(rep1.tGoToMoveMs);
      Serial.print(F("  tGoToHoistStopMs: ")); Serial.println(rep1.tGoToHoistStopMs);
      Serial.print(F("  tGoToFinishedMs: ")); Serial.println(rep1.tGoToFinishedMs);
      Serial.print(F("  startDelayCmdMs: ")); Serial.println(rep1.startDelayCmdMs);
      Serial.print(F("  lowerDelayCmdMs: ")); Serial.println(rep1.lowerDelayCmdMs);
      Serial.print(F("  hoistDelayCmdMs: ")); Serial.println(rep1.hoistDelayCmdMs);
    } else {
      Serial.println(F("\nNode1 Finish Report: (missing/invalid)"));
    }

    if (runType != 'S') {
      if (rep2.valid) {
        Serial.println(F("\nNode2 Finish Report:"));
        Serial.print(F("  avgSpeedCenter: ")); Serial.println(rep2.avgSpeedCenter, 3);
        Serial.print(F("  measuredWinchHeight: ")); Serial.println(rep2.measuredWinchHeight, 3);
        Serial.print(F("  measuredMaxDepth: ")); Serial.println(rep2.measuredMaxDepth, 3);
        Serial.print(F("  tGoToMoveMs: ")); Serial.println(rep2.tGoToMoveMs);
        Serial.print(F("  tGoToHoistStopMs: ")); Serial.println(rep2.tGoToHoistStopMs);
        Serial.print(F("  tGoToFinishedMs: ")); Serial.println(rep2.tGoToFinishedMs);
        Serial.print(F("  startDelayCmdMs: ")); Serial.println(rep2.startDelayCmdMs);
        Serial.print(F("  lowerDelayCmdMs: ")); Serial.println(rep2.lowerDelayCmdMs);
        Serial.print(F("  hoistDelayCmdMs: ")); Serial.println(rep2.hoistDelayCmdMs);
      } else {
        Serial.println(F("\nNode2 Finish Report: (missing/invalid)"));
      }
    }

    Serial.println(F("\n================================================"));

    if (runsCompleted < runsInBatch) {
      Serial.println(F("Next run in batch? (y/n)"));
    }
  }

/* ========================= Parsing: User Input ========================= */
  bool parseLowerWinchLine(char *line, float &d1, float &d2) {
    // Accept:
    //   W,depth1,depth2
    //   W,depth
    // Also tolerates (optional): <W,depth1,depth2>
    //
    // IMPORTANT:
    //   splitCsv() is destructive (it replaces commas with '\0'). This parser is
    //   called before parseRunConfigLine(), so we MUST NOT tokenize the caller's
    //   buffer unless this really is a W command.
    if (!line) return false;

    // Quick reject (non-destructive) for anything that is not a W command.
    const char *s = line;
    if (s[0] == '<') s++;                 // allow optional '<'
    if (!(s[0] == 'W' || s[0] == 'w')) return false;

    // Work on a local copy so we don't corrupt the original input line.
    char copy[USER_BUF_MAX];
    strncpy(copy, s, sizeof(copy));
    copy[sizeof(copy) - 1] = 0;

    // Strip optional trailing '>'
    size_t L = strlen(copy);
    if (L && copy[L - 1] == '>') copy[L - 1] = 0;

    static const uint8_t MAXTOK = 4;
    char *tok[MAXTOK];
    uint8_t n = splitCsv(copy, tok, MAXTOK);
    if (n < 2) return false;
    if (!(tok[0][0] == 'W' || tok[0][0] == 'w')) return false;

    d1 = (float)atof(tok[1]);
    if (n >= 3) d2 = (float)atof(tok[2]);
    else d2 = d1;

    return true;
  }

  bool parseRunConfigLine(char *line) {
    // S/T => 18 tokens, I => 34 tokens
    static const uint8_t MAXTOK = 40;
    char *tok[MAXTOK];
    // Allow optional angle-bracket framing (<...>) on user input.
      char *p = line;
      if (p && p[0] == '<') p++;
      if (p) {
        size_t L = strlen(p);
        if (L && p[L - 1] == '>') p[L - 1] = 0;
      }
    uint8_t n = splitCsv(p, tok, MAXTOK);
    if (n == 0) return false;

    char t = tok[0][0];
    if (!(t == 'S' || t == 'T' || t == 'I')) return false;

    if ((t == 'S' || t == 'T') && n != 18) {
      Serial.print(F("ERROR: S/T expects 18 fields, got ")); Serial.println(n);
      return false;
    }
    if (t == 'I' && n != 34) {
      Serial.print(F("ERROR: I expects 34 fields, got ")); Serial.println(n);
      return false;
    }

    auto toLong = [](const char *s)->long { return atol(s); };
    auto toFloat = [](const char *s)->float { return (float)atof(s); };
    auto toU8 = [](const char *s)->uint8_t { return clampU8(atoi(s), 0, 100); };

    runType = t;

    int idx = 1;
    base1.speedMean = toFloat(tok[idx++]);
    base1.speedVar = toFloat(tok[idx++]);
    base1.winchHeightMean = toFloat(tok[idx++]);
    base1.winchHeightVar = toFloat(tok[idx++]);

    base1.startDelayMeanMs = toLong(tok[idx++]);
    base1.startDelayVarMs  = toLong(tok[idx++]);
    base1.lowerDelayMeanMs = toLong(tok[idx++]);
    base1.lowerDelayVarMs  = toLong(tok[idx++]);
    base1.hoistDelayMeanMs = toLong(tok[idx++]);
    base1.hoistDelayVarMs  = toLong(tok[idx++]);

    base1.hoistStopPosMean = toFloat(tok[idx++]);
    base1.hoistStopPosVar  = toFloat(tok[idx++]);
    base1.lowerLoadDepthMean = toFloat(tok[idx++]);
    base1.lowerLoadDepthVar  = toFloat(tok[idx++]);

    base1.chanceRaiseLower = toU8(tok[idx++]);
    base1.startSideWeight  = toU8(tok[idx++]);

    if (runType == 'I') {
      base2.speedMean = toFloat(tok[idx++]);
      base2.speedVar = toFloat(tok[idx++]);
      base2.winchHeightMean = toFloat(tok[idx++]);
      base2.winchHeightVar = toFloat(tok[idx++]);

      base2.startDelayMeanMs = toLong(tok[idx++]);
      base2.startDelayVarMs  = toLong(tok[idx++]);
      base2.lowerDelayMeanMs = toLong(tok[idx++]);
      base2.lowerDelayVarMs  = toLong(tok[idx++]);
      base2.hoistDelayMeanMs = toLong(tok[idx++]);
      base2.hoistDelayVarMs  = toLong(tok[idx++]);

      base2.hoistStopPosMean = toFloat(tok[idx++]);
      base2.hoistStopPosVar  = toFloat(tok[idx++]);
      base2.lowerLoadDepthMean = toFloat(tok[idx++]);
      base2.lowerLoadDepthVar  = toFloat(tok[idx++]);

      base2.chanceRaiseLower = toU8(tok[idx++]);
      base2.startSideWeight  = toU8(tok[idx++]);
    } else {
      // For tied-in, the node2 distribution mirrors node1 (but per requirements
      // we send identical per-run values anyway).
      base2 = base1;
    }

    runsInBatch = (uint16_t)max(0L, toLong(tok[n - 1]));
    if (runsInBatch == 0) {
      Serial.println(F("ERROR: runsInBatch must be >= 1"));
      return false;
    }

    runsCompleted = 0;
    return true;
  }

/* ========================= User Serial Handling ========================= */
  void handleUserLine(char *line) {
    sanitizeInPlace(line);
    if (line[0] == 0) return;

    // Global E-STOP (any state)
    if ((line[0] == 'x' || line[0] == 'X') && line[1] == 0) {
      Serial.println(F("USER E-STOP: Broadcasting <X> (and legacy <x>) and returning to Wait_for_user_input."));
      broadcastEmergencyStop();
      clearAllStoredConfig();
      printHelp();
      return;
    }

    // WAIT_FOR_BATCH_DECISION expects y/n only
    if (currentState == WAIT_FOR_BATCH_DECISION && nextRunQuestion) {
      if ((line[0] == 'y' || line[0] == 'Y') && line[1] == 0) {
        // Generate new random values from previous input and broadcast
        nextRunQuestion = false;

        generatePerRunCommands();
        clearPerRunTracking();
        sendRunCommandPacket();

        // Per requirements: transition directly to Wait_for_ready
        currentState = WAIT_FOR_READY;
        Serial.println(F("Sent next run packet. Waiting for READY..."));
        return;
      }
      if ((line[0] == 'n' || line[0] == 'N') && line[1] == 0) {
        nextRunQuestion = false;
        Serial.println(F("Batch aborted. Clearing stored values. العودة to Wait_for_user_input."));
        clearAllStoredConfig();
        printHelp();
        return;
      }

      Serial.println(F("Please answer y or n."));
      return;
    }

    // Only accept new run config / lower-winch commands in WAIT_FOR_USER_INPUT
    if (currentState != WAIT_FOR_USER_INPUT) {
      Serial.println(F("Manager busy. Use X/x for emergency stop, or wait for the current run to finish."));
      return;
    }

    // Lower winch command
    float d1 = 0.0f, d2 = 0.0f;
    if (parseLowerWinchLine(line, d1, d2)) {
      Serial.println(F("Broadcasting lower winch command <W,...>."));
      sendLowerWinchPacket(d1, d2);
      // Per requirements: stay in WAIT_FOR_USER_INPUT
      return;
    }

    // Run config command
    if (parseRunConfigLine(line)) {
      Serial.print(F("Stored run config. runType="));
      Serial.print(runType);
      Serial.print(F(", runsInBatch="));
      Serial.println(runsInBatch);

      // Generate per-run randomized values and send
      generatePerRunCommands();
      clearPerRunTracking();
      sendRunCommandPacket();

      // Per requirements: go to WAIT_FOR_ACKNOWLEDGEMENT
      ackWaitStartMs = millis();
      currentState = WAIT_FOR_ACKNOWLEDGEMENT;
      Serial.println(F("Waiting for <A#> acknowledgements..."));
      return;
    }

    Serial.println(F("Unrecognized command."));
    printHelp();
  }

  void handleUserChar(char c) {
    if (c == '\r') return;

    if (c == '\n') {
      userBuf[userLen] = 0;
      handleUserLine(userBuf);
      userLen = 0;
      return;
    }

    if (userLen + 1 < USER_BUF_MAX) {
      userBuf[userLen++] = c;
    } else {
      Serial.println(F("User input too long; clearing buffer."));
      userLen = 0;
    }
  }

/* ========================= XBee Receive Handling ========================= */
  void handleXBeePayload(char *payload) {
    sanitizeInPlace(payload);
    if (payload[0] == 0) return;

    // Ignore node-to-node special messages if they ever arrive framed
    if (payload[0] == '!') {
      // e.g. "!X!"
      Serial.print(F("RX: <")); Serial.print(payload); Serial.println(F("> (ignored - node-to-node flag)"));
      return;
    }

    // Battery warning: <B1> or <B2> (global)
    if (payload[0] == 'B' && (payload[1] == '1' || payload[1] == '2') && payload[2] == 0) {
      Serial.print(F("RX BATTERY WARNING: <"));
      Serial.print(payload);
      Serial.println(F(">"));
      Serial.println(F("Emergency stop was issued by a node due to low battery."));
      Serial.println(F("Broadcasting <X> (and legacy <x>) and returning to Wait_for_user_input."));

      broadcastEmergencyStop();
      clearAllStoredConfig();
      printHelp();
      return;
    }

    // ACK: <A1> / <A2>
    if (payload[0] == 'A' && (payload[1] == '1' || payload[1] == '2') && payload[2] == 0) {
      if (payload[1] == '1') ack1 = true;
      else ack2 = true;
      Serial.print(F("RX ACK: <")); Serial.print(payload); Serial.println(F(">"));
      return;
    }

    // READY: <R1> / <R2>
    if (payload[0] == 'R' && (payload[1] == '1' || payload[1] == '2') && payload[2] == 0) {
      if (payload[1] == '1') ready1 = true;
      else ready2 = true;
      Serial.print(F("RX READY: <")); Serial.print(payload); Serial.println(F(">"));
      return;
    }

    // FINISH: <F#, ...>
    // Expected token order:
    // F#, avgSpeedCenter, measuredWinchHeight, measuredMaxDepth,
    //     tGoToMoveMs, tGoToHoistStopMs, tGoToFinishedMs,
    //     startDelayCmdMs, lowerDelayCmdMs, hoistDelayCmdMs
    if (payload[0] == 'F' && (payload[1] == '1' || payload[1] == '2')) {
      // Copy before tokenizing
      char copy[XBEE_FRAME_MAX];
      strncpy(copy, payload, sizeof(copy));
      copy[sizeof(copy) - 1] = 0;

      static const uint8_t MAXTOK = 16;
      char *tok[MAXTOK];
      uint8_t n = splitCsv(copy, tok, MAXTOK);

      uint8_t node = (payload[1] == '1') ? 1 : 2;

      if (n >= 10) {
        FinishReport rep;
        rep.valid = true;
        rep.avgSpeedCenter = (float)atof(tok[1]);
        rep.measuredWinchHeight = (float)atof(tok[2]);
        rep.measuredMaxDepth = (float)atof(tok[3]);
        rep.tGoToMoveMs = (uint32_t)atol(tok[4]);
        rep.tGoToHoistStopMs = (uint32_t)atol(tok[5]);
        rep.tGoToFinishedMs = (uint32_t)atol(tok[6]);
        rep.startDelayCmdMs = (uint32_t)atol(tok[7]);
        rep.lowerDelayCmdMs = (uint32_t)atol(tok[8]);
        rep.hoistDelayCmdMs = (uint32_t)atol(tok[9]);

        if (node == 1) {
          rep1 = rep;
          fin1 = true;
        } else {
          rep2 = rep;
          fin2 = true;
        }

        Serial.print(F("RX FINISH: <F"));
        Serial.print(node);
        Serial.println(F(",...>"));
        return;
      } else {
        // Fallback: accept as finished but mark invalid
        if (node == 1) fin1 = true;
        else fin2 = true;

        Serial.print(F("RX FINISH (unparsed): <"));
        Serial.print(payload);
        Serial.println(F(">"));
        return;
      }
    }

    // Unknown
    Serial.print(F("RX: <"));
    Serial.print(payload);
    Serial.println(F("> (ignored)"));
  }

  void handleXBeeChar(char c) {
    if (!inXBeeFrame) {
      if (c == '<') {
        inXBeeFrame = true;
        xbeeFrameLen = 0;
      }
      return;
    }

    if (c == '>') {
      xbeeFrame[xbeeFrameLen] = 0;
      inXBeeFrame = false;
      handleXBeePayload(xbeeFrame);
      return;
    }

    if (c == '\r' || c == '\n') return;

    if (xbeeFrameLen + 1 < XBEE_FRAME_MAX) {
      xbeeFrame[xbeeFrameLen++] = c;
    } else {
      // overflow protection
      inXBeeFrame = false;
      xbeeFrameLen = 0;
      Serial.println(F("WARNING: XBee frame overflow; dropped."));
    }
  }

/* ========================= Setup / Loop ========================= */
  void setup() {
    Serial.begin(115200);
    xbeeSerial.begin(9600);

    randomSeed(analogRead(A0));

    clearAllStoredConfig();
    Serial.println(F("Manager started."));
    printHelp();
  }

  void loop() {
    /* ---------- Global checks: user serial ---------- */
      while (Serial.available()) {
        handleUserChar((char)Serial.read());
      }

    /* ---------- Global checks: XBee serial ---------- */
      xbeeSerial.listen();
      while (xbeeSerial.available()) {
        handleXBeeChar((char)xbeeSerial.read());
      }

    /* ---------- State Machine ---------- */
      switch (currentState) {

        case WAIT_FOR_USER_INPUT:
          // Everything handled in handleUserLine()
          break;

        case WAIT_FOR_ACKNOWLEDGEMENT: {
          uint8_t req = requiredMaskForRunType(runType);
          uint8_t have = haveMaskAck();

          if ((have & req) == req) {
            Serial.println(F("All required ACKs received. Transitioning to Wait_for_ready."));
            currentState = WAIT_FOR_READY;
            break;
          }

          if ((millis() - ackWaitStartMs) > ACK_TIMEOUT_MS) {
            Serial.println(F("ERROR: ACK timeout elapsed. Returning to Wait_for_user_input."));
            clearAllStoredConfig();
            printHelp();
            break;
          }
        } break;

        case WAIT_FOR_READY: {
          uint8_t req = requiredMaskForRunType(runType);
          uint8_t have = haveMaskReady();

          if ((have & req) == req) {
            Serial.println(F("All required READYs received. Broadcasting <G>."));
            xbeeSendCharFramed(START_CMD);
            currentState = WAIT_FOR_FINISHED;
          }
        } break;

        case WAIT_FOR_FINISHED: {
          uint8_t req = requiredMaskForRunType(runType);
          uint8_t have = haveMaskFinished();

          if ((have & req) == req) {
            runsCompleted++;

            // Print measured values from finish handshake
            printFinishSummaryAndPrompt();

            if (runsCompleted < runsInBatch) {
              nextRunQuestion = true;
              currentState = WAIT_FOR_BATCH_DECISION; // per requirements
            } else {
              Serial.println(F("\nBatch complete. Clearing stored values and returning to Wait_for_user_input."));
              clearAllStoredConfig();
              printHelp();
            }
          }
        } break;

        case WAIT_FOR_BATCH_DECISION:
          // handled in handleUserLine() when nextRunQuestion==true
          break;

        default:
          clearAllStoredConfig();
          printHelp();
          break;
      }
  }