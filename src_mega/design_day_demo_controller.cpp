#include <Arduino.h>

#include "RobotProtocol.h"

/*
  design_day_demo_controller.cpp

  Operator-side design day controller.

  Inputs:
  - A0 potentiometer commands robot drive motion forward/backward
  - D3 contact switch commands winch up
  - D5 contact switch commands winch down

  Safety / arbitration rules:
  - only one input source can be active at a time
  - a non-zero potentiometer command blocks both switches
  - an active switch blocks the other switch and the potentiometer
  - serial command 'x' latches an emergency stop until 'c' clears it

  Communications:
  - USB Serial is used for debug/status and emergency-stop commands
  - XBee packets go out over Serial2 using framed payloads: <CMD,...>
*/

#define xbeeSerial Serial2

const uint32_t USB_SERIAL_BAUD_RATE = 115200;
const uint32_t XBEE_BAUD_RATE = 9600;

const uint8_t POTENTIOMETER_PIN = A0;
const uint8_t CONTACT_SWITCH_PINS[] = {3, 5};
const uint8_t CONTACT_SWITCH_COUNT =
    sizeof(CONTACT_SWITCH_PINS) / sizeof(CONTACT_SWITCH_PINS[0]);
const uint32_t CONTACT_SWITCH_DEBOUNCE_MS = 25;

const int POT_CENTER_LOW_RAW = 502;
const int POT_CENTER_HIGH_RAW = 503;
const int POT_DEADZONE_COUNTS = 10;
const int DEFAULT_MAX_MOTOR_SPEED = 400;

const uint32_t COMMAND_SEND_INTERVAL_MS = 50;
const uint32_t STATUS_PRINT_INTERVAL_MS = 250;
const uint32_t ROBOT_STATUS_STALE_MS = 1000;

enum ActiveInputSource : uint8_t {
  ACTIVE_INPUT_NONE = 0,
  ACTIVE_INPUT_POTENTIOMETER,
  ACTIVE_INPUT_SWITCH_D3,
  ACTIVE_INPUT_SWITCH_D5
};

bool debouncedContactSwitchPressed[CONTACT_SWITCH_COUNT] = {false};
bool lastRawContactSwitchPressed[CONTACT_SWITCH_COUNT] = {false};
uint32_t lastRawContactSwitchChangeMs[CONTACT_SWITCH_COUNT] = {0};

ActiveInputSource activeInputSource = ACTIVE_INPUT_NONE;
bool emergencyStopLatched = false;

int currentPotValue = 0;
int currentRawMotorCommand = 0;
uint32_t currentCommandSequence = 0;

DemoStatusPacket latestRobotStatus = {};
bool robotStatusValid = false;
uint32_t lastRobotStatusReceivedMs = 0;

uint32_t lastCommandSendMs = 0;
uint32_t lastStatusPrintMs = 0;

bool xbeeFrameInProgress = false;
char xbeeFrameBuffer[ROBOT_PROTOCOL_MAX_PAYLOAD_LENGTH] = {0};
size_t xbeeFrameLength = 0;

int mapPotentiometerToMotorCommand(int rawValue, int maxMotorSpeed);
bool readContactSwitchRawPressed(uint8_t switchIndex);
void initializeContactSwitchDebounce(uint32_t nowMs);
void updateDebouncedContactSwitches(uint32_t nowMs);
void updateActiveInputSource(int potentiometerCommand);

int getActiveDriveMotorCommand();
DemoWinchCommand getActiveWinchCommand();
bool isContactSwitchRegisteredPressed(uint8_t switchIndex);
const char *getActiveInputSourceName();

void handleUsbSerialInput();
void handleUsbSerialCommand(char command);
void printHelp();

void sendCommandPacket(uint32_t nowMs);
void sendFramedXbeePayload(const char *payload);
void handleXbeeInput(uint32_t nowMs);
void handleXbeePayload(char *payload, uint32_t nowMs);
void handleXbeeCharacter(char incoming, uint32_t nowMs);

void printStatusLine();

void setup() {
  Serial.begin(USB_SERIAL_BAUD_RATE);
  xbeeSerial.begin(XBEE_BAUD_RATE);

  pinMode(POTENTIOMETER_PIN, INPUT);
  for (uint8_t i = 0; i < CONTACT_SWITCH_COUNT; ++i) {
    pinMode(CONTACT_SWITCH_PINS[i], INPUT_PULLUP);
  }

  initializeContactSwitchDebounce(millis());

  Serial.println("design_day_demo_controller");
  Serial.println("Potentiometer: A0");
  Serial.println("Switch D3: winch up");
  Serial.println("Switch D5: winch down");
  Serial.println("XBee: Serial2 framed command packets");
  printHelp();
}

void loop() {
  const uint32_t nowMs = millis();

  handleUsbSerialInput();
  handleXbeeInput(nowMs);

  updateDebouncedContactSwitches(nowMs);

  currentPotValue = analogRead(POTENTIOMETER_PIN);
  currentRawMotorCommand =
      mapPotentiometerToMotorCommand(currentPotValue, DEFAULT_MAX_MOTOR_SPEED);
  updateActiveInputSource(currentRawMotorCommand);

  if (lastCommandSendMs == 0 ||
      (nowMs - lastCommandSendMs) >= COMMAND_SEND_INTERVAL_MS) {
    sendCommandPacket(nowMs);
    lastCommandSendMs = nowMs;
  }

  if (lastStatusPrintMs == 0 ||
      (nowMs - lastStatusPrintMs) >= STATUS_PRINT_INTERVAL_MS) {
    printStatusLine();
    lastStatusPrintMs = nowMs;
  }
}

int mapPotentiometerToMotorCommand(int rawValue, int maxMotorSpeed) {
  const int deadzoneLow = POT_CENTER_LOW_RAW - POT_DEADZONE_COUNTS;
  const int deadzoneHigh = POT_CENTER_HIGH_RAW + POT_DEADZONE_COUNTS;

  if (rawValue >= deadzoneLow && rawValue <= deadzoneHigh) {
    return 0;
  }

  if (rawValue < deadzoneLow) {
    return (int)map(rawValue, 0, deadzoneLow - 1, -maxMotorSpeed, -1);
  }

  return (int)map(rawValue, deadzoneHigh + 1, 1023, 1, maxMotorSpeed);
}

bool readContactSwitchRawPressed(uint8_t switchIndex) {
  return digitalRead(CONTACT_SWITCH_PINS[switchIndex]) == LOW;
}

void initializeContactSwitchDebounce(uint32_t nowMs) {
  for (uint8_t i = 0; i < CONTACT_SWITCH_COUNT; ++i) {
    const bool rawPressed = readContactSwitchRawPressed(i);
    debouncedContactSwitchPressed[i] = rawPressed;
    lastRawContactSwitchPressed[i] = rawPressed;
    lastRawContactSwitchChangeMs[i] = nowMs;
  }
}

void updateDebouncedContactSwitches(uint32_t nowMs) {
  for (uint8_t i = 0; i < CONTACT_SWITCH_COUNT; ++i) {
    const bool rawPressed = readContactSwitchRawPressed(i);

    if (rawPressed != lastRawContactSwitchPressed[i]) {
      lastRawContactSwitchPressed[i] = rawPressed;
      lastRawContactSwitchChangeMs[i] = nowMs;
    }

    if (debouncedContactSwitchPressed[i] != rawPressed &&
        (nowMs - lastRawContactSwitchChangeMs[i]) >=
            CONTACT_SWITCH_DEBOUNCE_MS) {
      debouncedContactSwitchPressed[i] = rawPressed;
    }
  }
}

void updateActiveInputSource(int potentiometerCommand) {
  if (activeInputSource == ACTIVE_INPUT_POTENTIOMETER) {
    if (potentiometerCommand == 0) {
      activeInputSource = ACTIVE_INPUT_NONE;
    }
    return;
  }

  if (activeInputSource == ACTIVE_INPUT_SWITCH_D3) {
    if (!debouncedContactSwitchPressed[0]) {
      activeInputSource = ACTIVE_INPUT_NONE;
    }
    return;
  }

  if (activeInputSource == ACTIVE_INPUT_SWITCH_D5) {
    if (!debouncedContactSwitchPressed[1]) {
      activeInputSource = ACTIVE_INPUT_NONE;
    }
    return;
  }

  if (potentiometerCommand != 0) {
    activeInputSource = ACTIVE_INPUT_POTENTIOMETER;
    return;
  }

  if (debouncedContactSwitchPressed[0]) {
    activeInputSource = ACTIVE_INPUT_SWITCH_D3;
    return;
  }

  if (debouncedContactSwitchPressed[1]) {
    activeInputSource = ACTIVE_INPUT_SWITCH_D5;
  }
}

int getActiveDriveMotorCommand() {
  if (emergencyStopLatched) {
    return 0;
  }

  return activeInputSource == ACTIVE_INPUT_POTENTIOMETER
             ? currentRawMotorCommand
             : 0;
}

DemoWinchCommand getActiveWinchCommand() {
  if (emergencyStopLatched) {
    return DEMO_WINCH_COMMAND_NONE;
  }

  if (activeInputSource == ACTIVE_INPUT_SWITCH_D3) {
    return DEMO_WINCH_COMMAND_UP;
  }

  if (activeInputSource == ACTIVE_INPUT_SWITCH_D5) {
    return DEMO_WINCH_COMMAND_DOWN;
  }

  return DEMO_WINCH_COMMAND_NONE;
}

bool isContactSwitchRegisteredPressed(uint8_t switchIndex) {
  if (switchIndex == 0) {
    return activeInputSource == ACTIVE_INPUT_SWITCH_D3 &&
           debouncedContactSwitchPressed[switchIndex] && !emergencyStopLatched;
  }

  return activeInputSource == ACTIVE_INPUT_SWITCH_D5 &&
         debouncedContactSwitchPressed[switchIndex] && !emergencyStopLatched;
}

const char *getActiveInputSourceName() {
  if (emergencyStopLatched) {
    return "ESTOP";
  }

  switch (activeInputSource) {
    case ACTIVE_INPUT_POTENTIOMETER:
      return "POT";
    case ACTIVE_INPUT_SWITCH_D3:
      return "D3";
    case ACTIVE_INPUT_SWITCH_D5:
      return "D5";
    default:
      return "NONE";
  }
}

void handleUsbSerialInput() {
  while (Serial.available() > 0) {
    const char incoming = (char)Serial.read();
    if (incoming == '\r' || incoming == '\n') {
      continue;
    }

    handleUsbSerialCommand((char)tolower(incoming));
  }
}

void handleUsbSerialCommand(char command) {
  if (command == 'x') {
    emergencyStopLatched = true;
    Serial.println("Emergency stop latched. Send 'c' to clear.");
    return;
  }

  if (command == 'c') {
    emergencyStopLatched = false;
    Serial.println("Emergency stop cleared.");
    return;
  }

  if (command == 'p') {
    printStatusLine();
    return;
  }

  if (command == '?') {
    printHelp();
  }
}

void printHelp() {
  Serial.println("Serial commands:");
  Serial.println("  x = latch emergency stop");
  Serial.println("  c = clear emergency stop latch");
  Serial.println("  p = print one status line");
  Serial.println("  ? = print this help");
}

void sendCommandPacket(uint32_t nowMs) {
  (void)nowMs;

  DemoCommandPacket packet;
  packet.driveCommand = (int16_t)getActiveDriveMotorCommand();
  packet.winchCommand = getActiveWinchCommand();
  packet.emergencyStop = emergencyStopLatched;
  packet.sequence = ++currentCommandSequence;

  char payload[ROBOT_PROTOCOL_MAX_PAYLOAD_LENGTH];
  if (!formatDemoCommandPayload(packet, payload, sizeof(payload))) {
    Serial.println("Failed to format outbound command packet.");
    return;
  }

  sendFramedXbeePayload(payload);
}

void sendFramedXbeePayload(const char *payload) {
  xbeeSerial.print('<');
  xbeeSerial.print(payload);
  xbeeSerial.print('>');
  xbeeSerial.print('\n');
}

void handleXbeeInput(uint32_t nowMs) {
  while (xbeeSerial.available() > 0) {
    handleXbeeCharacter((char)xbeeSerial.read(), nowMs);
  }
}

void handleXbeeCharacter(char incoming, uint32_t nowMs) {
  if (incoming == '<') {
    xbeeFrameInProgress = true;
    xbeeFrameLength = 0;
    xbeeFrameBuffer[0] = '\0';
    return;
  }

  if (!xbeeFrameInProgress) {
    return;
  }

  if (incoming == '>') {
    xbeeFrameBuffer[xbeeFrameLength] = '\0';
    xbeeFrameInProgress = false;
    handleXbeePayload(xbeeFrameBuffer, nowMs);
    xbeeFrameLength = 0;
    return;
  }

  if (incoming == '\r' || incoming == '\n') {
    return;
  }

  if (xbeeFrameLength < (sizeof(xbeeFrameBuffer) - 1)) {
    xbeeFrameBuffer[xbeeFrameLength++] = incoming;
    xbeeFrameBuffer[xbeeFrameLength] = '\0';
    return;
  }

  xbeeFrameInProgress = false;
  xbeeFrameLength = 0;
}

void handleXbeePayload(char *payload, uint32_t nowMs) {
  DemoStatusPacket parsedStatus = {};
  if (!parseDemoStatusPayload(payload, parsedStatus)) {
    return;
  }

  latestRobotStatus = parsedStatus;
  robotStatusValid = true;
  lastRobotStatusReceivedMs = nowMs;
}

void printStatusLine() {
  const uint32_t nowMs = millis();

  Serial.print("pot=");
  Serial.print(currentPotValue);
  Serial.print(" raw_cmd=");
  Serial.print(currentRawMotorCommand);
  Serial.print(" active=");
  Serial.print(getActiveInputSourceName());
  Serial.print(" tx_drive=");
  Serial.print(getActiveDriveMotorCommand());
  Serial.print(" tx_winch=");
  Serial.print(getDemoWinchCommandName(getActiveWinchCommand()));
  Serial.print(" seq=");
  Serial.print((unsigned long)currentCommandSequence);

  for (uint8_t i = 0; i < CONTACT_SWITCH_COUNT; ++i) {
    Serial.print(" sw(D");
    Serial.print(CONTACT_SWITCH_PINS[i]);
    Serial.print(")=");
    Serial.print(isContactSwitchRegisteredPressed(i) ? "PRESSED" : "RELEASED");
  }

  if (robotStatusValid &&
      (nowMs - lastRobotStatusReceivedMs) <= ROBOT_STATUS_STALE_MS) {
    Serial.print(" robot_drive=");
    Serial.print(latestRobotStatus.appliedDriveCommand);
    Serial.print(" robot_winch=");
    Serial.print(getDemoWinchCommandName(latestRobotStatus.activeWinchCommand));
    Serial.print(" home=");
    Serial.print(latestRobotStatus.homeSwitchPressed ? "PRESSED" : "OPEN");
    Serial.print(" estop=");
    Serial.print(latestRobotStatus.emergencyStopActive ? "ON" : "OFF");
    Serial.print(" timeout=");
    Serial.print(latestRobotStatus.communicationTimedOut ? "YES" : "NO");
    Serial.print(" busy=");
    Serial.print(latestRobotStatus.winchBusy ? "YES" : "NO");
    Serial.print(" rx_seq=");
    Serial.print((unsigned long)latestRobotStatus.lastCommandSequence);
  } else {
    Serial.print(" robot_status=STALE");
  }

  Serial.println();
}
