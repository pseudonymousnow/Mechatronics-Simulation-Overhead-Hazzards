#include <Arduino.h>
#include <Servo.h>

#include "DualG2HighPowerMotorShield.h"
#include "RobotProtocol.h"
#include "WinchControl.h"

/*
  design_day_demo_robot.cpp

  Robot-side design day demo sketch.

  Responsibilities:
  - receive wireless command packets from the handheld controller over XBee
  - apply the drive motor command to M1 for forward/backward rail motion
  - use WinchControl on M2 for non-blocking winch raise/lower behavior
  - enforce safety rules for emergency stop and communication timeout

  Packet expectations:
    <CMD,driveCmd,winchCmd,estop,sequence>

  Status packets sent back to the controller:
    <STS,appliedDrive,activeWinch,homeSwitch,estop,timeout,winchBusy,lastSeq>
*/

#define xbeeSerial Serial2

#pragma region Hardware_Objects

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
Servo pawlServo;

#pragma endregion

#pragma region Communication_Configuration

const uint32_t USB_SERIAL_BAUD_RATE = 115200;
const uint32_t XBEE_BAUD_RATE = 9600;
const uint32_t COMMAND_TIMEOUT_MS = 500;
const uint32_t STATUS_SEND_INTERVAL_MS = 200;
const uint32_t LOCAL_STATUS_PRINT_INTERVAL_MS = 500;
const uint32_t USB_SERIAL_STARTUP_DELAY_MS = 500;

#pragma endregion

#pragma region Drive_Configuration

const bool FLIP_DRIVE_MOTOR = false;
const int DRIVE_MAX_MOTOR_COMMAND = 400;

#pragma endregion

#pragma region Winch_Configuration

const uint8_t PAWL_SERVO_PIN = 13;
const uint8_t WINCH_HOME_SWITCH_PIN = 3;

const uint8_t PAWL_SERVO_LOCK_POS = 90;
const uint8_t PAWL_SERVO_OPEN_POS = 0;

const uint8_t I2C_MUX_ADDR = 0x70;
const uint8_t AS5600_ADDR = 0x36;
const uint8_t AS5600_RAW_ANGLE_REG = 0x0C;
const uint8_t WINCH_ENCODER_MUX_CH = 1;
const bool FLIP_WINCH_ENCODER_SIGN = true;
const int WINCH_ENCODER_COUNTS_PER_REV = 4096;
const int WINCH_WRAP_THRESHOLD = WINCH_ENCODER_COUNTS_PER_REV / 2;
const int WINCH_NOISE_THRESHOLD = 4;

const float WINCH_DRUM_RADIUS_IN = (1.502f / 2.0f) + (1.0f / 32.0f);

const bool FLIP_WINCH_MOTOR_DIRECTION = true;
const int WINCH_RAISE_SPEED_CMD = 200;
const int WINCH_LOWER_SPEED_CMD = -200;
const int WINCH_HOMING_SLOW_SPEED_CMD = 140;
const int WINCH_MAX_MOTOR_COMMAND = 400;
const int WINCH_JOG_UP_MIN_COMMAND = 80;
const uint32_t PAWL_LOCK_DELAY_MS = 500;
const uint32_t PAWL_UNLOCK_DELAY_MS = 400;

const float JOG_UP_TARGET_SPEED_IN_S = 0.15f;
const float JOG_UP_RELIEF_DISTANCE_IN = 0.0625f;
const float JOG_UP_MAX_TRAVEL_IN = 0.50f;
const uint32_t JOG_UP_RELIEF_DEBOUNCE_MS = 200;
const float JOG_UP_CMD_RATE_PER_SEC = 350.0f;
const float JOG_UP_KP_SPEED = 900.0f;

const float HOMING_BACKOFF_DISTANCE_IN = 1.0f;
const float HOMING_MAX_SEARCH_TRAVEL_IN = 60.0f;
const float HOMING_SLOWDOWN_THRESHOLD_IN = -2.0f;
const float HOMING_MAX_OVERSHOOT_IN = 0.25f;

const uint32_t WINCH_FILTER_MIN_DT_MS = 10;
const float WINCH_FILTER_TAU_S = 0.05f;
const float DEFAULT_TEST_LOWER_TARGET_IN = -10.0f;

#pragma endregion

#pragma region Runtime_State

DemoCommandPacket latestCommand = {0,
                                   DEMO_WINCH_COMMAND_NONE,
                                   false,
                                   0};
DemoWinchCommand activeWinchCommand = DEMO_WINCH_COMMAND_NONE;

int appliedDriveMotorCommand = 0;
bool commandEverReceived = false;
bool communicationTimedOut = true;
bool emergencyStopActive = false;
bool safetyStopIssued = false;

uint32_t lastCommandReceivedMs = 0;
uint32_t lastStatusSendMs = 0;
uint32_t lastLocalStatusPrintMs = 0;

bool xbeeFrameInProgress = false;
char xbeeFrameBuffer[ROBOT_PROTOCOL_MAX_PAYLOAD_LENGTH] = {0};
size_t xbeeFrameLength = 0;

#pragma endregion

#pragma region Function_Prototypes

void configureWinchControl();

void handleXbeeInput(uint32_t nowMs);
void handleXbeeCharacter(char incoming, uint32_t nowMs);
void handleXbeePayload(char *payload, uint32_t nowMs);
void sendFramedXbeePayload(const char *payload);
void sendStatusPacket();
void handleUsbDebugInput();
void printUsbDebugHelp();

void updateCommandTimeout(uint32_t nowMs);
void applyDriveMotorCommand(int motorCommand);
void stopDriveMotor();
void issueSafetyStop();
void serviceDriveAndWinchCommands();
void serviceRequestedWinchCommand();

void printStatusLine();

#pragma endregion

void setup() {
  Serial.begin(USB_SERIAL_BAUD_RATE);
  delay(USB_SERIAL_STARTUP_DELAY_MS);
  Serial.println("design_day_demo_robot booting");
  Serial.println("USB serial monitor is debug/status only. Robot commands come from Serial2 XBee.");
  Serial.flush();

  xbeeSerial.begin(XBEE_BAUD_RATE);

  Serial.println("Initializing drive motor shield...");
  Serial.flush();
  md.init();
  md.enableDrivers();
  md.flipM1(FLIP_DRIVE_MOTOR);
  stopDriveMotor();

  pawlServo.write(90); // this to make sure it starts lockedx

  Serial.println("Initializing WinchControl...");
  Serial.flush();
  configureWinchControl();
  const bool beginSucceeded = beginWinchControl(0.0f);

  Serial.println("design_day_demo_robot");
  Serial.println("Drive motor: M1");
  Serial.println("Winch motor: M2 via WinchControl");
  Serial.println("Wireless commands: Serial2 XBee framed packets");

  if (!beginSucceeded) {
    Serial.println("WinchControl begin failed. Check encoder and home-switch wiring.");
  }
}

void loop() {
  const uint32_t nowMs = millis();

  handleUsbDebugInput();
  handleXbeeInput(nowMs);
  updateCommandTimeout(nowMs);
  serviceDriveAndWinchCommands();
  updateWinchControl();

  if (lastStatusSendMs == 0 ||
      (nowMs - lastStatusSendMs) >= STATUS_SEND_INTERVAL_MS) {
    sendStatusPacket();
    lastStatusSendMs = nowMs;
  }

  if (lastLocalStatusPrintMs == 0 ||
      (nowMs - lastLocalStatusPrintMs) >= LOCAL_STATUS_PRINT_INTERVAL_MS) {
    printStatusLine();
    lastLocalStatusPrintMs = nowMs;
  }
}

void configureWinchControl() {
  setWinchHardwareBindings(&md, &pawlServo);
  setWinchHardwarePins(PAWL_SERVO_PIN,
                       WINCH_HOME_SWITCH_PIN,
                       FLIP_WINCH_MOTOR_DIRECTION);

  setWinchPawlParameters(PAWL_SERVO_LOCK_POS,
                         PAWL_SERVO_OPEN_POS,
                         PAWL_LOCK_DELAY_MS,
                         PAWL_UNLOCK_DELAY_MS);

  setWinchEncoderParameters(WINCH_ENCODER_MUX_CH,
                            WINCH_DRUM_RADIUS_IN,
                            FLIP_WINCH_ENCODER_SIGN,
                            WINCH_ENCODER_COUNTS_PER_REV,
                            WINCH_WRAP_THRESHOLD,
                            WINCH_NOISE_THRESHOLD,
                            I2C_MUX_ADDR,
                            AS5600_ADDR,
                            AS5600_RAW_ANGLE_REG);

  setWinchMotionParameters(WINCH_RAISE_SPEED_CMD,
                           WINCH_LOWER_SPEED_CMD,
                           WINCH_HOMING_SLOW_SPEED_CMD,
                           WINCH_MAX_MOTOR_COMMAND,
                           WINCH_JOG_UP_MIN_COMMAND,
                           DEFAULT_TEST_LOWER_TARGET_IN,
                           JOG_UP_TARGET_SPEED_IN_S,
                           JOG_UP_RELIEF_DISTANCE_IN,
                           JOG_UP_MAX_TRAVEL_IN,
                           JOG_UP_CMD_RATE_PER_SEC,
                           JOG_UP_KP_SPEED,
                           JOG_UP_RELIEF_DEBOUNCE_MS);

  setWinchSpeedFilterParameters(WINCH_FILTER_MIN_DT_MS, WINCH_FILTER_TAU_S);
  setWinchHomingParameters(HOMING_BACKOFF_DISTANCE_IN,
                           HOMING_MAX_SEARCH_TRAVEL_IN,
                           HOMING_SLOWDOWN_THRESHOLD_IN,
                           HOMING_MAX_OVERSHOOT_IN);
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
  DemoCommandPacket parsedPacket = {};
  if (!parseDemoCommandPayload(payload, parsedPacket)) {
    return;
  }

  latestCommand = parsedPacket;
  emergencyStopActive = parsedPacket.emergencyStop;
  commandEverReceived = true;
  communicationTimedOut = false;
  lastCommandReceivedMs = nowMs;
}

void sendFramedXbeePayload(const char *payload) {
  xbeeSerial.print('<');
  xbeeSerial.print(payload);
  xbeeSerial.print('>');
  xbeeSerial.print('\n');
}

void sendStatusPacket() {
  DemoStatusPacket statusPacket;
  statusPacket.appliedDriveCommand = (int16_t)appliedDriveMotorCommand;
  statusPacket.activeWinchCommand = activeWinchCommand;
  statusPacket.homeSwitchPressed = isWinchHomeSwitchPressed();
  statusPacket.emergencyStopActive = emergencyStopActive;
  statusPacket.communicationTimedOut = communicationTimedOut;
  statusPacket.winchBusy = isWinchBusy();
  statusPacket.lastCommandSequence = latestCommand.sequence;

  char payload[ROBOT_PROTOCOL_MAX_PAYLOAD_LENGTH];
  if (!formatDemoStatusPayload(statusPacket, payload, sizeof(payload))) {
    Serial.println("Failed to format status packet.");
    return;
  }

  sendFramedXbeePayload(payload);
}

void handleUsbDebugInput() {
  while (Serial.available() > 0) {
    const char command = (char)Serial.read();

    switch (command) {
      case '?':
        printUsbDebugHelp();
        break;
      case 'p':
      case 'P':
        printStatusLine();
        break;
      case '\r':
      case '\n':
        break;
      default:
        Serial.println("Unknown USB debug command. Type ? for help.");
        break;
    }
  }
}

void printUsbDebugHelp() {
  Serial.println("USB debug commands:");
  Serial.println("  p = print one robot status line");
  Serial.println("  ? = print this help");
  Serial.println("Robot motion commands are received from Serial2 XBee, not USB Serial.");
}

void updateCommandTimeout(uint32_t nowMs) {
  if (!commandEverReceived) {
    communicationTimedOut = true;
    return;
  }

  communicationTimedOut =
      (nowMs - lastCommandReceivedMs) > COMMAND_TIMEOUT_MS;
}

void applyDriveMotorCommand(int motorCommand) {
  appliedDriveMotorCommand =
      constrain(motorCommand, -DRIVE_MAX_MOTOR_COMMAND, DRIVE_MAX_MOTOR_COMMAND);
  md.setM1Speed(appliedDriveMotorCommand);
}

void stopDriveMotor() {
  applyDriveMotorCommand(0);
}

void issueSafetyStop() {
  stopDriveMotor();

  if (activeWinchCommand != DEMO_WINCH_COMMAND_NONE || isWinchBusy()) {
    finishWinchAction(true);
  }

  activeWinchCommand = DEMO_WINCH_COMMAND_NONE;
}

void serviceDriveAndWinchCommands() {
  const bool shouldForceStop = emergencyStopActive || communicationTimedOut;

  if (shouldForceStop) {
    if (!safetyStopIssued) {
      issueSafetyStop();
      safetyStopIssued = true;
    }
    return;
  }

  safetyStopIssued = false;

  serviceRequestedWinchCommand();

  const bool winchOwnsTheCommand =
      latestCommand.winchCommand != DEMO_WINCH_COMMAND_NONE;
  const int requestedDriveCommand =
      winchOwnsTheCommand ? 0 : latestCommand.driveCommand;

  if (md.getM1Fault()) {
    stopDriveMotor();
    Serial.println("Drive motor fault detected. Drive stopped.");
    return;
  }

  applyDriveMotorCommand(requestedDriveCommand);
}

void serviceRequestedWinchCommand() {
  if (latestCommand.winchCommand == DEMO_WINCH_COMMAND_NONE) {
    if (activeWinchCommand != DEMO_WINCH_COMMAND_NONE || isWinchBusy()) {
      finishWinchAction(true);
    }
    activeWinchCommand = DEMO_WINCH_COMMAND_NONE;
    return;
  }

  if (latestCommand.winchCommand == DEMO_WINCH_COMMAND_UP &&
      isWinchHomeSwitchPressed()) {
    if (activeWinchCommand != DEMO_WINCH_COMMAND_NONE || isWinchBusy()) {
      finishWinchAction(true);
    }
    activeWinchCommand = DEMO_WINCH_COMMAND_NONE;
    return;
  }

  if (activeWinchCommand == latestCommand.winchCommand) {
    return;
  }

  if (activeWinchCommand != DEMO_WINCH_COMMAND_NONE || isWinchBusy()) {
    finishWinchAction(true);
    activeWinchCommand = DEMO_WINCH_COMMAND_NONE;
    return;
  }

  if (latestCommand.winchCommand == DEMO_WINCH_COMMAND_UP) {
    startWinchAction(WINCH_ACTION_RAISE);
    activeWinchCommand = DEMO_WINCH_COMMAND_UP;
    return;
  }

  startWinchAction(WINCH_ACTION_MANUAL_LOWER);
  activeWinchCommand = DEMO_WINCH_COMMAND_DOWN;
}

void printStatusLine() {
  const WinchStatus winchStatus = getWinchStatus();

  Serial.print("seq=");
  Serial.print((unsigned long)latestCommand.sequence);
  Serial.print(" drive_cmd=");
  Serial.print(latestCommand.driveCommand);
  Serial.print(" drive_applied=");
  Serial.print(appliedDriveMotorCommand);
  Serial.print(" winch_req=");
  Serial.print(getDemoWinchCommandName(latestCommand.winchCommand));
  Serial.print(" winch_active=");
  Serial.print(getDemoWinchCommandName(activeWinchCommand));
  Serial.print(" estop=");
  Serial.print(emergencyStopActive ? "ON" : "OFF");
  Serial.print(" timeout=");
  Serial.print(communicationTimedOut ? "YES" : "NO");
  Serial.print(" winch_step=");
  Serial.print(getWinchStepName(winchStatus.currentStep));
  Serial.print(" winch_pos=");
  Serial.print(winchStatus.positionIn, 3);
  Serial.print(" home=");
  Serial.print(winchStatus.homeSwitchPressed ? "PRESSED" : "OPEN");
  Serial.println();
}
