#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include "DualG2HighPowerMotorShield.h"

/*
  contact_switch_drive_test.cpp

  Drive motor M1 continuously using a commanded motor value until the contact
  switch is pressed. When the switch is hit, the sketch:
  1. stops the motor,
  2. waits briefly so the drivetrain is not shocked,
  3. reverses direction,
  4. increments a direction-change counter.

  Serial commands:
    g <motorcommand>  Start the run using the requested motor command.
    x                 Stop the robot immediately.
    p                 Print one status line.
    ?                 Print help.
*/

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
DualG2HighPowerMotorShield md(M1nSLEEP, M1DIR, M1PWM, M1nFAULT, M1CS, M2nSLEEP, M2DIR, M2PWM, M2nFAULT, M2CS);

const bool FLIP_DRIVE_MOTOR = false;
const uint8_t CONTACT_SWITCH_PIN = 2;  
const int MAX_MOTOR_CMD = 400;
const uint32_t DIRECTION_CHANGE_PAUSE_MS = 300;
const uint32_t STATUS_PRINT_INTERVAL_MS = 500;

enum RunState : uint8_t {
  STATE_IDLE = 0,
  STATE_RUNNING,
  STATE_REVERSAL_PAUSE
};

RunState runState = STATE_IDLE;

int runMotorCommand = 0;
int appliedMotorCommand = 0;
int pendingMotorCommandAfterPause = 0;
uint32_t directionChangeCount = 0;
uint32_t reversalPauseStartedMs = 0;
uint32_t lastStatusPrintMs = 0;
bool switchWasPressed = false;
bool switchReadyForNextTrigger = true;

char serialBuffer[48];
uint8_t serialIndex = 0;

bool isContactSwitchPressed();
void applyMotorCommand(int motorCommand);
void stopMotor();
void startRun(int motorCommand);
void stopRun(const char *message);
void updateDriveState();
void handleSerialInput();
void processCommand(char *commandLine);
void printHelp();
void printStatus();
const char *getRunStateName(RunState state);

void setup() {
  Serial.begin(115200);

  pinMode(CONTACT_SWITCH_PIN, INPUT_PULLUP);

  md.init();
  md.enableDrivers();
  md.flipM1(FLIP_DRIVE_MOTOR);
  stopMotor();

  printHelp();
  Serial.println("Contact switch uses INPUT_PULLUP, so LOW means pressed.");
}

void loop() {
  handleSerialInput();
  updateDriveState();

  if (runState != STATE_IDLE) {
    const uint32_t nowMs = millis();
    if (lastStatusPrintMs == 0 ||
        (nowMs - lastStatusPrintMs) >= STATUS_PRINT_INTERVAL_MS) {
      printStatus();
      lastStatusPrintMs = nowMs;
    }
  }
}

bool isContactSwitchPressed() {
  return digitalRead(CONTACT_SWITCH_PIN) == LOW;
}

void applyMotorCommand(int motorCommand) {
  appliedMotorCommand = constrain(motorCommand, -MAX_MOTOR_CMD, MAX_MOTOR_CMD);
  md.setM1Speed(appliedMotorCommand);
}

void stopMotor() {
  applyMotorCommand(0);
}

void startRun(int motorCommand) {
  runMotorCommand = constrain(motorCommand, -MAX_MOTOR_CMD, MAX_MOTOR_CMD);
  directionChangeCount = 0;
  pendingMotorCommandAfterPause = 0;
  reversalPauseStartedMs = 0;
  runState = STATE_RUNNING;
  switchWasPressed = isContactSwitchPressed();
  switchReadyForNextTrigger = !switchWasPressed;
  lastStatusPrintMs = 0;

  applyMotorCommand(runMotorCommand);

  Serial.print("Run started with motor command ");
  Serial.print(runMotorCommand);
  if (switchWasPressed) {
    Serial.println(". Switch is already pressed, waiting for release before counting a hit.");
  } else {
    Serial.println(".");
  }
}

void stopRun(const char *message) {
  runState = STATE_IDLE;
  runMotorCommand = 0;
  pendingMotorCommandAfterPause = 0;
  reversalPauseStartedMs = 0;
  switchWasPressed = isContactSwitchPressed();
  switchReadyForNextTrigger = !switchWasPressed;
  lastStatusPrintMs = 0;
  stopMotor();

  if (message != NULL && message[0] != '\0') {
    Serial.println(message);
  }
}

void updateDriveState() {
  if (md.getM1Fault()) {
    stopRun("Motor fault detected. Drive stopped.");
    return;
  }

  const bool switchPressed = isContactSwitchPressed();

  if (!switchPressed) {
    switchReadyForNextTrigger = true;
  }

  if (runState == STATE_IDLE) {
    switchWasPressed = switchPressed;
    return;
  }

  if (runState == STATE_RUNNING) {
    const bool pressedEdge = switchPressed && !switchWasPressed;
    if (pressedEdge && switchReadyForNextTrigger) {
      stopMotor();
      ++directionChangeCount;
      pendingMotorCommandAfterPause = -runMotorCommand;
      reversalPauseStartedMs = millis();
      runState = STATE_REVERSAL_PAUSE;
      switchReadyForNextTrigger = false;

      Serial.print("Switch hit. Pausing before reversing. Direction changes: ");
      Serial.println(directionChangeCount);
    }
  } else if (runState == STATE_REVERSAL_PAUSE) {
    if ((millis() - reversalPauseStartedMs) >= DIRECTION_CHANGE_PAUSE_MS) {
      runMotorCommand = pendingMotorCommandAfterPause;
      pendingMotorCommandAfterPause = 0;
      applyMotorCommand(runMotorCommand);
      runState = STATE_RUNNING;

      Serial.print("Direction reversed. New motor command: ");
      Serial.println(runMotorCommand);
    }
  }

  switchWasPressed = switchPressed;
}

void handleSerialInput() {
  while (Serial.available() > 0) {
    const char incoming = (char)Serial.read();

    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      serialBuffer[serialIndex] = '\0';
      processCommand(serialBuffer);
      serialIndex = 0;
      continue;
    }

    if (serialIndex < (sizeof(serialBuffer) - 1)) {
      serialBuffer[serialIndex++] = incoming;
    }
  }
}

void processCommand(char *commandLine) {
  if (commandLine[0] == '\0') {
    return;
  }

  char *verb = strtok(commandLine, " ");
  if (verb == NULL) {
    return;
  }

  if (strcmp(verb, "g") == 0) {
    char *arg = strtok(NULL, " ");
    if (arg == NULL) {
      Serial.println("Usage: g <motorcommand>");
      return;
    }

    const long parsedCommand = strtol(arg, NULL, 10);
    if (parsedCommand == 0) {
      Serial.println("Motor command must be non-zero.");
      return;
    }

    startRun((int)parsedCommand);
    return;
  }

  if (strcmp(verb, "x") == 0) {
    stopRun("Drive stopped.");
    return;
  }

  if (strcmp(verb, "p") == 0) {
    printStatus();
    return;
  }

  if (strcmp(verb, "?") == 0 || strcmp(verb, "h") == 0) {
    printHelp();
    return;
  }

  Serial.println("Unknown command. Use ? for help.");
}

void printHelp() {
  Serial.println("Contact switch drive test");
  Serial.println("Commands:");
  Serial.println("  g <motorcommand> -> start driving with that motor command");
  Serial.println("  x                -> stop the robot");
  Serial.println("  p                -> print one status line");
  Serial.println("  ?                -> print this help");
  Serial.print("Switch pin: ");
  Serial.println(CONTACT_SWITCH_PIN);
  Serial.print("Reverse pause (ms): ");
  Serial.println(DIRECTION_CHANGE_PAUSE_MS);
}

void printStatus() {
  Serial.print("State: ");
  Serial.print(getRunStateName(runState));
  Serial.print(" | Cmd: ");
  Serial.print(appliedMotorCommand);
  Serial.print(" | NextCmd: ");
  Serial.print(runMotorCommand);
  Serial.print(" | Changes: ");
  Serial.print(directionChangeCount);
  Serial.print(" | Switch: ");
  Serial.print(isContactSwitchPressed() ? "PRESSED" : "OPEN");

  if (md.getM1Fault()) {
    Serial.print(" | MOTOR_FAULT");
  }

  Serial.println();
}

const char *getRunStateName(RunState state) {
  switch (state) {
    case STATE_IDLE:
      return "IDLE";
    case STATE_RUNNING:
      return "RUNNING";
    case STATE_REVERSAL_PAUSE:
      return "PAUSE";
    default:
      return "?";
  }
}
