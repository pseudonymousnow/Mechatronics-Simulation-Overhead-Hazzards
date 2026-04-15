#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "DualG2HighPowerMotorShield.h"

/*
  serial_motor_command_test.cpp

  Very simple Mega serial motor command test.

  Default behavior:
  - send an integer from -400 to 400 over USB Serial
  - the command is written to M2
  - select M1 instead with "m1", or toggle with "t"
  - "x" latches an emergency stop and stops both motors
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

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint32_t SERIAL_STARTUP_DELAY_MS = 500;
const uint32_t FAULT_PRINT_INTERVAL_MS = 500;
const int MAX_MOTOR_COMMAND = 400;

// Match the motor direction settings used by design_day_demo_robot.cpp.
const bool FLIP_M1_MOTOR = false;
const bool FLIP_M2_MOTOR = true;

enum ActiveMotor : uint8_t {
  ACTIVE_M1 = 1,
  ACTIVE_M2 = 2
};

ActiveMotor activeMotor = ACTIVE_M2;

int appliedM1Command = 0;
int appliedM2Command = 0;
bool emergencyStopActive = false;
bool motorFaultLatched = false;
uint32_t lastFaultPrintMs = 0;

char serialBuffer[48];
uint8_t serialIndex = 0;

void handleSerialInput();
void processCommand(char *commandLine);
void processMotorCommandToken(const char *token);
void applyMotorCommand(int motorCommand);
void selectActiveMotor(ActiveMotor motor);
void toggleActiveMotor();
void stopBothMotors();
void emergencyStop(const char *reason);
void clearEmergencyStop();
void checkMotorFaults();
void printHelp();
void printStatus();
char *trimWhitespace(char *text);
void lowercase(char *text);
const char *activeMotorName();
bool parseMotorCommand(const char *token, int *motorCommand);
bool hasExtraToken();

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(SERIAL_STARTUP_DELAY_MS);

  md.init();
  md.enableDrivers();
  md.flipM1(FLIP_M1_MOTOR);
  md.flipM2(FLIP_M2_MOTOR);
  stopBothMotors();

  printHelp();
  printStatus();
}

void loop() {
  handleSerialInput();
  checkMotorFaults();
}

void handleSerialInput() {
  while (Serial.available() > 0) {
    const char incoming = Serial.read();

    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      serialBuffer[serialIndex] = '\0';
      processCommand(serialBuffer);
      serialIndex = 0;
      continue;
    }

    if (serialIndex < sizeof(serialBuffer) - 1) {
      serialBuffer[serialIndex++] = incoming;
    } else {
      serialIndex = 0;
      stopBothMotors();
      Serial.println("Serial command too long. Motors stopped.");
    }
  }
}

void processCommand(char *commandLine) {
  char *command = trimWhitespace(commandLine);
  if (command[0] == '\0') {
    return;
  }

  char *token = strtok(command, " \t");
  if (token == NULL) {
    return;
  }

  lowercase(token);

  if (strcmp(token, "h") == 0 || strcmp(token, "?") == 0) {
    printHelp();
    return;
  }

  if (strcmp(token, "p") == 0) {
    printStatus();
    return;
  }

  if (strcmp(token, "x") == 0) {
    emergencyStop("Emergency stop command received.");
    return;
  }

  if (strcmp(token, "r") == 0) {
    clearEmergencyStop();
    return;
  }

  if (strcmp(token, "t") == 0) {
    toggleActiveMotor();
    return;
  }

  if (strcmp(token, "m1") == 0) {
    selectActiveMotor(ACTIVE_M1);

    char *speedToken = strtok(NULL, " \t");
    if (speedToken != NULL) {
      processMotorCommandToken(speedToken);
    }
    return;
  }

  if (strcmp(token, "m2") == 0) {
    selectActiveMotor(ACTIVE_M2);

    char *speedToken = strtok(NULL, " \t");
    if (speedToken != NULL) {
      processMotorCommandToken(speedToken);
    }
    return;
  }

  processMotorCommandToken(token);
}

void processMotorCommandToken(const char *token) {
  int motorCommand = 0;

  if (!parseMotorCommand(token, &motorCommand) || hasExtraToken()) {
    Serial.println("Invalid command. Send h for help.");
    return;
  }

  if (emergencyStopActive) {
    Serial.println("Command ignored. Emergency stop is active. Send r to re-enable.");
    return;
  }

  applyMotorCommand(motorCommand);
  Serial.print("Applied ");
  Serial.print(activeMotorName());
  Serial.print(" command: ");
  Serial.println(motorCommand);
}

void applyMotorCommand(int motorCommand) {
  const int constrainedCommand =
      constrain(motorCommand, -MAX_MOTOR_COMMAND, MAX_MOTOR_COMMAND);

  if (activeMotor == ACTIVE_M1) {
    appliedM1Command = constrainedCommand;
    appliedM2Command = 0;
  } else {
    appliedM1Command = 0;
    appliedM2Command = constrainedCommand;
  }

  md.setSpeeds(appliedM1Command, appliedM2Command);
}

void selectActiveMotor(ActiveMotor motor) {
  if (activeMotor != motor) {
    stopBothMotors();
    activeMotor = motor;
  }

  Serial.print("Selected ");
  Serial.print(activeMotorName());
  Serial.println(".");
}

void toggleActiveMotor() {
  const ActiveMotor nextMotor =
      (activeMotor == ACTIVE_M1) ? ACTIVE_M2 : ACTIVE_M1;
  selectActiveMotor(nextMotor);
}

void stopBothMotors() {
  appliedM1Command = 0;
  appliedM2Command = 0;
  md.setSpeeds(0, 0);
}

void emergencyStop(const char *reason) {
  stopBothMotors();
  emergencyStopActive = true;

  if (reason != NULL && reason[0] != '\0') {
    Serial.println(reason);
  }

  Serial.println("Both motors stopped. Send r to re-enable motor commands.");
}

void clearEmergencyStop() {
  if (md.getM1Fault() || md.getM2Fault()) {
    stopBothMotors();
    emergencyStopActive = true;
    motorFaultLatched = true;
    Serial.println("Cannot clear emergency stop while a motor fault is active.");
    printStatus();
    return;
  }

  emergencyStopActive = false;
  motorFaultLatched = false;
  stopBothMotors();
  Serial.println("Emergency stop cleared. Motors remain at 0.");
  printStatus();
}

void checkMotorFaults() {
  if (!md.getM1Fault() && !md.getM2Fault()) {
    return;
  }

  emergencyStopActive = true;
  motorFaultLatched = true;
  stopBothMotors();

  const uint32_t nowMs = millis();
  if (lastFaultPrintMs == 0 ||
      (nowMs - lastFaultPrintMs) >= FAULT_PRINT_INTERVAL_MS) {
    lastFaultPrintMs = nowMs;
    Serial.println("Motor fault detected. Both motors stopped.");
    printStatus();
  }
}

void printHelp() {
  Serial.println();
  Serial.println("Mega serial motor command test");
  Serial.println("Default output is M2. Commands are read from USB Serial.");
  Serial.println("Commands:");
  Serial.println("  -400..400      -> command the selected motor");
  Serial.println("  m2             -> select M2");
  Serial.println("  m1             -> select M1");
  Serial.println("  m2 <cmd>       -> select M2 and apply command");
  Serial.println("  m1 <cmd>       -> select M1 and apply command");
  Serial.println("  t              -> toggle selected motor, stopping both first");
  Serial.println("  x              -> emergency stop, latch both motors at 0");
  Serial.println("  r              -> clear emergency stop latch");
  Serial.println("  p              -> print status");
  Serial.println("  h or ?         -> print this help");
  Serial.println();
}

void printStatus() {
  Serial.print("Selected: ");
  Serial.print(activeMotorName());
  Serial.print(" | M1 cmd: ");
  Serial.print(appliedM1Command);
  Serial.print(" | M2 cmd: ");
  Serial.print(appliedM2Command);
  Serial.print(" | e-stop: ");
  Serial.print(emergencyStopActive ? "ON" : "off");
  Serial.print(" | M1 fault: ");
  Serial.print(md.getM1Fault() ? "YES" : "no");
  Serial.print(" | M2 fault: ");
  Serial.println(md.getM2Fault() ? "YES" : "no");
}

char *trimWhitespace(char *text) {
  while (isspace((unsigned char)*text)) {
    ++text;
  }

  if (text[0] == '\0') {
    return text;
  }

  char *end = text + strlen(text) - 1;
  while (end > text && isspace((unsigned char)*end)) {
    *end = '\0';
    --end;
  }

  return text;
}

void lowercase(char *text) {
  while (*text != '\0') {
    *text = (char)tolower((unsigned char)*text);
    ++text;
  }
}

const char *activeMotorName() {
  return (activeMotor == ACTIVE_M1) ? "M1" : "M2";
}

bool parseMotorCommand(const char *token, int *motorCommand) {
  if (token == NULL || token[0] == '\0') {
    return false;
  }

  char *endPtr = NULL;
  const long parsedCommand = strtol(token, &endPtr, 10);

  if (endPtr == token || *endPtr != '\0') {
    return false;
  }

  if (parsedCommand < -MAX_MOTOR_COMMAND ||
      parsedCommand > MAX_MOTOR_COMMAND) {
    return false;
  }

  *motorCommand = (int)parsedCommand;
  return true;
}

bool hasExtraToken() {
  return strtok(NULL, " \t") != NULL;
}
