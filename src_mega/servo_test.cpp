#include <Arduino.h>
#include <Servo.h>

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

/*
  servo_test.cpp

  Serial-driven servo tester for the Mega.

  Usage:
  - Open Serial Monitor at 115200 baud.
  - Type a signed integer, then press Enter.
  - The value is passed directly to Servo.write(...).
  - Standard Arduino Servo behavior applies:
      values below 0 are treated as 0 degrees
      values above 180 are treated as 180 degrees
*/

namespace {

Servo testServo;

const uint8_t SERVO_PIN = 13;

char serialBuffer[32];
uint8_t serialIndex = 0;

void printHelp() {
  Serial.println();
  Serial.println("servo_test ready");
  Serial.print("Servo pin: ");
  Serial.println(SERVO_PIN);
  Serial.println("Enter a signed integer and press Enter.");
  Serial.println("The value is passed directly to Servo.write(...).");
  Serial.println("Example commands: -90, 0, 90, 130, 180");
  Serial.println("Type 'help' to print this message again.");
  Serial.println();
}

void moveServoToRawCommand(int servoCommand) {
  testServo.write(servoCommand);

  Serial.print("Raw Servo.write command: ");
  Serial.print(servoCommand);
  Serial.print(" | Servo.read(): ");
  Serial.println(testServo.read());
}

char *skipLeadingWhitespace(char *text) {
  while (*text != '\0' && isspace((unsigned char)*text)) {
    ++text;
  }
  return text;
}

bool isEndOfTextAfterWhitespace(const char *text) {
  while (*text != '\0') {
    if (!isspace((unsigned char)*text)) {
      return false;
    }
    ++text;
  }
  return true;
}

void processSerialLine(char *line) {
  char *trimmedLine = skipLeadingWhitespace(line);

  if (*trimmedLine == '\0') {
    return;
  }

  if (strcmp(trimmedLine, "help") == 0 || strcmp(trimmedLine, "?") == 0) {
    printHelp();
    return;
  }

  char *parseEnd = NULL;
  const long requestedPosition = strtol(trimmedLine, &parseEnd, 10);

  if (parseEnd == trimmedLine || !isEndOfTextAfterWhitespace(parseEnd)) {
    Serial.print("Invalid command: ");
    Serial.println(trimmedLine);
    Serial.println("Enter an integer like -90, 0, 90, or 180.");
    return;
  }

  moveServoToRawCommand((int)requestedPosition);
}

void handleSerialInput() {
  while (Serial.available() > 0) {
    const char incoming = (char)Serial.read();

    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      serialBuffer[serialIndex] = '\0';
      processSerialLine(serialBuffer);
      serialIndex = 0;
      continue;
    }

    if (serialIndex >= sizeof(serialBuffer) - 1) {
      serialIndex = 0;
      Serial.println("Input too long. Command cleared.");
      continue;
    }

    serialBuffer[serialIndex++] = incoming;
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  testServo.attach(SERVO_PIN);

  moveServoToRawCommand(0);
  printHelp();
}

void loop() {
  handleSerialInput();
}
