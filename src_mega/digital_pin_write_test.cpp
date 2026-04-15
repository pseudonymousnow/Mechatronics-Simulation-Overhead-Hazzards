#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

/*
  Mega digital pin write test

  Open Serial Monitor at 115200 baud and send commands like:
    7 high
    7 low
    8 high
    8 low
*/

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint8_t MAX_MEGA_DIGITAL_PIN = 69; // D0-D53 plus A0-A15 as D54-D69.

char serialBuffer[32];
uint8_t serialIndex = 0;

void printHelp() {
  Serial.println();
  Serial.println("Mega digital pin write test");
  Serial.println("Commands:");
  Serial.println("  <pin> high");
  Serial.println("  <pin> low");
  Serial.println("Examples:");
  Serial.println("  7 high");
  Serial.println("  7 low");
  Serial.println("  8 high");
  Serial.println("  8 low");
  Serial.println();
}

void lowercase(char *text) {
  while (*text != '\0') {
    *text = (char)tolower((unsigned char)*text);
    ++text;
  }
}

void processCommand(char *line) {
  char *pinToken = strtok(line, " \t");
  char *stateToken = strtok(NULL, " \t");

  if (pinToken == NULL || stateToken == NULL) {
    printHelp();
    return;
  }

  char *endPtr = NULL;
  const long parsedPin = strtol(pinToken, &endPtr, 10);
  if (endPtr == pinToken || *endPtr != '\0' ||
      parsedPin < 0 || parsedPin > MAX_MEGA_DIGITAL_PIN) {
    Serial.println("Invalid pin. Use 0 through 69 on an Arduino Mega.");
    return;
  }

  lowercase(stateToken);

  int state = LOW;
  if (strcmp(stateToken, "high") == 0 || strcmp(stateToken, "1") == 0) {
    state = HIGH;
  } else if (strcmp(stateToken, "low") == 0 || strcmp(stateToken, "0") == 0) {
    state = LOW;
  } else {
    Serial.println("Invalid state. Use high, low, 1, or 0.");
    return;
  }

  const uint8_t pin = (uint8_t)parsedPin;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, state);

  Serial.print("Pin ");
  Serial.print(pin);
  Serial.print(" set ");
  Serial.println(state == HIGH ? "HIGH" : "LOW");
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(500);
  printHelp();
}

void loop() {
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
      Serial.println("Command too long.");
    }
  }
}
