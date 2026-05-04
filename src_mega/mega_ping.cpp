#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

/*
  mega_ping.cpp

  Mega companion to src_uno/uno_ping.cpp.

  Wiring:
  - XBee TX -> Mega RX2 (pin 17)
  - XBee RX -> Mega TX2 (pin 16)
  - XBee GND -> Mega GND

  Behavior:
  - sends <PING,myId,seq> about once per second on Serial2
  - prints every received PING
  - replies with <PONG,peerId,seq,myId>
  - prints every received PONG addressed to this Mega
*/

#define xbeeSerial Serial2

const uint32_t USB_SERIAL_BAUD_RATE = 115200;
const uint32_t XBEE_BAUD_RATE = 9600;
const uint32_t PING_INTERVAL_MS = 1000;
const uint16_t PING_JITTER_MS = 400;
const size_t XBEE_FRAME_MAX = 64;

uint16_t myId = 0;
uint32_t nextPingSequence = 1;
uint32_t pingSentCount = 0;
uint32_t pingReceivedCount = 0;
uint32_t pongSentCount = 0;
uint32_t pongReceivedCount = 0;
uint32_t invalidFrameCount = 0;
uint32_t nextPingDueMs = 0;

bool xbeeFrameInProgress = false;
char xbeeFrameBuffer[XBEE_FRAME_MAX] = {0};
size_t xbeeFrameLength = 0;

void printHelp();
void scheduleNextPing();
void sendPing();
void sendPong(uint16_t peerId, uint32_t sequence);
void sendFramedPayload(const char *payload);
void handleUsbInput();
void handleXbeeInput();
void handleXbeeChar(char incoming);
void handleXbeePayload(char *payload);
bool splitThreeFields(char *payload,
                      char **field0,
                      char **field1,
                      char **field2);

void setup() {
  Serial.begin(USB_SERIAL_BAUD_RATE);
  xbeeSerial.begin(XBEE_BAUD_RATE);

  randomSeed(analogRead(A0) ^ micros());
  myId = (uint16_t)random(1000, 9999);

  Serial.println(F("mega_ping"));
  Serial.print(F("USB baud: "));
  Serial.println((unsigned long)USB_SERIAL_BAUD_RATE);
  Serial.print(F("XBee baud: "));
  Serial.println((unsigned long)XBEE_BAUD_RATE);
  Serial.print(F("My temporary ID: "));
  Serial.println((unsigned)myId);
  Serial.println(F("XBee port: Serial2 (TX2=16, RX2=17)"));
  printHelp();

  scheduleNextPing();
}

void loop() {
  handleUsbInput();
  handleXbeeInput();

  const uint32_t nowMs = millis();
  if ((int32_t)(nowMs - nextPingDueMs) >= 0) {
    sendPing();
    scheduleNextPing();
  }
}

void printHelp() {
  Serial.println(F("USB commands:"));
  Serial.println(F("  p = send one ping now"));
  Serial.println(F("  ? = print this help"));
}

void scheduleNextPing() {
  nextPingDueMs = millis() + PING_INTERVAL_MS + (uint32_t)random(PING_JITTER_MS);
}

void sendPing() {
  char payload[XBEE_FRAME_MAX];
  snprintf(payload,
           sizeof(payload),
           "PING,%u,%lu",
           (unsigned)myId,
           (unsigned long)nextPingSequence);

  sendFramedPayload(payload);
  Serial.print(F("TX PING seq "));
  Serial.println((unsigned long)nextPingSequence);

  ++nextPingSequence;
  ++pingSentCount;
}

void sendPong(uint16_t peerId, uint32_t sequence) {
  char payload[XBEE_FRAME_MAX];
  snprintf(payload,
           sizeof(payload),
           "PONG,%u,%lu,%u",
           (unsigned)peerId,
           (unsigned long)sequence,
           (unsigned)myId);

  sendFramedPayload(payload);
  ++pongSentCount;
}

void sendFramedPayload(const char *payload) {
  xbeeSerial.print('<');
  xbeeSerial.print(payload);
  xbeeSerial.print('>');
  xbeeSerial.print('\n');
}

void handleUsbInput() {
  while (Serial.available() > 0) {
    const char incoming = (char)Serial.read();
    if (incoming == '\r' || incoming == '\n') {
      continue;
    }

    if (incoming == 'p' || incoming == 'P') {
      sendPing();
      scheduleNextPing();
    } else if (incoming == '?') {
      printHelp();
    }
  }
}

void handleXbeeInput() {
  while (xbeeSerial.available() > 0) {
    handleXbeeChar((char)xbeeSerial.read());
  }
}

void handleXbeeChar(char incoming) {
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
    handleXbeePayload(xbeeFrameBuffer);
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
  ++invalidFrameCount;
}

void handleXbeePayload(char *payload) {
  char *kind = NULL;
  char *field1 = NULL;
  char *field2 = NULL;

  if (!splitThreeFields(payload, &kind, &field1, &field2)) {
    ++invalidFrameCount;
    return;
  }

  if (strcmp(kind, "PING") == 0) {
    const uint16_t peerId = (uint16_t)strtoul(field1, NULL, 10);
    const uint32_t sequence = strtoul(field2, NULL, 10);

    if (peerId == myId) {
      return;
    }

    ++pingReceivedCount;
    Serial.print(F("RX PING from "));
    Serial.print((unsigned)peerId);
    Serial.print(F(" seq "));
    Serial.println((unsigned long)sequence);

    sendPong(peerId, sequence);
    return;
  }

  if (strcmp(kind, "PONG") == 0) {
    const uint16_t addressedId = (uint16_t)strtoul(field1, NULL, 10);
    const uint32_t sequence = strtoul(field2, NULL, 10);
    char *responderField = strchr(field2, ',');

    if (addressedId != myId) {
      return;
    }

    ++pongReceivedCount;
    Serial.print(F("RX PONG seq "));
    Serial.print((unsigned long)sequence);
    if (responderField != NULL) {
      Serial.print(F(" from "));
      Serial.print(responderField + 1);
    }
    Serial.println();
    return;
  }

  ++invalidFrameCount;
}

bool splitThreeFields(char *payload,
                      char **field0,
                      char **field1,
                      char **field2) {
  char *firstComma = strchr(payload, ',');
  if (firstComma == NULL) {
    return false;
  }

  *firstComma = '\0';
  char *secondComma = strchr(firstComma + 1, ',');
  if (secondComma == NULL) {
    return false;
  }

  *secondComma = '\0';
  *field0 = payload;
  *field1 = firstComma + 1;
  *field2 = secondComma + 1;
  return (**field0 != '\0' && **field1 != '\0' && **field2 != '\0');
}
