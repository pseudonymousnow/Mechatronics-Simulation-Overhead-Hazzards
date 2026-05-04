#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

/*
  mega_ping_responder.cpp

  Minimal Mega responder for src_uno/uno_ping.cpp.

  Wiring:
  - XBee TX -> Mega RX2 (pin 17)
  - XBee RX -> Mega TX2 (pin 16)
  - XBee GND -> Mega GND

  Behavior:
  - listens for <PING,peerId,seq> on Serial2
  - replies with <PONG,peerId,seq,myId>
  - prints packet activity over USB Serial
*/

#define xbeeSerial Serial2

const uint32_t USB_SERIAL_BAUD_RATE = 115200;
const uint32_t XBEE_BAUD_RATE = 9600;
const size_t XBEE_FRAME_MAX = 64;

uint16_t myId = 0;
uint32_t pingReceivedCount = 0;
uint32_t pongSentCount = 0;
uint32_t invalidFrameCount = 0;

bool xbeeFrameInProgress = false;
char xbeeFrameBuffer[XBEE_FRAME_MAX] = {0};
size_t xbeeFrameLength = 0;

void sendPong(uint16_t peerId, uint32_t sequence);
void sendFramedPayload(const char *payload);
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

  Serial.println(F("mega_ping_responder"));
  Serial.print(F("My temporary ID: "));
  Serial.println((unsigned)myId);
  Serial.println(F("Listening on Serial2 (TX2=16, RX2=17)."));
}

void loop() {
  handleXbeeInput();
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

  if (strcmp(kind, "PING") != 0) {
    ++invalidFrameCount;
    return;
  }

  const uint16_t peerId = (uint16_t)strtoul(field1, NULL, 10);
  const uint32_t sequence = strtoul(field2, NULL, 10);

  if (peerId == myId) {
    return;
  }

  ++pingReceivedCount;
  Serial.print(F("RX PING from "));
  Serial.print((unsigned)peerId);
  Serial.print(F(" seq "));
  Serial.print((unsigned long)sequence);
  Serial.print(F(" | sent "));
  Serial.print((unsigned long)(pongSentCount + 1));
  Serial.println(F(" PONGs"));

  sendPong(peerId, sequence);
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
