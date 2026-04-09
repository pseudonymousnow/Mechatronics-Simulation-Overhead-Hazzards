#include <Arduino.h>

/*
  design_day_demo_ping_robot.cpp

  Simple XBee ping responder for the robot-side Mega.

  Wiring:
  - XBee on Serial2 (TX2=16, RX2=17)

  Behavior:
  - listens for <PING,n>
  - replies with <PONG,n>
  - prints USB status so you can confirm packets are arriving
*/

#define xbeeSerial Serial2

const uint32_t USB_SERIAL_BAUD_RATE = 115200;
const uint32_t XBEE_BAUD_RATE = 9600;
const uint32_t STATUS_PRINT_INTERVAL_MS = 1000;
const size_t XBEE_FRAME_MAX = 48;

uint32_t pingReceivedCount = 0;
uint32_t pongSentCount = 0;
uint32_t invalidFrameCount = 0;
uint32_t lastPingSequence = 0;
uint32_t lastPingReceivedMs = 0;
uint32_t lastStatusPrintMs = 0;

bool xbeeFrameInProgress = false;
char xbeeFrameBuffer[XBEE_FRAME_MAX] = {0};
size_t xbeeFrameLength = 0;

void sendFramedPayload(const char *payload);
void handleXbeeInput();
void handleXbeeChar(char incoming);
void handleXbeePayload(char *payload);
void printStatusLine();

void setup() {
  Serial.begin(USB_SERIAL_BAUD_RATE);
  xbeeSerial.begin(XBEE_BAUD_RATE);

  Serial.println("design_day_demo_ping_robot");
  Serial.println("Listening on Serial2 for <PING,n> and replying with <PONG,n>.");
}

void loop() {
  const uint32_t nowMs = millis();

  handleXbeeInput();

  if (lastStatusPrintMs == 0 ||
      (nowMs - lastStatusPrintMs) >= STATUS_PRINT_INTERVAL_MS) {
    printStatusLine();
    lastStatusPrintMs = nowMs;
  }
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
  if (strncmp(payload, "PING,", 5) != 0) {
    ++invalidFrameCount;
    return;
  }

  lastPingSequence = strtoul(payload + 5, NULL, 10);
  lastPingReceivedMs = millis();
  ++pingReceivedCount;

  char response[XBEE_FRAME_MAX];
  snprintf(response,
           sizeof(response),
           "PONG,%lu",
           (unsigned long)lastPingSequence);
  sendFramedPayload(response);
  ++pongSentCount;

  Serial.print("Received PING ");
  Serial.print((unsigned long)lastPingSequence);
  Serial.println(" -> replied");
}

void printStatusLine() {
  Serial.print("recv=");
  Serial.print((unsigned long)pingReceivedCount);
  Serial.print(" sent=");
  Serial.print((unsigned long)pongSentCount);
  Serial.print(" last_ping=");
  Serial.print((unsigned long)lastPingSequence);
  Serial.print(" invalid=");
  Serial.print((unsigned long)invalidFrameCount);
  Serial.print(" last_ping_ms=");
  Serial.println((unsigned long)lastPingReceivedMs);
}
