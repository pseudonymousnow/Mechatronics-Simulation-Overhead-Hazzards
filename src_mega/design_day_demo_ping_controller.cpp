#include <Arduino.h>

/*
  design_day_demo_ping_controller.cpp

  Simple XBee ping test for the controller-side Mega.

  Wiring:
  - XBee on Serial2 (TX2=16, RX2=17)

  Behavior:
  - sends <PING,n> once per second
  - prints any valid <PONG,n> reply to USB Serial
  - 'p' over USB sends an immediate ping
  - '?' over USB prints help
*/

#define xbeeSerial Serial2

const uint32_t USB_SERIAL_BAUD_RATE = 115200;
const uint32_t XBEE_BAUD_RATE = 9600;
const uint32_t AUTO_PING_INTERVAL_MS = 1000;
const uint32_t STATUS_PRINT_INTERVAL_MS = 1000;
const size_t XBEE_FRAME_MAX = 48;

uint32_t nextPingSequence = 1;
uint32_t pingSentCount = 0;
uint32_t pongReceivedCount = 0;
uint32_t invalidFrameCount = 0;
uint32_t lastPingSentMs = 0;
uint32_t lastPongReceivedMs = 0;
uint32_t lastStatusPrintMs = 0;
uint32_t lastPongSequence = 0;

bool xbeeFrameInProgress = false;
char xbeeFrameBuffer[XBEE_FRAME_MAX] = {0};
size_t xbeeFrameLength = 0;

void printHelp();
void sendPing();
void sendFramedPayload(const char *payload);
void handleUsbInput();
void handleXbeeInput();
void handleXbeeChar(char incoming);
void handleXbeePayload(char *payload);
void printStatusLine();

void setup() {
  Serial.begin(USB_SERIAL_BAUD_RATE);
  xbeeSerial.begin(XBEE_BAUD_RATE);

  Serial.println("design_day_demo_ping_controller");
  Serial.println("Using Serial2 for XBee ping test.");
  printHelp();
}

void loop() {
  const uint32_t nowMs = millis();

  handleUsbInput();
  handleXbeeInput();

  if (lastPingSentMs == 0 ||
      (nowMs - lastPingSentMs) >= AUTO_PING_INTERVAL_MS) {
    sendPing();
  }

  if (lastStatusPrintMs == 0 ||
      (nowMs - lastStatusPrintMs) >= STATUS_PRINT_INTERVAL_MS) {
    printStatusLine();
    lastStatusPrintMs = nowMs;
  }
}

void printHelp() {
  Serial.println("USB commands:");
  Serial.println("  p = send one ping immediately");
  Serial.println("  ? = print this help");
}

void sendPing() {
  char payload[XBEE_FRAME_MAX];
  snprintf(payload,
           sizeof(payload),
           "PING,%lu",
           (unsigned long)nextPingSequence);

  sendFramedPayload(payload);
  lastPingSentMs = millis();
  ++pingSentCount;
  ++nextPingSequence;
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
  if (strncmp(payload, "PONG,", 5) != 0) {
    ++invalidFrameCount;
    return;
  }

  lastPongSequence = strtoul(payload + 5, NULL, 10);
  lastPongReceivedMs = millis();
  ++pongReceivedCount;

  Serial.print("Received PONG ");
  Serial.println((unsigned long)lastPongSequence);
}

void printStatusLine() {
  Serial.print("sent=");
  Serial.print((unsigned long)pingSentCount);
  Serial.print(" recv=");
  Serial.print((unsigned long)pongReceivedCount);
  Serial.print(" last_pong=");
  Serial.print((unsigned long)lastPongSequence);
  Serial.print(" invalid=");
  Serial.print((unsigned long)invalidFrameCount);
  Serial.print(" last_pong_ms=");
  Serial.println((unsigned long)lastPongReceivedMs);
}
