#include "RobotProtocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace {

const char *const kDemoCommandPacketType = "CMD";
const char *const kDemoStatusPacketType = "STS";

bool parseDemoWinchCommandToken(const char *token, DemoWinchCommand &command) {
  if (token == NULL || token[0] == '\0') {
    return false;
  }

  const int parsedValue = atoi(token);
  if (parsedValue < (int)DEMO_WINCH_COMMAND_NONE ||
      parsedValue > (int)DEMO_WINCH_COMMAND_DOWN) {
    return false;
  }

  command = (DemoWinchCommand)parsedValue;
  return true;
}

bool parseBoolToken(const char *token, bool &value) {
  if (token == NULL || token[0] == '\0') {
    return false;
  }

  const int parsedValue = atoi(token);
  value = (parsedValue != 0);
  return true;
}

}  // namespace

bool isRobotProtocolWhitespace(char value) {
  return value == ' ' || value == '\t' || value == '\r' || value == '\n';
}

void sanitizeRobotProtocolPayloadInPlace(char *text) {
  if (text == NULL) {
    return;
  }

  size_t writeIndex = 0;
  for (size_t readIndex = 0; text[readIndex] != '\0'; ++readIndex) {
    const char value = text[readIndex];
    if (value == '<' || value == '>' || value == '\r' || value == '\n') {
      continue;
    }
    text[writeIndex++] = value;
  }
  text[writeIndex] = '\0';

  size_t startIndex = 0;
  while (text[startIndex] != '\0' &&
         isRobotProtocolWhitespace(text[startIndex])) {
    ++startIndex;
  }

  if (startIndex > 0) {
    size_t compactIndex = 0;
    while (text[startIndex] != '\0') {
      text[compactIndex++] = text[startIndex++];
    }
    text[compactIndex] = '\0';
  }

  size_t length = strlen(text);
  while (length > 0 && isRobotProtocolWhitespace(text[length - 1])) {
    text[--length] = '\0';
  }
}

uint8_t splitRobotProtocolCsvInPlace(char *text,
                                     char *tokens[],
                                     uint8_t maxTokenCount) {
  if (text == NULL || tokens == NULL || maxTokenCount == 0) {
    return 0;
  }

  sanitizeRobotProtocolPayloadInPlace(text);
  if (text[0] == '\0') {
    return 0;
  }

  uint8_t tokenCount = 0;
  tokens[tokenCount++] = text;

  for (char *cursor = text; *cursor != '\0' && tokenCount < maxTokenCount;
       ++cursor) {
    if (*cursor == ',') {
      *cursor = '\0';
      sanitizeRobotProtocolPayloadInPlace(tokens[tokenCount - 1]);
      tokens[tokenCount++] = cursor + 1;
    }
  }

  sanitizeRobotProtocolPayloadInPlace(tokens[tokenCount - 1]);
  return tokenCount;
}

bool formatDemoCommandPayload(const DemoCommandPacket &packet,
                              char *buffer,
                              size_t bufferSize) {
  if (buffer == NULL || bufferSize == 0) {
    return false;
  }

  const int written = snprintf(buffer,
                               bufferSize,
                               "%s,%d,%u,%u,%lu",
                               kDemoCommandPacketType,
                               (int)packet.driveCommand,
                               (unsigned int)packet.winchCommand,
                               packet.emergencyStop ? 1U : 0U,
                               (unsigned long)packet.sequence);
  return written > 0 && (size_t)written < bufferSize;
}

bool parseDemoCommandPayload(char *payload, DemoCommandPacket &packet) {
  char *tokens[5] = {NULL};
  const uint8_t tokenCount = splitRobotProtocolCsvInPlace(payload,
                                                          tokens,
                                                          (uint8_t)(sizeof(tokens) /
                                                                    sizeof(tokens[0])));
  if (tokenCount != 5 || strcmp(tokens[0], kDemoCommandPacketType) != 0) {
    return false;
  }

  DemoWinchCommand parsedWinchCommand = DEMO_WINCH_COMMAND_NONE;
  bool parsedEmergencyStop = false;

  if (!parseDemoWinchCommandToken(tokens[2], parsedWinchCommand) ||
      !parseBoolToken(tokens[3], parsedEmergencyStop)) {
    return false;
  }

  packet.driveCommand = (int16_t)atoi(tokens[1]);
  packet.winchCommand = parsedWinchCommand;
  packet.emergencyStop = parsedEmergencyStop;
  packet.sequence = strtoul(tokens[4], NULL, 10);
  return true;
}

bool formatDemoStatusPayload(const DemoStatusPacket &packet,
                             char *buffer,
                             size_t bufferSize) {
  if (buffer == NULL || bufferSize == 0) {
    return false;
  }

  const int written = snprintf(buffer,
                               bufferSize,
                               "%s,%d,%u,%u,%u,%u,%u,%lu",
                               kDemoStatusPacketType,
                               (int)packet.appliedDriveCommand,
                               (unsigned int)packet.activeWinchCommand,
                               packet.homeSwitchPressed ? 1U : 0U,
                               packet.emergencyStopActive ? 1U : 0U,
                               packet.communicationTimedOut ? 1U : 0U,
                               packet.winchBusy ? 1U : 0U,
                               (unsigned long)packet.lastCommandSequence);
  return written > 0 && (size_t)written < bufferSize;
}

bool parseDemoStatusPayload(char *payload, DemoStatusPacket &packet) {
  char *tokens[8] = {NULL};
  const uint8_t tokenCount = splitRobotProtocolCsvInPlace(payload,
                                                          tokens,
                                                          (uint8_t)(sizeof(tokens) /
                                                                    sizeof(tokens[0])));
  if (tokenCount != 8 || strcmp(tokens[0], kDemoStatusPacketType) != 0) {
    return false;
  }

  DemoWinchCommand parsedWinchCommand = DEMO_WINCH_COMMAND_NONE;
  bool parsedHomeSwitchPressed = false;
  bool parsedEmergencyStopActive = false;
  bool parsedCommunicationTimedOut = false;
  bool parsedWinchBusy = false;

  if (!parseDemoWinchCommandToken(tokens[2], parsedWinchCommand) ||
      !parseBoolToken(tokens[3], parsedHomeSwitchPressed) ||
      !parseBoolToken(tokens[4], parsedEmergencyStopActive) ||
      !parseBoolToken(tokens[5], parsedCommunicationTimedOut) ||
      !parseBoolToken(tokens[6], parsedWinchBusy)) {
    return false;
  }

  packet.appliedDriveCommand = (int16_t)atoi(tokens[1]);
  packet.activeWinchCommand = parsedWinchCommand;
  packet.homeSwitchPressed = parsedHomeSwitchPressed;
  packet.emergencyStopActive = parsedEmergencyStopActive;
  packet.communicationTimedOut = parsedCommunicationTimedOut;
  packet.winchBusy = parsedWinchBusy;
  packet.lastCommandSequence = strtoul(tokens[7], NULL, 10);
  return true;
}

const char *getDemoWinchCommandName(DemoWinchCommand command) {
  switch (command) {
    case DEMO_WINCH_COMMAND_NONE:
      return "STOP";
    case DEMO_WINCH_COMMAND_UP:
      return "UP";
    case DEMO_WINCH_COMMAND_DOWN:
      return "DOWN";
    default:
      return "?";
  }
}
