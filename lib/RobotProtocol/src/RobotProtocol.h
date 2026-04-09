#pragma once

/*
  RobotProtocol.h

  Shared packet helpers for the design day controller/robot demo.

  The XBee transport uses framed packets:
    <CMD,...>
    <STS,...>

  This library keeps the payload format, parsing, and formatting in one place
  so both sketches stay focused on hardware behavior.
*/

#include <Arduino.h>

static const size_t ROBOT_PROTOCOL_MAX_PAYLOAD_LENGTH = 96;

enum DemoWinchCommand : uint8_t {
  DEMO_WINCH_COMMAND_NONE = 0,
  DEMO_WINCH_COMMAND_UP = 1,
  DEMO_WINCH_COMMAND_DOWN = 2
};

struct DemoCommandPacket {
  int16_t driveCommand;
  DemoWinchCommand winchCommand;
  bool emergencyStop;
  uint32_t sequence;
};

struct DemoStatusPacket {
  int16_t appliedDriveCommand;
  DemoWinchCommand activeWinchCommand;
  bool homeSwitchPressed;
  bool emergencyStopActive;
  bool communicationTimedOut;
  bool winchBusy;
  uint32_t lastCommandSequence;
};

bool isRobotProtocolWhitespace(char value);
void sanitizeRobotProtocolPayloadInPlace(char *text);
uint8_t splitRobotProtocolCsvInPlace(char *text,
                                     char *tokens[],
                                     uint8_t maxTokenCount);

bool formatDemoCommandPayload(const DemoCommandPacket &packet,
                              char *buffer,
                              size_t bufferSize);
bool parseDemoCommandPayload(char *payload, DemoCommandPacket &packet);

bool formatDemoStatusPayload(const DemoStatusPacket &packet,
                             char *buffer,
                             size_t bufferSize);
bool parseDemoStatusPayload(char *payload, DemoStatusPacket &packet);

const char *getDemoWinchCommandName(DemoWinchCommand command);
