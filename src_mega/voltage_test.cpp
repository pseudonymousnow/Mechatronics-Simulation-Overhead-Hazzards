#include <Arduino.h>
#include <Wire.h>

#include "EncoderMux.h"

/*
  voltage_test.cpp

  Purpose:
  - select mux channel 2 using the existing EncoderMux helper
  - talk to an Adafruit INA260 over raw I2C
  - print bus voltage, current, and power to the serial monitor

  Notes:
  - This sketch does not require the Adafruit INA260 library.
  - INA260 default I2C address is 0x40.
*/

#pragma region Configuration

const uint8_t INA260_MUX_CH = 2;
const uint8_t INA260_I2C_ADDR = 0x40;

const uint8_t INA260_REG_CONFIG = 0x00;
const uint8_t INA260_REG_CURRENT = 0x01;
const uint8_t INA260_REG_BUS_VOLTAGE = 0x02;
const uint8_t INA260_REG_POWER = 0x03;
const uint8_t INA260_REG_MANUFACTURER_ID = 0xFE;
const uint8_t INA260_REG_DIE_ID = 0xFF;

const uint32_t PRINT_INTERVAL_MS = 500;

#pragma endregion

#pragma region Globals

EncoderMuxConfig ina260MuxConfig = makeDefaultAs5600MuxConfig(INA260_MUX_CH);
uint32_t lastPrintMs = 0;

#pragma endregion

#pragma region Helpers

bool readIna260Register16(uint8_t reg, uint16_t &valueOut) {
  if (!selectMuxChannel(ina260MuxConfig)) {
    return false;
  }

  Wire.beginTransmission(INA260_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t requestedBytes = 2;
  const uint8_t receivedBytes = Wire.requestFrom((int)INA260_I2C_ADDR,
                                                 (int)requestedBytes);
  if (receivedBytes != requestedBytes) {
    return false;
  }

  const uint8_t msb = Wire.read();
  const uint8_t lsb = Wire.read();
  valueOut = ((uint16_t)msb << 8) | (uint16_t)lsb;
  return true;
}

bool readIna260SignedRegister16(uint8_t reg, int16_t &valueOut) {
  uint16_t rawValue = 0;
  if (!readIna260Register16(reg, rawValue)) {
    return false;
  }

  valueOut = (int16_t)rawValue;
  return true;
}

void printIna260Identity() {
  uint16_t manufacturerId = 0;
  uint16_t dieId = 0;

  const bool manufacturerReadOk =
      readIna260Register16(INA260_REG_MANUFACTURER_ID, manufacturerId);
  const bool dieIdReadOk =
      readIna260Register16(INA260_REG_DIE_ID, dieId);

  Serial.print("INA260 manufacturer ID: ");
  if (manufacturerReadOk) {
    Serial.println(manufacturerId, HEX);
  } else {
    Serial.println("read failed");
  }

  Serial.print("INA260 die ID: ");
  if (dieIdReadOk) {
    Serial.println(dieId, HEX);
  } else {
    Serial.println("read failed");
  }
}

void printIna260Measurements() {
  int16_t currentRaw = 0;
  uint16_t voltageRaw = 0;
  uint16_t powerRaw = 0;
  uint16_t configRaw = 0;

  const bool currentOk =
      readIna260SignedRegister16(INA260_REG_CURRENT, currentRaw);
  const bool voltageOk =
      readIna260Register16(INA260_REG_BUS_VOLTAGE, voltageRaw);
  const bool powerOk =
      readIna260Register16(INA260_REG_POWER, powerRaw);
  const bool configOk =
      readIna260Register16(INA260_REG_CONFIG, configRaw);

  if (!currentOk || !voltageOk || !powerOk) {
    Serial.print("INA260 read failed. current=");
    Serial.print(currentOk ? "ok" : "fail");
    Serial.print(" voltage=");
    Serial.print(voltageOk ? "ok" : "fail");
    Serial.print(" power=");
    Serial.println(powerOk ? "ok" : "fail");
    return;
  }

  const float currentMilliamps = (float)currentRaw * 1.25f;
  const float voltageMillivolts = (float)voltageRaw * 1.25f;
  const float powerMilliwatts = (float)powerRaw * 10.0f;

  Serial.print("Bus Voltage: ");
  Serial.print(voltageMillivolts / 1000.0f, 4);
  Serial.print(" V | Current: ");
  Serial.print(currentMilliamps, 2);
  Serial.print(" mA | Power: ");
  Serial.print(powerMilliwatts / 1000.0f, 4);
  Serial.print(" W");

  if (configOk) {
    Serial.print(" | Config: 0x");
    Serial.print(configRaw, HEX);
  }

  Serial.println();
}

#pragma endregion

#pragma region Setup_And_Loop

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("INA260 voltage test starting...");
  Serial.print("Using mux channel ");
  Serial.println(INA260_MUX_CH);
  Serial.print("INA260 I2C address: 0x");
  Serial.println(INA260_I2C_ADDR, HEX);

  printIna260Identity();
}

void loop() {
  const uint32_t nowMs = millis();
  if ((nowMs - lastPrintMs) < PRINT_INTERVAL_MS) {
    return;
  }

  lastPrintMs = nowMs;
  printIna260Measurements();
}

#pragma endregion
