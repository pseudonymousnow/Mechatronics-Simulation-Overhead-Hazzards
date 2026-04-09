#include <Arduino.h>

/*
  design_day_demo_controller.cpp

  Reads one potentiometer on A0 and two contact switches on D3 and D5, then
  prints one status line at a fixed interval.

  Wiring assumptions:
  - Potentiometer wiper -> A0
  - Contact switches use INPUT_PULLUP, so LOW means pressed
*/

const uint8_t POTENTIOMETER_PIN = A0;
const uint8_t CONTACT_SWITCH_PINS[] = {3, 5};
const uint8_t CONTACT_SWITCH_COUNT =
    sizeof(CONTACT_SWITCH_PINS) / sizeof(CONTACT_SWITCH_PINS[0]);
const int POT_CENTER_LOW_RAW = 502;
const int POT_CENTER_HIGH_RAW = 503;
const int POT_DEADZONE_COUNTS = 20;
const int DEFAULT_MAX_MOTOR_SPEED = 200;
const uint32_t STATUS_PRINT_INTERVAL_MS = 200;
const uint32_t CONTACT_SWITCH_DEBOUNCE_MS = 25;

enum ActiveInputSource : uint8_t {
  ACTIVE_INPUT_NONE = 0,
  ACTIVE_INPUT_POTENTIOMETER,
  ACTIVE_INPUT_SWITCH_D3,
  ACTIVE_INPUT_SWITCH_D5
};

uint32_t lastStatusPrintMs = 0;
bool debouncedContactSwitchPressed[CONTACT_SWITCH_COUNT] = {false};
bool lastRawContactSwitchPressed[CONTACT_SWITCH_COUNT] = {false};
uint32_t lastRawContactSwitchChangeMs[CONTACT_SWITCH_COUNT] = {0};
ActiveInputSource activeInputSource = ACTIVE_INPUT_NONE;
int currentPotValue = 0;
int currentRawMotorCommand = 0;

int mapPotentiometerToMotorCommand(int rawValue, int maxMotorSpeed);
bool readContactSwitchRawPressed(uint8_t switchIndex);
void initializeContactSwitchDebounce(uint32_t nowMs);
void updateDebouncedContactSwitches(uint32_t nowMs);
void updateActiveInputSource(int potentiometerCommand);
int getActiveMotorCommand(int potentiometerCommand);
bool isContactSwitchRegisteredPressed(uint8_t switchIndex);
const char *getActiveInputSourceName();
void printStatusLine();

void setup() {
  Serial.begin(115200);

  pinMode(POTENTIOMETER_PIN, INPUT);

  for (uint8_t i = 0; i < CONTACT_SWITCH_COUNT; ++i) {
    pinMode(CONTACT_SWITCH_PINS[i], INPUT_PULLUP);
  }
  initializeContactSwitchDebounce(millis());

  Serial.println("design_day_demo_controller");
  Serial.println("Potentiometer: A0");
  Serial.println("Switches: D3 and D5 using INPUT_PULLUP (LOW = pressed)");
  Serial.print("Debounce: ");
  Serial.print(CONTACT_SWITCH_DEBOUNCE_MS);
  Serial.println(" ms");
  Serial.println("Pot is mapped to a signed motor command centered at 502/503.");
}

void loop() {
  const uint32_t nowMs = millis();
  updateDebouncedContactSwitches(nowMs);

  currentPotValue = analogRead(POTENTIOMETER_PIN);
  currentRawMotorCommand =
      mapPotentiometerToMotorCommand(currentPotValue, DEFAULT_MAX_MOTOR_SPEED);
  updateActiveInputSource(currentRawMotorCommand);

  if (lastStatusPrintMs == 0 || (nowMs - lastStatusPrintMs) >= STATUS_PRINT_INTERVAL_MS) {
    printStatusLine();
    lastStatusPrintMs = nowMs;
  }
}

bool readContactSwitchRawPressed(uint8_t switchIndex) {
  return digitalRead(CONTACT_SWITCH_PINS[switchIndex]) == LOW;
}

void initializeContactSwitchDebounce(uint32_t nowMs) {
  for (uint8_t i = 0; i < CONTACT_SWITCH_COUNT; ++i) {
    const bool rawPressed = readContactSwitchRawPressed(i);
    debouncedContactSwitchPressed[i] = rawPressed;
    lastRawContactSwitchPressed[i] = rawPressed;
    lastRawContactSwitchChangeMs[i] = nowMs;
  }
}

void updateDebouncedContactSwitches(uint32_t nowMs) {
  for (uint8_t i = 0; i < CONTACT_SWITCH_COUNT; ++i) {
    const bool rawPressed = readContactSwitchRawPressed(i);

    if (rawPressed != lastRawContactSwitchPressed[i]) {
      lastRawContactSwitchPressed[i] = rawPressed;
      lastRawContactSwitchChangeMs[i] = nowMs;
    }

    if (debouncedContactSwitchPressed[i] != rawPressed &&
        (nowMs - lastRawContactSwitchChangeMs[i]) >= CONTACT_SWITCH_DEBOUNCE_MS) {
      debouncedContactSwitchPressed[i] = rawPressed;
    }
  }
}

void updateActiveInputSource(int potentiometerCommand) {
  if (activeInputSource == ACTIVE_INPUT_POTENTIOMETER) {
    if (potentiometerCommand == 0) {
      activeInputSource = ACTIVE_INPUT_NONE;
    }
    return;
  }

  if (activeInputSource == ACTIVE_INPUT_SWITCH_D3) {
    if (!debouncedContactSwitchPressed[0]) {
      activeInputSource = ACTIVE_INPUT_NONE;
    }
    return;
  }

  if (activeInputSource == ACTIVE_INPUT_SWITCH_D5) {
    if (!debouncedContactSwitchPressed[1]) {
      activeInputSource = ACTIVE_INPUT_NONE;
    }
    return;
  }

  if (potentiometerCommand != 0) {
    activeInputSource = ACTIVE_INPUT_POTENTIOMETER;
    return;
  }

  if (debouncedContactSwitchPressed[0]) {
    activeInputSource = ACTIVE_INPUT_SWITCH_D3;
    return;
  }

  if (debouncedContactSwitchPressed[1]) {
    activeInputSource = ACTIVE_INPUT_SWITCH_D5;
  }
}

int getActiveMotorCommand(int potentiometerCommand) {
  return activeInputSource == ACTIVE_INPUT_POTENTIOMETER ? potentiometerCommand : 0;
}

bool isContactSwitchRegisteredPressed(uint8_t switchIndex) {
  if (switchIndex == 0) {
    return activeInputSource == ACTIVE_INPUT_SWITCH_D3 &&
           debouncedContactSwitchPressed[switchIndex];
  }

  return activeInputSource == ACTIVE_INPUT_SWITCH_D5 &&
         debouncedContactSwitchPressed[switchIndex];
}

const char *getActiveInputSourceName() {
  if (activeInputSource == ACTIVE_INPUT_POTENTIOMETER) {
    return "POT";
  }

  if (activeInputSource == ACTIVE_INPUT_SWITCH_D3) {
    return "D3";
  }

  if (activeInputSource == ACTIVE_INPUT_SWITCH_D5) {
    return "D5";
  }

  return "NONE";
}

void printStatusLine() {
  const int activeMotorCommand = getActiveMotorCommand(currentRawMotorCommand);

  Serial.print("pot=");
  Serial.print(currentPotValue);
  Serial.print(" raw_cmd=");
  Serial.print(currentRawMotorCommand);
  Serial.print(" active=");
  Serial.print(getActiveInputSourceName());
  Serial.print(" cmd=");
  Serial.print(activeMotorCommand);

  for (uint8_t i = 0; i < CONTACT_SWITCH_COUNT; ++i) {
    const bool pressed = isContactSwitchRegisteredPressed(i);
    Serial.print(" sw(D");
    Serial.print(CONTACT_SWITCH_PINS[i]);
    Serial.print(")=");
    Serial.print(pressed ? "PRESSED" : "RELEASED");
  }

  Serial.println();
}

int mapPotentiometerToMotorCommand(int rawValue, int maxMotorSpeed) {
  const int deadzoneLow = POT_CENTER_LOW_RAW - POT_DEADZONE_COUNTS;
  const int deadzoneHigh = POT_CENTER_HIGH_RAW + POT_DEADZONE_COUNTS;

  if (rawValue >= deadzoneLow && rawValue <= deadzoneHigh) {
    return 0;
  }

  if (rawValue < deadzoneLow) {
    return (int)map(rawValue, 0, deadzoneLow - 1, -maxMotorSpeed, -1);
  }

  return (int)map(rawValue, deadzoneHigh + 1, 1023, 1, maxMotorSpeed);
}
