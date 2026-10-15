#include <Arduino.h>

// Detect contact pairs without assuming which wires are rows or columns.
// The confirmed TX2 terminal is GPIO17; GPIO1/TX0 remains the serial output.
static const uint8_t KEYPAD_PINS[] = {17, 5, 13, 14, 19, 21};
static const uint8_t PIN_COUNT = sizeof(KEYPAD_PINS) / sizeof(KEYPAD_PINS[0]);
static const uint32_t SETTLE_US = 200;
static const uint32_t DEBOUNCE_MS = 40;

static uint8_t driveIndex = 0;
static bool driveActive = false;
static uint32_t driveStartedUs = 0;
static uint16_t frameMask = 0;
static uint16_t candidateMask = 0;
static uint16_t stableMask = 0;
static uint32_t candidateSinceMs = 0;

// Each unordered pair gets one bit: six wires have fifteen possible pairs.
static uint16_t pairBit(uint8_t first, uint8_t second) {
  if (first > second) {
    const uint8_t tmp = first;
    first = second;
    second = tmp;
  }
  const uint8_t bit = first * (2 * PIN_COUNT - first - 1) / 2 +
    (second - first - 1);
  return static_cast<uint16_t>(1U << bit);
}

static void reportContacts(uint16_t contacts) {
  if (contacts == 0) {
    Serial.println("RELEASE");
    return;
  }
  if ((contacts & (contacts - 1)) != 0) {
    Serial.println("MULTIPLE CONTACTS: press only one key at a time.");
    return;
  }
  for (uint8_t first = 0; first < PIN_COUNT; ++first) {
    for (uint8_t second = first + 1; second < PIN_COUNT; ++second) {
      if (contacts == pairBit(first, second)) {
        Serial.printf("GPIO%d <-> GPIO%d\n", KEYPAD_PINS[first], KEYPAD_PINS[second]);
        return;
      }
    }
  }
}

static void updateContacts(uint16_t contacts, uint32_t nowMs) {
  if (contacts != candidateMask) {
    candidateMask = contacts;
    candidateSinceMs = nowMs;
  }
  if (contacts != stableMask &&
      static_cast<uint32_t>(nowMs - candidateSinceMs) >= DEBOUNCE_MS) {
    stableMask = contacts;
    reportContacts(stableMask);
  }
}

void setup() {
  // Release every pin first, including any outputs left by a warm restart.
  for (uint8_t index = 0; index < PIN_COUNT; ++index) {
    pinMode(KEYPAD_PINS[index], INPUT);
  }
  for (uint8_t index = 0; index < PIN_COUNT; ++index) {
    pinMode(KEYPAD_PINS[index], INPUT_PULLUP);
  }

  Serial.begin(115200);
  Serial.println("KEYPAD WIRING TEST - 115200 baud");
  Serial.println("Press one key at a time, then release it.");
  Serial.println("Record the physical key label and its GPIO pair.");
  Serial.printf("Pins: GPIO%d GPIO%d GPIO%d GPIO%d GPIO%d GPIO%d\n",
    KEYPAD_PINS[0], KEYPAD_PINS[1], KEYPAD_PINS[2],
    KEYPAD_PINS[3], KEYPAD_PINS[4], KEYPAD_PINS[5]);
  Serial.println("Only contact detection; no scale actions or NVS writes.");
  candidateSinceMs = millis();
}

void loop() {
  if (!driveActive) {
    // Preload LOW while still an input: never drive HIGH or two pins together.
    digitalWrite(KEYPAD_PINS[driveIndex], LOW);
    pinMode(KEYPAD_PINS[driveIndex], OUTPUT);
    driveStartedUs = micros();
    driveActive = true;
    return;
  }
  if (static_cast<uint32_t>(micros() - driveStartedUs) < SETTLE_US) {
    return;
  }

  for (uint8_t senseIndex = 0; senseIndex < PIN_COUNT; ++senseIndex) {
    if (senseIndex != driveIndex && digitalRead(KEYPAD_PINS[senseIndex]) == LOW) {
      frameMask |= pairBit(driveIndex, senseIndex);
    }
  }
  pinMode(KEYPAD_PINS[driveIndex], INPUT_PULLUP);
  driveActive = false;

  if (++driveIndex == PIN_COUNT) {
    driveIndex = 0;
    updateContacts(frameMask, millis());
    frameMask = 0;
  }
}
