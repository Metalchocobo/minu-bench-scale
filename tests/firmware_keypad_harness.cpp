// Compile this harness together with the production keypad.cpp, using tests/stubs.
#include "../firmware/esp32_hx711_serial/keypad.h"

#include <cstdlib>
#include <initializer_list>
#include <iostream>
#include <vector>

static uint32_t nowMs = 100;
static int levels[40] = {};
static int modes[40] = {};
// Independent physical layout measured with the contact-pair diagnostic sketch.
// These values must not be derived from the production configuration under test.
static const int physicalRows[4] = {13, 14, 19, 21};
static const int physicalColumns[2] = {17, 5};
static bool contacts[4][2] = {};
static std::vector<KeyCode> events;
static const char* scenario = "initialization";
static unsigned int scenarios = 0;

static void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL [" << scenario << "] " << message << " at " << nowMs << " ms\n";
    std::exit(1);
  }
}

void pinMode(int pin, int mode) {
  const bool row = pin == 13 || pin == 14 || pin == 19 || pin == 21;
  const bool column = pin == 17 || pin == 5;
  require((row && mode == OUTPUT) || (column && mode == INPUT_PULLUP),
          "GPIO role must match the measured physical matrix");
  modes[pin] = mode;
}
void digitalWrite(int pin, int value) {
  require(pin >= 0 && pin < 40 && modes[pin] == OUTPUT,
          "scanner must only drive configured row outputs");
  levels[pin] = value;
}
void delayMicroseconds(unsigned int) {}
unsigned long millis() { return nowMs; }

int digitalRead(int pin) {
  // Real electrical row/column layout; a pressed switch pulls a column low
  // only while the scanner drives its row low.
  const int column = pin == 17 ? 0 : (pin == 5 ? 1 : -1);
  require(column >= 0, "unexpected input pin");
  require(modes[pin] == INPUT_PULLUP, "scanner must sense a pulled-up column");
  for (int row = 0; row < 4; ++row) {
    if (levels[physicalRows[row]] == LOW && contacts[row][column]) return LOW;
  }
  return HIGH;
}

static void setKeys(std::initializer_list<KeyCode> keys) {
  for (auto& row : contacts) for (bool& pressed : row) pressed = false;
  for (KeyCode key : keys) {
    switch (key) {
      case KEY_SKIP: contacts[0][0] = true; break;
      case KEY_SLEEP: contacts[0][1] = true; break;
      case KEY_ENTER: contacts[1][0] = true; break;
      case KEY_CLEAR: contacts[1][1] = true; break;
      case KEY_WIFI: contacts[2][0] = true; break;
      case KEY_TOTAL: contacts[2][1] = true; break;
      case KEY_MODE: contacts[3][0] = true; break;
      case KEY_TARE: contacts[3][1] = true; break;
      default: require(false, "only physical keys may close matrix contacts");
    }
  }
}

static void advance(uint32_t duration) {
  for (uint32_t elapsed = 0; elapsed < duration; ++elapsed) {
    ++nowMs;
    keypad_update(nowMs);
    const KeyCode event = keypad_get_event();
    if (event != KEY_NONE) events.push_back(event);
    require(keypad_get_event() == KEY_NONE, "an event must be consumed once");
  }
}

static void expectEvents(std::initializer_list<KeyCode> expected) {
  require(events == std::vector<KeyCode>(expected), "unexpected event sequence");
  events.clear();
}

static void reset(const char* name) {
  scenario = name;
  ++scenarios;
  nowMs = 100;
  setKeys({});
  events.clear();
  keypad_init();
}

static void testExportedPins() {
  reset("exported sleep/wake GPIOs and initialization");
  const int* rows = nullptr;
  const int* columns = nullptr;
  int rowCount = 0;
  int columnCount = 0;
  keypad_get_pins(&rows, &rowCount, &columns, &columnCount);
  require(rows != nullptr && columns != nullptr, "pin arrays must be exported for sleep");
  require(rowCount == 4 && columnCount == 2, "sleep must use four rows and two columns");
  for (int index = 0; index < rowCount; ++index) {
    require(rows[index] == physicalRows[index], "exported row differs from measured wiring");
    require(modes[rows[index]] == OUTPUT && levels[rows[index]] == HIGH,
            "initialization must leave every row output inactive");
  }
  for (int index = 0; index < columnCount; ++index) {
    require(columns[index] == physicalColumns[index], "exported wake column differs from measured wiring");
    require(modes[columns[index]] == INPUT_PULLUP, "wake columns must use pull-ups");
  }
}

static void testSingleRelease(KeyCode key) {
  reset(key == KEY_WIFI ? "single WIFI" : "single TOTAL");
  setKeys({key});
  advance(20);
  require(!keypad_is_pressed(key), "press debounce must reject a short pulse");
  setKeys({});
  advance(45);
  expectEvents({});

  setKeys({key});
  advance(45);
  require(keypad_is_pressed(key), "single key must become held");
  expectEvents({});
  advance(1000);
  expectEvents({});
  setKeys({});
  advance(20);
  expectEvents({});
  setKeys({key});
  advance(45);
  expectEvents({});
  setKeys({});
  advance(45);
  require(!keypad_is_pressed(key), "release must clear held state");
  expectEvents({key});
  advance(100);
  expectEvents({});

  setKeys({key});
  advance(45);
  setKeys({});
  advance(45);
  expectEvents({key});
}

static void testCombo(KeyCode first, KeyCode releaseFirst) {
  reset("combo order and release fencing");
  if (first != KEY_NONE) {
    setKeys({first});
    advance(45);
    expectEvents({});
  }
  setKeys({KEY_WIFI, KEY_TOTAL});
  advance(45);
  require(keypad_is_pressed(KEY_BATTERY), "both keys must activate battery diagnostics");
  require(!keypad_is_pressed(KEY_WIFI) && !keypad_is_pressed(KEY_TOTAL),
          "chord must not report its component keys as active");
  expectEvents({KEY_BATTERY});
  advance(300);
  expectEvents({});

  const KeyCode remaining = releaseFirst == KEY_WIFI ? KEY_TOTAL : KEY_WIFI;
  setKeys({remaining});
  advance(20);
  setKeys({KEY_WIFI, KEY_TOTAL});
  advance(45);
  require(keypad_is_pressed(KEY_BATTERY), "release bounce must preserve a held chord");
  expectEvents({});

  setKeys({remaining});
  advance(45);
  require(!keypad_is_pressed(KEY_BATTERY), "partial release must close diagnostics");
  expectEvents({});
  setKeys({KEY_WIFI, KEY_TOTAL});
  advance(45);
  require(!keypad_is_pressed(KEY_BATTERY), "repress before full release must stay consumed");
  expectEvents({});

  setKeys({});
  advance(20);
  setKeys({remaining});
  advance(45);
  expectEvents({});
  setKeys({KEY_WIFI, KEY_TOTAL});
  advance(45);
  require(!keypad_is_pressed(KEY_BATTERY), "full-release bounce must not rearm the chord");
  expectEvents({});
  setKeys({});
  advance(45);
  setKeys({KEY_WIFI, KEY_TOTAL});
  advance(45);
  require(keypad_is_pressed(KEY_BATTERY), "full release must allow a new chord");
  expectEvents({KEY_BATTERY});
  setKeys({});
  advance(45);
  expectEvents({});
}

static void testWake(bool bothHeld) {
  reset(bothHeld ? "wake with combo" : "wake WIFI then add TOTAL");
  if (bothHeld) setKeys({KEY_WIFI, KEY_TOTAL});
  else setKeys({KEY_WIFI});
  keypad_suppress_wake_key();
  advance(45);
  setKeys({KEY_WIFI, KEY_TOTAL});
  advance(45);
  require(!keypad_is_pressed(KEY_BATTERY), "wake gesture must not activate diagnostics");
  expectEvents({});
  setKeys({KEY_TOTAL});
  advance(45);
  expectEvents({});
  setKeys({});
  advance(20);
  setKeys({KEY_WIFI});
  advance(45);
  expectEvents({});
  setKeys({});
  advance(45);
  setKeys({KEY_WIFI, KEY_TOTAL});
  advance(45);
  expectEvents({KEY_BATTERY});
}

static void testSingleWake(KeyCode key) {
  reset("single wake key suppressed through release bounce");
  setKeys({key});
  keypad_suppress_wake_key();
  advance(200);
  require(!keypad_is_pressed(key), "held wake key must not become an application press");
  expectEvents({});
  setKeys({});
  advance(20);
  setKeys({key});
  advance(45);
  expectEvents({});
  setKeys({});
  advance(45);
  expectEvents({});

  setKeys({key});
  advance(45);
  require(keypad_is_pressed(key), "fresh press after wake release must be recognized");
  if (key == KEY_WIFI || key == KEY_TOTAL) expectEvents({});
  else expectEvents({key});
  setKeys({});
  advance(45);
  if (key == KEY_WIFI || key == KEY_TOTAL) expectEvents({key});
  else expectEvents({});
}

static void testOrdinaryRowPair(KeyCode first, KeyCode second) {
  reset("ordinary row pair must not become the WIFI/TOTAL chord");
  setKeys({first, second});
  advance(45);
  require(!keypad_is_pressed(KEY_BATTERY), "only physical WIFI/TOTAL may activate diagnostics");
  expectEvents({first});
  setKeys({});
  advance(45);
  expectEvents({});
}

static void testStuckCombo() {
  reset("stuck combo");
  setKeys({KEY_WIFI, KEY_TOTAL});
  advance(45);
  expectEvents({KEY_BATTERY});
  advance(15010);
  require(!keypad_is_pressed(KEY_BATTERY), "stuck chord must stop being held");
  expectEvents({});
  setKeys({KEY_TOTAL});
  advance(45);
  setKeys({KEY_WIFI, KEY_TOTAL});
  advance(45);
  expectEvents({});
  setKeys({});
  advance(45);
  setKeys({KEY_WIFI});
  advance(45);
  setKeys({});
  advance(45);
  expectEvents({KEY_WIFI});
}

static void testStuckSingle(KeyCode key) {
  reset("stuck release-action key");
  setKeys({key});
  advance(15060);
  require(!keypad_is_pressed(key), "stuck single key must stop being held");
  expectEvents({});
  setKeys({KEY_WIFI, KEY_TOTAL});
  advance(45);
  expectEvents({});
  setKeys({});
  advance(45);
  setKeys({key});
  advance(45);
  setKeys({});
  advance(45);
  expectEvents({key});
}

static void testOrdinaryKey(KeyCode key) {
  reset("ordinary press-action key");
  setKeys({key});
  advance(20);
  expectEvents({});
  advance(25);
  require(keypad_is_pressed(key), "ordinary key must become held");
  expectEvents({key});
  advance(15010);
  require(!keypad_is_pressed(key), "ordinary stuck key must be suppressed");
  expectEvents({});
  setKeys({});
  advance(45);
  setKeys({key});
  advance(45);
  expectEvents({key});
  setKeys({});
  advance(45);
  expectEvents({});
}

static void testOtherKeyDuringSuppression() {
  reset("other key while TARE stuck");
  setKeys({KEY_TARE});
  advance(45);
  expectEvents({KEY_TARE});
  advance(15010);
  expectEvents({});
  setKeys({KEY_TARE, KEY_ENTER});
  advance(45);
  expectEvents({KEY_ENTER});
}

int main() {
  testExportedPins();
  testSingleRelease(KEY_WIFI);
  testSingleRelease(KEY_TOTAL);
  for (KeyCode first : {KEY_NONE, KEY_WIFI, KEY_TOTAL}) {
    for (KeyCode releaseFirst : {KEY_WIFI, KEY_TOTAL}) testCombo(first, releaseFirst);
  }
  testWake(false);
  testWake(true);
  // WIFI is already covered by testWake(false); cover each other physical key.
  for (KeyCode key : {KEY_TARE, KEY_ENTER, KEY_TOTAL, KEY_SLEEP, KEY_SKIP, KEY_CLEAR, KEY_MODE}) {
    testSingleWake(key);
  }
  testOrdinaryRowPair(KEY_SKIP, KEY_SLEEP);
  testOrdinaryRowPair(KEY_ENTER, KEY_CLEAR);
  testOrdinaryRowPair(KEY_MODE, KEY_TARE);
  testStuckCombo();
  testStuckSingle(KEY_WIFI);
  testStuckSingle(KEY_TOTAL);
  for (KeyCode key : {KEY_TARE, KEY_ENTER, KEY_SLEEP, KEY_SKIP, KEY_CLEAR, KEY_MODE}) {
    testOrdinaryKey(key);
  }
  testOtherKeyDuringSuppression();
  std::cout << "PASS: production keypad matrix/chord harness (" << scenarios << " scenarios)\n";
  return 0;
}
