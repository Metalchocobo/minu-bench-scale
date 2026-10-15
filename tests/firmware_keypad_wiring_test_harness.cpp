// Compile this harness using tests/keypad_wiring_stubs; it executes the real sketch.
#include <Arduino.h>

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

static uint64_t nowUs = 100000;
static int modes[64] = {};
static int outputLevels[64] = {};
static bool electricalContacts[64][64] = {};
static std::vector<std::string> serialLines;
static std::string serialPending;
static const char* scenario = "initialization";
static unsigned int scenarios = 0;
static unsigned long gpioTransitions = 0;
MockSerial Serial;

static void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL [" << scenario << "] " << message << " at " << nowUs << " us\n";
    for (const auto& line : serialLines) std::cerr << "  serial: " << line << '\n';
    std::exit(1);
  }
}

#include "../firmware/keypad_wiring_test/keypad_wiring_test.ino"

static bool isKeypadPin(int pin) {
  for (uint8_t index = 0; index < PIN_COUNT; ++index) {
    if (KEYPAD_PINS[index] == pin) return true;
  }
  return false;
}

static void checkElectricalSafety() {
  unsigned int driven = 0;
  for (int pin = 0; pin < 64; ++pin) {
    if (modes[pin] == OUTPUT) {
      ++driven;
      require(isKeypadPin(pin), "scanner drove a pin outside its configured list");
      require(outputLevels[pin] == LOW, "an output must always drive LOW");
    }
  }
  require(driven <= 1, "scanner must never configure two outputs together");
}

void pinMode(int pin, int mode) {
  require(pin >= 0 && pin < 64 && isKeypadPin(pin), "unexpected pinMode pin");
  require(mode == INPUT || mode == INPUT_PULLUP || mode == OUTPUT, "unexpected GPIO mode");
  modes[pin] = mode;
  ++gpioTransitions;
  checkElectricalSafety();
}

void digitalWrite(int pin, int value) {
  require(pin >= 0 && pin < 64 && isKeypadPin(pin), "unexpected digitalWrite pin");
  require(value == LOW, "scanner must never request HIGH, even before OUTPUT");
  outputLevels[pin] = value;
  ++gpioTransitions;
  checkElectricalSafety();
}

int digitalRead(int pin) {
  require(pin >= 0 && pin < 64 && isKeypadPin(pin), "unexpected digitalRead pin");
  require(modes[pin] == INPUT_PULLUP, "scanner must sense a pulled-up input");
  checkElectricalSafety();

  // Closed switches are an electrical graph. Two held keys can also create
  // indirect contact paths; model those paths rather than only direct edges.
  bool visited[64] = {};
  int pending[64] = {};
  unsigned int head = 0;
  unsigned int tail = 1;
  pending[0] = pin;
  visited[pin] = true;
  while (head < tail) {
    const int current = pending[head++];
    if (modes[current] == OUTPUT && outputLevels[current] == LOW) return LOW;
    for (uint8_t index = 0; index < PIN_COUNT; ++index) {
      const int next = KEYPAD_PINS[index];
      if (!visited[next] && electricalContacts[current][next]) {
        visited[next] = true;
        pending[tail++] = next;
      }
    }
  }
  return HIGH;
}

unsigned long millis() { return static_cast<uint32_t>(nowUs / 1000); }
unsigned long micros() { return static_cast<uint32_t>(nowUs); }

static void captureSerial(const char* value) {
  serialPending += value;
  std::string::size_type end = 0;
  while ((end = serialPending.find('\n')) != std::string::npos) {
    serialLines.push_back(serialPending.substr(0, end));
    serialPending.erase(0, end + 1);
  }
}

void MockSerial::begin(unsigned long baud) { require(baud == 115200, "unexpected serial baud"); }
void MockSerial::println(const char* value) { captureSerial(value); captureSerial("\n"); }
void MockSerial::println() { captureSerial("\n"); }
int MockSerial::printf(const char* format, ...) {
  char buffer[512];
  va_list arguments;
  va_start(arguments, format);
  const int written = std::vsnprintf(buffer, sizeof(buffer), format, arguments);
  va_end(arguments);
  require(written >= 0 && static_cast<size_t>(written) < sizeof(buffer), "serial output truncated");
  captureSerial(buffer);
  return written;
}

static void advance(uint32_t durationUs) {
  const uint64_t end = nowUs + durationUs;
  while (nowUs < end) {
    nowUs += 10;
    loop();
    checkElectricalSafety();
  }
}

static void setContacts(std::initializer_list<std::pair<uint8_t, uint8_t>> pairs) {
  for (auto& row : electricalContacts) for (bool& closed : row) closed = false;
  for (const auto& pair : pairs) {
    require(pair.first < PIN_COUNT && pair.second < PIN_COUNT && pair.first != pair.second,
            "invalid contact pair");
    const int first = KEYPAD_PINS[pair.first];
    const int second = KEYPAD_PINS[pair.second];
    electricalContacts[first][second] = electricalContacts[second][first] = true;
  }
}

static std::string pairMessage(uint8_t first, uint8_t second) {
  return "GPIO" + std::to_string(KEYPAD_PINS[first]) + " <-> GPIO" +
    std::to_string(KEYPAD_PINS[second]);
}

static void expect(std::initializer_list<std::string> messages) {
  require(serialPending.empty(), "serial event must be a complete line");
  require(serialLines == std::vector<std::string>(messages), "unexpected event sequence");
  serialLines.clear();
}

static void startScenario(const char* name) {
  scenario = name;
  ++scenarios;
  setContacts({});
  advance(70000);
  serialLines.clear();
  require(serialPending.empty(), "previous scenario left incomplete serial output");
}

static void testPair(uint8_t first, uint8_t second) {
  startScenario("single pair, debounce, hold, release and repress");
  setContacts({{first, second}});
  advance(10000);
  expect({});
  setContacts({});
  advance(70000);
  expect({});

  setContacts({{first, second}});
  advance(20000);
  expect({});
  setContacts({});
  advance(10000);
  setContacts({{first, second}});
  advance(20000);
  expect({});
  advance(50000);
  expect({pairMessage(first, second)});
  advance(500000);
  expect({});

  setContacts({});
  advance(20000);
  expect({});
  setContacts({{first, second}});
  advance(70000);
  expect({});
  setContacts({});
  advance(70000);
  expect({"RELEASE"});
  advance(70000);
  expect({});

  setContacts({{first, second}});
  advance(70000);
  expect({pairMessage(first, second)});
  setContacts({});
  advance(70000);
  expect({"RELEASE"});
}

static void testMultiple(const std::pair<uint8_t, uint8_t>& first,
                         const std::pair<uint8_t, uint8_t>& second) {
  startScenario("two simultaneous keys including indirect paths");
  setContacts({first, second});
  advance(70000);
  expect({"MULTIPLE CONTACTS: press only one key at a time."});
  advance(100000);
  expect({});
  setContacts({});
  advance(70000);
  expect({"RELEASE"});
}

static void testPartialRelease() {
  startScenario("multi-key partial release resolves the remaining contact");
  setContacts({{uint8_t{0}, uint8_t{1}}, {uint8_t{1}, uint8_t{2}}});
  advance(70000);
  expect({"MULTIPLE CONTACTS: press only one key at a time."});
  setContacts({{uint8_t{1}, uint8_t{2}}});
  advance(70000);
  expect({pairMessage(1, 2)});
  setContacts({});
  advance(70000);
  expect({"RELEASE"});
}

static void testClockWrap(bool milliseconds) {
  startScenario(milliseconds ? "millis rollover during press debounce" :
                              "micros rollover during drive settling");
  nowUs = milliseconds ? (static_cast<uint64_t>(UINT32_MAX) - 30) * 1000 :
                          static_cast<uint64_t>(UINT32_MAX) - 10000;
  setContacts({{uint8_t{0}, uint8_t{5}}});
  advance(70000);
  expect({pairMessage(0, 5)});
  setContacts({});
  advance(70000);
  expect({"RELEASE"});
}

int main() {
  require(PIN_COUNT == 6, "the diagnostic scanner must have six configured wires");
  for (uint8_t first = 0; first < PIN_COUNT; ++first) {
    require(KEYPAD_PINS[first] < 64, "GPIO outside mock range");
    for (uint8_t second = first + 1; second < PIN_COUNT; ++second) {
      require(KEYPAD_PINS[first] != KEYPAD_PINS[second], "duplicate configured GPIO");
    }
  }
  setup();
  require(!serialLines.empty(), "setup must provide serial instructions");
  serialLines.clear();
  advance(70000);
  expect({});

  std::vector<std::pair<uint8_t, uint8_t>> pairs;
  for (uint8_t first = 0; first < PIN_COUNT; ++first) {
    for (uint8_t second = first + 1; second < PIN_COUNT; ++second) {
      pairs.emplace_back(first, second);
      testPair(first, second);
    }
  }
  require(pairs.size() == 15, "all unordered contact pairs must be exercised");
  for (size_t first = 0; first < pairs.size(); ++first) {
    for (size_t second = first + 1; second < pairs.size(); ++second) {
      testMultiple(pairs[first], pairs[second]);
    }
  }
  testPartialRelease();
  testClockWrap(false);
  testClockWrap(true);
  std::cout << "PASS: wiring-test sketch (" << scenarios << " scenarios, "
            << gpioTransitions << " GPIO transitions checked)\n";
  return 0;
}
