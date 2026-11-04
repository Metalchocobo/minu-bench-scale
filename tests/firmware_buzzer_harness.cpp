// Compile with production buzzer.cpp and tests/buzzer_stubs.
// GPIO/PWM and delay calls are recorded; the alert state machine is production code.
#include "../firmware/esp32_hx711_serial/buzzer.h"

#include <cstdlib>
#include <iostream>
#include <limits>
#include <vector>

void playToneHz(uint32_t frequency, uint32_t durationMs);

BuzzerTestSerial Serial;
static uint32_t nowMs = 0;
static uint32_t outputHz = 0;
static unsigned int delayCalls = 0;
static unsigned int attachCalls = 0;
static std::vector<uint32_t> toneWrites;
static const char* scenario = "initialization";
static unsigned int scenarios = 0;
static unsigned int assertions = 0;

static void require(bool condition, const char* message) {
  ++assertions;
  if (!condition) {
    std::cerr << "FAIL [" << scenario << "]: " << message << '\n';
    std::exit(1);
  }
}

bool ledcAttach(uint8_t pin, uint32_t frequency, uint8_t resolution) {
  require(pin == 22, "buzzer attaches to GPIO22");
  require(frequency > 0 && resolution > 0, "PWM attach uses valid parameters");
  ++attachCalls;
  return true;
}

double ledcWriteTone(uint8_t pin, uint32_t frequency) {
  require(pin == 22, "tone uses GPIO22");
  outputHz = frequency;
  toneWrites.push_back(frequency);
  return frequency;
}

void delay(uint32_t durationMs) {
  ++delayCalls;
  nowMs += durationMs;
}

unsigned long millis() { return nowMs; }

static void update(bool overloaded, uint32_t timestamp) {
  nowMs = timestamp;
  buzzerUpdateOverload(overloaded, timestamp);
}

static void start(const char* name) {
  scenario = name;
  ++scenarios;
  nowMs = 0;
  delayCalls = 0;
  toneWrites.clear();
  buzzerInit();
  require(outputHz == 0, "initialization silences buzzer");
  toneWrites.clear();
}

static void genericBeeps() {
  playToneHz(2200, 100);
  playToneHz(0, 100);
  buzzerKeyClick();
  buzzerOk();
  buzzerWarn();
  buzzerError();
  buzzerMqttRx();
}

int main() {
  start("exact double-beep timing boundaries");
  update(true, 1000);
  require(outputHz == 1500, "first tone begins on overload edge");
  update(true, 1119);
  require(outputHz == 1500, "first tone lasts through 119 ms");
  update(true, 1120);
  require(outputHz == 0, "first tone ends at 120 ms");
  update(true, 1199);
  require(outputHz == 0, "gap lasts through 199 ms");
  update(true, 1200);
  require(outputHz == 1500, "second tone begins at 200 ms");
  update(true, 1319);
  require(outputHz == 1500, "second tone lasts through 319 ms");
  update(true, 1320);
  require(outputHz == 0, "pattern ends at 320 ms");
  require(delayCalls == 0, "alert timing never calls delay");

  start("sustained overload sounds once and rearms only after clear");
  update(true, 10);
  update(true, 330);
  const std::size_t completedWrites = toneWrites.size();
  update(true, 10000);
  update(true, 20000);
  require(outputHz == 0, "persistent overload does not start another pattern");
  require(toneWrites.size() == completedWrites, "persistent overload does not reissue PWM after completion");
  update(false, 20001);
  update(true, 20002);
  require(outputHz == 1500, "new overload after clear starts a fresh pattern");
  require(delayCalls == 0, "edge-trigger and rearm remain nonblocking");

  start("clearing during tone immediately silences and rearms");
  update(true, 500);
  update(false, 550);
  require(outputHz == 0, "clear interrupts the current tone");
  update(false, 900);
  require(outputHz == 0, "safe state remains silent");
  update(true, 901);
  require(outputHz == 1500, "clear permits another overload edge");
  update(true, 1221);
  require(outputHz == 0, "rearmed pattern completes on time");
  require(delayCalls == 0, "early clear never blocks");

  start("cancel silences without retriggering a latched overload");
  update(true, 100);
  buzzerCancelOverload();
  require(outputHz == 0, "cancel immediately silences PWM");
  update(true, 101);
  update(true, 5000);
  require(outputHz == 0, "cancel retains overload latch");
  update(false, 5001);
  update(true, 5002);
  require(outputHz == 1500, "clear rearms after cancellation");
  require(delayCalls == 0, "cancellation is nonblocking");

  start("cancellation in the gap suppresses the second tone");
  update(true, 100);
  update(true, 220);
  require(outputHz == 0, "pattern is in its silent gap");
  buzzerCancelOverload();
  update(true, 300);
  require(outputHz == 0, "cancelled pattern does not play its second tone");
  update(true, 1000);
  require(outputHz == 0, "cancelled latched overload stays silent");

  start("elapsed time remains correct across uint32 rollover");
  const uint32_t origin = std::numeric_limits<uint32_t>::max() - 50u;
  update(true, origin);
  require(outputHz == 1500, "rollover test starts first tone");
  update(true, origin + 119u);
  require(outputHz == 1500, "first tone spans millis rollover");
  update(true, origin + 120u);
  require(outputHz == 0, "gap begins after rollover");
  update(true, origin + 200u);
  require(outputHz == 1500, "second tone begins after rollover");
  update(true, origin + 320u);
  require(outputHz == 0, "pattern finishes after rollover");
  update(true, origin + 10000u);
  require(outputHz == 0, "rollover does not retrigger persistent overload");
  require(delayCalls == 0, "rollover handling remains nonblocking");

  start("late polling skips elapsed phases without extending pattern");
  update(true, 100);
  update(true, 350);
  require(outputHz == 1500, "late poll directly selects the active second tone");
  update(true, 420);
  require(outputHz == 0, "late phase does not extend completion deadline");
  update(false, 500);
  update(true, 600);
  update(true, 2000);
  require(outputHz == 0, "poll past entire pattern silences without replaying missing phases");
  require(delayCalls == 0, "late polling does not replay phases with delays");

  start("overload pattern has priority over generic beeps");
  update(true, 1000);
  std::size_t before = toneWrites.size();
  genericBeeps();
  require(outputHz == 1500, "generic tones cannot stop active overload tone");
  require(toneWrites.size() == before, "generic beeps issue no PWM writes during overload pattern");
  require(delayCalls == 0, "generic beeps add no delays during overload pattern");
  update(true, 1120);
  before = toneWrites.size();
  genericBeeps();
  require(outputHz == 0, "generic beeps do not fill the overload pattern gap");
  require(toneWrites.size() == before, "silent gap retains overload priority");
  require(delayCalls == 0, "silent gap remains nonblocking");
  update(true, 1200);
  require(outputHz == 1500, "second overload tone still occurs after suppressed beeps");
  update(true, 1320);
  const unsigned int delaysBefore = delayCalls;
  buzzerOk();
  require(delayCalls > delaysBefore, "normal beep behavior resumes after pattern completion");
  require(outputHz == 0, "normal confirmation beep finishes silent");

  start("initialization resets a previous overload latch");
  update(true, 100);
  buzzerCancelOverload();
  buzzerInit();
  update(true, 200);
  require(outputHz == 1500, "reinitialized driver can start a new pattern");
  require(delayCalls == 0, "initialization and alert do not delay");
  require(attachCalls == scenarios + 1, "each initialization attaches PWM once");

  std::cout << "PASS: " << scenarios << " buzzer scenarios, " << assertions << " assertions\n";
}
