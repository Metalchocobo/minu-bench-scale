// Compile with the production battery_monitor.cpp and tests/battery_stubs.
// These tests exercise the public API, real filtering and real debounce logic.
#include "../firmware/esp32_hx711_serial/battery_monitor.h"
#include <Adafruit_INA219.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

BatteryTestSerial Serial;
BatteryTestINA::Sample BatteryTestINA::sample;

static uint32_t nowMs = 100;
static std::string scenario = "initialization";
static unsigned int scenarios = 0;

unsigned long millis() { return nowMs; }

static void require(bool condition, const char* message) {
  if (!condition) {
    const BatteryStatus status = battery_get_status();
    std::cerr << "FAIL [" << scenario << "] " << message
              << " at " << nowMs << " ms, voltage=" << status.voltage_V
              << ", charging=" << status.charging
              << ", valid=" << status.valid << '\n';
    std::exit(1);
  }
}

static void reset(const std::string& name, float voltage = 6.70f,
                  float current = 200.0f, bool present = true) {
  scenario = name;
  ++scenarios;
  nowMs = 100;
  BatteryTestINA::sample = BatteryTestINA::Sample{};
  BatteryTestINA::sample.busVoltage_V = voltage;
  BatteryTestINA::sample.current_mA = current;
  BatteryTestINA::sample.present = present;
  battery_init();
  require(!battery_get_status().charging, "initialization must clear indication");
  require(!battery_has_fresh_sample(nowMs), "initialization has no fresh sample");
}

static void tick(uint32_t elapsedMs = 500) {
  nowMs += elapsedMs;
  battery_update(nowMs);
}

static void hold(uint32_t elapsedMs) {
  require(elapsedMs % 500 == 0, "test duration must align with sensor cadence");
  for (uint32_t elapsed = 0; elapsed < elapsedMs; elapsed += 500) tick();
}

static void enterCharging() {
  tick();
  require(battery_has_fresh_sample(nowMs), "first sample must be valid and fresh");
  require(!battery_get_status().charging, "first high sample is not sufficient");
  hold(4500);
  require(!battery_get_status().charging, "entry must wait the full five seconds");
  tick();
  require(battery_get_status().charging, "five seconds at 6.70 V must activate");
}

static void beginLowVoltage() {
  // Cross the filtered exit threshold, then hold the raw reading at 6.68 V.
  // An EMA approaching an exact boundary from above need not cross it.
  BatteryTestINA::sample.busVoltage_V = 6.50f;
  tick();
  require(battery_get_status().voltage_V <= 6.68f,
          "the filtered voltage must enter the exit band");
  require(battery_get_status().charging, "a low sample must not end indication");
  BatteryTestINA::sample.busVoltage_V = 6.68f;
}

static void testVoltageEntryWithPositiveLoad() {
  reset("positive load does not hide voltage-based entry");
  enterCharging();
  require(battery_get_status().current_mA > 0.0f,
          "test must retain positive load current");
}

static void testHysteresisAndExit() {
  reset("6.69 V holds indication and 6.68 V exits after ten seconds");
  enterCharging();
  BatteryTestINA::sample.busVoltage_V = 6.69f;
  hold(30000);
  require(battery_get_status().charging, "dead band must preserve active indication");
  beginLowVoltage();
  hold(9500);
  require(battery_get_status().charging, "exit must wait the full ten seconds");
  tick();
  require(!battery_get_status().charging, "ten low seconds must end indication");
}

static void testNoMinimumOnTime() {
  reset("exit does not retain the old twenty-second minimum-on time");
  enterCharging();
  beginLowVoltage();
  hold(10000);
  require(!battery_get_status().charging, "ten low seconds must suffice immediately after entry");
}

static void testInactiveDeadBand() {
  reset("6.69 V must not start an inactive indication", 6.69f);
  hold(30000);
  require(!battery_get_status().charging, "dead band must preserve inactive indication");
}

static void testNegativeCurrentBelowThreshold() {
  reset("negative current below voltage threshold must not activate", 6.50f, -500.0f);
  hold(30000);
  require(battery_get_status().current_mA < 0.0f, "test must retain negative current");
  require(!battery_get_status().charging, "current sign must not control indication");
}

static void testEntryNoiseResetsCandidate() {
  reset("a filtered dip restarts the entry debounce");
  tick();
  hold(4500);
  BatteryTestINA::sample.busVoltage_V = 6.64f;
  tick();
  require(battery_get_status().voltage_V < 6.70f, "noise must cross the entry threshold");
  require(!battery_get_status().charging, "a dip must not finish the candidate");
  BatteryTestINA::sample.busVoltage_V = 6.80f;
  tick();
  require(battery_get_status().voltage_V >= 6.70f, "voltage must return to entry band");
  BatteryTestINA::sample.busVoltage_V = 6.70f;
  hold(4500);
  require(!battery_get_status().charging, "old candidate time must not carry over");
  tick();
  require(battery_get_status().charging, "a new full debounce must activate");
}

static void testExitNoiseResetsCandidate() {
  reset("a filtered rise restarts the exit debounce");
  enterCharging();
  beginLowVoltage();
  hold(9500);
  BatteryTestINA::sample.busVoltage_V = 6.80f;
  tick();
  require(battery_get_status().voltage_V > 6.68f, "noise must cross the exit threshold");
  require(battery_get_status().charging, "a rise must preserve indication");
  beginLowVoltage();
  hold(9500);
  require(battery_get_status().charging, "old exit time must not carry over");
  tick();
  require(!battery_get_status().charging, "a new full exit debounce must deactivate");
}

static void testReadFailures() {
  for (int failedRead = 0; failedRead < 3; ++failedRead) {
    reset("INA transaction failure " + std::to_string(failedRead));
    enterCharging();
    if (failedRead == 0) BatteryTestINA::sample.busOk = false;
    if (failedRead == 1) BatteryTestINA::sample.shuntOk = false;
    if (failedRead == 2) BatteryTestINA::sample.currentOk = false;
    tick();
    require(!battery_get_status().valid, "a failed transaction must invalidate the sample");
    require(!battery_get_status().charging, "a failed transaction must clear indication");
    require(!battery_has_fresh_sample(nowMs), "invalid samples cannot be fresh");
    BatteryTestINA::sample.busOk = true;
    BatteryTestINA::sample.shuntOk = true;
    BatteryTestINA::sample.currentOk = true;
    enterCharging();
  }
}

static void testImplausibleSamples() {
  for (int invalidValue = 0; invalidValue < 4; ++invalidValue) {
    reset("implausible INA sample " + std::to_string(invalidValue));
    enterCharging();
    if (invalidValue == 0) BatteryTestINA::sample.busVoltage_V = std::numeric_limits<float>::quiet_NaN();
    if (invalidValue == 1) BatteryTestINA::sample.busVoltage_V = 9.10f;
    if (invalidValue == 2) BatteryTestINA::sample.shuntVoltage_mV = 331.0f;
    if (invalidValue == 3) BatteryTestINA::sample.current_mA = 3501.0f;
    tick();
    require(!battery_get_status().valid, "implausible values must invalidate the sample");
    require(!battery_get_status().charging, "implausible values must clear indication");
  }
}

static void testStaleActiveIndication() {
  reset("a stale active sample cannot stay indicated on recovery");
  enterCharging();
  require(battery_has_fresh_sample(nowMs + 1500), "freshness includes the 1500 ms boundary");
  require(!battery_has_fresh_sample(nowMs + 1501), "UI must reject samples older than 1500 ms");
  tick(1501);
  require(battery_has_fresh_sample(nowMs), "recovery sample must become fresh");
  require(!battery_get_status().charging, "a stale gap must clear the previous indication");
  hold(4500);
  require(!battery_get_status().charging, "recovery must restart the entry debounce");
  tick();
  require(battery_get_status().charging, "five fresh high seconds must reactivate");
}

static void testStalePendingCandidate() {
  reset("a missing interval cannot complete the entry debounce");
  tick();
  hold(4500);
  tick(2000);
  require(!battery_get_status().charging, "time without fresh samples must not count");
  hold(4500);
  require(!battery_get_status().charging, "candidate must restart after the gap");
  tick();
  require(battery_get_status().charging, "a new uninterrupted debounce must activate");
}

static void testSensorCadenceAndVoltageReconstruction() {
  reset("sensor cadence and bus-plus-shunt voltage", 6.68f);
  BatteryTestINA::sample.shuntVoltage_mV = 20.0f;
  tick();
  require(std::fabs(battery_get_status().voltage_V - 6.70f) < 0.00001f,
          "battery voltage must include the shunt drop");
  const unsigned int reads = BatteryTestINA::sample.reads;
  const uint32_t lastValidMs = battery_get_status().lastValidMs;
  tick(499);
  require(BatteryTestINA::sample.reads == reads, "sensor must not be read before 500 ms");
  require(battery_get_status().lastValidMs == lastValidMs,
          "skipped updates must not make the sample newer");
  tick(1);
  require(BatteryTestINA::sample.reads == reads + 3, "sensor must be read at 500 ms");
}

static void testBarBoundaries() {
  struct ExpectedBand { float voltage; BatteryLevel level; };
  const ExpectedBand bands[] = {
    {6.60f, BATT_LEVEL_FULL}, {6.599f, BATT_LEVEL_GOOD},
    {6.50f, BATT_LEVEL_GOOD}, {6.499f, BATT_LEVEL_LOW},
    {6.35f, BATT_LEVEL_LOW}, {6.349f, BATT_LEVEL_CRITICAL},
    {6.10f, BATT_LEVEL_CRITICAL}, {6.099f, BATT_LEVEL_EMPTY}
  };
  for (const ExpectedBand& band : bands) {
    reset("display band at " + std::to_string(band.voltage), band.voltage);
    tick();
    require(battery_get_status().valid, "band test must use a valid sample");
    require(battery_get_status().level == band.level, "wrong level at a display boundary");
    require(!battery_get_status().charging, "display level must not activate indication");
  }
}

static void testMissingSensorAndReinitialization() {
  reset("reinitialization clears a previously active monitor");
  enterCharging();
  battery_init();
  require(!battery_get_status().charging, "reinitialization must clear indication");
  require(!battery_has_fresh_sample(nowMs), "reinitialization must clear freshness");
  enterCharging();

  reset("missing INA sensor", 6.70f, 200.0f, false);
  hold(30000);
  require(!battery_is_available(), "missing sensor must remain unavailable");
  require(!battery_get_status().charging, "missing sensor must not indicate charging");
  require(BatteryTestINA::sample.reads == 0, "missing sensor must not be polled");
}

int main() {
  testVoltageEntryWithPositiveLoad();
  testHysteresisAndExit();
  testNoMinimumOnTime();
  testInactiveDeadBand();
  testNegativeCurrentBelowThreshold();
  testEntryNoiseResetsCandidate();
  testExitNoiseResetsCandidate();
  testReadFailures();
  testImplausibleSamples();
  testStaleActiveIndication();
  testStalePendingCandidate();
  testSensorCadenceAndVoltageReconstruction();
  testBarBoundaries();
  testMissingSensorAndReinitialization();
  std::cout << "PASS: " << scenarios << " battery scenarios using production battery_monitor.cpp\n";
}
