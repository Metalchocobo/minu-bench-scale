// Compile with the production battery_monitor.cpp and tests/battery_stubs.
// These tests exercise the public API, real filtering and real debounce logic.
#include "../firmware/esp32_hx711_serial/battery_monitor.h"
#include "../firmware/esp32_hx711_serial/config/config_battery.h"
#include "../firmware/esp32_hx711_serial/config/config_pins.h"
#include <Adafruit_INA219.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

BatteryTestSerial Serial;
BatteryTestINA::Sample BatteryTestINA::sample;
BatteryTestGPIO::State BatteryTestGPIO::state;

static uint32_t nowMs = 100;
static std::string scenario = "initialization";
static unsigned int scenarios = 0;

unsigned long millis() { return nowMs; }

void pinMode(uint8_t pin, uint8_t mode) {
  BatteryTestGPIO::state.configuredPin = pin;
  BatteryTestGPIO::state.configuredMode = mode;
}

int digitalRead(uint8_t pin) {
  if (pin != USB_PRESENT_PIN) {
    std::cerr << "FAIL: unexpected GPIO read " << static_cast<int>(pin) << '\n';
    std::exit(1);
  }
  ++BatteryTestGPIO::state.reads;
  return BatteryTestGPIO::state.level;
}

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

static void setUsb(bool present) {
  BatteryTestGPIO::state.level = present ? LOW : HIGH;
}

static void reset(const std::string& name, float voltage = 6.70f,
                  float current = 200.0f, bool sensorPresent = true,
                  bool usbPresent = false, uint32_t initialMs = 100) {
  scenario = name;
  ++scenarios;
  nowMs = initialMs;
  BatteryTestINA::sample = BatteryTestINA::Sample{};
  BatteryTestINA::sample.busVoltage_V = voltage;
  BatteryTestINA::sample.current_mA = current;
  BatteryTestINA::sample.present = sensorPresent;
  BatteryTestGPIO::state = BatteryTestGPIO::State{};
  setUsb(usbPresent);
  battery_init();
  require(USB_PRESENT_PIN == 36 && USB_PRESENT_ACTIVE_LOW,
          "the installed PC817 must use active-low GPIO36");
  require(BatteryTestGPIO::state.configuredPin == USB_PRESENT_PIN,
          "USB presence pin must be configured");
  require(BatteryTestGPIO::state.configuredMode == INPUT,
          "GPIO36 must use INPUT with the external pull-up");
  require(BatteryTestGPIO::state.reads == 0, "initialization must not sample USB");
  require(!battery_get_status().charging, "initialization must clear indication");
  require(!battery_has_fresh_sample(nowMs), "initialization has no fresh INA sample");
}

static void tick(uint32_t elapsedMs = 250) {
  nowMs += elapsedMs;
  battery_update(nowMs);
}

static void hold(uint32_t elapsedMs) {
  require(elapsedMs % 250 == 0, "test duration must align with USB polling");
  for (uint32_t elapsed = 0; elapsed < elapsedMs; elapsed += 250) tick();
}

static void connectUsb() {
  setUsb(true);
  tick(0);
  require(!battery_get_status().charging, "USB entry must start with a candidate");
  tick(249);
  require(!battery_get_status().charging, "249 ms must not confirm USB entry");
  tick(1);
  require(battery_get_status().charging, "250 ms must confirm USB entry");
}

static void disconnectUsb() {
  setUsb(false);
  tick(0);
  require(battery_get_status().charging, "USB removal must start with a candidate");
  tick(249);
  require(battery_get_status().charging, "249 ms must not confirm USB removal");
  tick(1);
  require(!battery_get_status().charging, "250 ms must confirm USB removal");
}

static void testUsbPolarityAndDebounce() {
  reset("active-low USB presence and 250 ms transitions", 6.90f);
  hold(30000);
  require(!battery_get_status().charging, "high battery voltage without USB must stay off");
  connectUsb();
  disconnectUsb();
}

static void testUsbAtBootAndTimeZero() {
  reset("USB connected at boot with millis zero", 6.50f, 200.0f, true, true, 0);
  tick(0);
  tick(249);
  require(!battery_get_status().charging, "a time-zero candidate must wait 250 ms");
  tick(1);
  require(battery_get_status().charging, "time zero must be a valid candidate timestamp");
  require(BatteryTestINA::sample.reads == 0, "USB detection must precede the INA cadence");
}

static void testBatteryValuesDoNotControlUsb() {
  reset("USB presence is independent of battery voltage and load current", 5.60f, 900.0f);
  connectUsb();
  hold(1000);
  const BatteryStatus low = battery_get_status();
  require(low.valid && battery_has_fresh_sample(nowMs), "low battery must remain measurable");
  require(low.level == BATT_LEVEL_EMPTY, "USB must not mask the empty battery level");
  require(low.voltage_V < BatteryConfig::V_HARD_SLEEP_MIN_V,
          "USB must not clamp or raise the voltage used by hard-low protection");
  require(low.charging && low.current_mA > 0.0f,
          "positive load current and low voltage must not hide USB");
  disconnectUsb();
  BatteryTestINA::sample.busVoltage_V = 6.90f;
  BatteryTestINA::sample.current_mA = -500.0f;
  hold(30000);
  require(battery_get_status().voltage_V > 6.70f, "test must reach high battery voltage");
  require(battery_get_status().current_mA < 0.0f, "test must retain negative current");
  require(!battery_get_status().charging, "voltage and current sign cannot invent USB presence");
}

static void testEntryNoiseResetsCandidate() {
  reset("a USB bounce restarts entry debounce");
  setUsb(true);
  tick(0);
  tick(249);
  setUsb(false);
  tick(0);
  tick(1);
  connectUsb();
}

static void testExitNoiseResetsCandidate() {
  reset("a USB bounce restarts removal debounce");
  connectUsb();
  setUsb(false);
  tick(0);
  tick(249);
  setUsb(true);
  tick(0);
  tick(1);
  disconnectUsb();
}

static void testPollingGapRestartsCandidates() {
  reset("a polling gap cannot complete entry debounce");
  setUsb(true);
  tick(0);
  tick(249);
  tick(251);
  require(!battery_get_status().charging, "an unsampled gap must restart USB entry");
  tick(249);
  require(!battery_get_status().charging, "entry needs a new full debounce after a gap");
  tick(1);
  require(battery_get_status().charging, "entry must recover after 250 observed ms");

  scenario = "a polling gap cannot complete removal debounce";
  ++scenarios;
  setUsb(false);
  tick(0);
  tick(249);
  tick(251);
  require(battery_get_status().charging, "an unsampled gap must preserve confirmed USB state");
  tick(249);
  require(battery_get_status().charging, "removal needs a new full debounce after a gap");
  tick(1);
  require(!battery_get_status().charging, "removal must recover after 250 observed ms");
}

static void testRollover() {
  reset("USB debounce across millis rollover", 6.50f, 200.0f, true, false,
        std::numeric_limits<uint32_t>::max() - 100);
  connectUsb();
  disconnectUsb();
}

static void testReadFailures() {
  for (int failedRead = 0; failedRead < 3; ++failedRead) {
    reset("INA transaction failure " + std::to_string(failedRead));
    connectUsb();
    hold(1000);
    require(battery_has_fresh_sample(nowMs), "test needs a valid INA baseline");
    if (failedRead == 0) BatteryTestINA::sample.busOk = false;
    if (failedRead == 1) BatteryTestINA::sample.shuntOk = false;
    if (failedRead == 2) BatteryTestINA::sample.currentOk = false;
    hold(500);
    require(!battery_get_status().valid, "a failed transaction must invalidate the INA sample");
    require(!battery_has_fresh_sample(nowMs), "invalid INA samples cannot be fresh");
    require(battery_get_status().charging, "an INA failure must not clear USB presence");
    disconnectUsb();
    BatteryTestINA::sample.busOk = true;
    BatteryTestINA::sample.shuntOk = true;
    BatteryTestINA::sample.currentOk = true;
    hold(500);
    require(battery_has_fresh_sample(nowMs), "valid INA reads must recover freshness");
    require(!battery_get_status().charging, "INA recovery must not re-enable disconnected USB");
    connectUsb();
  }
}

static void testImplausibleSamples() {
  for (int invalidValue = 0; invalidValue < 4; ++invalidValue) {
    reset("implausible INA sample " + std::to_string(invalidValue));
    connectUsb();
    hold(1000);
    const BatteryStatus before = battery_get_status();
    if (invalidValue == 0) BatteryTestINA::sample.busVoltage_V = std::numeric_limits<float>::quiet_NaN();
    if (invalidValue == 1) BatteryTestINA::sample.busVoltage_V = 9.10f;
    if (invalidValue == 2) BatteryTestINA::sample.shuntVoltage_mV = 331.0f;
    if (invalidValue == 3) BatteryTestINA::sample.current_mA = 3501.0f;
    hold(500);
    const BatteryStatus after = battery_get_status();
    require(!after.valid, "implausible values must invalidate the INA sample");
    require(!battery_has_fresh_sample(nowMs), "implausible INA samples cannot drive safety");
    require(after.voltage_V == before.voltage_V && after.current_mA == before.current_mA,
            "rejected INA samples must not contaminate the filters");
    require(after.lastValidMs == before.lastValidMs, "rejected samples must not refresh their age");
    require(after.charging, "implausible INA values must not clear USB presence");
    disconnectUsb();
  }
}

static void testStaleInaDoesNotControlUsb() {
  reset("stale INA sample leaves USB independent");
  connectUsb();
  hold(1000);
  const uint32_t lastValidMs = battery_get_status().lastValidMs;
  require(battery_has_fresh_sample(lastValidMs + 1500), "freshness includes the 1500 ms boundary");
  require(!battery_has_fresh_sample(lastValidMs + 1501), "safety must reject older INA samples");
  tick(1501);
  require(battery_has_fresh_sample(nowMs), "recovery INA sample must become fresh");
  require(battery_get_status().charging, "a stale INA gap must preserve confirmed USB presence");
  disconnectUsb();
}

static void testSensorCadenceAndVoltageReconstruction() {
  reset("sensor cadence and bus-plus-shunt voltage", 6.68f);
  BatteryTestINA::sample.shuntVoltage_mV = 20.0f;
  tick(500);
  require(std::fabs(battery_get_status().voltage_V - 6.70f) < 0.00001f,
          "battery voltage must include the shunt drop");
  const unsigned int reads = BatteryTestINA::sample.reads;
  const uint32_t lastValidMs = battery_get_status().lastValidMs;
  const unsigned int usbReads = BatteryTestGPIO::state.reads;
  tick(249);
  tick(250);
  require(BatteryTestINA::sample.reads == reads, "INA must not be read before 500 ms");
  require(BatteryTestGPIO::state.reads == usbReads + 2, "USB must be polled between INA reads");
  require(battery_get_status().lastValidMs == lastValidMs,
          "skipped INA updates must not make the sample newer");
  tick(1);
  require(BatteryTestINA::sample.reads == reads + 3, "INA must be read at 500 ms");
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
    tick(500);
    require(battery_get_status().valid, "band test must use a valid INA sample");
    require(battery_get_status().level == band.level, "wrong level at a display boundary");
    require(!battery_get_status().charging, "display level must not activate USB indication");
  }
}

static void testMissingSensorAndReinitialization() {
  reset("reinitialization clears a previously active USB monitor");
  connectUsb();
  const unsigned int usbReads = BatteryTestGPIO::state.reads;
  battery_init();
  require(BatteryTestGPIO::state.reads == usbReads, "reinitialization must not sample USB");
  require(!battery_get_status().charging, "reinitialization must clear USB indication");
  require(!battery_has_fresh_sample(nowMs), "reinitialization must clear INA freshness");
  connectUsb();

  reset("missing INA sensor does not disable USB presence", 6.70f, 200.0f, false);
  connectUsb();
  hold(30000);
  require(!battery_is_available(), "missing INA must remain unavailable");
  require(!battery_get_status().valid, "missing INA cannot supply a valid sample");
  require(!battery_has_fresh_sample(nowMs), "missing INA cannot supply safety evidence");
  require(battery_get_status().charging, "USB must remain detected without INA");
  disconnectUsb();
  require(BatteryTestINA::sample.reads == 0, "missing INA must not be polled");
}

int main() {
  testUsbPolarityAndDebounce();
  testUsbAtBootAndTimeZero();
  testBatteryValuesDoNotControlUsb();
  testEntryNoiseResetsCandidate();
  testExitNoiseResetsCandidate();
  testPollingGapRestartsCandidates();
  testRollover();
  testReadFailures();
  testImplausibleSamples();
  testStaleInaDoesNotControlUsb();
  testSensorCadenceAndVoltageReconstruction();
  testBarBoundaries();
  testMissingSensorAndReinitialization();
  std::cout << "PASS: " << scenarios << " battery/USB scenarios using production battery_monitor.cpp\n";
}
