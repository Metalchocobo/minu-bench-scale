// Compile with production scale_state.cpp, scale_filters.cpp and tests/scale_stubs.
// Exercises real calibration persistence and tare APIs, without a copied model.
#include "../firmware/esp32_hx711_serial/scale_state.h"
#include <Preferences.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

ScaleTestSerial Serial;
static unsigned long nowMs = 100;
unsigned long millis() { return nowMs; }

static const long emptyPlateRaw = 100000;
static const float cpg = 200.0f;
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

static void near(float actual, float expected, const char* message) {
  require(std::isfinite(actual) && std::fabs(actual - expected) < 0.02f, message);
}

static long rawFor(float addedGrams) {
  return emptyPlateRaw + static_cast<long>(addedGrams * cpg);
}

static float netFor(long raw) {
  return static_cast<float>(raw - ScaleState::effectiveOffsetCounts()) /
    ScaleState::getScaleCpg();
}

static void start(const char* name) {
  scenario = name;
  ++scenarios;
  nowMs += 2000;
  (void)ScaleState::tareCancel(nowMs);
  ScaleState::setTareUiActive(false);
  ScaleState::setCalibrationZeroRaw(emptyPlateRaw);
  ScaleState::setScaleCpg(cpg);
  ScaleState::setZtCounts(0);
  ScaleState::resetFiltersAndState();
  ScaleTestNvs::state = {};
}

static void manualTare(long raw) {
  ScaleState::tareStart(nowMs);
  ScaleState::TareResult result = ScaleState::TARE_RESULT_PENDING;
  for (unsigned int elapsed = 10; elapsed <= 1200; elapsed += 10) {
    nowMs += 10;
    ScaleState::tareAccumSample(raw);
    result = ScaleState::tareUpdate(nowMs);
    if (result != ScaleState::TARE_RESULT_PENDING) break;
  }
  require(result == ScaleState::TARE_RESULT_APPLIED, "manual tare succeeds");
  ScaleState::setTareUiActive(false);
}

int main() {
  start("empty plate allowance and exact total-load boundary");
  near(ScaleConfig::CAPACITY_DEAD_LOAD_G, 500.0f, "fixed plate allowance is 500 g");
  near(ScaleState::totalLoadGrams(emptyPlateRaw), 500.0f, "empty calibrated plate includes allowance");
  near(ScaleState::totalLoadGrams(rawFor(14499.0f)), 14999.0f, "total includes plate plus applied load");
  require(!ScaleState::updateOverload(rawFor(14499.0f), 14499.0f, false), "one gram below 15 kg is allowed");
  require(ScaleState::updateOverload(rawFor(14500.0f), 14500.0f, false), "exactly 15 kg enters overload");
  require(ScaleState::updateOverload(rawFor(14501.0f), 14501.0f, false), "above 15 kg enters overload");

  start("hysteresis requires unloading below 14.95 kg");
  require(ScaleState::updateOverload(rawFor(14499.0f), 0.0f, true), "overload stays latched below entry");
  require(ScaleState::updateOverload(rawFor(14450.0f), 0.0f, true), "exact exit boundary stays latched");
  require(!ScaleState::updateOverload(rawFor(14449.0f), 0.0f, true), "one gram below exit clears overload");
  require(!ScaleState::updateOverload(rawFor(14475.0f), 0.0f, false), "hysteresis band does not create a new fault");

  start("manual tare cannot hide accumulated physical load");
  manualTare(rawFor(13000.0f));
  near(netFor(rawFor(13000.0f)), 0.0f, "manual tare resets operational net");
  near(ScaleState::totalLoadGrams(rawFor(13000.0f)), 13500.0f, "manual tare retains full physical load");
  const long raw = rawFor(14500.0f);
  near(netFor(raw), 1500.0f, "only new ingredient remains in net");
  require(ScaleState::updateOverload(raw, netFor(raw), false), "small net still overloads accumulated total");
  require(ScaleState::getCalibrationZeroRaw() == emptyPlateRaw, "manual tare preserves calibration reference");

  start("successive ENTER working tares and undo preserve total");
  require(ScaleState::tareApplyWorking(rawFor(7000.0f), nowMs), "first ingredient ENTER tare succeeds");
  const long previousOffset = ScaleState::getOffsetRaw();
  require(ScaleState::tareApplyWorking(rawFor(14000.0f), nowMs + 1), "second ingredient ENTER tare succeeds");
  near(netFor(rawFor(14500.0f)), 500.0f, "successive ENTER shows current ingredient only");
  require(ScaleState::updateOverload(rawFor(14500.0f), netFor(rawFor(14500.0f)), false), "ENTER cannot avoid capacity limit");
  ScaleState::setOffsetRaw(previousOffset);
  ScaleState::setZtCounts(0);
  ScaleState::resetFiltersAndState();
  near(ScaleState::totalLoadGrams(rawFor(14500.0f)), 15000.0f, "undo-style offset restore leaves total unchanged");
  require(ScaleState::getCalibrationZeroRaw() == emptyPlateRaw, "working tares and undo preserve reference");

  start("zero tracking and display resets cannot move capacity");
  const long boundaryRaw = rawFor(14500.0f);
  ScaleState::setOffsetRaw(boundaryRaw);
  ScaleState::setZtCounts(400);
  ScaleState::applyZtStepTowardZero(0.1f);
  require(ScaleState::getZtCounts() != 0, "zero tracking has a real nonzero offset");
  near(ScaleState::totalLoadGrams(boundaryRaw), 15000.0f, "zero tracking does not subtract physical load");
  ScaleState::setMode("live");
  ScaleState::setMode("work");
  ScaleState::resetFiltersAndState();
  require(ScaleState::updateOverload(boundaryRaw, 0.0f, false), "mode changes and filter reset preserve overload detection");

  start("cal save after tare persists calibration rather than runtime zero");
  manualTare(rawFor(12000.0f));
  require(ScaleState::saveToNVS(), "calibration save succeeds");
  require(ScaleTestNvs::state.offset == emptyPlateRaw, "saved zero excludes runtime container and ingredients");
  ScaleState::setCalibrationZeroRaw(-200000);
  ScaleState::setScaleCpg(100.0f);
  ScaleState::loadFromNVS();
  require(ScaleState::getCalibrationZeroRaw() == emptyPlateRaw, "reboot restores independent calibration reference");
  require(ScaleState::getOffsetRaw() == emptyPlateRaw, "cal load restores operational calibration zero");
  near(ScaleState::getScaleCpg(), cpg, "reboot restores CPG");

  start("boot or watchdog auto tare with load cannot defeat overload");
  require(ScaleState::saveToNVS(), "initial calibration saved");
  ScaleState::setOffsetRaw(0);
  ScaleState::setZtCounts(0);
  ScaleState::loadFromNVS();
  const long loadedBootRaw = rawFor(15000.0f);
  // The production boot auto-tare writes only this operational offset.
  ScaleState::setOffsetRaw(loadedBootRaw);
  ScaleState::resetFiltersAndState();
  near(netFor(loadedBootRaw), 0.0f, "loaded boot can have zero displayed net");
  near(ScaleState::totalLoadGrams(loadedBootRaw), 15500.0f, "boot still measures all load plus plate");
  require(ScaleState::updateOverload(loadedBootRaw, 0.0f, false), "loaded reboot is still overloaded");

  start("legacy NVS offset is used as calibration baseline");
  ScaleTestNvs::state.namespacePresent = true;
  ScaleTestNvs::state.hasOffset = true;
  ScaleTestNvs::state.hasScale = true;
  ScaleTestNvs::state.offset = 321000;
  ScaleTestNvs::state.scale = 250.0f;
  ScaleState::loadFromNVS();
  require(ScaleState::getCalibrationZeroRaw() == 321000, "existing offset_counts migrates without rebasing");
  near(ScaleState::totalLoadGrams(321000 + 250 * 14500), 15000.0f, "legacy calibration obeys new physical limit");
  require(ScaleTestNvs::state.writes == 0, "loading legacy calibration does not write flash");

  start("negative net measurement limit remains enforced");
  require(ScaleState::updateOverload(emptyPlateRaw, -16001.0f, false), "negative net overload remains active");
  require(!ScaleState::updateOverload(emptyPlateRaw, -16000.0f, false), "existing exact net boundary unchanged");
  require(ScaleState::updateOverload(emptyPlateRaw, -15950.0f, true), "negative net exit boundary remains latched");
  require(!ScaleState::updateOverload(emptyPlateRaw, -15949.0f, true), "negative net below exit clears fault");
  require(ScaleState::updateOverload(rawFor(14500.0f), -15949.0f, true), "safe net cannot clear unsafe total");

  start("invalid calibration or measurement fails closed");
  for (float invalid : {0.0f, -200.0f, std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::infinity()}) {
    ScaleState::setScaleCpg(invalid);
    require(ScaleState::updateOverload(emptyPlateRaw, 0.0f, false), "invalid CPG enters fault");
    require(ScaleState::updateOverload(emptyPlateRaw, 0.0f, true), "invalid CPG cannot clear existing fault");
  }
  ScaleState::setScaleCpg(cpg);
  require(ScaleState::updateOverload(emptyPlateRaw, std::numeric_limits<float>::quiet_NaN(), false), "NaN net fails closed");
  require(ScaleState::updateOverload(emptyPlateRaw, std::numeric_limits<float>::infinity(), false), "infinite net fails closed");
  require(!ScaleState::updateOverload(emptyPlateRaw, 0.0f, true), "valid safe sample recovers from invalid data");

  std::cout << "PASS: " << scenarios << " capacity scenarios, " << assertions << " assertions\n";
}
