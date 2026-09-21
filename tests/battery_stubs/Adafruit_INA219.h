#pragma once

#include <stdint.h>

namespace BatteryTestINA {
struct Sample {
  bool present = true;
  bool busOk = true;
  bool shuntOk = true;
  bool currentOk = true;
  float busVoltage_V = 6.70f;
  float shuntVoltage_mV = 0.0f;
  float current_mA = 200.0f;
  unsigned int reads = 0;
};

extern Sample sample;
}

class Adafruit_INA219 {
 public:
  explicit Adafruit_INA219(uint8_t) {}
  bool begin() { return BatteryTestINA::sample.present; }
  void setCalibration_32V_2A() {}
  bool success() const { return lastReadOk; }

  float getBusVoltage_V() {
    ++BatteryTestINA::sample.reads;
    lastReadOk = BatteryTestINA::sample.busOk;
    return BatteryTestINA::sample.busVoltage_V;
  }

  float getShuntVoltage_mV() {
    ++BatteryTestINA::sample.reads;
    lastReadOk = BatteryTestINA::sample.shuntOk;
    return BatteryTestINA::sample.shuntVoltage_mV;
  }

  float getCurrent_mA() {
    ++BatteryTestINA::sample.reads;
    lastReadOk = BatteryTestINA::sample.currentOk;
    return BatteryTestINA::sample.current_mA;
  }

 private:
  bool lastReadOk = true;
};
