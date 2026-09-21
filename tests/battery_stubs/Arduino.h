#pragma once

#include <stdint.h>

#define F(value) value

class BatteryTestSerial {
 public:
  template <typename Value> void print(const Value&) {}
  template <typename Value> void print(const Value&, int) {}
  template <typename Value> void println(const Value&) {}
};

extern BatteryTestSerial Serial;
unsigned long millis();
