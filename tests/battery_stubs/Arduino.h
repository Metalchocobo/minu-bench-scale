#pragma once

#include <stdint.h>

#define F(value) value
#define HIGH 0x1
#define LOW 0x0
#define INPUT 0x01

namespace BatteryTestGPIO {
struct State {
  int level = HIGH;
  int configuredPin = -1;
  int configuredMode = -1;
  unsigned int reads = 0;
};

extern State state;
}

class BatteryTestSerial {
 public:
  template <typename Value> void print(const Value&) {}
  template <typename Value> void print(const Value&, int) {}
  template <typename Value> void println(const Value&) {}
};

extern BatteryTestSerial Serial;
unsigned long millis();
void pinMode(uint8_t pin, uint8_t mode);
int digitalRead(uint8_t pin);
