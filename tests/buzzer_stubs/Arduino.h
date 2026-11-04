#pragma once

#include <cstddef>
#include <cstdint>

#define F(value) value

class BuzzerTestSerial {
public:
  template <typename T> void println(const T&) {}
};

extern BuzzerTestSerial Serial;
bool ledcAttach(uint8_t pin, uint32_t frequency, uint8_t resolution);
double ledcWriteTone(uint8_t pin, uint32_t frequency);
void delay(uint32_t durationMs);
unsigned long millis();
