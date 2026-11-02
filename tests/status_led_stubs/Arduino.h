#pragma once

#include <stdint.h>

static const int LOW = 0;
static const int HIGH = 1;
static const int OUTPUT = 1;

#define F(value) value

struct SerialStub {
  void println(const char*) {}
};
extern SerialStub Serial;

void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
bool ledcAttachChannel(uint8_t pin, uint32_t frequency, uint8_t resolution,
                       uint8_t channel);
bool ledcDetach(uint8_t pin);
bool ledcWrite(uint8_t pin, uint32_t duty);
