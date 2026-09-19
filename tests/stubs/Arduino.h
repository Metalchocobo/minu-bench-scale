#pragma once

#include <stdint.h>

static const int LOW = 0;
static const int HIGH = 1;
static const int OUTPUT = 1;
static const int INPUT_PULLUP = 2;

void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
int digitalRead(int pin);
void delayMicroseconds(unsigned int us);
unsigned long millis();
