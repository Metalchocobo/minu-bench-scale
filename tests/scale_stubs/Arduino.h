#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

#define F(value) value

using std::max;
using std::min;

template <typename T>
T constrain(T value, T low, T high) {
  return std::max(low, std::min(value, high));
}

class ScaleTestSerial {
public:
  template <typename T> void print(const T&) {}
  template <typename T> void print(const T&, int) {}
  template <typename T> void println(const T&) {}
  template <typename T> void println(const T&, int) {}
};

extern ScaleTestSerial Serial;
unsigned long millis();
