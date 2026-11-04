#pragma once

#include "../scale_stubs/Arduino.h"

// ESP32 libc supplies strlcpy; MSVC's host libc does not.
inline std::size_t strlcpy(char* output, const char* input, std::size_t capacity) {
  const std::size_t length = std::strlen(input);
  if (capacity != 0) {
    const std::size_t copied = std::min(length, capacity - 1);
    std::memcpy(output, input, copied);
    output[copied] = '\0';
  }
  return length;
}
