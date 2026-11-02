#pragma once

#include <stdint.h>

namespace LedConfig {

// ESP32 channel 8 is in the low-speed group, separate from the buzzer's
// auto-allocated high-speed channel and its changing tone frequency.
static const uint8_t PWM_CHANNEL = 8;
static const uint32_t PWM_HZ = 5000;
static const uint8_t PWM_BITS = 8;
static const uint32_t UPDATE_MS = 10;

// A short, deliberately uneven envelope simulates speech; BUSY is binary,
// so these pulses do not represent the actual audio amplitude.
static const uint16_t VOICE_PULSE_MS[] = {220, 300, 180, 360};
static const uint8_t VOICE_PEAK[] = {255, 150, 220, 180};

} // namespace LedConfig
