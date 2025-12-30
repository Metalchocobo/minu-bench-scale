#pragma once

#include <Arduino.h>

// Minimal HX711 driver (no external libraries).
//
// Assunzioni:
// - HX711 alimentato a 5 V (o 3.3 V). Se alimentato a 5 V, DOUT va portato a 3.3 V verso ESP32.
// - Lettura 24-bit signed (two's complement), restituita come long.

enum HX711Gain {
  // Numero di impulsi extra dopo i 24 bit.
  // 1 -> CH A, gain 128
  // 2 -> CH B, gain 32
  // 3 -> CH A, gain 64
  HX711_GAIN_128_A = 1,
  HX711_GAIN_32_B  = 2,
  HX711_GAIN_64_A  = 3
};

void hx711_begin(int doutPin, int sckPin);
void hx711_set_gain(HX711Gain gain);

// DOUT LOW => dato pronto.
bool hx711_is_ready();

// Attende DOUT LOW, ritorna true se pronto entro timeout.
bool hx711_wait_ready(uint32_t timeoutMs);

// Lettura non-bloccante (assume che hx711_is_ready() sia true).
long hx711_read();

// Lettura bloccante con timeout (ms). Ritorna 0 su timeout.
long hx711_read_blocking(uint32_t timeoutMs);
