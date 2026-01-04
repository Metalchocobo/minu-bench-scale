#pragma once
#include <Arduino.h>

// Minimal HX711 driver (no external libraries).
// - DOUT LOW => dato pronto
// - Lettura 24-bit signed (two's complement), restituita come long
//
// NOTE HW: se HX711 è a 5 V, porta DOUT a 3.3 V verso ESP32 (partitore resistivo).
// SCK può essere 3.3 V (consigliato). Evita level shifter "I2C-style" su SCK.

enum HX711Gain {
  // Numero di impulsi extra dopo i 24 bit:
  // 1 -> CH A, gain 128
  // 2 -> CH B, gain 32
  // 3 -> CH A, gain 64
  HX711_GAIN_128_A = 1,
  HX711_GAIN_32_B  = 2,
  HX711_GAIN_64_A  = 3,
};

struct HX711Result {
  bool ok;
  long value;
};

void hx711_begin(int doutPin, int sckPin, HX711Gain gain = HX711_GAIN_128_A, uint8_t pulseUs = 1);
void hx711_set_gain(HX711Gain gain);

// DOUT LOW => data ready
bool hx711_is_ready();

// Attende DOUT LOW. Ritorna true se pronto entro timeoutMs.
bool hx711_wait_ready(uint32_t timeoutMs);

// Lettura non-bloccante (assume che hx711_is_ready() sia true).
long hx711_read();

// Lettura bloccante con timeout (ms). Ritorna {ok=false} su timeout.
HX711Result hx711_read_blocking(uint32_t timeoutMs);
