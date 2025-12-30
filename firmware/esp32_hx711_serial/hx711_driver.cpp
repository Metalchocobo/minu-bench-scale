#include "hx711_driver.h"

static int g_dout = -1;
static int g_sck  = -1;
static HX711Gain g_gain = HX711_GAIN_128_A;

// Timing: l'HX711 è abbastanza tollerante. Usiamo impulsi molto corti,
// ma senza scendere sotto 0.2 µs (Arduino digitalWrite è comunque più lento).
static inline void pulseSckOnce() {
  digitalWrite(g_sck, HIGH);
  // delayMicroseconds(1) qui sarebbe "troppo", ma non fa male.
  // Preferiamo niente delay: digitalWrite su ESP32 introduce già latenza.
  digitalWrite(g_sck, LOW);
}

void hx711_begin(int doutPin, int sckPin) {
  g_dout = doutPin;
  g_sck  = sckPin;

  pinMode(g_dout, INPUT);
  pinMode(g_sck, OUTPUT);
  digitalWrite(g_sck, LOW);
}

void hx711_set_gain(HX711Gain gain) {
  g_gain = gain;
}

bool hx711_is_ready() {
  if (g_dout < 0) return false;
  return digitalRead(g_dout) == LOW;
}

bool hx711_wait_ready(uint32_t timeoutMs) {
  if (g_dout < 0) return false;

  uint32_t t0 = millis();
  while (!hx711_is_ready()) {
    if ((millis() - t0) > timeoutMs) {
      return false;
    }
    delay(1);
  }
  return true;
}

static long hx711_read_raw_24() {
  // Legge 24 bit MSB-first.
  uint32_t data = 0;
  for (int i = 0; i < 24; i++) {
    digitalWrite(g_sck, HIGH);
    data = (data << 1) | (uint32_t)(digitalRead(g_dout) & 0x1);
    digitalWrite(g_sck, LOW);
  }

  // Impulsi extra per impostare gain/canale per la prossima conversione.
  for (int i = 0; i < (int)g_gain; i++) {
    pulseSckOnce();
  }

  // Sign-extend 24-bit two's complement.
  if (data & 0x800000) {
    data |= 0xFF000000;
  }
  return (long)((int32_t)data);
}

long hx711_read() {
  if (g_dout < 0 || g_sck < 0) return 0;
  return hx711_read_raw_24();
}

long hx711_read_blocking(uint32_t timeoutMs) {
  if (g_dout < 0 || g_sck < 0) return 0;

  if (!hx711_wait_ready(timeoutMs)) {
    return 0;
  }
  return hx711_read_raw_24();
}
