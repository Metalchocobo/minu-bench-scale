#include <Arduino.h>
#include "buzzer.h"

static const int BUZZER_PIN      = 22;
static const uint8_t RES_BITS    = 10;   // 10 bit

// ---------- SUPPORTO GENERICO ----------

struct Note {
  uint16_t freqHz;  // 0 = pausa
  uint16_t durMs;
};

static void buzzerSilence() {
  ledcWriteTone(BUZZER_PIN, 0); // 0 = off
}

void playToneHz(uint32_t freq, uint32_t durationMs) {
  if (freq == 0 || durationMs == 0) {
    buzzerSilence();
    if (durationMs > 0) delay(durationMs);
    return;
  }

  ledcWriteTone(BUZZER_PIN, freq);
  delay(durationMs);
  buzzerSilence();
}

// ---------- INIZIALIZZAZIONE ----------

void buzzerInit() {
  bool ok = ledcAttach(BUZZER_PIN, 4000, RES_BITS); // freq base qualsiasi
  if (!ok) {
    Serial.println("[BUZZER] ERRORE: ledcAttach fallita");
  } else {
    Serial.println("[BUZZER] Inizializzato su GPIO22");
  }
  buzzerSilence();
}

// ---------- FANFARA DI BOOT ----------
//
// (sale) ta ra ta ta
// (scende poco, ondina) taa
// (sale) ta ta taaa
//
// Frequenze in DO maggiore, zona C5–C6.

Note bootMelody[] = {
  // C#5 ♪
  { 554,  83 },   // C#5
  {   0,  83 },   // pausa breve

  // C#5 ♪
  { 554,  83 },   // C#5
  {   0,  83 },   // pausa breve

  // C#5 ♪♪ (un po' più lunga)
  { 554, 166 },   // C#5
  {   0,  83 },   // pausa breve

  // C#5 ─── (lunga)
  { 554, 415 },   // C#5

  // A4 ─── (lunga)
  { 440, 498 },   // A4

  // B4 ─── (lunga)
  { 494, 498 },   // B4

  // C#5 ♪
  { 554,  83 },   // C#5
  {   0, 249 },   // pausa un po' più lunga

  // B4 ♪
  { 494,  83 },   // B4

  // C#5 ───── (coda finale fino a 3 s totali)
  { 554, 581 }    // C#5
};


static const size_t bootMelodyLen = sizeof(bootMelody) / sizeof(bootMelody[0]);

void buzzerBootMelody() {
  for (size_t i = 0; i < bootMelodyLen; ++i) {
    playToneHz(bootMelody[i].freqHz, bootMelody[i].durMs);
  }
  buzzerSilence();
}

// ---------- SUONI DI INTERFACCIA ----------

// Click tasto: beep corto e abbastanza acuto
void buzzerKeyClick() {
  playToneHz(3500, 35);  // 3,5 kHz, 35 ms
}

// Beep "OK": tono medio, un po' più lungo
void buzzerOk() {
  playToneHz(2500, 90);  // 2,5 kHz, 90 ms
}

// Errore: doppio beep più basso
void buzzerError() {
  for (int i = 0; i < 2; ++i) {
    playToneHz(1500, 120); // 1,5 kHz
    delay(80);
  }
}
