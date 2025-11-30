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

static void playToneHz(uint32_t freq, uint32_t durationMs) {
  if (freq == 0) {
    buzzerSilence();
    delay(durationMs);
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

static Note bootMelody[] = {
  // (SALE) "ta ra ta ta"
  {  523, 140 },  // C5  ta
  {  659, 140 },  // E5  ra
  {  784, 140 },  // G5  ta
  {  880, 140 },  // A5  ta
  {    0,  40 },  // pausa breve

  // (SCENDE POCO, ONDINA) "taa"
  {  988, 140 },  // B5  ta
  {  880, 140 },  // A5  ra (scende poco)
  {  784, 140 },  // G5  ta
  {  880, 360 },  // A5  taa (più lunga)
  {    0,  40 },  // pausa breve

  // (SALE) "ta ta taaa"
  {  880, 140 },  // A5  ta
  {  988, 140 },  // B5  ta
  { 1047, 900 }   // C6  taaa (finale pieno)
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
