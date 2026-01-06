#pragma once

#include <Arduino.h>

// HX711 runtime "health" state machine.
// Obiettivo: rilevare assenza di campioni validi senza bloccare la loop.

enum HxHealthState : uint8_t {
  HX_HEALTH_OK = 0,
  HX_HEALTH_WARN,
  HX_HEALTH_ERROR,
  HX_HEALTH_ERROR_HARD,
};

struct HxHealthConfig {
  // Soglie temporali
  uint32_t warnAfterMs;        // entra in WARN se manca un campione valido per >= warnAfterMs
  uint32_t errorAfterMs;       // entra in ERROR se manca un campione valido per >= errorAfterMs
  uint32_t hardAfterMs;        // passa a ERROR HARD se l'assenza dura >= hardAfterMs
  uint32_t okStableMs;         // torna a OK solo dopo okStableMs di campioni validi "stabili" (senza gap >= warnAfterMs)

  // Ultimo valore valido mostrabile in ERROR (soft)
  uint32_t lastValidMaxAgeMs;  // in ERROR (soft) mostra lastValid solo se non più vecchio di questa soglia
};

struct HxHealth {
  HxHealthConfig cfg;

  HxHealthState state;

  // Timestamp ultimo RAW letto con successo (campione valido)
  uint32_t lastRawMs;

  // Timestamp ultimo valore in grammi "accettato"/mostrato
  uint32_t lastValidMs;
  float    lastValidG;
  bool     hadValidEver;

  // Recovery: quando i campioni riprendono, attendi OK stabile
  uint32_t okCandidateStartMs;

  // Beep una tantum all'ingresso nello stato ERROR (soft o hard)
  bool beepPending;
};

void hxHealth_init(HxHealth* h, const HxHealthConfig& cfg);

// Nota un RAW valido letto dall'HX711 (hx711_is_ready=true + hx711_read completata)
void hxHealth_noteRaw(HxHealth* h, uint32_t nowMs);

// Nota il valore in grammi effettivamente "accettato" per la visualizzazione
void hxHealth_noteValid(HxHealth* h, uint32_t nowMs, float grams);

// Aggiorna la state machine (da chiamare ad ogni loop)
void hxHealth_update(HxHealth* h, uint32_t nowMs);

inline HxHealthState hxHealth_state(const HxHealth* h) { return h ? h->state : HX_HEALTH_ERROR_HARD; }
inline bool hxHealth_isWarn(const HxHealth* h) { return h && h->state == HX_HEALTH_WARN; }
inline bool hxHealth_isError(const HxHealth* h) { return h && (h->state == HX_HEALTH_ERROR || h->state == HX_HEALTH_ERROR_HARD); }
inline bool hxHealth_isHard(const HxHealth* h) { return h && h->state == HX_HEALTH_ERROR_HARD; }

// True una sola volta quando si entra in ERROR (soft o hard)
bool hxHealth_popEnterErrorBeep(HxHealth* h);

// In ERROR (soft) mostra lastValid solo se presente e recente
bool hxHealth_canShowLastValueSoft(const HxHealth* h, uint32_t nowMs);

inline float hxHealth_lastValueG(const HxHealth* h) { return h ? h->lastValidG : 0.0f; }
inline uint32_t hxHealth_lastValueAgeMs(const HxHealth* h, uint32_t nowMs) {
  if (!h || !h->hadValidEver) return 0xFFFFFFFFu;
  return (nowMs >= h->lastValidMs) ? (nowMs - h->lastValidMs) : 0;
}
