#include "hx_health.h"

static inline uint32_t msSince(uint32_t now, uint32_t thenMs) {
  if (thenMs == 0) return 0xFFFFFFFFu;
  return (now >= thenMs) ? (now - thenMs) : 0;
}

void hxHealth_init(HxHealth* h, const HxHealthConfig& cfg) {
  if (!h) return;
  h->cfg = cfg;
  h->state = HX_HEALTH_OK;
  // "Arm" the monitor at init: evitiamo un ERROR immediato al primo loop
  // (prima che arrivino i primi campioni reali).
  h->lastRawMs = millis();
  h->lastValidMs = 0;
  h->lastValidG = 0.0f;
  h->hadValidEver = false;
  h->okCandidateStartMs = 0;
  h->beepPending = false;
}

void hxHealth_noteRaw(HxHealth* h, uint32_t nowMs) {
  if (!h) return;
  h->lastRawMs = nowMs;
}

void hxHealth_backdateLastRaw(HxHealth* h, uint32_t lastRawMs) {
  if (!h) return;
  if (lastRawMs == 0) return;
  // Permettiamo solo di 'spostare indietro' lastRawMs (mai avanti) per mantenere coerenti le soglie.
  if (h->lastRawMs == 0 || lastRawMs < h->lastRawMs) {
    h->lastRawMs = lastRawMs;
  }
}


void hxHealth_noteValid(HxHealth* h, uint32_t nowMs, float grams) {
  if (!h) return;
  h->lastValidMs = nowMs;
  h->lastValidG = grams;
  h->hadValidEver = true;
}

bool hxHealth_popEnterErrorBeep(HxHealth* h) {
  if (!h) return false;
  if (!h->beepPending) return false;
  h->beepPending = false;
  return true;
}

bool hxHealth_canShowLastValueSoft(const HxHealth* h, uint32_t nowMs) {
  if (!h) return false;
  if (h->state != HX_HEALTH_ERROR) return false;      // solo ERROR soft
  if (!h->hadValidEver) return false;
  uint32_t age = msSince(nowMs, h->lastValidMs);
  return age <= h->cfg.lastValidMaxAgeMs;
}

void hxHealth_update(HxHealth* h, uint32_t nowMs) {
  if (!h) return;

  const HxHealthState prev = h->state;

  const uint32_t missingMs = msSince(nowMs, h->lastRawMs);

  // 1) Stato "desiderato" basato sulle soglie
  HxHealthState desired = HX_HEALTH_OK;

  // Se non abbiamo MAI avuto un valore valido, al primo ERROR diventa subito HARD
  const bool noValidEver = !h->hadValidEver;

  if (missingMs >= h->cfg.errorAfterMs) {
    if (missingMs >= h->cfg.hardAfterMs || noValidEver) {
      desired = HX_HEALTH_ERROR_HARD;
    } else {
      desired = HX_HEALTH_ERROR;
    }
  } else if (missingMs >= h->cfg.warnAfterMs) {
    desired = HX_HEALTH_WARN;
  } else {
    desired = HX_HEALTH_OK;
  }

  // 2) Recovery: torna OK solo dopo okStableMs senza gap >= warnAfterMs
  if (desired == HX_HEALTH_OK && h->state != HX_HEALTH_OK) {
    if (h->okCandidateStartMs == 0) {
      h->okCandidateStartMs = nowMs;
    }

    const uint32_t stableMs = msSince(nowMs, h->okCandidateStartMs);
    if (stableMs >= h->cfg.okStableMs) {
      h->state = HX_HEALTH_OK;
      h->okCandidateStartMs = 0;
    } else {
      // Rimani nello stato degradato finché OK non è stabile
      h->state = prev;
    }
  } else {
    // Se siamo in uno stato non-OK (WARN/ERROR/HARD), resettiamo la finestra OK
    if (desired != HX_HEALTH_OK) {
      h->okCandidateStartMs = 0;
    }
    h->state = desired;
  }

  // 3) Evento "beep una tantum" all'ingresso in ERROR/HARD
  const bool prevWasError = (prev == HX_HEALTH_ERROR || prev == HX_HEALTH_ERROR_HARD);
  const bool nowIsError   = (h->state == HX_HEALTH_ERROR || h->state == HX_HEALTH_ERROR_HARD);
  if (!prevWasError && nowIsError) {
    h->beepPending = true;
  }

  // Quando si torna OK, si riabilita beep per eventuale prossimo episodio
  if (h->state == HX_HEALTH_OK) {
    // niente: il beepPending è già false dopo pop, ma qui garantiamo pulizia
    h->beepPending = false;
  }
}