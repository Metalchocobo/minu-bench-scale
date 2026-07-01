#pragma once

#include <Arduino.h>
#include "config/config_scale.h"

// =============================================================================
// SCALE_STATE.H - Stati bilancia, Zero-Tracking, Tara, Display
// =============================================================================

namespace ScaleState {

// ========================= STATI PRINCIPALI =========================
enum WeighState : uint8_t {
  STATE_STABLE = 0,
  STATE_UNSTABLE
};

enum TareMode : uint8_t {
  TARE_MODE_MANUAL = 0,
  TARE_MODE_AUTO
};

enum TareResult : uint8_t {
  TARE_RESULT_IDLE = 0,
  TARE_RESULT_PENDING,
  TARE_RESULT_APPLIED,
  TARE_RESULT_FAILED
};

enum TareUiState : uint8_t {
  TARE_UI_IDLE = 0,
  TARE_UI_SETTLING,
  TARE_UI_OK,
  TARE_UI_FAILED
};

// ========================= GETTERS/SETTERS GLOBALI =========================
// Offset e scale (calibrazione)
void   setOffsetRaw(long offset);
long   getOffsetRaw();
void   setScaleCpg(float cpg);
float  getScaleCpg();

// Zero-tracking counts (compensazione dinamica)
void   setZtCounts(long counts);
long   getZtCounts();
long   effectiveOffsetCounts();  // OFFSET_RAW + zero_track_counts

// Stato bilancia
void   setWeighState(WeighState st);
WeighState getWeighState();

// Abilitazioni
void   setStEnable(bool en);
bool   isStEnabled();
void   setZtEnable(bool en);
bool   isZtEnabled();

// Grams latched (per STABLE)
void   setGramsLatch(float g);
float  getGramsLatch();

// Display
void   setDispUnstable(float g);
float  getDispUnstable();
void   setDispInit(bool v);
bool   isDispInit();

// Ultimo display work/live
void   setDispWorkLast(long g);
long   getDispWorkLast();
void   setDispLiveLast(long g);
long   getDispLiveLast();
void   setDispLiveHalfLast(long gX2);
long   getDispLiveHalfLast();
long   halfToRoundedGram(long gX2);
const char* getStateLabelWorkLast();
void   setStateLabelWorkLast(const char* label);

// Ultimo valore raw filtrato (per calibrazione wizard)
void   setLastRawAvg(long raw);
long   getLastRawAvg();

// ========================= DISPLAY QUANTIZZAZIONE =========================
// Quantizza g con banda zero + isteresi (LIVE)
long displayLiveStable(float gIn, long prevDisp, uint32_t nowMs);
long displayLiveHalfStable(float gIn, long prevHalf, uint32_t nowMs);
long displayLiveQuant(float gIn, long prevDisp);

// Quantizza g con banda zero + isteresi (WORK)
long displayWorkQuant(float gIn, long prevDisp);

// ========================= ZERO-TRACKING =========================
// Micro-ZT lento (tick periodico)
void applyZtStepTowardZero(float gLive);

// Check "quiete" per ZT
bool isQuietForZt();

// Snap-to-zero su scarico
bool trySnapOnUnload(float gLive, float slopeGps, bool quietNow);

// ========================= TARA =========================
// Accumula campioni per tara
void tareAccumSample(long rawUse);

// Avvia raccolta tara
void tareStart(uint32_t nowMs, TareMode mode = TARE_MODE_MANUAL);

// Aggiorna/applica tara. La tara cambia offset solo se la finestra e' stabile.
TareResult tareUpdate(uint32_t nowMs);

// Compat: true solo quando la tara viene applicata con successo.
bool tareMaybeApply(uint32_t nowMs);

// UI tara attiva?
bool isTareUiActive();
void setTareUiActive(bool v);
uint32_t getTareUiStartMs();
void setTareUiStartMs(uint32_t ms);
uint32_t getTareUiEndMs();
void setTareUiEndMs(uint32_t ms);
TareMode getTareMode();
TareUiState getTareUiState();

// ========================= INATTIVITÀ =========================
// Nota attività peso (reset timer se variazione > soglia)
void inactivityNoteWeightActivity(uint32_t nowMs, long gDispNow);

// Ritorna true una sola volta se c'è stata variazione peso (pattern one-shot)
bool popWeightActivity();

// ========================= RESET =========================
void resetFiltersAndState();

// ========================= PERSISTENZA NVS =========================
void loadFromNVS();
bool saveToNVS();

// ========================= MODALITÀ PRESET =========================
// Imposta preset: "work" o "live"
void setMode(const char* mode);

} // namespace ScaleState
