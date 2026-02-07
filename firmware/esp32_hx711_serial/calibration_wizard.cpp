#include "calibration_wizard.h"
#include "scale_state.h"
#include "buzzer.h"
#include <math.h>

namespace CalWizard {

// ========================= STATO INTERNO =========================
static CalStep g_step = CAL_IDLE;

// Long press tracking: SKIP tenuto per 5 secondi
static uint32_t g_longPressStartMs = 0;
static bool g_longPressActive = false;

// Dati acquisizione
static long g_zeroRaw      = 0;          // Raw a vuoto (step ZERO)
static long g_refRaw       = 0;          // Raw con peso (step PLACE)
static uint16_t g_refWeightG = 2000;     // Peso riferimento selezionato

// Campionamento per acquisizione stabile
static long     g_sampleAccum = 0;
static uint8_t  g_sampleCount = 0;
static const uint8_t SAMPLES_NEEDED = 32;
static uint32_t g_sampleStartMs = 0;
static const uint32_t SAMPLE_SETTLE_MS = 300;

// Anti-duplicati: traccia ultimo raw visto
static long g_lastSeenRaw = 0;
static bool g_lastSeenRawValid = false;

// ========================= UTILITY INTERNE =========================

static void resetSampling() {
  g_sampleAccum = 0;
  g_sampleCount = 0;
  g_sampleStartMs = millis();
  g_lastSeenRaw = 0;
  g_lastSeenRawValid = false;
}

static bool accumulateSample(uint32_t nowMs) {
  if ((nowMs - g_sampleStartMs) < SAMPLE_SETTLE_MS) {
    return false;
  }

  long raw = ScaleState::getLastRawAvg();

  // Evita di accumulare lo stesso valore più volte
  if (g_lastSeenRawValid && raw == g_lastSeenRaw) {
    return (g_sampleCount >= SAMPLES_NEEDED);
  }

  g_lastSeenRaw = raw;
  g_lastSeenRawValid = true;

  g_sampleAccum += raw;
  g_sampleCount++;

  // Debug: mostra ogni campione
  if (g_sampleCount <= 5 || g_sampleCount == SAMPLES_NEEDED) {
    Serial.print(F("[CAL] Sample #"));
    Serial.print(g_sampleCount);
    Serial.print(F(": raw="));
    Serial.print(raw);
    Serial.print(F(", accum="));
    Serial.println(g_sampleAccum);
  }

  return (g_sampleCount >= SAMPLES_NEEDED);
}

static long finalizeSample() {
  if (g_sampleCount == 0) return 0;
  return g_sampleAccum / g_sampleCount;
}

static uint16_t stepWeight(uint16_t current, int16_t direction) {
  int16_t step = (current >= 2000) ? REF_WEIGHT_STEP_FINE_G : REF_WEIGHT_STEP_G;
  int32_t newVal = (int32_t)current + direction * step;

  if (newVal < REF_WEIGHT_MIN_G) newVal = REF_WEIGHT_MIN_G;
  if (newVal > REF_WEIGHT_MAX_G) newVal = REF_WEIGHT_MAX_G;

  return (uint16_t)newVal;
}

static bool validateCpg(float cpg) {
  return (cpg >= CPG_MIN && cpg <= CPG_MAX && !isnan(cpg) && !isinf(cpg));
}

// ========================= API PUBBLICHE =========================

bool isActive() {
  return (g_step != CAL_IDLE);
}

CalStep getStep() {
  return g_step;
}

uint16_t getRefWeightG() {
  return g_refWeightG;
}

bool wantsCustomRender() {
  if (g_step != CAL_IDLE) return true;
  // Mostra UI long press solo dopo LONG_PRESS_SHOW_UI_MS (2 sec)
  if (g_longPressActive && g_longPressStartMs != 0) {
    uint32_t elapsed = millis() - g_longPressStartMs;
    if (elapsed >= LONG_PRESS_SHOW_UI_MS) return true;
  }
  return false;
}

uint8_t getSampleProgress() {
  if (g_step != CAL_STEP_ZERO && g_step != CAL_STEP_PLACE) return 0;
  if (g_sampleCount >= SAMPLES_NEEDED) return 100;
  return (uint8_t)((g_sampleCount * 100UL) / SAMPLES_NEEDED);
}

float getCalculatedCpg() {
  if (g_refWeightG == 0) return 0.0f;
  return (float)(g_refRaw - g_zeroRaw) / (float)g_refWeightG;
}

void abort() {
  g_step = CAL_IDLE;
  g_longPressStartMs = 0;
  g_longPressActive = false;
  buzzerError();
  Serial.println(F("[CAL] Wizard annullato"));
}

// ========================= LONG PRESS SU SKIP =========================

bool isLongPressInProgress() {
  // Ritorna true solo dopo LONG_PRESS_SHOW_UI_MS (2 sec)
  if (!g_longPressActive || g_longPressStartMs == 0) return false;
  uint32_t elapsed = millis() - g_longPressStartMs;
  return (elapsed >= LONG_PRESS_SHOW_UI_MS);
}

uint8_t getLongPressProgress(uint32_t nowMs) {
  if (!g_longPressActive || g_longPressStartMs == 0) return 0;
  uint32_t elapsed = nowMs - g_longPressStartMs;
  if (elapsed < LONG_PRESS_SHOW_UI_MS) return 0;
  if (elapsed >= LONG_PRESS_MS) return 100;
  // Progresso da 0% (a 2s) a 100% (a 5s)
  uint32_t visibleElapsed = elapsed - LONG_PRESS_SHOW_UI_MS;
  uint32_t visibleDuration = LONG_PRESS_MS - LONG_PRESS_SHOW_UI_MS;
  return (uint8_t)((visibleElapsed * 100UL) / visibleDuration);
}

bool updateLongPress(uint32_t nowMs) {
  // Solo quando wizard non è attivo
  if (g_step != CAL_IDLE) {
    g_longPressActive = false;
    g_longPressStartMs = 0;
    return false;
  }

  bool skipPressed = keypad_is_pressed(KEY_SKIP);

  if (skipPressed) {
    if (!g_longPressActive) {
      // Inizio long press
      g_longPressActive = true;
      g_longPressStartMs = nowMs;
    } else {
      // Long press in corso: controlla se completato
      uint32_t elapsed = nowMs - g_longPressStartMs;
      if (elapsed >= LONG_PRESS_MS) {
        // Long press completato! Avvia wizard
        g_step = CAL_STEP_ZERO;
        g_longPressActive = false;
        g_longPressStartMs = 0;
        resetSampling();
        buzzerOk();
        Serial.println(F("[CAL] ========================================"));
        Serial.println(F("[CAL] Wizard calibrazione avviato (SKIP 5s)"));
        Serial.println(F("[CAL] STEP 1/4: Piatto vuoto - premi ENTER"));
        Serial.println(F("[CAL] ========================================"));
        return true;
      }
    }
  } else {
    // SKIP rilasciato: reset
    if (g_longPressActive) {
      g_longPressActive = false;
      g_longPressStartMs = 0;
    }
  }

  return false;
}

void update(uint32_t nowMs, KeyCode key) {
  // In IDLE non fa nulla - il long press è gestito da updateLongPress()
  if (g_step == CAL_IDLE) {
    return;
  }

  // ========== STEP_ZERO: Acquisisci zero ==========
  if (g_step == CAL_STEP_ZERO) {
    if (key == KEY_CLEAR) {
      abort();
      return;
    }

    if (key == KEY_ENTER) {
      Serial.print(F("[CAL] ENTER premuto. Campioni: "));
      Serial.println(g_sampleCount);

      if (g_sampleCount >= SAMPLES_NEEDED) {
        g_zeroRaw = finalizeSample();
        Serial.print(F("[CAL] Zero raw = "));
        Serial.println(g_zeroRaw);
        g_step = CAL_STEP_PLACE;
        resetSampling();
        buzzerOk();
        Serial.println(F("[CAL] STEP 2/4: Appoggia peso riferimento - premi ENTER"));
      } else {
        buzzerWarn();
        Serial.println(F("[CAL] Attendi, campionamento in corso..."));
      }
      return;
    }

    accumulateSample(nowMs);
    return;
  }

  // ========== STEP_PLACE: Appoggia peso di riferimento ==========
  if (g_step == CAL_STEP_PLACE) {
    if (key == KEY_CLEAR) {
      abort();
      return;
    }

    if (key == KEY_ENTER) {
      Serial.print(F("[CAL] ENTER premuto. Campioni: "));
      Serial.println(g_sampleCount);

      if (g_sampleCount >= SAMPLES_NEEDED) {
        g_refRaw = finalizeSample();

        long delta = g_refRaw - g_zeroRaw;
        Serial.print(F("[CAL] Ref raw = "));
        Serial.print(g_refRaw);
        Serial.print(F(", Zero raw = "));
        Serial.print(g_zeroRaw);
        Serial.print(F(", Delta = "));
        Serial.println(delta);

        if (delta < 0) delta = -delta;
        if (delta < 1000) {
          buzzerError();
          Serial.println(F("[CAL] ERRORE: differenza raw troppo piccola, peso non rilevato"));
          return;
        }

        g_step = CAL_STEP_VALUE;
        buzzerOk();
        Serial.println(F("[CAL] STEP 3/4: Seleziona peso (SKIP=+, TARE=-) - premi ENTER"));
      } else {
        buzzerWarn();
        Serial.println(F("[CAL] Attendi, campionamento in corso..."));
      }
      return;
    }

    accumulateSample(nowMs);
    return;
  }

  // ========== STEP_VALUE: Seleziona valore peso ==========
  if (g_step == CAL_STEP_VALUE) {
    if (key == KEY_CLEAR) {
      abort();
      return;
    }

    if (key == KEY_SKIP) {
      g_refWeightG = stepWeight(g_refWeightG, +1);
      buzzerKeyClick();
      Serial.print(F("[CAL] Peso ref = "));
      Serial.print(g_refWeightG);
      Serial.println(F(" g"));
      return;
    }

    if (key == KEY_TARE) {
      g_refWeightG = stepWeight(g_refWeightG, -1);
      buzzerKeyClick();
      Serial.print(F("[CAL] Peso ref = "));
      Serial.print(g_refWeightG);
      Serial.println(F(" g"));
      return;
    }

    if (key == KEY_ENTER) {
      float cpg = (float)(g_refRaw - g_zeroRaw) / (float)g_refWeightG;

      Serial.print(F("[CAL] ENTER premuto. CPG calcolato = "));
      Serial.println(cpg, 4);
      Serial.print(F("[CAL]   g_zeroRaw = ")); Serial.println(g_zeroRaw);
      Serial.print(F("[CAL]   g_refRaw  = ")); Serial.println(g_refRaw);
      Serial.print(F("[CAL]   delta     = ")); Serial.println(g_refRaw - g_zeroRaw);
      Serial.print(F("[CAL]   peso      = ")); Serial.println(g_refWeightG);
      Serial.print(F("[CAL] Range valido: "));
      Serial.print(CPG_MIN, 0);
      Serial.print(F(" - "));
      Serial.println(CPG_MAX, 0);

      if (!validateCpg(cpg)) {
        buzzerError();
        Serial.print(F("[CAL] ERRORE: CPG "));
        Serial.print(cpg, 4);
        Serial.println(F(" fuori range!"));
        Serial.println(F("[CAL] Controlla: peso corretto? Bilancia stabile?"));
        return;
      }

      g_step = CAL_STEP_CONFIRM;
      buzzerOk();
      Serial.println(F("[CAL] STEP 4/4: Conferma (ENTER=salva, CLEAR=annulla)"));
      return;
    }

    return;
  }

  // ========== STEP_CONFIRM: Conferma e salva ==========
  if (g_step == CAL_STEP_CONFIRM) {
    if (key == KEY_CLEAR) {
      abort();
      return;
    }

    if (key == KEY_ENTER) {
      long delta = g_refRaw - g_zeroRaw;
      float cpg = (float)delta / (float)g_refWeightG;

      Serial.println(F("[CAL] ========================================"));
      Serial.println(F("[CAL] CALCOLO CPG:"));
      Serial.print(F("[CAL]   Zero raw  = ")); Serial.println(g_zeroRaw);
      Serial.print(F("[CAL]   Ref raw   = ")); Serial.println(g_refRaw);
      Serial.print(F("[CAL]   Delta     = ")); Serial.println(delta);
      Serial.print(F("[CAL]   Peso ref  = ")); Serial.print(g_refWeightG); Serial.println(F(" g"));
      Serial.print(F("[CAL]   CPG       = ")); Serial.print(delta); Serial.print(F(" / "));
      Serial.print(g_refWeightG); Serial.print(F(" = ")); Serial.println(cpg, 4);
      Serial.println(F("[CAL] ========================================"));

      if (!validateCpg(cpg)) {
        buzzerError();
        Serial.print(F("[CAL] ERRORE: CPG ")); Serial.print(cpg, 4);
        Serial.print(F(" fuori range ")); Serial.print(CPG_MIN, 0);
        Serial.print(F("-")); Serial.println(CPG_MAX, 0);
        abort();
        return;
      }

      ScaleState::setOffsetRaw(g_zeroRaw);
      ScaleState::setScaleCpg(cpg);
      ScaleState::setZtCounts(0);
      ScaleState::saveToNVS();
      ScaleState::resetFiltersAndState();

      g_step = CAL_IDLE;
      buzzerOk();
      delay(100);
      buzzerOk();

      // Verifica valori salvati
      Serial.println(F("[CAL] CALIBRAZIONE SALVATA - VERIFICA:"));
      Serial.print(F("[CAL]   Offset salvato = ")); Serial.println(ScaleState::getOffsetRaw());
      Serial.print(F("[CAL]   CPG salvato    = ")); Serial.println(ScaleState::getScaleCpg(), 4);
      Serial.print(F("[CAL]   ZT counts      = ")); Serial.println(ScaleState::getZtCounts());

      return;
    }

    return;
  }
}

} // namespace CalWizard
