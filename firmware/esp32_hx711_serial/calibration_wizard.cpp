#include "calibration_wizard.h"
#include "keypad.h"
#include "scale_state.h"
#include "buzzer.h"
#include <math.h>

namespace CalWizard {

// ========================= STATO INTERNO =========================
static CalStep g_step = CAL_IDLE;

// Long press tracking
static bool     g_longPressStarted = false;
static uint32_t g_longPressStartMs = 0;

// Dati acquisizione
static long g_zeroRaw      = 0;          // Raw a vuoto (step ZERO)
static long g_refRaw       = 0;          // Raw con peso (step PLACE)
static uint16_t g_refWeightG = 2000;     // Peso riferimento selezionato

// Campionamento per acquisizione stabile
static long     g_sampleAccum = 0;
static uint8_t  g_sampleCount = 0;
static const uint8_t SAMPLES_NEEDED = 32;
static uint32_t g_sampleStartMs = 0;
static const uint32_t SAMPLE_SETTLE_MS = 300;  // Ignora primi 300ms

// ========================= UTILITY INTERNE =========================

static void resetSampling() {
  g_sampleAccum = 0;
  g_sampleCount = 0;
  g_sampleStartMs = millis();
}

static bool accumulateSample(uint32_t nowMs) {
  // Aspetta settle time
  if ((nowMs - g_sampleStartMs) < SAMPLE_SETTLE_MS) {
    return false;
  }

  long raw = ScaleState::getLastRawAvg();
  g_sampleAccum += raw;
  g_sampleCount++;

  return (g_sampleCount >= SAMPLES_NEEDED);
}

static long finalizeSample() {
  if (g_sampleCount == 0) return 0;
  return g_sampleAccum / g_sampleCount;
}

static uint16_t stepWeight(uint16_t current, int16_t direction) {
  // Sotto 2000g: step di 500g
  // Sopra 2000g: step di 50g
  int16_t step = (current >= 2000) ? REF_WEIGHT_STEP_FINE_G : REF_WEIGHT_STEP_G;

  int32_t newVal = (int32_t)current + direction * step;

  // Clamp
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
  return (g_step != CAL_IDLE);
}

bool isLongPressInProgress() {
  return (g_step == CAL_IDLE && g_longPressStarted);
}

uint8_t getLongPressProgress(uint32_t nowMs) {
  if (!g_longPressStarted || g_step != CAL_IDLE) return 0;
  uint32_t elapsed = nowMs - g_longPressStartMs;
  if (elapsed >= LONG_PRESS_MS) return 100;
  return (uint8_t)((elapsed * 100UL) / LONG_PRESS_MS);
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
  g_longPressStarted = false;
  buzzerError();
  Serial.println(F("[CAL] Wizard annullato"));
}

void update(uint32_t nowMs) {
  // ========== GESTIONE LONG PRESS (solo in IDLE) ==========
  if (g_step == CAL_IDLE) {
    bool tarePressed = keypad_is_pressed(KEY_TARE);

    if (tarePressed && !g_longPressStarted) {
      // Inizia tracking long press
      g_longPressStarted = true;
      g_longPressStartMs = nowMs;
    }
    else if (tarePressed && g_longPressStarted) {
      // Controlla durata
      if ((nowMs - g_longPressStartMs) >= LONG_PRESS_MS) {
        // Long press completato: avvia wizard
        g_step = CAL_STEP_ZERO;
        g_longPressStarted = false;
        resetSampling();
        buzzerOk();
        Serial.println(F("[CAL] Wizard avviato - STEP_ZERO"));
      }
    }
    else if (!tarePressed) {
      // Rilasciato prima del tempo
      g_longPressStarted = false;
    }

    return;
  }

  // ========== STEP_ZERO: Acquisisci zero ==========
  if (g_step == CAL_STEP_ZERO) {
    // CLEAR = annulla
    KeyCode ev = keypad_get_event();
    if (ev == KEY_CLEAR) {
      abort();
      return;
    }

    // ENTER = conferma zero
    if (ev == KEY_ENTER) {
      if (g_sampleCount >= SAMPLES_NEEDED) {
        g_zeroRaw = finalizeSample();
        g_step = CAL_STEP_PLACE;
        resetSampling();
        buzzerOk();
        Serial.print(F("[CAL] Zero raw = "));
        Serial.println(g_zeroRaw);
        Serial.println(F("[CAL] STEP_PLACE - Appoggia peso riferimento"));
      } else {
        // Non abbastanza campioni
        buzzerWarn();
      }
      return;
    }

    // Accumula campioni in background
    accumulateSample(nowMs);
    return;
  }

  // ========== STEP_PLACE: Appoggia peso di riferimento ==========
  if (g_step == CAL_STEP_PLACE) {
    KeyCode ev = keypad_get_event();

    if (ev == KEY_CLEAR) {
      abort();
      return;
    }

    // ENTER = conferma peso appoggiato
    if (ev == KEY_ENTER) {
      if (g_sampleCount >= SAMPLES_NEEDED) {
        g_refRaw = finalizeSample();

        // Verifica che il raw sia diverso dallo zero
        long delta = g_refRaw - g_zeroRaw;
        if (delta < 0) delta = -delta;

        if (delta < 1000) {
          // Differenza troppo piccola, probabilmente non c'è peso
          buzzerError();
          Serial.println(F("[CAL] ERRORE: differenza raw troppo piccola"));
          return;
        }

        g_step = CAL_STEP_VALUE;
        buzzerOk();
        Serial.print(F("[CAL] Ref raw = "));
        Serial.println(g_refRaw);
        Serial.println(F("[CAL] STEP_VALUE - Seleziona peso in grammi"));
      } else {
        buzzerWarn();
      }
      return;
    }

    // Accumula campioni
    accumulateSample(nowMs);
    return;
  }

  // ========== STEP_VALUE: Seleziona valore peso ==========
  if (g_step == CAL_STEP_VALUE) {
    KeyCode ev = keypad_get_event();

    if (ev == KEY_CLEAR) {
      abort();
      return;
    }

    // SKIP = aumenta peso
    if (ev == KEY_SKIP) {
      g_refWeightG = stepWeight(g_refWeightG, +1);
      buzzerKeyClick();
      Serial.print(F("[CAL] Peso ref = "));
      Serial.println(g_refWeightG);
      return;
    }

    // TARE = diminuisci peso
    if (ev == KEY_TARE) {
      g_refWeightG = stepWeight(g_refWeightG, -1);
      buzzerKeyClick();
      Serial.print(F("[CAL] Peso ref = "));
      Serial.println(g_refWeightG);
      return;
    }

    // ENTER = conferma e vai a CONFIRM
    if (ev == KEY_ENTER) {
      // Calcola CPG preliminare per verifica
      float cpg = (float)(g_refRaw - g_zeroRaw) / (float)g_refWeightG;

      if (!validateCpg(cpg)) {
        buzzerError();
        Serial.print(F("[CAL] ERRORE: CPG fuori range = "));
        Serial.println(cpg, 4);
        return;
      }

      g_step = CAL_STEP_CONFIRM;
      buzzerOk();
      Serial.print(F("[CAL] CPG calcolato = "));
      Serial.println(cpg, 4);
      Serial.println(F("[CAL] STEP_CONFIRM - ENTER=salva, CLEAR=annulla"));
      return;
    }

    return;
  }

  // ========== STEP_CONFIRM: Conferma e salva ==========
  if (g_step == CAL_STEP_CONFIRM) {
    KeyCode ev = keypad_get_event();

    if (ev == KEY_CLEAR) {
      abort();
      return;
    }

    // ENTER = salva calibrazione
    if (ev == KEY_ENTER) {
      float cpg = (float)(g_refRaw - g_zeroRaw) / (float)g_refWeightG;

      // Double check validazione
      if (!validateCpg(cpg)) {
        buzzerError();
        Serial.println(F("[CAL] ERRORE: CPG non valido, calibrazione annullata"));
        abort();
        return;
      }

      // Applica calibrazione
      ScaleState::setOffsetRaw(g_zeroRaw);
      ScaleState::setScaleCpg(cpg);
      ScaleState::setZtCounts(0);  // Reset zero-tracking
      ScaleState::saveToNVS();
      ScaleState::resetFiltersAndState();

      g_step = CAL_IDLE;
      buzzerOk();
      delay(100);
      buzzerOk();  // Doppio beep = successo

      Serial.println(F("[CAL] ===== CALIBRAZIONE COMPLETATA ====="));
      Serial.print(F("[CAL] OFFSET = "));
      Serial.println(g_zeroRaw);
      Serial.print(F("[CAL] CPG = "));
      Serial.println(cpg, 4);
      Serial.print(F("[CAL] REF_G = "));
      Serial.println(g_refWeightG);

      return;
    }

    return;
  }
}

} // namespace CalWizard
