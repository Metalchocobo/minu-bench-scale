#include "calibration_wizard.h"
#include "remote_console.h"
#include "scale_state.h"
#include "buzzer.h"
#include <math.h>

namespace CalWizard {

// ========================= STATO INTERNO =========================
static CalStep g_step = CAL_IDLE;
static uint32_t g_messageUntilMs = 0;

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
static const uint32_t SAVE_MESSAGE_MS = 2500;

// Anti-duplicati: traccia ultimo raw visto
static long g_lastSeenRaw = 0;
static bool g_lastSeenRawValid = false;

// Per STEP_PLACE: aspetta che il peso venga rilevato
static bool g_weightDetected = false;
static const long WEIGHT_DETECT_THRESHOLD = 50000;  // Differenza minima per rilevare peso

// ========================= UTILITY INTERNE =========================

static void resetSampling() {
  g_sampleAccum = 0;
  g_sampleCount = 0;
  g_sampleStartMs = millis();
  g_lastSeenRaw = 0;
  g_lastSeenRawValid = false;
  g_weightDetected = false;
}

static bool accumulateSample(uint32_t nowMs) {
  // Già abbastanza campioni? Non accumulare più!
  if (g_sampleCount >= SAMPLES_NEEDED) {
    return true;
  }

  if ((nowMs - g_sampleStartMs) < SAMPLE_SETTLE_MS) {
    return false;
  }

  long raw = ScaleState::getLastRawAvg();

  // Evita di accumulare lo stesso valore più volte
  if (g_lastSeenRawValid && raw == g_lastSeenRaw) {
    return false;
  }

  g_lastSeenRaw = raw;
  g_lastSeenRawValid = true;

  g_sampleAccum += raw;
  g_sampleCount++;

  // Debug: mostra ogni campione
  if (g_sampleCount <= 5 || g_sampleCount == SAMPLES_NEEDED) {
    ConsoleLog.print(F("[CAL] Sample #"));
    ConsoleLog.print(g_sampleCount);
    ConsoleLog.print(F(": raw="));
    ConsoleLog.print(raw);
    ConsoleLog.print(F(", accum="));
    ConsoleLog.println(g_sampleAccum);
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
  g_messageUntilMs = 0;
  g_longPressStartMs = 0;
  g_longPressActive = false;
  buzzerError();
  ConsoleLog.println(F("[CAL] Wizard annullato"));
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
        ConsoleLog.println(F("[CAL] ========================================"));
        ConsoleLog.println(F("[CAL] Wizard calibrazione avviato (SKIP 5s)"));
        ConsoleLog.println(F("[CAL] STEP 1/4: Piatto vuoto - premi ENTER"));
        ConsoleLog.println(F("[CAL] ========================================"));
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

void cancelLongPress() {
  if (g_step == CAL_IDLE) {
    g_longPressActive = false;
    g_longPressStartMs = 0;
  }
}

void update(uint32_t nowMs, KeyCode key) {
  // In IDLE non fa nulla - il long press è gestito da updateLongPress()
  if (g_step == CAL_IDLE) {
    return;
  }

  if (g_step == CAL_STEP_SAVED || g_step == CAL_STEP_SAVE_ERROR) {
    if (g_messageUntilMs != 0 && (int32_t)(nowMs - g_messageUntilMs) >= 0) {
      g_step = CAL_IDLE;
      g_messageUntilMs = 0;
    }
    return;
  }

  // ========== STEP_ZERO: Acquisisci zero ==========
  if (g_step == CAL_STEP_ZERO) {
    if (key == KEY_UNDO) {
      abort();
      return;
    }

    if (key == KEY_ENTER) {
      ConsoleLog.print(F("[CAL] ENTER premuto. Campioni: "));
      ConsoleLog.println(g_sampleCount);

      if (g_sampleCount >= SAMPLES_NEEDED) {
        g_zeroRaw = finalizeSample();
        ConsoleLog.print(F("[CAL] Zero raw = "));
        ConsoleLog.println(g_zeroRaw);
        g_step = CAL_STEP_PLACE;
        resetSampling();
        buzzerOk();
        ConsoleLog.println(F("[CAL] STEP 2/4: Appoggia peso riferimento - premi ENTER"));
      } else {
        buzzerWarn();
        ConsoleLog.println(F("[CAL] Attendi, campionamento in corso..."));
      }
      return;
    }

    accumulateSample(nowMs);
    return;
  }

  // ========== STEP_PLACE: Appoggia peso di riferimento ==========
  if (g_step == CAL_STEP_PLACE) {
    if (key == KEY_UNDO) {
      abort();
      return;
    }

    if (key == KEY_ENTER) {
      // Primo ENTER: inizia campionamento (peso stabile)
      if (!g_weightDetected) {
        long currentRaw = ScaleState::getLastRawAvg();
        long diff = currentRaw - g_zeroRaw;
        if (diff < 0) diff = -diff;

        if (diff < WEIGHT_DETECT_THRESHOLD) {
          buzzerError();
          ConsoleLog.println(F("[CAL] ERRORE: nessun peso rilevato! Appoggia il peso e riprova."));
          return;
        }

        // Peso presente, inizia campionamento
        g_weightDetected = true;
        g_sampleAccum = 0;
        g_sampleCount = 0;
        g_sampleStartMs = millis();
        g_lastSeenRaw = 0;
        g_lastSeenRawValid = false;
        buzzerOk();
        ConsoleLog.print(F("[CAL] Peso rilevato (raw="));
        ConsoleLog.print(currentRaw);
        ConsoleLog.println(F("). Campionamento..."));
        return;
      }

      // Secondo ENTER: finalizza se campionamento completo
      ConsoleLog.print(F("[CAL] ENTER premuto. Campioni: "));
      ConsoleLog.println(g_sampleCount);

      if (g_sampleCount >= SAMPLES_NEEDED) {
        g_refRaw = finalizeSample();

        long delta = g_refRaw - g_zeroRaw;
        ConsoleLog.print(F("[CAL] Ref raw = "));
        ConsoleLog.print(g_refRaw);
        ConsoleLog.print(F(", Zero raw = "));
        ConsoleLog.print(g_zeroRaw);
        ConsoleLog.print(F(", Delta = "));
        ConsoleLog.println(delta);

        if (delta < 0) delta = -delta;
        if (delta < 1000) {
          buzzerError();
          ConsoleLog.println(F("[CAL] ERRORE: differenza raw troppo piccola, peso non rilevato"));
          return;
        }

        g_step = CAL_STEP_VALUE;
        buzzerOk();
        ConsoleLog.println(F("[CAL] STEP 3/4: Seleziona peso (SKIP=+, TARE=-) - premi ENTER"));
      } else {
        buzzerWarn();
        ConsoleLog.println(F("[CAL] Attendi, campionamento in corso..."));
      }
      return;
    }

    // Campiona solo se peso già rilevato (dopo primo ENTER)
    if (g_weightDetected) {
      accumulateSample(nowMs);
    }
    return;
  }

  // ========== STEP_VALUE: Seleziona valore peso ==========
  if (g_step == CAL_STEP_VALUE) {
    if (key == KEY_UNDO) {
      abort();
      return;
    }

    if (key == KEY_SKIP) {
      g_refWeightG = stepWeight(g_refWeightG, +1);
      buzzerKeyClick();
      ConsoleLog.print(F("[CAL] Peso ref = "));
      ConsoleLog.print(g_refWeightG);
      ConsoleLog.println(F(" g"));
      return;
    }

    if (key == KEY_TARE) {
      g_refWeightG = stepWeight(g_refWeightG, -1);
      buzzerKeyClick();
      ConsoleLog.print(F("[CAL] Peso ref = "));
      ConsoleLog.print(g_refWeightG);
      ConsoleLog.println(F(" g"));
      return;
    }

    if (key == KEY_ENTER) {
      float cpg = (float)(g_refRaw - g_zeroRaw) / (float)g_refWeightG;

      ConsoleLog.print(F("[CAL] ENTER premuto. CPG calcolato = "));
      ConsoleLog.println(cpg, 4);
      ConsoleLog.print(F("[CAL]   g_zeroRaw = ")); ConsoleLog.println(g_zeroRaw);
      ConsoleLog.print(F("[CAL]   g_refRaw  = ")); ConsoleLog.println(g_refRaw);
      ConsoleLog.print(F("[CAL]   delta     = ")); ConsoleLog.println(g_refRaw - g_zeroRaw);
      ConsoleLog.print(F("[CAL]   peso      = ")); ConsoleLog.println(g_refWeightG);
      ConsoleLog.print(F("[CAL] Range valido: "));
      ConsoleLog.print(CPG_MIN, 0);
      ConsoleLog.print(F(" - "));
      ConsoleLog.println(CPG_MAX, 0);

      if (!validateCpg(cpg)) {
        buzzerError();
        ConsoleLog.print(F("[CAL] ERRORE: CPG "));
        ConsoleLog.print(cpg, 4);
        ConsoleLog.println(F(" fuori range!"));
        ConsoleLog.println(F("[CAL] Controlla: peso corretto? Bilancia stabile?"));
        return;
      }

      g_step = CAL_STEP_CONFIRM;
      buzzerOk();
      ConsoleLog.println(F("[CAL] STEP 4/4: Conferma (ENTER=salva, UNDO=annulla)"));
      return;
    }

    return;
  }

  // ========== STEP_CONFIRM: Conferma e salva ==========
  if (g_step == CAL_STEP_CONFIRM) {
    if (key == KEY_UNDO) {
      abort();
      return;
    }

    if (key == KEY_ENTER) {
      long delta = g_refRaw - g_zeroRaw;
      float cpg = (float)delta / (float)g_refWeightG;

      ConsoleLog.println(F("[CAL] ========================================"));
      ConsoleLog.println(F("[CAL] CALCOLO CPG:"));
      ConsoleLog.print(F("[CAL]   Zero raw  = ")); ConsoleLog.println(g_zeroRaw);
      ConsoleLog.print(F("[CAL]   Ref raw   = ")); ConsoleLog.println(g_refRaw);
      ConsoleLog.print(F("[CAL]   Delta     = ")); ConsoleLog.println(delta);
      ConsoleLog.print(F("[CAL]   Peso ref  = ")); ConsoleLog.print(g_refWeightG); ConsoleLog.println(F(" g"));
      ConsoleLog.print(F("[CAL]   CPG       = ")); ConsoleLog.print(delta); ConsoleLog.print(F(" / "));
      ConsoleLog.print(g_refWeightG); ConsoleLog.print(F(" = ")); ConsoleLog.println(cpg, 4);
      ConsoleLog.println(F("[CAL] ========================================"));

      if (!validateCpg(cpg)) {
        buzzerError();
        ConsoleLog.print(F("[CAL] ERRORE: CPG ")); ConsoleLog.print(cpg, 4);
        ConsoleLog.print(F(" fuori range ")); ConsoleLog.print(CPG_MIN, 0);
        ConsoleLog.print(F("-")); ConsoleLog.println(CPG_MAX, 0);
        abort();
        return;
      }

      long expectedOffset = g_zeroRaw;
      float expectedCpg = cpg;

      ScaleState::setCalibrationZeroRaw(expectedOffset);
      ScaleState::setScaleCpg(cpg);
      ScaleState::setZtCounts(0);

      bool saved = ScaleState::saveToNVS();
      bool verified = false;
      if (saved) {
        ScaleState::loadFromNVS();
        verified =
          (ScaleState::getOffsetRaw() == expectedOffset) &&
          (fabsf(ScaleState::getScaleCpg() - expectedCpg) <= 0.01f);
      }
      ScaleState::setZtCounts(0);
      ScaleState::resetFiltersAndState();

      if (saved && verified) {
        g_step = CAL_STEP_SAVED;
        g_messageUntilMs = nowMs + SAVE_MESSAGE_MS;
        buzzerOk();
        delay(100);
        buzzerOk();
      } else {
        g_step = CAL_STEP_SAVE_ERROR;
        g_messageUntilMs = nowMs + SAVE_MESSAGE_MS;
        buzzerError();
      }

      // Verifica valori salvati
      ConsoleLog.println((saved && verified) ? F("[CAL] CALIBRAZIONE SALVATA - VERIFICA:") : F("[CAL] ERRORE SALVATAGGIO - VERIFICA:"));
      ConsoleLog.print(F("[CAL]   Offset salvato = ")); ConsoleLog.println(ScaleState::getOffsetRaw());
      ConsoleLog.print(F("[CAL]   CPG salvato    = ")); ConsoleLog.println(ScaleState::getScaleCpg(), 4);
      ConsoleLog.print(F("[CAL]   ZT counts      = ")); ConsoleLog.println(ScaleState::getZtCounts());

      return;
    }

    return;
  }
}

} // namespace CalWizard
