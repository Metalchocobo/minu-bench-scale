#pragma once

#include <Arduino.h>
#include "keypad.h"

// =============================================================================
// CALIBRATION_WIZARD.H - Wizard calibrazione on-display per Minù Bench Scale
// =============================================================================
// Attivato tenendo premuto SKIP per 5 secondi.
// 4 step: ZERO -> PLACE -> VALUE -> CONFIRM
// Non bloccante: state machine gestita da update().

namespace CalWizard {

// ========================= STEP CALIBRAZIONE =========================
enum CalStep : uint8_t {
  CAL_IDLE = 0,       // Wizard non attivo
  CAL_STEP_ZERO,      // Acquisisci zero (piatto vuoto)
  CAL_STEP_PLACE,     // Appoggia peso di riferimento
  CAL_STEP_VALUE,     // Seleziona valore peso (500..20000 g)
  CAL_STEP_CONFIRM,   // Conferma e salva
  CAL_STEP_SAVED,     // Salvataggio confermato
  CAL_STEP_SAVE_ERROR // Errore salvataggio/verifica
};

// ========================= CONFIG =========================
static const uint16_t REF_WEIGHT_MIN_G    = 500;    // Peso minimo selezionabile
static const uint16_t REF_WEIGHT_MAX_G    = 20000;  // Peso massimo selezionabile
static const uint16_t REF_WEIGHT_STEP_G   = 500;    // Step sotto 2000g
static const uint16_t REF_WEIGHT_STEP_FINE_G = 50;  // Step sopra 2000g

// Soglie validazione CPG (count-per-gram)
static const float CPG_MIN = 20.0f;     // Minimo ragionevole
static const float CPG_MAX = 1000.0f;   // Massimo ragionevole

// Tempo di pressione SKIP per avviare wizard (5 secondi)
static const uint32_t LONG_PRESS_MS = 5000;

// Tempo minimo prima di mostrare l'UI del long press (2 secondi)
static const uint32_t LONG_PRESS_SHOW_UI_MS = 2000;

// ========================= API =========================

// Ritorna true se il wizard è attivo
bool isActive();

// Ritorna lo step corrente
CalStep getStep();

// Aggiorna il tracking del long press su SKIP.
// Chiamare ogni ciclo loop() con nowMs = millis().
// Ritorna true se il wizard è stato appena avviato.
bool updateLongPress(uint32_t nowMs);

// Ritorna true se il long press su SKIP è in corso
bool isLongPressInProgress();

// Ritorna il progresso del long press (0-100)
uint8_t getLongPressProgress(uint32_t nowMs);

// Aggiorna la state machine del wizard (solo quando attivo).
// Chiamare ogni ciclo loop() con nowMs = millis() e l'evento tastiera.
// Il wizard gestisce i tasti internamente quando attivo.
void update(uint32_t nowMs, KeyCode key);

// Ritorna il valore peso di riferimento corrente (per UI)
uint16_t getRefWeightG();

// Ritorna true se il wizard richiede un render custom
bool wantsCustomRender();

// Ritorna progresso campionamento step corrente (0-100)
uint8_t getSampleProgress();

// Ritorna il CPG calcolato (valido solo in STEP_CONFIRM)
float getCalculatedCpg();

// Reset wizard allo stato IDLE (abort)
void abort();

} // namespace CalWizard
