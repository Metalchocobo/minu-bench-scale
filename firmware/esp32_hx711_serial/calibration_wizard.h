#pragma once

#include <Arduino.h>
#include "keypad.h"

// =============================================================================
// CALIBRATION_WIZARD.H - Wizard calibrazione on-display per Minù Bench Scale
// =============================================================================
// Attivato premendo CLEAR, poi TARE entro 1 secondo.
// 4 step: ZERO -> PLACE -> VALUE -> CONFIRM
// Non bloccante: state machine gestita da update().

namespace CalWizard {

// ========================= STEP CALIBRAZIONE =========================
enum CalStep : uint8_t {
  CAL_IDLE = 0,       // Wizard non attivo
  CAL_STEP_ZERO,      // Acquisisci zero (piatto vuoto)
  CAL_STEP_PLACE,     // Appoggia peso di riferimento
  CAL_STEP_VALUE,     // Seleziona valore peso (500..20000 g)
  CAL_STEP_CONFIRM    // Conferma e salva
};

// ========================= CONFIG =========================
static const uint16_t REF_WEIGHT_MIN_G    = 500;    // Peso minimo selezionabile
static const uint16_t REF_WEIGHT_MAX_G    = 20000;  // Peso massimo selezionabile
static const uint16_t REF_WEIGHT_STEP_G   = 500;    // Step sotto 2000g
static const uint16_t REF_WEIGHT_STEP_FINE_G = 50;  // Step sopra 2000g

// Soglie validazione CPG (count-per-gram)
static const float CPG_MIN = 20.0f;     // Minimo ragionevole
static const float CPG_MAX = 1000.0f;   // Massimo ragionevole

// Finestra temporale per combo CLEAR+TARE
static const uint32_t COMBO_WINDOW_MS = 1000;

// ========================= API =========================

// Ritorna true se il wizard è attivo
bool isActive();

// Ritorna lo step corrente
CalStep getStep();

// Controlla se il tasto fa parte della combo per avviare il wizard.
// Chiamare dal loop principale PRIMA di gestire il tasto.
// Ritorna true se il wizard è stato avviato (il tasto non va gestito altrove).
bool checkComboAndStart(KeyCode key, uint32_t nowMs);

// Aggiorna la state machine del wizard (solo quando attivo).
// Chiamare ogni ciclo loop() con nowMs = millis().
void update(uint32_t nowMs);

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
