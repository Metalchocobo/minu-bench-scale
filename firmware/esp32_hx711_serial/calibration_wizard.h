#pragma once

#include <Arduino.h>

// =============================================================================
// CALIBRATION_WIZARD.H - Wizard calibrazione on-display per Minù Bench Scale
// =============================================================================
// Attivato da long press (3s) su TARE.
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
static const uint32_t LONG_PRESS_MS       = 3000;   // 3 secondi per attivare
static const uint16_t REF_WEIGHT_MIN_G    = 500;    // Peso minimo selezionabile
static const uint16_t REF_WEIGHT_MAX_G    = 20000;  // Peso massimo selezionabile
static const uint16_t REF_WEIGHT_STEP_G   = 500;    // Step sotto 2000g
static const uint16_t REF_WEIGHT_STEP_FINE_G = 50;  // Step sopra 2000g

// Soglie validazione CPG (count-per-gram)
static const float CPG_MIN = 50.0f;    // Minimo ragionevole
static const float CPG_MAX = 500.0f;   // Massimo ragionevole

// ========================= API =========================

// Ritorna true se il wizard è attivo
bool isActive();

// Ritorna lo step corrente
CalStep getStep();

// Aggiorna la state machine del wizard.
// Gestisce long press su TARE, transizioni tra step, acquisizione dati.
// Chiamare ogni ciclo loop() con nowMs = millis().
void update(uint32_t nowMs);

// Ritorna il valore peso di riferimento corrente (per UI)
uint16_t getRefWeightG();

// Ritorna true se il wizard richiede un render custom (impedisce renderWeight normale)
bool wantsCustomRender();

// Ritorna true se l'utente sta tenendo premuto TARE (long press in corso)
bool isLongPressInProgress();

// Ritorna progresso long press (0-100)
uint8_t getLongPressProgress(uint32_t nowMs);

// Ritorna progresso campionamento step corrente (0-100)
uint8_t getSampleProgress();

// Ritorna il CPG calcolato (valido solo in STEP_CONFIRM)
float getCalculatedCpg();

// Reset wizard allo stato IDLE (abort)
void abort();

} // namespace CalWizard
