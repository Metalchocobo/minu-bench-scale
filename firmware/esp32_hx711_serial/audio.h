#pragma once

#include <Arduino.h>
#include "config/config_pins.h"
#include "config/config_audio.h"
#include "dfplayer_driver.h"

// =============================================================================
// AUDIO.H - Modulo audio DFPlayer con namespace Audio::
// =============================================================================

namespace Audio {

// ========================= STATI =========================
enum State : uint8_t {
  IDLE = 0,
  GAP,
  POWERING,
  STARTING,
  PLAYING,
  TAILING,
  RECOVERING_OFF,
  RECOVERING_ON
};

// ========================= API PRINCIPALI =========================

// Inizializza il modulo audio (pin, stato)
void begin();

// Porta DFPlayer in stato "pronto" (alimentato + UART + volume)
void primeReady();

// Richiede riproduzione MP3 (non bloccante, gestisce priorità/coda)
void requestPlayMp3(uint16_t track, uint32_t capSeconds = 0);

// Richiesta MP3 a bassa priorità (scartata se c'è già qualcosa)
void requestPlayMp3Low(uint16_t track, uint32_t capSeconds = 0);

// Imposta volume (0..30)
void setVolume(uint8_t vol);

// Stop riproduzione (senza spegnere DFPlayer)
void stopNow(bool clearQueue = false);

// Hard reset DFPlayer (power-cycle via MOSFET, non-blocking recovery)
void hardReset(bool clearQueue = true);

// Toggle manuale da tastiera persistito in NVS. OFF taglia VCC e salva uno
// snapshot diagnostico; ON avvia un power-cycle non bloccante.
// Ritorna il nuovo stato abilitato.
bool toggleEnabled();

// Stato operativo richiesto e readiness effettiva lato firmware.
bool isEnabled();
bool isReady();

// Spegne completamente DFPlayer (per standby/light-sleep)
void powerOffNow();

// Task da chiamare nel loop principale
void task(uint32_t nowMs);

// C'è audio attivo o in coda?
bool isActive();

// Svuota la coda
void queueClear();

// Debug: stampa stato su Serial
void debugStatus();

// Flight recorder: ring buffer RAM + ultimo snapshot persistente selettivo.
void printHistory();
void clearHistory();
bool hasSavedDiagnostic();

// ========================= ACCESSO ALLO STATO =========================

State getState();
uint8_t getVolume();

} // namespace Audio
