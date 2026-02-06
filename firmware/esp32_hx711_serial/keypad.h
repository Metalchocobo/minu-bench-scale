#pragma once
#include <Arduino.h>

enum KeyCode {
  KEY_NONE = 0,
  KEY_TARE,
  KEY_ENTER,
  KEY_WIFI,
  KEY_SLEEP,
  KEY_SKIP,
  KEY_TOTAL,
  KEY_CLEAR,
  KEY_MODE
};

// Inizializza i pin e lo stato interno
void keypad_init();

// Da chiamare nel loop principale con now = millis().
// Debounce + evento one-shot su pressione (transizione stabile NONE -> KEY).
void keypad_update(uint32_t nowMs);

// Ritorna l'ULTIMO tasto "nuovo" premuto (one-shot).
// - Tenere premuto non genera eventi ripetuti
// - Il rilascio non genera evento
// Se non ci sono eventi nuovi, ritorna KEY_NONE.
KeyCode keypad_get_event();

// Ritorna true se il tasto specificato è attualmente premuto (stabile).
// NON consuma l'evento, serve per rilevare long press.
bool keypad_is_pressed(KeyCode key);

// Utility: espone i pin usati (righe/colonne) per setup wakeup in sleep.
void keypad_get_pins(const int** rows, int* nRows, const int** cols, int* nCols);
