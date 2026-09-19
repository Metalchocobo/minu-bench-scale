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
  KEY_MODE,
  KEY_BATTERY  // Virtual key: WIFI + TOTAL on the same matrix row.
};

// Inizializza i pin e lo stato interno
void keypad_init();

// Suppress the key that is physically held after a GPIO wake until release.
void keypad_suppress_wake_key();

// Da chiamare nel loop principale con now = millis().
// Debounce + one-shot. WIFI/TOTAL fire on release, allowing the battery chord.
void keypad_update(uint32_t nowMs);

// Ritorna l'ULTIMO tasto "nuovo" premuto (one-shot).
// - Tenere premuto non genera eventi ripetuti
// - WIFI/TOTAL fire on release unless consumed by the battery chord.
// Se non ci sono eventi nuovi, ritorna KEY_NONE.
KeyCode keypad_get_event();

// Ritorna true se il tasto specificato è attualmente premuto (stabile).
// NON consuma l'evento, serve per rilevare long press.
bool keypad_is_pressed(KeyCode key);

// Utility: espone i pin usati (righe/colonne) per setup wakeup in sleep.
void keypad_get_pins(const int** rows, int* nRows, const int** cols, int* nCols);
