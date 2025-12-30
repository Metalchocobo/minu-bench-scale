#pragma once
#include <Arduino.h>

enum KeyCode {
  KEY_NONE = 0,
  KEY_TARE,
  KEY_ENTER,
  KEY_ZERO,
  KEY_UP,
  KEY_UNIT,
  KEY_SET,
  KEY_CALI,
  KEY_MODE
};

// Inizializza i pin e lo stato interno
void keypad_init();

// Da chiamare nel loop principale con now = millis()
void keypad_update(uint32_t nowMs);

// Ritorna l'ULTIMO tasto "nuovo" premuto.
// Se non ci sono eventi nuovi, ritorna KEY_NONE.
KeyCode keypad_get_event();
