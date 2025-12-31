#include "keypad.h"

// Righe (filo 1..4) - come da README
static const int rowPins[4] = {
  17,  // R1 = filo 1
  5,   // R2 = filo 2
  13,  // R3 = filo 3
  14   // R4 = filo 4
};

// Colonne (filo 5..6) - come da README
static const int colPins[2] = {
  19,  // C1 = filo 5
  21   // C2 = filo 6
};

// Mappa riga/colonna -> tasto
static KeyCode keyMap[4][2] = {
  // C1 (filo 5)     C2 (filo 6)
  {KEY_TARE,   KEY_ENTER}, // R1 (filo 1)
  {KEY_ZERO,   KEY_UP},    // R2 (filo 2)
  {KEY_UNIT,   KEY_SET},   // R3 (filo 3)
  {KEY_CALI,   KEY_MODE}   // R4 (filo 4)
};

// Stato interno per debounce
static KeyCode rawKeyLast    = KEY_NONE;
static uint32_t rawChangeMs  = 0;

static KeyCode stableKey     = KEY_NONE;
static KeyCode lastReported  = KEY_NONE;

// Ultimo evento "nuovo" da consegnare
static KeyCode pendingEvent  = KEY_NONE;

// Parametro debounce (ms)
static const uint32_t DEBOUNCE_MS = 40;

// Scansione singola (senza debounce pesante)
static KeyCode keypad_scan_once() {
  for (int r = 0; r < 4; r++) {
    // Attiva solo questa riga
    for (int rr = 0; rr < 4; rr++) {
      digitalWrite(rowPins[rr], (rr == r) ? LOW : HIGH);
    }

    delayMicroseconds(200); // assestamento

    for (int c = 0; c < 2; c++) {
      int val = digitalRead(colPins[c]);
      if (val == LOW) {
        return keyMap[r][c];
      }
    }
  }

  return KEY_NONE;
}

void keypad_init() {
  // Righe in output, inattive HIGH
  for (int r = 0; r < 4; r++) {
    pinMode(rowPins[r], OUTPUT);
    digitalWrite(rowPins[r], HIGH);
  }

  // Colonne in input con pull-up
  for (int c = 0; c < 2; c++) {
    pinMode(colPins[c], INPUT_PULLUP);
  }

  rawKeyLast    = KEY_NONE;
  rawChangeMs   = 0;
  stableKey     = KEY_NONE;
  lastReported  = KEY_NONE;
  pendingEvent  = KEY_NONE;
}

static KeyCode lastStableKey = KEY_NONE;

void keypad_update(uint32_t nowMs) {
  (void)nowMs; // non usato

  KeyCode k = keypad_scan_once();

  if (k != lastStableKey) {
    if (k != KEY_NONE) {
      // genera evento quando passa da NONE -> tasto
      pendingEvent = k;
    }
    lastStableKey = k;
  }
}

KeyCode keypad_get_event() {
  KeyCode ev = pendingEvent;
  pendingEvent = KEY_NONE;
  return ev;
}

void keypad_get_pins(const int** rows, int* nRows, const int** cols, int* nCols) {
  if (rows)  *rows  = rowPins;
  if (nRows) *nRows = 4;
  if (cols)  *cols  = colPins;
  if (nCols) *nCols = 2;
}
