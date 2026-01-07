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
  {KEY_SKIP,   KEY_ENTER}, // R1 (filo 1)
  {KEY_WIFI,   KEY_TOTAL},    // R2 (filo 2)
  {KEY_SLEEP, KEY_MODE},   // R3 (filo 3)
  {KEY_CLEAR,   KEY_TARE}   // R4 (filo 4)
};

// Stato interno tastiera.
// Obiettivo:
// - Debounce per eliminare rimbalzi (contatto meccanico)
// - Evento "one-shot" solo su pressione (transizione stabile NONE -> KEY)
// Side effects:
// - Tenere premuto non genera eventi ripetuti
// - Il rilascio non genera evento
static KeyCode rawKeyLast   = KEY_NONE;
static uint32_t rawChangeMs = 0;
static KeyCode stableKey    = KEY_NONE;

// Ultimo evento "nuovo" da consegnare
static KeyCode pendingEvent = KEY_NONE;

// Debounce (ms): 30-50 ms è la forchetta tipica. Qui 40 ms.
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

  rawKeyLast   = KEY_NONE;
  rawChangeMs  = 0;
  stableKey    = KEY_NONE;
  pendingEvent = KEY_NONE;
}

void keypad_update(uint32_t nowMs) {
  // 1) Legge il valore raw.
  const KeyCode raw = keypad_scan_once();

  // 2) Traccia l'istante dell'ultimo cambiamento raw.
  if (raw != rawKeyLast) {
    rawKeyLast = raw;
    rawChangeMs = nowMs;
  }

  // 3) Promuove a "stabile" solo se è rimasto uguale per DEBOUNCE_MS.
  if (raw != stableKey && (uint32_t)(nowMs - rawChangeMs) >= DEBOUNCE_MS) {
    const KeyCode prev = stableKey;
    stableKey = raw;

    // Evento one-shot: solo su pressione (NONE -> KEY).
    if (prev == KEY_NONE && stableKey != KEY_NONE) {
      pendingEvent = stableKey;
    }
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
