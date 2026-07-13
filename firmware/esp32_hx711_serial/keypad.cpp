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
static uint32_t stableSinceMs = 0;
static KeyCode suppressedKey = KEY_NONE;
static uint32_t suppressedReleaseSinceMs = 0;

// Ultimo evento "nuovo" da consegnare
static KeyCode pendingEvent = KEY_NONE;

// Debounce (ms): 30-50 ms è la forchetta tipica. Qui 40 ms.
static const uint32_t DEBOUNCE_MS = 40;
static const uint32_t STUCK_KEY_MS = 15000;

static bool keypad_raw_is_pressed(KeyCode target) {
  if (target == KEY_NONE) return false;

  for (int r = 0; r < 4; r++) {
    for (int rr = 0; rr < 4; rr++) {
      digitalWrite(rowPins[rr], (rr == r) ? LOW : HIGH);
    }

    delayMicroseconds(200);

    for (int c = 0; c < 2; c++) {
      if (keyMap[r][c] == target && digitalRead(colPins[c]) == LOW) {
        return true;
      }
    }
  }

  return false;
}

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
        KeyCode key = keyMap[r][c];
        if (key == suppressedKey) {
          continue;
        }
        return key;
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
  stableSinceMs = 0;
  suppressedKey = KEY_NONE;
  suppressedReleaseSinceMs = 0;
  pendingEvent = KEY_NONE;
}

void keypad_suppress_wake_key() {
  suppressedKey = keypad_scan_once();
  suppressedReleaseSinceMs = 0;
  rawKeyLast = KEY_NONE;
  rawChangeMs = millis();
  stableKey = KEY_NONE;
  stableSinceMs = rawChangeMs;
  pendingEvent = KEY_NONE;
}

void keypad_update(uint32_t nowMs) {
  if (suppressedKey != KEY_NONE) {
    if (keypad_raw_is_pressed(suppressedKey)) {
      suppressedReleaseSinceMs = 0;
    } else if (suppressedReleaseSinceMs == 0) {
      suppressedReleaseSinceMs = nowMs;
    } else if ((uint32_t)(nowMs - suppressedReleaseSinceMs) >= DEBOUNCE_MS) {
      suppressedKey = KEY_NONE;
      suppressedReleaseSinceMs = 0;
    }
  }

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
    stableSinceMs = nowMs;

    // Evento one-shot: solo su pressione (NONE -> KEY).
    if (prev == KEY_NONE && stableKey != KEY_NONE) {
      pendingEvent = stableKey;
    }
  }

  if (stableKey != KEY_NONE &&
      suppressedKey == KEY_NONE &&
      (uint32_t)(nowMs - stableSinceMs) >= STUCK_KEY_MS) {
    suppressedKey = stableKey;
    suppressedReleaseSinceMs = 0;
    stableKey = KEY_NONE;
    rawKeyLast = KEY_NONE;
    rawChangeMs = nowMs;
    stableSinceMs = nowMs;
    pendingEvent = KEY_NONE;
  }
}

KeyCode keypad_get_event() {
  KeyCode ev = pendingEvent;
  pendingEvent = KEY_NONE;
  return ev;
}

bool keypad_is_pressed(KeyCode key) {
  // Ritorna true se il tasto specificato è attualmente premuto (stabile)
  return (stableKey == key);
}

void keypad_get_pins(const int** rows, int* nRows, const int** cols, int* nCols) {
  if (rows)  *rows  = rowPins;
  if (nRows) *nRows = 4;
  if (cols)  *cols  = colPins;
  if (nCols) *nCols = 2;
}
