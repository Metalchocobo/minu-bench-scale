#pragma once
#include <Arduino.h>

void buzzerInit();

void buzzerBootMelody();  // fanfara di boot

void buzzerKeyClick();    // click tasto
void buzzerOk();          // beep conferma
void buzzerError();       // doppio beep errore
void buzzerWarn();        // beep avviso (batteria scarica)
void buzzerMqttRx();      // beep distintivo ricezione comando weigh

// One non-blocking double beep per overload entry; call on every loop.
void buzzerUpdateOverload(bool active, uint32_t nowMs);
// Stop the pending alert before sleep without rearming an ongoing overload.
void buzzerCancelOverload();
