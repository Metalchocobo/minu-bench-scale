#pragma once
#include <Arduino.h>

void buzzerInit();

void buzzerBootMelody();  // fanfara di boot

void buzzerKeyClick();    // click tasto
void buzzerOk();          // beep conferma
void buzzerError();       // doppio beep errore
void buzzerWarn();        // beep avviso (batteria scarica)
