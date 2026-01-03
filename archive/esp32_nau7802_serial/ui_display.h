#pragma once

#include <Arduino.h>

// Inizializza l'hardware display (SPI + OLED)
void ui_init();

// Mostra la schermata di boot (logo + Ronin 00)
// line1/line2 vengono mostrate in basso; possono essere "" se non servono.
void ui_showBoot(const char* line1, const char* line2);

// Schermata di errore bloccante durante l'avvio
// (es. periferiche non trovate). hint tipico: "Premi TARA per continuare".
void ui_showError(const char* title, const char* detail, const char* hint);

// Render principale del layout peso
//  - gDisp: peso da mostrare (grammi, intero)
//  - stateLabel: "STABLE", "UNSTABLE" o "LIVE"
void ui_renderWeight(long gDisp, const char* stateLabel);
