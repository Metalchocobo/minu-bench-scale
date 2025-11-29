#pragma once

#include <Arduino.h>

// Inizializza l'hardware display (SPI + OLED)
void ui_init();

// Mostra la schermata di boot (logo + Ronin 00)
void ui_showBoot();

// Render principale del layout peso
//  - gDisp: peso da mostrare (grammi, intero)
//  - stateLabel: "STABLE", "UNSTABLE" o "LIVE"
void ui_renderWeight(long gDisp, const char* stateLabel);
