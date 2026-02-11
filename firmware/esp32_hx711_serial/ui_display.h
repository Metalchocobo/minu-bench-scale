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

// Avviso batteria scarica e spegnimento imminente
void ui_showBatteryShutdown(float voltage_V);

// Abilita/disabilita power-save del display (riduce consumi)
void ui_powerSave(bool enable);

// Abilita/disabilita icona triangolo warning (HX WARN) in alto a destra
void ui_setHxWarn(bool enable);

// Render principale del layout peso
//  - gDisp: peso da mostrare (grammi, intero)
//  - stateLabel: "STABLE", "UNSTABLE" o "LIVE"
void ui_renderWeight(long gDisp, const char* stateLabel);

// Overlay TARA: mostra testo + barra (0..100)
void ui_renderTareProgress(uint8_t progressPct);

// Schermata ERROR runtime HX711 (bloccante)
// - hard=true  -> non fidarsi del valore (non mostra last)
// - showLast=true -> mostra lastG come "Ultimo valore valido"
void ui_renderHxError(bool hard, bool showLast, long lastG);
// Schermata transizione standby (Zzz...)
void ui_renderSleepZzz();

// ========================= CALIBRATION WIZARD UI =========================
// Step ZERO: Acquisisci zero (piatto vuoto)
// progress: 0-100 indica quanti campioni sono stati raccolti
void ui_renderCalStepZero(uint8_t progress);

// Step PLACE: Appoggia peso di riferimento
// progress: 0-100 indica quanti campioni sono stati raccolti
void ui_renderCalStepPlace(uint8_t progress);

// Step VALUE: Seleziona peso di riferimento
// refWeightG: valore peso corrente selezionato (es. 2000)
void ui_renderCalStepValue(uint16_t refWeightG);

// Step CONFIRM: Conferma calibrazione
// refWeightG: peso di riferimento, cpg: count-per-gram calcolato
void ui_renderCalStepConfirm(uint16_t refWeightG, float cpg);

// Long press progress: mostra barra di avanzamento durante long press
// progress: 0-100 (0 = appena iniziato, 100 = wizard si attiva)
void ui_renderCalLongPress(uint8_t progress);

