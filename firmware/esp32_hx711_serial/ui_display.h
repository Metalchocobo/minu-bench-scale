#pragma once

#include <Arduino.h>
#include "scale_state.h"

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
//  - showTarget: se true mostra peso target (da MQTT weigh)
//  - targetWeight: peso target in grammi (float)
//  - ingredientName: nome ingrediente (se disponibile), troncato lato UI
//  - mqttConnected: true se connesso al broker MQTT
//  - mqttVisible: true se l'icona MQTT deve essere visibile (WiFi connesso)
void ui_renderWeight(long gDisp, const char* stateLabel,
                     bool showTarget = false, float targetWeight = 0.0f,
                     const char* ingredientName = nullptr,
                     bool mqttConnected = false, bool mqttVisible = false);

// Render LIVE fixed-point half grams (g * 2), always with .0/.5.
void ui_renderWeightLiveHalf(long gDispX2, const char* stateLabel,
                             bool showTarget = false, float targetWeight = 0.0f,
                             const char* ingredientName = nullptr,
                             bool mqttConnected = false, bool mqttVisible = false);

// Overlay TARA: mostra stato stabilizzazione, senza barra a tempo.
void ui_renderTareStatus(ScaleState::TareUiState state, bool autoMode);

// Schermata ERROR runtime HX711 (bloccante)
// - hard=true  -> non fidarsi del valore (non mostra last)
// - showLast=true -> mostra lastG come "Ultimo valore valido"
void ui_renderHxError(bool hard, bool showLast, long lastG);

// Persistent positive/negative capacity fault; weight actions are blocked.
void ui_renderOverload();
// Schermata transizione standby (Zzz...)
void ui_renderSleepZzz();

// Overlay controllo manuale DFPlayer.
void ui_renderAudioStatus(bool enabled, bool ready, bool logSaved);

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

// Step SAVE RESULT: esito salvataggio calibrazione
void ui_renderCalSaveResult(bool ok);

// Long press progress: mostra barra di avanzamento durante long press
// progress: 0-100 (0 = appena iniziato, 100 = wizard si attiva)
void ui_renderCalLongPress(uint8_t progress);

// ========================= WEIGH STACK OVERLAYS =========================

// Overlay: confronto totale logico vs netto fisico
// gTotal: somma stack, gNet: peso netto reale, count: elementi stack
void ui_renderStackCompare(long gTotal, long gNet, int count);

// Overlay: feedback CLEAR (pop/clear all)
// msg: "POP" o "CLEAR ALL", remaining: elementi rimasti nello stack
void ui_renderStackClear(const char* msg, int remaining);

