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

// Stato rete (WiFi/OTA) da mostrare nell'icona in alto.
struct UiNetStatus {
  bool enabled;        // WiFi abilitato
  bool connected;      // WiFi connesso
  bool otaEnabled;     // OTA attivo insieme al WiFi
  bool hasCredentials; // Sono state configurate credenziali valide
};
void ui_setNetStatus(const UiNetStatus &status);

// Render principale del layout peso
//  - gDisp: peso da mostrare (grammi, intero)
//  - stateLabel: "STABLE", "UNSTABLE" o "LIVE"
void ui_renderWeight(long gDisp, const char* stateLabel);

// Overlay TARA: mostra testo + barra (0..100)
void ui_renderTareProgress(uint8_t progressPct);
// Schermata transizione standby (Zzz...)
void ui_renderSleepZzz();
