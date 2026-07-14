#pragma once

#include <Arduino.h>

// =============================================================================
// CONFIG_AUDIO.H - Configurazione audio DFPlayer per Minù Bench Scale
// =============================================================================

namespace AudioConfig {

// ========================= DFPLAYER GENERALE =========================
static const uint8_t  DEFAULT_VOLUME = 20;      // Volume 0..30
static const uint32_t BAUD_RATE      = 9600;

// ========================= TIMING STATE MACHINE =========================
static const uint32_t POWERUP_MS       = 250;   // Tempo dopo power-on
static const uint32_t BUSY_DETECT_MS   = 800;   // Finestra auto-detect polarità BUSY
static const uint32_t END_STABLE_MS    = 250;   // "idle" stabile per considerare finito
static const uint32_t TAIL_OFF_MS      = 150;   // Evita click post-stop
static const uint32_t RESTART_GAP_MS   = 200;   // Gap minimo stop->play (anti-troncamento)
static const uint32_t MIN_PLAY_MS      = 300;   // Ignora BUSY=idle nei primi ms
static const uint32_t PLAY_TIMEOUT_MS  = 30000; // Paracadute se BUSY non funziona
static const uint32_t RECOVERY_OFF_MS  = 800;   // Hard power-cycle window after DFPlayer lockup

// ========================= CODA AUDIO =========================
static const uint8_t QUEUE_MAX_SIZE = 16;

// ========================= ANTI-RETRIGGER =========================
static const uint32_t REQUEST_GUARD_MS = 250;   // Evita richieste duplicate ravvicinate

// ========================= TRACK MUST-PLAY (non interrompibili) =========================
// Questi track non possono essere interrotti e vengono accodati
inline bool isMustPlayTrack(uint16_t track) {
  return (track == 2  || track == 7  || track == 8  ||
          track == 12 || track == 13 || track == 14 ||
          track == 15 || track == 16);
}

// ========================= LEGENDA MP3 =========================
// 0001 = Avvio bilancia
// 0002 = Boot completato
// 0003 = Standby inattività (Zzz...)
// 0004 = Wake inattività
// 0005 = WiFi modulo ON
// 0006 = WiFi modulo OFF
// 0007 = Errore WiFi (cooldown 60s)
// 0008 = Connessione WiFi OK
// 0011 = Batteria bassa (0 tacche)
// 0012 = Batteria critica (countdown)
// 0013 = Standby batteria scarica
// 0014 = Errore INA219
// 0015 = Errore HX711
// 0016 = Tare not completed (boot or manual)
// 0017 = Modalità WORK
// 0018 = Modalità LIVE

namespace Track {
  static const uint16_t BOOT_START      = 1;
  static const uint16_t BOOT_COMPLETE   = 2;
  static const uint16_t SLEEP_ENTER     = 3;
  static const uint16_t SLEEP_EXIT      = 4;
  static const uint16_t WIFI_MODULE_ON  = 5;   // Rinominato da WIFI_ON (conflitto macro ESP32)
  static const uint16_t WIFI_MODULE_OFF = 6;   // Rinominato da WIFI_OFF (conflitto macro ESP32)
  static const uint16_t WIFI_ERROR      = 7;
  static const uint16_t WIFI_CONNECTED  = 8;
  static const uint16_t ENTER_PRESSED  = 9;
  static const uint16_t SKIP_PRESSED   = 10;
  static const uint16_t BATT_LOW       = 11;
  static const uint16_t BATT_CRITICAL  = 12;
  static const uint16_t BATT_SLEEP     = 13;
  static const uint16_t ERROR_INA      = 14;
  static const uint16_t ERROR_HX       = 15;
  static const uint16_t ERROR_TARE     = 16;
  static const uint16_t MODE_WORK      = 17;
  static const uint16_t MODE_LIVE      = 18;
  static const uint16_t TOTAL_PRESSED  = 20;
  static const uint16_t CLEAR_PRESSED  = 21;
}

} // namespace AudioConfig
