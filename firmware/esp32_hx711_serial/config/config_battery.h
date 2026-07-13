#pragma once

#include <Arduino.h>

// =============================================================================
// CONFIG_BATTERY.H - Soglie e timing per gestione batteria/shutdown
// =============================================================================

namespace BatteryConfig {

// ========================= SOGLIE TENSIONE =========================
// Tensione sotto la quale inizia il countdown per lo sleep
static const float V_SAFE_SHUTDOWN_MIN_V   = 5.80f;

// Tensione sopra la quale il countdown viene annullato
static const float V_SAFE_SHUTDOWN_CLEAR_V = 5.90f;

// Tensione di uscita dal countdown (con isteresi)
static const float V_SAFE_SHUTDOWN_EXIT_V  = 5.85f;

// Tensione critica: sleep dopo conferma temporale HARD_SLEEP_DEBOUNCE_MS
static const float V_HARD_SLEEP_MIN_V      = 5.70f;

// Reject broken-bus/out-of-range INA219 samples before filters and safety logic.
static const float SENSOR_MIN_V             = 3.0f;
static const float SENSOR_MAX_V             = 9.0f;
static const float SENSOR_MAX_SHUNT_MV      = 330.0f;
static const float SENSOR_MAX_CURRENT_MA    = 3500.0f;

// ========================= TIMING SHUTDOWN =========================
// Debounce prima di iniziare countdown (evita falsi positivi)
static const uint32_t SAFE_SHUTDOWN_DEBOUNCE_MS   = 5000;
static const uint32_t HARD_SLEEP_DEBOUNCE_MS      = 3000;
static const uint32_t SAMPLE_FRESH_MAX_MS         = 1500;

// Tempo stabile sopra EXIT_V per uscire dal countdown
static const uint32_t SAFE_SHUTDOWN_EXIT_STABLE_MS = 15000;

// Durata countdown pre-sleep
static const uint32_t PRE_SLEEP_COUNTDOWN_MS      = 120000;  // 2 minuti

// Tempo a cui suonare l'audio di metà countdown
static const uint32_t PRE_SLEEP_MID_AUDIO_MS      = 60000;   // 1 minuto

// ========================= TIMING BEEP =========================
// Cooldown tra beep batteria vuota (0 tacche)
static const uint32_t BEEP_EMPTY_MS           = 300000;  // 5 minuti

// Intervallo beep durante countdown
static const uint32_t BEEP_COUNTDOWN_MS       = 10000;   // 10 secondi

// ========================= TIMING STATO BATTERIA =========================
// Debounce ingresso stato EMPTY (0 tacche)
static const uint32_t BATT_EMPTY_DEBOUNCE_MS  = 10000;   // 10 secondi

// Tempo stabile per recovery da EMPTY
static const uint32_t BATT_RECOVERY_STABLE_MS = 30000;   // 30 secondi

// Cooldown tra avvisi audio batteria bassa
static const uint32_t BATT_ALERT_COOLDOWN_MS  = 300000;  // 5 minuti

// Durata schermata Zzz prima del light-sleep
static const uint32_t BATT_ZZZ_MS             = 5000;    // 5 secondi

} // namespace BatteryConfig
