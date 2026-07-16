#pragma once

#include <Arduino.h>

// =============================================================================
// CONFIG_SCALE.H - Configurazione bilancia per Minù Bench Scale
// =============================================================================

namespace ScaleConfig {

// ========================= CALIBRAZIONE (seed) =========================
// Valori indicativi, sovrascritti da NVS + CAL
static const long  DEFAULT_REF_RAW  = 265290;   // raw medio con 2000 g
static const long  DEFAULT_ZERO_RAW = 51471;    // raw medio a vuoto
static const long  DEFAULT_REF_G    = 2000;     // grammi di riferimento

inline float defaultCpg() {
  return (float)(DEFAULT_REF_RAW - DEFAULT_ZERO_RAW) / (float)DEFAULT_REF_G;
}

// ========================= HX711 =========================
static const uint8_t  HX711_PULSE_US       = 1;     // Micro-delay per lettura
static const uint32_t HX711_INIT_TIMEOUT_MS = 1200;  // Timeout init

// ========================= WATCHDOG / BUS TIMEOUT =========================
static const uint32_t LOOP_WDT_TIMEOUT_MS      = 8000;   // Task watchdog loop
static const uint32_t MQTT_TLS_WDT_TIMEOUT_MS  = 20000;  // Guard durante handshake TLS
static const uint16_t I2C_TIMEOUT_MS           = 50;     // Timeout transazioni Wire

// ========================= AUTO-TARE BOOT =========================
static const bool     AUTO_TARE_ON_BOOT         = true;
static const uint16_t AUTO_TARE_SETTLE_MS       = 300;
static const uint16_t AUTO_TARE_MAX_MS          = 2000;
static const uint8_t  AUTO_TARE_TARGET_SAMPLES  = 64;
static const uint8_t  AUTO_TARE_MIN_SAMPLES     = 32;
static const uint8_t  AUTO_TARE_TRIM_SAMPLES    = 8;

// ========================= TARE UI =========================
static const uint8_t  TARE_SAMPLE_BUF_N      = 64;    // Finestra mobile campioni tara
static const uint32_t TARE_OK_HOLD_MS        = 200;   // Feedback breve a tara applicata
static const uint32_t TARE_FAIL_HOLD_MS      = 1200;  // Remains visible after the error beep

// Manual tare at 80-95 SPS. It starts after the debounced key release and
// averages ambient vibration instead of requiring a perfectly quiet bench.
static const uint32_t TARE_MANUAL_SETTLE_MS         = 60;
static const uint32_t TARE_MANUAL_MIN_MS            = 700;
static const uint32_t TARE_MANUAL_MAX_MS            = 1600;
static const uint32_t TARE_MANUAL_ARM_MAX_MS        = 1500;
static const uint8_t  TARE_MANUAL_WINDOW_SAMPLES    = 64;
static const uint8_t  TARE_MANUAL_TRIM_SAMPLES      = 8;
static const uint8_t  TARE_MANUAL_EDGE_SAMPLES      = 16;
static const uint8_t  TARE_MANUAL_STABLE_SAMPLES    = 1;
static const uint32_t TARE_MANUAL_SAMPLE_MAX_AGE_MS = 75;
static const uint32_t TARE_MANUAL_WINDOW_MAX_MS     = 1000;
// Only sustained drift rejects a manual tare. Sample range is diagnostic:
// periodic vibration widens it without moving the robust center.
static const float    TARE_MANUAL_MAX_SLOPE_GPS     = 10.0f;

// LIVE ENTER keeps its stricter quiet-weight gate; it is intentionally
// independent from the more tolerant explicit TARE operation.
static const float    LIVE_ENTER_RANGE_G            = 2.5f;
static const float    LIVE_ENTER_SLOPE_GPS          = 2.0f;

// Bounded ENTER acquisition used when the normal STABLE gate is not ready.
// Raw samples are decimated to about 40 Hz so the 64-slot window spans most
// of the 1.5 s progress bar. Range is diagnostic; sustained drift rejects.
// The fallback slope is deliberately looser than the normal LIVE gate so a
// centered liquid oscillation is not mistaken for continued pouring.
static const uint32_t ENTER_CAPTURE_MAX_MS            = 1500;
static const uint32_t ENTER_CAPTURE_SETTLE_MS         = 60;
static const uint8_t  ENTER_CAPTURE_SAMPLE_BUF_N      = 64;
static const uint8_t  ENTER_CAPTURE_MIN_SAMPLES       = 48;
static const uint8_t  ENTER_CAPTURE_RAW_DECIMATION    = 2;
static const uint32_t ENTER_CAPTURE_SAMPLE_MAX_AGE_MS = 75;
static const uint32_t ENTER_CAPTURE_WINDOW_MIN_MS     = 1000;
static const uint32_t ENTER_CAPTURE_WINDOW_MAX_MS     = 1600;
static const float    ENTER_CAPTURE_MAX_SLOPE_GPS     = 4.0f;
static const uint32_t ENTER_SUCCESS_HOLD_MS            = 1300;
static const uint32_t ENTER_FAIL_HOLD_MS               = 2000;

// The post-ENTER working tare never opens a second sampling window: the quiet
// path uses the current filtered WORK raw; the fallback uses its robust center.

// ========================= LIMITE DISPLAY =========================
static const float MAX_DISPLAY_G   = 16000.0f;      // Soglia fault simmetrica ±16 kg
static const float OVERLOAD_EXIT_G = 15950.0f;      // 50 g hysteresis

// ========================= FILTRI =========================
static const int   MA_DEFAULT      = 6;             // Media mobile (work)
static const int   MA_FINE         = 4;             // Media mobile (live)
static const float DB_UNSTABLE_N   = 0.10f;         // Deadband UNSTABLE (normal)
static const float DB_UNSTABLE_F   = 0.05f;         // Deadband UNSTABLE (fine)

// ========================= STATI STABLE/UNSTABLE =========================
static const bool     ST_ENABLE_DEFAULT  = true;
static const float    ST_LEAVE_DELTA_G   = 0.80f;   // |gLive - gLatch| >= -> UNSTABLE
static const float    ST_ENTER_RANGE_G   = 0.60f;   // range < -> STABLE
static const uint32_t ST_TO_STABLE_MS    = 1200;    // Finestra valutazione (1.2s)

// ========================= ZERO-TRACKING =========================
static const bool     ZT_ENABLE_DEFAULT  = true;

// Micro-correzione lenta vicino a 0
static const float    ZT_WINDOW_G        = 1.5f;    // Attivo se |g| <= 1.5 g
static const uint32_t ZT_QUIET_MS        = 1200;    // "quiete" per 1.2 s
static const float    ZT_QUIET_RANGE_G   = 0.4f;    // range <= 0.4 g
static const float    ZT_QUIET_SLOPE_GPS = 0.3f;    // |slope| <= 0.3 g/s
static const uint32_t ZT_PERIOD_MS       = 1000;    // Passo ogni 1.0 s
static const float    ZT_STEP_G          = 0.02f;   // ~0.02 g per passo
static const float    ZT_MAX_G           = 3.0f;    // Cap totale ±3 g

// Snap-to-zero su scarico
static const bool     UNLOAD_SNAP_ENABLE    = true;
static const float    UNLOAD_CROSS_WIN_G    = 3.0f;    // Esegui snap se |g| <= 3 g
static const float    UNLOAD_SLOPE_GPS_NEG  = -5.0f;   // Pendenza <= -5 g/s
static const uint32_t UNLOAD_SLOPE_WIN_MS   = 400;     // Finestra slope
static const float    UNLOAD_STABLE_RANGE_G = 0.8f;    // Range "quiete"
static const uint32_t UNLOAD_STABLE_MS      = 300;     // ms di quiete
static const uint32_t UNLOAD_COOLDOWN_MS    = 2000;    // Minimo tra due snap
static const float    UNLOAD_SNAP_MAX_G     = 2.0f;    // Correzione max snap

// ========================= CAMPIONAMENTO =========================
static const uint32_t SAMPLE_MS      = 60;          // ~16 Hz logica
static const uint32_t OLED_UPDATE_MS = 60;          // Refresh OLED

// ========================= DECIMAZIONE HX711 (80 SPS) =========================
static const uint8_t DECIM_WORK_N = 5;              // ~16 Hz (stabilità)
static const uint8_t DECIM_LIVE_N = 3;              // ~26 Hz (reattività)

// ========================= ANTI-SPIKE GUARD =========================
static const float    RAW_SPIKE_JUMP_G     = 150.0f;  // Salto minimo per attivare
static const float    RAW_SPIKE_CONFIRM_G  = 80.0f;   // Conferma dal successivo
static const uint32_t RAW_SPIKE_TIMEOUT_MS = 250;     // Timeout pending

// ========================= DISPLAY QUANTIZZAZIONE =========================
// LIVE
static const float    DISP_ZERO_ENTER_LIVE_G  = 0.50f;
static const float    DISP_ZERO_EXIT_LIVE_G   = 0.35f;
static const float    DISP_STEP_HYS_LIVE_G    = 0.12f;
static const uint16_t DISP_LIVE_REVERSAL_LOCK_MS = 160;
static const float    LIVE_EMA_ALPHA          = 0.45f;
static const uint8_t  DISP_LIVE_HALF_WIN_N    = 16;     // ~600 ms at 80 SPS / N=3
static const float    DISP_LIVE_HALF_LOW_RATIO  = 0.30f;
static const float    DISP_LIVE_HALF_HIGH_RATIO = 0.70f;
static const float    DISP_LIVE_HALF_STICKY_G = 0.08f;  // Hysteresis around half-step midpoint

// WORK
static const float    DISP_ZERO_ENTER_WORK_G  = 1.30f;
static const float    DISP_ZERO_EXIT_WORK_G   = 0.90f;
static const float    DISP_STEP_HYS_WORK_G    = 0.20f;

// ========================= INATTIVITÀ =========================
static const uint32_t INACTIVITY_SLEEP_MS     = 5UL * 60UL * 1000UL;  // 5 minuti
static const uint32_t INACTIVITY_ZZZ_MS       = 5000;                 // Schermata Zzz
static const long     INACTIVITY_WEIGHT_DELTA_G = 5;                  // ±5 g = "fermo"

// ========================= HX HEALTH =========================
static const uint32_t HX_HEALTH_WARN_MS       = 500;
static const uint32_t HX_HEALTH_ERROR_MS      = 3000;
static const uint32_t HX_HEALTH_HARD_MS       = 30000;
static const uint32_t HX_HEALTH_OK_STABLE_MS  = 800;
static const uint32_t HX_HEALTH_LAST_VALID_MS = 30000;

} // namespace ScaleConfig
