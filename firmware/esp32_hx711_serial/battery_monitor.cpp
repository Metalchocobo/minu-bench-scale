#include "battery_monitor.h"
#include "config/config_battery.h"
#include "config/config_pins.h"

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <math.h>

// -----------------------------------------------------------------------------
// CONFIGURAZIONE INA219 / FILTRI / SOGLIE
// -----------------------------------------------------------------------------

// Indirizzo I2C del CJMCU-219 (tipicamente 0x40)
static const uint8_t INA219_I2C_ADDR = 0x40;

// Intervallo minimo tra due letture reali dal sensore
static const uint32_t READ_INTERVAL_MS = 500;

// Fattore di filtro esponenziale (0..1); valori più alti = risposta più rapida
static const float ALPHA_VOLTAGE = 0.2f;
static const float ALPHA_CURRENT = 0.3f;

// -----------------------------------------------------------------------------
// STATO INTERNO
// -----------------------------------------------------------------------------

static Adafruit_INA219 g_ina219(INA219_I2C_ADDR);
static bool g_inited = false;

static BatteryStatus g_status;

static bool g_haveSample = false;
static float g_vFilt = 0.0f;
static float g_iFilt = 0.0f;

static uint32_t g_lastReadMs = 0;
static bool g_invalidLogged = false;

static bool g_usbCandidateValid = false;
static bool g_usbCandidatePresent = false;
static uint32_t g_usbCandidateSinceMs = 0;
static uint32_t g_usbLastReadMs = 0;

// -----------------------------------------------------------------------------
// FUNZIONI INTERNE
// -----------------------------------------------------------------------------

static float lowPassUpdate(float prev, float value, float alpha) {
  if (!g_haveSample) return value;
  return prev + alpha * (value - prev);
}

static BatteryLevel levelFromVoltage(float v) {
  if (v >= BatteryConfig::V_FULL_MIN) {
    return BATT_LEVEL_FULL;
  } else if (v >= BatteryConfig::V_GOOD_MIN) {
    return BATT_LEVEL_GOOD;
  } else if (v >= BatteryConfig::V_LOW_MIN) {
    return BATT_LEVEL_LOW;
  } else if (v >= BatteryConfig::V_CRITICAL_MIN) {
    return BATT_LEVEL_CRITICAL;
  } else {
    return BATT_LEVEL_EMPTY;
  }
}

static void resetUsbPresence() {
  g_status.charging = false;
  g_usbCandidateValid = false;
  g_usbCandidateSinceMs = 0;
  g_usbLastReadMs = 0;
}

static void updateUsbPresence(uint32_t nowMs) {
  const bool present = digitalRead(USB_PRESENT_PIN) ==
    (USB_PRESENT_ACTIVE_LOW ? LOW : HIGH);
  // A pause, including light-sleep, is not evidence of continuous stability.
  if (!g_usbCandidateValid || present != g_usbCandidatePresent ||
      (nowMs - g_usbLastReadMs) > BatteryConfig::USB_PRESENCE_DEBOUNCE_MS) {
    g_usbCandidatePresent = present;
    g_usbCandidateSinceMs = nowMs;
    g_usbCandidateValid = true;
  }
  g_usbLastReadMs = nowMs;
  if ((nowMs - g_usbCandidateSinceMs) >= BatteryConfig::USB_PRESENCE_DEBOUNCE_MS) {
    g_status.charging = g_usbCandidatePresent;
  }
}

static void invalidateSample() {
  g_status.valid = false;
  if (!g_invalidLogged) {
    Serial.println(F("[BATT] Lettura INA219 non valida"));
    g_invalidLogged = true;
  }
}

// -----------------------------------------------------------------------------
// API
// -----------------------------------------------------------------------------

void battery_init() {
  // GPIO36 has no internal pull-up; the PC817 circuit provides it externally.
  pinMode(USB_PRESENT_PIN, INPUT);
  resetUsbPresence();
  // Si assume che Wire.begin(...) sia già stato chiamato nel setup
  if (!g_ina219.begin()) {
    // Keep USB-presence polling available even when the INA219 is missing.
    Serial.println(F("[BATT] ERRORE: INA219 non trovato su I2C"));
    g_status.voltage_V = 0.0f;
    g_status.current_mA = 0.0f;
    g_status.level = BATT_LEVEL_EMPTY;
    g_status.charging = false;
    g_status.valid = false;
    g_status.lastValidMs = 0;
    g_inited = false;
    return;
  }

  // Range 32V / 2A, più che sufficiente per batteria 6 V e assorbimenti della bilancia
  g_ina219.setCalibration_32V_2A();

  g_status.voltage_V = 0.0f;
  g_status.current_mA = 0.0f;
  g_status.level = BATT_LEVEL_EMPTY;
  g_status.charging = false;
  g_status.valid = false;
  g_status.lastValidMs = 0;

  g_haveSample = false;
  g_lastReadMs = 0;
  g_invalidLogged = false;
  g_inited = true;

  Serial.println(F("[BATT] INA219 inizializzato (32V / 2A, addr 0x40)"));
}

bool battery_is_available() {
  return g_inited;
}

void battery_update(uint32_t nowMs) {
  updateUsbPresence(nowMs);
  if (!g_inited) return;

  if (nowMs - g_lastReadMs < READ_INTERVAL_MS) {
    return;  // ancora troppo presto per una nuova lettura
  }
  g_lastReadMs = nowMs;

  // Letture base dal sensore
  float busVoltage_V = g_ina219.getBusVoltage_V();
  if (!g_ina219.success()) {
    invalidateSample();
    return;
  }
  float shuntVoltage_mV = g_ina219.getShuntVoltage_mV();
  if (!g_ina219.success()) {
    invalidateSample();
    return;
  }
  float current_mA = g_ina219.getCurrent_mA();
  if (!g_ina219.success()) {
    invalidateSample();
    return;
  }

  // Tensione reale batteria ≈ tensione lato carico + caduta sullo shunt
  float vBatt_V = busVoltage_V + (shuntVoltage_mV / 1000.0f);

  if (!isfinite(busVoltage_V) || !isfinite(shuntVoltage_mV) ||
      !isfinite(current_mA) || !isfinite(vBatt_V) ||
      vBatt_V < BatteryConfig::SENSOR_MIN_V ||
      vBatt_V > BatteryConfig::SENSOR_MAX_V ||
      fabsf(shuntVoltage_mV) > BatteryConfig::SENSOR_MAX_SHUNT_MV ||
      fabsf(current_mA) > BatteryConfig::SENSOR_MAX_CURRENT_MA) {
    invalidateSample();
    return;
  }
  g_invalidLogged = false;

  // Aggiorniamo i filtri
  g_vFilt = lowPassUpdate(g_vFilt, vBatt_V, ALPHA_VOLTAGE);
  g_iFilt = lowPassUpdate(g_iFilt, current_mA, ALPHA_CURRENT);
  g_haveSample = true;

  g_status.voltage_V = g_vFilt;
  g_status.current_mA = g_iFilt;
  g_status.level = levelFromVoltage(g_vFilt);
  g_status.valid = true;
  g_status.lastValidMs = nowMs;

}

BatteryStatus battery_get_status() {
  return g_status;
}

bool battery_has_fresh_sample(uint32_t nowMs) {
  return g_inited && g_status.valid && g_status.lastValidMs != 0 &&
    (nowMs - g_status.lastValidMs) <= BatteryConfig::SAMPLE_FRESH_MAX_MS;
}

void battery_debug_print(const BatteryStatus &st) {
  Serial.print(F("[BATT] V="));
  Serial.print(st.voltage_V, 3);
  Serial.print(F(" V  I="));
  Serial.print(st.current_mA, 0);
  Serial.print(F(" mA  lvl="));

  switch (st.level) {
    case BATT_LEVEL_FULL:     Serial.print(F("FULL")); break;
    case BATT_LEVEL_GOOD:     Serial.print(F("GOOD")); break;
    case BATT_LEVEL_LOW:      Serial.print(F("LOW")); break;
    case BATT_LEVEL_CRITICAL: Serial.print(F("CRITICAL")); break;
    case BATT_LEVEL_EMPTY:    Serial.print(F("EMPTY")); break;
  }

  Serial.print(F("  charging="));
  Serial.print(st.charging ? F("YES") : F("NO"));
  Serial.print(F("  valid="));
  Serial.println(st.valid ? F("YES") : F("NO"));
}
