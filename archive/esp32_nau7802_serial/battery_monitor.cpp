#include "battery_monitor.h"

#include <Wire.h>
#include <Adafruit_INA219.h>

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

// Soglia per considerare la batteria "in carica" (mA)
// Corrente < -I_CHARGE_THRESHOLD_MA => sta entrando corrente in batteria
static const float I_CHARGE_THRESHOLD_MA = 30.0f;

// Soglie di tensione (V) per i 4 livelli batteria, sul valore filtrato
static const float V_FULL_MIN      = 6.40f;
static const float V_GOOD_MIN      = 6.20f;
static const float V_LOW_MIN       = 6.00f;
static const float V_EMPTY_MIN     = 5.80f;
// Sotto V_EMPTY_MIN => LEVEL_EMPTY

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

// -----------------------------------------------------------------------------
// FUNZIONI INTERNE
// -----------------------------------------------------------------------------

static float lowPassUpdate(float prev, float value, float alpha) {
  if (!g_haveSample) return value;
  return prev + alpha * (value - prev);
}

static BatteryLevel levelFromVoltage(float v) {
  if (v >= V_FULL_MIN) {
    return BATT_LEVEL_FULL;
  } else if (v >= V_GOOD_MIN) {
    return BATT_LEVEL_GOOD;
  } else if (v >= V_LOW_MIN) {
    return BATT_LEVEL_LOW;
  } else if (v >= V_EMPTY_MIN) {
    return BATT_LEVEL_CRITICAL;
  } else {
    return BATT_LEVEL_EMPTY;
  }
}

// -----------------------------------------------------------------------------
// API
// -----------------------------------------------------------------------------

void battery_init() {
  // Si assume che Wire.begin(...) sia già stato chiamato nel setup
  if (!g_ina219.begin()) {
    // Se fallisce l'init, lasciamo g_inited = false; la update non farà nulla.
    Serial.println(F("[BATT] ERRORE: INA219 non trovato su I2C"));
    g_status.voltage_V = 0.0f;
    g_status.current_mA = 0.0f;
    g_status.level = BATT_LEVEL_EMPTY;
    g_status.charging = false;
    g_inited = false;
    return;
  }

  // Range 32V / 2A, più che sufficiente per batteria 6 V e assorbimenti della bilancia
  g_ina219.setCalibration_32V_2A();

  g_status.voltage_V = 0.0f;
  g_status.current_mA = 0.0f;
  g_status.level = BATT_LEVEL_EMPTY;
  g_status.charging = false;

  g_haveSample = false;
  g_lastReadMs = 0;
  g_inited = true;

  Serial.println(F("[BATT] INA219 inizializzato (32V / 2A, addr 0x40)"));
}

bool battery_is_available() {
  return g_inited;
}

void battery_update(uint32_t nowMs) {
  if (!g_inited) return;

  if (nowMs - g_lastReadMs < READ_INTERVAL_MS) {
    return;  // ancora troppo presto per una nuova lettura
  }
  g_lastReadMs = nowMs;

  // Letture base dal sensore
  float busVoltage_V      = g_ina219.getBusVoltage_V();       // tensione tra GND e V-
  float shuntVoltage_mV   = g_ina219.getShuntVoltage_mV();    // tra V- e V+
  float current_mA        = g_ina219.getCurrent_mA();         // segno: V+ -> V-

  // Tensione reale batteria ≈ tensione lato carico + caduta sullo shunt
  float vBatt_V = busVoltage_V + (shuntVoltage_mV / 1000.0f);

  // Aggiorniamo i filtri
  g_vFilt = lowPassUpdate(g_vFilt, vBatt_V, ALPHA_VOLTAGE);
  g_iFilt = lowPassUpdate(g_iFilt, current_mA, ALPHA_CURRENT);
  g_haveSample = true;

  g_status.voltage_V = g_vFilt;
  g_status.current_mA = g_iFilt;
  g_status.level = levelFromVoltage(g_vFilt);

  // Logica "in carica": corrente verso la batteria (segno negativo) sopra soglia
  bool chargingNow = (g_iFilt < -I_CHARGE_THRESHOLD_MA);

  // Facile isteresi per evitare lampeggi:
  // - se già in carica, usciamo solo quando la corrente sale sopra (-I_CHARGE_THRESHOLD_MA / 2)
  if (g_status.charging) {
    if (g_iFilt > -(I_CHARGE_THRESHOLD_MA * 0.5f)) {
      g_status.charging = false;
    }
  } else {
    if (chargingNow) {
      g_status.charging = true;
    }
  }
}

BatteryStatus battery_get_status() {
  return g_status;
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
  Serial.println(st.charging ? F("YES") : F("NO"));
}
