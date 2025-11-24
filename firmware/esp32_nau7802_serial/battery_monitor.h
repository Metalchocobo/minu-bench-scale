#pragma once

#include <Arduino.h>

enum BatteryLevel {
  BATT_LEVEL_FULL = 0,
  BATT_LEVEL_GOOD,
  BATT_LEVEL_LOW,
  BATT_LEVEL_CRITICAL
};

struct BatteryStatus {
  float voltage_V;     // Tensione batteria filtrata (V)
  float current_mA;    // Corrente filtrata (mA), >0 = scarica, <0 = carica
  BatteryLevel level;  // Livello batteria (4 step)
  bool charging;       // true = in carica (corrente che entra nella batteria)
};

// Da chiamare dopo che il bus I2C è stato inizializzato (Wire.begin(...))
void battery_init();

// Da chiamare nel loop principale, passando millis()
void battery_update(uint32_t nowMs);

// Restituisce l'ultimo stato calcolato (copia per valore)
BatteryStatus battery_get_status();

// Stampa su Serial una riga di debug leggibile
void battery_debug_print(const BatteryStatus &st);
