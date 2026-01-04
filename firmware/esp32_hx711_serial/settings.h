#pragma once

#include <Arduino.h>

// Gestione centralizzata dei parametri persistenti (Preferences/NVS).
// - Parametri scala (offset, scala, peso di riferimento)
// - Config WiFi/OTA (abilitazione + credenziali)
// - Soglie batteria configurabili

struct ScaleSettings {
  long  offsetRaw;
  float scaleCpg;
  long  refG;
};

struct WifiCredential {
  String ssid;
  String password;
};

struct WifiSettings {
  bool wifiEnabled;  // true se deve avviare WiFi in background
  bool otaEnabled;   // true se, quando WiFi è attivo, abilitiamo anche OTA
  WifiCredential primary;
  WifiCredential secondary;
};

struct BatteryThresholds {
  float fullMin;
  float goodMin;
  float lowMin;
  float criticalMin;
};

struct PersistedSettings {
  ScaleSettings      scale;
  WifiSettings       wifi;
  BatteryThresholds  battery;
};

namespace Settings {
  // Valori di default (usati se NVS è vuoto o contiene dati invalidi)
  constexpr float DEFAULT_BATT_FULL_MIN     = 6.35f;
  constexpr float DEFAULT_BATT_GOOD_MIN     = 6.20f;
  constexpr float DEFAULT_BATT_LOW_MIN      = 6.05f;
  constexpr float DEFAULT_BATT_CRITICAL_MIN = 5.90f;

  // Carica tutti i parametri da NVS (namespace "minu_scale"), valorizzando out con default se necessario.
  void loadAll(PersistedSettings &out);

  // Salvataggi individuali (usano lo stesso namespace NVS).
  void saveScale(const ScaleSettings &scale);
  void saveWifi(const WifiSettings &wifi);
  void saveBattery(const BatteryThresholds &batt);
}
