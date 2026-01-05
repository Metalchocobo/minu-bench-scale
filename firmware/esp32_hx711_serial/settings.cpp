#include "settings.h"

#include <Preferences.h>
#include <math.h>

namespace {
  constexpr const char* NVS_NAMESPACE = "minu_scale";

  // Chiavi NVS
  constexpr const char* KEY_OFFSET_RAW   = "offset";
  constexpr const char* KEY_SCALE_CPG    = "scale";
  constexpr const char* KEY_REF_G        = "ref_g";

  constexpr const char* KEY_WIFI_ENABLED = "wifi_en";
  constexpr const char* KEY_OTA_ENABLED  = "ota_en";
  constexpr const char* KEY_WIFI1_SSID   = "wifi1_ssid";
  constexpr const char* KEY_WIFI1_PASS   = "wifi1_pwd";
  constexpr const char* KEY_WIFI2_SSID   = "wifi2_ssid";
  constexpr const char* KEY_WIFI2_PASS   = "wifi2_pwd";

  constexpr const char* KEY_BATT_FULL    = "batt_full";
  constexpr const char* KEY_BATT_GOOD    = "batt_good";
  constexpr const char* KEY_BATT_LOW     = "batt_low";
  constexpr const char* KEY_BATT_CRIT    = "batt_crit";

  String readStringPref(Preferences &prefs, const char* key) {
    String value = prefs.getString(key, "");
    value.trim();
    return value;
  }

  void sanitizeWifiConfig(WifiSettings &wifi) {
    // WiFi e OTA hanno senso solo se c'è almeno un SSID valido.
    bool hasAnySsid = wifi.primary.ssid.length() > 0 || wifi.secondary.ssid.length() > 0;
    if (!hasAnySsid) {
      wifi.wifiEnabled = false;
      wifi.otaEnabled = false;
    }
  }

  void ensureBatteryOrder(BatteryThresholds &batt) {
    // Se i valori in NVS sono incoerenti, ripristiniamo i default.
    if (!(batt.criticalMin < batt.lowMin && batt.lowMin < batt.goodMin && batt.goodMin < batt.fullMin)) {
      batt.fullMin     = Settings::DEFAULT_BATT_FULL_MIN;
      batt.goodMin     = Settings::DEFAULT_BATT_GOOD_MIN;
      batt.lowMin      = Settings::DEFAULT_BATT_LOW_MIN;
      batt.criticalMin = Settings::DEFAULT_BATT_CRITICAL_MIN;
    }
  }
} // namespace

namespace Settings {

void loadAll(PersistedSettings &out) {
  Preferences prefs;
  prefs.begin(NVS_NAMESPACE, true);

  // Scala
  out.scale.offsetRaw = prefs.getLong(KEY_OFFSET_RAW, 0);
  out.scale.scaleCpg  = prefs.getFloat(KEY_SCALE_CPG, NAN);
  out.scale.refG      = prefs.getLong(KEY_REF_G, 0);

  // WiFi/OTA
  out.wifi.wifiEnabled = prefs.getBool(KEY_WIFI_ENABLED, false);
  out.wifi.otaEnabled  = prefs.getBool(KEY_OTA_ENABLED, false);
  out.wifi.primary.ssid     = readStringPref(prefs, KEY_WIFI1_SSID);
  out.wifi.primary.password = readStringPref(prefs, KEY_WIFI1_PASS);
  out.wifi.secondary.ssid   = readStringPref(prefs, KEY_WIFI2_SSID);
  out.wifi.secondary.password = readStringPref(prefs, KEY_WIFI2_PASS);
  sanitizeWifiConfig(out.wifi);

  // Batteria
  out.battery.fullMin     = prefs.getFloat(KEY_BATT_FULL,    DEFAULT_BATT_FULL_MIN);
  out.battery.goodMin     = prefs.getFloat(KEY_BATT_GOOD,    DEFAULT_BATT_GOOD_MIN);
  out.battery.lowMin      = prefs.getFloat(KEY_BATT_LOW,     DEFAULT_BATT_LOW_MIN);
  out.battery.criticalMin = prefs.getFloat(KEY_BATT_CRIT,    DEFAULT_BATT_CRITICAL_MIN);
  ensureBatteryOrder(out.battery);

  prefs.end();

  // Fallback scala se non presente (seed di default)
  if (out.scale.scaleCpg <= 0.01f || isnan(out.scale.scaleCpg)) {
    out.scale.scaleCpg = NAN; // sarà sostituito dal chiamante con DEFAULT_CPG
  }
  if (out.scale.offsetRaw == 0) {
    out.scale.offsetRaw = 0; // lasciamo lo zero; il chiamante può gestire default/auto-tare
  }
  if (out.scale.refG <= 0) {
    out.scale.refG = 0;
  }
}

void saveScale(const ScaleSettings &scale) {
  Preferences prefs;
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putLong (KEY_OFFSET_RAW, scale.offsetRaw);
  prefs.putFloat(KEY_SCALE_CPG,  scale.scaleCpg);
  prefs.putLong (KEY_REF_G,      scale.refG);
  prefs.end();
}

void saveWifi(const WifiSettings &wifi) {
  Preferences prefs;
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putBool(KEY_WIFI_ENABLED, wifi.wifiEnabled);
  prefs.putBool(KEY_OTA_ENABLED,  wifi.otaEnabled);
  prefs.putString(KEY_WIFI1_SSID, wifi.primary.ssid);
  prefs.putString(KEY_WIFI1_PASS, wifi.primary.password);
  prefs.putString(KEY_WIFI2_SSID, wifi.secondary.ssid);
  prefs.putString(KEY_WIFI2_PASS, wifi.secondary.password);
  prefs.end();
}

void saveBattery(const BatteryThresholds &batt) {
  Preferences prefs;
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putFloat(KEY_BATT_FULL,    batt.fullMin);
  prefs.putFloat(KEY_BATT_GOOD,    batt.goodMin);
  prefs.putFloat(KEY_BATT_LOW,     batt.lowMin);
  prefs.putFloat(KEY_BATT_CRIT,    batt.criticalMin);
  prefs.end();
}

} // namespace Settings
