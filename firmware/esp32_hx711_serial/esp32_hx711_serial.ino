// =============================================================================
// ESP32 + HX711 + OLED SSD1322 + INA219 — Minù Bench Scale (Modular Refactor)
// =============================================================================
// Architettura modulare:
//   - config/config_pins.h   : Pin hardware
//   - config/config_audio.h  : Configurazione DFPlayer
//   - config/config_scale.h  : Parametri bilancia
//   - audio.h/cpp            : Modulo Audio (namespace Audio::)
//   - scale_filters.h/cpp    : Filtri (namespace ScaleFilters::)
//   - scale_state.h/cpp      : Stati (namespace ScaleState::)
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <driver/gpio.h>
#include <math.h>

// ========================= CONFIGURAZIONE =========================
#include "config/config_pins.h"
#include "config/config_audio.h"
#include "config/config_scale.h"
#include "config/config_battery.h"

// ========================= MODULI =========================
#include "audio.h"
#include "scale_filters.h"
#include "scale_state.h"

// ========================= DRIVER =========================
#include "hx711_driver.h"
#include "hx_health.h"
#include "battery_monitor.h"
#include "ui_display.h"
#include "keypad.h"
#include "buzzer.h"
#include "net_ota_cloud.h"
#include "wifi_store.h"
#include "ota_store.h"
#include "calibration_wizard.h"
#include "weigh_stack.h"

// ========================= ALIAS =========================
using namespace ScaleConfig;
using namespace AudioConfig;
using namespace BatteryConfig;

// ========================= VARIABILI GLOBALI =========================
// Necessaria per ui_display.cpp
bool stEnable = ST_ENABLE_DEFAULT;

// HX health
static HxHealth g_hxHealth;
static const HxHealthConfig HX_HEALTH_CFG = {
  .warnAfterMs       = HX_HEALTH_WARN_MS,
  .errorAfterMs      = HX_HEALTH_ERROR_MS,
  .hardAfterMs       = HX_HEALTH_HARD_MS,
  .okStableMs        = HX_HEALTH_OK_STABLE_MS,
  .lastValidMaxAgeMs = HX_HEALTH_LAST_VALID_MS
};

static bool g_hxBeginOk = false;
static bool g_isBooting = true;

// Ultimo valore raw filtrato (per calibrazione via seriale)
static long g_lastRawFiltered = 0;

// Timing
static uint32_t lastOledMs     = 0;
static uint32_t lastSampleMs   = 0;
static uint32_t g_lastKeyPressMs = 0;
static uint32_t g_lastZTtickMs = 0;

// Decimazione
static long     gWorkSum = 0;
static uint8_t  gWorkCnt = 0;
static long     gLiveSum = 0;
static uint8_t  gLiveCnt = 0;

// EMA Live
static float    gLiveEma        = 0.0f;
static bool     gLiveEmaInit    = false;
static bool     gLiveEmaPrevStEnable = true;

// Peso corrente
static float gLive = 0.0f;

// Inattività
enum InactivityStage : uint8_t { INACT_NONE = 0, INACT_ZZZ = 1 };
static InactivityStage g_inactivitySleepStage = INACT_NONE;
static uint32_t g_inactivityStageStartMs = 0;
static bool     g_inactivitySleepArmed = true;

// WiFi
#if ENABLE_WIFI_OTA
static bool g_wifiUserEnabled = true;
static bool g_wifiSetupDone   = false;
static bool g_wifiPrevConnected = false;
static uint32_t g_wifiErrLastMs = 0;
static uint32_t g_wifiAudioSuppressUntilMs = 0;
static bool g_wifiSuppressNextConnectAnnounce = false;
static const uint32_t WIFI_ERR_COOLDOWN_MS = 60000;
#endif

// ========================= WEIGH STACK: LONG PRESS & OVERLAY =========================
static const uint32_t LONG_PRESS_MS = 2000;  // 2 seconds for long press

// CLEAR key long press tracking
static uint32_t g_clearPressStartMs   = 0;
static bool     g_clearLongPressFired = false;

// Overlay state (temporary display for TOTAL/NET/CLEAR feedback)
enum OverlayType : uint8_t {
  OVERLAY_NONE = 0,
  OVERLAY_STACK_COMPARE,
  OVERLAY_CLEAR_FEEDBACK
};
static OverlayType g_overlayType      = OVERLAY_NONE;
static uint32_t    g_overlayStartMs   = 0;
static const uint32_t OVERLAY_TIMEOUT_MS = 10000; // 10 seconds auto-dismiss

// Cached overlay values (captured at overlay start)
static long g_overlayTotal    = 0;
static long g_overlayNet      = 0;
static int  g_overlayCount    = 0;
static char g_overlayMsg[12]  = {0};

// Manual TARE reference capture: save ref offset only after tare is applied
static bool g_captureReferenceAfterTare = false;

// Runtime recovery survives WDT resets but is lost on power loss.
struct RuntimeTareRecovery {
  uint32_t magic;
  uint16_t version;
  uint16_t valid;
  long offsetRaw;
  long ztCounts;
  uint32_t crc;
};

static const uint32_t RECOVERY_MAGIC = 0x4D545245UL;  // MTRE
static const uint16_t RECOVERY_VERSION = 1;
RTC_NOINIT_ATTR static RuntimeTareRecovery g_runtimeTareRecovery;

static void showStackCompareOverlay(uint32_t nowMs) {
  float stackTotal = WeighStack::total();
  long totalDisp = (stackTotal >= 0.0f) ? (long)(stackTotal + 0.5f) : (long)(stackTotal - 0.5f);

  long netDisp = 0;
  if (WeighStack::hasReference()) {
    long refOffset = WeighStack::getReferenceOffset();
    float cpg = ScaleState::getScaleCpg();
    long rawNow = ScaleState::getLastRawAvg();
    float netG = (cpg > 0.01f) ? ((float)(rawNow - refOffset) / cpg) : 0.0f;
    netDisp = (netG >= 0.0f) ? (long)(netG + 0.5f) : (long)(netG - 0.5f);
  } else {
    netDisp = (long)(stEnable ? ScaleState::getDispWorkLast() : ScaleState::getDispLiveLast());
    Serial.println(F("[STACK] No reference yet, NET fallback to current display"));
  }

  g_overlayType = OVERLAY_STACK_COMPARE;
  g_overlayStartMs = nowMs;
  g_overlayTotal = totalDisp;
  g_overlayNet = netDisp;
  g_overlayCount = WeighStack::count();

  // Render subito per feedback immediato anche se la UI TARE e' attiva
  ui_renderStackCompare(g_overlayTotal, g_overlayNet, g_overlayCount);
  lastOledMs = nowMs;

  Serial.print(F("[STACK] TOT=")); Serial.print(totalDisp);
  Serial.print(F("g NET=")); Serial.print(netDisp);
  Serial.print(F("g DELTA=")); Serial.print(netDisp - totalDisp);
  Serial.println(F("g"));
}

// Log HX
static bool     g_hxLogEnabled  = false;
static uint32_t g_hxLogPeriodMs = 250;
static uint32_t g_hxLogLastMs   = 0;

// Debug batteria
static uint32_t lastBattDebug = 0;

// ========================= HELPER SLEEP LED =========================
static inline void sleepLedOn()  { digitalWrite(SLEEP_LED_PIN, SLEEP_LED_ACTIVE_HIGH ? HIGH : LOW); }
static inline void sleepLedOff() { digitalWrite(SLEEP_LED_PIN, SLEEP_LED_ACTIVE_HIGH ? LOW : HIGH); }

// ========================= FORWARD DECLARATIONS =========================
void printHelp();
void serial_task();
void parseCommand(const char* cmd);
void handleKeyEvent(KeyCode key);
bool autoTareOnBoot(uint16_t maxSamples);
bool initHX711();
long readRawHXOnceBlocking(uint32_t timeoutMs);
static void runtimeTareRecoverySave();
static bool runtimeTareRecoveryTryRestore();
static void runtimeTareRecoveryClear();
static void setupLoopWatchdog();
static void enterInactivityLightSleep();
static bool maybeEnterSafeShutdown(uint32_t nowMs);

#if ENABLE_WIFI_OTA
static bool loadWifiUserEnabledFromNVS();
static void saveWifiUserEnabledToNVS(bool en);
static void wifiAudioUpdate(uint32_t now);
#endif

// ========================= RUNTIME TARE RECOVERY =========================
static uint32_t recoveryMix(uint32_t h, uint32_t v) {
  h ^= v;
  h *= 16777619UL;
  return h;
}

static uint32_t runtimeTareRecoveryCrc(const RuntimeTareRecovery& r) {
  uint32_t h = 2166136261UL;
  h = recoveryMix(h, r.magic);
  h = recoveryMix(h, ((uint32_t)r.version << 16) | r.valid);
  h = recoveryMix(h, (uint32_t)r.offsetRaw);
  h = recoveryMix(h, (uint32_t)r.ztCounts);
  return h;
}

static bool runtimeTareRecoveryIsWdtReset() {
  esp_reset_reason_t reason = esp_reset_reason();
  return reason == ESP_RST_WDT ||
         reason == ESP_RST_TASK_WDT ||
         reason == ESP_RST_INT_WDT;
}

static bool runtimeTareRecoveryIsValid() {
  if (g_runtimeTareRecovery.magic != RECOVERY_MAGIC) return false;
  if (g_runtimeTareRecovery.version != RECOVERY_VERSION) return false;
  if (g_runtimeTareRecovery.valid != 1) return false;
  return g_runtimeTareRecovery.crc == runtimeTareRecoveryCrc(g_runtimeTareRecovery);
}

static void runtimeTareRecoveryClear() {
  g_runtimeTareRecovery.magic = RECOVERY_MAGIC;
  g_runtimeTareRecovery.version = RECOVERY_VERSION;
  g_runtimeTareRecovery.valid = 0;
  g_runtimeTareRecovery.offsetRaw = 0;
  g_runtimeTareRecovery.ztCounts = 0;
  g_runtimeTareRecovery.crc = runtimeTareRecoveryCrc(g_runtimeTareRecovery);
}

static void runtimeTareRecoverySave() {
  g_runtimeTareRecovery.magic = RECOVERY_MAGIC;
  g_runtimeTareRecovery.version = RECOVERY_VERSION;
  g_runtimeTareRecovery.valid = 1;
  g_runtimeTareRecovery.offsetRaw = ScaleState::getOffsetRaw();
  g_runtimeTareRecovery.ztCounts = ScaleState::getZtCounts();
  g_runtimeTareRecovery.crc = runtimeTareRecoveryCrc(g_runtimeTareRecovery);
}

static bool runtimeTareRecoveryTryRestore() {
  if (!runtimeTareRecoveryIsWdtReset()) {
    runtimeTareRecoveryClear();
    return false;
  }
  if (!runtimeTareRecoveryIsValid()) return false;

  ScaleState::setOffsetRaw(g_runtimeTareRecovery.offsetRaw);
  ScaleState::setZtCounts(g_runtimeTareRecovery.ztCounts);
  ScaleState::resetFiltersAndState();
  return true;
}

// ========================= WATCHDOG =========================
static void setupLoopWatchdog() {
  esp_task_wdt_config_t wdtCfg = {
    .timeout_ms = LOOP_WDT_TIMEOUT_MS,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdtCfg);
  esp_task_wdt_add(NULL);
  Serial.println(F("[WDT] Task Watchdog attivo (8s)"));
}

// ========================= PRINT HELP =========================
void printHelp() {
  Serial.println(F("\n--- COMANDI SERIALI ---"));
  Serial.println(F("hxlog on/off, hxlog rate <ms>"));
  Serial.println(F("mp3 <track> [sec], stop, vol <0-30>, mp3 status"));
  Serial.println(F("wifi set <slot> \"SSID\" \"PASS\", wifi creds, wifi apply"));
  Serial.println(F("ota status, ota set \"PASS\", ota clear"));
  Serial.println(F("mqtt set \"host\" \"user\" \"pass\", mqtt creds, mqtt clear, mqtt apply, mqtt status"));
  Serial.println(F("cal status, cal zero, cal ref <g>, cal save, cal load"));
  Serial.println(F("stack [status], stack list, stack clear"));
  Serial.println(F("-----------------------\n"));
}

// ========================= SERIAL TASK =========================
static char g_cmdBuf[128];
static uint8_t g_cmdLen = 0;

void serial_task() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (g_cmdLen > 0) {
        g_cmdBuf[g_cmdLen] = '\0';
        parseCommand(g_cmdBuf);
        g_cmdLen = 0;
      }
    } else if (g_cmdLen < sizeof(g_cmdBuf) - 1) {
      g_cmdBuf[g_cmdLen++] = c;
    }
  }
}

void parseCommand(const char* cmd) {
  // hxlog
  if (strncmp(cmd, "hxlog ", 6) == 0) {
    const char* arg = cmd + 6;
    if (strcmp(arg, "on") == 0) {
      g_hxLogEnabled = true;
      Serial.println(F("[CMD] hxlog ON"));
    } else if (strcmp(arg, "off") == 0) {
      g_hxLogEnabled = false;
      Serial.println(F("[CMD] hxlog OFF"));
    } else if (strncmp(arg, "rate ", 5) == 0) {
      int r = atoi(arg + 5);
      if (r >= 50 && r <= 5000) {
        g_hxLogPeriodMs = r;
        Serial.print(F("[CMD] hxlog rate "));
        Serial.println(r);
      }
    }
    return;
  }

  // mp3
  if (strncmp(cmd, "mp3 ", 4) == 0) {
    const char* arg = cmd + 4;
    if (strcmp(arg, "status") == 0) {
      Audio::debugStatus();
    } else {
      int track = 0, sec = 0;
      int n = sscanf(arg, "%d %d", &track, &sec);
      if (n >= 1 && track > 0) {
        Audio::requestPlayMp3((uint16_t)track, (uint32_t)sec);
      }
    }
    return;
  }

  if (strcmp(cmd, "stop") == 0) {
    Audio::stopNow(true);
    Serial.println(F("[CMD] stop"));
    return;
  }

  if (strncmp(cmd, "vol ", 4) == 0) {
    int v = atoi(cmd + 4);
    Audio::setVolume((uint8_t)v);
    Serial.print(F("[CMD] vol "));
    Serial.println(v);
    return;
  }

#if ENABLE_WIFI_OTA
  // wifi
  if (strncmp(cmd, "wifi ", 5) == 0) {
    const char* arg = cmd + 5;

    if (strncmp(arg, "set ", 4) == 0) {
      int slot = 0;
      char ssid[64] = {0}, pass[64] = {0};
      const char* p = arg + 4;
      slot = atoi(p);
      while (*p && *p != ' ') p++;
      while (*p == ' ') p++;

      if (*p == '"') {
        p++;
        int i = 0;
        while (*p && *p != '"' && i < 63) ssid[i++] = *p++;
        if (*p == '"') p++;
        while (*p == ' ') p++;
        if (*p == '"') {
          p++;
          i = 0;
          while (*p && *p != '"' && i < 63) pass[i++] = *p++;
        }
      }

      if (slot >= 1 && slot <= 2 && strlen(ssid) > 0) {
        WifiStore::saveSlot(slot, ssid, pass);
        Serial.print(F("[WIFI] Slot "));
        Serial.print(slot);
        Serial.println(F(" salvato"));
      }
      return;
    }

    if (strncmp(arg, "creds", 5) == 0) {
      char s1[64], p1[64], s2[64], p2[64];
      WifiStore::loadSlot(1, s1, 64, p1, 64);
      WifiStore::loadSlot(2, s2, 64, p2, 64);
      Serial.print(F("Slot 1: ")); Serial.println(s1[0] ? s1 : "(vuoto)");
      Serial.print(F("Slot 2: ")); Serial.println(s2[0] ? s2 : "(vuoto)");

      if (strstr(arg, "showpass 1")) {
        Serial.print(F("Pass 1: ")); Serial.println(p1);
      } else if (strstr(arg, "showpass 2")) {
        Serial.print(F("Pass 2: ")); Serial.println(p2);
      }
      return;
    }

    if (strncmp(arg, "clear ", 6) == 0) {
      const char* what = arg + 6;
      if (strcmp(what, "all") == 0) {
        WifiStore::clearSlot(1);
        WifiStore::clearSlot(2);
        Serial.println(F("[WIFI] Tutte le credenziali cancellate"));
      } else {
        int slot = atoi(what);
        if (slot >= 1 && slot <= 2) {
          WifiStore::clearSlot(slot);
          Serial.print(F("[WIFI] Slot ")); Serial.print(slot); Serial.println(F(" cancellato"));
        }
      }
      return;
    }

    if (strcmp(arg, "apply") == 0) {
      Net::wifiReloadCredsAndRestart();
      Serial.println(F("[WIFI] Credenziali ricaricate"));
      return;
    }
  }

  // ota
  if (strncmp(cmd, "ota ", 4) == 0) {
    const char* arg = cmd + 4;

    if (strcmp(arg, "status") == 0) {
      char hash[33] = {0};
      if (OtaStore::loadHash(hash, sizeof(hash))) {
        Serial.println(F("[OTA] Password configurata (hash MD5)"));
      } else {
        Serial.println(F("[OTA] Nessuna password"));
      }
      return;
    }

    if (strncmp(arg, "set ", 4) == 0) {
      const char* p = arg + 4;
      if (*p == '"') {
        p++;
        char pass[64] = {0};
        int i = 0;
        while (*p && *p != '"' && i < 63) pass[i++] = *p++;
        OtaStore::savePassword(pass);
        Serial.println(F("[OTA] Password salvata (richiede reboot)"));
        if (strstr(arg, "reboot")) {
          delay(500);
          ESP.restart();
        }
      }
      return;
    }

    if (strncmp(arg, "clear", 5) == 0) {
      OtaStore::clear();
      Serial.println(F("[OTA] Password cancellata (richiede reboot)"));
      if (strstr(arg, "reboot")) {
        delay(500);
        ESP.restart();
      }
      return;
    }

    if (strcmp(arg, "reboot") == 0) {
      Serial.println(F("[OTA] Reboot..."));
      delay(500);
      ESP.restart();
      return;
    }
  }
#endif

#if ENABLE_MQTT
  // mqtt
  if (strncmp(cmd, "mqtt ", 5) == 0) {
    const char* arg = cmd + 5;

    if (strncmp(arg, "set ", 4) == 0) {
      // Formato: mqtt set "host" "user" "pass"
      const char* p = arg + 4;
      while (*p == ' ') p++;

      char host[65] = {0}, user[33] = {0}, pass[65] = {0};

      // Parse host
      if (*p == '"') {
        p++;
        int i = 0;
        while (*p && *p != '"' && i < 64) host[i++] = *p++;
        if (*p == '"') p++;
        while (*p == ' ') p++;
      }

      // Parse user
      if (*p == '"') {
        p++;
        int i = 0;
        while (*p && *p != '"' && i < 32) user[i++] = *p++;
        if (*p == '"') p++;
        while (*p == ' ') p++;
      }

      // Parse pass
      if (*p == '"') {
        p++;
        int i = 0;
        while (*p && *p != '"' && i < 64) pass[i++] = *p++;
      }

      if (strlen(host) > 0) {
        MqttStore::save(host, user, pass);
        Serial.print(F("[MQTT] Credenziali salvate. Host: "));
        Serial.println(host);
        Serial.println(F("[MQTT] Usa 'mqtt apply' per applicare senza reboot"));
      } else {
        Serial.println(F("[MQTT] Errore: host vuoto. Formato: mqtt set \"host\" \"user\" \"pass\""));
      }
      return;
    }

    if (strncmp(arg, "creds", 5) == 0) {
      char h[65] = {0}, u[33] = {0}, pw[65] = {0};
      bool ok = MqttStore::load(h, 65, u, 33, pw, 65);
      if (ok) {
        Serial.print(F("[MQTT] Host: ")); Serial.println(h);
        Serial.print(F("[MQTT] User: ")); Serial.println(u[0] ? u : "(vuoto)");
        if (strstr(arg, "showpass")) {
          Serial.print(F("[MQTT] Pass: ")); Serial.println(pw);
        } else {
          // Password mascherata
          size_t n = strlen(pw);
          size_t stars = (n == 0) ? 0 : (n <= 8 ? n : 8);
          char masked[9] = {0};
          for (size_t i = 0; i < stars; i++) masked[i] = '*';
          Serial.print(F("[MQTT] Pass: ")); Serial.println(masked);
        }
      } else {
        Serial.println(F("[MQTT] Nessuna credenziale configurata"));
      }
      return;
    }

    if (strncmp(arg, "clear", 5) == 0) {
      MqttStore::clear();
      Serial.println(F("[MQTT] Credenziali cancellate (richiede reboot o mqtt apply)"));
      return;
    }

    if (strcmp(arg, "apply") == 0) {
      Net::mqttReloadCreds();
      Serial.println(F("[MQTT] Credenziali ricaricate"));
      return;
    }

    if (strcmp(arg, "status") == 0) {
      Serial.println(F("[MQTT] --- Stato MQTT ---"));
      Serial.print(F("  Configurato: ")); Serial.println(Net::isMqttConfigured() ? "si" : "no");
      Serial.print(F("  Connesso:    ")); Serial.println(Net::isMqttConnected() ? "si" : "no");
      Serial.print(F("  Scale ID:    ")); Serial.println(Net::getScaleId());
      Serial.print(F("  Cmd attivo:  ")); Serial.println(Net::isMqttCommandActive() ? "si" : "no");
      if (Net::isMqttCommandActive()) {
        Serial.print(F("  UUID:        ")); Serial.println(Net::getMqttCommandUuid());
        Serial.print(F("  Target (g):  ")); Serial.println(Net::getMqttTargetWeight(), 1);
      }
      return;
    }

    Serial.println(F("[MQTT] Comandi: mqtt set \"host\" \"user\" \"pass\" | mqtt creds [showpass] | mqtt clear | mqtt apply | mqtt status"));
    return;
  }
#endif

  // stack
  if (strncmp(cmd, "stack", 5) == 0) {
    const char* arg = cmd + 5;
    while (*arg == ' ') arg++;

    if (strcmp(arg, "status") == 0 || *arg == '\0') {
      Serial.println(F("[STACK] --- Stato stack ---"));
      Serial.print(F("  Count: ")); Serial.println(WeighStack::count());
      Serial.print(F("  Total: ")); Serial.print(WeighStack::total(), 0); Serial.println(F("g"));
      Serial.print(F("  Ref:   ")); Serial.println(WeighStack::hasReference() ? "si" : "no");
      if (WeighStack::hasReference()) {
        Serial.print(F("  RefOff: ")); Serial.println(WeighStack::getReferenceOffset());
      }
      return;
    }

    if (strcmp(arg, "list") == 0) {
      int n = WeighStack::count();
      Serial.print(F("[STACK] ")); Serial.print(n); Serial.println(F(" items:"));
      for (int i = 0; i < n; i++) {
        Serial.print(F("  [")); Serial.print(i);
        Serial.print(F("] ")); Serial.print(WeighStack::get(i), 0);
        Serial.println(F("g"));
      }
      Serial.print(F("  TOTAL: ")); Serial.print(WeighStack::total(), 0); Serial.println(F("g"));
      return;
    }

    if (strcmp(arg, "clear") == 0) {
      WeighStack::clear();
      Serial.println(F("[STACK] Cleared"));
      return;
    }

    Serial.println(F("[STACK] Comandi: stack [status] | stack list | stack clear"));
    return;
  }

  // cal (calibrazione)
  if (strncmp(cmd, "cal ", 4) == 0) {
    const char* arg = cmd + 4;

    if (strcmp(arg, "status") == 0) {
      Serial.println(F("[CAL] --- Stato calibrazione ---"));
      Serial.print(F("  Offset:   ")); Serial.println(ScaleState::getOffsetRaw());
      Serial.print(F("  CPG:      ")); Serial.println(ScaleState::getScaleCpg(), 4);
      Serial.print(F("  ZT:       ")); Serial.println(ScaleState::getZtCounts());
      Serial.print(F("  RawFilt:  ")); Serial.println(g_lastRawFiltered);
      float cpg = ScaleState::getScaleCpg();
      long off = ScaleState::effectiveOffsetCounts();
      float grams = (cpg > 0.01f) ? ((float)(g_lastRawFiltered - off) / cpg) : 0.0f;
      Serial.print(F("  Grammi:   ")); Serial.println(grams, 1);
      return;
    }

    if (strcmp(arg, "zero") == 0) {
      if (hxHealth_isError(&g_hxHealth)) {
        Serial.println(F("[CAL] Errore: HX in stato ERROR, calibrazione disabilitata"));
        return;
      }
      long newOffset = g_lastRawFiltered;
      ScaleState::setOffsetRaw(newOffset);
      ScaleState::setZtCounts(0);
      ScaleState::resetFiltersAndState();
      runtimeTareRecoverySave();
      Serial.print(F("[CAL] Zero impostato. Nuovo OFFSET=")); Serial.println(newOffset);
      Serial.println(F("[CAL] Usa 'cal save' per salvare in NVS"));
      return;
    }

    if (strncmp(arg, "ref ", 4) == 0) {
      if (hxHealth_isError(&g_hxHealth)) {
        Serial.println(F("[CAL] Errore: HX in stato ERROR, calibrazione disabilitata"));
        return;
      }
      float refGrams = atof(arg + 4);
      if (refGrams <= 0.0f) {
        Serial.println(F("[CAL] Errore: peso di riferimento deve essere > 0"));
        return;
      }
      long offset = ScaleState::getOffsetRaw();
      long delta = g_lastRawFiltered - offset;
      if (delta == 0) {
        Serial.println(F("[CAL] Errore: delta=0, posiziona un peso sulla bilancia"));
        return;
      }
      float newCpg = (float)delta / refGrams;

      // Validazione CPG (stesse soglie del wizard)
      if (newCpg < CalWizard::CPG_MIN || newCpg > CalWizard::CPG_MAX) {
        Serial.print(F("[CAL] ATTENZIONE: CPG fuori range tipico ("));
        Serial.print(CalWizard::CPG_MIN, 0);
        Serial.print(F("-"));
        Serial.print(CalWizard::CPG_MAX, 0);
        Serial.print(F("): "));
        Serial.println(newCpg, 4);
        Serial.println(F("[CAL] Verifica il peso di riferimento e riprova."));
        return;
      }

      ScaleState::setScaleCpg(newCpg);
      ScaleState::resetFiltersAndState();
      Serial.print(F("[CAL] CPG calcolato: ")); Serial.println(newCpg, 4);
      Serial.println(F("[CAL] Usa 'cal save' per salvare in NVS"));
      return;
    }

    if (strcmp(arg, "save") == 0) {
      ScaleState::saveToNVS();
      Serial.println(F("[CAL] Calibrazione salvata in NVS"));
      return;
    }

    if (strcmp(arg, "load") == 0) {
      ScaleState::loadFromNVS();
      ScaleState::resetFiltersAndState();
      runtimeTareRecoverySave();
      Serial.println(F("[CAL] Calibrazione caricata da NVS"));
      return;
    }

    Serial.println(F("[CAL] Comandi: cal status | cal zero | cal ref <grammi> | cal save | cal load"));
    return;
  }

  Serial.print(F("[CMD] Sconosciuto: "));
  Serial.println(cmd);
}

// ========================= NVS WIFI USER PREF =========================
#if ENABLE_WIFI_OTA
static bool loadWifiUserEnabledFromNVS() {
  Preferences prefs;
  prefs.begin("minu_sys", true);
  bool en = prefs.getBool("wifi_user", true);
  prefs.end();
  return en;
}

static void saveWifiUserEnabledToNVS(bool en) {
  Preferences prefs;
  prefs.begin("minu_sys", false);
  prefs.putBool("wifi_user", en);
  prefs.end();
}
#endif

// ========================= KEYPAD HANDLER =========================
void handleKeyEvent(KeyCode key) {
  switch (key) {
    case KEY_TARE:
      Serial.println(F("[KEYPAD] TARE pressed"));
      buzzerKeyClick();

      // Dismiss overlay if active
      if (g_overlayType != OVERLAY_NONE) {
        g_overlayType = OVERLAY_NONE;
        lastOledMs = 0;
      }

      if (hxHealth_isError(&g_hxHealth)) {
        Serial.println(F("[TARE] Disabilitata (HX error)"));
        buzzerError();
        return;
      }

      // Mark reference capture to happen after tare is actually applied
      g_captureReferenceAfterTare = true;

      // Reset weigh stack (new weighing session)
      WeighStack::clear();
      Serial.println(F("[STACK] Stack cleared (new session)"));

      ScaleState::tareStart(millis());
      break;

    case KEY_MODE:
      Serial.println(F("[KEYPAD] MODE pressed"));
      buzzerKeyClick();
      stEnable = !stEnable;
      ScaleState::setStEnable(stEnable);
      ScaleState::setZtEnable(stEnable);
      ScaleState::resetFiltersAndState();

      if (stEnable) {
        Audio::requestPlayMp3(Track::MODE_WORK);
        Serial.println(F("[MODE] WORK"));
      } else {
        Audio::requestPlayMp3(Track::MODE_LIVE);
        Serial.println(F("[MODE] LIVE"));
      }
      break;

#if ENABLE_WIFI_OTA
    case KEY_WIFI: {
      Serial.println(F("[KEYPAD] WIFI pressed"));
      buzzerKeyClick();

      g_wifiUserEnabled = !g_wifiUserEnabled;
      saveWifiUserEnabledToNVS(g_wifiUserEnabled);

      if (g_wifiUserEnabled) {
        Audio::requestPlayMp3(Track::WIFI_MODULE_ON);
        Serial.println(F("[WIFI] ON"));
        if (!g_wifiSetupDone) {
          Net::wifiSetup();
          g_wifiSetupDone = true;
        } else {
          Net::wifiResume();
        }
      } else {
        Audio::requestPlayMp3(Track::WIFI_MODULE_OFF);
        Serial.println(F("[WIFI] OFF"));
        Net::wifiSuspend();
      }
      break;
    }
#endif

    case KEY_SLEEP: {
      Serial.println(F("[KEYPAD] SLEEP pressed"));
      buzzerKeyClick();

      ui_renderSleepZzz();
      Audio::requestPlayMp3(Track::SLEEP_ENTER);
      g_inactivitySleepStage = INACT_ZZZ;
      g_inactivityStageStartMs = g_lastKeyPressMs;
      lastOledMs = 0;
      break;
    }

    case KEY_ENTER: {
      Serial.println(F("[KEYPAD] ENTER pressed"));
      buzzerKeyClick();

      // Dismiss overlay if active
      if (g_overlayType != OVERLAY_NONE) {
        g_overlayType = OVERLAY_NONE;
        lastOledMs = 0;
      }

      // Read current display weight
      float currentWeight = (float)(stEnable ? ScaleState::getDispWorkLast() : ScaleState::getDispLiveLast());

      // If weight <= 0, ignore everything (no push, no tare, no MQTT)
      if (currentWeight <= 0.0f) {
        Serial.println(F("[STACK] Weight <= 0, ignoring ENTER"));
        buzzerWarn();
        break;
      }

      // Push to stack
      bool pushed = WeighStack::push(currentWeight);
      if (pushed) {
        Serial.print(F("[STACK] Push: "));
        Serial.print(currentWeight, 0);
        Serial.print(F("g (count="));
        Serial.print(WeighStack::count());
        Serial.println(F(")"));
      } else {
        Serial.println(F("[STACK] Full, cannot push"));
        buzzerError();
      }

      Audio::requestPlayMp3(Track::ENTER_PRESSED);

      // MQTT confirm (if active)
#if ENABLE_MQTT
      if (pushed && Net::isMqttCommandActive() && Net::isMqttConnected()) {
        // Send exactly the registered weight to Laravel payload actual_weight
        float registeredWeight = WeighStack::get(WeighStack::count() - 1);
        Net::mqttPublishConfirm(registeredWeight);
        Net::mqttClearActiveCommand();
      }
#endif

      // Auto-tare (working tare: does NOT reset stack, does NOT update reference)
      if (!hxHealth_isError(&g_hxHealth)) {
        ScaleState::tareStart(millis());
        Serial.println(F("[STACK] Auto-tare (working)"));
      }
      break;
    }

    case KEY_SKIP:
      Serial.println(F("[KEYPAD] SKIP pressed"));
      buzzerKeyClick();
#if ENABLE_MQTT
      if (Net::isMqttCommandActive() && Net::isMqttConnected()) {
        Audio::requestPlayMp3(Track::SKIP_PRESSED);
        Net::mqttPublishSkip();
        Net::mqttClearActiveCommand();
      } else if (Net::isMqttConnected() && !Net::isMqttCommandActive()) {
        // Nessun comando attivo: bip di avviso sordo
        buzzerWarn();
      } else {
        Audio::requestPlayMp3(Track::SKIP_PRESSED);
      }
#else
      Audio::requestPlayMp3(Track::SKIP_PRESSED);
#endif
      break;

    case KEY_TOTAL:
      Serial.println(F("[KEYPAD] TOTAL pressed"));
      buzzerKeyClick();

      if (g_overlayType == OVERLAY_STACK_COMPARE) {
        // Second press: dismiss overlay
        g_overlayType = OVERLAY_NONE;
        lastOledMs = 0;
        Serial.println(F("[STACK] TOTAL dismiss overlay"));
      } else {
        showStackCompareOverlay(millis());
        Audio::requestPlayMp3(Track::TOTAL_PRESSED);
      }
      break;

    case KEY_CLEAR:
      Serial.println(F("[KEYPAD] CLEAR pressed (tracking long press)"));
      buzzerKeyClick();
      // Start long press tracking; short/long action handled in loop
      g_clearPressStartMs = millis();
      g_clearLongPressFired = false;
      break;

    default:
      break;
  }
}

// ========================= AUTO-TARE BOOT =========================
bool autoTareOnBoot(uint16_t maxSamples) {
  if (!AUTO_TARE_ON_BOOT) {
    Serial.println(F("[BOOT] Auto-TARE disattivata."));
    return true;
  }

  Serial.println(F("[BOOT] Auto-TARE (robusta) ..."));
  delay(AUTO_TARE_SETTLE_MS);

  const uint16_t minSamples = AUTO_TARE_MIN_SAMPLES;
  const uint16_t MAX_BUF = 64;
  if (maxSamples < minSamples) maxSamples = minSamples;
  if (maxSamples > MAX_BUF)    maxSamples = MAX_BUF;

  long buf[MAX_BUF];
  uint16_t n = 0;
  uint32_t quietStart = 0;

  while (n < maxSamples) {
    long r = readRawHXOnceBlocking(200);
    if (r == 0) {
      Serial.println(F("[BOOT] Auto-TARE FAIL (timeout)"));
      return false;
    }

    buf[n++] = r;

    if (n >= minSamples) {
      long mn = buf[n - minSamples], mx = mn;
      for (uint16_t i = n - minSamples; i < n; i++) {
        long v = buf[i];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
      }

      float rangeG = 999999.0f;
      float cpg = ScaleState::getScaleCpg();
      if (cpg > 0.01f) rangeG = (float)(mx - mn) / cpg;

      if (rangeG <= AUTO_TARE_RANGE_G) {
        if (quietStart == 0) quietStart = millis();
        if (millis() - quietStart >= AUTO_TARE_QUIET_MS) break;
      } else {
        quietStart = 0;
      }
    }
  }

  // Trimmed mean
  long tmp[MAX_BUF];
  for (uint16_t i = 0; i < n; i++) tmp[i] = buf[i];

  for (uint16_t i = 1; i < n; i++) {
    long key = tmp[i];
    int j = (int)i - 1;
    while (j >= 0 && tmp[j] > key) {
      tmp[j + 1] = tmp[j];
      j--;
    }
    tmp[j + 1] = key;
  }

  uint16_t trim = (n >= 40) ? (n / 10) : 0;
  uint16_t i0 = trim, i1 = n - trim;

  int64_t s = 0;
  uint16_t useN = 0;
  for (uint16_t i = i0; i < i1; i++) {
    s += (int64_t)tmp[i];
    useN++;
  }
  if (useN == 0) {
    Serial.println(F("[BOOT] Auto-TARE FAIL (useN=0)"));
    return false;
  }

  long rawZero = (long)(s / (int64_t)useN);
  ScaleState::setOffsetRaw(rawZero);
  ScaleState::setZtCounts(0);
  ScaleState::resetFiltersAndState();

  Serial.print(F("[BOOT] Auto-TARE OK. OFFSET_RAW="));
  Serial.println(rawZero);

  return true;
}

// ========================= INIT HX711 =========================
bool initHX711() {
  Serial.println(F("[HX711] Init..."));
  g_hxBeginOk = false;

  hx711_begin(HX711_DOUT_PIN, HX711_SCK_PIN, HX711_GAIN_128_A, HX711_PULSE_US);

  uint32_t t0 = millis();
  while (millis() - t0 < HX711_INIT_TIMEOUT_MS) {
    if (hx711_is_ready()) {
      (void)hx711_read();
      g_hxBeginOk = true;
      Serial.println(F("[HX711] OK"));
      return true;
    }
    delay(10);
  }

  Serial.println(F("[HX711] FAIL - DOUT non pronto"));
  return false;
}

long readRawHXOnceBlocking(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (hx711_is_ready()) {
      return hx711_read();
    }
    delay(2);
  }
  return 0;
}

// ========================= WIFI AUDIO UPDATE =========================
#if ENABLE_WIFI_OTA
static void wifiAudioUpdate(uint32_t now) {
  if (g_isBooting) {
    g_wifiPrevConnected = WiFi.isConnected();
    return;
  }
  if (!g_wifiUserEnabled) {
    g_wifiPrevConnected = false;
    return;
  }

  if (g_wifiAudioSuppressUntilMs != 0 && (int32_t)(now - g_wifiAudioSuppressUntilMs) < 0) {
    const bool c = WiFi.isConnected();
    g_wifiPrevConnected = c;
    if (c && g_wifiSuppressNextConnectAnnounce) g_wifiSuppressNextConnectAnnounce = false;
    return;
  }
  if (g_wifiAudioSuppressUntilMs != 0) g_wifiAudioSuppressUntilMs = 0;

  const bool connectedNow = WiFi.isConnected();
  if (connectedNow != g_wifiPrevConnected) {
    g_wifiPrevConnected = connectedNow;
    if (connectedNow) {
      if (g_wifiSuppressNextConnectAnnounce) {
        g_wifiSuppressNextConnectAnnounce = false;
      } else {
        Audio::requestPlayMp3(Track::WIFI_CONNECTED);
      }
    }
  }

  if (!connectedNow) {
    wl_status_t st = WiFi.status();
    if (st == WL_CONNECT_FAILED || st == WL_NO_SSID_AVAIL) {
      if (g_wifiErrLastMs == 0 || (now - g_wifiErrLastMs) >= WIFI_ERR_COOLDOWN_MS) {
        Audio::requestPlayMp3(Track::WIFI_ERROR);
        g_wifiErrLastMs = now;
      }
    }
  }
}
#endif

// ========================= BOOT HELPERS =========================
static void bootPump(uint32_t ms) {
  uint32_t start = millis();
  while (millis() - start < ms) {
    esp_task_wdt_reset();
#if ENABLE_WIFI_OTA
    Net::update();
    wifiAudioUpdate(millis());
#endif
    uint32_t now = millis();
    keypad_update(now);
    Audio::task(now);
    (void)keypad_get_event();
    delay(10);
  }
}

static void bootShow(const char* line1, const char* line2, uint16_t holdMs = 350) {
  ui_showBoot(line1, line2);
  bootPump(holdMs);
}

static void bootWaitTare() {
  while (true) {
    esp_task_wdt_reset();
    uint32_t now = millis();
    keypad_update(now);
    Audio::task(now);
#if ENABLE_WIFI_OTA
    Net::update();
    wifiAudioUpdate(now);
#endif
    KeyCode k = keypad_get_event();
    if (k == KEY_TARE) {
      buzzerKeyClick();
      return;
    }
    delay(10);
  }
}

// ========================= INACTIVITY SLEEP =========================
static void enterInactivityLightSleep() {
  Audio::powerOffNow();
  sleepLedOn();

#if ENABLE_MQTT
  Net::mqttSuspend();
#endif
#if ENABLE_WIFI_OTA
  bool wasWifiOn = g_wifiUserEnabled && g_wifiSetupDone;
  if (wasWifiOn) Net::wifiSuspend();
#endif

  const int* rows = nullptr; int nRows = 0;
  const int* cols = nullptr; int nCols = 0;
  keypad_get_pins(&rows, &nRows, &cols, &nCols);

  for (int i = 0; i < nRows; i++) {
    pinMode(rows[i], OUTPUT);
    digitalWrite(rows[i], LOW);
  }
  for (int i = 0; i < nCols; i++) {
    pinMode(cols[i], INPUT_PULLUP);
    gpio_wakeup_enable((gpio_num_t)cols[i], GPIO_INTR_LOW_LEVEL);
  }

  ui_powerSave(true);

  esp_sleep_enable_gpio_wakeup();

  delay(20);
  esp_light_sleep_start();

  // Wake
  sleepLedOff();
  ui_powerSave(false);

  keypad_init();

  Audio::begin();
  Audio::primeReady();
  Audio::requestPlayMp3(Track::SLEEP_EXIT);

#if ENABLE_WIFI_OTA
  if (wasWifiOn) {
    g_wifiAudioSuppressUntilMs = millis() + 5000;
    g_wifiSuppressNextConnectAnnounce = true;
    Net::wifiResume();
  }
#endif

  g_lastKeyPressMs = millis();
  g_inactivitySleepArmed = true;
  lastOledMs = 0;

  ScaleState::resetFiltersAndState();
}

// ========================= BATTERIA SAFETY STATE =========================
static uint32_t g_lowBattSinceMs      = 0;
static uint32_t g_preSleepStartMs     = 0;
static uint32_t g_countdownExitSinceMs = 0;
static uint32_t g_lastEmptyBeepMs     = 0;
static uint32_t g_lastCountdownBeepMs = 0;
static bool     g_countdownMidMp3Played = false;
static uint32_t g_emptyCandidateSinceMs = 0;
static bool     g_emptyWarnActive       = false;
static uint32_t g_recoverySinceMs = 0;
static uint32_t g_lastMp3BattLowMs  = 0;
static uint32_t g_lastMp3BattCritMs = 0;

enum BattSleepStage : uint8_t { BATT_SLEEP_NONE = 0, BATT_SLEEP_ZZZ = 1 };
static BattSleepStage g_battSleepStage = BATT_SLEEP_NONE;
static uint32_t g_battStageStartMs = 0;

static void enterLowBatteryLightSleep() {
  if (battery_is_available()) {
    BatteryStatus st = battery_get_status();
    ui_showBatteryShutdown(st.voltage_V);
  } else {
    ui_showError("BATTERIA", "INA219 non trovato", "");
  }

  delay(80);
  Audio::powerOffNow();

#if ENABLE_MQTT
  Net::mqttSuspend();
#endif
#if ENABLE_WIFI_OTA
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
#endif

  const int* rows = nullptr; int nRows = 0;
  const int* cols = nullptr; int nCols = 0;
  keypad_get_pins(&rows, &nRows, &cols, &nCols);

  for (int i = 0; i < nRows; i++) {
    pinMode(rows[i], OUTPUT);
    digitalWrite(rows[i], LOW);
  }
  for (int i = 0; i < nCols; i++) {
    pinMode(cols[i], INPUT_PULLUP);
    gpio_wakeup_enable((gpio_num_t)cols[i], GPIO_INTR_LOW_LEVEL);
  }

  ui_powerSave(true);
  esp_sleep_enable_gpio_wakeup();

  delay(20);
  esp_light_sleep_start();
  ESP.restart();
}

static bool maybeEnterSafeShutdown(uint32_t nowMs) {
  if (!battery_is_available()) return false;
  BatteryStatus st = battery_get_status();

  if (st.charging) {
    g_lowBattSinceMs = 0;
    g_preSleepStartMs = 0;
    g_lastEmptyBeepMs = 0;
    g_lastCountdownBeepMs = 0;
    g_countdownMidMp3Played = false;
    g_countdownExitSinceMs = 0;
    g_emptyCandidateSinceMs = 0;
    g_emptyWarnActive = false;
    g_recoverySinceMs = 0;
    g_lastMp3BattLowMs = 0;
    g_lastMp3BattCritMs = 0;
    g_battSleepStage = BATT_SLEEP_NONE;
    g_battStageStartMs = 0;
    return false;
  }

  if (st.voltage_V >= V_SAFE_SHUTDOWN_CLEAR_V) {
    g_lowBattSinceMs = 0;
    g_preSleepStartMs = 0;
    g_lastCountdownBeepMs = 0;
    g_countdownMidMp3Played = false;
    g_countdownExitSinceMs = 0;
    g_emptyCandidateSinceMs = 0;
    g_emptyWarnActive = false;
    g_battSleepStage = BATT_SLEEP_NONE;
    g_battStageStartMs = 0;

    if (g_recoverySinceMs == 0) g_recoverySinceMs = nowMs;
    if ((nowMs - g_recoverySinceMs) >= BATT_RECOVERY_STABLE_MS) {
      g_lastEmptyBeepMs = 0;
      g_lastMp3BattLowMs = 0;
      g_lastMp3BattCritMs = 0;
      g_recoverySinceMs = 0;
    }
    return false;
  }

  g_recoverySinceMs = 0;

  if (g_battSleepStage == BATT_SLEEP_ZZZ) {
    ui_renderSleepZzz();
    if (st.voltage_V <= V_HARD_SLEEP_MIN_V || (nowMs - g_battStageStartMs) >= BATT_ZZZ_MS) {
      enterLowBatteryLightSleep();
    }
    return true;
  }

  const bool inEmptyZone = (st.level == BATT_LEVEL_EMPTY) && (st.voltage_V > V_SAFE_SHUTDOWN_MIN_V);
  if (inEmptyZone) {
    if (!g_emptyWarnActive) {
      if (g_emptyCandidateSinceMs == 0) g_emptyCandidateSinceMs = nowMs;
      if ((nowMs - g_emptyCandidateSinceMs) >= BATT_EMPTY_DEBOUNCE_MS) {
        g_emptyWarnActive = true;
      }
    }
  } else {
    g_emptyCandidateSinceMs = 0;
    g_emptyWarnActive = false;
  }

  if (g_emptyWarnActive) {
    if (g_lastMp3BattLowMs == 0 || (nowMs - g_lastMp3BattLowMs) >= BATT_ALERT_COOLDOWN_MS) {
      Audio::requestPlayMp3(Track::BATT_LOW);
      g_lastMp3BattLowMs = nowMs;
    }
    if (g_lastEmptyBeepMs == 0 || (nowMs - g_lastEmptyBeepMs) >= BEEP_EMPTY_MS) {
      buzzerWarn();
      g_lastEmptyBeepMs = nowMs;
    }
    return false;
  }

  if (st.voltage_V <= V_HARD_SLEEP_MIN_V) {
    if (g_lastMp3BattCritMs == 0 || (nowMs - g_lastMp3BattCritMs) >= BATT_ALERT_COOLDOWN_MS) {
      Audio::requestPlayMp3(Track::BATT_CRITICAL);
      g_lastMp3BattCritMs = nowMs;
    }
    buzzerWarn();
    enterLowBatteryLightSleep();
    return true;
  }

  if (g_preSleepStartMs != 0) {
    if (st.voltage_V >= V_SAFE_SHUTDOWN_EXIT_V) {
      if (g_countdownExitSinceMs == 0) g_countdownExitSinceMs = nowMs;
      if ((nowMs - g_countdownExitSinceMs) >= SAFE_SHUTDOWN_EXIT_STABLE_MS) {
        g_lowBattSinceMs = 0;
        g_preSleepStartMs = 0;
        g_lastCountdownBeepMs = 0;
        g_countdownMidMp3Played = false;
        g_battSleepStage = BATT_SLEEP_NONE;
        g_battStageStartMs = 0;
        g_countdownExitSinceMs = 0;
        g_lastMp3BattCritMs = 0;
        return false;
      }
    } else {
      g_countdownExitSinceMs = 0;
    }

    ui_showBatteryShutdown(st.voltage_V);

    const uint32_t elapsedMs = (uint32_t)(nowMs - g_preSleepStartMs);
    if (!g_countdownMidMp3Played && elapsedMs >= PRE_SLEEP_MID_AUDIO_MS) {
      Audio::requestPlayMp3(Track::BATT_CRITICAL);
      g_lastMp3BattCritMs = nowMs;
      g_countdownMidMp3Played = true;
    }

    if (g_lastCountdownBeepMs == 0 || (nowMs - g_lastCountdownBeepMs) >= BEEP_COUNTDOWN_MS) {
      buzzerWarn();
      g_lastCountdownBeepMs = nowMs;
    }

    if ((nowMs - g_preSleepStartMs) >= PRE_SLEEP_COUNTDOWN_MS) {
      if (g_battSleepStage == BATT_SLEEP_NONE) {
        ui_renderSleepZzz();
        Audio::requestPlayMp3(Track::BATT_SLEEP);
        g_battSleepStage = BATT_SLEEP_ZZZ;
        g_battStageStartMs = nowMs;
      }
    }
    return true;
  }

  const bool wantsCountdown = (st.voltage_V <= V_SAFE_SHUTDOWN_MIN_V) ||
                             (g_lowBattSinceMs != 0 && st.voltage_V < V_SAFE_SHUTDOWN_EXIT_V);

  if (wantsCountdown) {
    if (st.voltage_V <= V_SAFE_SHUTDOWN_MIN_V && g_lowBattSinceMs == 0) g_lowBattSinceMs = nowMs;

    if ((nowMs - g_lowBattSinceMs) < SAFE_SHUTDOWN_DEBOUNCE_MS) {
      return false;
    }

    if (g_preSleepStartMs == 0) {
      g_preSleepStartMs = nowMs;
      g_lastCountdownBeepMs = 0;
      g_countdownExitSinceMs = 0;
      g_countdownMidMp3Played = false;
      Audio::requestPlayMp3(Track::BATT_CRITICAL);
      g_lastMp3BattCritMs = nowMs;
    }
    return true;
  }

  if (st.voltage_V >= V_SAFE_SHUTDOWN_EXIT_V) {
    g_lowBattSinceMs = 0;
  }
  g_preSleepStartMs = 0;
  g_lastCountdownBeepMs = 0;
  g_countdownMidMp3Played = false;
  g_countdownExitSinceMs = 0;
  g_battSleepStage = BATT_SLEEP_NONE;
  g_battStageStartMs = 0;
  return false;
}

// ========================= SETUP =========================
void setup() {
  Serial.begin(115200);
  delay(50);

  pinMode(SLEEP_LED_PIN, OUTPUT);
  sleepLedOff();
  g_lastKeyPressMs = millis();
  setupLoopWatchdog();

  // Display subito
  ui_init();
  ui_showBoot("Accensione...", "");

  // Buzzer
  buzzerInit();

  Serial.println(F("\n=== ESP32 + HX711 — Modular Refactor ==="));
  printHelp();

  // Tastierino
  keypad_init();

  // Audio
  Audio::begin();
  Audio::primeReady();
  Audio::requestPlayMp3(Track::BOOT_START);

#if ENABLE_WIFI_OTA
  g_wifiUserEnabled = loadWifiUserEnabledFromNVS();
  Net::otaSetup("minu-bench-scale");

  if (g_wifiUserEnabled) {
    Net::wifiSetup();
    g_wifiSetupDone = true;
  } else {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    g_wifiSetupDone = false;
    Serial.println(F("[NET] WiFi OFF (preferenza utente)"));
  }

#if ENABLE_MQTT
  Net::mqttSetup();
#endif
#endif

  // I2C
  bootShow("I2C", "Avvio bus...");
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  bootShow("I2C", "OK");

  // HX711
  bootShow("Sensore peso", "Init HX711...");
  bool hxOk = initHX711();

  while (!g_hxBeginOk) {
    ui_showError("HX711", "Modulo non trovato", "Premi TARA per ritentare");
    Audio::requestPlayMp3(Track::ERROR_HX);
    bootWaitTare();
    bootShow("Sensore peso", "Ritentativo...");
    hxOk = initHX711();
  }

  if (!hxOk) {
    ui_showError("HX711", "Init: FAIL", "Premi TARA per continuare");
    Audio::requestPlayMp3(Track::ERROR_HX);
    bootWaitTare();
  }
  bootShow("Sensore peso", "OK");

  // NVS
  bootShow("Parametri", "Carico NVS...");
  ScaleState::loadFromNVS();
  stEnable = ScaleState::isStEnabled();
  bootShow("Parametri", "OK");

  // Preset
  bootShow("Preset", "WORK");
  ScaleState::setMode("work");
  stEnable = true;

  bool restoredRuntimeTare = runtimeTareRecoveryTryRestore();
  if (restoredRuntimeTare) {
    Serial.println(F("[RECOVERY] Tara runtime ripristinata dopo WDT"));
    bootShow("Recovery", "Tara ripristinata");
  }

  // Auto-TARE
  if (AUTO_TARE_ON_BOOT && !restoredRuntimeTare) {
    bootShow("Auto-TARE", "In corso...");
    bool tareOk = autoTareOnBoot(AUTO_TARE_SAMPLES);
    if (!tareOk) {
      ui_showError("Auto-TARE", "Timeout/FAIL", "Premi TARA per continuare");
      Audio::requestPlayMp3(Track::ERROR_TARE);
      bootWaitTare();
    } else {
      runtimeTareRecoverySave();
    }
  }

  // Batteria
  bootShow("Batteria", "Init INA219...");
  battery_init();
  if (!battery_is_available()) {
    ui_showError("Batteria", "INA219 non trovato", "Premi TARA per continuare");
    Audio::requestPlayMp3(Track::ERROR_INA);
    bootWaitTare();
  }

  bootShow("Avvio", "Completato");
  Audio::requestPlayMp3(Track::BOOT_COMPLETE);
  g_isBooting = false;

  // HX health
  hxHealth_init(&g_hxHealth, HX_HEALTH_CFG);

  lastOledMs = millis();

}

// ========================= LOOP =========================
void loop() {
  // Reset Task Watchdog (evita reboot se loop è vivo)
  esp_task_wdt_reset();

#if ENABLE_WIFI_OTA
  Net::update();
  wifiAudioUpdate(millis());
#endif

#if ENABLE_MQTT
  // Due bip di avviso se MQTT si disconnette (WiFi ancora up)
  if (Net::mqttPopDisconnectBeep() && WiFi.isConnected()) {
    buzzerWarn();
    delay(120);
    buzzerWarn();
  }

  // Bip distintivo quando arriva un nuovo comando weigh da MQTT
  if (Net::mqttPopCommandRxBeep()) {
    buzzerMqttRx();
  }
#endif

  serial_task();

  uint32_t now = millis();

  // Audio
  Audio::task(now);

  // Tastiera
  keypad_update(now);
  KeyCode key = keypad_get_event();

  // Calibration Wizard: long press su SKIP per 5 secondi per avviare
  CalWizard::updateLongPress(now);

  // Aggiorna state machine del wizard, passando l'evento tastiera
  CalWizard::update(now, key);

  // Se il wizard è attivo, i tasti vengono gestiti da CalWizard::update()
  bool wizardHandledKey = CalWizard::isActive() &&
    (key == KEY_TARE || key == KEY_ENTER || key == KEY_SKIP || key == KEY_CLEAR);

  if (key != KEY_NONE && !wizardHandledKey) {
    g_lastKeyPressMs = now;
    if (g_inactivitySleepStage != INACT_NONE) {
      g_inactivitySleepStage = INACT_NONE;
      g_inactivityStageStartMs = 0;
      Audio::stopNow(true);
      lastOledMs = 0;
    }
    handleKeyEvent(key);
  }

  // Se il wizard è attivo o long press in corso, resetta timer inattività
  if ((CalWizard::isActive() || CalWizard::isLongPressInProgress()) && key != KEY_NONE) {
    g_lastKeyPressMs = now;
  }

  // ====================== WEIGH STACK: CLEAR LONG PRESS ======================

  // CLEAR key: long press detection (2s -> clear all)
  if (g_clearPressStartMs != 0) {
    if (keypad_is_pressed(KEY_CLEAR)) {
      // Still held
      if (!g_clearLongPressFired && (int32_t)(now - g_clearPressStartMs) >= (int32_t)LONG_PRESS_MS) {
        // Long press fired: clear entire stack
        g_clearLongPressFired = true;
        WeighStack::clear();
        Serial.println(F("[STACK] CLEAR long press -> stack cleared"));

        g_overlayType = OVERLAY_CLEAR_FEEDBACK;
        g_overlayStartMs = now;
        strlcpy(g_overlayMsg, "CLEAR ALL", sizeof(g_overlayMsg));
        g_overlayCount = 0;
        lastOledMs = 0;

        buzzerOk();
        delay(80);
        buzzerOk();  // double beep for clear all
        Audio::requestPlayMp3(Track::CLEAR_PRESSED);
      }
    } else {
      // Released
      if (!g_clearLongPressFired) {
        // Short press: pop last item
        if (WeighStack::count() > 0) {
          float popped = WeighStack::get(WeighStack::count() - 1);
          WeighStack::pop();
          Serial.print(F("[STACK] POP: "));
          Serial.print(popped, 0);
          Serial.print(F("g (remaining="));
          Serial.print(WeighStack::count());
          Serial.println(F(")"));

          g_overlayType = OVERLAY_CLEAR_FEEDBACK;
          g_overlayStartMs = now;
          strlcpy(g_overlayMsg, "POP", sizeof(g_overlayMsg));
          g_overlayCount = WeighStack::count();
          lastOledMs = 0;

          buzzerKeyClick();
        } else {
          Serial.println(F("[STACK] POP: stack empty"));
          buzzerWarn();
        }
        Audio::requestPlayMp3(Track::CLEAR_PRESSED);
      }
      g_clearPressStartMs = 0;
    }
  }

  // Overlay timeout: auto-dismiss after 3 seconds or any key press
  if (g_overlayType != OVERLAY_NONE) {
    if ((int32_t)(now - g_overlayStartMs) >= (int32_t)OVERLAY_TIMEOUT_MS) {
      g_overlayType = OVERLAY_NONE;
      lastOledMs = 0;  // force redraw of normal screen
    }
    // Any key dismisses overlay (already handled in TARE/ENTER/TOTAL above)
    // CLEAR has dedicated short/long handling
    if (key != KEY_NONE && key != KEY_CLEAR) {
      // Non-CLEAR keys dismiss immediately
      // (CLEAR already starts its own tracking)
    }
  }

  // Batteria
  battery_update(now);
  if (maybeEnterSafeShutdown(now)) return;

  // Debug batteria ogni 3s
  if (now - lastBattDebug > 3000) {
    lastBattDebug = now;
    BatteryStatus st = battery_get_status();
    battery_debug_print(st);
  }

  // ====================== HX711 LETTURA ======================
  bool haveNewWork = false;
  long rawAvg = 0;

  // Flatline detection
  static long     s_hxRawFlatLast = 0;
  static bool     s_hxRawFlatHas = false;
  static uint32_t s_hxRawFlatStartMs = 0;
  static bool     s_hxRawFlatLatched = false;

  if (hx711_is_ready()) {
    long raw = hx711_read();

    // Health: flatline check
    bool rawOkForHealth = true;
    if (!s_hxRawFlatHas) {
      s_hxRawFlatHas = true;
      s_hxRawFlatLast = raw;
      s_hxRawFlatStartMs = 0;
      s_hxRawFlatLatched = false;
    } else {
      long d = raw - s_hxRawFlatLast;
      if (d < 0) d = -d;
      if (d <= 1) {
        if (s_hxRawFlatStartMs == 0) s_hxRawFlatStartMs = now;
        if ((now - s_hxRawFlatStartMs) > 1500) {
          rawOkForHealth = false;
          if (!s_hxRawFlatLatched) {
            hxHealth_backdateLastRaw(&g_hxHealth, s_hxRawFlatStartMs);
            s_hxRawFlatLatched = true;
          }
        }
      } else {
        s_hxRawFlatStartMs = 0;
        s_hxRawFlatLatched = false;
      }
      s_hxRawFlatLast = raw;
    }

    if (rawOkForHealth) hxHealth_noteRaw(&g_hxHealth, now);

    // Mediana
    long rawMed = ScaleFilters::pushMedian3(raw);

    // Spike guard
    long rawUse = rawMed;
    float cpg = ScaleState::getScaleCpg();
    if (ScaleFilters::rawSpikeGuard(rawMed, now, cpg, &rawUse)) {
      ScaleState::tareAccumSample(rawUse);

      gLiveSum += rawUse;
      gLiveCnt++;
      gWorkSum += rawUse;
      gWorkCnt++;
    }

    // LIVE: N=3
    if (gLiveCnt >= DECIM_LIVE_N) {
      long rawLive = gLiveSum / (long)DECIM_LIVE_N;
      gLiveSum = 0;
      gLiveCnt = 0;

      long offEff = ScaleState::effectiveOffsetCounts();
      float gFast = (cpg > 0.01f) ? ((float)(rawLive - offEff) / cpg) : 0.0f;

      // EMA
      if (gLiveEmaPrevStEnable != stEnable) {
        gLiveEmaInit = false;
        gLiveEmaPrevStEnable = stEnable;
      }
      float gFastDisp = gFast;
      if (!stEnable) {
        if (!gLiveEmaInit) {
          gLiveEma = gFast;
          gLiveEmaInit = true;
        } else {
          gLiveEma = gLiveEma + LIVE_EMA_ALPHA * (gFast - gLiveEma);
        }
        gFastDisp = gLiveEma;
      }

      long dispLiveHalf = ScaleState::displayLiveHalfStable(gFastDisp, ScaleState::getDispLiveHalfLast(), now);
      long dispLive = ScaleState::halfToRoundedGram(dispLiveHalf);
      ScaleState::setDispLiveHalfLast(dispLiveHalf);
      ScaleState::setDispLiveLast(dispLive);

      if (!stEnable) {
        hxHealth_noteValid(&g_hxHealth, now, (float)dispLive);
      }
    }

    // WORK: N=5
    if (gWorkCnt >= DECIM_WORK_N) {
      long rawWork = gWorkSum / (long)DECIM_WORK_N;
      gWorkSum = 0;
      gWorkCnt = 0;

      lastSampleMs = now;
      rawAvg = ScaleFilters::pushMA(rawWork);
      g_lastRawFiltered = rawAvg;  // Per calibrazione via seriale
      ScaleState::setLastRawAvg(rawAvg);  // Per calibrazione wizard
      haveNewWork = true;
    }
  }

  if (haveNewWork) {
    long offEff = ScaleState::effectiveOffsetCounts();
    float cpg = ScaleState::getScaleCpg();
    gLive = (cpg > 0.01f) ? (float)(rawAvg - offEff) / cpg : 0.0f;

    ScaleFilters::pushHist(gLive);

    int n05    = constrain((int)ceil(500.0f / SAMPLE_MS), 1, 64);
    int n2s    = constrain((int)ceil(2000.0f / SAMPLE_MS), 1, 64);
    int nSlope = constrain((int)ceil(800.0f / SAMPLE_MS), 2, 64);

    float rng05     = ScaleFilters::rangeLastNSamples(n05);
    float rng2s     = ScaleFilters::rangeLastNSamples(n2s);
    float slope_gps = ScaleFilters::slopeLastNSamples(nSlope);

    // State machine
    long gDisp = 0;
    bool overload = false;
    const char* modeLabel = stEnable ? (ScaleState::getWeighState() == ScaleState::STATE_STABLE ? "STABLE" : "UNSTABLE") : "LIVE";

    if (!stEnable) {
      if (!ScaleState::isDispInit()) {
        ScaleState::setDispUnstable(gLive);
        ScaleState::setDispInit(true);
      }
      if (fabsf(gLive - ScaleState::getDispUnstable()) > DB_UNSTABLE_N) {
        ScaleState::setDispUnstable(gLive);
      }
      gDisp = ScaleState::displayLiveQuant(ScaleState::getDispUnstable(), ScaleState::getDispLiveLast());
    } else {
      if (ScaleState::getWeighState() == ScaleState::STATE_STABLE) {
        if (fabsf(gLive - ScaleState::getGramsLatch()) >= ST_LEAVE_DELTA_G) {
          ScaleState::setWeighState(ScaleState::STATE_UNSTABLE);
          ScaleState::setDispInit(false);
        }
        gDisp = ScaleState::displayWorkQuant(ScaleState::getGramsLatch(), ScaleState::getDispWorkLast());
      } else {
        if (!ScaleState::isDispInit()) {
          ScaleState::setDispUnstable(gLive);
          ScaleState::setDispInit(true);
        }
        if (fabsf(gLive - ScaleState::getDispUnstable()) > DB_UNSTABLE_N) {
          ScaleState::setDispUnstable(gLive);
        }
        gDisp = ScaleState::displayWorkQuant(ScaleState::getDispUnstable(), ScaleState::getDispWorkLast());

        int nStab = constrain((int)ceil((float)ST_TO_STABLE_MS / SAMPLE_MS), 2, 64);
        float rngStab = ScaleFilters::rangeLastNSamples(nStab);
        if (rngStab < ST_ENTER_RANGE_G) {
          ScaleState::setWeighState(ScaleState::STATE_STABLE);
          ScaleState::setGramsLatch(gLive);
          gDisp = ScaleState::displayWorkQuant(ScaleState::getGramsLatch(), ScaleState::getDispWorkLast());
        }
      }
    }

    // Zero-Tracking
    if (ScaleState::isZtEnabled()) {
      if (UNLOAD_SNAP_ENABLE) {
        int nSlopeUnload = constrain((int)ceil((float)UNLOAD_SLOPE_WIN_MS / SAMPLE_MS), 2, 64);
        float slopeUnload = ScaleFilters::slopeLastNSamples(nSlopeUnload);
        int nSt = constrain((int)ceil((float)UNLOAD_STABLE_MS / SAMPLE_MS), 2, 64);
        float rngSt = ScaleFilters::rangeLastNSamples(nSt);
        bool quietNow = (rngSt <= UNLOAD_STABLE_RANGE_G);
        (void)ScaleState::trySnapOnUnload(gLive, slopeUnload, quietNow);
      }

      if (fabsf(gLive) <= ZT_WINDOW_G && ScaleState::isQuietForZt()) {
        if (now - g_lastZTtickMs >= ZT_PERIOD_MS) {
          ScaleState::applyZtStepTowardZero(gLive);
          g_lastZTtickMs = now;
        }
      }
    }

    // Clamp
    float base = (!stEnable ? ScaleState::getDispUnstable() : (ScaleState::getWeighState() == ScaleState::STATE_STABLE ? ScaleState::getGramsLatch() : ScaleState::getDispUnstable()));
    if (fabsf(base) > MAX_DISPLAY_G) {
      overload = FLAG_OVERLOAD;
      if (gDisp > (long)MAX_DISPLAY_G)  gDisp = (long)MAX_DISPLAY_G;
      if (gDisp < -(long)MAX_DISPLAY_G) gDisp = -(long)MAX_DISPLAY_G;
    }

    // Log
    if (g_hxLogEnabled && (now - g_hxLogLastMs) >= g_hxLogPeriodMs) {
      g_hxLogLastMs = now;
      Serial.print(F("rawAvg="));   Serial.print(rawAvg);
      Serial.print(F("  g="));      Serial.print(gLive, 2);
      Serial.print(F("  gDisp=")); Serial.print(gDisp);
      Serial.print(F("  state=")); Serial.print(modeLabel);
      Serial.print(F("  rng0.5s=")); Serial.print(rng05, 2);
      Serial.print(F("  slope=")); Serial.print(slope_gps, 2);
      Serial.print(F("  rng2s=")); Serial.print(rng2s, 2);
      Serial.print(F("  over="));  Serial.println(overload ? 1 : 0);
    }

    ScaleState::setDispWorkLast(gDisp);
    ScaleState::setStateLabelWorkLast(modeLabel);

    if (stEnable) {
      hxHealth_noteValid(&g_hxHealth, now, (float)ScaleState::getDispWorkLast());
    }
  }

  // Tara apply
  bool tareApplied = ScaleState::tareMaybeApply(millis());
  if (tareApplied) {
    runtimeTareRecoverySave();
  }
  if (tareApplied && g_captureReferenceAfterTare) {
    long ref = ScaleState::effectiveOffsetCounts();
    WeighStack::setReferenceOffset(ref);
    g_captureReferenceAfterTare = false;
    Serial.print(F("[STACK] Ref offset saved: "));
    Serial.println(ref);
  }

  // HX health update
  hxHealth_update(&g_hxHealth, now);
  ui_setHxWarn(hxHealth_isWarn(&g_hxHealth));

  if (hxHealth_popEnterErrorBeep(&g_hxHealth)) {
    Audio::requestPlayMp3(Track::ERROR_HX);
  }

  // Inattività: nota variazione peso
  ScaleState::inactivityNoteWeightActivity(now, stEnable ? ScaleState::getDispWorkLast() : ScaleState::getDispLiveLast());

  // Se il peso è variato, resetta il timer inattività (evita sleep mentre si versa)
  if (ScaleState::popWeightActivity()) {
    g_lastKeyPressMs = now;
  }

  // Sleep per inattività
  bool allowInactivitySleep = g_inactivitySleepArmed;
  if (g_inactivitySleepStage == INACT_NONE && Audio::isActive()) allowInactivitySleep = false;
#if ENABLE_WIFI_OTA
  if (Net::isOtaInProgress()) allowInactivitySleep = false;
#endif

  if (!allowInactivitySleep) {
    g_inactivitySleepStage = INACT_NONE;
    g_inactivityStageStartMs = 0;
  } else {
    if (g_inactivitySleepStage == INACT_NONE) {
      if ((now - g_lastKeyPressMs) >= INACTIVITY_SLEEP_MS) {
        ui_renderSleepZzz();
        Audio::requestPlayMp3(Track::SLEEP_ENTER);
        g_inactivitySleepStage = INACT_ZZZ;
        g_inactivityStageStartMs = now;
        lastOledMs = 0;
      }
    } else {
      if ((now - g_inactivityStageStartMs) >= INACTIVITY_ZZZ_MS) {
        g_inactivitySleepStage = INACT_NONE;
        g_inactivityStageStartMs = 0;
        enterInactivityLightSleep();
        return;
      }
    }
  }

  // OLED update
  if (now - lastOledMs >= OLED_UPDATE_MS) {
    lastOledMs = now;

    if (hxHealth_isError(&g_hxHealth)) {
      const bool hard = hxHealth_isHard(&g_hxHealth);
      ui_renderHxError(hard, false, 0);
      return;
    }

    if (g_inactivitySleepStage == INACT_ZZZ) {
      ui_renderSleepZzz();
    } else if (CalWizard::isLongPressInProgress()) {
      // Long press in corso: mostra barra di progresso
      ui_renderCalLongPress(CalWizard::getLongPressProgress(now));
    } else if (CalWizard::isActive()) {
      // Calibration wizard UI
      CalWizard::CalStep step = CalWizard::getStep();
      switch (step) {
        case CalWizard::CAL_STEP_ZERO:
          ui_renderCalStepZero(CalWizard::getSampleProgress());
          break;
        case CalWizard::CAL_STEP_PLACE:
          ui_renderCalStepPlace(CalWizard::getSampleProgress());
          break;
        case CalWizard::CAL_STEP_VALUE:
          ui_renderCalStepValue(CalWizard::getRefWeightG());
          break;
        case CalWizard::CAL_STEP_CONFIRM:
          ui_renderCalStepConfirm(CalWizard::getRefWeightG(), CalWizard::getCalculatedCpg());
          break;
        default:
          break;
      }
    } else {
      bool drewTare = false;

      if (ScaleState::isTareUiActive()) {
        uint32_t elapsed = now - ScaleState::getTareUiStartMs();
        uint8_t pct = (elapsed >= TARE_UI_CLAMP_MS) ? 100 : (uint8_t)((elapsed * 100UL) / TARE_UI_CLAMP_MS);
        ui_renderTareProgress(pct);
        drewTare = true;

        if (now >= ScaleState::getTareUiEndMs()) {
          ScaleState::setTareUiActive(false);
        }
      }

      if (g_overlayType != OVERLAY_NONE) {
        // Weigh stack overlay active
        switch (g_overlayType) {
          case OVERLAY_STACK_COMPARE:
            ui_renderStackCompare(g_overlayTotal, g_overlayNet, g_overlayCount);
            break;
          case OVERLAY_CLEAR_FEEDBACK:
            ui_renderStackClear(g_overlayMsg, g_overlayCount);
            break;
          default:
            break;
        }
      } else if (!drewTare) {
#if ENABLE_MQTT
        bool mqttTarget = Net::isMqttCommandActive();
        float mqttTW    = Net::getMqttTargetWeight();
        const char* mqttName = Net::getMqttCommandName();
        bool mqttConn   = Net::isMqttConnected();
        bool mqttVis    = WiFi.isConnected();
#else
        bool mqttTarget = false;
        float mqttTW    = 0.0f;
        const char* mqttName = nullptr;
        bool mqttConn   = false;
        bool mqttVis    = false;
#endif
        if (!stEnable) {
          ui_renderWeightLiveHalf(ScaleState::getDispLiveHalfLast(), "LIVE",
                                  mqttTarget, mqttTW, mqttName, mqttConn, mqttVis);
        } else {
          ui_renderWeight(ScaleState::getDispWorkLast(), ScaleState::getStateLabelWorkLast(),
                          mqttTarget, mqttTW, mqttName, mqttConn, mqttVis);
        }
      }
    }
  }
}
