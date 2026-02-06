#include "net_ota_cloud.h"

namespace Net {

#if ENABLE_WIFI_OTA

// ========================= STATO INTERNO =========================
static bool     wifiConfigured = false;
static bool     wifiPaused     = false;
static uint32_t wifiLastRunMs  = 0;
static bool     wifiWasConnected = false;

static uint8_t  wifiTriedThisRound = 0;
static uint8_t  wifiAttemptIdx     = 0;
static uint32_t wifiAttemptStartMs = 0;
static uint32_t wifiRetryAtMs      = 0;

static const uint32_t WIFI_TICK_MS             = 200;
static const uint32_t WIFI_CONNECT_TIMEOUT_MS  = 8000;
static const uint32_t WIFI_BACKOFF_MS          = 3000;

static bool     otaConfigured  = false;
static bool     otaBegun       = false;
static bool     otaInProgress  = false;
static const char* otaHostname = "minu-bench-scale";

static bool otaPassConfigured = false;
static char otaPassHash[OtaStore::MD5_HEX_LEN + 1] = {0};

struct WifiCred { const char* ssid; const char* pass; };
static char wifiSsid1[WifiStore::SSID_MAX_LEN + 1] = {0};
static char wifiPass1[WifiStore::PASS_MAX_LEN + 1] = {0};
static char wifiSsid2[WifiStore::SSID_MAX_LEN + 1] = {0};
static char wifiPass2[WifiStore::PASS_MAX_LEN + 1] = {0};
static WifiCred wifiCreds[2];
static uint8_t  wifiCredsN = 0;

// ========================= FUNZIONI INTERNE =========================

static void wifiRebuildCredList() {
  wifiCredsN = 0;
  if (WifiStore::isConfiguredSsid(wifiSsid1)) {
    wifiCreds[wifiCredsN++] = { wifiSsid1, wifiPass1 };
  }
  if (WifiStore::isConfiguredSsid(wifiSsid2)) {
    wifiCreds[wifiCredsN++] = { wifiSsid2, wifiPass2 };
  }
}

static void wifiLoadCredsFromNVS(bool log) {
  WifiStore::loadSlot(1, wifiSsid1, sizeof(wifiSsid1), wifiPass1, sizeof(wifiPass1));
  WifiStore::loadSlot(2, wifiSsid2, sizeof(wifiSsid2), wifiPass2, sizeof(wifiPass2));
  wifiRebuildCredList();
  if (log) {
    Serial.print(F("[NET] WiFi creds: "));
    Serial.print(wifiCredsN);
    Serial.println(F(" rete/i configurata/e."));
  }
}

static void otaLoadPasswordHashFromNVS(bool log) {
  otaPassConfigured = OtaStore::loadHash(otaPassHash, sizeof(otaPassHash));
  if (log) {
    if (otaPassConfigured) {
      Serial.println(F("[OTA] Password attiva (salvata in NVS come hash)."));
    } else {
      Serial.println(F("[OTA] ATTENZIONE: nessuna password OTA impostata."));
    }
  }
}

static void wifiKickAttempt(uint8_t idx) {
  if (wifiCredsN == 0) return;
  wifiAttemptIdx = idx % wifiCredsN;
  wifiAttemptStartMs = millis();
  wifiTriedThisRound = (wifiTriedThisRound < wifiCredsN) ? (wifiTriedThisRound + 1) : wifiCredsN;

  WiFi.begin(wifiCreds[wifiAttemptIdx].ssid, wifiCreds[wifiAttemptIdx].pass);
  Serial.print(F("[NET] WiFi: tentativo connessione a: "));
  Serial.println(wifiCreds[wifiAttemptIdx].ssid);
}

// ========================= FUNZIONI PUBBLICHE =========================

uint8_t wifiCredCount() { return wifiCredsN; }

void wifiSetup() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);

  wifiConfigured = true;
  wifiPaused     = false;
  wifiLastRunMs  = 0;
  wifiWasConnected = false;
  wifiTriedThisRound = 0;
  wifiAttemptIdx = 0;
  wifiAttemptStartMs = 0;
  wifiRetryAtMs = 0;
  wifiLoadCredsFromNVS(true);
  if (wifiCredsN > 0) {
    Serial.println(F("[NET] WiFi avviato (background)."));
    wifiKickAttempt(0);
  } else {
    Serial.println(F("[NET] WiFi ON ma nessuna credenziale configurata. Usa la seriale per impostarle."));
  }
}

void wifiSuspend() {
  if (!wifiConfigured) return;
  wifiPaused = true;
  otaBegun = false;
  otaInProgress = false;
  wifiWasConnected = false;
  wifiAttemptStartMs = 0;
  wifiRetryAtMs = 0;
  wifiTriedThisRound = 0;
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

void wifiResume() {
  if (!wifiConfigured) return;
  wifiPaused = false;
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  wifiLastRunMs = 0;
  wifiWasConnected = false;
  wifiAttemptStartMs = 0;
  wifiRetryAtMs = 0;
  wifiTriedThisRound = 0;

  wifiLoadCredsFromNVS(false);
  if (wifiCredsN > 0) {
    wifiKickAttempt(0);
  }
}

void wifiReloadCredsAndRestart() {
  if (!wifiConfigured || wifiPaused) {
    wifiLoadCredsFromNVS(true);
    return;
  }

  wifiLoadCredsFromNVS(true);
  otaBegun = false;
  otaInProgress = false;
  wifiWasConnected = false;
  wifiAttemptStartMs = 0;
  wifiRetryAtMs = 0;
  wifiTriedThisRound = 0;

  WiFi.disconnect(false);
  if (wifiCredsN > 0) {
    wifiKickAttempt(0);
  } else {
    Serial.println(F("[NET] Nessuna credenziale configurata: tentativi sospesi."));
  }
}

bool isOtaInProgress() { return otaInProgress; }

void otaSetup(const char* hostname) {
  otaHostname  = hostname;
  otaConfigured = true;
  otaBegun      = false;

  ArduinoOTA.setHostname(otaHostname);

  otaLoadPasswordHashFromNVS(true);
  if (otaPassConfigured) {
    ArduinoOTA.setPasswordHash(otaPassHash);
  }

  ArduinoOTA.onStart([]() {
    Serial.println(F("[OTA] Update iniziato"));
    otaInProgress = true;
  });

  ArduinoOTA.onEnd([]() {
    Serial.println(F("\n[OTA] Update finito"));
    otaInProgress = false;
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    if (total == 0) return;
    Serial.printf("[OTA] Avanzamento: %u%%\r", (progress * 100) / total);
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA] Errore[%u]\n", error);
    otaInProgress = false;
  });

  Serial.println(F("[OTA] Configurato. Si attiva quando il WiFi si connette."));
}

#endif // ENABLE_WIFI_OTA

#if ENABLE_ARDUINO_CLOUD
void cloudSetup() {
  Serial.println(F("[CLOUD] initProperties()..."));
  initProperties();

  Serial.println(F("[CLOUD] ArduinoCloud.begin()..."));
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);

  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
}
#endif // ENABLE_ARDUINO_CLOUD

void update() {
#if ENABLE_WIFI_OTA
  if (wifiConfigured && !wifiPaused) {
    uint32_t now = millis();
    if (now - wifiLastRunMs >= WIFI_TICK_MS) {
      wifiLastRunMs = now;

      bool connectedNow = WiFi.isConnected();
      if (connectedNow && !wifiWasConnected) {
        wifiWasConnected = true;
        Serial.print(F("[NET] Connesso a: "));
        Serial.println(WiFi.SSID());
        Serial.print(F("[NET] IP: "));
        Serial.println(WiFi.localIP());
      } else if (!connectedNow && wifiWasConnected) {
        wifiWasConnected = false;
        Serial.println(F("[NET] WiFi disconnesso."));
      }

      if (otaConfigured && !otaBegun && WiFi.isConnected()) {
        ArduinoOTA.begin();
        otaBegun = true;
        Serial.println(F("[OTA] Pronto (WiFi connesso). In Arduino IDE seleziona la porta di rete dell'ESP32."));
      }

      if (!connectedNow) {
        if (wifiCredsN == 0) {
          wifiAttemptStartMs = 0;
          wifiRetryAtMs = 0;
          wifiTriedThisRound = 0;
        } else {
          if (wifiRetryAtMs != 0 && (int32_t)(now - wifiRetryAtMs) < 0) {
            // ancora in backoff
          } else {
            if (wifiRetryAtMs != 0 && (int32_t)(now - wifiRetryAtMs) >= 0) {
              wifiRetryAtMs = 0;
              wifiTriedThisRound = 0;
              wifiAttemptStartMs = 0;
            }

            if (wifiAttemptStartMs == 0) {
              wifiKickAttempt(0);
            } else {
              if ((now - wifiAttemptStartMs) >= WIFI_CONNECT_TIMEOUT_MS) {
                if (wifiTriedThisRound < wifiCredsN) {
                  wifiKickAttempt(wifiTriedThisRound);
                } else {
                  wifiAttemptStartMs = 0;
                  wifiRetryAtMs = now + WIFI_BACKOFF_MS;
                  wifiTriedThisRound = 0;
                }
              }
            }
          }
        }
      } else {
        wifiAttemptStartMs = 0;
        wifiRetryAtMs = 0;
        wifiTriedThisRound = 0;
      }
    }
  }

  if (otaBegun) {
    ArduinoOTA.handle();
  }
#endif

#if ENABLE_ARDUINO_CLOUD
  ArduinoCloud.update();
#endif
}

} // namespace Net
