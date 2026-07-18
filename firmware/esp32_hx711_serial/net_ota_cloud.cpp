#include "net_ota_cloud.h"
#include "config/config_scale.h"
#include <esp_mac.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace Net {

#if ENABLE_MQTT
static void mqttAbortForOta();
#endif

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
static uint8_t  wifiPendingAttemptIdx = 0;
static uint32_t wifiPendingAttemptAtMs = 0;

static const uint32_t WIFI_TICK_MS             = 200;
static const uint32_t WIFI_CONNECT_TIMEOUT_MS  = 8000;
static const uint32_t WIFI_RECONFIG_PAUSE_MS   = 300;
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

static void configureLoopWdt(uint32_t timeoutMs) {
  esp_task_wdt_config_t cfg = {
    .timeout_ms = timeoutMs,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_err_t cfgErr = esp_task_wdt_reconfigure(&cfg);
  if (cfgErr != ESP_OK) {
    Serial.print(F("[WDT] cfg fail "));
    Serial.println((int)cfgErr);
    return;
  }

  esp_err_t statusErr = esp_task_wdt_status(NULL);
  if (statusErr != ESP_OK) {
    esp_err_t addErr = esp_task_wdt_add(NULL);
    statusErr = esp_task_wdt_status(NULL);
    if (statusErr != ESP_OK) {
      Serial.print(F("[WDT] add fail "));
      Serial.println((int)addErr);
      return;
    }
  }

  if (esp_task_wdt_reset() != ESP_OK) {
    Serial.println(F("[WDT] feed fail"));
  }
}

static void restoreLoopWdt() {
  (void)esp_task_wdt_add(NULL);
  configureLoopWdt(ScaleConfig::LOOP_WDT_TIMEOUT_MS);
}

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
  uint8_t nextIdx = idx % wifiCredsN;
  uint32_t now = millis();

  wl_status_t beginStatus = WiFi.begin(wifiCreds[nextIdx].ssid, wifiCreds[nextIdx].pass);
  if (beginStatus == WL_CONNECT_FAILED) {
    wifiAttemptStartMs = 0;
    wifiPendingAttemptIdx = nextIdx;
    wifiPendingAttemptAtMs = now + WIFI_RECONFIG_PAUSE_MS;
    return;
  }

  wifiPendingAttemptAtMs = 0;
  wifiAttemptIdx = nextIdx;
  wifiAttemptStartMs = now;
  wifiTriedThisRound = (wifiTriedThisRound < wifiCredsN) ? (wifiTriedThisRound + 1) : wifiCredsN;

  Serial.print(F("[NET] WiFi: tentativo connessione a: "));
  Serial.println(wifiCreds[wifiAttemptIdx].ssid);
}

static void wifiScheduleAttempt(uint8_t idx, uint32_t now) {
  if (wifiCredsN == 0) return;
  WiFi.disconnect(false, false);
  wifiAttemptStartMs = 0;
  wifiPendingAttemptIdx = idx % wifiCredsN;
  wifiPendingAttemptAtMs = now + WIFI_RECONFIG_PAUSE_MS;
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
  wifiPendingAttemptAtMs = 0;
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
  wifiPendingAttemptAtMs = 0;
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
  wifiPendingAttemptAtMs = 0;
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
  wifiPendingAttemptAtMs = 0;
  wifiTriedThisRound = 0;

  if (wifiCredsN > 0) {
    wifiScheduleAttempt(0, millis());
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
#if ENABLE_MQTT
    mqttAbortForOta();
#endif
    otaInProgress = true;
    esp_task_wdt_delete(NULL);  // Rimuove loopTask dal WDT durante upload
  });

  ArduinoOTA.onEnd([]() {
    Serial.println(F("\n[OTA] Update finito"));
    otaInProgress = false;
    restoreLoopWdt();  // Riaggiunge loopTask al WDT
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    if (total == 0) return;
    Serial.printf("[OTA] Avanzamento: %u%%\r", (progress * 100) / total);
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA] Errore[%u]\n", error);
    otaInProgress = false;
    restoreLoopWdt();
  });

  Serial.println(F("[OTA] Configurato. Si attiva quando il WiFi si connette."));
}

#endif // ENABLE_WIFI_OTA

// =============================================================================
// MQTT CLIENT
// =============================================================================
#if ENABLE_MQTT

// ========================= ISRG Root X1 (Let's Encrypt) =========================
// Fonte: https://letsencrypt.org/certs/isrgrootx1.pem
static const char MQTT_CA_CERT[] =
  "-----BEGIN CERTIFICATE-----\n"
  "MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw\n"
  "TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n"
  "cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4\n"
  "WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu\n"
  "ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY\n"
  "MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc\n"
  "h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+\n"
  "0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U\n"
  "A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW\n"
  "T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH\n"
  "B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC\n"
  "B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv\n"
  "KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn\n"
  "OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn\n"
  "jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw\n"
  "qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI\n"
  "rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV\n"
  "HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq\n"
  "hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL\n"
  "ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ\n"
  "3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK\n"
  "NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5\n"
  "ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur\n"
  "TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC\n"
  "jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc\n"
  "oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq\n"
  "4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA\n"
  "mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d\n"
  "emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=\n"
  "-----END CERTIFICATE-----\n";

// ========================= CREDENZIALI MQTT (da NVS) =========================
static char   mqtt_host[MqttStore::HOST_MAX_LEN + 1] = {0};
static char   mqtt_user[MqttStore::USER_MAX_LEN + 1] = {0};
static char   mqtt_pass[MqttStore::PASS_MAX_LEN + 1] = {0};
static bool   mqtt_credsLoaded = false;

// ========================= STATO MQTT =========================
static WiFiClientSecure mqttWifiClient;
static PubSubClient     mqttClient(mqttWifiClient);

static char   mqtt_scaleId[13]  = {0};  // MAC lowercase, no separators (12 hex + null)
static char   mqtt_topicCmd[48] = {0};  // minu/scale/{id}/command
static char   mqtt_topicRsp[48] = {0};  // minu/scale/{id}/response
static char   mqtt_topicSts[48] = {0};  // minu/scale/{id}/status
static char   mqtt_topicAck[48] = {0};  // minu/scale/{id}/ack
static char   mqtt_topicOwner[48] = {0}; // minu/scale/{id}/owner
static char   mqtt_clientId[32] = {0};  // scale-{scaleId}

// Stato comando attivo
static bool   mqtt_cmdActive       = false;
static char   mqtt_cmdUuid[37]     = {0};  // UUID standard 36 chars + null
static char   mqtt_cmdSessionId[65] = {0}; // Empty for legacy commands
static char   mqtt_cmdCommandId[65] = {0}; // Empty for pre-v1.3 commands
static char   mqtt_cmdConnectionId[65] = {0}; // Current browser document
static char   mqtt_lastSessionId[65] = {0};
static char   mqtt_cmdName[32]     = {0};  // Nome ingrediente (troncato)
static float  mqtt_cmdTargetWeight = 0.0f;
static uint32_t mqtt_cmdProductId  = 0;
static bool   mqtt_commandSnapshotReady = false;

// Browser owner snapshot. Modern commands are actionable only while this
// exact session/connection lease is fresh; retained commands cannot remain
// actionable after the browser that published them disappears.
static bool     mqtt_ownerKnown = false;
static bool     mqtt_ownerActive = false;
static char     mqtt_ownerSessionId[65] = {0};
static char     mqtt_ownerConnectionId[65] = {0};
static char     mqtt_ownerLeaseId[65] = {0};
static uint32_t mqtt_ownerTimestamp = 0;
static uint32_t mqtt_ownerReceivedMs = 0;
static uint32_t mqtt_ownerRemainingMs = 0;
// Transport resets invalidate freshness but must not erase the last evidence
// used to fence a stale retained owner/weigh pair after local fallback.
static uint32_t mqtt_lastSeenOwnerTimestamp = 0;
static char     mqtt_lastSeenOwnerConnectionId[65] = {0};
static const uint32_t MQTT_OWNER_TTL_MS = 30000;
static const uint32_t MQTT_OWNER_OPERATOR_SILENCE_MS = 15000;
static const uint32_t MQTT_OWNER_CLOCK_SKEW_SEC = 5;

// Single-entry RAM outbox for session-aware responses.
enum MqttResponseKind : uint8_t {
  MQTT_RESPONSE_NONE = 0,
  MQTT_RESPONSE_CONFIRM,
  MQTT_RESPONSE_SKIP,
  MQTT_RESPONSE_UNDO
};
static bool     mqtt_responsePending = false;
static MqttResponseKind mqtt_responseKind = MQTT_RESPONSE_NONE;
static char     mqtt_responseId[25] = {0};
static char     mqtt_responsePayload[384] = {0};
static uint32_t mqtt_responseLastPublishMs = 0;
static uint32_t mqtt_responseStartedMs = 0;
static uint32_t mqtt_responseCounter = 0;
static const uint32_t MQTT_RESPONSE_RETRY_MS = 1000;
static const uint32_t MQTT_OPERATOR_LOCK_MAX_MS = 10000;

// Network durability and physical operability are separate concerns. Once the
// remote consumer is gone (or delivery exceeds the bounded UI lock), the
// original response keeps retrying byte-for-byte while the scale works locally.
static bool     mqtt_localFallbackActive = false;
static uint32_t mqtt_localFallbackOwnerTimestamp = 0;
static char     mqtt_localFallbackOwnerConnectionId[65] = {0};
static bool     mqtt_remoteUnavailableTracking = false;
static uint32_t mqtt_remoteUnavailableSinceMs = 0;
static bool     mqtt_detachedUndoFlag = false;

// Idempotent, transient Manager request to execute the physical ENTER flow.
enum MqttConfirmRequestState : uint8_t {
  MQTT_CONFIRM_REQUEST_NONE = 0,
  MQTT_CONFIRM_REQUEST_QUEUED,
  MQTT_CONFIRM_REQUEST_RUNNING,
  MQTT_CONFIRM_REQUEST_STAGED,
  MQTT_CONFIRM_REQUEST_FAILED
};

struct MqttConfirmRequestFingerprint {
  char requestId[65];
  char sessionId[65];
  char connectionId[65];
  char commandId[65];
  char uuid[37];
  uint32_t productId;
};

static MqttConfirmRequestState mqtt_confirmRequestState = MQTT_CONFIRM_REQUEST_NONE;
static MqttConfirmRequestFingerprint mqtt_confirmRequest = {};
static char mqtt_confirmRequestReason[20] = {0};
static const uint8_t MQTT_CONFIRM_TERMINAL_CACHE_SIZE = 2;
static bool mqtt_terminalRequestValid[MQTT_CONFIRM_TERMINAL_CACHE_SIZE] = {};
static MqttConfirmRequestFingerprint
  mqtt_terminalRequests[MQTT_CONFIRM_TERMINAL_CACHE_SIZE] = {};
static char mqtt_terminalRequestStates[MQTT_CONFIRM_TERMINAL_CACHE_SIZE][10] = {};
static char mqtt_terminalRequestReasons[MQTT_CONFIRM_TERMINAL_CACHE_SIZE][20] = {};
static uint8_t mqtt_terminalRequestNext = 0;

// Riconnessione con backoff esponenziale
static bool     mqtt_setupDone       = false;
static bool     mqtt_wasPrevConnected = false;
static uint32_t mqtt_lastAttemptMs   = 0;
static uint32_t mqtt_backoffMs       = 2000;
static const uint32_t MQTT_BACKOFF_INIT = 2000;
static const uint32_t MQTT_BACKOFF_MAX  = 30000;
static const uint16_t MQTT_KEEPALIVE_SEC = 15;
static const uint16_t MQTT_SOCKET_TIMEOUT_SEC = 1;
static const uint32_t MQTT_TCP_CONNECT_TIMEOUT_MS = 2500;
static const uint32_t MQTT_IO_TIMEOUT_MS = 500;
static const uint8_t MQTT_TLS_HANDSHAKE_TIMEOUT_SEC = 10;

// TCP/TLS/MQTT connection establishment runs outside the Arduino loop. The
// main task remains free to sample the keypad and scale while DNS/TCP/TLS wait.
enum MqttConnectTaskState : uint8_t {
  MQTT_CONNECT_IDLE = 0,
  MQTT_CONNECT_RUNNING,
  MQTT_CONNECT_DONE
};
enum MqttConnectAbortState : uint8_t {
  MQTT_CONNECT_ABORT_NONE = 0,
  MQTT_CONNECT_ABORT_OFFLINE,
  MQTT_CONNECT_ABORT_SLEEPING
};
static volatile MqttConnectTaskState mqtt_connectTaskState = MQTT_CONNECT_IDLE;
static volatile MqttConnectAbortState mqtt_connectAbortState = MQTT_CONNECT_ABORT_NONE;
static volatile bool mqtt_connectTaskResult = false;
static volatile bool mqtt_connectTaskWasAborted = false;
static bool mqtt_reloadAfterConnect = false;
static portMUX_TYPE mqtt_connectTaskMux = portMUX_INITIALIZER_UNLOCKED;
static const uint32_t MQTT_CONNECT_TASK_STACK = 8192;

// Flag per segnalare disconnessione (per buzzer nel .ino)
static bool     mqtt_disconnectBeep  = false;

// Flag per segnalare ricezione di un nuovo comando weigh (bip distintivo)
static bool     mqtt_commandRxBeep   = false;
static bool     mqtt_activityFlag    = false;
static bool     mqtt_undoAckFlag     = false;
static bool     mqtt_identityValid   = false;

// Forward declaration
static void mqttCallback(char* topic, byte* payload, unsigned int length);
static bool mqttAttemptConnect();
static void mqttConnectTaskEntry(void* parameter);
static bool mqttStartConnectTask();
static bool mqttConnectTaskIsRunning();
static bool mqttClientConnectedForMain();
static bool mqttRequestConnectAbort(MqttConnectAbortState state);
static bool mqttConnectAbortRequested();
static void mqttResetCommand();
static void mqttResetConfirmRequest();
static void mqttResetOwnerSnapshot();
static void mqttDeactivateOwnerSnapshot();
static void mqttHandleOwner(const JsonDocument& doc);
static bool mqttOwnerAuthorizes(const char* sessionId, const char* connectionId);
static bool mqttOwnerConnectionAuthorizes(const char* connectionId);
static void mqttResetPendingResponse();
static void mqttEnterLocalFallback();
static void mqttLeaveLocalFallback();
static void mqttUpdateLocalFallback();
static bool mqttPublishPendingResponse();
static bool mqttPublishCommandAck();
static bool mqttPublishConfirmRequestAck(const char* requestId,
                                         const char* commandId,
                                         const char* connectionId,
                                         const char* state,
                                         const char* reason);
static bool mqttConfirmRequestMatches(
  const MqttConfirmRequestFingerprint& stored,
  const char* requestId, const char* sessionId, const char* connectionId,
  const char* commandId, const char* uuid, uint32_t productId);
static void mqttStoreConfirmRequest(
  MqttConfirmRequestFingerprint& stored,
  const char* requestId, const char* sessionId, const char* connectionId,
  const char* commandId, const char* uuid, uint32_t productId);
static int8_t mqttFindTerminalConfirmRequest(const char* requestId);
static void mqttRememberTerminalConfirmRequest(
  const char* requestId, const char* sessionId, const char* connectionId,
  const char* commandId, const char* uuid, uint32_t productId,
  const char* state, const char* reason);
static void mqttGenerateResponseId();
static bool mqttPublishStatus(const char* state);

// ========================= HELPER: MAC → scale_id =========================
static bool buildScaleId() {
  uint8_t mac[6] = {0};
  if (esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK) return false;

  bool anyNonZero = false;
  for (uint8_t value : mac) anyNonZero = anyNonZero || (value != 0);
  if (!anyNonZero) return false;

  snprintf(mqtt_scaleId, sizeof(mqtt_scaleId),
    "%02x%02x%02x%02x%02x%02x",
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return strlen(mqtt_scaleId) == 12;
}

static void buildTopics() {
  snprintf(mqtt_topicCmd, sizeof(mqtt_topicCmd), "minu/scale/%s/command",  mqtt_scaleId);
  snprintf(mqtt_topicRsp, sizeof(mqtt_topicRsp), "minu/scale/%s/response", mqtt_scaleId);
  snprintf(mqtt_topicSts, sizeof(mqtt_topicSts), "minu/scale/%s/status",   mqtt_scaleId);
  snprintf(mqtt_topicAck, sizeof(mqtt_topicAck), "minu/scale/%s/ack",      mqtt_scaleId);
  snprintf(mqtt_topicOwner, sizeof(mqtt_topicOwner), "minu/scale/%s/owner", mqtt_scaleId);
  snprintf(mqtt_clientId, sizeof(mqtt_clientId), "scale-%s",               mqtt_scaleId);
}

static void mqttResetCommand() {
  mqtt_cmdActive = false;
  mqtt_cmdUuid[0] = '\0';
  mqtt_cmdSessionId[0] = '\0';
  mqtt_cmdCommandId[0] = '\0';
  mqtt_cmdConnectionId[0] = '\0';
  mqtt_cmdName[0] = '\0';
  mqtt_cmdTargetWeight = 0.0f;
  mqtt_cmdProductId = 0;
  mqttResetConfirmRequest();
}

static void mqttResetConfirmRequest() {
  mqtt_confirmRequestState = MQTT_CONFIRM_REQUEST_NONE;
  memset(&mqtt_confirmRequest, 0, sizeof(mqtt_confirmRequest));
  mqtt_confirmRequestReason[0] = '\0';
}

static bool mqttIsSafeCommandId(const char* commandId) {
  if (!commandId) return false;
  size_t length = strlen(commandId);
  if (length != 0 && length < 8) return false;
  for (const char* p = commandId; *p != '\0'; ++p) {
    bool alphaNumeric = (*p >= '0' && *p <= '9') ||
      (*p >= 'A' && *p <= 'Z') || (*p >= 'a' && *p <= 'z');
    if (!alphaNumeric && *p != '-' && *p != '_') return false;
  }
  return true;
}

static bool mqttConnectTaskIsRunning() {
  return mqtt_connectTaskState == MQTT_CONNECT_RUNNING;
}

static bool mqttClientConnectedForMain() {
  return mqtt_connectTaskState == MQTT_CONNECT_IDLE && mqttClient.connected();
}

static bool mqttRequestConnectAbort(MqttConnectAbortState state) {
  bool requested = false;
  portENTER_CRITICAL(&mqtt_connectTaskMux);
  if (mqtt_connectTaskState == MQTT_CONNECT_RUNNING) {
    if (state > mqtt_connectAbortState) mqtt_connectAbortState = state;
    requested = true;
  }
  portEXIT_CRITICAL(&mqtt_connectTaskMux);
  return requested;
}

static bool mqttConnectAbortRequested() {
  portENTER_CRITICAL(&mqtt_connectTaskMux);
  bool requested = mqtt_connectAbortState != MQTT_CONNECT_ABORT_NONE;
  portEXIT_CRITICAL(&mqtt_connectTaskMux);
  return requested;
}

static void mqttAbortForOta() {
  (void)mqttRequestConnectAbort(MQTT_CONNECT_ABORT_OFFLINE);
}

static void mqttResetOwnerSnapshot() {
  mqtt_ownerKnown = false;
  mqtt_ownerActive = false;
  mqtt_ownerSessionId[0] = '\0';
  mqtt_ownerConnectionId[0] = '\0';
  mqtt_ownerLeaseId[0] = '\0';
  mqtt_ownerTimestamp = 0;
  mqtt_ownerReceivedMs = 0;
  mqtt_ownerRemainingMs = 0;
}

static void mqttDeactivateOwnerSnapshot() {
  mqtt_ownerKnown = true;
  mqtt_ownerActive = false;
  mqtt_ownerReceivedMs = millis();
  mqtt_ownerRemainingMs = 0;
}

static bool mqttOwnerIsFresh() {
  uint32_t elapsedMs = (uint32_t)(millis() - mqtt_ownerReceivedMs);
  uint32_t ageAtReceiveMs = mqtt_ownerRemainingMs <= MQTT_OWNER_TTL_MS
    ? MQTT_OWNER_TTL_MS - mqtt_ownerRemainingMs : MQTT_OWNER_TTL_MS;
  bool operatorPresenceFresh = ageAtReceiveMs < MQTT_OWNER_OPERATOR_SILENCE_MS &&
    elapsedMs < (MQTT_OWNER_OPERATOR_SILENCE_MS - ageAtReceiveMs);
  return mqtt_ownerKnown && mqtt_ownerActive && mqttClientConnectedForMain() &&
    mqtt_ownerRemainingMs > 0 &&
    elapsedMs < mqtt_ownerRemainingMs &&
    operatorPresenceFresh;
}

static bool mqttOwnerAuthorizes(const char* sessionId, const char* connectionId) {
  return sessionId && connectionId && sessionId[0] != '\0' &&
    connectionId[0] != '\0' && mqttOwnerIsFresh() &&
    strcmp(mqtt_ownerSessionId, sessionId) == 0 &&
    strcmp(mqtt_ownerConnectionId, connectionId) == 0;
}

static bool mqttOwnerConnectionAuthorizes(const char* connectionId) {
  return connectionId && connectionId[0] != '\0' && mqttOwnerIsFresh() &&
    strcmp(mqtt_ownerConnectionId, connectionId) == 0;
}

static void mqttHandleOwner(const JsonDocument& doc) {
  const char* sessionId = doc["session_id"] | "";
  const char* connectionId = doc["connection_id"] | "";
  const char* leaseId = doc["lease_id"] | "";
  uint32_t timestamp = doc["timestamp"] | 0U;
  bool idsValid = sessionId[0] != '\0' && connectionId[0] != '\0' &&
    leaseId[0] != '\0' && strlen(sessionId) < sizeof(mqtt_ownerSessionId) &&
    strlen(connectionId) < sizeof(mqtt_ownerConnectionId) &&
    strlen(leaseId) < sizeof(mqtt_ownerLeaseId) &&
    mqttIsSafeCommandId(sessionId) && mqttIsSafeCommandId(connectionId) &&
    mqttIsSafeCommandId(leaseId);
  if (!idsValid || timestamp == 0) {
    mqttDeactivateOwnerSnapshot();
    return;
  }

  bool released = doc["user_id"].isNull();
  bool exactStoredLease = mqtt_ownerKnown &&
    strcmp(mqtt_ownerSessionId, sessionId) == 0 &&
    strcmp(mqtt_ownerConnectionId, connectionId) == 0 &&
    strcmp(mqtt_ownerLeaseId, leaseId) == 0;
  if (released && mqtt_ownerActive && !exactStoredLease) return;

  time_t epochNow = time(nullptr);
  if (epochNow < 1700000000 ||
      timestamp > (uint32_t)epochNow + MQTT_OWNER_CLOCK_SKEW_SEC) {
    mqttDeactivateOwnerSnapshot();
    return;
  }
  if (mqtt_ownerKnown && timestamp < mqtt_ownerTimestamp) return;
  if (strcmp(connectionId, mqtt_lastSeenOwnerConnectionId) == 0 &&
      timestamp < mqtt_lastSeenOwnerTimestamp) return;

  uint32_t remainingMs = 0;
  uint32_t ageSec = timestamp < (uint32_t)epochNow
    ? (uint32_t)epochNow - timestamp : 0;
  if (ageSec < MQTT_OWNER_TTL_MS / 1000UL) {
    remainingMs = MQTT_OWNER_TTL_MS - ageSec * 1000UL;
  }

  mqtt_ownerKnown = true;
  mqtt_ownerActive = !released && remainingMs > 0;
  strlcpy(mqtt_ownerSessionId, sessionId, sizeof(mqtt_ownerSessionId));
  strlcpy(mqtt_ownerConnectionId, connectionId, sizeof(mqtt_ownerConnectionId));
  strlcpy(mqtt_ownerLeaseId, leaseId, sizeof(mqtt_ownerLeaseId));
  mqtt_ownerTimestamp = timestamp;
  mqtt_ownerReceivedMs = millis();
  mqtt_ownerRemainingMs = remainingMs;
  mqtt_lastSeenOwnerTimestamp = timestamp;
  strlcpy(
    mqtt_lastSeenOwnerConnectionId, connectionId,
    sizeof(mqtt_lastSeenOwnerConnectionId));
}

static void mqttResetPendingResponse() {
  mqtt_responsePending = false;
  mqtt_responseKind = MQTT_RESPONSE_NONE;
  mqtt_responseId[0] = '\0';
  mqtt_responsePayload[0] = '\0';
  mqtt_responseLastPublishMs = 0;
  mqtt_responseStartedMs = 0;
}

static void mqttEnterLocalFallback() {
  if (mqtt_localFallbackActive) return;
  mqtt_localFallbackActive = true;
  mqtt_localFallbackOwnerTimestamp = mqtt_ownerTimestamp != 0
    ? mqtt_ownerTimestamp : mqtt_lastSeenOwnerTimestamp;
  const char* fallbackConnection = mqtt_ownerConnectionId[0] != '\0'
    ? mqtt_ownerConnectionId : mqtt_lastSeenOwnerConnectionId;
  strlcpy(
    mqtt_localFallbackOwnerConnectionId, fallbackConnection,
    sizeof(mqtt_localFallbackOwnerConnectionId));
  mqtt_activityFlag = true;
  if (mqtt_responsePending && mqtt_responseKind == MQTT_RESPONSE_UNDO) {
    mqtt_detachedUndoFlag = true;
  }
  if (!mqtt_responsePending &&
      (mqtt_confirmRequestState == MQTT_CONFIRM_REQUEST_QUEUED ||
       mqtt_confirmRequestState == MQTT_CONFIRM_REQUEST_RUNNING)) {
    mqttResetConfirmRequest();
  }
  Serial.println(F("[MQTT] Sessione remota sganciata; operatore in locale"));
}

static void mqttLeaveLocalFallback() {
  mqtt_localFallbackActive = false;
  mqtt_localFallbackOwnerTimestamp = 0;
  mqtt_localFallbackOwnerConnectionId[0] = '\0';
}

static void mqttUpdateLocalFallback() {
  if (mqtt_localFallbackActive || !mqtt_setupDone || !mqtt_identityValid) return;

  uint32_t now = millis();
  bool connected = mqttClientConnectedForMain();
  bool modernCommand = mqtt_cmdActive && mqtt_cmdConnectionId[0] != '\0';
  bool ownerScopedContext = !mqtt_cmdActive || modernCommand ||
    mqtt_responsePending;
  bool ownerFresh = mqttOwnerIsFresh();
  bool ownerUnavailable = connected && mqtt_ownerKnown && ownerScopedContext &&
    (!ownerFresh ||
     (modernCommand &&
      !mqttOwnerAuthorizes(mqtt_cmdSessionId, mqtt_cmdConnectionId)));
  bool responseLockExpired = mqtt_responsePending &&
    (uint32_t)(now - mqtt_responseStartedMs) >= MQTT_OPERATOR_LOCK_MAX_MS;
  bool ownerSnapshotUnknown = connected && !mqtt_ownerKnown &&
    ownerScopedContext;
  bool commandSnapshotUnknown = connected && modernCommand &&
    !mqtt_commandSnapshotReady;
  bool disconnectedRemoteState = !connected &&
    (mqtt_cmdActive || mqtt_responsePending);
  bool remoteConsumerUncertain = ownerSnapshotUnknown ||
    commandSnapshotUnknown || disconnectedRemoteState;

  if (remoteConsumerUncertain) {
    if (!mqtt_remoteUnavailableTracking) {
      mqtt_remoteUnavailableTracking = true;
      mqtt_remoteUnavailableSinceMs = now;
    }
  } else {
    mqtt_remoteUnavailableTracking = false;
    mqtt_remoteUnavailableSinceMs = 0;
  }

  bool remoteUnavailableExpired = mqtt_remoteUnavailableTracking &&
    (uint32_t)(now - mqtt_remoteUnavailableSinceMs) >= MQTT_OPERATOR_LOCK_MAX_MS;
  if (ownerUnavailable || responseLockExpired || remoteUnavailableExpired) {
    mqttEnterLocalFallback();
  }
}

static void mqttGenerateResponseId() {
  snprintf(mqtt_responseId, sizeof(mqtt_responseId), "%08lx%08lx%08lx",
    (unsigned long)esp_random(), (unsigned long)millis(),
    (unsigned long)++mqtt_responseCounter);
}

static bool mqttPublishPendingResponse() {
  if (!mqtt_responsePending || !mqttClientConnectedForMain()) return false;
  bool published = mqttClient.publish(
    mqtt_topicRsp,
    (const uint8_t*)mqtt_responsePayload,
    strlen(mqtt_responsePayload),
    false
  );
  // Space attempts from completion, not from their start: even a slow failed
  // TLS write leaves a full retry interval for keypad/scale processing.
  mqtt_responseLastPublishMs = millis();
  if (!published) Serial.println(F("[MQTT] Response publish fallito; retry attivo"));
  return published;
}

static bool mqttPublishCommandAck() {
  if (!mqtt_cmdActive || mqtt_cmdCommandId[0] == '\0' ||
      !mqttClientConnectedForMain()) return false;

  char payload[384];
  int written = snprintf(payload, sizeof(payload),
    "{\"type\":\"command_ack\",\"command_id\":\"%s\",\"session_id\":\"%s\",\"connection_id\":\"%s\",\"uuid\":\"%s\",\"product_id\":%lu,\"state\":\"active\"}",
    mqtt_cmdCommandId, mqtt_cmdSessionId, mqtt_cmdConnectionId, mqtt_cmdUuid,
    (unsigned long)mqtt_cmdProductId);
  return written > 0 && written < (int)sizeof(payload) &&
    mqttClient.publish(
      mqtt_topicRsp, (const uint8_t*)payload, strlen(payload), false);
}

static bool mqttPublishConfirmRequestAck(const char* requestId,
                                         const char* commandId,
                                         const char* connectionId,
                                         const char* state,
                                         const char* reason) {
  if (!mqttClientConnectedForMain() || !requestId || !commandId ||
      !connectionId || !state) return false;
  char payload[384];
  int written = snprintf(payload, sizeof(payload),
    "{\"type\":\"confirm_request_ack\",\"request_id\":\"%s\",\"command_id\":\"%s\",\"connection_id\":\"%s\",\"state\":\"%s\",\"reason\":\"%s\"}",
    requestId, commandId, connectionId, state, reason ? reason : "");
  return written > 0 && written < (int)sizeof(payload) &&
    mqttClient.publish(
      mqtt_topicRsp, (const uint8_t*)payload, strlen(payload), false);
}

static bool mqttConfirmRequestMatches(
    const MqttConfirmRequestFingerprint& stored,
    const char* requestId, const char* sessionId, const char* connectionId,
    const char* commandId, const char* uuid, uint32_t productId) {
  return strcmp(stored.requestId, requestId) == 0 &&
    strcmp(stored.sessionId, sessionId) == 0 &&
    strcmp(stored.connectionId, connectionId) == 0 &&
    strcmp(stored.commandId, commandId) == 0 &&
    strcmp(stored.uuid, uuid) == 0 && stored.productId == productId;
}

static void mqttStoreConfirmRequest(
    MqttConfirmRequestFingerprint& stored,
    const char* requestId, const char* sessionId, const char* connectionId,
    const char* commandId, const char* uuid, uint32_t productId) {
  strlcpy(stored.requestId, requestId, sizeof(stored.requestId));
  strlcpy(stored.sessionId, sessionId, sizeof(stored.sessionId));
  strlcpy(stored.connectionId, connectionId, sizeof(stored.connectionId));
  strlcpy(stored.commandId, commandId, sizeof(stored.commandId));
  strlcpy(stored.uuid, uuid, sizeof(stored.uuid));
  stored.productId = productId;
}

static int8_t mqttFindTerminalConfirmRequest(const char* requestId) {
  for (uint8_t index = 0; index < MQTT_CONFIRM_TERMINAL_CACHE_SIZE; index++) {
    if (mqtt_terminalRequestValid[index] &&
        strcmp(mqtt_terminalRequests[index].requestId, requestId) == 0) {
      return (int8_t)index;
    }
  }
  return -1;
}

static void mqttRememberTerminalConfirmRequest(
    const char* requestId, const char* sessionId, const char* connectionId,
    const char* commandId, const char* uuid, uint32_t productId,
    const char* state, const char* reason) {
  int8_t knownIndex = mqttFindTerminalConfirmRequest(requestId);
  uint8_t index = knownIndex >= 0
    ? (uint8_t)knownIndex : mqtt_terminalRequestNext;
  mqttStoreConfirmRequest(
    mqtt_terminalRequests[index], requestId, sessionId, connectionId,
    commandId, uuid, productId);
  strlcpy(
    mqtt_terminalRequestStates[index], state,
    sizeof(mqtt_terminalRequestStates[index]));
  strlcpy(
    mqtt_terminalRequestReasons[index], reason ? reason : "",
    sizeof(mqtt_terminalRequestReasons[index]));
  mqtt_terminalRequestValid[index] = true;
  if (knownIndex < 0) {
    mqtt_terminalRequestNext =
      (uint8_t)((index + 1) % MQTT_CONFIRM_TERMINAL_CACHE_SIZE);
  }
  mqttPublishConfirmRequestAck(
    requestId, commandId, connectionId, state, reason);
}

static bool mqttPublishStatus(const char* state) {
  if (!mqttClient.connected() || !state) return false;
  char payload[256];
  int written = snprintf(payload, sizeof(payload),
    "{\"type\":\"status\",\"state\":\"%s\",\"scale_id\":\"%s\",\"name\":\"%s\",\"firmware_version\":\"%s\"}",
    state, mqtt_scaleId, MQTT_SCALE_NAME, MQTT_FW_VERSION);
  return written > 0 && written < (int)sizeof(payload) &&
    mqttClient.publish(mqtt_topicSts, (const uint8_t*)payload, strlen(payload), true);
}

// ========================= CALLBACK RICEZIONE COMANDI =========================
static void mqttCallback(char* topic, byte* payload, unsigned int length) {
  bool isCommandTopic = (strcmp(topic, mqtt_topicCmd) == 0);
  bool isAckTopic = (strcmp(topic, mqtt_topicAck) == 0);
  bool isOwnerTopic = (strcmp(topic, mqtt_topicOwner) == 0);
  if (!isCommandTopic && !isAckTopic && !isOwnerTopic) return;

  if (isCommandTopic && length == 0) {
    if (!mqtt_responsePending &&
        mqtt_confirmRequestState != MQTT_CONFIRM_REQUEST_QUEUED &&
        mqtt_confirmRequestState != MQTT_CONFIRM_REQUEST_RUNNING &&
        mqtt_confirmRequestState != MQTT_CONFIRM_REQUEST_STAGED) {
      mqttResetCommand();
    }
    mqtt_commandSnapshotReady = true;
    return;
  }

  if (isOwnerTopic && length == 0) {
    mqttResetOwnerSnapshot();
    mqtt_ownerKnown = true;
    return;
  }

  // Parse JSON
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    if (isOwnerTopic) mqttDeactivateOwnerSnapshot();
    Serial.print(F("[MQTT] JSON parse error: "));
    Serial.println(err.c_str());
    return;
  }

  if (isOwnerTopic) {
    mqttHandleOwner(doc);
    return;
  }

  const char* type = doc["type"];
  if (!type) {
    Serial.println(F("[MQTT] Comando senza campo 'type', ignorato"));
    return;
  }

  if (isAckTopic) {
    const char* responseId = doc["response_id"];
    if (strcmp(type, "response_ack") == 0 && responseId &&
        mqtt_responsePending && strcmp(responseId, mqtt_responseId) == 0) {
      Serial.print(F("[MQTT] Response ACK: "));
      Serial.println(responseId);
      if (mqtt_responseKind == MQTT_RESPONSE_UNDO &&
          !mqtt_localFallbackActive) mqtt_undoAckFlag = true;
      mqttResetPendingResponse();
      mqttResetCommand();
    }
    return;
  }

  if (strcmp(type, "confirm_request") == 0) {
    const char* requestId = doc["request_id"] | "";
    const char* sessionId = doc["session_id"] | "";
    const char* connectionId = doc["connection_id"] | "";
    const char* commandId = doc["command_id"] | "";
    const char* uuid = doc["uuid"] | "";
    uint32_t productId = doc["product_id"].is<uint32_t>()
      ? doc["product_id"].as<uint32_t>() : 0;

    bool idsValid = requestId[0] != '\0' && commandId[0] != '\0' &&
      sessionId[0] != '\0' && connectionId[0] != '\0' && uuid[0] != '\0' &&
      strlen(requestId) < sizeof(mqtt_confirmRequest.requestId) &&
      strlen(sessionId) < sizeof(mqtt_confirmRequest.sessionId) &&
      strlen(connectionId) < sizeof(mqtt_confirmRequest.connectionId) &&
      strlen(commandId) < sizeof(mqtt_confirmRequest.commandId) &&
      strlen(uuid) < sizeof(mqtt_confirmRequest.uuid) &&
      mqttIsSafeCommandId(requestId) && mqttIsSafeCommandId(connectionId) &&
      mqttIsSafeCommandId(commandId);
    if (!idsValid) {
      Serial.println(F("[MQTT] confirm_request con ID non valido"));
      return;
    }

    if (mqtt_confirmRequestState != MQTT_CONFIRM_REQUEST_NONE &&
        strcmp(mqtt_confirmRequest.requestId, requestId) == 0) {
      if (!mqttConfirmRequestMatches(
            mqtt_confirmRequest, requestId, sessionId, connectionId,
            commandId, uuid, productId)) {
        mqttPublishConfirmRequestAck(
          requestId, commandId, connectionId, "rejected", "conflict");
        return;
      }
      const char* replayState = "accepted";
      if (mqtt_confirmRequestState == MQTT_CONFIRM_REQUEST_STAGED) replayState = "staged";
      else if (mqtt_confirmRequestState == MQTT_CONFIRM_REQUEST_FAILED) replayState = "failed";
      mqttPublishConfirmRequestAck(
        requestId, commandId, mqtt_confirmRequest.connectionId,
        replayState, mqtt_confirmRequestReason);
      return;
    }

    int8_t terminalIndex = mqttFindTerminalConfirmRequest(requestId);
    if (terminalIndex >= 0) {
      uint8_t index = (uint8_t)terminalIndex;
      if (mqttConfirmRequestMatches(
            mqtt_terminalRequests[index], requestId, sessionId, connectionId,
            commandId, uuid, productId)) {
        mqttPublishConfirmRequestAck(
          requestId, commandId, connectionId,
          mqtt_terminalRequestStates[index], mqtt_terminalRequestReasons[index]);
      } else {
        mqttPublishConfirmRequestAck(
          requestId, commandId, connectionId, "rejected", "conflict");
      }
      return;
    }

    bool exactCommand = mqtt_cmdActive &&
      mqttOwnerAuthorizes(sessionId, connectionId) &&
      strcmp(mqtt_cmdSessionId, sessionId) == 0 &&
      strcmp(mqtt_cmdConnectionId, connectionId) == 0 &&
      strcmp(mqtt_cmdCommandId, commandId) == 0 &&
      strcmp(mqtt_cmdUuid, uuid) == 0 && mqtt_cmdProductId == productId;
    if (!exactCommand) {
      mqttRememberTerminalConfirmRequest(
        requestId, sessionId, connectionId, commandId, uuid, productId,
        "rejected", "stale");
      return;
    }

    if (mqtt_responsePending ||
        (mqtt_confirmRequestState != MQTT_CONFIRM_REQUEST_NONE &&
         mqtt_confirmRequestState != MQTT_CONFIRM_REQUEST_FAILED)) {
      mqttRememberTerminalConfirmRequest(
        requestId, sessionId, connectionId, commandId, uuid, productId,
        "rejected", "busy");
      return;
    }

    mqttStoreConfirmRequest(
      mqtt_confirmRequest, requestId, sessionId, connectionId,
      commandId, uuid, productId);
    mqtt_confirmRequestReason[0] = '\0';
    mqtt_confirmRequestState = MQTT_CONFIRM_REQUEST_QUEUED;
    mqtt_activityFlag = true;
    mqttPublishConfirmRequestAck(
      requestId, commandId, connectionId, "accepted", "");
    Serial.println(F("[MQTT] confirm_request accodata"));

  } else if (strcmp(type, "weigh") == 0) {
    const char* uuid   = doc["uuid"];
    const char* sessionId = doc["session_id"] | "";
    const char* connectionId = doc["connection_id"] | "";
    const char* commandId = doc["command_id"] | "";
    float targetWeight = doc["target_weight"] | 0.0f;
    uint32_t productId = doc["product_id"].is<uint32_t>()
      ? doc["product_id"].as<uint32_t>() : 0;
    const char* name   = doc["name"] | "";

    if (!uuid || strlen(uuid) == 0 || strlen(uuid) >= sizeof(mqtt_cmdUuid) ||
        strlen(sessionId) >= sizeof(mqtt_cmdSessionId) ||
        strlen(connectionId) >= sizeof(mqtt_cmdConnectionId) ||
        strlen(commandId) >= sizeof(mqtt_cmdCommandId) ||
        !mqttIsSafeCommandId(connectionId) || !mqttIsSafeCommandId(commandId)) {
      Serial.println(F("[MQTT] weigh con ID non valido, ignorato"));
      return;
    }

    if (connectionId[0] != '\0' &&
        !mqttOwnerAuthorizes(sessionId, connectionId)) {
      Serial.println(F("[MQTT] weigh ignorato: owner non valido"));
      return;
    }

    // A retained owner/weigh pair from the detached document cannot silently
    // reactivate local mode after reconnect. Require a newer heartbeat or a
    // genuinely new browser connection before accepting the replay.
    bool fallbackHasFreshEvidence = !mqtt_localFallbackActive ||
      connectionId[0] == '\0' ||
      mqtt_ownerTimestamp > mqtt_localFallbackOwnerTimestamp ||
      strcmp(connectionId, mqtt_localFallbackOwnerConnectionId) != 0;
    if (!fallbackHasFreshEvidence) {
      Serial.println(F("[MQTT] weigh ignorato: owner non rinnovato"));
      return;
    }

    if (mqtt_responsePending) {
      Serial.println(F("[MQTT] CMD weigh ignorato: response ACK pending"));
      return;
    }
    if (mqtt_confirmRequestState == MQTT_CONFIRM_REQUEST_QUEUED ||
        mqtt_confirmRequestState == MQTT_CONFIRM_REQUEST_RUNNING) {
      Serial.println(F("[MQTT] CMD weigh ignorato: confirm_request pending"));
      return;
    }

    float targetDelta = targetWeight - mqtt_cmdTargetWeight;
    if (targetDelta < 0.0f) targetDelta = -targetDelta;
    bool sameTarget = (targetDelta < 0.05f);
    bool sameUuid = (strcmp(mqtt_cmdUuid, uuid) == 0);
    bool sameSession = (strcmp(mqtt_cmdSessionId, sessionId) == 0);
    bool sameProduct = (mqtt_cmdProductId == productId);
    char normalizedName[sizeof(mqtt_cmdName)];
    strlcpy(normalizedName, name, sizeof(normalizedName));
    bool sameName = (strcmp(mqtt_cmdName, normalizedName) == 0);
    bool sameCommandId = mqtt_cmdActive && commandId[0] != '\0' &&
      mqtt_cmdCommandId[0] != '\0' &&
      strcmp(mqtt_cmdCommandId, commandId) == 0;

    // command_id is an idempotency key. A replay cannot rewrite the active
    // command, even if a conflicting payload reused the same ID.
    if (sameCommandId) {
      bool samePayload = sameUuid && sameSession && sameProduct &&
        sameTarget && sameName;
      if (samePayload) {
        mqttLeaveLocalFallback();
        mqtt_remoteUnavailableTracking = false;
        mqtt_remoteUnavailableSinceMs = 0;
        strlcpy(mqtt_cmdConnectionId, connectionId, sizeof(mqtt_cmdConnectionId));
        mqtt_commandSnapshotReady = true;
        Serial.println(F("[MQTT] CMD weigh duplicato: stato invariato"));
        mqttPublishCommandAck();
      } else {
        Serial.println(F("[MQTT] CMD weigh command_id conflittuale: ignorato"));
      }
      return;
    }

    bool sameCommand = sameUuid && sameSession && sameProduct &&
      strcmp(mqtt_cmdCommandId, commandId) == 0;
    bool isNewWeigh = (!mqtt_cmdActive) || (!sameCommand) || (!sameTarget);

    // Salva comando attivo
    mqttResetConfirmRequest();
    strncpy(mqtt_cmdUuid, uuid, sizeof(mqtt_cmdUuid) - 1);
    mqtt_cmdUuid[sizeof(mqtt_cmdUuid) - 1] = '\0';
    strncpy(mqtt_cmdSessionId, sessionId, sizeof(mqtt_cmdSessionId) - 1);
    mqtt_cmdSessionId[sizeof(mqtt_cmdSessionId) - 1] = '\0';
    strncpy(mqtt_cmdCommandId, commandId, sizeof(mqtt_cmdCommandId) - 1);
    mqtt_cmdCommandId[sizeof(mqtt_cmdCommandId) - 1] = '\0';
    strncpy(mqtt_cmdConnectionId, connectionId, sizeof(mqtt_cmdConnectionId) - 1);
    mqtt_cmdConnectionId[sizeof(mqtt_cmdConnectionId) - 1] = '\0';
    strncpy(mqtt_cmdName, name, sizeof(mqtt_cmdName) - 1);
    mqtt_cmdName[sizeof(mqtt_cmdName) - 1] = '\0';
    mqtt_cmdTargetWeight = targetWeight;
    mqtt_cmdProductId = productId;
    mqtt_cmdActive = true;
    mqttLeaveLocalFallback();
    mqtt_remoteUnavailableTracking = false;
    mqtt_remoteUnavailableSinceMs = 0;
    mqtt_commandSnapshotReady = true;
    mqtt_activityFlag = true;
    if (sessionId[0] != '\0') {
      strlcpy(mqtt_lastSessionId, sessionId, sizeof(mqtt_lastSessionId));
    }

    if (isNewWeigh) {
      mqtt_commandRxBeep = true;
    }

    Serial.print(F("[MQTT] CMD weigh: uuid="));
    Serial.print(mqtt_cmdUuid);
    Serial.print(F(" target="));
    Serial.print(mqtt_cmdTargetWeight, 1);
    Serial.print(F("g name="));
    Serial.println(name);
    mqttPublishCommandAck();

  } else if (strcmp(type, "clear") == 0) {
    const char* sessionId = doc["session_id"] | "";
    const char* connectionId = doc["connection_id"] | "";
    const char* commandId = doc["command_id"] | "";
    const char* lifecycle = doc["lifecycle"] | "";
    if (strlen(sessionId) >= sizeof(mqtt_cmdSessionId) ||
        strlen(connectionId) >= sizeof(mqtt_cmdConnectionId) ||
        strlen(commandId) >= sizeof(mqtt_cmdCommandId) ||
        !mqttIsSafeCommandId(connectionId) ||
        !mqttIsSafeCommandId(commandId)) return;

    // pagehide is an explicit, fenced detach intent. Unlike the normal clear
    // used by REST-before-ACK, it may release the physical operator while an
    // immutable response remains in retry. A late old page cannot override a
    // different live owner.
    bool lifecyclePagehide = strcmp(lifecycle, "pagehide") == 0;
    bool lifecycleCommandExact = lifecyclePagehide && mqtt_cmdActive &&
      mqtt_cmdConnectionId[0] != '\0' && connectionId[0] != '\0' &&
      strcmp(mqtt_cmdSessionId, sessionId) == 0 &&
      strcmp(mqtt_cmdCommandId, commandId) == 0 &&
      strcmp(mqtt_cmdConnectionId, connectionId) == 0 &&
      (!mqttOwnerIsFresh() || mqttOwnerAuthorizes(sessionId, connectionId));
    bool lifecycleOwnerExact = lifecyclePagehide &&
      mqttOwnerAuthorizes(sessionId, connectionId);
    bool lifecycleExact = lifecycleCommandExact || lifecycleOwnerExact;
    if (lifecycleExact) {
      mqttEnterLocalFallback();
      mqtt_commandSnapshotReady = true;
      if (mqtt_responsePending) {
        Serial.println(F("[MQTT] pagehide: response in retry, tasti locali liberi"));
        return;
      }
      // The owner detach is independent from command deletion. Only the exact
      // raw command fence may erase it; a takeover page can still free the
      // operator without deleting a predecessor/legacy retained command.
      if (lifecycleCommandExact) mqttResetCommand();
      Serial.println(lifecycleCommandExact
        ? F("[MQTT] pagehide: comando sganciato")
        : F("[MQTT] pagehide: sessione sganciata"));
      return;
    }

    // Pending response bytes are immutable. Only the matching response_ack
    // may close the outbox and reset its command.
    if (mqtt_responsePending) {
      Serial.println(F("[MQTT] CMD clear ignorato: response ACK pending"));
      return;
    }
    if (mqtt_confirmRequestState == MQTT_CONFIRM_REQUEST_QUEUED ||
        mqtt_confirmRequestState == MQTT_CONFIRM_REQUEST_RUNNING) {
      Serial.println(F("[MQTT] CMD clear ignorato: confirm_request pending"));
      return;
    }

    bool initialCommandSnapshot = !mqtt_commandSnapshotReady;
    if (!mqtt_cmdActive) {
      mqtt_commandSnapshotReady = true;
      return;
    }
    if (initialCommandSnapshot && mqtt_cmdConnectionId[0] != '\0' &&
        connectionId[0] != '\0') {
      mqttResetCommand();
      mqtt_commandSnapshotReady = true;
      Serial.println(F("[MQTT] Snapshot clear: comando raw rimosso"));
      return;
    }
    bool activeFullyLegacy = mqtt_cmdCommandId[0] == '\0' &&
      mqtt_cmdSessionId[0] == '\0';
    bool sessionMatches = activeFullyLegacy ||
      strcmp(mqtt_cmdSessionId, sessionId) == 0;
    bool activeFenced = (mqtt_cmdCommandId[0] != '\0');
    bool commandMatches = activeFenced
      ? commandId[0] != '\0' && strcmp(mqtt_cmdCommandId, commandId) == 0
      : commandId[0] == '\0';
    // With a live owner, only its current connection may clear. Without one,
    // the exact retained clear is the broker's authoritative tombstone and
    // must also clean a raw command preserved across an offline interval.
    bool connectionMatches = mqtt_cmdConnectionId[0] == '\0' ||
      (connectionId[0] != '\0' &&
       (!mqttOwnerIsFresh() || mqttOwnerConnectionAuthorizes(connectionId)));
    if (sessionMatches && commandMatches && connectionMatches) {
      if (sessionId[0] != '\0') {
        strlcpy(mqtt_lastSessionId, sessionId, sizeof(mqtt_lastSessionId));
      }
      mqttResetCommand();
      mqtt_commandSnapshotReady = true;
      mqtt_activityFlag = true;
      Serial.println(F("[MQTT] CMD clear: tornato in idle"));
    } else {
      Serial.println(F("[MQTT] CMD clear fence mismatch, ignorato"));
    }

  } else {
    Serial.print(F("[MQTT] Comando tipo sconosciuto: "));
    Serial.println(type);
  }
}

// ========================= CONNESSIONE AL BROKER =========================
static bool mqttAttemptConnect() {
  if (mqttConnectAbortRequested()) return false;
  // Restore the longer establishment timeout before every attempt. A shorter
  // read/write timeout is installed only after the full MQTT handshake.
  mqttWifiClient.setConnectionTimeout(MQTT_TCP_CONNECT_TIMEOUT_MS);

  // Prepara LWT payload (include name per discovery dal browser)
  char lwtPayload[192];
  snprintf(lwtPayload, sizeof(lwtPayload),
    "{\"type\":\"status\",\"state\":\"offline\",\"scale_id\":\"%s\",\"name\":\"%s\"}",
    mqtt_scaleId, MQTT_SCALE_NAME);

  // Verifica che il clock sia sincronizzato (NTP), necessario per TLS
  time_t now = time(nullptr);
  if (now < 1700000000) {  // Prima di ~nov 2023 = clock non sincronizzato
    Serial.println(F("[MQTT] Clock non sincronizzato (NTP in corso), rinvio"));
    return false;
  }

  Serial.print(F("[MQTT] Connessione a "));
  Serial.print(mqtt_host);
  Serial.print(F(":"));
  Serial.print(MQTT_PORT);
  Serial.println(F("..."));

  // Connessione TLS manuale con hostname (per SNI)
  // PubSubClient risolve il DNS e passa l'IP a WiFiClientSecure,
  // perdendo il hostname necessario per SNI. Lo facciamo noi prima.
  if (!mqttWifiClient.connected()) {
    Serial.println(F("[MQTT] TLS handshake..."));
    bool tlsOk = mqttWifiClient.connect(mqtt_host, MQTT_PORT);
    if (!tlsOk) {
      Serial.println(F("[MQTT] TLS handshake fallito"));
      return false;
    }
    Serial.println(F("[MQTT] TLS OK"));
  }
  if (mqttConnectAbortRequested()) return false;

  // connect(clientId, user, pass, willTopic, willQos, willRetain, willMessage)
  bool ok = mqttClient.connect(
    mqtt_clientId,
    mqtt_user,
    mqtt_pass,
    mqtt_topicSts,     // LWT topic
    1,                 // LWT QoS
    true,              // LWT retain
    lwtPayload         // LWT payload
  );

  if (!ok) {
    Serial.print(F("[MQTT] Connessione fallita, rc="));
    Serial.println(mqttClient.state());
    return false;
  }
  if (mqttConnectAbortRequested()) return false;

  Serial.println(F("[MQTT] Connesso al broker!"));

  // Owner is subscribed first so a retained command cannot become actionable
  // before its browser lease snapshot is available.
  bool ownerSubscribed = mqttClient.subscribe(mqtt_topicOwner, 1);
  bool commandSubscribed = mqttClient.subscribe(mqtt_topicCmd, 1);
  bool ackSubscribed = mqttClient.subscribe(mqtt_topicAck, 1);
  if (!ownerSubscribed || !commandSubscribed || !ackSubscribed) {
    Serial.println(F("[MQTT] Subscribe owner/command/ack fallita"));
    mqttClient.disconnect();
    return false;
  }
  if (mqttConnectAbortRequested()) return false;

  // Pubblica status online (retained) — include name per discovery dal browser
  if (mqttPublishStatus("online")) {
    Serial.println(F("[MQTT] Status 'online' pubblicato"));
  } else {
    Serial.println(F("[MQTT] Status 'online' non pubblicato"));
  }

  Serial.println(F("[MQTT] Subscribed owner + command + ack"));

  // From this point the main loop owns normal traffic. Bound TLS reads/writes
  // well below the 8-second loop WDT; connect restores the longer timeout.
  mqttWifiClient.setConnectionTimeout(MQTT_IO_TIMEOUT_MS);

  return true;
}

static void mqttConnectTaskEntry(void* parameter) {
  (void)parameter;
  bool connected = mqttAttemptConnect();

  while (true) {
    MqttConnectAbortState abortState;
    portENTER_CRITICAL(&mqtt_connectTaskMux);
    abortState = mqtt_connectAbortState;
    if (abortState == MQTT_CONNECT_ABORT_NONE) {
      mqtt_connectTaskResult = connected;
      mqtt_connectTaskState = MQTT_CONNECT_DONE;
      portEXIT_CRITICAL(&mqtt_connectTaskMux);
      break;
    }
    mqtt_connectAbortState = MQTT_CONNECT_ABORT_NONE;
    mqtt_connectTaskWasAborted = true;
    portEXIT_CRITICAL(&mqtt_connectTaskMux);

    const char* state = abortState == MQTT_CONNECT_ABORT_SLEEPING
      ? "sleeping" : "offline";
    if (connected && mqttClient.connected()) {
      if (mqttPublishStatus(state)) mqttClient.disconnect();
      else mqttWifiClient.stop();
    } else {
      mqttWifiClient.stop();
    }
    connected = false;
  }

  vTaskDelete(nullptr);
}

static bool mqttStartConnectTask() {
  portENTER_CRITICAL(&mqtt_connectTaskMux);
  if (mqtt_connectTaskState != MQTT_CONNECT_IDLE) {
    portEXIT_CRITICAL(&mqtt_connectTaskMux);
    return false;
  }
  mqtt_connectTaskResult = false;
  mqtt_connectTaskWasAborted = false;
  mqtt_connectAbortState = MQTT_CONNECT_ABORT_NONE;
  mqtt_connectTaskState = MQTT_CONNECT_RUNNING;
  portEXIT_CRITICAL(&mqtt_connectTaskMux);

  BaseType_t created = xTaskCreatePinnedToCore(
    mqttConnectTaskEntry, "mqtt-connect", MQTT_CONNECT_TASK_STACK,
    nullptr, 1, nullptr, 0);
  if (created == pdPASS) return true;

  portENTER_CRITICAL(&mqtt_connectTaskMux);
  mqtt_connectTaskState = MQTT_CONNECT_IDLE;
  portEXIT_CRITICAL(&mqtt_connectTaskMux);
  Serial.println(F("[MQTT] Task connessione non avviato"));
  return false;
}

// ========================= FUNZIONI PUBBLICHE MQTT =========================

void mqttSetup() {
  mqtt_identityValid = buildScaleId();
  if (!mqtt_identityValid) {
    mqtt_scaleId[0] = '\0';
    mqtt_credsLoaded = false;
    mqtt_setupDone = true;
    Serial.println(F("[MQTT] eFuse MAC non valida; MQTT disabilitato"));
    return;
  }
  buildTopics();

  Serial.print(F("[MQTT] scale_id = "));
  Serial.println(mqtt_scaleId);

  if (!mqttClient.setBufferSize(MQTT_MAX_PACKET_SIZE)) {
    Serial.println(F("[MQTT] Buffer 512 alloc fallita; MQTT disabilitato"));
    mqtt_setupDone = true;
    return;
  }

  // Carica credenziali da NVS
  mqtt_credsLoaded = MqttStore::load(
    mqtt_host, sizeof(mqtt_host),
    mqtt_user, sizeof(mqtt_user),
    mqtt_pass, sizeof(mqtt_pass)
  );

  if (!mqtt_credsLoaded) {
    Serial.println(F("[MQTT] ATTENZIONE: credenziali non configurate. Usa la seriale: mqtt set \"host\" \"user\" \"pass\""));
    Serial.println(F("[MQTT] Setup completato (in attesa di configurazione)"));
    mqtt_setupDone = true;
    return;
  }

  Serial.print(F("[MQTT] Host: "));
  Serial.println(mqtt_host);
  Serial.print(F("[MQTT] User: "));
  Serial.println(mqtt_user);

  // NTP necessario per validazione certificato TLS (ESP32 parte da epoch 1970)
  configTime(3600, 3600, "pool.ntp.org", "time.google.com");  // CET + DST
  Serial.println(F("[MQTT] NTP configurato (pool.ntp.org)"));

  mqttWifiClient.setCACert(MQTT_CA_CERT);
  mqttWifiClient.setConnectionTimeout(MQTT_TCP_CONNECT_TIMEOUT_MS);
  mqttWifiClient.setHandshakeTimeout(MQTT_TLS_HANDSHAKE_TIMEOUT_SEC);
  mqttClient.setServer(mqtt_host, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(MQTT_KEEPALIVE_SEC);
  mqttClient.setSocketTimeout(MQTT_SOCKET_TIMEOUT_SEC);

  mqtt_setupDone = true;
  mqtt_wasPrevConnected = false;
  mqtt_lastAttemptMs = 0;
  mqtt_backoffMs = MQTT_BACKOFF_INIT;
  mqtt_disconnectBeep = false;
  mqtt_commandRxBeep = false;

  mqtt_activityFlag = false;
  mqtt_undoAckFlag = false;

  Serial.println(F("[MQTT] Setup completato"));
}

void mqttSuspend(const char* state) {
  if (!mqtt_setupDone) return;
  if (state && strcmp(state, "offline") == 0 &&
      (mqtt_cmdConnectionId[0] != '\0' || mqtt_responsePending)) {
    mqttEnterLocalFallback();
  }
  MqttConnectAbortState abortState = state && strcmp(state, "sleeping") == 0
    ? MQTT_CONNECT_ABORT_SLEEPING : MQTT_CONNECT_ABORT_OFFLINE;
  bool workerOwnsClient = mqttRequestConnectAbort(abortState);
  if (!workerOwnsClient && mqtt_connectTaskState == MQTT_CONNECT_DONE) {
    portENTER_CRITICAL(&mqtt_connectTaskMux);
    mqtt_connectTaskState = MQTT_CONNECT_IDLE;
    mqtt_connectAbortState = MQTT_CONNECT_ABORT_NONE;
    mqtt_connectTaskResult = false;
    mqtt_connectTaskWasAborted = false;
    portEXIT_CRITICAL(&mqtt_connectTaskMux);
  }
  bool hadActiveState = workerOwnsClient || mqttClientConnectedForMain() ||
    mqtt_wasPrevConnected || (mqtt_lastAttemptMs != 0) || mqtt_cmdActive;
  if (!workerOwnsClient && mqttClientConnectedForMain()) {
    // Disconnect cleanly only after the retained sleeping state is accepted.
    // Otherwise close the transport so the broker publishes the offline LWT.
    if (mqttPublishStatus(state ? state : "offline")) mqttClient.disconnect();
    else mqttWifiClient.stop();
  }
  mqtt_wasPrevConnected = false;
  mqttResetOwnerSnapshot();
  mqtt_commandSnapshotReady = false;
  mqtt_lastAttemptMs = 0;
  mqtt_backoffMs = MQTT_BACKOFF_INIT;
  if (hadActiveState) {
    Serial.println(F("[MQTT] Sospeso"));
  }
}

bool isMqttConnected() {
  return mqtt_setupDone && mqtt_identityValid && mqttClientConnectedForMain();
}

bool isMqttConnectInProgress() {
  return mqttConnectTaskIsRunning();
}

const char* getScaleId() {
  return mqtt_scaleId;
}

bool isMqttCommandActive() {
  return mqtt_cmdActive;
}

bool isMqttCommandActionable() {
  return mqtt_cmdActive && !mqtt_localFallbackActive &&
    (mqtt_cmdConnectionId[0] == '\0' ||
     (mqtt_commandSnapshotReady &&
      mqttOwnerAuthorizes(mqtt_cmdSessionId, mqtt_cmdConnectionId)));
}

bool isMqttResponsePending() {
  return mqtt_responsePending;
}

bool isMqttResponseOperatorLocked() {
  return mqtt_responsePending && !mqtt_localFallbackActive;
}

bool isMqttLocalFallbackActive() {
  return mqtt_localFallbackActive;
}

bool isMqttManagerAttached() {
  return !mqtt_localFallbackActive && mqttOwnerIsFresh();
}

bool mqttDetachForLocalInput() {
  if (!mqtt_responsePending) return false;
  mqttEnterLocalFallback();
  return true;
}

float getMqttTargetWeight() {
  return mqtt_cmdTargetWeight;
}

const char* getMqttCommandUuid() {
  return mqtt_cmdUuid;
}

const char* getMqttCommandName() {
  return mqtt_cmdName;
}

const char* getMqttCommandSessionId() {
  return mqtt_cmdSessionId;
}

const char* getMqttCommandConnectionId() {
  return mqtt_cmdConnectionId;
}

const char* getMqttCommandId() {
  return mqtt_cmdCommandId;
}

const char* getMqttLastSessionId() {
  return mqtt_lastSessionId;
}

uint32_t getMqttCommandProductId() {
  return mqtt_cmdProductId;
}

bool mqttPopConfirmRequest() {
  if (mqtt_confirmRequestState != MQTT_CONFIRM_REQUEST_QUEUED) return false;
  if (!isMqttCommandActionable()) {
    mqttFailConfirmRequest("owner");
    return false;
  }
  mqtt_confirmRequestState = MQTT_CONFIRM_REQUEST_RUNNING;
  return true;
}

void mqttMarkConfirmRequestStaged() {
  if (mqtt_confirmRequestState != MQTT_CONFIRM_REQUEST_RUNNING) return;
  mqtt_confirmRequestState = MQTT_CONFIRM_REQUEST_STAGED;
  mqtt_confirmRequestReason[0] = '\0';
  mqttPublishConfirmRequestAck(
    mqtt_confirmRequest.requestId, mqtt_confirmRequest.commandId,
    mqtt_confirmRequest.connectionId, "staged", "");
}

void mqttFailConfirmRequest(const char* reason) {
  if (mqtt_confirmRequestState != MQTT_CONFIRM_REQUEST_RUNNING &&
      mqtt_confirmRequestState != MQTT_CONFIRM_REQUEST_QUEUED) return;
  mqtt_confirmRequestState = MQTT_CONFIRM_REQUEST_FAILED;
  strlcpy(
    mqtt_confirmRequestReason, reason ? reason : "not_ready",
    sizeof(mqtt_confirmRequestReason));
  mqttRememberTerminalConfirmRequest(
    mqtt_confirmRequest.requestId, mqtt_confirmRequest.sessionId,
    mqtt_confirmRequest.connectionId, mqtt_confirmRequest.commandId,
    mqtt_confirmRequest.uuid, mqtt_confirmRequest.productId,
    "failed", mqtt_confirmRequestReason);
}

bool mqttConfirmActiveCommand(const char* expectedUuid, float actualWeight,
                              char* responseIdOut, size_t responseIdOutSize) {
  if (responseIdOut && responseIdOutSize > 0) responseIdOut[0] = '\0';
  if (!isMqttCommandActionable() || mqtt_responsePending || !expectedUuid ||
      strcmp(mqtt_cmdUuid, expectedUuid) != 0) {
    return false;
  }

  char weightStr[16];
  dtostrf(actualWeight, 1, 1, weightStr);

  // Legacy browser compatibility: one attempt and no ACK outbox.
  if (mqtt_cmdSessionId[0] == '\0') {
    if (!mqttClientConnectedForMain()) return false;
    char legacyPayload[256];
    int written = mqtt_cmdCommandId[0] != '\0'
      ? snprintf(legacyPayload, sizeof(legacyPayload),
          "{\"type\":\"confirm\",\"uuid\":\"%s\",\"product_id\":%lu,\"command_id\":\"%s\",\"actual_weight\":%s}",
          expectedUuid, (unsigned long)mqtt_cmdProductId,
          mqtt_cmdCommandId, weightStr)
      : snprintf(legacyPayload, sizeof(legacyPayload),
          "{\"type\":\"confirm\",\"uuid\":\"%s\",\"product_id\":%lu,\"actual_weight\":%s}",
          expectedUuid, (unsigned long)mqtt_cmdProductId, weightStr);
    if (written < 0 || written >= (int)sizeof(legacyPayload)) return false;
    bool published = mqttClient.publish(
      mqtt_topicRsp, (const uint8_t*)legacyPayload, strlen(legacyPayload), false);
    if (!published) return false;
    mqttResetCommand();
    return true;
  }

  mqttGenerateResponseId();
  int written = mqtt_cmdCommandId[0] != '\0'
    ? snprintf(mqtt_responsePayload, sizeof(mqtt_responsePayload),
        "{\"type\":\"confirm\",\"uuid\":\"%s\",\"product_id\":%lu,\"session_id\":\"%s\",\"command_id\":\"%s\",\"response_id\":\"%s\",\"actual_weight\":%s}",
        expectedUuid, (unsigned long)mqtt_cmdProductId, mqtt_cmdSessionId,
        mqtt_cmdCommandId, mqtt_responseId, weightStr)
    : snprintf(mqtt_responsePayload, sizeof(mqtt_responsePayload),
        "{\"type\":\"confirm\",\"uuid\":\"%s\",\"product_id\":%lu,\"session_id\":\"%s\",\"response_id\":\"%s\",\"actual_weight\":%s}",
        expectedUuid, (unsigned long)mqtt_cmdProductId, mqtt_cmdSessionId,
        mqtt_responseId, weightStr);
  if (written < 0 || written >= (int)sizeof(mqtt_responsePayload)) {
    mqttResetPendingResponse();
    return false;
  }
  mqtt_responseKind = MQTT_RESPONSE_CONFIRM;
  mqtt_responsePending = true;
  mqtt_responseStartedMs = millis();
  mqtt_responseLastPublishMs = 0;
  mqttPublishPendingResponse();
  if (responseIdOut && responseIdOutSize > 0) {
    strlcpy(responseIdOut, mqtt_responseId, responseIdOutSize);
  }
  return true;
}

bool mqttSkipActiveCommand() {
  if (!isMqttCommandActionable() || mqtt_responsePending) return false;

  if (mqtt_cmdSessionId[0] == '\0') {
    if (!mqttClientConnectedForMain()) return false;
    char legacyPayload[224];
    int written = mqtt_cmdCommandId[0] != '\0'
      ? snprintf(legacyPayload, sizeof(legacyPayload),
          "{\"type\":\"skip\",\"uuid\":\"%s\",\"product_id\":%lu,\"command_id\":\"%s\"}",
          mqtt_cmdUuid, (unsigned long)mqtt_cmdProductId, mqtt_cmdCommandId)
      : snprintf(legacyPayload, sizeof(legacyPayload),
          "{\"type\":\"skip\",\"uuid\":\"%s\",\"product_id\":%lu}",
          mqtt_cmdUuid, (unsigned long)mqtt_cmdProductId);
    if (written < 0 || written >= (int)sizeof(legacyPayload)) return false;
    bool published = mqttClient.publish(
      mqtt_topicRsp, (const uint8_t*)legacyPayload, strlen(legacyPayload), false);
    if (!published) return false;
    mqttResetCommand();
    return true;
  }

  mqttGenerateResponseId();
  int written = mqtt_cmdCommandId[0] != '\0'
    ? snprintf(mqtt_responsePayload, sizeof(mqtt_responsePayload),
        "{\"type\":\"skip\",\"uuid\":\"%s\",\"product_id\":%lu,\"session_id\":\"%s\",\"command_id\":\"%s\",\"response_id\":\"%s\"}",
        mqtt_cmdUuid, (unsigned long)mqtt_cmdProductId,
        mqtt_cmdSessionId, mqtt_cmdCommandId, mqtt_responseId)
    : snprintf(mqtt_responsePayload, sizeof(mqtt_responsePayload),
        "{\"type\":\"skip\",\"uuid\":\"%s\",\"product_id\":%lu,\"session_id\":\"%s\",\"response_id\":\"%s\"}",
        mqtt_cmdUuid, (unsigned long)mqtt_cmdProductId,
        mqtt_cmdSessionId, mqtt_responseId);
  if (written < 0 || written >= (int)sizeof(mqtt_responsePayload)) {
    mqttResetPendingResponse();
    return false;
  }
  mqtt_responseKind = MQTT_RESPONSE_SKIP;
  mqtt_responsePending = true;
  mqtt_responseStartedMs = millis();
  mqtt_responseLastPublishMs = 0;
  mqttPublishPendingResponse();
  return true;
}

bool mqttStageUndo(const char* uuid, uint32_t productId,
                   const char* undoOfResponseId, const char* sessionId) {
  if (mqtt_responsePending || !mqtt_setupDone || !mqtt_identityValid ||
      !uuid || uuid[0] == '\0' ||
      strlen(uuid) >= sizeof(mqtt_cmdUuid) || productId == 0 ||
      !undoOfResponseId || undoOfResponseId[0] == '\0' ||
      strlen(undoOfResponseId) >= sizeof(mqtt_responseId)) return false;

  const char* targetSession = (sessionId && sessionId[0] != '\0')
    ? sessionId : mqtt_lastSessionId;
  if (!targetSession || targetSession[0] == '\0' ||
      strlen(targetSession) >= sizeof(mqtt_lastSessionId)) return false;

  mqttGenerateResponseId();
  int written = snprintf(mqtt_responsePayload, sizeof(mqtt_responsePayload),
    "{\"type\":\"undo\",\"uuid\":\"%s\",\"product_id\":%lu,\"session_id\":\"%s\",\"response_id\":\"%s\",\"undo_of_response_id\":\"%s\"}",
    uuid, (unsigned long)productId, targetSession, mqtt_responseId,
    undoOfResponseId);
  if (written < 0 || written >= (int)sizeof(mqtt_responsePayload)) {
    mqttResetPendingResponse();
    return false;
  }
  strlcpy(mqtt_lastSessionId, targetSession, sizeof(mqtt_lastSessionId));
  mqtt_responseKind = MQTT_RESPONSE_UNDO;
  mqtt_responsePending = true;
  mqtt_responseStartedMs = millis();
  mqtt_responseLastPublishMs = 0;
  if (mqtt_localFallbackActive) mqtt_detachedUndoFlag = true;
  mqttPublishPendingResponse();
  return true;
}

// Chiamata internamente da update() — gestisce riconnessione con backoff
static void mqttUpdateInternal() {
  if (!mqtt_setupDone) return;

  // The worker owns mqttWifiClient/mqttClient exclusively while RUNNING.
  // Consume its result before any main-loop access to either object.
  bool connectCompleted = false;
  bool connectSucceeded = false;
  bool connectAborted = false;
  portENTER_CRITICAL(&mqtt_connectTaskMux);
  if (mqtt_connectTaskState == MQTT_CONNECT_RUNNING) {
    portEXIT_CRITICAL(&mqtt_connectTaskMux);
    return;
  }
  if (mqtt_connectTaskState == MQTT_CONNECT_DONE) {
    connectCompleted = true;
    connectSucceeded = mqtt_connectTaskResult;
    connectAborted = mqtt_connectTaskWasAborted;
    mqtt_connectTaskState = MQTT_CONNECT_IDLE;
    mqtt_connectAbortState = MQTT_CONNECT_ABORT_NONE;
    mqtt_connectTaskResult = false;
    mqtt_connectTaskWasAborted = false;
  }
  portEXIT_CRITICAL(&mqtt_connectTaskMux);

  if (mqtt_reloadAfterConnect) {
    mqtt_reloadAfterConnect = false;
    mqttReloadCreds();
    return;
  }

  if (connectCompleted && connectSucceeded) {
    bool transportStillUsable = mqttClient.connected() &&
      mqtt_identityValid && mqtt_credsLoaded && WiFi.isConnected();
#if ENABLE_WIFI_OTA
    transportStillUsable = transportStillUsable && !otaInProgress;
#endif
    if (transportStillUsable) {
      // No retained callback ran in the worker. Start a fresh owner/command
      // snapshot before mqttClient.loop() begins delivery on the main task.
      mqttResetOwnerSnapshot();
      mqtt_commandSnapshotReady = false;
      mqtt_wasPrevConnected = true;
      mqtt_backoffMs = MQTT_BACKOFF_INIT;
      mqtt_disconnectBeep = false;
    } else {
      mqttWifiClient.stop();
      connectSucceeded = false;
      connectAborted = true;
    }
  }

  if (connectCompleted && !connectSucceeded && !connectAborted) {
    mqtt_lastAttemptMs = millis();
    mqtt_backoffMs *= 2;
    if (mqtt_backoffMs > MQTT_BACKOFF_MAX) mqtt_backoffMs = MQTT_BACKOFF_MAX;
    Serial.print(F("[MQTT] Prossimo tentativo tra "));
    Serial.print(mqtt_backoffMs / 1000);
    Serial.println(F("s"));
  }

  if (!mqtt_identityValid) return;
  if (!mqtt_credsLoaded) return;  // Credenziali non configurate: non tentare
#if ENABLE_WIFI_OTA
  if (otaInProgress) return;
#endif
  if (!WiFi.isConnected()) {
    // WiFi giù: reset stato MQTT
    if (mqtt_wasPrevConnected) {
      mqtt_wasPrevConnected = false;
      mqttResetOwnerSnapshot();
      mqtt_commandSnapshotReady = false;
      Serial.println(F("[MQTT] WiFi perso, MQTT disconnesso"));
    }
    mqtt_backoffMs = MQTT_BACKOFF_INIT;
    mqtt_lastAttemptMs = 0;
    return;
  }

  bool connectedNow = mqttClientConnectedForMain();

  // Transizione: era connesso → disconnesso
  if (mqtt_wasPrevConnected && !connectedNow) {
    Serial.println(F("[MQTT] Disconnesso dal broker"));
    mqtt_disconnectBeep = true;  // segnala al .ino per buzzerWarn x2
    mqtt_backoffMs = MQTT_BACKOFF_INIT;
    mqtt_lastAttemptMs = 0;
    mqttResetOwnerSnapshot();
    mqtt_commandSnapshotReady = false;
  }
  mqtt_wasPrevConnected = connectedNow;

  if (connectedNow) {
    // Connesso: processa messaggi in arrivo
    mqttClient.loop();
    uint32_t now = millis();
    if (mqtt_responsePending &&
        (mqtt_responseLastPublishMs == 0 ||
         (now - mqtt_responseLastPublishMs) >= MQTT_RESPONSE_RETRY_MS)) {
      mqttPublishPendingResponse();
    }
    return;
  }

  // Non connesso: tenta riconnessione con backoff
  uint32_t now = millis();
  if (mqtt_lastAttemptMs != 0 && (now - mqtt_lastAttemptMs) < mqtt_backoffMs) {
    return;  // ancora in attesa
  }

  mqtt_lastAttemptMs = now;
  if (!mqttStartConnectTask()) {
    // Task allocation failures follow the same bounded retry policy.
    mqtt_lastAttemptMs = millis();
    mqtt_backoffMs = mqtt_backoffMs * 2;
    if (mqtt_backoffMs > MQTT_BACKOFF_MAX) mqtt_backoffMs = MQTT_BACKOFF_MAX;
    Serial.print(F("[MQTT] Prossimo tentativo tra "));
    Serial.print(mqtt_backoffMs / 1000);
    Serial.println(F("s"));
  }
}

// Consumato dal .ino per emettere buzzerWarn x2
bool mqttPopDisconnectBeep() {
  if (mqtt_disconnectBeep) {
    mqtt_disconnectBeep = false;
    return true;
  }
  return false;
}

bool mqttPopCommandRxBeep() {
  if (mqtt_commandRxBeep) {
    mqtt_commandRxBeep = false;
    return true;
  }
  return false;
}

bool mqttPopActivity() {
  if (!mqtt_activityFlag) return false;
  mqtt_activityFlag = false;
  return true;
}

bool mqttPopUndoAck() {
  if (!mqtt_undoAckFlag) return false;
  mqtt_undoAckFlag = false;
  return true;
}

bool mqttPopDetachedUndo() {
  if (!mqtt_detachedUndoFlag) return false;
  mqtt_detachedUndoFlag = false;
  return true;
}

void mqttReloadCreds() {
  if (!mqtt_identityValid) {
    Serial.println(F("[MQTT] eFuse MAC non valida; reload ignorato"));
    return;
  }
  if (mqttRequestConnectAbort(MQTT_CONNECT_ABORT_OFFLINE)) {
    mqtt_reloadAfterConnect = true;
    Serial.println(F("[MQTT] Reload credenziali accodato"));
    return;
  }
  mqtt_reloadAfterConnect = false;
  if (mqtt_connectTaskState == MQTT_CONNECT_DONE) {
    portENTER_CRITICAL(&mqtt_connectTaskMux);
    mqtt_connectTaskState = MQTT_CONNECT_IDLE;
    mqtt_connectAbortState = MQTT_CONNECT_ABORT_NONE;
    mqtt_connectTaskResult = false;
    mqtt_connectTaskWasAborted = false;
    portEXIT_CRITICAL(&mqtt_connectTaskMux);
  }
  // Disconnetti se connesso
  if (mqttClientConnectedForMain()) {
    mqttClient.disconnect();
  }

  if (!mqttClient.setBufferSize(MQTT_MAX_PACKET_SIZE)) {
    Serial.println(F("[MQTT] Buffer 512 alloc fallita"));
    return;
  }

  // Ricarica da NVS
  mqtt_credsLoaded = MqttStore::load(
    mqtt_host, sizeof(mqtt_host),
    mqtt_user, sizeof(mqtt_user),
    mqtt_pass, sizeof(mqtt_pass)
  );

  if (!mqtt_credsLoaded) {
    Serial.println(F("[MQTT] Credenziali non configurate"));
    return;
  }

  Serial.print(F("[MQTT] Credenziali ricaricate. Host: "));
  Serial.println(mqtt_host);
  Serial.print(F("[MQTT] User: "));
  Serial.println(mqtt_user);

  // Riconfigura il client con il nuovo host
  mqttClient.setServer(mqtt_host, MQTT_PORT);
  mqttClient.setKeepAlive(MQTT_KEEPALIVE_SEC);
  mqttClient.setSocketTimeout(MQTT_SOCKET_TIMEOUT_SEC);

  // Reset backoff per tentare subito
  mqtt_wasPrevConnected = false;
  mqttResetOwnerSnapshot();
  mqtt_commandSnapshotReady = false;
  mqtt_lastAttemptMs = 0;
  mqtt_backoffMs = MQTT_BACKOFF_INIT;
  mqtt_disconnectBeep = false;
  mqtt_commandRxBeep = false;
  mqtt_activityFlag = false;

  // NTP (nel caso non fosse stato configurato al setup perché mancavano le credenziali)
  configTime(3600, 3600, "pool.ntp.org", "time.google.com");

  // TLS
  mqttWifiClient.setCACert(MQTT_CA_CERT);
  mqttWifiClient.setConnectionTimeout(MQTT_TCP_CONNECT_TIMEOUT_MS);
  mqttWifiClient.setHandshakeTimeout(MQTT_TLS_HANDSHAKE_TIMEOUT_SEC);
  mqttClient.setCallback(mqttCallback);

  Serial.println(F("[MQTT] Pronto per la riconnessione"));
}

bool isMqttConfigured() {
  return mqtt_credsLoaded;
}

#endif // ENABLE_MQTT

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
          wifiPendingAttemptAtMs = 0;
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

            if (wifiPendingAttemptAtMs != 0) {
              if ((int32_t)(now - wifiPendingAttemptAtMs) >= 0) {
                wifiKickAttempt(wifiPendingAttemptIdx);
              }
            } else if (wifiAttemptStartMs == 0) {
              wifiKickAttempt(0);
            } else {
              if ((now - wifiAttemptStartMs) >= WIFI_CONNECT_TIMEOUT_MS) {
                if (wifiTriedThisRound < wifiCredsN) {
                  wifiScheduleAttempt(wifiTriedThisRound, now);
                } else {
                  WiFi.disconnect(false, false);
                  wifiAttemptStartMs = 0;
                  wifiRetryAtMs = now + WIFI_BACKOFF_MS;
                  wifiPendingAttemptAtMs = 0;
                  wifiTriedThisRound = 0;
                }
              }
            }
          }
        }
      } else {
        wifiAttemptStartMs = 0;
        wifiRetryAtMs = 0;
        wifiPendingAttemptAtMs = 0;
        wifiTriedThisRound = 0;
      }
    }
  }

  if (otaBegun) {
    ArduinoOTA.handle();
  }
#endif

#if ENABLE_MQTT
  mqttUpdateLocalFallback();
  mqttUpdateInternal();
  mqttUpdateLocalFallback();
#endif

#if ENABLE_ARDUINO_CLOUD
  ArduinoCloud.update();
#endif
}

} // namespace Net
