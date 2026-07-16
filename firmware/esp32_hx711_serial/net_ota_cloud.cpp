#include "net_ota_cloud.h"
#include "config/config_scale.h"
#include <esp_mac.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

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
  "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\n"
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
static char   mqtt_clientId[32] = {0};  // scale-{scaleId}

// Stato comando attivo
static bool   mqtt_cmdActive       = false;
static char   mqtt_cmdUuid[37]     = {0};  // UUID standard 36 chars + null
static char   mqtt_cmdSessionId[65] = {0}; // Empty for legacy commands
static char   mqtt_cmdCommandId[65] = {0}; // Empty for pre-v1.3 commands
static char   mqtt_lastSessionId[65] = {0};
static char   mqtt_cmdName[32]     = {0};  // Nome ingrediente (troncato)
static float  mqtt_cmdTargetWeight = 0.0f;
static uint32_t mqtt_cmdProductId  = 0;

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
static uint32_t mqtt_responseCounter = 0;
static const uint32_t MQTT_RESPONSE_RETRY_MS = 1000;

// Riconnessione con backoff esponenziale
static bool     mqtt_setupDone       = false;
static bool     mqtt_wasPrevConnected = false;
static uint32_t mqtt_lastAttemptMs   = 0;
static uint32_t mqtt_backoffMs       = 2000;
static const uint32_t MQTT_BACKOFF_INIT = 2000;
static const uint32_t MQTT_BACKOFF_MAX  = 30000;
static const uint16_t MQTT_KEEPALIVE_SEC = 15;
static const uint16_t MQTT_SOCKET_TIMEOUT_SEC = 2;

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
static void mqttResetCommand();
static void mqttResetPendingResponse();
static bool mqttPublishPendingResponse();
static bool mqttPublishCommandAck();
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
  snprintf(mqtt_clientId, sizeof(mqtt_clientId), "scale-%s",               mqtt_scaleId);
}

static void mqttResetCommand() {
  mqtt_cmdActive = false;
  mqtt_cmdUuid[0] = '\0';
  mqtt_cmdSessionId[0] = '\0';
  mqtt_cmdCommandId[0] = '\0';
  mqtt_cmdName[0] = '\0';
  mqtt_cmdTargetWeight = 0.0f;
  mqtt_cmdProductId = 0;
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

static void mqttResetPendingResponse() {
  mqtt_responsePending = false;
  mqtt_responseKind = MQTT_RESPONSE_NONE;
  mqtt_responseId[0] = '\0';
  mqtt_responsePayload[0] = '\0';
  mqtt_responseLastPublishMs = 0;
}

static void mqttGenerateResponseId() {
  snprintf(mqtt_responseId, sizeof(mqtt_responseId), "%08lx%08lx%08lx",
    (unsigned long)esp_random(), (unsigned long)millis(),
    (unsigned long)++mqtt_responseCounter);
}

static bool mqttPublishPendingResponse() {
  if (!mqtt_responsePending || !mqttClient.connected()) return false;
  mqtt_responseLastPublishMs = millis();
  bool published = mqttClient.publish(
    mqtt_topicRsp,
    (const uint8_t*)mqtt_responsePayload,
    strlen(mqtt_responsePayload),
    false
  );
  if (!published) Serial.println(F("[MQTT] Response publish fallito; retry attivo"));
  return published;
}

static bool mqttPublishCommandAck() {
  if (!mqtt_cmdActive || mqtt_cmdCommandId[0] == '\0' ||
      !mqttClient.connected()) return false;

  char payload[320];
  int written = snprintf(payload, sizeof(payload),
    "{\"type\":\"command_ack\",\"command_id\":\"%s\",\"session_id\":\"%s\",\"uuid\":\"%s\",\"product_id\":%lu,\"state\":\"active\"}",
    mqtt_cmdCommandId, mqtt_cmdSessionId, mqtt_cmdUuid,
    (unsigned long)mqtt_cmdProductId);
  return written > 0 && written < (int)sizeof(payload) &&
    mqttClient.publish(
      mqtt_topicRsp, (const uint8_t*)payload, strlen(payload), false);
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
  if (!isCommandTopic && !isAckTopic) return;

  // Parse JSON
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.print(F("[MQTT] JSON parse error: "));
    Serial.println(err.c_str());
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
      if (mqtt_responseKind == MQTT_RESPONSE_UNDO) mqtt_undoAckFlag = true;
      mqttResetPendingResponse();
      mqttResetCommand();
    }
    return;
  }

  if (strcmp(type, "weigh") == 0) {
    const char* uuid   = doc["uuid"];
    const char* sessionId = doc["session_id"] | "";
    const char* commandId = doc["command_id"] | "";
    float targetWeight = doc["target_weight"] | 0.0f;
    uint32_t productId = doc["product_id"].is<uint32_t>()
      ? doc["product_id"].as<uint32_t>() : 0;
    const char* name   = doc["name"] | "";

    if (!uuid || strlen(uuid) == 0 || strlen(uuid) >= sizeof(mqtt_cmdUuid) ||
        strlen(sessionId) >= sizeof(mqtt_cmdSessionId) ||
        strlen(commandId) >= sizeof(mqtt_cmdCommandId) ||
        !mqttIsSafeCommandId(commandId)) {
      Serial.println(F("[MQTT] weigh con ID non valido, ignorato"));
      return;
    }

    if (mqtt_responsePending) {
      Serial.println(F("[MQTT] CMD weigh ignorato: response ACK pending"));
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
    strncpy(mqtt_cmdUuid, uuid, sizeof(mqtt_cmdUuid) - 1);
    mqtt_cmdUuid[sizeof(mqtt_cmdUuid) - 1] = '\0';
    strncpy(mqtt_cmdSessionId, sessionId, sizeof(mqtt_cmdSessionId) - 1);
    mqtt_cmdSessionId[sizeof(mqtt_cmdSessionId) - 1] = '\0';
    strncpy(mqtt_cmdCommandId, commandId, sizeof(mqtt_cmdCommandId) - 1);
    mqtt_cmdCommandId[sizeof(mqtt_cmdCommandId) - 1] = '\0';
    strncpy(mqtt_cmdName, name, sizeof(mqtt_cmdName) - 1);
    mqtt_cmdName[sizeof(mqtt_cmdName) - 1] = '\0';
    mqtt_cmdTargetWeight = targetWeight;
    mqtt_cmdProductId = productId;
    mqtt_cmdActive = true;
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
    const char* commandId = doc["command_id"] | "";
    if (strlen(sessionId) >= sizeof(mqtt_cmdSessionId) ||
        strlen(commandId) >= sizeof(mqtt_cmdCommandId) ||
        !mqttIsSafeCommandId(commandId)) return;

    // Pending response bytes are immutable. Only the matching response_ack
    // may close the outbox and reset its command.
    if (mqtt_responsePending) {
      Serial.println(F("[MQTT] CMD clear ignorato: response ACK pending"));
      return;
    }

    if (!mqtt_cmdActive) return;
    bool activeFullyLegacy = mqtt_cmdCommandId[0] == '\0' &&
      mqtt_cmdSessionId[0] == '\0';
    bool sessionMatches = activeFullyLegacy ||
      strcmp(mqtt_cmdSessionId, sessionId) == 0;
    bool activeFenced = (mqtt_cmdCommandId[0] != '\0');
    bool commandMatches = activeFenced
      ? commandId[0] != '\0' && strcmp(mqtt_cmdCommandId, commandId) == 0
      : commandId[0] == '\0';
    if (sessionMatches && commandMatches) {
      if (sessionId[0] != '\0') {
        strlcpy(mqtt_lastSessionId, sessionId, sizeof(mqtt_lastSessionId));
      }
      mqttResetCommand();
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
    configureLoopWdt(ScaleConfig::MQTT_TLS_WDT_TIMEOUT_MS);
    bool tlsOk = mqttWifiClient.connect(mqtt_host, MQTT_PORT);
    configureLoopWdt(ScaleConfig::LOOP_WDT_TIMEOUT_MS);
    if (!tlsOk) {
      Serial.println(F("[MQTT] TLS handshake fallito"));
      return false;
    }
    Serial.println(F("[MQTT] TLS OK"));
  }

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

  Serial.println(F("[MQTT] Connesso al broker!"));

  // Session-aware responses require both command delivery and ACK delivery.
  bool commandSubscribed = mqttClient.subscribe(mqtt_topicCmd, 1);
  bool ackSubscribed = mqttClient.subscribe(mqtt_topicAck, 1);
  if (!commandSubscribed || !ackSubscribed) {
    Serial.println(F("[MQTT] Subscribe command/ack fallita"));
    mqttClient.disconnect();
    return false;
  }

  // Pubblica status online (retained) — include name per discovery dal browser
  if (mqttPublishStatus("online")) {
    Serial.println(F("[MQTT] Status 'online' pubblicato"));
  } else {
    Serial.println(F("[MQTT] Status 'online' non pubblicato"));
  }

  Serial.println(F("[MQTT] Subscribed command + ack"));

  return true;
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
  mqttWifiClient.setHandshakeTimeout(10);  // 10s per TLS handshake
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
  bool hadActiveState = mqttClient.connected() || mqtt_wasPrevConnected || (mqtt_lastAttemptMs != 0) || mqtt_cmdActive;
  if (mqttClient.connected()) {
    // Disconnect cleanly only after the retained sleeping state is accepted.
    // Otherwise close the transport so the broker publishes the offline LWT.
    if (mqttPublishStatus(state ? state : "offline")) mqttClient.disconnect();
    else mqttWifiClient.stop();
  }
  mqtt_wasPrevConnected = false;
  mqtt_lastAttemptMs = 0;
  mqtt_backoffMs = MQTT_BACKOFF_INIT;
  if (hadActiveState) {
    Serial.println(F("[MQTT] Sospeso"));
  }
}

bool isMqttConnected() {
  return mqtt_setupDone && mqtt_identityValid && mqttClient.connected();
}

const char* getScaleId() {
  return mqtt_scaleId;
}

bool isMqttCommandActive() {
  return mqtt_cmdActive;
}

bool isMqttResponsePending() {
  return mqtt_responsePending;
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

const char* getMqttCommandId() {
  return mqtt_cmdCommandId;
}

const char* getMqttLastSessionId() {
  return mqtt_lastSessionId;
}

uint32_t getMqttCommandProductId() {
  return mqtt_cmdProductId;
}

bool mqttConfirmActiveCommand(const char* expectedUuid, float actualWeight,
                              char* responseIdOut, size_t responseIdOutSize) {
  if (responseIdOut && responseIdOutSize > 0) responseIdOut[0] = '\0';
  if (!mqtt_cmdActive || mqtt_responsePending || !expectedUuid ||
      strcmp(mqtt_cmdUuid, expectedUuid) != 0) {
    return false;
  }

  char weightStr[16];
  dtostrf(actualWeight, 1, 1, weightStr);

  // Legacy browser compatibility: one attempt and no ACK outbox.
  if (mqtt_cmdSessionId[0] == '\0') {
    if (!mqttClient.connected()) return false;
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
  mqtt_responseLastPublishMs = 0;
  mqttPublishPendingResponse();
  if (responseIdOut && responseIdOutSize > 0) {
    strlcpy(responseIdOut, mqtt_responseId, responseIdOutSize);
  }
  return true;
}

bool mqttSkipActiveCommand() {
  if (!mqtt_cmdActive || mqtt_responsePending) return false;

  if (mqtt_cmdSessionId[0] == '\0') {
    if (!mqttClient.connected()) return false;
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
  mqtt_responseLastPublishMs = 0;
  mqttPublishPendingResponse();
  return true;
}

// Chiamata internamente da update() — gestisce riconnessione con backoff
static void mqttUpdateInternal() {
  if (!mqtt_setupDone) return;
  if (!mqtt_identityValid) return;
  if (!mqtt_credsLoaded) return;  // Credenziali non configurate: non tentare
  if (!WiFi.isConnected()) {
    // WiFi giù: reset stato MQTT
    if (mqtt_wasPrevConnected) {
      mqtt_wasPrevConnected = false;
      Serial.println(F("[MQTT] WiFi perso, MQTT disconnesso"));
    }
    mqtt_backoffMs = MQTT_BACKOFF_INIT;
    mqtt_lastAttemptMs = 0;
    return;
  }

  bool connectedNow = mqttClient.connected();

  // Transizione: era connesso → disconnesso
  if (mqtt_wasPrevConnected && !connectedNow) {
    Serial.println(F("[MQTT] Disconnesso dal broker"));
    mqtt_disconnectBeep = true;  // segnala al .ino per buzzerWarn x2
    mqtt_backoffMs = MQTT_BACKOFF_INIT;
    mqtt_lastAttemptMs = 0;
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
  if (mqttAttemptConnect()) {
    mqtt_wasPrevConnected = true;
    mqtt_backoffMs = MQTT_BACKOFF_INIT;  // reset backoff
    mqtt_disconnectBeep = false;
  } else {
    // Backoff esponenziale: 2s → 4s → 8s → 16s → 30s (cap)
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

void mqttReloadCreds() {
  if (!mqtt_identityValid) {
    Serial.println(F("[MQTT] eFuse MAC non valida; reload ignorato"));
    return;
  }
  // Disconnetti se connesso
  if (mqttClient.connected()) {
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
  mqtt_lastAttemptMs = 0;
  mqtt_backoffMs = MQTT_BACKOFF_INIT;
  mqtt_disconnectBeep = false;
  mqtt_commandRxBeep = false;
  mqtt_activityFlag = false;

  // NTP (nel caso non fosse stato configurato al setup perché mancavano le credenziali)
  configTime(3600, 3600, "pool.ntp.org", "time.google.com");

  // TLS
  mqttWifiClient.setCACert(MQTT_CA_CERT);
  mqttWifiClient.setHandshakeTimeout(10);
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
  mqttUpdateInternal();
#endif

#if ENABLE_ARDUINO_CLOUD
  ArduinoCloud.update();
#endif
}

} // namespace Net
