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

// =============================================================================
// MQTT CLIENT
// =============================================================================
#if ENABLE_MQTT

// ========================= ISRG Root X1 (Let's Encrypt) =========================
static const char MQTT_CA_CERT[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogZiUvsEkrBVuo/Hv6JHYR+REvCqHkEB44Y1NLRL0vv/4nX
e+WObf8Tn7UaREgALOo+lfKJlxeT2ykVPFYbqjJRH2GP+gDRq6a/8LKEUCg9l6T
f3JiEfrlBNeRVLCchVGJq+R0bfVWgeyBmQjm/Kfzx1pCwT4peMFGCYP+eFJFVZYE
sNH4Ag7uKsd9h8RqPDhDJSDXU4WV1BrQOGB8NRGX/0u0yYMAQ0LjdVbM9OKfBdsM
0L2utrsV/XA+wsol5sDkADN4VyFE+9mRjNwBtgfqMss79raDmzOkcMl3NpK7JKKn
fCRidlgrkJwXh6K3zDkMComxUvMFqemHpJqVNkUMP4q9BrKRA7Pxs5npaEGCH5Y4
lXi3mzaElQ2B2YNPa0FGmJk7dJCnNRnGGnnYbgDGNU/jc3bNjKBQAb4zhYJWh/GZ
VwOTO5OcNFk6FIYV5bR/4X0J9TIDAoKZPP+c8L1JdwwH3XU9KWCLHEB/qA0=
-----END CERTIFICATE-----
)EOF";

// ========================= STATO MQTT =========================
static WiFiClientSecure mqttWifiClient;
static PubSubClient     mqttClient(mqttWifiClient);

static char   mqtt_scaleId[13]  = {0};  // MAC lowercase, no separators (12 hex + null)
static char   mqtt_topicCmd[48] = {0};  // minu/scale/{id}/command
static char   mqtt_topicRsp[48] = {0};  // minu/scale/{id}/response
static char   mqtt_topicSts[48] = {0};  // minu/scale/{id}/status
static char   mqtt_clientId[32] = {0};  // scale-{scaleId}

// Stato comando attivo
static bool   mqtt_cmdActive       = false;
static char   mqtt_cmdUuid[37]     = {0};  // UUID standard 36 chars + null
static float  mqtt_cmdTargetWeight = 0.0f;

// Riconnessione con backoff esponenziale
static bool     mqtt_setupDone       = false;
static bool     mqtt_wasPrevConnected = false;
static uint32_t mqtt_lastAttemptMs   = 0;
static uint32_t mqtt_backoffMs       = 2000;
static const uint32_t MQTT_BACKOFF_INIT = 2000;
static const uint32_t MQTT_BACKOFF_MAX  = 30000;

// Flag per segnalare disconnessione (per buzzer nel .ino)
static bool     mqtt_disconnectBeep  = false;

// Forward declaration
static void mqttCallback(char* topic, byte* payload, unsigned int length);
static bool mqttAttemptConnect();

// ========================= HELPER: MAC → scale_id =========================
static void buildScaleId() {
  String mac = WiFi.macAddress();  // "AA:BB:CC:DD:EE:FF"
  int j = 0;
  for (unsigned int i = 0; i < mac.length() && j < 12; i++) {
    char c = mac.charAt(i);
    if (c != ':') {
      mqtt_scaleId[j++] = (c >= 'A' && c <= 'F') ? (c + 32) : c;  // lowercase
    }
  }
  mqtt_scaleId[j] = '\0';
}

static void buildTopics() {
  snprintf(mqtt_topicCmd, sizeof(mqtt_topicCmd), "minu/scale/%s/command",  mqtt_scaleId);
  snprintf(mqtt_topicRsp, sizeof(mqtt_topicRsp), "minu/scale/%s/response", mqtt_scaleId);
  snprintf(mqtt_topicSts, sizeof(mqtt_topicSts), "minu/scale/%s/status",   mqtt_scaleId);
  snprintf(mqtt_clientId, sizeof(mqtt_clientId), "scale-%s",               mqtt_scaleId);
}

// ========================= CALLBACK RICEZIONE COMANDI =========================
static void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Ignora topic diversi dal nostro command (safety)
  if (strcmp(topic, mqtt_topicCmd) != 0) return;

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

  if (strcmp(type, "weigh") == 0) {
    const char* uuid   = doc["uuid"];
    float targetWeight = doc["target_weight"] | 0.0f;
    const char* name   = doc["name"] | "";

    if (!uuid || strlen(uuid) == 0) {
      Serial.println(F("[MQTT] weigh senza uuid, ignorato"));
      return;
    }

    // Salva comando attivo
    strncpy(mqtt_cmdUuid, uuid, sizeof(mqtt_cmdUuid) - 1);
    mqtt_cmdUuid[sizeof(mqtt_cmdUuid) - 1] = '\0';
    mqtt_cmdTargetWeight = targetWeight;
    mqtt_cmdActive = true;

    Serial.print(F("[MQTT] CMD weigh: uuid="));
    Serial.print(mqtt_cmdUuid);
    Serial.print(F(" target="));
    Serial.print(mqtt_cmdTargetWeight, 1);
    Serial.print(F("g name="));
    Serial.println(name);

  } else if (strcmp(type, "clear") == 0) {
    mqtt_cmdActive = false;
    mqtt_cmdUuid[0] = '\0';
    mqtt_cmdTargetWeight = 0.0f;
    Serial.println(F("[MQTT] CMD clear: tornato in idle"));

  } else {
    Serial.print(F("[MQTT] Comando tipo sconosciuto: "));
    Serial.println(type);
  }
}

// ========================= CONNESSIONE AL BROKER =========================
static bool mqttAttemptConnect() {
  // Prepara LWT payload
  char lwtPayload[128];
  snprintf(lwtPayload, sizeof(lwtPayload),
    "{\"type\":\"status\",\"state\":\"offline\",\"scale_id\":\"%s\"}", mqtt_scaleId);

  Serial.print(F("[MQTT] Connessione a "));
  Serial.print(MQTT_HOST);
  Serial.print(F(":"));
  Serial.print(MQTT_PORT);
  Serial.println(F("..."));

  // connect(clientId, user, pass, willTopic, willQos, willRetain, willMessage)
  bool ok = mqttClient.connect(
    mqtt_clientId,
    MQTT_USER,
    MQTT_PASS,
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

  // Pubblica status online (retained)
  char statusPayload[160];
  snprintf(statusPayload, sizeof(statusPayload),
    "{\"type\":\"status\",\"state\":\"online\",\"scale_id\":\"%s\",\"firmware_version\":\"%s\"}",
    mqtt_scaleId, MQTT_FW_VERSION);
  mqttClient.publish(mqtt_topicSts, (const uint8_t*)statusPayload, strlen(statusPayload), true);
  Serial.println(F("[MQTT] Status 'online' pubblicato"));

  // Subscribe al topic command
  mqttClient.subscribe(mqtt_topicCmd, 1);
  Serial.print(F("[MQTT] Subscribed a: "));
  Serial.println(mqtt_topicCmd);

  return true;
}

// ========================= FUNZIONI PUBBLICHE MQTT =========================

void mqttSetup() {
  buildScaleId();
  buildTopics();

  Serial.print(F("[MQTT] scale_id = "));
  Serial.println(mqtt_scaleId);

  mqttWifiClient.setCACert(MQTT_CA_CERT);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  mqtt_setupDone = true;
  mqtt_wasPrevConnected = false;
  mqtt_lastAttemptMs = 0;
  mqtt_backoffMs = MQTT_BACKOFF_INIT;
  mqtt_disconnectBeep = false;

  Serial.println(F("[MQTT] Setup completato"));
}

void mqttSuspend() {
  if (!mqtt_setupDone) return;
  if (mqttClient.connected()) {
    mqttClient.disconnect();
  }
  mqtt_wasPrevConnected = false;
  mqtt_lastAttemptMs = 0;
  mqtt_backoffMs = MQTT_BACKOFF_INIT;
  mqtt_cmdActive = false;
  mqtt_cmdUuid[0] = '\0';
  mqtt_cmdTargetWeight = 0.0f;
  Serial.println(F("[MQTT] Sospeso"));
}

bool isMqttConnected() {
  return mqtt_setupDone && mqttClient.connected();
}

const char* getScaleId() {
  return mqtt_scaleId;
}

bool isMqttCommandActive() {
  return mqtt_cmdActive;
}

float getMqttTargetWeight() {
  return mqtt_cmdTargetWeight;
}

const char* getMqttCommandUuid() {
  return mqtt_cmdUuid;
}

void mqttClearActiveCommand() {
  mqtt_cmdActive = false;
  mqtt_cmdUuid[0] = '\0';
  mqtt_cmdTargetWeight = 0.0f;
}

void mqttPublishConfirm(float actualWeight) {
  if (!mqttClient.connected() || !mqtt_cmdActive) return;

  char buf[160];
  // Formatta con 1 decimale
  char weightStr[16];
  dtostrf(actualWeight, 1, 1, weightStr);
  snprintf(buf, sizeof(buf),
    "{\"type\":\"confirm\",\"uuid\":\"%s\",\"actual_weight\":%s}",
    mqtt_cmdUuid, weightStr);

  mqttClient.publish(mqtt_topicRsp, (const uint8_t*)buf, strlen(buf), false);
  Serial.print(F("[MQTT] Pubblicato confirm: "));
  Serial.println(buf);
}

void mqttPublishSkip() {
  if (!mqttClient.connected() || !mqtt_cmdActive) return;

  char buf[96];
  snprintf(buf, sizeof(buf),
    "{\"type\":\"skip\",\"uuid\":\"%s\"}", mqtt_cmdUuid);

  mqttClient.publish(mqtt_topicRsp, (const uint8_t*)buf, strlen(buf), false);
  Serial.print(F("[MQTT] Pubblicato skip: "));
  Serial.println(buf);
}

// Chiamata internamente da update() — gestisce riconnessione con backoff
static void mqttUpdateInternal() {
  if (!mqtt_setupDone) return;
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

#if ENABLE_MQTT
  mqttUpdateInternal();
#endif

#if ENABLE_ARDUINO_CLOUD
  ArduinoCloud.update();
#endif
}

} // namespace Net
