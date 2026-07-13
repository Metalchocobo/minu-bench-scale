#pragma once

// Gestione WiFi + OTA + MQTT + Arduino Cloud per Minù Bench Scale
// NOTE:
//  - Di default è attivo solo WiFi + OTA via Arduino IDE.
//  - Arduino Cloud è disabilitato; abilitalo cambiando ENABLE_ARDUINO_CLOUD a 1
//    e aggiungendo thingProperties.h generato da Arduino Cloud nella stessa cartella.

// Abilita / disabilita i vari moduli
#define ENABLE_WIFI_OTA       1   // 1 = attiva WiFi + OTA
#define ENABLE_ARDUINO_CLOUD  0   // 1 = attiva Arduino Cloud (richiede librerie + thingProperties.h)
#define ENABLE_MQTT           1   // 1 = attiva client MQTT per integrazione gestionale

#if ENABLE_WIFI_OTA
  #include <WiFi.h>
  #include <ESPmDNS.h>
  #include <WiFiUdp.h>
  #include <ArduinoOTA.h>
  #include "wifi_store.h"
  #include "ota_store.h"
#endif

#if ENABLE_MQTT
  // Header-side size; mqttSetup/mqttReloadCreds also enforce 512 at runtime.
  #define MQTT_MAX_PACKET_SIZE 512
  #include <WiFiClientSecure.h>
  #include <PubSubClient.h>
  #include <ArduinoJson.h>
  #include "mqtt_store.h"
#endif

#if ENABLE_ARDUINO_CLOUD
  #include <ArduinoIoTCloud.h>
  #include <Arduino_ConnectionHandler.h>
  #include "thingProperties.h"
#endif

// ========================= MQTT CONFIG =========================
#if ENABLE_MQTT
  // Host, username e password NON sono nel codice: si configurano via seriale
  // e vengono salvati in NVS (persistono tra aggiornamenti firmware).
  // Comandi: mqtt set "host" "user" "pass", mqtt creds, mqtt clear, mqtt apply
  #define MQTT_PORT       8883          // MQTTS (TLS)
  #define MQTT_FW_VERSION "1.1.0"
  #define MQTT_SCALE_NAME "Minu Bench Scale"  // Nome visibile nel browser
#endif

namespace Net {

#if ENABLE_WIFI_OTA
  // Inizializza WiFi STA (NON BLOCCANTE)
  void wifiSetup();

  // Sospende WiFi/OTA (prima di light-sleep)
  void wifiSuspend();

  // Ripristina WiFi dopo wake
  void wifiResume();

  // Ricarica credenziali da NVS e riavvia tentativi
  void wifiReloadCredsAndRestart();

  // Numero di credenziali configurate
  uint8_t wifiCredCount();

  // Configura OTA (registra handler)
  void otaSetup(const char* hostname = "minu-bench-scale");

  // True se OTA update in corso
  bool isOtaInProgress();
#endif

#if ENABLE_MQTT
  // Inizializza MQTT (da chiamare dopo wifiSetup)
  void mqttSetup();

  // Sospende MQTT (prima di light-sleep)
  void mqttSuspend();

  // True se connesso al broker MQTT
  bool isMqttConnected();

  // Stage a confirm for the active command. Session-aware commands stay active
  // until the browser acknowledges the generated response_id.
  bool mqttConfirmActiveCommand(const char* expectedUuid, float actualWeight);

  // Stage/publish a skip for the active command.
  bool mqttSkipActiveCommand();

  // Restituisce il scale_id (MAC formattato)
  const char* getScaleId();

  // Stato comando attivo
  bool   isMqttCommandActive();
  bool   isMqttResponsePending();
  float  getMqttTargetWeight();
  const char* getMqttCommandUuid();
  const char* getMqttCommandName();

  // Ritorna true una sola volta se MQTT si è disconnesso (per buzzer x2 nel .ino)
  bool mqttPopDisconnectBeep();

  // Ritorna true una sola volta quando arriva un nuovo comando weigh
  bool mqttPopCommandRxBeep();

  // Ricarica credenziali MQTT da NVS e riconnette (senza reboot)
  void mqttReloadCreds();

  // True se le credenziali MQTT sono configurate in NVS
  bool isMqttConfigured();
#endif

#if ENABLE_ARDUINO_CLOUD
  // Inizializza Arduino Cloud
  void cloudSetup();
#endif

  // Da chiamare nel loop principale
  void update();

} // namespace Net
