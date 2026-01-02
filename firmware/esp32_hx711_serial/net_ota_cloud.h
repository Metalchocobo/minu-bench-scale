#pragma once

// Gestione WiFi + OTA + Arduino Cloud per Minù Bench Scale
// NOTE:
//  - Di default è attivo solo WiFi + OTA via Arduino IDE.
//  - Arduino Cloud è disabilitato; abilitalo cambiando ENABLE_ARDUINO_CLOUD a 1
//    e aggiungendo thingProperties.h generato da Arduino Cloud nella stessa cartella.

// Abilita / disabilita i vari moduli
#define ENABLE_WIFI_OTA       1   // 1 = attiva WiFi + OTA
#define ENABLE_ARDUINO_CLOUD  0   // 1 = attiva Arduino Cloud (richiede librerie + thingProperties.h)

#if ENABLE_WIFI_OTA
  #include <WiFi.h>
  #include <WiFiMulti.h>
  #include <ESPmDNS.h>
  #include <WiFiUdp.h>
  #include <ArduinoOTA.h>
#endif

#if ENABLE_ARDUINO_CLOUD
  #include <ArduinoIoTCloud.h>
  #include <Arduino_ConnectionHandler.h>
  #include "thingProperties.h"
#endif

namespace Net {

#if ENABLE_WIFI_OTA
  // Gestore multi-SSID
  static WiFiMulti wifiMulti;

  // Stato interno (WiFi non bloccante)
  static bool     wifiConfigured = false;
  static bool     wifiPaused     = false;
  static uint32_t wifiLastRunMs  = 0;
  static bool     wifiWasConnected = false;

  // OTA non bloccante: config (handler) e begin quando WiFi è connesso
  static bool     otaConfigured  = false;
  static bool     otaBegun       = false;
  static bool     otaInProgress  = false;
  static const char* otaHostname = "minu-bench-scale";

  // Due reti fittizie da sostituire con i tuoi dati reali
  static const char* WIFI_SSID_1 = "Shadowfiend";
  static const char* WIFI_PASS_1 = "questa dannata rete";
  static const char* WIFI_SSID_2 = "Laboratorio di Minu'";
  static const char* WIFI_PASS_2 = "questa dannata rete";

  // Inizializza WiFi STA (NON BLOCCANTE): la connessione avviene in background dentro Net::update()
  inline void wifiSetup() {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);

    // Aggiungi qui le reti disponibili (puoi aggiungerne altre se vuoi)
    wifiMulti.addAP(WIFI_SSID_1, WIFI_PASS_1);
    wifiMulti.addAP(WIFI_SSID_2, WIFI_PASS_2);

    wifiConfigured = true;
    wifiPaused     = false;
    wifiLastRunMs  = 0;
    wifiWasConnected = false;
    Serial.println(F("[NET] WiFi avviato (background)."));
  }

  // Sospende WiFi/OTA (utile prima di andare in light-sleep)
  inline void wifiSuspend() {
    if (!wifiConfigured) return;
    wifiPaused = true;
    otaBegun = false;
    otaInProgress = false;
    wifiWasConnected = false;
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }

  // Ripristina WiFi (senza ri-aggiungere le reti)
  inline void wifiResume() {
    if (!wifiConfigured) return;
    wifiPaused = false;
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);
    wifiLastRunMs = 0;
    wifiWasConnected = false;
    // OTA verrà ri-abilitato automaticamente quando torni connesso (dentro update())
  }

  inline bool isOtaInProgress() { return otaInProgress; }

  // Configura OTA via Arduino IDE (porta di rete)
  // NON BLOCCANTE: registra gli handler e fa begin() appena il WiFi risulta connesso.
  inline void otaSetup(const char* hostname = "minu-bench-scale") {
    otaHostname  = hostname;
    otaConfigured = true;
    otaBegun      = false;

    ArduinoOTA.setHostname(otaHostname);

    ArduinoOTA.onStart([]() {
      Serial.println(F("[OTA] Update iniziato"));
      otaInProgress = true;
      // Se serve, qui puoi sospendere logica della bilancia
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
  // Inizializza Arduino Cloud dopo che il WiFi è attivo
  inline void cloudSetup() {
    Serial.println(F("[CLOUD] initProperties()..."));
    initProperties(); // definito in thingProperties.h

    Serial.println(F("[CLOUD] ArduinoCloud.begin()..."));
    ArduinoCloud.begin(ArduinoIoTPreferredConnection);

    setDebugMessageLevel(2);
    ArduinoCloud.printDebugInfo();
  }
#endif // ENABLE_ARDUINO_CLOUD

  // Da chiamare nel loop principale
  inline void update() {
  #if ENABLE_WIFI_OTA
    // Tick WiFi (ogni ~1s) per tentare/ri-tentare la connessione
    if (wifiConfigured && !wifiPaused) {
      uint32_t now = millis();
      if (now - wifiLastRunMs >= 1000) {
        wifiLastRunMs = now;
        wifiMulti.run();

        // Log transizioni
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

        // Avvia OTA appena siamo connessi
        if (otaConfigured && !otaBegun && WiFi.isConnected()) {
          ArduinoOTA.begin();
          otaBegun = true;
          Serial.println(F("[OTA] Pronto (WiFi connesso). In Arduino IDE seleziona la porta di rete dell'ESP32."));
        }
      }
    }

    // Gestione OTA solo se attivato
    if (otaBegun) {
      ArduinoOTA.handle();
    }
  #endif

  #if ENABLE_ARDUINO_CLOUD
    ArduinoCloud.update();
  #endif
  }

} // namespace Net
