#pragma once

// Gestione WiFi + OTA + Arduino Cloud per Minù Bench Scale
// NOTE:
//  - WiFi/OTA sono compilati ma disattivati di default e dipendono da credenziali
//    presenti in Preferences (vedi settings.h).
//  - Arduino Cloud è disabilitato; abilitalo cambiando ENABLE_ARDUINO_CLOUD a 1
//    e aggiungendo thingProperties.h generato da Arduino Cloud nella stessa cartella.

// Abilita / disabilita i vari moduli
#define ENABLE_WIFI_OTA       1   // 1 = compila WiFi + OTA, runtime controllato via Preferences
#define ENABLE_ARDUINO_CLOUD  0   // 1 = attiva Arduino Cloud (richiede librerie + thingProperties.h)

#include "settings.h"

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

struct NetStatus {
  bool enabled;         // WiFi attivato dal configuration (cred + flag)
  bool connected;       // WiFi::isConnected()
  bool otaEnabled;      // OTA attivo (config + WiFi on)
  bool hasCredentials;  // almeno un SSID valido in config
};

namespace Net {

#if ENABLE_WIFI_OTA
  // Gestore multi-SSID
  static WiFiMulti wifiMulti;

  // Stato interno (WiFi non bloccante)
  static bool     wifiConfigured   = false;
  static bool     wifiPaused       = false;
  static uint32_t wifiLastRunMs    = 0;
  static bool     wifiWasConnected = false;
  static WifiSettings g_wifiSettings;
  static bool     g_hasCredentials = false;

  // OTA non bloccante: config (handler) e begin quando WiFi è connesso
  static bool     otaBegun       = false;
  static bool     otaInProgress  = false;
  static const char* otaHostname = "minu-bench-scale";

  inline bool hasAnyCredential(const WifiSettings& cfg) {
    return cfg.primary.ssid.length() > 0 || cfg.secondary.ssid.length() > 0;
  }

  // Configura WiFi manager ma NON abilita ancora OTA/Cloud.
  inline void begin(const WifiSettings &cfg) {
    g_wifiSettings = cfg;
    g_hasCredentials = hasAnyCredential(cfg);

    if (!cfg.wifiEnabled || !g_hasCredentials) {
      wifiConfigured = false;
      wifiPaused = true;
      wifiWasConnected = false;
      otaBegun = false;
      otaInProgress = false;
      return;
    }

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);

    wifiMulti = WiFiMulti();
    if (cfg.primary.ssid.length() > 0) {
      wifiMulti.addAP(cfg.primary.ssid.c_str(), cfg.primary.password.c_str());
    }
    if (cfg.secondary.ssid.length() > 0) {
      wifiMulti.addAP(cfg.secondary.ssid.c_str(), cfg.secondary.password.c_str());
    }

    wifiConfigured = true;
    wifiPaused     = false;
    wifiLastRunMs  = 0;
    wifiWasConnected = false;

    Serial.println(F("[NET] WiFi abilitato da config (background)."));

    if (cfg.otaEnabled) {
      ArduinoOTA.setHostname(otaHostname);

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
    }
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

  // Da chiamare nel loop principale
  inline void update() {
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
        if (g_wifiSettings.otaEnabled && !otaBegun && WiFi.isConnected()) {
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
  }

  inline NetStatus status() {
    const bool enabled = wifiConfigured && !wifiPaused;
    const bool connected = enabled && WiFi.isConnected();
    return NetStatus{
      enabled,
      connected,
      enabled && g_wifiSettings.otaEnabled,
      g_hasCredentials
    };
  }

#else
  // Stub quando il WiFi non è compilato
  inline void begin(const WifiSettings&) {}
  inline void wifiSuspend() {}
  inline void wifiResume() {}
  inline void update() {}
  inline bool isOtaInProgress() { return false; }
  inline NetStatus status() { return NetStatus{false, false, false, false}; }
#endif

} // namespace Net
