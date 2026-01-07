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
  // Stato interno WiFi (non bloccante)
  static bool     wifiConfigured = false;
  static bool     wifiPaused     = false;
  static uint32_t wifiLastRunMs  = 0;
  static bool     wifiWasConnected = false;

  // Connessione non bloccante (senza WiFiMulti.run, che su ESP32 può bloccare per secondi)
  // Strategia: tentativi sequenziali su una lista di SSID, con timeout e backoff.
  static uint8_t  wifiTriedThisRound = 0;   // quanti SSID provati in questo giro
  static uint8_t  wifiAttemptIdx     = 0;   // SSID attuale
  static uint32_t wifiAttemptStartMs = 0;   // 0 = nessun tentativo in corso
  static uint32_t wifiRetryAtMs      = 0;   // backoff prima di riprovare da capo

  static const uint32_t WIFI_TICK_MS             = 200;   // polling leggero
  static const uint32_t WIFI_CONNECT_TIMEOUT_MS  = 8000;  // timeout per singolo SSID
  static const uint32_t WIFI_BACKOFF_MS          = 3000;  // pausa dopo aver provato tutti gli SSID

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

  struct WifiCred { const char* ssid; const char* pass; };
  static const WifiCred WIFI_CREDS[] = {
    { WIFI_SSID_1, WIFI_PASS_1 },
    { WIFI_SSID_2, WIFI_PASS_2 },
  };
  static const uint8_t WIFI_CREDS_N = (uint8_t)(sizeof(WIFI_CREDS) / sizeof(WIFI_CREDS[0]));

  inline void wifiKickAttempt(uint8_t idx) {
    if (WIFI_CREDS_N == 0) return;
    wifiAttemptIdx = idx % WIFI_CREDS_N;
    wifiAttemptStartMs = millis();
    wifiTriedThisRound = (wifiTriedThisRound < WIFI_CREDS_N) ? (wifiTriedThisRound + 1) : WIFI_CREDS_N;

    // WiFi.begin() su ESP32 è asincrona: ritorna subito, la connessione avviene in background.
    WiFi.begin(WIFI_CREDS[wifiAttemptIdx].ssid, WIFI_CREDS[wifiAttemptIdx].pass);
    Serial.print(F("[NET] WiFi: tentativo connessione a: "));
    Serial.println(WIFI_CREDS[wifiAttemptIdx].ssid);
  }

  // Inizializza WiFi STA (NON BLOCCANTE): la connessione avviene in background dentro Net::update()
  inline void wifiSetup() {
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
    Serial.println(F("[NET] WiFi avviato (background)."));

    // Avvia subito il primo tentativo (non blocca)
    wifiKickAttempt(0);
  }

  // Sospende WiFi/OTA (utile prima di andare in light-sleep)
  inline void wifiSuspend() {
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

  // Ripristina WiFi (senza ri-aggiungere le reti)
  inline void wifiResume() {
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
    // OTA verrà ri-abilitato automaticamente quando torni connesso (dentro update())

    // Riparti subito con un tentativo (non blocca)
    wifiKickAttempt(0);
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
    // Tick WiFi (non bloccante): niente WiFiMulti.run(), che può bloccare display e HX.
    if (wifiConfigured && !wifiPaused) {
      uint32_t now = millis();
      if (now - wifiLastRunMs >= WIFI_TICK_MS) {
        wifiLastRunMs = now;

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

        // Se non siamo connessi, gestiamo tentativi / timeout / backoff in modo non bloccante.
        if (!connectedNow) {
          // Se siamo in backoff, aspetta.
          if (wifiRetryAtMs != 0 && (int32_t)(now - wifiRetryAtMs) < 0) {
            // ancora in backoff
          } else {
            if (wifiRetryAtMs != 0 && (int32_t)(now - wifiRetryAtMs) >= 0) {
              // backoff finito: riparti da capo
              wifiRetryAtMs = 0;
              wifiTriedThisRound = 0;
              wifiAttemptStartMs = 0;
            }

            // Se non c'è un tentativo in corso, avvialo.
            if (wifiAttemptStartMs == 0) {
              wifiKickAttempt(0);
            } else {
              // Timeout del tentativo attuale
              if ((now - wifiAttemptStartMs) >= WIFI_CONNECT_TIMEOUT_MS) {
                if (wifiTriedThisRound < WIFI_CREDS_N) {
                  wifiKickAttempt(wifiTriedThisRound);
                } else {
                  // Provati tutti: backoff e poi riparti
                  wifiAttemptStartMs = 0;
                  wifiRetryAtMs = now + WIFI_BACKOFF_MS;
                  wifiTriedThisRound = 0;
                }
              }
            }
          }
        } else {
          // Connesso: reset tentativi
          wifiAttemptStartMs = 0;
          wifiRetryAtMs = 0;
          wifiTriedThisRound = 0;
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
