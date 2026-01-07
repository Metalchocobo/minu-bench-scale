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
  #include "wifi_store.h"
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

  // Credenziali WiFi (max 2) caricate da NVS (Preferences).
  // NOTA: non vanno mai hardcodate / versionate nel repo.
  struct WifiCred { const char* ssid; const char* pass; };

  static char wifiSsid1[WifiStore::SSID_MAX_LEN + 1] = {0};
  static char wifiPass1[WifiStore::PASS_MAX_LEN + 1] = {0};
  static char wifiSsid2[WifiStore::SSID_MAX_LEN + 1] = {0};
  static char wifiPass2[WifiStore::PASS_MAX_LEN + 1] = {0};

  static WifiCred wifiCreds[2];
  static uint8_t  wifiCredsN = 0;

  inline void wifiRebuildCredList() {
    wifiCredsN = 0;
    if (WifiStore::isConfiguredSsid(wifiSsid1)) {
      wifiCreds[wifiCredsN++] = { wifiSsid1, wifiPass1 };
    }
    if (WifiStore::isConfiguredSsid(wifiSsid2)) {
      wifiCreds[wifiCredsN++] = { wifiSsid2, wifiPass2 };
    }
  }

  inline void wifiLoadCredsFromNVS(bool log = false) {
    WifiStore::loadSlot(1, wifiSsid1, sizeof(wifiSsid1), wifiPass1, sizeof(wifiPass1));
    WifiStore::loadSlot(2, wifiSsid2, sizeof(wifiSsid2), wifiPass2, sizeof(wifiPass2));
    wifiRebuildCredList();
    if (log) {
      Serial.print(F("[NET] WiFi creds: "));
      Serial.print(wifiCredsN);
      Serial.println(F(" rete/i configurata/e."));
    }
  }

  inline uint8_t wifiCredCount() { return wifiCredsN; }

  inline void wifiKickAttempt(uint8_t idx) {
    if (wifiCredsN == 0) {
      // Nessuna credenziale configurata: niente tentativi.
      return;
    }
    wifiAttemptIdx = idx % wifiCredsN;
    wifiAttemptStartMs = millis();
    wifiTriedThisRound = (wifiTriedThisRound < wifiCredsN) ? (wifiTriedThisRound + 1) : wifiCredsN;

    // WiFi.begin() su ESP32 è asincrona: ritorna subito, la connessione avviene in background.
    WiFi.begin(wifiCreds[wifiAttemptIdx].ssid, wifiCreds[wifiAttemptIdx].pass);
    Serial.print(F("[NET] WiFi: tentativo connessione a: "));
    Serial.println(wifiCreds[wifiAttemptIdx].ssid);
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
    wifiLoadCredsFromNVS(true);
    if (wifiCredsN > 0) {
      Serial.println(F("[NET] WiFi avviato (background)."));
      // Avvia subito il primo tentativo (non blocca)
      wifiKickAttempt(0);
    } else {
      Serial.println(F("[NET] WiFi ON ma nessuna credenziale configurata. Usa la seriale per impostarle."));
    }
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

    wifiLoadCredsFromNVS(false);
    if (wifiCredsN > 0) {
      // Riparti subito con un tentativo (non blocca)
      wifiKickAttempt(0);
    }
  }

  // Ricarica le credenziali da NVS e riavvia i tentativi (senza reboot).
  inline void wifiReloadCredsAndRestart() {
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
          // Nessuna credenziale configurata: non spammare tentativi.
          if (wifiCredsN == 0) {
            wifiAttemptStartMs = 0;
            wifiRetryAtMs = 0;
            wifiTriedThisRound = 0;
          } else {
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
                if (wifiTriedThisRound < wifiCredsN) {
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
