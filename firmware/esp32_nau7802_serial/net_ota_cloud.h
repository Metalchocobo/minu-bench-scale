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

  // Due reti fittizie da sostituire con i tuoi dati reali
  static const char* WIFI_SSID_1 = "Shadowfiend";
  static const char* WIFI_PASS_1 = "questa dannata rete";
  static const char* WIFI_SSID_2 = "Laboratorio di Minu'";
  static const char* WIFI_PASS_2 = "questa dannata rete";

  // Inizializza WiFi STA e tenta connessione alle reti configurate
  inline void wifiSetup() {
    WiFi.mode(WIFI_STA);

    // Aggiungi qui le reti disponibili (puoi aggiungerne altre se vuoi)
    wifiMulti.addAP(WIFI_SSID_1, WIFI_PASS_1);
    wifiMulti.addAP(WIFI_SSID_2, WIFI_PASS_2);

    Serial.println(F("[NET] Connessione WiFi..."));
    int retries = 0;

    while (wifiMulti.run() != WL_CONNECTED) {
      delay(500);
      Serial.print('.');
      retries++;
      if (retries > 60) {  // ~30 secondi
        Serial.println(F("\n[NET] WiFi non disponibile, proseguo offline."));
        break;
      }
    }

    if (WiFi.isConnected()) {
      Serial.print(F("\n[NET] Connesso a: "));
      Serial.println(WiFi.SSID());
      Serial.print(F("[NET] IP: "));
      Serial.println(WiFi.localIP());
    }
  }

  // Configura OTA via Arduino IDE (porta di rete)
  inline void otaSetup(const char* hostname = "minu-bench-scale") {
    if (!WiFi.isConnected()) {
      Serial.println(F("[OTA] WiFi non connesso, OTA disabilitato."));
      return;
    }

    ArduinoOTA.setHostname(hostname);

    ArduinoOTA.onStart([]() {
      Serial.println(F("[OTA] Update iniziato"));
      // Se serve, qui puoi sospendere logica della bilancia
    });

    ArduinoOTA.onEnd([]() {
      Serial.println(F("\n[OTA] Update finito"));
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      if (total == 0) return;
      Serial.printf("[OTA] Avanzamento: %u%%\r", (progress * 100) / total);
    });

    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("[OTA] Errore[%u]\n", error);
    });

    ArduinoOTA.begin();
    Serial.println(F("[OTA] Pronto. In Arduino IDE seleziona la porta di rete dell'ESP32."));
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
    ArduinoOTA.handle();
  #endif

  #if ENABLE_ARDUINO_CLOUD
    ArduinoCloud.update();
  #endif
  }

} // namespace Net
