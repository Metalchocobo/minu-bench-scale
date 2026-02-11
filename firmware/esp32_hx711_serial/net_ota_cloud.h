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
  #include "ota_store.h"
#endif

#if ENABLE_ARDUINO_CLOUD
  #include <ArduinoIoTCloud.h>
  #include <Arduino_ConnectionHandler.h>
  #include "thingProperties.h"
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

#if ENABLE_ARDUINO_CLOUD
  // Inizializza Arduino Cloud
  void cloudSetup();
#endif

  // Da chiamare nel loop principale
  void update();

} // namespace Net
