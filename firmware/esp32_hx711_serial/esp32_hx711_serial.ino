
// =======================================================
// ESP32 DevKit + HX711 + OLED SSD1322 256x64
// Versione "light" con stati STABLE/UNSTABLE + ZT + OLED
//  • HX711: DOUT=GPIO35, SCK=GPIO16 (RX2)
//    NOTE: il rate 10 Hz / 80 Hz è HARDWARE (pin RATE del modulo)
//  • INA219 su I2C (monitor batteria)
//  • OLED SSD1322 su SPI hardware (CLK=18, MOSI=23)
//    CS=D21, DC=D22, RST=D19
// =======================================================

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <U8g2lib.h>
#include "battery_monitor.h"
#include "hx711_driver.h"
#include "hx_health.h"
#include "ui_display.h"
#include "keypad.h"
#include "scale_core.h"
#include <Preferences.h>
#include <math.h>
#include "buzzer.h"
#include "net_ota_cloud.h"
#include "ota_store.h"
#include "dfplayer_driver.h"
#include <esp_sleep.h>
#include <driver/gpio.h>

Preferences prefs;

// ========================= WIFI (preferenza utente persistente) =========================
// g_wifiUserEnabled = scelta dell'utente (memorizzata in NVS), NON lo stato momentaneo (es. WiFi spento in sleep).
static bool g_wifiUserEnabled = true;
static bool g_wifiSetupDone   = false;
static const char* NVS_NS_SYS       = "minu_sys";
static const char* NVS_KEY_WIFI_USER = "wifi_user";

// Audio eventi WiFi (annunci connessione). Alcune condizioni richiedono soppressione:
// - dopo il wake da sleep (prima non partiva alcun annuncio)
// - mai interrompere altri MP3 (WiFi = bassa priorità)
static bool     g_wifiPrevConnected = false;
static uint32_t g_wifiErrLastMs = 0;
static const uint32_t WIFI_ERR_COOLDOWN_MS = 60000;
static bool     g_wifiSuppressNextConnectAnnounce = false;
static uint32_t g_wifiAudioSuppressUntilMs = 0;

static bool loadWifiUserEnabledFromNVS() {
  prefs.begin(NVS_NS_SYS, true);
  bool v = prefs.getBool(NVS_KEY_WIFI_USER, true);
  prefs.end();
  return v;
}

static void saveWifiUserEnabledToNVS(bool v) {
  prefs.begin(NVS_NS_SYS, false);
  prefs.putBool(NVS_KEY_WIFI_USER, v);
  prefs.end();
  Serial.print(F("[NVS] WiFi preferenza utente = "));
  Serial.println(v ? F("ON") : F("OFF"));
}

static void wifiApplyUserSetting(bool enable) {
#if ENABLE_WIFI_OTA
  if (enable) {
    if (!g_wifiSetupDone) {
      Net::wifiSetup();
      g_wifiSetupDone = true;
    } else {
      Net::wifiResume();
    }
  } else {
    if (g_wifiSetupDone) {
      Net::wifiSuspend();
    } else {
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
    }
  }
#elif ENABLE_ARDUINO_CLOUD
  // Cloud richiede WiFi: se un domani abiliti ENABLE_ARDUINO_CLOUD, gestisci qui la preferenza utente.
  (void)enable;
#endif
}

static void wifiToggleUserSetting() {
  // Se l'utente tocca il tasto WiFi, consideriamo l'azione "volontaria":
  // sblocchiamo eventuali soppressioni post-wake.
  g_wifiAudioSuppressUntilMs = 0;
  g_wifiSuppressNextConnectAnnounce = false;
  g_wifiPrevConnected = WiFi.isConnected();

  g_wifiUserEnabled = !g_wifiUserEnabled;
  saveWifiUserEnabledToNVS(g_wifiUserEnabled);
  wifiApplyUserSetting(g_wifiUserEnabled);
}

// ========================= POWER SAVE (inattività) =========================
// Dopo N ms senza tasti, entra in light-sleep; wake su qualunque tasto.
// Requisiti: nessun reset di stato/pesata.
static const uint32_t INACTIVITY_SLEEP_MS = 5UL * 60UL * 1000UL; // 5 minuti

// LED esterno per indicare chiaramente lo sleep (display spento)
static const int  SLEEP_LED_PIN = 15;          // GPIO15
static const bool SLEEP_LED_ACTIVE_HIGH = true; // HIGH = acceso (GPIO -> R -> anodo, catodo a GND)

// DFPlayer Mini (UART)
// TX DFPlayer dedicato: GPIO4. LED sleep dedicato: GPIO15.
static const DFPlayer::Pins DFPLAYER_PINS = { .tx = 4, .rx = 34 }; // RX su GPIO34 (input-only), TX su GPIO4
static const uint8_t DFPLAYER_DEFAULT_VOL = 20; // 0..30

// Power-cut DFPlayer (high-side)
static const int  DFPLAYER_EN_PIN   = 2;   // GPIO2 -> NPN base (via 10k). HIGH = DFPlayer ON
static const int  DFPLAYER_BUSY_PIN = 39;  // GPIO39 <- DFPlayer BUSY (GPIO39 non ha pull interni; se BUSY è instabile aggiungi pull-up esterno a 3V3)

static uint8_t g_dfplayerVol = DFPLAYER_DEFAULT_VOL;

static uint32_t g_lastKeyPressMs = 0;
// ====================== HX711 serial logging (toggle) ======================
static bool     g_hxLogEnabled   = false;      // default OFF: il monitor seriale resta pulito
static uint32_t g_hxLogPeriodMs  = 250;        // rate log quando ON
static uint32_t g_hxLogLastMs    = 0;

static bool     g_inactivitySleepArmed = true;

// ========================= HX711 HEALTH (runtime) =========================
// Stato sensore HX711: OK / WARN / ERROR / ERROR HARD
static HxHealth g_hxHealth;
static const HxHealthConfig HX_HEALTH_CFG = {
  .warnAfterMs       = 500,
  .errorAfterMs      = 3000,
  .hardAfterMs       = 30000,
  .okStableMs        = 800,
  .lastValidMaxAgeMs = 30000,
};

// forward decl: lastOledMs viene definito più sotto (serve per forzare refresh post-wake)
extern unsigned long lastOledMs;

static inline void sleepLedSet(bool on) {
  // Assume pinMode(SLEEP_LED_PIN, OUTPUT) already set
  if (SLEEP_LED_ACTIVE_HIGH) {
    digitalWrite(SLEEP_LED_PIN, on ? HIGH : LOW);
  } else {
    digitalWrite(SLEEP_LED_PIN, on ? LOW : HIGH);
  }
}

// LED OFF
static inline void sleepLedOff() {
  pinMode(SLEEP_LED_PIN, OUTPUT);
  sleepLedSet(false);
}

static void sleepPrepareWakeFromKeypad() {
  // Prepara wake da tastiera:
  // - Righe a LOW fisso
  // - Colonne INPUT_PULLUP
  // Premendo un tasto, una colonna viene tirata LOW -> wake.
  const int* rows = nullptr; int nRows = 0;
  const int* cols = nullptr; int nCols = 0;
  keypad_get_pins(&rows, &nRows, &cols, &nCols);

  for (int i = 0; i < nRows; i++) {
    pinMode(rows[i], OUTPUT);
    digitalWrite(rows[i], LOW);
  }
  for (int i = 0; i < nCols; i++) {
    pinMode(cols[i], INPUT_PULLUP);
    gpio_wakeup_enable((gpio_num_t)cols[i], GPIO_INTR_LOW_LEVEL);
  }
}



// ========================= DFPLAYER AUDIO (power-gated, OFF solo in standby) =========================
// Obiettivo:
// - poter suonare un MP3 (es: /MP3/0001.mp3) su evento
// - spegnere davvero il DFPlayer (o metterlo in sleep) quando non serve
// - usare BUSY per capire quando il brano è finito, senza conoscere la durata
//
// NOTE HW IMPORTANTI:
// - GPIO39 NON ha pull-up/down interni.
//   In molti DFPlayer (soprattutto cloni) il pin BUSY può risultare "flottante" a riposo:
//   in quel caso aggiungi un pull-up esterno verso 3V3 (10k..47k). Evita il pulldown a GND.
// - GPIO4: TX DFPlayer. GPIO15: LED sleep (dedicato).

enum AudioState : uint8_t { AUDIO_IDLE = 0, AUDIO_GAP, AUDIO_POWERING, AUDIO_STARTING, AUDIO_PLAYING, AUDIO_TAILING };
static AudioState g_audioState = AUDIO_IDLE;
static uint16_t   g_audioTrack = 0;
static uint32_t   g_audioStateMs = 0;
static uint32_t   g_audioStartMs = 0;
static uint32_t   g_audioTimeoutMs = 0;
static uint32_t   g_audioLastStopMs = 0;
static uint16_t   g_audioGapTrack = 0;
static uint32_t   g_audioGapCapSec = 0;
static uint32_t   g_audioGapUntilMs = 0;
static int        g_busyIdle = -1;
static int        g_busyPlaying = -1;
static bool       g_busyPolarityKnown = false;
static uint32_t   g_busyIdleSinceMs = 0;

// Coda MP3 (FIFO): nessun MP3 interrompe l'altro.
// Nota: evitiamo di usare una struct nei parametri delle funzioni della coda,
// perché l'auto-prototyping dell'IDE Arduino può generare prototipi prima della
// definizione del tipo e rompere la compilazione.
static const uint8_t AUDIO_Q_MAX = 16;
static uint16_t g_audioQ_track[AUDIO_Q_MAX];
static uint32_t g_audioQ_capSec[AUDIO_Q_MAX];
static bool     g_audioQ_low[AUDIO_Q_MAX];
static uint8_t  g_audioQCount = 0;

static inline bool audio_isMustPlayTrack(uint16_t t) {
  return (t == 2 || t == 7 || t == 8 || t == 12 || t == 13 || t == 14 || t == 15 || t == 16);
}

// Stato alimentazione DFPlayer (power-gating)
static bool       g_dfpPowered   = false;
static bool       g_dfpUartReady = false;

static const uint32_t DFP_POWERUP_MS      = 250;   // tempo minimo dopo power-on
static const uint32_t DFP_BUSY_DETECT_MS  = 800;   // finestra per auto-detect polarità BUSY
static const uint32_t DFP_END_STABLE_MS   = 250;   // quanto deve restare "idle" per considerare finito
static const uint32_t DFP_TAIL_OFF_MS     = 150;   // evita click tagliando subito dopo stop

static const uint32_t DFP_RESTART_GAP_MS  = 200;   // gap minimo tra stop -> play (anti-troncamento)
static const uint32_t DFP_MIN_PLAY_MS     = 300;   // ignora BUSY=idle nei primissimi ms (glitch BUSY)
static const uint32_t DFP_PLAY_TIMEOUT_MS = 30000; // paracadute: se BUSY non funziona

static inline void dfpPowerSet(bool on) {
  digitalWrite(DFPLAYER_EN_PIN, on ? HIGH : LOW);
}

static int dfpBusyReadStable() {
  // Lettura NON bloccante: BUSY è un livello logico stabile.
  // (In passato campionavamo con delay per attenuare pin flottanti; ora preferiamo
  // non rallentare la loop mentre l'audio sta suonando.)
  return (digitalRead(DFPLAYER_BUSY_PIN) != 0) ? 1 : 0;
}

static void audio_stopNow(bool clearQueue = false);

static void audio_begin() {
  pinMode(DFPLAYER_EN_PIN, OUTPUT);
  dfpPowerSet(false);
  g_dfpPowered = false;
  g_dfpUartReady = false;

  pinMode(DFPLAYER_BUSY_PIN, INPUT);

  // A DFPlayer spento teniamo UART in Hi-Z: evita back-powering.
  pinMode(DFPLAYER_PINS.tx, INPUT);
  if (DFPLAYER_PINS.rx >= 0) {
    pinMode(DFPLAYER_PINS.rx, INPUT);
  }

  DFPlayer::end();
  // Cancella eventuale play deferito (gap)
  g_audioGapTrack = 0;
  g_audioGapCapSec = 0;
  g_audioGapUntilMs = 0;

  g_audioState = AUDIO_IDLE;
  g_busyIdle = -1;
  g_busyPlaying = -1;
  g_busyPolarityKnown = false;
  g_busyIdleSinceMs = 0;
}

// Porta DFPlayer in stato "pronto" (alimentato + UART attiva + volume impostato), SENZA riprodurre.
// Nota: manteniamo comunque il percorso "cold-start" dentro audio_requestPlayMp3(), utile se in futuro
// si vorrà tornare all'accensione solo-on-demand.
static void audio_primeReady() {
  if (g_dfpPowered && g_dfpUartReady) return;

  // Se era spento, accendilo e lascia stabilizzare l'alimentazione.
  if (!g_dfpPowered) {
    dfpPowerSet(true);
    g_dfpPowered = true;

    // Mantieni TX basso durante il boot del DFPlayer
    pinMode(DFPLAYER_PINS.tx, OUTPUT);
    digitalWrite(DFPLAYER_PINS.tx, LOW);

    delay(DFP_POWERUP_MS);
  } else {
    // Già alimentato ma UART non pronta: attesa minima
    delay(20);
  }

  DFPlayer::restoreAfterEspWake(DFPLAYER_PINS, g_dfplayerVol);
  g_dfpUartReady = true;
  delay(20);

  // Baseline BUSY in idle (utile per il detect di START/END quando parte il primo play)
  g_busyIdle = dfpBusyReadStable();
}

static inline bool audio_isActive() {
  return (g_audioState != AUDIO_IDLE) || (g_audioQCount > 0);
}

static inline void audio_queueClear() {
  g_audioQCount = 0;
}

static bool audio_queuePush(uint16_t track, uint32_t capSeconds, bool low) {
  if (track < 1) track = 1;

  if (g_audioQCount >= AUDIO_Q_MAX) {
    // Se è un annuncio low, lo scartiamo.
    if (low) return false;

    // Se è un brano "must-play", proviamo a fare spazio togliendo un low in coda.
    if (audio_isMustPlayTrack(track)) {
      for (int i = (int)g_audioQCount - 1; i >= 0; i--) {
        if (g_audioQ_low[i]) {
          for (int j = i; j < (int)g_audioQCount - 1; j++) {
            g_audioQ_track[j] = g_audioQ_track[j + 1];
            g_audioQ_capSec[j] = g_audioQ_capSec[j + 1];
            g_audioQ_low[j]   = g_audioQ_low[j + 1];
          }
          g_audioQCount--;
          break;
        }
      }
      if (g_audioQCount >= AUDIO_Q_MAX) {
        // Ancora pieno: scarta l'ultimo (più recente) per preservare l'ordine già in coda.
        g_audioQCount--;
      }
    } else {
      // Non must-play e coda piena: scarta la richiesta.
      return false;
    }
  }

  g_audioQ_track[g_audioQCount] = track;
  g_audioQ_capSec[g_audioQCount] = capSeconds;
  g_audioQ_low[g_audioQCount] = low;
  g_audioQCount++;
  return true;
}

static bool audio_queuePop(uint16_t* outTrack, uint32_t* outCapSeconds, bool* outLow) {
  if (g_audioQCount == 0) return false;
  if (outTrack) *outTrack = g_audioQ_track[0];
  if (outCapSeconds) *outCapSeconds = g_audioQ_capSec[0];
  if (outLow) *outLow = g_audioQ_low[0];
  for (uint8_t i = 0; i + 1 < g_audioQCount; i++) {
    g_audioQ_track[i] = g_audioQ_track[i + 1];
    g_audioQ_capSec[i] = g_audioQ_capSec[i + 1];
    g_audioQ_low[i] = g_audioQ_low[i + 1];
  }
  g_audioQCount--;
  return true;
}

static void audio_startNow(uint16_t track, uint32_t capSeconds) {
  if (track < 1) track = 1;

  g_audioTrack = track;
  g_audioTimeoutMs = (capSeconds > 0) ? (capSeconds * 1000UL) : DFP_PLAY_TIMEOUT_MS;
  if (g_audioTimeoutMs < 3000) g_audioTimeoutMs = 3000;

  g_busyPolarityKnown = false;
  g_busyIdleSinceMs = 0;

  Serial.print(F("[MP3] play /MP3/"));
  if (track < 10) Serial.print('0');
  if (track < 100) Serial.print('0');
  if (track < 1000) Serial.print('0');
  Serial.print(track);
  Serial.println(F(".mp3"));

  const uint32_t now = millis();

  // Anti-troncamento: evita stop->play troppo ravvicinati (DFPlayer può tagliare il secondo MP3)
  if (g_audioLastStopMs != 0 && (now - g_audioLastStopMs) < DFP_RESTART_GAP_MS) {
    g_audioGapTrack = track;
    g_audioGapCapSec = capSeconds;
    g_audioGapUntilMs = g_audioLastStopMs + DFP_RESTART_GAP_MS;
    g_audioState = AUDIO_GAP;
    g_audioStateMs = now;
    return;
  }

  // Se il DFPlayer è già alimentato e UART pronta, non fare power-cycle: play diretto.
  if (g_dfpPowered && g_dfpUartReady) {
    g_busyIdle = dfpBusyReadStable();
    DFPlayer::playMp3(g_audioTrack);

    g_audioStartMs = now;
    g_audioState = AUDIO_STARTING;
    g_audioStateMs = now;
    return;
  }

  // Percorso cold-start: accendi DFPlayer e aspetta boot.
  g_audioState = AUDIO_POWERING;
  g_audioStateMs = now;
  g_audioStartMs = 0;

  dfpPowerSet(true);
  g_dfpPowered = true;

  // Mantieni TX basso durante il boot del DFPlayer
  pinMode(DFPLAYER_PINS.tx, OUTPUT);
  digitalWrite(DFPLAYER_PINS.tx, LOW);
}

static void audio_kickIfIdle() {
  if (g_audioState != AUDIO_IDLE) return;
  uint16_t t = 0;
  uint32_t cap = 0;
  bool low = false;
  if (audio_queuePop(&t, &cap, &low)) {
    (void)low; // attualmente non serve: è già stata gestita in push
    audio_startNow(t, cap);
  }
}

static void audio_requestPlayMp3(uint16_t track, uint32_t capSeconds = 0) {
  if (track < 1) track = 1;

  // Guard anti-retrigger: evita di rilanciare lo stesso MP3 in raffica (rimbalzi tasto, eventi duplicati).
  // Side effect: lo stesso track richiesto più volte entro la finestra viene ignorato.
  static uint16_t s_lastReqTrack = 0;
  static uint32_t s_lastReqMs = 0;
  const uint32_t nowReq = millis();
  const uint32_t REQ_GUARD_MS = 250;
  if (track == s_lastReqTrack && (uint32_t)(nowReq - s_lastReqMs) < REQ_GUARD_MS) {
    return;
  }
  s_lastReqTrack = track;
  s_lastReqMs = nowReq;

  const bool mustPlay = audio_isMustPlayTrack(track);
  const bool playing = (g_audioState != AUDIO_IDLE);
  const bool curMustPlay = playing && audio_isMustPlayTrack(g_audioTrack);

  if (mustPlay) {
    // Must-play: NON interrompibile. Può però interrompere un audio interrompibile in corso.
    if (playing && !curMustPlay) {
      audio_stopNow(false);
    }

    // Se possiamo partire subito, fallo.
    if (g_audioState == AUDIO_IDLE && g_audioQCount == 0) {
      audio_startNow(track, capSeconds);
      return;
    }

    // Altrimenti, metti in coda FIFO (solo per must-play) e parti appena possibile.
    (void)audio_queuePush(track, capSeconds, false);
    audio_kickIfIdle();
    return;
  }

  // Interrompibile:
  // - se è in corso un must-play, NON mettiamo in coda (verrebbe riprodotto in ritardo)
  if (curMustPlay) {
    return;
  }

  // - se ci sono must-play in coda, non disturbare: facciamo passare prima quelli
  if (g_audioQCount > 0) {
    audio_kickIfIdle();
    return;
  }

  // - se stiamo suonando un audio interrompibile, interrompilo e riparti subito
  if (playing) {
    audio_stopNow(false);
  }
  audio_startNow(track, capSeconds);
}


// Richiesta MP3 a bassa priorità: non deve disturbare la UX.
// Strategia: se c'è già qualcosa in riproduzione o in coda, scarta.
static void audio_requestPlayMp3_low(uint16_t track, uint32_t capSeconds = 0) {
  if (g_audioState != AUDIO_IDLE) return;
  if (g_audioQCount > 0) return;
  (void)audio_queuePush(track, capSeconds, true);
  audio_kickIfIdle();
}



static void audio_setVolume(uint8_t vol) {
  if (vol > 30) vol = 30;
  g_dfplayerVol = vol;

  // Se la UART DFPlayer è pronta, applica subito il volume anche se non sta suonando.
  if (g_dfpUartReady) {
    DFPlayer::setVolume(vol);
  }
}

static void audio_stopNow(bool clearQueue) {
  // Stop riproduzione, ma NON tagliare l'alimentazione.
  // L'idea è: DFPlayer resta acceso durante l'uso; lo spegniamo solo entrando in standby/light-sleep.
  const bool wasTailing = (g_audioState == AUDIO_TAILING);

  // Se stiamo chiudendo un brano già stoppato (TAILING), evitare stop ridondanti e soprattutto
  // non aggiornare lastStopMs (altrimenti aggiungiamo un gap extra tra i brani in coda).
  if (g_dfpUartReady && !wasTailing) {
    DFPlayer::stop();
    g_audioLastStopMs = millis();
  }

  if (clearQueue) {
    audio_queueClear();
  }

  // Cancella eventuale play deferito (gap)
  g_audioGapTrack = 0;
  g_audioGapCapSec = 0;
  g_audioGapUntilMs = 0;

  g_audioState = AUDIO_IDLE;
  g_busyIdle = -1;
  g_busyPlaying = -1;
  g_busyPolarityKnown = false;
  g_busyIdleSinceMs = 0;
}


static void audio_powerOffNow() {
  // Spegnendo l'audio, svuotiamo la coda: al wake riparte pulito.
  audio_queueClear();

  // Spegne davvero DFPlayer (MOSFET high-side) + mette UART in Hi-Z per evitare back-powering.
  if (g_dfpUartReady) {
    DFPlayer::stop();
    delay(50);
    DFPlayer::end();
    g_dfpUartReady = false;
  }

  pinMode(DFPLAYER_PINS.tx, INPUT);
  if (DFPLAYER_PINS.rx >= 0) pinMode(DFPLAYER_PINS.rx, INPUT);

  dfpPowerSet(false);
  g_dfpPowered = false;

  g_audioState = AUDIO_IDLE;
  g_busyIdle = -1;
  g_busyPlaying = -1;
  g_busyPolarityKnown = false;
  g_busyIdleSinceMs = 0;
}

static void audio_task(uint32_t now) {
  switch (g_audioState) {
    case AUDIO_IDLE:
      // Se c'è qualcosa in coda, parti subito (senza bloccare la loop).
      audio_kickIfIdle();
      return;

    
    case AUDIO_GAP:
      if (now < g_audioGapUntilMs) return;
      // Scaduto il gap: avvia il track deferito
      g_audioState = AUDIO_IDLE;
      audio_startNow(g_audioGapTrack, g_audioGapCapSec);
      return;

case AUDIO_POWERING:
      if ((now - g_audioStateMs) < DFP_POWERUP_MS) return;

      // Attiva UART solo quando serve suonare
      DFPlayer::restoreAfterEspWake(DFPLAYER_PINS, g_dfplayerVol);
      g_dfpUartReady = true;
      delay(20);

      // Idle BUSY prima del play
      g_busyIdle = dfpBusyReadStable();

      // Avvia play da cartella /MP3/0001.mp3 ...
      DFPlayer::playMp3(g_audioTrack);
      g_audioStartMs = now;

      g_audioState = AUDIO_STARTING;
      g_audioStateMs = now;
      return;

    case AUDIO_STARTING: {
      int b = dfpBusyReadStable();

      if (b != g_busyIdle) {
        g_busyPlaying = b;
        g_busyPolarityKnown = true;
        g_audioState = AUDIO_PLAYING;
        g_audioStateMs = now;
        return;
      }

      if ((now - g_audioStateMs) >= DFP_BUSY_DETECT_MS) {
        // Non abbiamo visto transizione: assumiamo "playing = !idle" (molto spesso BUSY è LOW mentre suona)
        g_busyPlaying = (g_busyIdle == 0) ? 1 : 0;
        g_busyPolarityKnown = false;
        g_audioState = AUDIO_PLAYING;
        g_audioStateMs = now;
        return;
      }
      return;
    }

    case AUDIO_PLAYING: {
      // Paracadute se BUSY non è affidabile
      if (g_audioStartMs != 0 && (now - g_audioStartMs) >= g_audioTimeoutMs) {
        Serial.println(F("[MP3] timeout (BUSY non affidabile?) -> stop"));
        DFPlayer::stop();
        g_audioLastStopMs = now;
        g_audioState = AUDIO_TAILING;
        g_audioStateMs = now;
        return;
      }

      int b = dfpBusyReadStable();

      // Se BUSY è tornato al livello "idle" in modo stabile, consideriamo finito.
      if (b == g_busyIdle) {
        // Evita glitch BUSY: nei primissimi ms dopo il play, ignoriamo un idle "falso".
        if (g_audioStartMs != 0 && (now - g_audioStartMs) < DFP_MIN_PLAY_MS) {
          g_busyIdleSinceMs = 0;
          return;
        }
        if (g_busyIdleSinceMs == 0) g_busyIdleSinceMs = now;
        if ((now - g_busyIdleSinceMs) >= DFP_END_STABLE_MS) {
          Serial.println(F("[MP3] fine"));
          DFPlayer::stop();
          g_audioLastStopMs = now;
          g_audioState = AUDIO_TAILING;
          g_audioStateMs = now;
        }
      } else {
        g_busyIdleSinceMs = 0;
      }
      return;
    }

    case AUDIO_TAILING:
      if ((now - g_audioStateMs) < DFP_TAIL_OFF_MS) return;
      audio_stopNow();
      return;
  }
}

static void audio_debugStatus() {
  Serial.print(F("[MP3] state=")); Serial.print((int)g_audioState);
  Serial.print(F(" track=")); Serial.print(g_audioTrack);
  Serial.print(F(" vol=")); Serial.print(g_dfplayerVol);
  Serial.print(F(" BUSY=")); Serial.print(digitalRead(DFPLAYER_BUSY_PIN));
  Serial.print(F(" idle=")); Serial.print(g_busyIdle);
  Serial.print(F(" playLvl=")); Serial.print(g_busyPlaying);
  Serial.print(F(" polKnown=")); Serial.println(g_busyPolarityKnown ? "yes" : "no");
}

static void enterInactivityLightSleep() {
  Serial.println(F("[SLEEP] Inattività: entro in light-sleep (wake da tastiera)"));

  // Spegni DFPlayer solo quando entriamo in standby/light-sleep
  audio_powerOffNow();

#if ENABLE_WIFI_OTA
  // In sleep spegniamo il WiFi, ma NON tocchiamo la preferenza utente.
  if (g_wifiUserEnabled) {
    Net::wifiSuspend();
  }
#elif ENABLE_ARDUINO_CLOUD
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
#endif

  // LED sleep ON + display OFF
  pinMode(SLEEP_LED_PIN, OUTPUT);
  sleepLedSet(true);
  ui_powerSave(true);

  // Wake da tastiera
  sleepPrepareWakeFromKeypad();

  // Abilita wake GPIO (light-sleep) e vai in sleep (niente timer)
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
  esp_sleep_enable_gpio_wakeup();
  delay(20);
  esp_light_sleep_start();

  // --- Wake ---
  Serial.println(F("[SLEEP] Wake da tastiera"));

  sleepLedOff();
  ui_powerSave(false);

  // Ripristina configurazione tastiera (righe HIGH inattive, input pull-up)
  keypad_init();

#if ENABLE_WIFI_OTA
  // Ripristina WiFi solo se l'utente lo aveva lasciato ON.
  if (g_wifiUserEnabled) {
    Net::wifiResume();
  }
#endif

  // WiFi audio: al wake NON vogliamo annunciare la riconnessione (prima non partiva)
  // e soprattutto non deve troncare 0004.mp3.
  g_wifiPrevConnected = WiFi.isConnected();
  g_wifiSuppressNextConnectAnnounce = g_wifiUserEnabled;
  g_wifiAudioSuppressUntilMs = millis() + 8000; // finestra prudente per evitare sovrapposizioni

  // Evita rientro immediato in sleep
  g_lastKeyPressMs = millis();
  lastOledMs = 0; // forza refresh veloce dopo wake

  // DFPlayer pronto subito al wake (così un evento audio può partire senza cold-start)
  audio_primeReady();
  // 0004.mp3 = Uscita risparmio energetico
  audio_requestPlayMp3(4);
}


// Stato boot (true finché non finisce setup)
static bool g_isBooting = true;

// Inattività: dopo 5 min senza tasti mostriamo Zzz... per 5s e durante suona 0003,
// poi entriamo in light-sleep (wake da tastiera). Qualsiasi tasto annulla la sequenza.
enum InactivitySleepStage : uint8_t { INACT_NONE = 0, INACT_ZZZ = 1 };
static InactivitySleepStage g_inactivitySleepStage = INACT_NONE;
static uint32_t g_inactivityStageStartMs = 0;
static const uint32_t INACTIVITY_ZZZ_MS = 5000; // schermata Zzz... (5s)

// (WiFi audio state spostato vicino alla sezione WiFi preferenza utente)
 
// Stato boot (per gestire errori "hard" vs "ack")
static bool g_hxBeginOk = false;

// ========================= HX711 (driver separato) =========================
// Pins richiesti dall'utente: SCK su RX2 (GPIO16), DOUT su GPIO35
static const int HX711_SCK_PIN  = 16;
static const int HX711_DOUT_PIN = 35;

// Gain: A=128 (1 impulso extra dopo i 24 bit)
static const HX711Gain HX711_GAIN = HX711_GAIN_128_A;

// Timing: micro-delay per stabilizzare lettura (valori piccoli, ma non a 0)
static const uint8_t HX711_PULSE_US = 1;

// Se DOUT non va mai LOW, consideriamo "sensore non trovato"
static const uint32_t HX711_INIT_TIMEOUT_MS = 1200;

static long readRawHXOnceBlocking(uint32_t timeoutMs){
  return hx711_read_blocking(timeoutMs);
}

static long readRawHXAvg(int n){
  if (n <= 1) return readRawHXOnceBlocking(200);
  long s = 0;
  for(int i=0;i<n;i++){
    long r = readRawHXOnceBlocking(200);
    if (r == 0) return 0;
    s += r;
  }
  return s / n;
}

// ========================= CONFIG =========================

// [0] Calibrazione seed (valori indicativi, sovrascritti da NVS + CAL)
const long  DEFAULT_REF_RAW  = 265290;   // raw medio con 2000 g
const long  DEFAULT_ZERO_RAW = 51471;    // raw medio a vuoto
const long  DEFAULT_REF_G    = 2000;     // grammi di riferimento
const float DEFAULT_CPG      = float(DEFAULT_REF_RAW - DEFAULT_ZERO_RAW) / float(DEFAULT_REF_G);
// seed: 500 counts/grammo (HX711)
// NOTE: se arrivi da firmware NAU, la scala salvata in NVS NON è valida: fai CAL.

// [1] Auto-TARE all’avvio (semplice, niente logiche complicate)
const bool     AUTO_TARE_ON_BOOT   = true;
const uint16_t AUTO_TARE_SAMPLES   = 64;
const uint16_t AUTO_TARE_SETTLE_MS = 400;  // discard iniziale (assestamento)
const uint16_t AUTO_TARE_MIN_SAMPLES = 32;    // minimo campioni (≈0.4 s @ 80 SPS)
const float    AUTO_TARE_RANGE_G    = 0.40f;  // finestra di quiete (range) per fermarsi prima
const uint32_t AUTO_TARE_QUIET_MS    = 400;    // deve restare in range per questo tempo

// [2] Limite operativo visuale (±16 kg)
const float MAX_DISPLAY_G = 16000.0f;
const bool  FLAG_OVERLOAD = true; // logga over=1 se clamp

// [3] Filtri display
int   MA_DEFAULT       = 6;      // normal / work
int   MA_FINE          = 4;      // fine / live
float DB_UNSTABLE_N    = 0.10f;  // deadband in UNSTABLE (normal)
float DB_UNSTABLE_F    = 0.05f;  // deadband in UNSTABLE (fine)
float       deadbandUnstable = DB_UNSTABLE_N;

// [4] Stati STABLE/UNSTABLE (semplificati)
const bool  ST_ENABLE_DEFAULT   = true;
float       ST_LEAVE_DELTA_G    = 0.80f;  // se |gLive - gLatch| ≥ 0.80 g → UNSTABLE
float       ST_ENTER_RANGE_G    = 0.60f;  // range < 0.60 g → STABLE
uint32_t    ST_TO_STABLE_MS     = 1200;   // finestra per valutare il rientro (1.2 s)

// [5] Zero-Tracking (solo near-zero + snap allo scarico)
const bool  ZT_ENABLE_DEFAULT   = true;

// 5a) Micro-correzione lenta vicino a 0
float       ZT_WINDOW_G         = 1.5f;     // attivo se |g| ≤ 1.5 g
uint32_t    ZT_QUIET_MS         = 1200;     // serve “quiete” per almeno 1.2 s
float       ZT_QUIET_RANGE_G    = 0.4f;     // range ≤ 0.4 g
float       ZT_QUIET_SLOPE_GPS  = 0.3f;     // |slope| ≤ 0.3 g/s
uint32_t    ZT_PERIOD_MS        = 1000;     // passo ogni 1.0 s
float       ZT_STEP_G           = 0.02f;    // ~0.02 g per passo
float       ZT_MAX_G            = 3.0f;     // cap totale ±3 g

// 5b) Snap-to-zero su scarico
const bool  UNLOAD_SNAP_ENABLE   = true;
float       UNLOAD_CROSS_WIN_G   = 3.0f;    // esegui snap se |g| ≤ 3 g (meno aggressivo)
float       UNLOAD_SLOPE_GPS_NEG = -5.0f;   // pendenza ≤ -5 g/s (meno falsi positivi)
uint32_t    UNLOAD_SLOPE_WIN_MS  = 400;     // finestra per slope scarico
float       UNLOAD_STABLE_RANGE_G= 0.8f;    // range “quiete” per snap
uint32_t    UNLOAD_STABLE_MS     = 300;     // ms di quiete
uint32_t    UNLOAD_COOLDOWN_MS   = 2000;    // minimo tra due snap
float       UNLOAD_SNAP_MAX_G    = 2.0f;    // correzione max snap (g)

// [6] Campionamento logica (non il sample rate del sensore)
const uint32_t SAMPLE_MS = 60; // ~16 Hz logica (HX711 10 Hz o 80 Hz, hardware)

// [OLED] Cadenzamento refresh
const uint32_t OLED_UPDATE_MS = 60;


// ========================= PIN & OGGETTI =========================

// HX711: I2C spostato su D27 (SDA) e D26 (SCL) per averli vicini
const int I2C_SDA = 32;  // etichetta scheda: D27 (GPIO27)
const int I2C_SCL = 33;  // etichetta scheda: D26 (GPIO26)



// ========================= CALIBRAZIONE RUNTIME =========================
// g = (rawAvg - (OFFSET_RAW + zero_track_counts)) / SCALE_CPG
long  OFFSET_RAW = 0;
float SCALE_CPG  = DEFAULT_CPG;
long  REF_G      = DEFAULT_REF_G;

// Zero-Tracking accumulatore (solo software)
long  zero_track_counts = 0;
bool  ztEnable = ZT_ENABLE_DEFAULT;
// ====================== TARE UI clamp + robust tare (80 SPS) ======================
// Dopo la pressione TARA:
//  - oscura il peso e mostra "- TARA -" + barra di stabilizzazione per 1.5s
//  - ignora un breve "settling" iniziale (pressione tasto / vibrazioni)
//  - raccoglie campioni raw fino a fine barra e applica l'OFFSET_RAW SOLO alla fine
static bool     gTareUiActive = false;
static bool     gTareOffsetApplied = false;
static uint32_t gTareUiStartMs = 0;
static uint32_t gTareUiEndMs   = 0;
static uint32_t gTareCollectStartMs = 0;
static uint32_t gTareCollectEndMs   = 0;
static int64_t  gTareSum = 0;
static uint32_t gTareCnt = 0;

static const uint32_t TARE_UI_CLAMP_MS  = 1500;
static const uint32_t TARE_SETTLE_MS   = 250;  // ignora vibrazioni iniziali
static const uint32_t TARE_MIN_SAMPLES = 40;   // ~0.5s @ 80 SPS (dopo settling)

static void tareStart(uint32_t nowMs){
  gTareUiActive = true;
  gTareOffsetApplied = false;
  gTareUiStartMs = nowMs;
  gTareUiEndMs   = nowMs + TARE_UI_CLAMP_MS;

  gTareCollectStartMs = nowMs + TARE_SETTLE_MS;
  gTareCollectEndMs   = gTareUiEndMs;   // raccogli fino a fine barra

  gTareSum = 0;
  gTareCnt = 0;
}

static void tareAccumSample(long rawSample){
  if (!gTareUiActive) return;
  if (gTareOffsetApplied) return;

  uint32_t nowMs = millis();
  if (nowMs < gTareCollectStartMs) return;
  if (nowMs > gTareCollectEndMs) return;

  gTareSum += (int64_t)rawSample;
  gTareCnt++;
}

static void tareMaybeApply(uint32_t nowMs){
  if (!gTareUiActive) return;
  if (gTareOffsetApplied) return;
  if (nowMs < gTareUiEndMs) return; // applica offset solo a fine barra

  // Non-bloccante:
  // Se per qualsiasi motivo non sono arrivati abbastanza campioni (HX non pronto / glitch),
  // evitiamo un fallback blocking che congelerebbe UI/audio/wifi.
  if (gTareCnt == 0) {
    Serial.println(F("[TARE] FAIL: nessun campione raccolto"));
    gTareOffsetApplied = true;
    return;
  }

  long rawZero = (long)(gTareSum / (int64_t)gTareCnt);

  if (gTareCnt < TARE_MIN_SAMPLES) {
    Serial.print(F("[TARE] WARN: campioni insufficienti ("));
    Serial.print(gTareCnt);
    Serial.println(F("). Offset applicato comunque (non-bloccante)."));
  }

  OFFSET_RAW = rawZero;
  zero_track_counts = 0;
  resetFiltersAndState();
  Serial.print(F("[TARE] OFFSET_RAW="));
  Serial.println(OFFSET_RAW);

  gTareOffsetApplied = true;
}




// ========================= FILTRI =========================
long medBuf[3] = {0,0,0};
int  medIdx=0, medCount=0;

long median3(long a,long b,long c){
  long M = max(a, max(b,c));
  long m = min(a, min(b,c));
  return a + b + c - M - m;
}

long pushMedian3(long x){
  medBuf[medIdx] = x;
  medIdx = (medIdx + 1) % 3;
  if (medCount < 3) medCount++;
  if (medCount < 3){
    long s=0;
    for(int i=0;i<medCount;i++) s += medBuf[i];
    return s / (medCount ? medCount : 1);
  }
  return median3(medBuf[0], medBuf[1], medBuf[2]);
}

void resetMedian(){
  medIdx=0; medCount=0;
  medBuf[0]=medBuf[1]=medBuf[2]=0;
}

long maBuf[8];
int  maIdx=0, maCount=0, maN=MA_DEFAULT;
long maSum=0;

void setMA(int n){
  maN = constrain(n,1,8);
  maIdx=0; maCount=0; maSum=0;
  for(int i=0;i<8;i++) maBuf[i]=0;
}

long pushMA(long x){
  if (maCount == maN) {
    maSum -= maBuf[maIdx];
  } else {
    maCount++;
  }
  maBuf[maIdx] = x;
  maSum += x;
  maIdx++;
  if (maIdx >= maN) maIdx = 0;
  return maSum / (maCount ? maCount : 1);
}

// ========================= STORICO (range/slope) =========================
const int MAX_HIST = 64;
float gHist[MAX_HIST];
int   hIdx=0, hCount=0;

void pushHist(float g){
  gHist[hIdx] = g;
  hIdx = (hIdx + 1) % MAX_HIST;
  if (hCount < MAX_HIST) hCount++;
}

float histAt(int k){
  if (hCount == 0) return 0.0f;
  k = constrain(k, 0, min(hCount, MAX_HIST) - 1);
  int idx = (hIdx - 1 - k + MAX_HIST) % MAX_HIST;
  return gHist[idx];
}

float rangeLastNSamples(int n){
  n = min(n, hCount);
  if (n <= 0) return 0.0f;
  float mn = histAt(0), mx = mn;
  for(int i=1;i<n;i++){
    float v = histAt(i);
    if (v < mn) mn = v;
    if (v > mx) mx = v;
  }
  return mx - mn;
}

float slopeLastNSamples(int n){
  n = min(n, hCount);
  if (n < 2) return 0.0f;
  float last  = histAt(0);
  float first = histAt(n-1);
  float dt    = (float)(n-1) * (SAMPLE_MS / 1000.0f);
  if (dt <= 0.0f) return 0.0f;
  return (last - first) / dt; // g/s
}

// ========================= STATI & DISPLAY =========================
enum State { STABLE, UNSTABLE };
bool  stEnable = ST_ENABLE_DEFAULT;
State state    = UNSTABLE;

enum ScaleMode { SCALE_MODE_WORK = 0, SCALE_MODE_LIVE };
ScaleMode currentMode = SCALE_MODE_WORK;


float gLive = 0.0f;
float gLatch = 0.0f;
float dispUnstable = 0.0f;
bool  dispInit = false;

unsigned long lastSampleMs  = 0;
unsigned long lastZTtickMs  = 0;
unsigned long lastSnapMs    = 0;
unsigned long lastOledMs    = 0;


// --- Decimazione HX711 quando lavori a 80 SPS ---
// Obiettivo: ridurre rumore senza perdere reattività.
// WORK: usato per logiche STABLE/UNSTABLE, ZT, ecc.  (N più alto)
// LIVE: usato SOLO per visualizzazione in modalità LIVE (stEnable=false) (N più basso)
static const uint8_t DECIM_WORK_N = 5;       // ~16 Hz se HX=80 SPS
static const uint8_t DECIM_LIVE_N = 2;       // ~40 Hz se HX=80 SPS (solo display in LIVE)
// Display (solo visualizzazione) con isteresi:
// - Zero band con soglia di ingresso (enter) e soglia di uscita (exit):
//   evita che vicino allo zero si accendano/spegnano continuamente ±1 g.
// - Isteresi sui cambi di intero: riduce il flicker tra N e N+1 mentre pesi.
//
// WORK: 0 finché |g| < 1.3 g (simmetrico + / -). Uscita dallo zero più bassa per ridurre flicker.
static const float DISP_ZERO_ENTER_WORK_G = 1.30f;
static const float DISP_ZERO_EXIT_WORK_G  = 0.90f;
static const float DISP_STEP_HYS_WORK_G   = 0.20f;

// LIVE: 0 finché |g| < 1.0 g, ma scatta subito a 1 quando tocchi 1.0.
// Per ridurre flicker, rientro a 0 solo sotto ~0.7 g.
static const float DISP_ZERO_ENTER_LIVE_G = 1.00f;
static const float DISP_ZERO_EXIT_LIVE_G  = 0.70f;
static const float DISP_STEP_HYS_LIVE_G   = 0.15f;

// Quantizzazione con isteresi (stateful: usa prev).
// Side effects: nessuno.
static inline long displayQuantizeHys(float g, float enterZeroAbs, float exitZeroAbs, float stepHysAbs, long prev) {
  // 1) Banda zero con isteresi
  if (prev == 0) {
    if (g >=  enterZeroAbs) return  1;
    if (g <= -enterZeroAbs) return -1;
    return 0;
  }

  // Se cambi segno ma sei ancora nella zona “vicino allo zero”, forza passaggio per 0.
  const int sPrev = (prev > 0) - (prev < 0);
  const int sG    = (g > 0.0f) - (g < 0.0f);
  if (sPrev != 0 && sG != 0 && sPrev != sG && fabsf(g) < enterZeroAbs) {
    return 0;
  }

  if (fabsf(g) <= exitZeroAbs) return 0;

  // 2) Isteresi su cambio di intero (anti-flicker)
  long d = prev;
  long target = lroundf(g);
  if (target > d) {
    const float th = (float)d + 0.5f + stepHysAbs;
    if (g >= th) d = target;
  } else if (target < d) {
    const float th = (float)d - 0.5f - stepHysAbs;
    if (g <= th) d = target;
  }
  return d;
}

static inline long displayWorkQuant(float g, long prev) {
  return displayQuantizeHys(g, DISP_ZERO_ENTER_WORK_G, DISP_ZERO_EXIT_WORK_G, DISP_STEP_HYS_WORK_G, prev);
}

static inline long displayLiveQuant(float g, long prev) {
  return displayQuantizeHys(g, DISP_ZERO_ENTER_LIVE_G, DISP_ZERO_EXIT_LIVE_G, DISP_STEP_HYS_LIVE_G, prev);
}

// Ultimi valori "pubblici" per il display
static long gDispWorkLast = 0;
static const char* gStateLabelWorkLast = "----";
static long gDispLiveLast = 0;

// Accumulatori per decimazione
static long gLiveSum = 0;
static uint8_t gLiveCnt = 0;
static long gWorkSum = 0;
static uint8_t gWorkCnt = 0;



static const long INACTIVITY_WEIGHT_DELTA_G = 5; // ±5 g considerati "fermo" per lo standby (reset solo se >5 g)
static long g_inactDispRef = 0;
static bool g_inactDispRefInit = false;

// Considera l'utente "attivo" quando il peso visualizzato si muove di più di ±INACTIVITY_WEIGHT_DELTA_G.
// Input: now = millis()
// Side effects: aggiorna g_lastKeyPressMs e, se stiamo entrando in standby (INACT_ZZZ), annulla la sequenza.
static void inactivity_noteWeightActivity(uint32_t now) {
  long cur = stEnable ? gDispWorkLast : gDispLiveLast;
  if (!g_inactDispRefInit) {
    g_inactDispRefInit = true;
    g_inactDispRef = cur;
    return;
  }
  long d = cur - g_inactDispRef;
  if (d < 0) d = -d;
  if (d > INACTIVITY_WEIGHT_DELTA_G) {
    g_lastKeyPressMs = now;
    g_inactDispRef = cur;
    if (g_inactivitySleepStage != INACT_NONE) {
      g_inactivitySleepStage = INACT_NONE;
      g_inactivityStageStartMs = 0;
      audio_stopNow(true);
      lastOledMs = 0; // ripristina UI normale subito
    }
  }
}


// ========================= UTILITY =========================
inline long effectiveOffsetCounts(){
  return OFFSET_RAW + zero_track_counts;
}

void resetFiltersAndState(){
  resetMedian();
  setMA(maN);
  hIdx = 0;
  hCount = 0;
  state = UNSTABLE;
  dispInit = false;

  // Reset decimazione / debounce (evita residui pre-tara)
  gLiveSum = 0;
  gLiveCnt = 0;
  gWorkSum = 0;
  gWorkCnt = 0;

  gDispWorkLast = 0;
  gDispLiveLast = 0;
  gStateLabelWorkLast = "----";
}

// ========================= NVS / HELP =========================
void printHelp(){
  Serial.println(F("\nComandi:"));
  Serial.println(F("  t              -> TARE"));
  Serial.println(F("  c <g>          -> CAL con peso noto (es: c 2000)"));
  Serial.println(F("  p              -> stampa parametri/stato"));
  Serial.println(F("  s              -> salva OFFSET/SCALE/REF in NVS"));
  Serial.println(F("  m work         -> preset lavoro (normal + stati)"));
  Serial.println(F("  m normal       -> MA=6, deadband 0.10 g, ST on"));
  Serial.println(F("  m fine         -> micro / live: MA=4, deadband 0.05 g, ST off"));
  Serial.println(F("  m live         -> alias di m fine"));
  Serial.println(F("  st on/off/?    -> abilita/disabilita stati STABLE/UNSTABLE"));
  Serial.println(F("  zt on/off/reset/? -> zero-tracking (near-zero + snap)"));
  Serial.println(F("  mp3 <n> [capSec] -> suona /MP3/000n.mp3 (capSec opzionale, paracadute)"));
  Serial.println(F("  mp3 ? / mp3 status -> stato DFPlayer/BUSY"));
  Serial.println(F("  stop             -> stop audio (DFPlayer resta ON finché operativo)"));
  Serial.println(F("  vol <0..30>       -> volume DFPlayer"));

  Serial.println(F("  wifi ?                     -> help WiFi credenziali"));
  Serial.println(F("  wifi status                -> stato WiFi + reti configurate"));
  Serial.println(F("  wifi creds                 -> lista SSID (password mascherata)"));
  Serial.println(F("  wifi creds showpass <1..2> -> stampa password in chiaro (recupero)"));
  Serial.println(F("  wifi set <1..2> \"SSID\" \"PASS\" -> imposta rete (max 2), salva in NVS"));
  Serial.println(F("  wifi clear <1..2|all>      -> cancella credenziali da NVS"));
  Serial.println(F("  wifi apply                 -> ricarica credenziali e riavvia tentativi"));

  Serial.println(F("  ota ? / ota help            -> help password OTA"));
  Serial.println(F("  ota status                 -> stato password OTA"));
  Serial.println(F("  ota set \"PASS\" [reboot]      -> imposta password OTA (salvata in NVS)"));
  Serial.println(F("  ota clear [reboot]          -> cancella password OTA"));
  Serial.println(F("  ota reboot                 -> reboot ESP32 (applica set/clear)"));

  Serial.println();
}

// ========================= SERIAL: tokenizer con virgolette =========================
// Supporta: wifi set 1 "SSID con spazi" "PASS con spazi"
static int tokenizeQuoted(const String& line, String* out, int maxOut) {
  int n = 0;
  int i = 0;
  const int L = (int)line.length();
  while (i < L && n < maxOut) {
    // skip spaces
    while (i < L && isspace((unsigned char)line[i])) i++;
    if (i >= L) break;

    String tok;
    if (line[i] == '"') {
      i++; // skip opening quote
      while (i < L && line[i] != '"') {
        tok += line[i++];
      }
      if (i < L && line[i] == '"') i++; // skip closing quote
    } else {
      while (i < L && !isspace((unsigned char)line[i])) {
        tok += line[i++];
      }
    }

    if (tok.length() > 0) {
      out[n++] = tok;
    }
  }
  return n;
}

static void wifiPrintHelp() {
  Serial.println(F("\nWiFi (credenziali in NVS, max 2):"));
  Serial.println(F("  wifi status"));
  Serial.println(F("  wifi creds"));
  Serial.println(F("  wifi creds showpass <1..2>"));
  Serial.println(F("  wifi set <1..2> \"SSID\" \"PASS\""));
  Serial.println(F("  wifi clear <1..2|all>"));
  Serial.println(F("  wifi apply"));
  Serial.println(F("Nota: se SSID contiene spazi, usa le virgolette doppie."));
}

static void otaPrintHelp() {
  Serial.println(F("\nOTA (password in NVS, MD5 hash):"));
  Serial.println(F("  ota status"));
  Serial.println(F("  ota set \"PASS\" [reboot]"));
  Serial.println(F("  ota clear [reboot]"));
  Serial.println(F("  ota reboot"));
  Serial.println(F("Nota: set/clear diventano effettivi dopo reboot."));
}

static void otaPrintStatus() {
  bool on = OtaStore::isConfigured();
  Serial.print(F("[OTA] Password: ") );
  Serial.println(on ? F("ON (hash in NVS)") : F("OFF"));
  Serial.println(F("[OTA] Per applicare set/clear: ota reboot (o reset)."));
}


static void wifiPrintCreds(bool showPass) {
  char ssid1[WifiStore::SSID_MAX_LEN + 1] = {0};
  char pass1[WifiStore::PASS_MAX_LEN + 1] = {0};
  char ssid2[WifiStore::SSID_MAX_LEN + 1] = {0};
  char pass2[WifiStore::PASS_MAX_LEN + 1] = {0};
  WifiStore::loadSlot(1, ssid1, sizeof(ssid1), pass1, sizeof(pass1));
  WifiStore::loadSlot(2, ssid2, sizeof(ssid2), pass2, sizeof(pass2));

  Serial.println(F("\n[WIFI] Credenziali salvate (NVS):"));
  for (int slot = 1; slot <= 2; slot++) {
    const char* ssid = (slot == 1) ? ssid1 : ssid2;
    const char* pass = (slot == 1) ? pass1 : pass2;
    Serial.print(F("  slot "));
    Serial.print(slot);
    Serial.print(F(": "));
    if (!WifiStore::isConfiguredSsid(ssid)) {
      Serial.println(F("(vuoto)"));
      continue;
    }
    Serial.print(F("SSID=\""));
    Serial.print(ssid);
    Serial.print(F("\"  PASS="));
    if (showPass) {
      Serial.println(pass);
    } else {
      char masked[12] = {0};
      WifiStore::maskPassword(pass, masked, sizeof(masked));
      Serial.println(masked);
    }
  }
}

static void wifiPrintStatus() {
  Serial.println();
  Serial.print(F("[WIFI] preferenza utente: "));
  Serial.println(g_wifiUserEnabled ? F("ON") : F("OFF"));

#if ENABLE_WIFI_OTA
  Serial.print(F("[WIFI] stato connessione: "));
  Serial.println(WiFi.isConnected() ? F("CONNECTED") : F("DISCONNECTED"));
  if (WiFi.isConnected()) {
    Serial.print(F("[WIFI] SSID: "));
    Serial.println(WiFi.SSID());
    Serial.print(F("[WIFI] IP: "));
    Serial.println(WiFi.localIP());
  }
#endif
  wifiPrintCreds(false);
}

// ========================= SERIAL (non-bloccante) =========================
// Bufferizza i caratteri e processa solo a newline, evitando timeout/attese.
// Questo evita freeze quando arrivano byte "sporchi" (rumore/EMI) mentre DFPlayer o WiFi sono attivi.
static char     g_serialLine[160] = {0};
static uint16_t g_serialLen       = 0;
static bool     g_serialOverflow  = false;

static void handleSerialCommand(String cmd) {
  cmd.trim();

  auto isDigitsOnly = [](const String& s) -> bool {
    if (s.length() == 0) return false;
    for (size_t i = 0; i < s.length(); i++) {
      if (!isDigit((unsigned char)s[i])) return false;
    }
    return true;
  };

  if (cmd.length() == 0) {
    // no-op
  }
  else if (cmd.equalsIgnoreCase("t")) {
    doTare();
  }
  else if (cmd.startsWith("c") || cmd.startsWith("C")){
    long ref = REF_G;
    int sp = cmd.indexOf(' ');
    if (sp > 0){
      long v = cmd.substring(sp+1).toInt();
      if (v > 0) ref = v;
    }
    REF_G = ref;
    Serial.print(F("[CAL] Riferimento: "));
    Serial.print(ref);
    Serial.println(F(" g (fai TARE, metti il peso noto, poi c)"));
    doCal(ref);
  }
  else if (cmd.equalsIgnoreCase("p")){
    float zt_g = (SCALE_CPG>0.01f) ? (float)zero_track_counts / SCALE_CPG : 0.0f;
    Serial.print(F("[INFO] OFFSET_RAW=")); Serial.print(OFFSET_RAW);
    Serial.print(F("  SCALE_CPG="));       Serial.print(SCALE_CPG,6);
    Serial.print(F("  REF_G="));           Serial.print(REF_G);
    Serial.print(F("  MA="));              Serial.print(maN);
    Serial.print(F("  DB_UNSTABLE="));     Serial.print(deadbandUnstable,2);
    Serial.print(F("  ST="));              Serial.print(stEnable?"on ":"off");
    Serial.print(F("  ZT="));              Serial.print(ztEnable?"on ":"off");
    Serial.print(F("  zt_cnt="));          Serial.print(zero_track_counts);
    Serial.print(F(" (≈"));                Serial.print(zt_g,2); Serial.println(F(" g)"));
  }
  else if (cmd.equalsIgnoreCase("s")){
    saveToNVS();
  }
  else if (cmd.startsWith("m ")){
    String m = cmd.substring(2);
    m.trim();
    setMode(m);
  }
  else if (cmd.equalsIgnoreCase("stop") || cmd.equalsIgnoreCase("mp3 stop")) {
    Serial.println(F("[MP3] stop"));
    audio_stopNow(true);
  }
  else if (cmd.equalsIgnoreCase("mp3 ?") || cmd.equalsIgnoreCase("mp3 status") || cmd.equalsIgnoreCase("audio ?")) {
    audio_debugStatus();
  }
  else if (cmd.startsWith("vol")) {
    // "vol <0..30>" set volume
    int sp = cmd.indexOf(' ');
    if (sp > 0) {
      int v = cmd.substring(sp + 1).toInt();
      if (v < 0) v = 0;
      if (v > 30) v = 30;
      audio_setVolume((uint8_t)v);
      Serial.print(F("[MP3] volume=")); Serial.println(v);
    } else {
      Serial.println(F("[MP3] uso: vol <0..30>"));
    }
  }
  
  else if (cmd.startsWith("hxlog")) {
    // hxlog on|off|?  ; hxlog rate <ms>
    if (cmd.equalsIgnoreCase("hxlog on")) {
      g_hxLogEnabled = true;
      Serial.println(F("[HXLOG] on"));
    } else if (cmd.equalsIgnoreCase("hxlog off")) {
      g_hxLogEnabled = false;
      Serial.println(F("[HXLOG] off"));
    } else if (cmd.equalsIgnoreCase("hxlog ?") || cmd.equalsIgnoreCase("hxlog")) {
      Serial.print(F("[HXLOG] "));
      Serial.print(g_hxLogEnabled ? "on" : "off");
      Serial.print(F("  rate="));
      Serial.print(g_hxLogPeriodMs);
      Serial.println(F("ms"));
    } else if (cmd.startsWith("hxlog rate")) {
      int sp = cmd.lastIndexOf(' ');
      if (sp > 0) {
        long ms = cmd.substring(sp + 1).toInt();
        if (ms < 50) ms = 50;
        if (ms > 5000) ms = 5000;
        g_hxLogPeriodMs = (uint32_t)ms;
        Serial.print(F("[HXLOG] rate="));
        Serial.print(g_hxLogPeriodMs);
        Serial.println(F("ms"));
      } else {
        Serial.println(F("[HXLOG] uso: hxlog rate <ms>  (50..5000)"));
      }
    } else {
      Serial.println(F("[HXLOG] comandi: hxlog on | hxlog off | hxlog ? | hxlog rate <ms>"));
    }
  }
else if (cmd.startsWith("mp3")) {
    // "mp3 <n> [capSec]" -> suona /MP3/000n.mp3
    int sp = cmd.indexOf(' ');
    if (sp > 0) {
      String rest = cmd.substring(sp + 1);
      rest.trim();
      int sp2 = rest.indexOf(' ');
      String sTrack = (sp2 > 0) ? rest.substring(0, sp2) : rest;
      String sCap   = (sp2 > 0) ? rest.substring(sp2 + 1) : String();
      sTrack.trim();
      sCap.trim();

      long n = sTrack.toInt();
      uint16_t track = (n <= 0) ? 1 : (uint16_t)n;
      uint32_t capSec = 0;
      if (sCap.length() > 0) {
        long cap = sCap.toInt();
        if (cap > 0) capSec = (uint32_t)cap;
      }

      audio_requestPlayMp3(track, capSec);
    } else {
      Serial.println(F("[MP3] uso: mp3 <n> [capSec]   (es: mp3 1, mp3 1 8)"));
    }
  }
  else if (cmd.equalsIgnoreCase("wifi") || cmd.startsWith("wifi ") || cmd.equalsIgnoreCase("wifi?") || cmd.equalsIgnoreCase("wifi ?")) {
    String t[8];
    int n = tokenizeQuoted(cmd, t, 8);
    // t[0] = wifi
    if (n <= 1) {
      wifiPrintStatus();
    } else {
      String sub = t[1];
      sub.toLowerCase();

      if (sub == "?" || sub == "help") {
        wifiPrintHelp();
      }
      else if (sub == "status") {
        wifiPrintStatus();
      }
      else if (sub == "creds") {
        if (n == 2) {
          wifiPrintCreds(false);
        } else {
          String sub2 = t[2];
          sub2.toLowerCase();
          if (sub2 == "showpass" && n >= 4) {
            int slot = t[3].toInt();
            if (slot == 1 || slot == 2) {
              Serial.println(F("[WIFI] ATTENZIONE: stampa password in chiaro."));
              if (slot == 1) {
                char ssid[WifiStore::SSID_MAX_LEN + 1] = {0};
                char pass[WifiStore::PASS_MAX_LEN + 1] = {0};
                WifiStore::loadSlot(1, ssid, sizeof(ssid), pass, sizeof(pass));
                Serial.print(F("slot 1 SSID=\"")); Serial.print(ssid); Serial.print(F("\" PASS=")); Serial.println(pass);
              } else {
                char ssid[WifiStore::SSID_MAX_LEN + 1] = {0};
                char pass[WifiStore::PASS_MAX_LEN + 1] = {0};
                WifiStore::loadSlot(2, ssid, sizeof(ssid), pass, sizeof(pass));
                Serial.print(F("slot 2 SSID=\"")); Serial.print(ssid); Serial.print(F("\" PASS=")); Serial.println(pass);
              }
            } else {
              Serial.println(F("[WIFI] uso: wifi creds showpass <1..2>"));
            }
          } else {
            wifiPrintHelp();
          }
        }
      }
      else if (sub == "set") {
        // wifi set <1..2> "SSID" "PASS"
        if (n >= 5) {
          int slot = t[2].toInt();
          if (slot != 1 && slot != 2) {
            Serial.println(F("[WIFI] slot non valido (usa 1 o 2)"));
          } else {
            bool ok = WifiStore::saveSlot((uint8_t)slot, t[3].c_str(), t[4].c_str());
            Serial.print(F("[WIFI] set slot "));
            Serial.print(slot);
            Serial.println(ok ? F(" OK") : F(" FAIL"));
#if ENABLE_WIFI_OTA
            // Se il WiFi è già configurato, ricarica e riprova senza reboot.
            if (g_wifiSetupDone) {
              Net::wifiReloadCredsAndRestart();
            }
#endif
          }
        } else {
          Serial.println(F("[WIFI] uso: wifi set <1..2> \"SSID\" \"PASS\""));
        }
      }
      else if (sub == "clear") {
        // wifi clear <1..2|all>
        if (n >= 3) {
          String which = t[2];
          which.toLowerCase();
          bool ok = false;
          if (which == "all") {
            ok = WifiStore::clearAll();
          } else {
            int slot = which.toInt();
            ok = (slot == 1 || slot == 2) ? WifiStore::clearSlot((uint8_t)slot) : false;
          }
          Serial.print(F("[WIFI] clear "));
          Serial.print(t[2]);
          Serial.println(ok ? F(" OK") : F(" FAIL"));
#if ENABLE_WIFI_OTA
          if (g_wifiSetupDone) {
            Net::wifiReloadCredsAndRestart();
          }
#endif
        } else {
          Serial.println(F("[WIFI] uso: wifi clear <1..2|all>"));
        }
      }
      else if (sub == "apply") {
#if ENABLE_WIFI_OTA
        if (g_wifiSetupDone) {
          Net::wifiReloadCredsAndRestart();
        } else {
          Serial.println(F("[WIFI] WiFi non inizializzato (preferenza OFF o ENABLE_WIFI_OTA=0)."));
        }
#else
        Serial.println(F("[WIFI] ENABLE_WIFI_OTA=0"));
#endif
      }
      else {
        wifiPrintHelp();
      }
    }
  }
  else if (cmd == "ota" || cmd.startsWith("ota ") || cmd.startsWith("OTA ") ) {
    String t[8];
    int n = tokenizeQuoted(cmd, t, 8);
    if (n <= 1) {
      otaPrintHelp();
    } else {
      String sub = t[1];
      sub.toLowerCase();
      bool doReboot = false;
      if (n >= 3) {
        String last = t[n-1];
        last.toLowerCase();
        if (last == "reboot") doReboot = true;
      }

      if (sub == "?" || sub == "help") {
        otaPrintHelp();
      } else if (sub == "status") {
        otaPrintStatus();
      } else if (sub == "set") {
        // ota set "PASS" [reboot]
        if (n >= 3) {
          bool ok = OtaStore::savePassword(t[2].c_str());
          Serial.println(ok ? F("[OTA] set OK (hash salvato).") : F("[OTA] set FAIL"));
          if (ok) Serial.println(F("[OTA] Ora fai: ota reboot (serve reboot per applicare)."));
          if (ok && doReboot) {
            delay(100);
            ESP.restart();
          }
        } else {
          Serial.println(F("[OTA] uso: ota set \"PASS\" [reboot]"));
        }
      } else if (sub == "clear") {
        // ota clear [reboot]
        bool ok = OtaStore::clear();
        Serial.println(ok ? F("[OTA] clear OK") : F("[OTA] clear FAIL"));
        Serial.println(F("[OTA] Ora fai: ota reboot (serve reboot per applicare)."));
        if (ok && doReboot) {
          delay(100);
          ESP.restart();
        }
      } else if (sub == "reboot") {
        Serial.println(F("[OTA] reboot..."));
        delay(100);
        ESP.restart();
      } else {
        otaPrintHelp();
      }
    }
  }
  else if (cmd.equalsIgnoreCase("st on")){
    stEnable = true;
    resetFiltersAndState();
    Serial.println(F("[ST] on"));
  }
  else if (cmd.equalsIgnoreCase("st off")){
    stEnable = false;
    resetFiltersAndState();
    Serial.println(F("[ST] off (LIVE)"));
  }
  else if (cmd.equalsIgnoreCase("st ?")){
    Serial.print(F("[ST] ")); Serial.println(stEnable?"on":"off");
  }
  else if (cmd.equalsIgnoreCase("zt on")){
    ztEnable = true;
    Serial.println(F("[ZT] on"));
  }
  else if (cmd.equalsIgnoreCase("zt off")){
    ztEnable = false;
    Serial.println(F("[ZT] off"));
  }
  else if (cmd.equalsIgnoreCase("zt reset")){
    zero_track_counts = 0;
    Serial.println(F("[ZT] accumulatore azzerato."));
  }
  else if (cmd.equalsIgnoreCase("zt ?")){
    float zt_g = (SCALE_CPG>0.01f) ? (float)zero_track_counts / SCALE_CPG : 0.0f;
    Serial.print(F("[ZT] ")); Serial.print(ztEnable?"on ":"off");
    Serial.print(F("  accum=")); Serial.print(zero_track_counts);
    Serial.print(F(" counts (≈")); Serial.print(zt_g,2); Serial.println(F(" g)"));
  }
  else if (isDigitsOnly(cmd)) {
    uint16_t t = (uint16_t)cmd.toInt();
    if (t == 0) t = 1;
    audio_requestPlayMp3(t);
  }
  else {
    printHelp();
  }
}

static void serial_task() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;

    if (ch == '\n') {
      if (!g_serialOverflow) {
        String cmd = String(g_serialLine);
        handleSerialCommand(cmd);
      } else {
        Serial.println(F("[SER] linea troppo lunga, scartata"));
      }
      g_serialLen = 0;
      g_serialOverflow = false;
      g_serialLine[0] = '\0';
      continue;
    }

    if (!g_serialOverflow) {
      if (g_serialLen < (sizeof(g_serialLine) - 1)) {
        g_serialLine[g_serialLen++] = ch;
        g_serialLine[g_serialLen] = '\0';
      } else {
        g_serialOverflow = true; // scarta fino al prossimo newline
      }
    }
  }
}

void loadFromNVS(){
  // Comportamento "come prima" (firmware NAU): nessuna firma sensore.
  // Se in NVS ci sono valori vecchi/non compatibili, vedrai pesi sballati -> fai CAL e poi salva.
  prefs.begin("minu_scale", true);
  long  off = prefs.getLong ("offset", 0);
  float sc  = prefs.getFloat("scale",  NAN);
  long  rg  = prefs.getLong ("ref_g",  DEFAULT_REF_G);
  prefs.end();

  SCALE_CPG = (!isnan(sc) && sc>0.01f) ? sc : DEFAULT_CPG;
  REF_G     = rg;
  OFFSET_RAW= (off != 0) ? off : DEFAULT_ZERO_RAW;
}

void saveToNVS(){
  prefs.begin("minu_scale", false);
  prefs.putLong("offset", OFFSET_RAW);
  prefs.putFloat("scale", SCALE_CPG);
  prefs.putLong("ref_g",  REF_G);
  prefs.end();
  Serial.println(F("[SAVE] Parametri salvati in NVS."));
}


// ========================= ZERO-TRACKING CORE =========================
bool isQuietForZT(){
  int nQuiet = constrain((int)ceil((float)ZT_QUIET_MS / (float)SAMPLE_MS), 2, MAX_HIST);
  float rngQ = rangeLastNSamples(nQuiet);
  float slpQ = fabsf(slopeLastNSamples(nQuiet));
  return (rngQ <= ZT_QUIET_RANGE_G) && (slpQ <= ZT_QUIET_SLOPE_GPS);
}

void applyZTstepTowardZero(float gNow){
  if (SCALE_CPG <= 0.01f) return;

  long stepCounts = lroundf(ZT_STEP_G * SCALE_CPG);
  if (stepCounts < 1) stepCounts = 1;
  long cap = lroundf(ZT_MAX_G * SCALE_CPG);

  if      (gNow >  ZT_STEP_G) zero_track_counts += stepCounts;
  else if (gNow < -ZT_STEP_G) zero_track_counts -= stepCounts;

  if (zero_track_counts >  cap) zero_track_counts =  cap;
  if (zero_track_counts < -cap) zero_track_counts = -cap;
}

// Snap-to-zero su scarico: ritorna true se eseguito
bool trySnapOnUnload(float gNow, float slope_now, bool quietNow){
  if (!UNLOAD_SNAP_ENABLE) return false;

  unsigned long now = millis();
  if (now - lastSnapMs < UNLOAD_COOLDOWN_MS) return false;

  if (slope_now > UNLOAD_SLOPE_GPS_NEG) return false;           // deve essere sufficientemente negativa
  if (fabsf(gNow) > UNLOAD_CROSS_WIN_G) return false;           // vicino allo zero

  if (!quietNow){
    int nSt = constrain((int)ceil((float)UNLOAD_STABLE_MS / (float)SAMPLE_MS), 2, MAX_HIST);
    float rngSt = rangeLastNSamples(nSt);
    if (rngSt > UNLOAD_STABLE_RANGE_G) return false;
  }

  float snap_g = gNow;
  if (snap_g >  UNLOAD_SNAP_MAX_G) snap_g =  UNLOAD_SNAP_MAX_G;
  if (snap_g < -UNLOAD_SNAP_MAX_G) snap_g = -UNLOAD_SNAP_MAX_G;

  long deltaCounts = lroundf(snap_g * SCALE_CPG);
  zero_track_counts += deltaCounts;

  long cap = lroundf(ZT_MAX_G * SCALE_CPG);
  if (zero_track_counts >  cap) zero_track_counts =  cap;
  if (zero_track_counts < -cap) zero_track_counts = -cap;

  lastSnapMs = now;
  Serial.println(F("[ZT] Snap-to-zero (scarico)."));
  return true;
}

// ========================= TARE / CAL / MODALITÀ =========================
void doTare(){
  // In HX ERROR: disabilita tara (azione potenzialmente pericolosa)
  if (hxHealth_isError(&g_hxHealth)) {
    Serial.println(F("[HX] TARE bloccata: HX711 in ERROR"));
    return;
  }
  // Avvia la tara non bloccante: UI clamp 1.5s + raccolta campioni per OFFSET_RAW
  if (gTareUiActive) return;
  tareStart(millis());
}
void doCal(long ref_g){
  // In HX ERROR: disabilita calibrazione
  if (hxHealth_isError(&g_hxHealth)) {
    Serial.println(F("[HX] CAL bloccata: HX711 in ERROR"));
    return;
  }
  long rawRef = readRawHXAvg(15);
  long delta  = rawRef - OFFSET_RAW;
  if (delta == 0) delta = 1;
  SCALE_CPG = (float)delta / (float)ref_g;
  zero_track_counts = 0;
  resetFiltersAndState();
  Serial.print(F("[CAL]  SCALE_CPG="));
  Serial.println(SCALE_CPG,6);
}

void setMode(const String& mode){
  if (mode.equalsIgnoreCase("work") || mode.equalsIgnoreCase("normal")){
    const bool changed = (currentMode != SCALE_MODE_WORK);
    maN = MA_DEFAULT;
    setMA(maN);
    deadbandUnstable = DB_UNSTABLE_N;
    stEnable = true;
    ztEnable = true;
    resetFiltersAndState();
    currentMode = SCALE_MODE_WORK;
    if (changed && !g_isBooting) {
      // 0017.mp3 = Modalità WORK
      audio_requestPlayMp3(17);
    }
    Serial.println(F("[MODE] work/normal: MA=6, DB=0.10 g, ST=on, ZT=on"));
  }
  else if (mode.equalsIgnoreCase("fine") || mode.equalsIgnoreCase("live")){
    const bool changed = (currentMode != SCALE_MODE_LIVE);
    maN = MA_FINE;
    setMA(maN);
    deadbandUnstable = DB_UNSTABLE_F;
    stEnable = false;   // live
    ztEnable = true;
    resetFiltersAndState();
    currentMode = SCALE_MODE_LIVE;
    if (changed && !g_isBooting) {
      // 0018.mp3 = Modalità LIVE
      audio_requestPlayMp3(18);
    }
    Serial.println(F("[MODE] fine/live: MA=4, DB=0.05 g, ST=off, ZT=on"));
  }
  else {
    Serial.println(F("[MODE] sconosciuta. Usa: m work | m normal | m fine | m live"));
  }
}


void toggleModeFromKeypad(){
  if (currentMode == SCALE_MODE_WORK){
    // Passa a modalità LIVE (fine)
    setMode("live");
  } else {
    // Torna a modalità WORK (normal)
    setMode("work");
  }
}

// Gestione degli eventi tastiera frontale
void handleKeyEvent(KeyCode key){
  switch (key){
    case KEY_TARE:
      Serial.println(F("[KEYPAD] TARE pressed"));
      doTare();
      buzzerKeyClick();
      break;

    case KEY_MODE:
      Serial.println(F("[KEYPAD] MODE pressed"));
      // Feedback immediato al tasto (il cambio modalità è annunciato anche via MP3 0017/0018)
      buzzerKeyClick();
      toggleModeFromKeypad();
      break;

    case KEY_WIFI:
      Serial.println(F("[KEYPAD] WIFI toggle"));
      // Feedback immediato al tasto: la connessione/disconnessione reale avviene in differita (background)
      buzzerKeyClick();
      wifiToggleUserSetting();
      // Audio: 0005=attivazione modulo WiFi (da tasto), 0006=disattivazione modulo WiFi (da tasto).
      // Nota: la connessione reale (quando avviene) viene annunciata da wifiAudioUpdate() con 0008.
      audio_requestPlayMp3(g_wifiUserEnabled ? 5 : 6);
      lastOledMs = 0;
      break;

    case KEY_SLEEP: {
      Serial.println(F("[KEYPAD] SLEEP pressed"));
#if ENABLE_WIFI_OTA
      // Safety: durante OTA non andare in sleep.
      if (Net::isOtaInProgress()) {
        Serial.println(F("[SLEEP] bloccato: OTA in corso"));
        buzzerWarn();
        break;
      }
#endif
      // Forza la stessa sequenza di standby dell'inattività: schermata Zzz... 5s + 0003.mp3,
      // poi light-sleep (wake da tastiera). Qualsiasi tasto durante i 5s annulla.
      ui_renderSleepZzz();
      audio_requestPlayMp3(3);
      g_inactivitySleepStage = INACT_ZZZ;
      // IMPORTANTISSIMO: usa lo stesso timestamp della loop (g_lastKeyPressMs) per evitare underflow
      // (se usiamo millis() qui, può risultare > 'now' e la sottrazione unsigned fa entrare in sleep subito).
      g_inactivityStageStartMs = g_lastKeyPressMs;
      lastOledMs = 0;
      break;
    }

    case KEY_ENTER:
      Serial.println(F("[KEYPAD] ENTER pressed"));
      buzzerKeyClick();
      // Placeholder: registrazione peso (vuota per ora)
      // Audio 0009
      audio_requestPlayMp3(9);
      break;

    case KEY_SKIP:
      Serial.println(F("[KEYPAD] SKIP pressed"));
      buzzerKeyClick();
      // Placeholder: salto ingrediente (vuota per ora)
      // Audio 0010
      audio_requestPlayMp3(10);
      break;

    case KEY_TOTAL:
      Serial.println(F("[KEYPAD] TOTAL pressed"));
      buzzerKeyClick();
      // Placeholder: mostra peso totale registrato (vuota per ora)
      // Audio 0020
      audio_requestPlayMp3(20);
      break;

    case KEY_CLEAR:
      Serial.println(F("[KEYPAD] CLEAR pressed"));
      buzzerKeyClick();
      // Placeholder: libera memoria (vuota per ora)
      // Audio 0021
      audio_requestPlayMp3(21);
      break;

    case KEY_NONE:
    default:
      break;
  }
}


// ========================= AUTO-TARE ALL’AVVIO =========================
bool autoTareOnBoot(uint16_t maxSamples){
  if (!AUTO_TARE_ON_BOOT){
    Serial.println(F("[BOOT] Auto-TARE disattivata."));
    return true;
  }

  // Approccio 1 (robusto ma veloce) @ 80 SPS:
  // 1) discard iniziale (assestamento)
  // 2) raccoglie fino a maxSamples (default 64)
  // 3) se trova una finestra "quiet" (range <= soglia per AUTO_TARE_QUIET_MS) può fermarsi prima
  // 4) imposta OFFSET_RAW con trimmed-mean sui campioni raccolti (taglia outlier)
  Serial.println(F("[BOOT] Auto-TARE (robusta) ..."));
  delay(AUTO_TARE_SETTLE_MS);

  const uint16_t minSamples = AUTO_TARE_MIN_SAMPLES;
  const uint16_t MAX_BUF = 64;
  if (maxSamples < minSamples) maxSamples = minSamples;
  if (maxSamples > MAX_BUF)    maxSamples = MAX_BUF;

  long buf[MAX_BUF];
  uint16_t n = 0;
  uint32_t quietStart = 0;

  while (n < maxSamples){
    long r = readRawHXOnceBlocking(200);
    if (r == 0) {
      Serial.println(F("[BOOT] Auto-TARE FAIL (timeout lettura HX711)"));
      return false;
    }

    buf[n++] = r;

    if (n >= minSamples){
      // Range sull'ultima finestra minSamples
      long mn = buf[n - minSamples];
      long mx = mn;
      for (uint16_t i = n - minSamples; i < n; i++){
        long v = buf[i];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
      }

      float rangeG = 999999.0f;
      if (SCALE_CPG > 0.01f){
        rangeG = float(mx - mn) / SCALE_CPG;
      }

      if (rangeG <= AUTO_TARE_RANGE_G){
        if (quietStart == 0) quietStart = millis();
        if (millis() - quietStart >= AUTO_TARE_QUIET_MS){
          break; // stabile abbastanza
        }
      } else {
        quietStart = 0;
      }
    }
  }

  // Trimmed mean sui campioni raccolti: taglia outlier (10% per lato se possibile).
  // Così riduci l'effetto di un singolo spike/colpo durante l'avvio.
  long tmp[MAX_BUF];
  for (uint16_t i = 0; i < n; i++) tmp[i] = buf[i];

  // insertion sort (n <= 64)
  for (uint16_t i = 1; i < n; i++){
    long key = tmp[i];
    int j = (int)i - 1;
    while (j >= 0 && tmp[j] > key){
      tmp[j + 1] = tmp[j];
      j--;
    }
    tmp[j + 1] = key;
  }

  uint16_t trim = (n >= 40) ? (uint16_t)(n / 10) : 0; // 10% per lato solo se abbastanza campioni
  uint16_t i0 = trim;
  uint16_t i1 = n - trim;

  int64_t s = 0;
  uint16_t useN = 0;
  for (uint16_t i = i0; i < i1; i++){
    s += (int64_t)tmp[i];
    useN++;
  }
  if (useN == 0) { // safety
    Serial.println(F("[BOOT] Auto-TARE FAIL (useN=0)"));
    return false;
  }

  long rawZero = (long)(s / (int64_t)useN);

  OFFSET_RAW = rawZero;
  zero_track_counts = 0;
  resetFiltersAndState();

  Serial.print(F("[BOOT] Auto-TARE OK. OFFSET_RAW="));
  Serial.print(OFFSET_RAW);
  Serial.print(F("  n="));    Serial.print(n);
  Serial.print(F("  useN=")); Serial.println(useN);

  if (n >= minSamples && quietStart == 0){
    Serial.println(F("[BOOT] Auto-TARE: finestra quiet non trovata, ma offset impostato comunque. Se serve: premi TARA."));
  }

  return true;
}


// ========================= INIT HX711 =========================
bool initHX711(){
  Serial.println(F("[HX711] Init HX711..."));

  g_hxBeginOk = false;

  hx711_begin(HX711_DOUT_PIN, HX711_SCK_PIN, HX711_GAIN, HX711_PULSE_US);

  uint32_t t0 = millis();
  while (millis() - t0 < HX711_INIT_TIMEOUT_MS) {
    if (hx711_is_ready()) {
      // prima lettura di stabilizzazione (scarta)
      (void)hx711_read();
      g_hxBeginOk = true;
      Serial.println(F("[HX711] OK (DOUT ready)"));
      return true;
    }
    delay(10);
  }

  Serial.println(F("[HX711] FAIL - DOUT non pronto (cablaggio/alimentazione/livello logico?)"));
  return false;
}

// ------------------------- BOOT UX helpers -------------------------
// ------------------------- WiFi audio (eventi) -------------------------
static void wifiAudioUpdate(uint32_t now) {
#if ENABLE_WIFI_OTA || ENABLE_ARDUINO_CLOUD
  // Evita di fare annunci durante il boot (per non sovrapporre 0001/0002).
  if (g_isBooting) {
    g_wifiPrevConnected = WiFi.isConnected();
    return;
  }

  // Se l'utente ha disattivato il WiFi, non fare annunci (né errori).
  if (!g_wifiUserEnabled) {
    g_wifiPrevConnected = false;
    return;
  }

  // Silenzio post-wake: evita che l'audio WiFi parta subito dopo lo sleep
  // (es. durante 0004.mp3) e replica il comportamento precedente (niente annuncio al wake).
  if (g_wifiAudioSuppressUntilMs != 0 && (int32_t)(now - g_wifiAudioSuppressUntilMs) < 0) {
    const bool c = WiFi.isConnected();
    g_wifiPrevConnected = c;
    if (c && g_wifiSuppressNextConnectAnnounce) {
      g_wifiSuppressNextConnectAnnounce = false;
    }
    return;
  }
  if (g_wifiAudioSuppressUntilMs != 0) g_wifiAudioSuppressUntilMs = 0; // finestra scaduta

  const bool connectedNow = WiFi.isConnected();
  if (connectedNow != g_wifiPrevConnected) {
    g_wifiPrevConnected = connectedNow;
    // 0008 = connessione avvenuta.
    // - Al wake: non annunciamo la riconnessione.
    if (connectedNow) {
      if (g_wifiSuppressNextConnectAnnounce) {
        g_wifiSuppressNextConnectAnnounce = false;
      } else {
        audio_requestPlayMp3(8);
      }
    }
  }

  // 0007=errore connessione (cooldown)
  if (!connectedNow) {
    wl_status_t st = WiFi.status();
    if (st == WL_CONNECT_FAILED || st == WL_NO_SSID_AVAIL) {
      if (g_wifiErrLastMs == 0 || (now - g_wifiErrLastMs) >= WIFI_ERR_COOLDOWN_MS) {
        audio_requestPlayMp3(7);
        g_wifiErrLastMs = now;
      }
    }
  }
#endif
}

static void bootPump(uint32_t ms) {
  uint32_t start = millis();
  while (millis() - start < ms) {
#if ENABLE_WIFI_OTA || ENABLE_ARDUINO_CLOUD
    Net::update();
#endif
    // Manteniamo viva la lettura tastiera anche durante boot (utile se qualcosa si blocca)
    uint32_t now = millis();
    keypad_update(now);
    audio_task(now);
    wifiAudioUpdate(now);
    (void)keypad_get_event();
    delay(10);
  }
}

static void bootShow(const char* line1, const char* line2, uint16_t holdMs = 350) {
  ui_showBoot(line1, line2);
  bootPump(holdMs);
}

static void bootWaitTare() {
  while (true) {
    uint32_t now = millis();
    keypad_update(now);
    audio_task(now);
    wifiAudioUpdate(now);
    KeyCode k = keypad_get_event();
    if (k == KEY_TARE) {
      buzzerKeyClick();
      return;
    }
#if ENABLE_WIFI_OTA || ENABLE_ARDUINO_CLOUD
    Net::update();
#endif
    delay(10);
  }
}


// ========================= SETUP =========================
void setup(){
  Serial.begin(115200);
  delay(50);

  // LED esterno: indicatore sleep (OFF all'avvio)
  sleepLedOff();
  g_lastKeyPressMs = millis();

  // DISPLAY SUBITO (per mostrare che la bilancia sta avviando)
  ui_init();
  ui_showBoot("Accensione...", "");

  // Suono subito dopo che il display è acceso
  buzzerInit();
  // buzzerBootMelody() resta nel codice, ma non lo lanciamo se usiamo l'MP3 di avvio.
  static const bool BOOT_BUZZER_MELODY_ENABLED = false;
  if (BOOT_BUZZER_MELODY_ENABLED) {
    buzzerBootMelody();
  }
  
  Serial.println(F("\n=== ESP32 + HX711 — versione light + OLED SSD1322 ==="));

  printHelp();

  // Tastierino (non mostriamo la fase, ma ci serve già da boot per gestire ACK errori)
  keypad_init();

  // Audio DFPlayer: init pin/driver e lo portiamo subito "pronto" all'avvio,
  // così possiamo riprodurre un suono immediatamente (senza cold-start sul primo mp3).
  // Nota: il percorso cold-start resta disponibile dentro audio_requestPlayMp3().
  audio_begin();
  audio_primeReady();
  // 0001.mp3 = Avvio bilancia
  audio_requestPlayMp3(1);


#if ENABLE_WIFI_OTA
  // Preferenza utente WiFi (persistente in NVS):
  // - ON  => abilita WiFi background (OTA disponibile)
  // - OFF => WiFi resta spento finché l'utente non lo riattiva dal tasto
  g_wifiUserEnabled = loadWifiUserEnabledFromNVS();

  // OTA handlers si configurano sempre: si attivano quando il WiFi risulta connesso.
  Net::otaSetup("minu-bench-scale");

  if (g_wifiUserEnabled) {
    Net::wifiSetup();
    g_wifiSetupDone = true;
  } else {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    g_wifiSetupDone = false;
    Serial.println(F("[NET] WiFi OFF (preferenza utente)"));
  }
#endif

#if ENABLE_ARDUINO_CLOUD
  Net::cloudSetup();
#endif


  // ------------------------ Sequenza boot "leggibile" ------------------------
  bootShow("I2C", "Avvio bus...");
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  bootShow("I2C", "OK");

  // HX711
  bootShow("Sensore peso", "Init HX711...");
  bool hxOk = initHX711();

  // Se il modulo non è trovato, è un errore duro: ritenta su TARE
  while (!g_hxBeginOk) {
    ui_showError("HX711", "Modulo non trovato", "Premi TARA per ritentare");
    // 0015.mp3 = Errore sensore peso (HX)
    audio_requestPlayMp3(15);
    bootWaitTare();
    bootShow("Sensore peso", "Ritentativo...");
    hxOk = initHX711();
  }

  // Config/AFE fallite: mostriamo errore, ma dopo ACK si procede (può comunque funzionare)
  if (!hxOk) {
    ui_showError("HX711", "Init: FAIL", "Premi TARA per continuare");
    // 0015.mp3 = Errore sensore peso (HX)
    audio_requestPlayMp3(15);
    bootWaitTare();
  }
  bootShow("Sensore peso", "OK");

  // NVS
  bootShow("Parametri", "Carico NVS...");
  loadFromNVS();
  bootShow("Parametri", "OK");

  // Preset iniziale
  bootShow("Preset", "WORK");
  // Preset WORK = modalità "work/normal" (MA default, ST on, ZT on)
  setMode("work");

  // Auto-TARE
  if (AUTO_TARE_ON_BOOT) {
    bootShow("Auto-TARE", "In corso...");
    bool tareOk = autoTareOnBoot(AUTO_TARE_SAMPLES);
    if (!tareOk) {
      ui_showError("Auto-TARE", "Timeout/FAIL", "Premi TARA per continuare");
      // 0016.mp3 = Errore TARA al boot
      audio_requestPlayMp3(16);
      bootWaitTare();
    }
  }

  // Batteria
  bootShow("Batteria", "Init INA219...");
  battery_init();
  if (!battery_is_available()) {
    ui_showError("Batteria", "INA219 non trovato", "Premi TARA per continuare");
    // 0014.mp3 = Errore lettura batteria (INA)
    audio_requestPlayMp3(14);
    bootWaitTare();
  }

  bootShow("Avvio", "Completato");

  // 0002.mp3 = Boot completato
  audio_requestPlayMp3(2);
  g_isBooting = false;

  // Arm HX health monitor (runtime)
  hxHealth_init(&g_hxHealth, HX_HEALTH_CFG);

  lastOledMs = millis();

}

//debug batteria
static uint32_t lastBattDebug = 0;

// ========================= BATTERIA SLA 6V: sicurezza ESP =========================
// Sotto certe tensioni (filtrate) il buck a 5V può perdere margine -> latenze/reset.
// Strategia:
//  - 0 tacche (EMPTY): icona batteria lampeggiante + 0011.mp3 + beep (cooldown 5 min)
//  - "fase di stacco" (<= V_SAFE_SHUTDOWN_MIN_V): mostra solo avviso + 0012.mp3 (all'ingresso e a metà) + beep ogni 10s per 120s, poi LIGHT-SLEEP
//  - Wake SOLO da tastiera (qualsiasi tasto). Niente wake automatico.

static const float    V_SAFE_SHUTDOWN_MIN_V     = 5.80f;   // inizia countdown per sleep (V)
static const float    V_SAFE_SHUTDOWN_CLEAR_V   = 5.90f;   // isteresi: annulla countdown se risali sopra
// Uscita "soft" dal countdown: se recuperi stabilmente sopra questa soglia, annulliamo il countdown.
// Deve stare > V_SAFE_SHUTDOWN_MIN_V, e idealmente coincide con il ritorno a 1 tacca (>= V_CRITICAL_MIN).
static const float    V_SAFE_SHUTDOWN_EXIT_V    = 5.85f;
static const float    V_HARD_SLEEP_MIN_V        = 5.70f;   // troppo bassa: vai a sleep subito
static const uint32_t SAFE_SHUTDOWN_DEBOUNCE_MS = 5000;    // deve restare sotto soglia per 5s

// Deve restare sopra V_SAFE_SHUTDOWN_EXIT_V per un po' prima di annullare il countdown.
static const uint32_t SAFE_SHUTDOWN_EXIT_STABLE_MS = 15000; // 15s

static const uint32_t PRE_SLEEP_COUNTDOWN_MS    = 120000;   // 120s avviso prima dello sleep
static const uint32_t PRE_SLEEP_MID_AUDIO_MS    = (PRE_SLEEP_COUNTDOWN_MS / 2); // 0012 anche a metà countdown
static const uint32_t BEEP_EMPTY_MS             = 300000;   // beep ogni 5 min (0 tacche)
static const uint32_t BEEP_COUNTDOWN_MS         = 10000;    // beep ogni 10s durante countdown

// Robustezza anti-flapping:
// - ingresso stato '0 tacche' (EMPTY): deve restare in EMPTY per un po' (con V > 5.80) prima di avvisare
// - uscita (recupero): deve restare sopra clear per un po' prima di resettare i cooldown
// - avvisi sonori (mp3) non si ripetono prima di 5 minuti
static const uint32_t BATT_EMPTY_DEBOUNCE_MS    = 10000;    // EMPTY stabile per 10s prima di avviso '0 tacche'
static const uint32_t BATT_RECOVERY_STABLE_MS   = 30000;    // sopra clear per 30s prima di resettare gli avvisi
static const uint32_t BATT_ALERT_COOLDOWN_MS    = 300000;   // 5 min

static uint32_t g_lowBattSinceMs      = 0;
static uint32_t g_preSleepStartMs     = 0;
static uint32_t g_countdownExitSinceMs = 0;
static uint32_t g_lastEmptyBeepMs     = 0;
static uint32_t g_lastCountdownBeepMs = 0;
// Durante countdown: riproduci 0012 anche a metà (una sola volta per episodio)
static bool     g_countdownMidMp3Played = false;

// Anti-flapping stato '0 tacche'
static uint32_t g_emptyCandidateSinceMs = 0;
static bool     g_emptyWarnActive       = false;

// Recupero stabile sopra soglia clear
static uint32_t g_recoverySinceMs = 0;

// Cooldown mp3 batteria
static uint32_t g_lastMp3BattLowMs  = 0; // 0011
static uint32_t g_lastMp3BattCritMs = 0; // 0012

// Pre-sleep "Zzz..." per batteria scarica (prima del light-sleep): 5s + 0013
enum BattSleepStage : uint8_t { BATT_SLEEP_NONE = 0, BATT_SLEEP_ZZZ = 1 };
static BattSleepStage g_battSleepStage = BATT_SLEEP_NONE;
static uint32_t g_battStageStartMs = 0;
static const uint32_t BATT_ZZZ_MS = 5000;

static void enterLowBatteryLightSleep() {
  // Ultima schermata: avviso e richiesta alimentatore, poi sleep.
  if (battery_is_available()) {
    BatteryStatus st = battery_get_status();

    ui_showBatteryShutdown(st.voltage_V);
  } else {
    ui_showError("BATTERIA", "INA219 non trovato", "");
  }

  // Un piccolo margine per far vedere la schermata
  delay(80);

  // Spegni DFPlayer (MOSFET) prima del light-sleep per ridurre consumi
  audio_powerOffNow();

  // Riduci consumi
#if ENABLE_WIFI_OTA || ENABLE_ARDUINO_CLOUD
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
#endif

  // Prepara wake da tastiera:
  // - Righe a LOW fisso
  // - Colonne INPUT_PULLUP
  // Premendo un tasto, una colonna viene tirata LOW -> wake.
  const int* rows = nullptr; int nRows = 0;
  const int* cols = nullptr; int nCols = 0;
  keypad_get_pins(&rows, &nRows, &cols, &nCols);

  for (int i = 0; i < nRows; i++) {
    pinMode(rows[i], OUTPUT);
    digitalWrite(rows[i], LOW);
  }
  for (int i = 0; i < nCols; i++) {
    pinMode(cols[i], INPUT_PULLUP);
    // abilita wake su livello LOW (solo in light-sleep)
    gpio_wakeup_enable((gpio_num_t)cols[i], GPIO_INTR_LOW_LEVEL);
  }

  // Spegni display (riduce consumi). Rimane spento finché non riparti.
  ui_powerSave(true);

  // Abilita wake GPIO (light-sleep) e vai in sleep.
  // Niente timer wake: resta lì finché non premi un tasto.
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
  esp_sleep_enable_gpio_wakeup();

  delay(20);
  esp_light_sleep_start();

  // Dopo wake: riparti pulito (re-init periferiche).
  ESP.restart();
}

static bool maybeEnterSafeShutdown(uint32_t nowMs) {
  if (!battery_is_available()) return false;
  BatteryStatus st = battery_get_status();

  // In carica: azzera subito tutta la logica di safety e avvisi.
  if (st.charging) {
    g_lowBattSinceMs = 0;
    g_preSleepStartMs = 0;
    g_lastEmptyBeepMs = 0;
    g_lastCountdownBeepMs = 0;
    g_countdownMidMp3Played = false;
    g_countdownExitSinceMs = 0;

    g_emptyCandidateSinceMs = 0;
    g_emptyWarnActive = false;

    g_recoverySinceMs = 0;

    g_lastMp3BattLowMs = 0;
    g_lastMp3BattCritMs = 0;

    g_battSleepStage = BATT_SLEEP_NONE;
    g_battStageStartMs = 0;
    return false;
  }

  // Sopra soglia clear: annulla subito countdown/stacco.
  // Gli avvisi si riarmeranno solo dopo un recupero stabile, per evitare flapping.
  if (st.voltage_V >= V_SAFE_SHUTDOWN_CLEAR_V) {
    // Stop countdown e pre-sleep.
    g_lowBattSinceMs = 0;
    g_preSleepStartMs = 0;
    g_lastCountdownBeepMs = 0;
    g_countdownMidMp3Played = false;
    g_countdownExitSinceMs = 0;

    // Fuori dalla zona EMPTY: stop avviso 0 tacche.
    g_emptyCandidateSinceMs = 0;
    g_emptyWarnActive = false;

    // Se eravamo nella schermata pre-sleep, annulla.
    g_battSleepStage = BATT_SLEEP_NONE;
    g_battStageStartMs = 0;

    // Recovery stabile: dopo X secondi sopra clear, possiamo riazzerare i cooldown
    // così al prossimo vero episodio avvisiamo subito.
    if (g_recoverySinceMs == 0) g_recoverySinceMs = nowMs;
    if ((nowMs - g_recoverySinceMs) >= BATT_RECOVERY_STABLE_MS) {
      g_lastEmptyBeepMs = 0;
      g_lastMp3BattLowMs = 0;
      g_lastMp3BattCritMs = 0;
      g_recoverySinceMs = 0;
    }

    return false;
  }

  // Sotto clear: non siamo in recupero.
  g_recoverySinceMs = 0;

  // Se siamo nella fase "Zzz" pre-sleep per batteria (0013):
  // mostra Z Z Z per 5s, poi vai in light-sleep.
  if (g_battSleepStage == BATT_SLEEP_ZZZ) {
    ui_renderSleepZzz();
    if (st.voltage_V <= V_HARD_SLEEP_MIN_V || (nowMs - g_battStageStartMs) >= BATT_ZZZ_MS) {
      enterLowBatteryLightSleep(); // non ritorna
    }
    return true; // oscuriamo la pesata
  }

  // Zona "0 tacche" (UI = EMPTY): ingresso debounced.
  // Importante: l'avviso 0011 deve essere coerente con la UI, quindi qui guardiamo
  // direttamente lo stato calcolato dal battery_monitor (st.level) e NON la finestra
  // di tensione del countdown.
  // Inoltre limitiamo la zona "0 tacche" al cuscinetto prima del countdown: sopra 5.80 V.
  const bool inEmptyZone = (st.level == BATT_LEVEL_EMPTY) && (st.voltage_V > V_SAFE_SHUTDOWN_MIN_V);
  if (inEmptyZone) {
    if (!g_emptyWarnActive) {
      if (g_emptyCandidateSinceMs == 0) g_emptyCandidateSinceMs = nowMs;
      if ((nowMs - g_emptyCandidateSinceMs) >= BATT_EMPTY_DEBOUNCE_MS) {
        g_emptyWarnActive = true;
      }
    }
  } else {
    g_emptyCandidateSinceMs = 0;
    g_emptyWarnActive = false;
  }

  // Stato "0 tacche": beep raro, UI normale
  if (g_emptyWarnActive) {
    // 0011.mp3 = Batteria bassa (non ripetere prima di 5 minuti)
    if (g_lastMp3BattLowMs == 0 || (nowMs - g_lastMp3BattLowMs) >= BATT_ALERT_COOLDOWN_MS) {
      audio_requestPlayMp3(11);
      g_lastMp3BattLowMs = nowMs;
    }

    // Buzzer: non ripetere troppo spesso
    if (g_lastEmptyBeepMs == 0 || (nowMs - g_lastEmptyBeepMs) >= BEEP_EMPTY_MS) {
      buzzerWarn();
      g_lastEmptyBeepMs = nowMs;
    }

    return false; // non oscurare la pesata
  }

  // Sotto hard-min: sleep immediato (dopo una schermata)
  if (st.voltage_V <= V_HARD_SLEEP_MIN_V) {
    // 0012.mp3 = Batteria critica (non ripetere prima di 5 minuti)
    if (g_lastMp3BattCritMs == 0 || (nowMs - g_lastMp3BattCritMs) >= BATT_ALERT_COOLDOWN_MS) {
      audio_requestPlayMp3(12);
      g_lastMp3BattCritMs = nowMs;
    }
    buzzerWarn();
    enterLowBatteryLightSleep(); // non ritorna
    return true;
  }

  // Countdown già avviato: resta stabile anche se la tensione rimbalza appena sopra 5.80.
  // Uscita solo se recuperi stabilmente sopra V_SAFE_SHUTDOWN_EXIT_V (o sopra CLEAR, gestito sopra).
  if (g_preSleepStartMs != 0) {
    if (st.voltage_V >= V_SAFE_SHUTDOWN_EXIT_V) {
      if (g_countdownExitSinceMs == 0) g_countdownExitSinceMs = nowMs;
      if ((nowMs - g_countdownExitSinceMs) >= SAFE_SHUTDOWN_EXIT_STABLE_MS) {
        // Cancella countdown: consideriamo questo un "nuovo episodio" -> riarma anche l'audio 0012.
        g_lowBattSinceMs = 0;
        g_preSleepStartMs = 0;
        g_lastCountdownBeepMs = 0;
        g_countdownMidMp3Played = false;
        g_battSleepStage = BATT_SLEEP_NONE;
        g_battStageStartMs = 0;
        g_countdownExitSinceMs = 0;
        g_lastMp3BattCritMs = 0;
        return false;
      }
    } else {
      g_countdownExitSinceMs = 0;
    }

    // Durante countdown: mostra SOLO avviso
    ui_showBatteryShutdown(st.voltage_V);

    // 0012 a metà countdown (una sola volta per episodio)
    const uint32_t elapsedMs = (uint32_t)(nowMs - g_preSleepStartMs);
    if (!g_countdownMidMp3Played && elapsedMs >= PRE_SLEEP_MID_AUDIO_MS) {
      audio_requestPlayMp3(12);
      g_lastMp3BattCritMs = nowMs;
      g_countdownMidMp3Played = true;
    }

    if (g_lastCountdownBeepMs == 0 || (nowMs - g_lastCountdownBeepMs) >= BEEP_COUNTDOWN_MS) {
      buzzerWarn();
      g_lastCountdownBeepMs = nowMs;
    }

    if ((nowMs - g_preSleepStartMs) >= PRE_SLEEP_COUNTDOWN_MS) {
      // Prima di andare in sleep per batteria: schermata Zzz... 5s + 0013.mp3
      if (g_battSleepStage == BATT_SLEEP_NONE) {
        ui_renderSleepZzz();
        audio_requestPlayMp3(13);
        g_battSleepStage = BATT_SLEEP_ZZZ;
        g_battStageStartMs = nowMs;
      }
    }

    return true; // oscuriamo la pesata mentre siamo in countdown
  }

  // Entrata countdown: una volta che vai sotto 5.80, non resettare subito se rimbalzi dentro il "cuscinetto"
  // (5.80..5.85). Questo evita oscillazioni e rende l'ingresso più affidabile.
  const bool wantsCountdown = (st.voltage_V <= V_SAFE_SHUTDOWN_MIN_V) ||
                             (g_lowBattSinceMs != 0 && st.voltage_V < V_SAFE_SHUTDOWN_EXIT_V);

  if (wantsCountdown) {
    if (st.voltage_V <= V_SAFE_SHUTDOWN_MIN_V && g_lowBattSinceMs == 0) g_lowBattSinceMs = nowMs;

    // Debounce prima di entrare nella fase "stacco"
    // Nota: il timer parte al primo "vero" passaggio sotto 5.80; piccoli rimbalzi nel cuscinetto non lo annullano.
    if ((nowMs - g_lowBattSinceMs) < SAFE_SHUTDOWN_DEBOUNCE_MS) {
      return false; // ancora nessun avviso bloccante
    }

    // Avvio countdown
    if (g_preSleepStartMs == 0) {
      g_preSleepStartMs = nowMs;
      g_lastCountdownBeepMs = 0;
      g_countdownExitSinceMs = 0;
      g_countdownMidMp3Played = false;

      // 0012.mp3 = Batteria critica: suona subito all'ingresso countdown (poi di nuovo a metà).
      audio_requestPlayMp3(12);
      g_lastMp3BattCritMs = nowMs;
    }

    // Da qui in poi il countdown è gestito dal ramo "g_preSleepStartMs != 0" sopra.
    // Ma in questo giro di loop possiamo già oscurare la pesata.
    return true;
  }

  // Fuori dalla zona countdown (>= 1 tacca): disarma l'ingresso.
  if (st.voltage_V >= V_SAFE_SHUTDOWN_EXIT_V) {
    g_lowBattSinceMs = 0;
  }
  g_preSleepStartMs = 0;
  g_lastCountdownBeepMs = 0;
  g_countdownMidMp3Played = false;
  g_countdownExitSinceMs = 0;
  g_battSleepStage = BATT_SLEEP_NONE;
  g_battStageStartMs = 0;
  return false;
}

// ========================= LOOP =========================
void loop(){
#if ENABLE_WIFI_OTA || ENABLE_ARDUINO_CLOUD
  Net::update();
  wifiAudioUpdate(millis());
#endif


  // --- Comandi seriale ---
  serial_task();

// --- Cadenzamento letture + tastiera ---
  unsigned long now = millis();

  // Audio DFPlayer state-machine (non blocca la pesata)
  audio_task(now);

  // Tastiera frontale
  keypad_update(now);
  KeyCode key = keypad_get_event();
  if (key != KEY_NONE){
    g_lastKeyPressMs = now;
    if (g_inactivitySleepStage != INACT_NONE) {
      // Se l'utente preme un tasto mentre stiamo per andare in standby, annulla la sequenza.
      g_inactivitySleepStage = INACT_NONE;
      g_inactivityStageStartMs = 0;
      audio_stopNow(true);
      lastOledMs = 0; // ripristina UI normale subito
    }
    handleKeyEvent(key);
  }

  // Stato batteria + safety shutdown (indipendente dal sensore peso)
  battery_update(now);
  if (maybeEnterSafeShutdown(now)) { return; }

	

// Ogni 3 secondi stampa lo stato batteria
  if (now - lastBattDebug > 3000) {
    lastBattDebug = now;
    BatteryStatus st = battery_get_status();
    battery_debug_print(st);
  }

  
// ====================== HX711 @ 80 SPS (o 10 SPS) + decimazione ======================
// Leggiamo ogni volta che HX711 ha un dato pronto.
// - WORK (N=5): alimenta stabilità/zero-tracking e quindi "qualità" della pesata.
// - LIVE (N=2): SOLO per display in modalità LIVE (stEnable=false), con debounce.
bool haveNewWork = false;
long rawAvg = 0; // per log/debug (WORK)
long raw = 0;

// HX health: rilevazione 'RAW flatline' per beccare il caso DOUT mascherato (es. partitore/pull-down).
// Se hx711_is_ready() è true e hx711_read() riesce, ma il raw resta identico (±1 count) per >1500 ms,
// trattiamo quei campioni come NON validi per HX health (non aggiorniamo lastRawMs).
static long     s_hxRawFlatLast = 0;
static bool     s_hxRawFlatHas = false;
static uint32_t s_hxRawFlatStartMs = 0;
static bool     s_hxRawFlatLatched = false;


if (hx711_is_ready()) {
  raw = hx711_read();

  // HX health: campione RAW valido letto (hx711_is_ready + hx711_read completata)
  // + filtro 'flatline': se il raw resta inchiodato (±1 count) per >1500 ms, non aggiorniamo lastRawMs.
  bool rawOkForHealth = true;
  if (!s_hxRawFlatHas) {
    s_hxRawFlatHas = true;
    s_hxRawFlatLast = raw;
    s_hxRawFlatStartMs = 0;
    s_hxRawFlatLatched = false;
  } else {
    long d = raw - s_hxRawFlatLast;
    if (d < 0) d = -d;
    if (d <= 1) {
      if (s_hxRawFlatStartMs == 0) s_hxRawFlatStartMs = now;
      if ((uint32_t)(now - s_hxRawFlatStartMs) > 1500) {
        rawOkForHealth = false;
        // Backdate una volta sola: così le soglie WARN/ERROR contano dall'inizio della flatline.
        if (!s_hxRawFlatLatched) {
          hxHealth_backdateLastRaw(&g_hxHealth, s_hxRawFlatStartMs);
          s_hxRawFlatLatched = true;
        }
      }
    } else {
      // raw si muove: reset finestra flatline
      s_hxRawFlatStartMs = 0;
      s_hxRawFlatLatched = false;
    }
    s_hxRawFlatLast = raw;
  }

  if (rawOkForHealth) {
    hxHealth_noteRaw(&g_hxHealth, now);
  }

  // Mediana veloce su stream raw (aiuta contro spike)
  long rawMed = pushMedian3(raw);

  tareAccumSample(rawMed);

  // Accumulo per LIVE e WORK
  gLiveSum += rawMed;
  gLiveCnt++;
  gWorkSum += rawMed;
  gWorkCnt++;

  // ---- LIVE: N=2 (solo display quando stEnable=false) ----
  if (gLiveCnt >= DECIM_LIVE_N) {
    long rawLive = gLiveSum / (long)DECIM_LIVE_N;
    gLiveSum = 0;
    gLiveCnt = 0;

    long offEff = effectiveOffsetCounts();
    float gFast = (SCALE_CPG > 0.01f) ? ((float)(rawLive - offEff) / SCALE_CPG) : 0.0f;
    // LIVE (solo display): quantizzazione con isteresi.
    // Requisito: scatta a 1 solo quando tocchi |g| >= 1.0 (entry), ma riduce flicker mentre pesi.
    gDispLiveLast = displayLiveQuant(gFast, gDispLiveLast);

    // HX health: in modalità LIVE lo schermo usa gDispLiveLast.
    if (!stEnable) {
      hxHealth_noteValid(&g_hxHealth, now, (float)gDispLiveLast);
    }
  }

  // ---- WORK: N=5 (logiche stabilità) ----
  if (gWorkCnt >= DECIM_WORK_N) {
    long rawWork = gWorkSum / (long)DECIM_WORK_N;
    gWorkSum = 0;
    gWorkCnt = 0;

    lastSampleMs = now;
    rawAvg = pushMA(rawWork); // MA applicata sul flusso WORK
    haveNewWork = true;
  }
}

if (haveNewWork) {
// 3) Conversione in grammi
  long  offEff = effectiveOffsetCounts();
  gLive = (SCALE_CPG > 0.01f) ? (float)(rawAvg - offEff) / SCALE_CPG : 0.0f;

  // 4) Storico (per range/slope)
  pushHist(gLive);

  int n05    = constrain((int)ceil(500.0f  / (float)SAMPLE_MS), 1, MAX_HIST);
  int n2s    = constrain((int)ceil(2000.0f / (float)SAMPLE_MS), 1, MAX_HIST);
  int nSlope = constrain((int)ceil(800.0f  / (float)SAMPLE_MS), 2, MAX_HIST);

  float rng05      = rangeLastNSamples(n05);
  float rng2s      = rangeLastNSamples(n2s);
  float slope_gps  = slopeLastNSamples(nSlope);

  // 5) State machine STABLE/UNSTABLE + display numerico
  long  gDisp = 0;
  bool  overload = false;
  const char* modeLabel = stEnable ? (state == STABLE ? "STABLE" : "UNSTABLE") : "LIVE";

  if (!stEnable){
    // LIVE: segue gLive con deadband leggera
    if (!dispInit){
      dispUnstable = gLive;
      dispInit = true;
    }
    if (fabsf(gLive - dispUnstable) > deadbandUnstable){
      dispUnstable = gLive;
    }
    gDisp = displayLiveQuant(dispUnstable, gDispLiveLast);
  } else {
    if (state == STABLE){
      // lascia STABLE se ti discosti dal latch
      if (fabsf(gLive - gLatch) >= ST_LEAVE_DELTA_G){
        state = UNSTABLE;
        dispInit = false;
      }
      gDisp = displayWorkQuant(gLatch, gDispWorkLast);
    } else {
      // UNSTABLE: approccia il live con deadband
      if (!dispInit){
        dispUnstable = gLive;
        dispInit = true;
      }
      if (fabsf(gLive - dispUnstable) > deadbandUnstable){
        dispUnstable = gLive;
      }
      gDisp = displayWorkQuant(dispUnstable, gDispWorkLast);

      // rientro STABLE se il range in finestra è piccolo
      int   nStab   = constrain((int)ceil((float)ST_TO_STABLE_MS / (float)SAMPLE_MS), 2, MAX_HIST);
      float rngStab = rangeLastNSamples(nStab);
      if (rngStab < ST_ENTER_RANGE_G){
        state  = STABLE;
        gLatch = gLive;
        gDisp  = displayWorkQuant(gLatch, gDispWorkLast);
      }
    }
  }

  // 6) Zero-Tracking (snap + micro) se attivo
  if (ztEnable){
    // a) Snap-to-zero su scarico
    if (UNLOAD_SNAP_ENABLE){
      int   nSlopeUnload = constrain((int)ceil((float)UNLOAD_SLOPE_WIN_MS / (float)SAMPLE_MS), 2, MAX_HIST);
      float slopeUnload  = slopeLastNSamples(nSlopeUnload);
      int   nSt          = constrain((int)ceil((float)UNLOAD_STABLE_MS / (float)SAMPLE_MS), 2, MAX_HIST);
      float rngSt        = rangeLastNSamples(nSt);
      bool  quietNow     = (rngSt <= UNLOAD_STABLE_RANGE_G);
      (void)trySnapOnUnload(gLive, slopeUnload, quietNow);
    }

    // b) Micro-ZT lento vicino a 0
    if (fabsf(gLive) <= ZT_WINDOW_G && isQuietForZT()){
      if (now - lastZTtickMs >= ZT_PERIOD_MS){
        applyZTstepTowardZero(gLive);
        lastZTtickMs = now;
      }
    }
  }

  // 7) Clamp ±16 kg
  float base = (!stEnable ? dispUnstable : (state==STABLE ? gLatch : dispUnstable));
  if (fabsf(base) > MAX_DISPLAY_G){
    overload = FLAG_OVERLOAD;
    if (gDisp >  (long)MAX_DISPLAY_G) gDisp =  (long)MAX_DISPLAY_G;
    if (gDisp < -(long)MAX_DISPLAY_G) gDisp = -(long)MAX_DISPLAY_G;
  }

  // 8) Log (solo se abilitato via comando seriale)
  if (g_hxLogEnabled && (now - g_hxLogLastMs) >= g_hxLogPeriodMs) {
    g_hxLogLastMs = now;
    // 8) Log
      Serial.print(F("rawAvg="));   Serial.print(rawAvg);
      Serial.print(F("  g="));      Serial.print(gLive,2);
      Serial.print(F("  gDisp="));  Serial.print(gDisp);
      Serial.print(F("  state="));  Serial.print(modeLabel);
      Serial.print(F("  rng0.5s="));Serial.print(rng05,2);
      Serial.print(F("  slope="));  Serial.print(slope_gps,2); Serial.print(F(" g/s"));
      Serial.print(F("  rng2s="));  Serial.print(rng2s,2);
      Serial.print(F("  over="));   Serial.print(overload ? 1 : 0);
      Serial.print(F("  zt_cnt=")); Serial.print(zero_track_counts);
      Serial.print(F("  off="));    Serial.print(OFFSET_RAW);
      Serial.print(F("  sc="));     Serial.println(SCALE_CPG,4);
  }


    // Pubblico i valori WORK per OLED (modalità WORK)
    gDispWorkLast = gDisp;
    gStateLabelWorkLast = modeLabel;

    // HX health: in modalità WORK lo schermo usa gDispWorkLast.
    if (stEnable) {
      hxHealth_noteValid(&g_hxHealth, now, (float)gDispWorkLast);
    }
}


// Applica OFFSET_RAW della tara appena terminata la raccolta campioni
  tareMaybeApply(now);

  // ====================== HX health (runtime) ======================
  // Aggiorna lo stato in modo timestamp-based (no blocchi)
  hxHealth_update(&g_hxHealth, now);
  ui_setHxWarn(hxHealth_isWarn(&g_hxHealth));

  // Audio: 0015.mp3 una sola volta all'ingresso in ERROR (soft o hard)
  if (hxHealth_popEnterErrorBeep(&g_hxHealth)) {
    audio_requestPlayMp3(15);
  }


  // Inattività: consideriamo attività anche una variazione "reale" del peso visualizzato.
  // Se il peso resta in un range ±5 g, per lo standby è come se fosse fermo.
  inactivity_noteWeightActivity(now);

  // Sleep per inattività (5 min senza tasti/attività). Evita di dormire durante OTA.
  bool allowInactivitySleep = g_inactivitySleepArmed;
  // Se non è in corso una sequenza standby, evita di dormire mentre è in corso un audio "non-standby".
  if (g_inactivitySleepStage == INACT_NONE && audio_isActive()) allowInactivitySleep = false;
#if ENABLE_WIFI_OTA
  if (Net::isOtaInProgress()) allowInactivitySleep = false;
#endif

  if (!allowInactivitySleep) {
    g_inactivitySleepStage = INACT_NONE;
    g_inactivityStageStartMs = 0;
  } else {
    if (g_inactivitySleepStage == INACT_NONE) {
      if ((now - g_lastKeyPressMs) >= INACTIVITY_SLEEP_MS) {
        // Transizione risparmio energetico: mostra Zzz... per 5s e durante suona 0003.
        ui_renderSleepZzz();
        // 0003.mp3 = Entrata risparmio energetico (inattività)
        audio_requestPlayMp3(3);
        g_inactivitySleepStage = INACT_ZZZ;
        g_inactivityStageStartMs = now;
        lastOledMs = 0; // forza refresh rapido
      }
    } else { // INACT_ZZZ
      if ((now - g_inactivityStageStartMs) >= INACTIVITY_ZZZ_MS) {
        g_inactivitySleepStage = INACT_NONE;
        g_inactivityStageStartMs = 0;
        enterInactivityLightSleep();
        return;
      }
    }
  }

// ====================== OLED update cadenzato (sempre) ======================
if (now - lastOledMs >= OLED_UPDATE_MS) {
  lastOledMs = now;

  // PRIORITÀ: schermata ERROR runtime HX711 (bloccante)
  if (hxHealth_isError(&g_hxHealth)) {
    const bool hard = hxHealth_isHard(&g_hxHealth);
    // Richiesta: in schermata ERRORE non mostrare l'ultimo peso rilevato.
    ui_renderHxError(hard, false, 0);
    return; // non disegnare la UI normale
  }

  // Standby transition: schermata Zzz... per 5s (prima del light-sleep)
  if (g_inactivitySleepStage == INACT_ZZZ) {
    ui_renderSleepZzz();
  } else {
    bool drewTare = false;

    // TARA: oscura il peso e mostra "- TARA -" + barra per 1.5s
    if (gTareUiActive) {
      uint32_t elapsed = now - gTareUiStartMs;
      uint8_t pct = (elapsed >= TARE_UI_CLAMP_MS) ? 100 : (uint8_t)((elapsed * 100UL) / TARE_UI_CLAMP_MS);
      ui_renderTareProgress(pct);
      drewTare = true;

      // chiudi la UI solo dopo aver disegnato l'ultimo frame (pct=100)
      if (now >= gTareUiEndMs) {
        gTareUiActive = false;
      }
    }

    if (!drewTare) {
      if (!stEnable) {
        // LIVE: usa flusso veloce (N=2) + debounce
        ui_renderWeight(gDispLiveLast, "LIVE");
      } else {
        ui_renderWeight(gDispWorkLast, gStateLabelWorkLast);
      }
    }
  }
}


}

