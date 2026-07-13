#include "audio.h"
#include <Preferences.h>

namespace Audio {

// ========================= STATO INTERNO =========================
static State     g_state      = IDLE;
static uint16_t  g_track      = 0;
static uint32_t  g_stateMs    = 0;
static uint32_t  g_startMs    = 0;
static uint32_t  g_timeoutMs  = 0;
static uint32_t  g_lastStopMs = 0;
static uint16_t  g_gapTrack   = 0;
static uint32_t  g_gapCapSec  = 0;
static uint32_t  g_gapUntilMs = 0;
static int       g_busyIdle   = -1;
static int       g_busyPlaying = -1;
static bool      g_busyPolarityKnown = false;
static uint32_t  g_busyIdleSinceMs   = 0;

// Stato alimentazione DFPlayer
static bool     g_dfpPowered   = false;
static bool     g_dfpUartReady = false;
static bool     g_userEnabled  = true;
static uint8_t  g_volume       = AudioConfig::DEFAULT_VOLUME;

// Coda FIFO
static uint16_t g_qTrack[AudioConfig::QUEUE_MAX_SIZE];
static uint32_t g_qCapSec[AudioConfig::QUEUE_MAX_SIZE];
static bool     g_qLow[AudioConfig::QUEUE_MAX_SIZE];
static uint8_t  g_qCount = 0;

// Anti-retrigger
static uint16_t g_lastReqTrack = 0;
static uint32_t g_lastReqMs    = 0;

// ========================= HELPER INTERNI =========================

static const DFPlayer::Pins g_pins = {
  .tx = DFPLAYER_TX_PIN,
  .rx = DFPLAYER_RX_PIN
};

static inline void dfpPowerSet(bool on) {
  digitalWrite(DFPLAYER_EN_PIN, on ? HIGH : LOW);
}

static int dfpBusyReadStable() {
  return (digitalRead(DFPLAYER_BUSY_PIN) != 0) ? 1 : 0;
}

// ========================= FLIGHT RECORDER =========================

enum DiagCode : uint8_t {
  DIAG_BEGIN = 1,
  DIAG_REQUEST,
  DIAG_PLAY,
  DIAG_NO_BUSY_EDGE,
  DIAG_TIMEOUT,
  DIAG_RESET,
  DIAG_READY,
  DIAG_POWER_OFF,
  DIAG_MANUAL_OFF,
  DIAG_MANUAL_ON,
  DIAG_DROP_DISABLED
};

struct DiagEntry {
  uint32_t ms;
  uint16_t track;
  uint8_t  code;
  uint8_t  state;
  int8_t   busy;
  uint8_t  en;
};

struct __attribute__((packed)) SavedSnapshot {
  uint16_t magic;
  uint8_t  version;
  uint8_t  reason;
  uint8_t  state;
  int8_t   busy;
  uint8_t  en;
  uint8_t  enabled;
  uint16_t track;
  uint32_t ms;
};

static const uint8_t  DIAG_MAX = 24;
static const uint16_t SNAPSHOT_MAGIC = 0xA7D1;
static DiagEntry g_diag[DIAG_MAX];
static uint8_t g_diagCount = 0;
static uint8_t g_diagNext  = 0;
static SavedSnapshot g_savedSnapshot = {};
static bool g_savedChecked = false;
static bool g_savedValid   = false;

static void printDiagCode(uint8_t code) {
  switch (code) {
    case DIAG_BEGIN:         Serial.print(F("BEGIN")); break;
    case DIAG_REQUEST:       Serial.print(F("REQ")); break;
    case DIAG_PLAY:          Serial.print(F("PLAY")); break;
    case DIAG_NO_BUSY_EDGE:  Serial.print(F("NO_BUSY")); break;
    case DIAG_TIMEOUT:       Serial.print(F("TIMEOUT")); break;
    case DIAG_RESET:         Serial.print(F("RESET")); break;
    case DIAG_READY:         Serial.print(F("READY")); break;
    case DIAG_POWER_OFF:     Serial.print(F("PWR_OFF")); break;
    case DIAG_MANUAL_OFF:    Serial.print(F("MAN_OFF")); break;
    case DIAG_MANUAL_ON:     Serial.print(F("MAN_ON")); break;
    case DIAG_DROP_DISABLED: Serial.print(F("DROP_OFF")); break;
    default:                 Serial.print('?'); break;
  }
}

static void diagRecord(uint8_t code, uint16_t track = 0) {
  DiagEntry& e = g_diag[g_diagNext];
  e.ms    = millis();
  e.track = (track != 0) ? track : g_track;
  e.code  = code;
  e.state = (uint8_t)g_state;
  e.busy  = (int8_t)dfpBusyReadStable();
  e.en    = (uint8_t)(digitalRead(DFPLAYER_EN_PIN) != 0);

  g_diagNext = (uint8_t)((g_diagNext + 1) % DIAG_MAX);
  if (g_diagCount < DIAG_MAX) g_diagCount++;
}

static void diagLoadSaved() {
  if (g_savedChecked) return;
  g_savedChecked = true;

  Preferences p;
  if (!p.begin("minu_audio", true)) return;
  if (p.getBytesLength("last") == sizeof(g_savedSnapshot)) {
    p.getBytes("last", &g_savedSnapshot, sizeof(g_savedSnapshot));
    g_savedValid = g_savedSnapshot.magic == SNAPSHOT_MAGIC &&
                   g_savedSnapshot.version == 1;
  }
  p.end();
}

static void diagPersist(uint8_t reason) {
  SavedSnapshot snap = {
    .magic = SNAPSHOT_MAGIC,
    .version = 1,
    .reason = reason,
    .state = (uint8_t)g_state,
    .busy = (int8_t)dfpBusyReadStable(),
    .en = (uint8_t)(digitalRead(DFPLAYER_EN_PIN) != 0),
    .enabled = (uint8_t)g_userEnabled,
    .track = g_track,
    .ms = millis()
  };

  Preferences p;
  if (!p.begin("minu_audio", false)) return;
  bool ok = p.putBytes("last", &snap, sizeof(snap)) == sizeof(snap);
  p.end();
  if (ok) {
    g_savedSnapshot = snap;
    g_savedChecked = true;
    g_savedValid = true;
  }
}

static bool queuePush(uint16_t track, uint32_t capSec, bool low) {
  if (track < 1) track = 1;

  if (g_qCount >= AudioConfig::QUEUE_MAX_SIZE) {
    if (low) return false;

    if (AudioConfig::isMustPlayTrack(track)) {
      for (int i = (int)g_qCount - 1; i >= 0; i--) {
        if (g_qLow[i]) {
          for (int j = i; j < (int)g_qCount - 1; j++) {
            g_qTrack[j]  = g_qTrack[j + 1];
            g_qCapSec[j] = g_qCapSec[j + 1];
            g_qLow[j]    = g_qLow[j + 1];
          }
          g_qCount--;
          break;
        }
      }
      if (g_qCount >= AudioConfig::QUEUE_MAX_SIZE) {
        g_qCount--;
      }
    } else {
      return false;
    }
  }

  g_qTrack[g_qCount]  = track;
  g_qCapSec[g_qCount] = capSec;
  g_qLow[g_qCount]    = low;
  g_qCount++;
  return true;
}

static bool queuePop(uint16_t* outTrack, uint32_t* outCapSec, bool* outLow) {
  if (g_qCount == 0) return false;
  if (outTrack)   *outTrack   = g_qTrack[0];
  if (outCapSec)  *outCapSec  = g_qCapSec[0];
  if (outLow)     *outLow     = g_qLow[0];
  for (uint8_t i = 0; i + 1 < g_qCount; i++) {
    g_qTrack[i]  = g_qTrack[i + 1];
    g_qCapSec[i] = g_qCapSec[i + 1];
    g_qLow[i]    = g_qLow[i + 1];
  }
  g_qCount--;
  return true;
}

static void startNow(uint16_t track, uint32_t capSeconds);
static void beginRecovery(bool clearQueue);

static void kickIfIdle() {
  if (g_state != IDLE) return;
  uint16_t t = 0;
  uint32_t cap = 0;
  bool low = false;
  if (queuePop(&t, &cap, &low)) {
    (void)low;
    startNow(t, cap);
  }
}

static inline bool isRecovering() {
  return (g_state == RECOVERING_OFF) || (g_state == RECOVERING_ON);
}

static void beginRecovery(bool clearQueue) {
  diagRecord(DIAG_RESET);

  if (clearQueue) {
    queueClear();
  }

  if (g_dfpUartReady) {
    DFPlayer::end();
    g_dfpUartReady = false;
  }

  pinMode(g_pins.tx, INPUT);
  if (g_pins.rx >= 0) {
    pinMode(g_pins.rx, INPUT);
  }

  dfpPowerSet(false);
  g_dfpPowered = false;

  g_gapTrack   = 0;
  g_gapCapSec  = 0;
  g_gapUntilMs = 0;

  g_track     = 0;
  g_startMs   = 0;
  g_timeoutMs = 0;

  g_busyIdle          = -1;
  g_busyPlaying       = -1;
  g_busyPolarityKnown = false;
  g_busyIdleSinceMs   = 0;

  g_state   = RECOVERING_OFF;
  g_stateMs = millis();
}

static void startNow(uint16_t track, uint32_t capSeconds) {
  if (track < 1) track = 1;

  g_track = track;
  g_timeoutMs = (capSeconds > 0) ? (capSeconds * 1000UL) : AudioConfig::PLAY_TIMEOUT_MS;
  if (g_timeoutMs < 3000) g_timeoutMs = 3000;

  g_busyPolarityKnown = false;
  g_busyIdleSinceMs = 0;

  Serial.print(F("[MP3] play /MP3/"));
  if (track < 10)   Serial.print('0');
  if (track < 100)  Serial.print('0');
  if (track < 1000) Serial.print('0');
  Serial.print(track);
  Serial.println(F(".mp3"));
  diagRecord(DIAG_PLAY, track);

  const uint32_t now = millis();

  // Anti-troncamento
  if (g_lastStopMs != 0 && (now - g_lastStopMs) < AudioConfig::RESTART_GAP_MS) {
    g_gapTrack   = track;
    g_gapCapSec  = capSeconds;
    g_gapUntilMs = g_lastStopMs + AudioConfig::RESTART_GAP_MS;
    g_state      = GAP;
    g_stateMs    = now;
    return;
  }

  // Se già pronto, play diretto
  if (g_dfpPowered && g_dfpUartReady) {
    g_busyIdle = dfpBusyReadStable();
    DFPlayer::playMp3(g_track);
    g_startMs  = now;
    g_state    = STARTING;
    g_stateMs  = now;
    return;
  }

  // Cold-start
  g_state   = POWERING;
  g_stateMs = now;
  g_startMs = 0;

  dfpPowerSet(true);
  g_dfpPowered = true;

  pinMode(g_pins.tx, OUTPUT);
  digitalWrite(g_pins.tx, LOW);
}

// ========================= API PUBBLICHE =========================

void begin() {
  pinMode(DFPLAYER_EN_PIN, OUTPUT);
  dfpPowerSet(false);
  g_dfpPowered   = false;
  g_dfpUartReady = false;

  pinMode(DFPLAYER_BUSY_PIN, INPUT);

  pinMode(g_pins.tx, INPUT);
  if (g_pins.rx >= 0) {
    pinMode(g_pins.rx, INPUT);
  }

  DFPlayer::end();

  g_gapTrack   = 0;
  g_gapCapSec  = 0;
  g_gapUntilMs = 0;

  g_state             = IDLE;
  g_busyIdle          = -1;
  g_busyPlaying       = -1;
  g_busyPolarityKnown = false;
  g_busyIdleSinceMs   = 0;

  diagLoadSaved();
  diagRecord(DIAG_BEGIN);
}

void primeReady() {
  if (!g_userEnabled) return;
  if (g_dfpPowered && g_dfpUartReady) return;

  if (!g_dfpPowered) {
    dfpPowerSet(true);
    g_dfpPowered = true;

    pinMode(g_pins.tx, OUTPUT);
    digitalWrite(g_pins.tx, LOW);

    delay(AudioConfig::POWERUP_MS);
  } else {
    delay(20);
  }

  DFPlayer::restoreAfterEspWake(g_pins, g_volume);
  g_dfpUartReady = true;
  delay(20);

  g_busyIdle = dfpBusyReadStable();
  diagRecord(DIAG_READY);
}

void requestPlayMp3(uint16_t track, uint32_t capSeconds) {
  if (track < 1) track = 1;

  if (!g_userEnabled) {
    diagRecord(DIAG_DROP_DISABLED, track);
    return;
  }
  diagRecord(DIAG_REQUEST, track);

  // Anti-retrigger
  const uint32_t nowReq = millis();
  if (track == g_lastReqTrack && (uint32_t)(nowReq - g_lastReqMs) < AudioConfig::REQUEST_GUARD_MS) {
    return;
  }
  g_lastReqTrack = track;
  g_lastReqMs    = nowReq;

  const bool mustPlay    = AudioConfig::isMustPlayTrack(track);
  if (isRecovering()) {
    (void)queuePush(track, capSeconds, !mustPlay);
    return;
  }

  const bool playing     = (g_state != IDLE);
  const bool curMustPlay = playing && AudioConfig::isMustPlayTrack(g_track);

  if (mustPlay) {
    if (playing && !curMustPlay) {
      stopNow(false);
    }

    if (g_state == IDLE && g_qCount == 0) {
      startNow(track, capSeconds);
      return;
    }

    (void)queuePush(track, capSeconds, false);
    kickIfIdle();
    return;
  }

  // Interrompibile
  if (curMustPlay) return;

  if (g_qCount > 0) {
    kickIfIdle();
    return;
  }

  if (playing) {
    stopNow(false);
  }
  startNow(track, capSeconds);
}

void requestPlayMp3Low(uint16_t track, uint32_t capSeconds) {
  if (!g_userEnabled) {
    diagRecord(DIAG_DROP_DISABLED, track);
    return;
  }
  if (g_state != IDLE) return;
  if (g_qCount > 0) return;
  (void)queuePush(track, capSeconds, true);
  kickIfIdle();
}

void setVolume(uint8_t vol) {
  if (vol > 30) vol = 30;
  g_volume = vol;

  if (g_userEnabled && g_dfpUartReady) {
    DFPlayer::setVolume(vol);
  }
}

void stopNow(bool clearQueue) {
  const bool wasTailing = (g_state == TAILING);

  if (g_dfpUartReady && !wasTailing) {
    DFPlayer::stop();
    g_lastStopMs = millis();
  }

  if (clearQueue) {
    queueClear();
  }

  g_gapTrack   = 0;
  g_gapCapSec  = 0;
  g_gapUntilMs = 0;

  g_state             = IDLE;
  g_busyIdle          = -1;
  g_busyPlaying       = -1;
  g_busyPolarityKnown = false;
  g_busyIdleSinceMs   = 0;
}

void hardReset(bool clearQueue) {
  Serial.println(F("[MP3] reset"));
  g_userEnabled = true;
  beginRecovery(clearQueue);
}

bool toggleEnabled() {
  if (g_userEnabled) {
    diagRecord(DIAG_MANUAL_OFF);
    diagPersist(DIAG_MANUAL_OFF);
    g_userEnabled = false;
    powerOffNow();
    Serial.println(F("[MP3] manual OFF; log salvato"));
  } else {
    g_userEnabled = true;
    diagRecord(DIAG_MANUAL_ON);
    beginRecovery(true);
    Serial.println(F("[MP3] manual ON"));
  }
  return g_userEnabled;
}

bool isEnabled() {
  return g_userEnabled;
}

bool isReady() {
  return g_userEnabled && g_dfpPowered && g_dfpUartReady && !isRecovering();
}

void powerOffNow() {
  diagRecord(DIAG_POWER_OFF);
  queueClear();

  if (g_dfpUartReady) {
    DFPlayer::stop();
    delay(50);
    DFPlayer::end();
    g_dfpUartReady = false;
  }

  pinMode(g_pins.tx, INPUT);
  if (g_pins.rx >= 0) pinMode(g_pins.rx, INPUT);

  dfpPowerSet(false);
  g_dfpPowered = false;

  g_state             = IDLE;
  g_busyIdle          = -1;
  g_busyPlaying       = -1;
  g_busyPolarityKnown = false;
  g_busyIdleSinceMs   = 0;
}

void task(uint32_t now) {
  if (!g_userEnabled) return;

  switch (g_state) {
    case IDLE:
      kickIfIdle();
      return;

    case GAP:
      if (now < g_gapUntilMs) return;
      g_state = IDLE;
      startNow(g_gapTrack, g_gapCapSec);
      return;

    case POWERING:
      if ((now - g_stateMs) < AudioConfig::POWERUP_MS) return;

      DFPlayer::restoreAfterEspWake(g_pins, g_volume);
      g_dfpUartReady = true;
      delay(20);

      g_busyIdle = dfpBusyReadStable();

      DFPlayer::playMp3(g_track);
      g_startMs = now;

      g_state   = STARTING;
      g_stateMs = now;
      return;

    case STARTING: {
      int b = dfpBusyReadStable();

      if (b != g_busyIdle) {
        g_busyPlaying       = b;
        g_busyPolarityKnown = true;
        g_state             = PLAYING;
        g_stateMs           = now;
        return;
      }

      if ((now - g_stateMs) >= AudioConfig::BUSY_DETECT_MS) {
        diagRecord(DIAG_NO_BUSY_EDGE);
        g_busyPlaying       = (g_busyIdle == 0) ? 1 : 0;
        g_busyPolarityKnown = false;
        g_state             = PLAYING;
        g_stateMs           = now;
        return;
      }
      return;
    }

    case PLAYING: {
      // Timeout paracadute
      if (g_startMs != 0 && (now - g_startMs) >= g_timeoutMs) {
        Serial.println(F("[MP3] timeout reset"));
        diagRecord(DIAG_TIMEOUT);
        diagPersist(DIAG_TIMEOUT);
        beginRecovery(true);
        return;
      }

      int b = dfpBusyReadStable();

      if (b == g_busyIdle) {
        if (g_startMs != 0 && (now - g_startMs) < AudioConfig::MIN_PLAY_MS) {
          g_busyIdleSinceMs = 0;
          return;
        }
        if (g_busyIdleSinceMs == 0) g_busyIdleSinceMs = now;
        if ((now - g_busyIdleSinceMs) >= AudioConfig::END_STABLE_MS) {
          Serial.println(F("[MP3] fine"));
          DFPlayer::stop();
          g_lastStopMs = now;
          g_state      = TAILING;
          g_stateMs    = now;
        }
      } else {
        g_busyIdleSinceMs = 0;
      }
      return;
    }

    case TAILING:
      if ((now - g_stateMs) < AudioConfig::TAIL_OFF_MS) return;
      stopNow();
      return;

    case RECOVERING_OFF:
      if ((now - g_stateMs) < AudioConfig::RECOVERY_OFF_MS) return;

      dfpPowerSet(true);
      g_dfpPowered = true;

      pinMode(g_pins.tx, OUTPUT);
      digitalWrite(g_pins.tx, LOW);

      g_state   = RECOVERING_ON;
      g_stateMs = now;
      return;

    case RECOVERING_ON:
      if ((now - g_stateMs) < AudioConfig::POWERUP_MS) return;

      DFPlayer::restoreAfterEspWake(g_pins, g_volume);
      g_dfpUartReady = true;
      delay(20);

      g_busyIdle = dfpBusyReadStable();
      g_state    = IDLE;
      g_stateMs  = now;
      diagRecord(DIAG_READY);
      kickIfIdle();
      return;
  }
}

bool isActive() {
  return g_userEnabled && ((g_state != IDLE) || (g_qCount > 0));
}

void queueClear() {
  g_qCount = 0;
}

void debugStatus() {
  Serial.print(F("[MP3] state="));    Serial.print((int)g_state);
  Serial.print(F(" track="));         Serial.print(g_track);
  Serial.print(F(" vol="));           Serial.print(g_volume);
  Serial.print(F(" enabled="));       Serial.print(g_userEnabled ? 1 : 0);
  Serial.print(F(" pwr="));           Serial.print(g_dfpPowered ? 1 : 0);
  Serial.print(F(" uart="));          Serial.print(g_dfpUartReady ? 1 : 0);
  Serial.print(F(" EN="));            Serial.print(digitalRead(DFPLAYER_EN_PIN));
  Serial.print(F(" BUSY="));          Serial.print(digitalRead(DFPLAYER_BUSY_PIN));
  Serial.print(F(" idle="));          Serial.print(g_busyIdle);
  Serial.print(F(" playLvl="));       Serial.print(g_busyPlaying);
  Serial.print(F(" polKnown="));      Serial.println(g_busyPolarityKnown ? "yes" : "no");
}

void printHistory() {
  diagLoadSaved();

  if (g_savedValid) {
    Serial.print(F("[MP3LOG] saved ms=")); Serial.print(g_savedSnapshot.ms);
    Serial.print(F(" ev=")); printDiagCode(g_savedSnapshot.reason);
    Serial.print(F(" st=")); Serial.print(g_savedSnapshot.state);
    Serial.print(F(" tr=")); Serial.print(g_savedSnapshot.track);
    Serial.print(F(" busy=")); Serial.print(g_savedSnapshot.busy);
    Serial.print(F(" en=")); Serial.print(g_savedSnapshot.en);
    Serial.print(F(" enabled=")); Serial.println(g_savedSnapshot.enabled);
  } else {
    Serial.println(F("[MP3LOG] saved none"));
  }

  Serial.print(F("[MP3LOG] ram count="));
  Serial.println(g_diagCount);
  uint8_t first = (g_diagCount < DIAG_MAX) ? 0 : g_diagNext;
  for (uint8_t i = 0; i < g_diagCount; i++) {
    const DiagEntry& e = g_diag[(uint8_t)((first + i) % DIAG_MAX)];
    Serial.print(F("[MP3LOG] ms=")); Serial.print(e.ms);
    Serial.print(F(" ev=")); printDiagCode(e.code);
    Serial.print(F(" st=")); Serial.print(e.state);
    Serial.print(F(" tr=")); Serial.print(e.track);
    Serial.print(F(" busy=")); Serial.print(e.busy);
    Serial.print(F(" en=")); Serial.println(e.en);
  }
}

void clearHistory() {
  g_diagCount = 0;
  g_diagNext = 0;
  memset(g_diag, 0, sizeof(g_diag));
  memset(&g_savedSnapshot, 0, sizeof(g_savedSnapshot));
  g_savedChecked = true;
  g_savedValid = false;

  Preferences p;
  if (p.begin("minu_audio", false)) {
    p.remove("last");
    p.end();
  }
  Serial.println(F("[MP3LOG] cleared"));
}

bool hasSavedDiagnostic() {
  diagLoadSaved();
  return g_savedValid;
}

State getState() {
  return g_state;
}

uint8_t getVolume() {
  return g_volume;
}

} // namespace Audio
