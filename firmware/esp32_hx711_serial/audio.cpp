#include "audio.h"

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
}

void primeReady() {
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
}

void requestPlayMp3(uint16_t track, uint32_t capSeconds) {
  if (track < 1) track = 1;

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
  if (g_state != IDLE) return;
  if (g_qCount > 0) return;
  (void)queuePush(track, capSeconds, true);
  kickIfIdle();
}

void setVolume(uint8_t vol) {
  if (vol > 30) vol = 30;
  g_volume = vol;

  if (g_dfpUartReady) {
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
  beginRecovery(clearQueue);
}

void powerOffNow() {
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
      kickIfIdle();
      return;
  }
}

bool isActive() {
  return (g_state != IDLE) || (g_qCount > 0);
}

void queueClear() {
  g_qCount = 0;
}

void debugStatus() {
  Serial.print(F("[MP3] state="));    Serial.print((int)g_state);
  Serial.print(F(" track="));         Serial.print(g_track);
  Serial.print(F(" vol="));           Serial.print(g_volume);
  Serial.print(F(" BUSY="));          Serial.print(digitalRead(DFPLAYER_BUSY_PIN));
  Serial.print(F(" idle="));          Serial.print(g_busyIdle);
  Serial.print(F(" playLvl="));       Serial.print(g_busyPlaying);
  Serial.print(F(" polKnown="));      Serial.println(g_busyPolarityKnown ? "yes" : "no");
}

State getState() {
  return g_state;
}

uint8_t getVolume() {
  return g_volume;
}

} // namespace Audio
