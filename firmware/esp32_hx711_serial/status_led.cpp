#include "status_led.h"

#include <Arduino.h>
#include "config/config_led.h"
#include "config/config_pins.h"

namespace StatusLed {

static bool g_pwmAttached = false;
static bool g_standby = false;
static bool g_voiceActive = false;
static uint8_t g_pulse = 0;
static uint32_t g_pulseStartMs = 0;
static uint32_t g_lastUpdateMs = 0;

static void writeBrightness(uint8_t brightness) {
  if (g_pwmAttached) {
    ledcWrite(SLEEP_LED_PIN, SLEEP_LED_ACTIVE_HIGH ? brightness : 255 - brightness);
  } else {
    // Keep a visible BUSY indication even if PWM could not be allocated.
    const bool on = brightness >= 128;
    digitalWrite(SLEEP_LED_PIN, on == SLEEP_LED_ACTIVE_HIGH ? HIGH : LOW);
  }
}

static void attachPwm() {
  g_pwmAttached = ledcAttachChannel(SLEEP_LED_PIN, LedConfig::PWM_HZ,
                                  LedConfig::PWM_BITS, LedConfig::PWM_CHANNEL);
  if (!g_pwmAttached) {
    pinMode(SLEEP_LED_PIN, OUTPUT);
    Serial.println(F("[LED] PWM unavailable"));
  }
  writeBrightness(0);
}

void begin() {
  g_standby = false;
  g_voiceActive = false;
  pinMode(SLEEP_LED_PIN, OUTPUT);
  digitalWrite(SLEEP_LED_PIN, SLEEP_LED_ACTIVE_HIGH ? LOW : HIGH);
  attachPwm();
}

void setStandby(bool standby) {
  if (standby == g_standby) return;
  g_standby = standby;
  g_voiceActive = false;
  if (standby) {
    if (g_pwmAttached) {
      ledcDetach(SLEEP_LED_PIN);
      g_pwmAttached = false;
    }
    pinMode(SLEEP_LED_PIN, OUTPUT);
    digitalWrite(SLEEP_LED_PIN, SLEEP_LED_ACTIVE_HIGH ? HIGH : LOW);
  } else {
    digitalWrite(SLEEP_LED_PIN, SLEEP_LED_ACTIVE_HIGH ? LOW : HIGH);
    attachPwm();
  }
}

void update(uint32_t nowMs, bool playerBusy) {
  if (g_standby) return;
  if (!playerBusy) {
    if (g_voiceActive) writeBrightness(0);
    g_voiceActive = false;
    return;
  }

  if (!g_voiceActive) {
    g_voiceActive = true;
    g_pulse = 0;
    g_pulseStartMs = nowMs;
  } else if ((uint32_t)(nowMs - g_lastUpdateMs) < LedConfig::UPDATE_MS) {
    return;
  }
  g_lastUpdateMs = nowMs;

  uint32_t elapsed = nowMs - g_pulseStartMs;
  uint16_t duration = LedConfig::VOICE_PULSE_MS[g_pulse];
  if (elapsed >= duration) {
    g_pulse = (uint8_t)((g_pulse + 1) %
      (sizeof(LedConfig::VOICE_PULSE_MS) / sizeof(LedConfig::VOICE_PULSE_MS[0])));
    g_pulseStartMs = nowMs;
    elapsed = 0;
    duration = LedConfig::VOICE_PULSE_MS[g_pulse];
  }

  const uint16_t riseMs = duration / 3;
  const uint32_t peak = LedConfig::VOICE_PEAK[g_pulse];
  const uint32_t level = elapsed < riseMs
    ? peak * elapsed / riseMs
    : peak * (duration - elapsed) / (duration - riseMs);
  // A square curve gives the low end more useful steps for a gentler fade.
  writeBrightness((uint8_t)(level * level / 255));
}

} // namespace StatusLed
