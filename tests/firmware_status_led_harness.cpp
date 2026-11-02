// Compile with production status_led.cpp and tests/status_led_stubs.
#include <Arduino.h>
#include "../firmware/esp32_hx711_serial/status_led.h"

#include <cstdlib>
#include <iostream>
#include <set>

SerialStub Serial;
static bool attached = false;
static bool allowAttach = true;
static uint32_t duty = 0;
static int gpioLevel = LOW;
static unsigned int writes = 0;
static unsigned int attachCount = 0;

static void require(bool ok, const char* message) {
  if (!ok) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

void pinMode(int pin, int mode) {
  require(pin == 15 && mode == OUTPUT, "only the status LED GPIO is driven");
}

void digitalWrite(int pin, int level) {
  require(pin == 15 && !attached, "static output must not compete with PWM");
  gpioLevel = level;
  ++writes;
}

bool ledcAttachChannel(uint8_t pin, uint32_t frequency, uint8_t resolution,
                       uint8_t channel) {
  require(pin == 15 && frequency == 5000 && resolution == 8,
          "PWM parameters must suit the external LED");
  require(channel == 8, "LED must be in a separate timer group from buzzer");
  require(!attached, "PWM must only be attached once");
  ++attachCount;
  attached = allowAttach;
  return attached;
}

bool ledcDetach(uint8_t pin) {
  require(pin == 15 && attached, "detach must target the active LED channel");
  attached = false;
  return true;
}

bool ledcWrite(uint8_t pin, uint32_t value) {
  require(pin == 15 && attached && value <= 255, "PWM duty must remain bounded");
  duty = value;
  ++writes;
  return true;
}

int main() {
  StatusLed::begin();
  require(attached && duty == 0, "startup must leave the LED off");
  const unsigned int idleWrites = writes;
  StatusLed::update(10, false);
  StatusLed::update(20, false);
  require(writes == idleWrites, "idle must not continually rewrite PWM");

  StatusLed::update(100, true);
  StatusLed::update(110, true);
  require(duty > 0 && duty < 64, "voice must start with a gentle rise");
  const uint32_t firstDuty = duty;
  StatusLed::update(115, true);
  require(duty == firstDuty, "envelope updates must be rate limited");
  std::set<uint32_t> levels;
  bool bright = false;
  for (uint32_t now = 120; now < 1100; now += 10) {
    StatusLed::update(now, true);
    levels.insert(duty);
    bright |= duty > 200;
  }
  require(levels.size() > 20 && bright,
          "BUSY must produce many brightness steps, including visible peaks");
  StatusLed::update(1091, false);
  require(duty == 0, "BUSY release must switch off without waiting for throttle");

  StatusLed::update(1200, true);
  StatusLed::update(1250, true);
  StatusLed::setStandby(true);
  require(!attached && gpioLevel == HIGH, "standby must use a static GPIO");
  const unsigned int sleepWrites = writes;
  StatusLed::update(1300, true);
  StatusLed::update(1400, false);
  StatusLed::setStandby(true);
  require(writes == sleepWrites && gpioLevel == HIGH,
          "voice updates must never change the standby indicator");
  StatusLed::setStandby(false);
  require(attached && duty == 0 && attachCount == 2,
          "wake must restore PWM once with LED off");
  StatusLed::update(UINT32_MAX - 50, true);
  StatusLed::update(19, true);
  require(duty > 200, "voice timing must survive millis rollover");
  StatusLed::update(20, false);
  require(duty == 0, "stop after rollover must switch off");

  StatusLed::setStandby(true);
  allowAttach = false;
  StatusLed::setStandby(false);
  require(!attached && gpioLevel == LOW, "failed PWM allocation must start off");
  StatusLed::update(2000, true);
  StatusLed::update(2070, true);
  require(gpioLevel == HIGH, "failed PWM must retain a visible BUSY fallback");
  StatusLed::update(2071, false);
  require(gpioLevel == LOW, "BUSY fallback must stop immediately");
  StatusLed::setStandby(true);
  require(gpioLevel == HIGH, "standby must also work without PWM");

  std::cout << "Status LED: startup, voice, stop, standby/wake, rollover and PWM failure passed\n";
}
