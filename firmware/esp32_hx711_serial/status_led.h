#pragma once

#include <stdint.h>

namespace StatusLed {

void begin();
void update(uint32_t nowMs, bool playerBusy);

// Detach PWM and drive a static GPIO for light-sleep, then restore PWM on wake.
void setStandby(bool standby);

} // namespace StatusLed
