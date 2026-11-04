#pragma once

#include <Arduino.h>
#include "remote_console_state.h"

#if defined(ARDUINO_ARCH_ESP32)
namespace RemoteConsole {

class LogOutput : public Print {
 public:
  using Print::write;
  size_t write(uint8_t value) override;
  size_t write(const uint8_t* buffer, size_t length) override;
};

void begin(const char* bootId);
void tick(uint32_t nowMs);
void receive(const uint8_t* payload, size_t length);
void publishPending();
void transportLost();
bool active();
bool popClosed();
bool takeCommand(char* output, size_t capacity);
void completeCommand(const char* error = "");
bool executingCommand();
// Sensitive USB-only output and raw invalid input bypass this tee entirely.
void capture(const uint8_t* buffer, size_t length);

}  // namespace RemoteConsole

extern RemoteConsole::LogOutput ConsoleLog;
#else
// Existing host harnesses deliberately use a minimal Arduino serial stub.
#define ConsoleLog Serial
#endif
