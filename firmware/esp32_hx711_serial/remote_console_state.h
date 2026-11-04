#pragma once

#include <stddef.h>
#include <stdint.h>

namespace RemoteConsoleCore {

constexpr size_t ID_SIZE = 65;
constexpr size_t COMMAND_SIZE = 128;
constexpr size_t LOG_SIZE = 2048;
constexpr uint32_t LEASE_MS = 30000;

enum class Action : uint8_t { Probe, Open, Poll, Exec, Close, Invalid };
enum class Outcome : uint8_t { Ok, Done, Busy, Rejected };

struct Request {
  Action action = Action::Invalid;
  char requestId[ID_SIZE] = {};
  char sessionId[ID_SIZE] = {};
  char bootId[9] = {};
  uint32_t expiresAt = 0;
  uint32_t cursor = 0;
  uint32_t sequence = 0;
  char command[COMMAND_SIZE] = {};
};

struct Result {
  Outcome outcome = Outcome::Rejected;
  const char* error = "invalid_request";
  bool queued = false;
  bool opened = false;
};

// This state is called only by the loop task. Ring access is protected by its
// Arduino adapter because network-worker diagnostics can append concurrently.
class Session {
 public:
  void reset(const char* bootId);
  Result receive(const Request& request, uint32_t nowMs, uint32_t nowEpoch);
  bool tick(uint32_t nowMs);
  bool active() const { return sessionId_[0] != '\0'; }
  bool pending() const { return pending_; }
  const char* sessionId() const { return sessionId_; }
  const char* bootId() const { return bootId_; }
  const char* command() const { return command_; }
  void complete(const char* error = "");
  void close(uint32_t nowMs);

 private:
  char bootId_[9] = {};
  char sessionId_[ID_SIZE] = {};
  char requestId_[ID_SIZE] = {};
  char command_[COMMAND_SIZE] = {};
  uint32_t leaseStartedMs_ = 0;
  uint32_t sequence_ = 0;
  bool pending_ = false;
  const char* lastError_ = "";
  char closedIds_[4][ID_SIZE] = {};
  uint32_t closedAtMs_[4] = {};
  void clearActive();
};

class LogRing {
 public:
  void clear();
  void append(const uint8_t* bytes, size_t length);
  size_t read(uint32_t cursor, char* output, size_t capacity,
              uint32_t& next, uint32_t& oldest, uint32_t& dropped) const;
  uint32_t cursor() const { return cursor_; }

 private:
  uint8_t bytes_[LOG_SIZE] = {};
  uint32_t cursor_ = 0;
  size_t length_ = 0;
};

bool commandIsSafe(const char* command);

}  // namespace RemoteConsoleCore
