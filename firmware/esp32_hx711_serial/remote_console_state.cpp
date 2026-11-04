#include "remote_console_state.h"

#include <string.h>

namespace RemoteConsoleCore {

static bool validId(const char* value) {
  if (!value || !*value) return false;
  for (size_t i = 0; value[i]; ++i) {
    const char c = value[i];
    if (i >= ID_SIZE - 1 || !((c >= 'a' && c <= 'z') ||
        (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
        c == '-' || c == '_')) return false;
  }
  return true;
}

bool commandIsSafe(const char* command) {
  if (!command || !*command || strlen(command) >= COMMAND_SIZE) return false;
  // Match the credential-display branches without banning the same text in a
  // newly configured password or SSID. Raw invalid input is never mirrored.
  if ((strncmp(command, "wifi creds", 10) == 0 ||
       strncmp(command, "mqtt creds", 10) == 0) && strstr(command, "showpass")) return false;
  for (const char* p = command; *p; ++p) {
    if (*p == '\r' || *p == '\n') return false;
  }
  return true;
}

void Session::reset(const char* bootId) {
  clearActive();
  memset(closedIds_, 0, sizeof(closedIds_));
  strncpy(bootId_, bootId ? bootId : "", sizeof(bootId_) - 1);
  bootId_[sizeof(bootId_) - 1] = '\0';
}

void Session::clearActive() {
  sessionId_[0] = '\0';
  requestId_[0] = '\0';
  command_[0] = '\0';
  sequence_ = 0;
  pending_ = false;
  lastError_ = "";
}

void Session::close(uint32_t nowMs) {
  if (active()) {
    for (size_t i = 0; i < 4; ++i) {
      if (!closedIds_[i][0]) {
        strcpy(closedIds_[i], sessionId_);
        closedAtMs_[i] = nowMs;
        break;
      }
    }
  }
  clearActive();
}

bool Session::tick(uint32_t nowMs) {
  for (size_t i = 0; i < 4; ++i) {
    if (closedIds_[i][0] && (uint32_t)(nowMs - closedAtMs_[i]) >= LEASE_MS + 1000)
      closedIds_[i][0] = '\0';
  }
  if (!active() || (uint32_t)(nowMs - leaseStartedMs_) < LEASE_MS) return false;
  close(nowMs);
  return true;
}

Result Session::receive(const Request& request, uint32_t nowMs, uint32_t nowEpoch) {
  tick(nowMs);
  if (!validId(request.requestId) || request.action == Action::Invalid ||
      (request.action != Action::Probe && !validId(request.sessionId))) return {};
  if (nowEpoch < 1700000000UL) return {Outcome::Rejected, "clock_unsynced"};
  if (request.expiresAt <= nowEpoch) return {Outcome::Rejected, "expired"};
  if (request.expiresAt - nowEpoch > 30) return {Outcome::Rejected, "invalid_expiry"};
  if ((request.action != Action::Probe || request.bootId[0]) &&
      strcmp(request.bootId, bootId_) != 0) return {Outcome::Rejected, "boot_changed"};
  if (request.action == Action::Probe) return {Outcome::Ok, ""};
  if (request.action == Action::Open) {
    if (active() && strcmp(sessionId_, request.sessionId) != 0)
      return {Outcome::Busy, "session_busy"};
    bool opened = !active();
    if (opened) {
      bool slotAvailable = false;
      for (size_t i = 0; i < 4; ++i) {
        if (strcmp(closedIds_[i], request.sessionId) == 0)
          return {Outcome::Rejected, "session_closed"};
        if (!closedIds_[i][0]) slotAvailable = true;
      }
      // Never evict an unexpired replay fence just to accept another open.
      if (!slotAvailable) return {Outcome::Busy, "session_busy"};
      clearActive();
      strcpy(sessionId_, request.sessionId);
    }
    leaseStartedMs_ = nowMs;
    return {Outcome::Ok, "", false, opened};
  }
  if (!active() || strcmp(sessionId_, request.sessionId) != 0)
    return {Outcome::Rejected, "session_closed"};
  if (request.action == Action::Poll) {
    leaseStartedMs_ = nowMs;
    return {Outcome::Ok, ""};
  }
  if (request.action == Action::Close) {
    close(nowMs);
    return {Outcome::Ok, ""};
  }
  if (request.action != Action::Exec) return {};
  if (request.sequence == sequence_ && sequence_ != 0) {
    if (strcmp(requestId_, request.requestId) != 0 ||
        strcmp(command_, request.command) != 0)
      return {Outcome::Rejected, "sequence_conflict"};
    return pending_ ? Result{Outcome::Busy, "executing"}
                    : Result{lastError_[0] ? Outcome::Rejected : Outcome::Done, lastError_};
  }
  if (request.sequence == 0 || request.sequence != sequence_ + 1 ||
      strcmp(requestId_, request.requestId) == 0)
    return {Outcome::Rejected, "sequence_conflict"};
  if (pending_) return {Outcome::Busy, "executing"};
  sequence_ = request.sequence;
  strcpy(requestId_, request.requestId);
  strcpy(command_, request.command);
  pending_ = true;
  lastError_ = "";
  if (!commandIsSafe(request.command)) {
    complete("command_forbidden");
    return {Outcome::Rejected, lastError_};
  }
  return {Outcome::Busy, "", true};
}

void Session::complete(const char* error) {
  pending_ = false;
  lastError_ = error ? error : "";
}

void LogRing::clear() {
  cursor_ = 0;
  length_ = 0;
}

void LogRing::append(const uint8_t* bytes, size_t length) {
  if (!bytes) return;
  for (size_t i = 0; i < length; ++i) {
    bytes_[cursor_ % LOG_SIZE] = bytes[i];
    ++cursor_;
    if (length_ < LOG_SIZE) ++length_;
  }
}

size_t LogRing::read(uint32_t cursor, char* output, size_t capacity,
                     uint32_t& next, uint32_t& oldest, uint32_t& dropped) const {
  oldest = cursor_ - (uint32_t)length_;
  dropped = 0;
  uint32_t available = cursor_ - cursor;
  if (available > length_) {
    // A future cursor is invalid; an old cursor has fallen out of the ring.
    if ((int32_t)(cursor - cursor_) > 0) cursor = cursor_;
    else { dropped = oldest - cursor; cursor = oldest; }
  }
  if (!output || capacity == 0) { next = cursor; return 0; }
  // Ring overwrite may have removed the leading bytes of a UTF-8 character.
  while (cursor != cursor_ && (bytes_[cursor % LOG_SIZE] & 0xc0) == 0x80) {
    ++cursor;
    ++dropped;
  }
  size_t count = 0;
  while (cursor != cursor_ && count + 1 < capacity) {
    uint8_t first = bytes_[cursor % LOG_SIZE];
    size_t width = first < 0x80 ? 1 : first >= 0xc2 && first <= 0xdf ? 2 :
      first >= 0xe0 && first <= 0xef ? 3 : first >= 0xf0 && first <= 0xf4 ? 4 : 0;
    bool valid = width != 0;
    for (size_t i = 1; valid && i < width && i < cursor_ - cursor; ++i) {
      uint8_t value = bytes_[(cursor + i) % LOG_SIZE];
      valid = (value & 0xc0) == 0x80;
      if (i == 1) valid = valid && !(first == 0xe0 && value < 0xa0) &&
        !(first == 0xed && value >= 0xa0) && !(first == 0xf0 && value < 0x90) &&
        !(first == 0xf4 && value >= 0x90);
    }
    if (valid && cursor_ - cursor < width) break;
    if (!valid || first == 0) { output[count++] = '?'; ++cursor; continue; }
    if (count + width >= capacity) break;
    for (size_t i = 0; i < width; ++i)
      output[count++] = (char)bytes_[(cursor + i) % LOG_SIZE];
    cursor += (uint32_t)width;
  }
  output[count] = '\0';
  next = cursor;
  return count;
}

}  // namespace RemoteConsoleCore
