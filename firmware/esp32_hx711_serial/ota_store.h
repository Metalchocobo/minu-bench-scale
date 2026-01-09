#pragma once

// OTA password storage (NVS) for Minù Bench Scale
// - Stores an MD5 hash (hex) used by ArduinoOTA.setPasswordHash()
// - Designed to persist across firmware updates (NVS is not erased by normal uploads/OTA)
// - Password itself is never stored or printed (only a hash)

#include <Arduino.h>

namespace OtaStore {

  static const size_t MD5_HEX_LEN = 32; // 32 hex chars

  // Load MD5 hash (hex, 32 chars). Returns true if configured.
  // out is always null-terminated.
  bool loadHash(char* out, size_t outSize);

  // Save password: computes MD5 hex and stores it.
  // Returns true on success.
  bool savePassword(const char* password);

  // Clear stored hash (disables OTA password after reboot).
  bool clear();

  // Returns true if a hash is currently stored.
  bool isConfigured();

} // namespace OtaStore
