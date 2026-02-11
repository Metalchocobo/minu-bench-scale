#pragma once

// MQTT credentials storage (NVS) for Minu Bench Scale
// - Stores host, username and password in ESP32 NVS (Preferences)
// - Designed to persist across firmware updates (NVS is not erased by normal uploads/OTA)
// - Same pattern as wifi_store.h and ota_store.h

#include <Arduino.h>

namespace MqttStore {

  static const size_t HOST_MAX_LEN = 64;
  static const size_t USER_MAX_LEN = 32;
  static const size_t PASS_MAX_LEN = 64;

  // Load credentials from NVS. Returns true if host is non-empty (configured).
  // All output buffers are always null-terminated.
  bool load(char* hostOut, size_t hostOutSize,
            char* userOut, size_t userOutSize,
            char* passOut, size_t passOutSize);

  // Save credentials to NVS. If host is empty/null, equivalent to clear().
  bool save(const char* host, const char* user, const char* pass);

  // Clear all stored credentials.
  bool clear();

  // Returns true if credentials are configured (host non-empty).
  bool isConfigured();

} // namespace MqttStore
