#pragma once

// WiFi credentials storage (NVS) for Minù Bench Scale
// - Stores up to 2 SSID/PASS pairs in ESP32 NVS (Preferences)
// - Designed to persist across firmware updates (NVS is not erased by normal uploads/OTA)

#include <Arduino.h>

namespace WifiStore {

  // Max lengths per common WiFi stacks
  static const size_t SSID_MAX_LEN = 32;
  static const size_t PASS_MAX_LEN = 63;

  // Load a slot (1..2). Returns true if SSID is non-empty.
  // ssidOut/passOut are always null-terminated.
  bool loadSlot(uint8_t slot, char* ssidOut, size_t ssidOutSize, char* passOut, size_t passOutSize);

  // Save a slot (1..2). If ssid is empty/null, the slot will be cleared.
  // Strings are truncated to SSID_MAX_LEN / PASS_MAX_LEN.
  bool saveSlot(uint8_t slot, const char* ssid, const char* pass);

  // Clear a slot (1..2).
  bool clearSlot(uint8_t slot);

  // Clear both slots.
  bool clearAll();

  // Helper: returns true if string looks like a configured SSID.
  inline bool isConfiguredSsid(const char* ssid) {
    return (ssid != nullptr) && (ssid[0] != '\0');
  }

  // Helper: create a masked password (stars) without revealing actual content.
  // Output is always null-terminated.
  void maskPassword(const char* pass, char* out, size_t outSize);

} // namespace WifiStore
