#pragma once

#include <stddef.h>

namespace ScaleIdentity {

constexpr size_t CODE_MAX_LEN = 12;
constexpr size_t NAME_MAX_LEN = 6 + CODE_MAX_LEN; // "Ronin " + code
constexpr char DEFAULT_CODE[] = "00";

// Call once at boot, before the display and network worker start. The active
// identity then stays immutable until reboot, including while MQTT reads it.
void load();
const char* code();
const char* name();
const char* savedCode();

// Codes are strings: preserve leading zeroes and accept ASCII A-Z/a-z/0-9/-/_.
bool isValidCode(const char* value);
// Persist for the next boot. A failed write leaves active/saved state unchanged.
bool saveCode(const char* value);

} // namespace ScaleIdentity
