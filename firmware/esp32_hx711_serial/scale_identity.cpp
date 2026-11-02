#include "scale_identity.h"

#include <Preferences.h>
#include <stdio.h>
#include <string.h>

namespace {
constexpr char NVS_NAMESPACE[] = "minu_identity";
constexpr char NVS_CODE_KEY[] = "code";
char activeCode[ScaleIdentity::CODE_MAX_LEN + 1] = "00";
char storedCode[ScaleIdentity::CODE_MAX_LEN + 1] = "00";
char activeName[ScaleIdentity::NAME_MAX_LEN + 1] = "Ronin 00";
}

namespace ScaleIdentity {

bool isValidCode(const char* value) {
  if (!value || !*value) return false;
  size_t length = 0;
  for (; value[length]; ++length) {
    const char c = value[length];
    if (length >= CODE_MAX_LEN ||
        !((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
          (c >= '0' && c <= '9') || c == '-' || c == '_')) return false;
  }
  return true;
}

void load() {
  strcpy(activeCode, DEFAULT_CODE);
  Preferences prefs;
  if (prefs.begin(NVS_NAMESPACE, true)) {
    char value[CODE_MAX_LEN + 1] = {};
    if (prefs.isKey(NVS_CODE_KEY) &&
        prefs.getString(NVS_CODE_KEY, value, sizeof(value)) > 0 &&
        isValidCode(value)) {
      strcpy(activeCode, value);
    }
    prefs.end();
  }
  strcpy(storedCode, activeCode);
  snprintf(activeName, sizeof(activeName), "Ronin %s", activeCode);
}

const char* code() { return activeCode; }
const char* name() { return activeName; }
const char* savedCode() { return storedCode; }

bool saveCode(const char* value) {
  if (!isValidCode(value)) return false;
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, false)) return false;
  const bool saved = prefs.putString(NVS_CODE_KEY, value) == strlen(value);
  prefs.end();
  if (saved) strcpy(storedCode, value);
  return saved;
}

} // namespace ScaleIdentity
