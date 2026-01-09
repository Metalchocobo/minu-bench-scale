#include "ota_store.h"

#include <Preferences.h>
#include <MD5Builder.h>

namespace {
  static const char* NVS_NS_OTA = "minu_ota";
  static const char* KEY_HASH  = "hash"; // MD5 hex

  inline void safeCopy(char* dst, size_t dstSize, const char* src) {
    if (!dst || dstSize == 0) return;
    if (!src) { dst[0] = '\0'; return; }
    strlcpy(dst, src, dstSize);
  }

  inline String truncateTo(String s, size_t maxLen) {
    if (s.length() > (int)maxLen) s = s.substring(0, (int)maxLen);
    return s;
  }

  inline String md5HexOf(const String& s) {
    MD5Builder md5;
    md5.begin();
    md5.add(s);
    md5.calculate();
    return md5.toString(); // 32 chars hex
  }
}

namespace OtaStore {

bool loadHash(char* out, size_t outSize) {
  safeCopy(out, outSize, "");

  Preferences p;
  p.begin(NVS_NS_OTA, true);
  String h = p.getString(KEY_HASH, "");
  p.end();

  h.trim();
  h = truncateTo(h, MD5_HEX_LEN);

  // Basic validation: must be 32 hex chars
  if (h.length() != (int)MD5_HEX_LEN) {
    safeCopy(out, outSize, "");
    return false;
  }
  for (size_t i = 0; i < MD5_HEX_LEN; i++) {
    char c = h[i];
    bool ok = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
    if (!ok) {
      safeCopy(out, outSize, "");
      return false;
    }
  }

  // ArduinoOTA expects lowercase MD5 (common examples). Normalize to lowercase.
  h.toLowerCase();
  safeCopy(out, outSize, h.c_str());
  return true;
}

bool savePassword(const char* password) {
  if (!password) return false;
  String pass = String(password);
  // Password can include spaces; do not trim. Only guard against empty.
  if (pass.length() == 0) return false;

  String h = md5HexOf(pass);
  h = truncateTo(h, MD5_HEX_LEN);
  h.toLowerCase();

  Preferences p;
  p.begin(NVS_NS_OTA, false);
  bool ok = p.putString(KEY_HASH, h) > 0;
  p.end();
  return ok;
}

bool clear() {
  Preferences p;
  p.begin(NVS_NS_OTA, false);
  p.remove(KEY_HASH);
  p.end();
  return true;
}

bool isConfigured() {
  char h[MD5_HEX_LEN + 1] = {0};
  return loadHash(h, sizeof(h));
}

} // namespace OtaStore
