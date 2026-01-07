#include "wifi_store.h"

#include <Preferences.h>
#include <string.h>

namespace {
  static const char* NVS_NS_WIFI = "minu_wifi";
  static const char* KEY_S1 = "s1";
  static const char* KEY_P1 = "p1";
  static const char* KEY_S2 = "s2";
  static const char* KEY_P2 = "p2";

  inline bool slotToKeys(uint8_t slot, const char*& ks, const char*& kp) {
    if (slot == 1) { ks = KEY_S1; kp = KEY_P1; return true; }
    if (slot == 2) { ks = KEY_S2; kp = KEY_P2; return true; }
    return false;
  }

  inline void safeCopy(char* dst, size_t dstSize, const char* src) {
    if (!dst || dstSize == 0) return;
    if (!src) { dst[0] = '\0'; return; }
    // strlcpy is available on ESP32/Arduino; keep it simple and safe.
    strlcpy(dst, src, dstSize);
  }

  inline String truncateTo(String s, size_t maxLen) {
    if (s.length() > (int)maxLen) s = s.substring(0, (int)maxLen);
    return s;
  }
}

namespace WifiStore {

bool loadSlot(uint8_t slot, char* ssidOut, size_t ssidOutSize, char* passOut, size_t passOutSize) {
  const char* ks = nullptr;
  const char* kp = nullptr;
  if (!slotToKeys(slot, ks, kp)) {
    safeCopy(ssidOut, ssidOutSize, "");
    safeCopy(passOut, passOutSize, "");
    return false;
  }

  Preferences p;
  p.begin(NVS_NS_WIFI, true);
  String ssid = p.getString(ks, "");
  String pass = p.getString(kp, "");
  p.end();

  ssid = truncateTo(ssid, SSID_MAX_LEN);
  pass = truncateTo(pass, PASS_MAX_LEN);

  safeCopy(ssidOut, ssidOutSize, ssid.c_str());
  safeCopy(passOut, passOutSize, pass.c_str());
  return ssid.length() > 0;
}

bool saveSlot(uint8_t slot, const char* ssid, const char* pass) {
  const char* ks = nullptr;
  const char* kp = nullptr;
  if (!slotToKeys(slot, ks, kp)) return false;

  String s = ssid ? String(ssid) : String();
  String pss = pass ? String(pass) : String();
  s.trim();
  // Passwords can legally contain spaces; do not trim pss.

  // If SSID is empty => clear
  if (s.length() == 0) {
    return clearSlot(slot);
  }

  s = truncateTo(s, SSID_MAX_LEN);
  pss = truncateTo(pss, PASS_MAX_LEN);

  Preferences p;
  p.begin(NVS_NS_WIFI, false);
  bool ok1 = p.putString(ks, s) > 0;
  // Save password even if empty (open network)
  bool ok2 = p.putString(kp, pss) >= 0;
  p.end();
  return ok1 && ok2;
}

bool clearSlot(uint8_t slot) {
  const char* ks = nullptr;
  const char* kp = nullptr;
  if (!slotToKeys(slot, ks, kp)) return false;

  Preferences p;
  p.begin(NVS_NS_WIFI, false);
  // Remove keys if present
  p.remove(ks);
  p.remove(kp);
  p.end();
  return true;
}

bool clearAll() {
  bool ok1 = clearSlot(1);
  bool ok2 = clearSlot(2);
  return ok1 && ok2;
}

void maskPassword(const char* pass, char* out, size_t outSize) {
  if (!out || outSize == 0) return;
  out[0] = '\0';
  if (!pass) return;

  size_t n = strlen(pass);
  // Keep it readable and avoid leaking exact length if someone cares.
  // 0 -> "" ; 1..8 -> that many stars ; >8 -> 8 stars
  size_t stars = (n == 0) ? 0 : (n <= 8 ? n : 8);
  if (stars == 0) {
    out[0] = '\0';
    return;
  }
  if (stars >= outSize) stars = outSize - 1;
  for (size_t i = 0; i < stars; i++) out[i] = '*';
  out[stars] = '\0';
}

} // namespace WifiStore
