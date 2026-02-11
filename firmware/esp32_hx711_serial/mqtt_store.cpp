#include "mqtt_store.h"

#include <Preferences.h>
#include <string.h>

namespace {
  static const char* NVS_NS_MQTT = "minu_mqtt";
  static const char* KEY_HOST = "host";
  static const char* KEY_USER = "user";
  static const char* KEY_PASS = "pass";

  inline void safeCopy(char* dst, size_t dstSize, const char* src) {
    if (!dst || dstSize == 0) return;
    if (!src) { dst[0] = '\0'; return; }
    strlcpy(dst, src, dstSize);
  }

  inline String truncateTo(String s, size_t maxLen) {
    if (s.length() > (int)maxLen) s = s.substring(0, (int)maxLen);
    return s;
  }
}

namespace MqttStore {

bool load(char* hostOut, size_t hostOutSize,
          char* userOut, size_t userOutSize,
          char* passOut, size_t passOutSize) {
  safeCopy(hostOut, hostOutSize, "");
  safeCopy(userOut, userOutSize, "");
  safeCopy(passOut, passOutSize, "");

  Preferences p;
  p.begin(NVS_NS_MQTT, true);
  String host = p.getString(KEY_HOST, "");
  String user = p.getString(KEY_USER, "");
  String pass = p.getString(KEY_PASS, "");
  p.end();

  host = truncateTo(host, HOST_MAX_LEN);
  user = truncateTo(user, USER_MAX_LEN);
  pass = truncateTo(pass, PASS_MAX_LEN);

  safeCopy(hostOut, hostOutSize, host.c_str());
  safeCopy(userOut, userOutSize, user.c_str());
  safeCopy(passOut, passOutSize, pass.c_str());
  return host.length() > 0;
}

bool save(const char* host, const char* user, const char* pass) {
  String h = host ? String(host) : String();
  String u = user ? String(user) : String();
  String pw = pass ? String(pass) : String();
  h.trim();
  // user and pass can contain spaces; do not trim

  if (h.length() == 0) {
    return clear();
  }

  h  = truncateTo(h,  HOST_MAX_LEN);
  u  = truncateTo(u,  USER_MAX_LEN);
  pw = truncateTo(pw, PASS_MAX_LEN);

  Preferences p;
  p.begin(NVS_NS_MQTT, false);
  bool ok1 = p.putString(KEY_HOST, h)  > 0;
  bool ok2 = p.putString(KEY_USER, u)  >= 0;  // user can be empty
  bool ok3 = p.putString(KEY_PASS, pw) >= 0;  // pass can be empty
  p.end();
  return ok1 && ok2 && ok3;
}

bool clear() {
  Preferences p;
  p.begin(NVS_NS_MQTT, false);
  p.remove(KEY_HOST);
  p.remove(KEY_USER);
  p.remove(KEY_PASS);
  p.end();
  return true;
}

bool isConfigured() {
  Preferences p;
  p.begin(NVS_NS_MQTT, true);
  String h = p.getString(KEY_HOST, "");
  p.end();
  h.trim();
  return h.length() > 0;
}

} // namespace MqttStore
