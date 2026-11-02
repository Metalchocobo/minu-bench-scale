#pragma once

#include <cstddef>
#include <cstring>
#include <string>

namespace IdentityTestNvs {
struct State {
  bool namespacePresent = false;
  bool keyPresent = false;
  bool failReadBegin = false;
  bool failWriteBegin = false;
  bool failRead = false;
  bool failWrite = false;
  std::string value;
  unsigned int writes = 0;
  unsigned int opens = 0;
  unsigned int closes = 0;
};
inline State state;
}

// Models the buffer overload and failure conventions of ESP32 Preferences.
class Preferences {
public:
  bool begin(const char* name, bool readOnly) {
    auto& nvs = IdentityTestNvs::state;
    if (std::strcmp(name, "minu_identity") != 0 ||
        (readOnly && (nvs.failReadBegin || !nvs.namespacePresent)) ||
        (!readOnly && nvs.failWriteBegin)) return false;
    if (!readOnly) nvs.namespacePresent = true;
    started_ = true;
    readOnly_ = readOnly;
    ++nvs.opens;
    return true;
  }

  bool isKey(const char* key) const {
    return started_ && std::strcmp(key, "code") == 0 &&
      IdentityTestNvs::state.keyPresent;
  }

  std::size_t getString(const char* key, char* destination, std::size_t maxLen) {
    const auto& nvs = IdentityTestNvs::state;
    if (!isKey(key) || nvs.failRead || nvs.value.size() + 1 > maxLen) return 0;
    std::memcpy(destination, nvs.value.c_str(), nvs.value.size() + 1);
    return nvs.value.size() + 1;
  }

  std::size_t putString(const char* key, const char* value) {
    auto& nvs = IdentityTestNvs::state;
    if (!started_ || readOnly_ || std::strcmp(key, "code") != 0 ||
        nvs.failWrite) return 0;
    nvs.value = value;
    nvs.keyPresent = true;
    ++nvs.writes;
    return nvs.value.size();
  }

  void end() {
    if (started_) ++IdentityTestNvs::state.closes;
    started_ = false;
  }

private:
  bool started_ = false;
  bool readOnly_ = false;
};
