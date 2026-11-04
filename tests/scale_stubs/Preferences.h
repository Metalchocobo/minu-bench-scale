#pragma once

#include <cstddef>
#include <cstring>

namespace ScaleTestNvs {
struct State {
  bool namespacePresent = false;
  bool hasOffset = false;
  bool hasScale = false;
  bool failReadBegin = false;
  bool failWriteBegin = false;
  bool failOffsetWrite = false;
  bool failScaleWrite = false;
  long offset = 0;
  float scale = 0.0f;
  unsigned int writes = 0;
};
inline State state;
}

class Preferences {
public:
  bool begin(const char* name, bool readOnly) {
    const auto& nvs = ScaleTestNvs::state;
    started_ = std::strcmp(name, "hxscale") == 0 &&
      !(readOnly && (!nvs.namespacePresent || nvs.failReadBegin)) &&
      !(!readOnly && nvs.failWriteBegin);
    readOnly_ = readOnly;
    if (started_ && !readOnly) ScaleTestNvs::state.namespacePresent = true;
    return started_;
  }

  long getLong(const char* key, long fallback) const {
    const auto& nvs = ScaleTestNvs::state;
    return started_ && nvs.hasOffset && std::strcmp(key, "offset_counts") == 0
      ? nvs.offset : fallback;
  }

  float getFloat(const char* key, float fallback) const {
    const auto& nvs = ScaleTestNvs::state;
    return started_ && nvs.hasScale && std::strcmp(key, "scale_cpg") == 0
      ? nvs.scale : fallback;
  }

  std::size_t putLong(const char* key, long value) {
    auto& nvs = ScaleTestNvs::state;
    if (!started_ || readOnly_ || nvs.failOffsetWrite ||
        std::strcmp(key, "offset_counts") != 0) return 0;
    nvs.offset = value;
    nvs.hasOffset = true;
    ++nvs.writes;
    return sizeof(value);
  }

  std::size_t putFloat(const char* key, float value) {
    auto& nvs = ScaleTestNvs::state;
    if (!started_ || readOnly_ || nvs.failScaleWrite ||
        std::strcmp(key, "scale_cpg") != 0) return 0;
    nvs.scale = value;
    nvs.hasScale = true;
    ++nvs.writes;
    return sizeof(value);
  }

  void end() { started_ = false; }

private:
  bool started_ = false;
  bool readOnly_ = false;
};
