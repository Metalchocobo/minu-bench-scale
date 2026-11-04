// Host-only JSON bridge to the production console state machine.
// Build with ArduinoJson's host-compatible headers and remote_console_state.cpp.
#include "../firmware/esp32_hx711_serial/remote_console_state.h"
#include <ArduinoJson.h>

#include <cstring>
#include <iostream>
#include <string>

using namespace RemoteConsoleCore;

static Action parseAction(const char* value) {
  if (!value) return Action::Invalid;
  static const char* names[] = {"probe", "open", "poll", "exec", "close"};
  for (size_t i = 0; i < 5; ++i) {
    if (std::strcmp(names[i], value) == 0) return static_cast<Action>(i);
  }
  return Action::Invalid;
}

static bool copyField(JsonVariantConst value, char* output, size_t capacity,
                      bool optional = false) {
  if (value.isNull() && optional) { output[0] = '\0'; return true; }
  if (!value.is<const char*>()) return false;
  JsonString text = value.as<JsonString>();
  if (text.size() >= capacity || std::strlen(text.c_str()) != text.size()) return false;
  std::memcpy(output, text.c_str(), text.size() + 1);
  return true;
}

int main() {
  Session session;
  unsigned executions = 0;
  std::string line;
  while (std::getline(std::cin, line)) {
    JsonDocument envelope;
    if (deserializeJson(envelope, line)) return 2;
    if (envelope["reset"].is<const char*>()) {
      session.reset(envelope["reset"].as<const char*>());
      executions = 0;
    }
    const char* wire = envelope["wire"] | "";
    JsonDocument document;
    Request request;
    bool valid = std::strlen(wire) <= 580 && !deserializeJson(document, wire);
    request.action = parseAction(document["action"] | "");
    valid = valid && document["v"].is<unsigned>() && document["v"].as<unsigned>() == 1 &&
      copyField(document["request_id"], request.requestId, sizeof(request.requestId)) &&
      copyField(document["session_id"], request.sessionId, sizeof(request.sessionId), request.action == Action::Probe) &&
      copyField(document["boot_id"], request.bootId, sizeof(request.bootId), request.action == Action::Probe) &&
      document["expires_at"].is<uint32_t>();
    request.expiresAt = document["expires_at"] | uint32_t{0};
    request.cursor = document["cursor"] | uint32_t{0};
    if (request.action == Action::Exec) {
      valid = valid && document["seq"].is<uint32_t>() &&
        copyField(document["command"], request.command, sizeof(request.command));
      request.sequence = document["seq"] | uint32_t{0};
    }
    Result result = valid
      ? session.receive(request, envelope["now_ms"].as<uint32_t>(), envelope["now_epoch"].as<uint32_t>())
      : Result{Outcome::Rejected, "invalid_request"};
    if (result.queued) {
      ++executions;
      session.complete();
      result = {Outcome::Done, ""};
    }
    JsonDocument response;
    static const char* states[] = {"ok", "done", "busy", "rejected"};
    response["v"] = 1;
    response["type"] = "console";
    response["action"] = document["action"];
    response["request_id"] = request.requestId;
    response["session_id"] = request.sessionId;
    response["boot_id"] = session.bootId();
    response["scale_id"] = "aabbccddeeff";
    response["console_version"] = 1;
    response["state"] = states[static_cast<unsigned>(result.outcome)];
    response["error"] = result.error;
    response["active"] = session.active();
    response["executions"] = executions;
    serializeJson(response, std::cout);
    std::cout << '\n' << std::flush;
  }
  return 0;
}
