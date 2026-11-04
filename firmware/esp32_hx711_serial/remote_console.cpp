#include "net_ota_cloud.h"
#include "remote_console.h"

#include <string.h>
#include <time.h>

RemoteConsole::LogOutput ConsoleLog;

size_t RemoteConsole::LogOutput::write(uint8_t value) {
  return write(&value, 1);
}

size_t RemoteConsole::LogOutput::write(const uint8_t* buffer, size_t length) {
  size_t written = Serial.write(buffer, length);
  capture(buffer, length);
  return written;
}

#if ENABLE_MQTT
namespace RemoteConsole {
using namespace RemoteConsoleCore;

static Session session;
static LogRing ring;
static portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;
static bool captureEnabled = false;
static bool closedEvent = false;
static bool responsePending = false;
static bool commandRunning = false;
static Request pendingRequest;
static Result pendingResult;

static const char* actionText(Action action) {
  switch (action) {
    case Action::Probe: return "probe";
    case Action::Open: return "open";
    case Action::Poll: return "poll";
    case Action::Exec: return "exec";
    case Action::Close: return "close";
    default: return "invalid";
  }
}

static Action parseAction(const char* action) {
  if (!action) return Action::Invalid;
  for (uint8_t i = 0; i < (uint8_t)Action::Invalid; ++i) {
    Action candidate = (Action)i;
    if (strcmp(action, actionText(candidate)) == 0) return candidate;
  }
  return Action::Invalid;
}

static void setCapture(bool enabled, bool clear = false) {
  portENTER_CRITICAL(&logMux);
  captureEnabled = enabled;
  if (clear) ring.clear();
  portEXIT_CRITICAL(&logMux);
}

void begin(const char* bootId) {
  session.reset(bootId);
  setCapture(false, true);
  responsePending = false;
  commandRunning = false;
}

void capture(const uint8_t* buffer, size_t length) {
  portENTER_CRITICAL(&logMux);
  if (captureEnabled) ring.append(buffer, length);
  portEXIT_CRITICAL(&logMux);
}

void tick(uint32_t nowMs) {
  if (session.tick(nowMs)) {
    setCapture(false);
    closedEvent = true;
    if (responsePending && pendingResult.queued && !commandRunning)
      pendingResult = {Outcome::Rejected, "session_closed"};
  }
}

bool active() { return session.active(); }
bool executingCommand() { return commandRunning; }

bool popClosed() {
  bool result = closedEvent;
  closedEvent = false;
  return result;
}

void transportLost() {
  if (session.active()) closedEvent = true;
  session.close(millis());
  setCapture(false);
  responsePending = false;
  commandRunning = false;
}

static bool copyField(JsonVariantConst value, char* output, size_t capacity,
                      bool optional = false) {
  if (value.isNull() && optional) { output[0] = '\0'; return true; }
  if (!value.is<const char*>()) return false;
  JsonString text = value.as<JsonString>();
  if (text.size() >= capacity || strlen(text.c_str()) != text.size()) return false;
  memcpy(output, text.c_str(), text.size() + 1);
  return true;
}

void receive(const uint8_t* payload, size_t length) {
  // A lost response is retried with the same request/sequence by the gateway.
  // Do not replace an accepted command or an already queued reply in a burst.
  if (responsePending || !payload || !length || length > 580) return;
  JsonDocument doc;
  if (deserializeJson(doc, payload, length) || !doc.is<JsonObject>() ||
      !doc["v"].is<unsigned int>() || doc["v"].as<unsigned int>() != 1) return;
  Request request;
  request.action = parseAction(doc["action"] | "");
  bool valid = copyField(doc["request_id"], request.requestId, sizeof(request.requestId)) &&
    copyField(doc["session_id"], request.sessionId, sizeof(request.sessionId),
              request.action == Action::Probe) &&
    copyField(doc["boot_id"], request.bootId, sizeof(request.bootId),
              request.action == Action::Probe) &&
    doc["expires_at"].is<uint32_t>();
  request.expiresAt = doc["expires_at"] | (uint32_t)0;
  if (doc["cursor"].isNull()) request.cursor = 0;
  else if (doc["cursor"].is<uint32_t>()) request.cursor = doc["cursor"].as<uint32_t>();
  else valid = false;
  if (request.action == Action::Exec) {
    valid = valid && doc["seq"].is<uint32_t>() &&
      copyField(doc["command"], request.command, sizeof(request.command));
    request.sequence = doc["seq"] | (uint32_t)0;
  }
  pendingRequest = request;
  pendingResult = valid
    ? session.receive(request, millis(), (uint32_t)time(nullptr))
    : Result{Outcome::Rejected, "invalid_request"};
  if (pendingResult.opened) setCapture(true, true);
  if (!session.active()) {
    setCapture(false);
    if (request.action == Action::Close && pendingResult.outcome == Outcome::Ok)
      closedEvent = true;
  }
  responsePending = true;
}

bool takeCommand(char* output, size_t capacity) {
  if (!responsePending || !pendingResult.queued || commandRunning ||
      !session.active() || !session.pending() || capacity < COMMAND_SIZE) return false;
  strcpy(output, session.command());
  commandRunning = true;
  return true;
}

void completeCommand(const char* error) {
  if (!commandRunning) return;
  commandRunning = false;
  session.complete(error);
  pendingResult = {error && *error ? Outcome::Rejected : Outcome::Done, error ? error : ""};
}

void publishPending() {
  if (!responsePending || pendingResult.queued || commandRunning) return;
  JsonDocument doc;
  doc["v"] = 1;
  doc["type"] = "console";
  doc["action"] = actionText(pendingRequest.action);
  doc["request_id"] = pendingRequest.requestId;
  doc["session_id"] = pendingRequest.sessionId;
  doc["boot_id"] = session.bootId();
  doc["scale_id"] = Net::getScaleId();
  static const char* states[] = {"ok", "done", "busy", "rejected"};
  doc["state"] = states[(uint8_t)pendingResult.outcome];
  doc["error"] = pendingResult.error;
  doc["console_version"] = 1;
  char data[161];
  if ((pendingRequest.action == Action::Poll || pendingRequest.action == Action::Open) &&
      pendingResult.outcome == Outcome::Ok) {
    uint32_t next, oldest, dropped;
    size_t capacity = sizeof(data);
    do {
      portENTER_CRITICAL(&logMux);
      ring.read(pendingRequest.cursor, data, capacity, next, oldest, dropped);
      portEXIT_CRITICAL(&logMux);
      doc["data"] = data;
      doc["cursor"] = next;
      doc["oldest_cursor"] = oldest;
      doc["dropped"] = dropped;
      if (measureJson(doc) <= 570 || capacity <= 2) break;
      capacity /= 2;
    } while (true);
  }
  char payload[584];
  size_t size = measureJson(doc);
  if (size >= sizeof(payload)) { responsePending = false; return; }
  serializeJson(doc, payload, sizeof(payload));
  if (Net::mqttPublishConsole(payload, size)) responsePending = false;
}

}  // namespace RemoteConsole
#else
void RemoteConsole::capture(const uint8_t*, size_t) {}
#endif
