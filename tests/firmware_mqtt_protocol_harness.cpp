#include <cassert>
#include <cstdint>
#include <iostream>
#include <string>
#include <utility>

namespace {

struct Weigh {
  std::string commandId;
  std::string sessionId;
  std::string uuid;
  uint32_t productId = 0;
  float targetWeight = 0.0f;
  std::string name;
};

enum class WeighResult {
  Activated,
  Duplicate,
  Conflict,
  IgnoredPending,
  Invalid,
};

enum class EnterMode {
  RemoteCommit,
  StandaloneCommit,
  BlockedNoCommand,
  BlockedOfflineCommand,
};

EnterMode evaluateEnter(bool mqttConnected, bool commandActive) {
  if (mqttConnected && !commandActive) return EnterMode::BlockedNoCommand;
  if (!mqttConnected && commandActive) return EnterMode::BlockedOfflineCommand;
  return commandActive ? EnterMode::RemoteCommit : EnterMode::StandaloneCommit;
}

bool isSafeCommandId(const std::string& commandId) {
  if ((!commandId.empty() && commandId.size() < 8) || commandId.size() > 64) {
    return false;
  }
  for (char value : commandId) {
    const bool alphaNumeric = (value >= '0' && value <= '9') ||
      (value >= 'A' && value <= 'Z') || (value >= 'a' && value <= 'z');
    if (!alphaNumeric && value != '-' && value != '_') return false;
  }
  return true;
}

std::string normalizedName(const std::string& name) {
  return name.substr(0, 31);
}

bool sameTarget(float left, float right) {
  float delta = left - right;
  if (delta < 0.0f) delta = -delta;
  return delta < 0.05f;
}

struct ProtocolModel {
  bool active = false;
  Weigh command;
  bool responsePending = false;
  std::string responseId;
  std::string responsePayload;
  unsigned activations = 0;
  unsigned commandAcks = 0;

  WeighResult weigh(const Weigh& incoming) {
    if (!isSafeCommandId(incoming.commandId)) return WeighResult::Invalid;
    if (responsePending) return WeighResult::IgnoredPending;

    if (active && !incoming.commandId.empty() &&
        incoming.commandId == command.commandId) {
      // The active command is authoritative even if a broken client reuses the
      // idempotency key with different content.
      if (incoming.sessionId != command.sessionId ||
          incoming.uuid != command.uuid ||
          incoming.productId != command.productId ||
          !sameTarget(incoming.targetWeight, command.targetWeight) ||
          normalizedName(incoming.name) != command.name) {
        return WeighResult::Conflict;
      }
      ++commandAcks;
      return WeighResult::Duplicate;
    }

    active = true;
    command = incoming;
    command.name = normalizedName(incoming.name);
    ++activations;
    if (!command.commandId.empty()) ++commandAcks;
    return WeighResult::Activated;
  }

  bool clear(const std::string& sessionId, const std::string& commandId) {
    if (responsePending || !active) return false;
    const bool fullyLegacy = command.commandId.empty() && command.sessionId.empty();
    const bool sessionMatches = fullyLegacy || sessionId == command.sessionId;
    const bool commandMatches = command.commandId.empty()
      ? commandId.empty()
      : !commandId.empty() && commandId == command.commandId;
    if (!sessionMatches || !commandMatches) return false;
    active = false;
    command = {};
    return true;
  }

  void stageResponse(std::string id, std::string payload) {
    assert(active && !responsePending);
    responsePending = true;
    responseId = std::move(id);
    responsePayload = std::move(payload);
  }

  bool responseAck(const std::string& id) {
    if (!responsePending || id != responseId) return false;
    responsePending = false;
    responseId.clear();
    responsePayload.clear();
    active = false;
    command = {};
    return true;
  }
};

void testFencedClear() {
  ProtocolModel model;
  const Weigh current{
    "command-new", "session-a", "uuid-a", 42, 100.0f, "Sugar"};
  assert(model.weigh(current) == WeighResult::Activated);
  assert(!model.clear("session-a", "command-old"));
  assert(!model.clear("session-b", "command-new"));
  assert(!model.clear("session-a", ""));
  assert(model.active && model.command.commandId == "command-new");
  assert(model.clear("session-a", "command-new"));
  assert(!model.active);

  const Weigh legacy{"", "session-legacy", "uuid-b", 7, 50.0f, "Flour"};
  assert(model.weigh(legacy) == WeighResult::Activated);
  assert(!model.clear("session-other", ""));
  assert(!model.clear("session-legacy", "unexpected-id"));
  assert(model.clear("session-legacy", ""));

  const Weigh fullyLegacy{"", "", "uuid-c", 8, 25.0f, "Butter"};
  assert(model.weigh(fullyLegacy) == WeighResult::Activated);
  assert(model.clear("session-from-new-manager", ""));

  assert(model.weigh(fullyLegacy) == WeighResult::Activated);
  assert(!model.clear("session-from-new-manager", "unexpected-id"));
  assert(model.active);
}

void testEnterRequiresCommandOnlyWhileConnected() {
  assert(evaluateEnter(true, true) == EnterMode::RemoteCommit);
  assert(evaluateEnter(true, false) == EnterMode::BlockedNoCommand);
  assert(evaluateEnter(false, true) == EnterMode::BlockedOfflineCommand);
  assert(evaluateEnter(false, false) == EnterMode::StandaloneCommit);
}

void testDuplicateIsIdempotentAndReacked() {
  ProtocolModel model;
  const Weigh original{"command-1", "session-1", "uuid-1", 10, 125.0f, "Milk"};
  assert(model.weigh(original) == WeighResult::Activated);
  assert(model.activations == 1 && model.commandAcks == 1);

  assert(model.weigh(original) == WeighResult::Duplicate);
  assert(model.activations == 1 && model.commandAcks == 2);
  assert(model.command.uuid == "uuid-1" && model.command.productId == 10);

  const Weigh conflicting{
    "command-1", "session-evil", "uuid-evil", 99, 999.0f, "Conflict"};
  assert(model.weigh(conflicting) == WeighResult::Conflict);
  assert(model.activations == 1 && model.commandAcks == 2);
  assert(model.command.sessionId == "session-1");
  assert(model.command.uuid == "uuid-1");
  assert(model.command.productId == 10);
  assert(model.command.targetWeight == 125.0f);
}

void testDuplicateNormalizesLongName() {
  ProtocolModel model;
  const Weigh original{
    "cmd-long-name", "session-1", "uuid-1", 10, 125.0f,
    "Ingredient name longer than thirty-one characters"};
  assert(original.name.size() > 31);
  assert(model.weigh(original) == WeighResult::Activated);
  assert(model.command.name.size() == 31);
  assert(model.weigh(original) == WeighResult::Duplicate);
  assert(model.activations == 1 && model.commandAcks == 2);
}

void testPendingResponseIsByteImmutable() {
  ProtocolModel model;
  const Weigh original{"command-1", "session-1", "uuid-1", 10, 125.0f, "Milk"};
  assert(model.weigh(original) == WeighResult::Activated);
  model.stageResponse(
    "response-1",
    "{\"type\":\"confirm\",\"session_id\":\"session-1\","
    "\"command_id\":\"command-1\",\"response_id\":\"response-1\"}");

  const std::string frozenPayload = model.responsePayload;
  const Weigh replacement{
    "command-2", "session-2", "uuid-2", 20, 250.0f, "Sugar"};
  const unsigned frozenAcks = model.commandAcks;

  assert(model.weigh(original) == WeighResult::IgnoredPending);
  assert(model.weigh(replacement) == WeighResult::IgnoredPending);
  assert(!model.clear("session-1", "command-1"));
  assert(!model.responseAck("response-wrong"));
  assert(model.responsePending);
  assert(model.responsePayload == frozenPayload);
  assert(model.command.commandId == "command-1");
  assert(model.commandAcks == frozenAcks);

  assert(model.responseAck("response-1"));
  assert(!model.responsePending && !model.active);
}

void testCommandIdValidation() {
  ProtocolModel model;
  const Weigh valid{
    "550e8400-e29b-41d4-a716-446655440000", "session", "uuid", 1, 1.0f, "Salt"};
  assert(model.weigh(valid) == WeighResult::Activated);

  ProtocolModel invalidModel;
  Weigh invalid = valid;
  invalid.commandId = std::string(7, 'x');
  assert(invalidModel.weigh(invalid) == WeighResult::Invalid);
  invalid.commandId = std::string(8, 'x');
  assert(invalidModel.weigh(invalid) == WeighResult::Activated);
  assert(invalidModel.clear("session", std::string(8, 'x')));
  invalid.commandId = "bad\"id";
  assert(invalidModel.weigh(invalid) == WeighResult::Invalid);
  invalid.commandId = "bad\\id";
  assert(invalidModel.weigh(invalid) == WeighResult::Invalid);
  invalid.commandId = std::string(65, 'x');
  assert(invalidModel.weigh(invalid) == WeighResult::Invalid);
  assert(!invalidModel.active && invalidModel.commandAcks == 1);
}

void testMaximumPayloadsFitBuffers() {
  const std::string commandId(64, 'c');
  const std::string sessionId(64, 's');
  const std::string uuid(36, 'u');
  const std::string responseId(24, 'r');
  const std::string topic = "minu/scale/123456789abc/response";

  const std::string commandAck =
    "{\"type\":\"command_ack\",\"command_id\":\"" + commandId +
    "\",\"session_id\":\"" + sessionId + "\",\"uuid\":\"" + uuid +
    "\",\"product_id\":4294967295,\"state\":\"active\"}";
  const std::string confirm =
    "{\"type\":\"confirm\",\"uuid\":\"" + uuid +
    "\",\"product_id\":4294967295,\"session_id\":\"" + sessionId +
    "\",\"command_id\":\"" + commandId + "\",\"response_id\":\"" +
    responseId + "\",\"actual_weight\":-123456.7}";

  assert(commandAck.size() < 320);
  assert(confirm.size() < 384);
  // MQTT_MAX_PACKET_SIZE also includes topic and packet framing.
  assert(topic.size() + commandAck.size() + 8 < 512);
  assert(topic.size() + confirm.size() + 8 < 512);
}

} // namespace

int main() {
  testFencedClear();
  testEnterRequiresCommandOnlyWhileConnected();
  testDuplicateIsIdempotentAndReacked();
  testDuplicateNormalizesLongName();
  testPendingResponseIsByteImmutable();
  testCommandIdValidation();
  testMaximumPayloadsFitBuffers();
  std::cout << "firmware MQTT protocol harness: OK\n";
  return 0;
}
