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
  std::string connectionId;

  Weigh() = default;
  Weigh(
      std::string commandIdValue, std::string sessionIdValue,
      std::string uuidValue, uint32_t productIdValue, float targetWeightValue,
      std::string nameValue, std::string connectionIdValue = {})
      : commandId(std::move(commandIdValue)),
        sessionId(std::move(sessionIdValue)), uuid(std::move(uuidValue)),
        productId(productIdValue), targetWeight(targetWeightValue),
        name(std::move(nameValue)), connectionId(std::move(connectionIdValue)) {}
};

struct ConfirmRequest {
  std::string requestId;
  std::string sessionId;
  std::string connectionId;
  std::string commandId;
  std::string uuid;
  uint32_t productId = 0;
};

bool sameConfirmRequest(const ConfirmRequest& left, const ConfirmRequest& right) {
  return left.requestId == right.requestId && left.sessionId == right.sessionId &&
    left.connectionId == right.connectionId && left.commandId == right.commandId &&
    left.uuid == right.uuid && left.productId == right.productId;
}

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

enum class ConfirmRequestResult {
  Accepted,
  AcceptedReplay,
  StagedReplay,
  FailedReplay,
  RejectedStale,
  RejectedConflict,
  RejectedBusy,
  Invalid,
};

enum class ConfirmRequestState {
  None,
  Queued,
  Running,
  Staged,
  Failed,
};

EnterMode evaluateEnter(
    bool mqttConnected, bool commandPresent, bool commandActionable,
    bool localFallback = false) {
  if (localFallback) return EnterMode::StandaloneCommit;
  if (mqttConnected && !commandActionable) return EnterMode::BlockedNoCommand;
  if (!mqttConnected && commandPresent) return EnterMode::BlockedOfflineCommand;
  return commandActionable ? EnterMode::RemoteCommit : EnterMode::StandaloneCommit;
}

enum class FallbackDecision {
  None,
  Grace,
  Immediate,
};

FallbackDecision evaluateFallback(
    bool connected, bool ownerKnown, bool ownerFresh, bool ownerExact,
    bool commandActive, bool modernCommand, bool commandSnapshotReady,
    bool responsePending, bool responseLockExpired = false) {
  const bool ownerScopedContext = !commandActive || modernCommand || responsePending;
  const bool ownerUnavailable = connected && ownerKnown && ownerScopedContext &&
    (!ownerFresh || (modernCommand && !ownerExact));
  if (ownerUnavailable || responseLockExpired) return FallbackDecision::Immediate;

  const bool ownerSnapshotUnknown = connected && !ownerKnown && ownerScopedContext;
  const bool commandSnapshotUnknown = connected && modernCommand &&
    !commandSnapshotReady;
  const bool disconnectedRemoteState = !connected &&
    (commandActive || responsePending);
  return (ownerSnapshotUnknown || commandSnapshotUnknown ||
          disconnectedRemoteState)
    ? FallbackDecision::Grace : FallbackDecision::None;
}

enum class ConnectWorkerState {
  Idle,
  Running,
  Done,
};

struct ConnectWorkerModel {
  ConnectWorkerState state = ConnectWorkerState::Idle;
  bool abortRequested = false;
  bool result = false;
  bool wasAborted = false;

  bool start() {
    if (state != ConnectWorkerState::Idle) return false;
    state = ConnectWorkerState::Running;
    abortRequested = false;
    result = false;
    wasAborted = false;
    return true;
  }

  bool requestAbort() {
    if (state != ConnectWorkerState::Running) return false;
    abortRequested = true;
    return true;
  }

  void finish(bool connected) {
    assert(state == ConnectWorkerState::Running);
    wasAborted = abortRequested;
    result = connected && !wasAborted;
    state = ConnectWorkerState::Done;
  }

  bool consume(bool& connected, bool& aborted) {
    if (state != ConnectWorkerState::Done) return false;
    connected = result;
    aborted = wasAborted;
    state = ConnectWorkerState::Idle;
    return true;
  }

  bool mainOwnsTransport() const {
    return state == ConnectWorkerState::Idle;
  }
};

bool commandBlocksSleep(
    bool commandActive, bool connected, bool actionable, bool localFallback) {
  return commandActive && !localFallback && (!connected || actionable);
}

struct TopBarState {
  bool mqttConnected;
  bool managerAttached;
};

TopBarState evaluateTopBar(
    bool mqttConnected, bool ownerFresh, bool localFallback) {
  return {
    mqttConnected,
    mqttConnected && ownerFresh && !localFallback,
  };
}

bool fallbackReattachAllowed(
    bool localFallback, const std::string& incomingConnectionId,
    uint32_t ownerTimestamp, uint32_t fallbackOwnerTimestamp,
    const std::string& fallbackConnectionId) {
  return !localFallback || incomingConnectionId.empty() ||
    ownerTimestamp > fallbackOwnerTimestamp ||
    incomingConnectionId != fallbackConnectionId;
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
  ConfirmRequestState confirmRequestState = ConfirmRequestState::None;
  ConfirmRequest activeConfirmRequest;
  bool terminalRequestValid[2] = {};
  ConfirmRequest terminalRequests[2];
  ConfirmRequestResult terminalResults[2] = {
    ConfirmRequestResult::Invalid, ConfirmRequestResult::Invalid};
  unsigned terminalRequestNext = 0;
  unsigned confirmExecutions = 0;

  void resetConfirmRequest() {
    confirmRequestState = ConfirmRequestState::None;
    activeConfirmRequest = {};
  }

  ConfirmRequestResult rememberTerminal(
      const ConfirmRequest& request, ConfirmRequestResult result) {
    unsigned index = terminalRequestNext;
    for (unsigned candidate = 0; candidate < 2; ++candidate) {
      if (terminalRequestValid[candidate] &&
          terminalRequests[candidate].requestId == request.requestId) {
        index = candidate;
        break;
      }
    }
    const bool replacingKnown = terminalRequestValid[index] &&
      terminalRequests[index].requestId == request.requestId;
    terminalRequestValid[index] = true;
    terminalRequests[index] = request;
    terminalResults[index] = result;
    if (!replacingKnown) terminalRequestNext = (index + 1) % 2;
    return result;
  }

  WeighResult weigh(const Weigh& incoming) {
    if (!isSafeCommandId(incoming.commandId) ||
        !isSafeCommandId(incoming.connectionId)) return WeighResult::Invalid;
    if (responsePending) return WeighResult::IgnoredPending;
    if (confirmRequestState == ConfirmRequestState::Queued ||
        confirmRequestState == ConfirmRequestState::Running) {
      return WeighResult::IgnoredPending;
    }

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
      command.connectionId = incoming.connectionId;
      return WeighResult::Duplicate;
    }

    resetConfirmRequest();
    active = true;
    command = incoming;
    command.name = normalizedName(incoming.name);
    ++activations;
    if (!command.commandId.empty()) ++commandAcks;
    return WeighResult::Activated;
  }

  bool clear(const std::string& sessionId, const std::string& commandId) {
    if (responsePending || confirmRequestState == ConfirmRequestState::Queued ||
        confirmRequestState == ConfirmRequestState::Running || !active) return false;
    const bool fullyLegacy = command.commandId.empty() && command.sessionId.empty();
    const bool sessionMatches = fullyLegacy || sessionId == command.sessionId;
    const bool commandMatches = command.commandId.empty()
      ? commandId.empty()
      : !commandId.empty() && commandId == command.commandId;
    if (!sessionMatches || !commandMatches) return false;
    active = false;
    command = {};
    resetConfirmRequest();
    return true;
  }

  ConfirmRequestResult confirmRequest(const ConfirmRequest& incoming) {
    if (!isSafeCommandId(incoming.requestId) || incoming.requestId.empty() ||
        !isSafeCommandId(incoming.commandId) || incoming.commandId.empty() ||
        !isSafeCommandId(incoming.connectionId) || incoming.connectionId.empty()) {
      return ConfirmRequestResult::Invalid;
    }

    if (confirmRequestState != ConfirmRequestState::None &&
        incoming.requestId == activeConfirmRequest.requestId) {
      if (!sameConfirmRequest(incoming, activeConfirmRequest)) {
        return ConfirmRequestResult::RejectedConflict;
      }
      if (confirmRequestState == ConfirmRequestState::Staged) {
        return ConfirmRequestResult::StagedReplay;
      }
      if (confirmRequestState == ConfirmRequestState::Failed) {
        return ConfirmRequestResult::FailedReplay;
      }
      return ConfirmRequestResult::AcceptedReplay;
    }
    for (unsigned index = 0; index < 2; ++index) {
      if (!terminalRequestValid[index] ||
          incoming.requestId != terminalRequests[index].requestId) continue;
      return sameConfirmRequest(incoming, terminalRequests[index])
        ? terminalResults[index] : ConfirmRequestResult::RejectedConflict;
    }

    const bool exact = active && incoming.sessionId == command.sessionId &&
      incoming.connectionId == command.connectionId &&
      incoming.commandId == command.commandId && incoming.uuid == command.uuid &&
      incoming.productId == command.productId;
    if (!exact) {
      return rememberTerminal(incoming, ConfirmRequestResult::RejectedStale);
    }
    if (responsePending || (confirmRequestState != ConfirmRequestState::None &&
        confirmRequestState != ConfirmRequestState::Failed)) {
      return rememberTerminal(incoming, ConfirmRequestResult::RejectedBusy);
    }
    activeConfirmRequest = incoming;
    confirmRequestState = ConfirmRequestState::Queued;
    return ConfirmRequestResult::Accepted;
  }

  bool popConfirmRequest() {
    if (confirmRequestState != ConfirmRequestState::Queued) return false;
    confirmRequestState = ConfirmRequestState::Running;
    ++confirmExecutions;
    return true;
  }

  void failConfirmRequest() {
    assert(confirmRequestState == ConfirmRequestState::Running);
    confirmRequestState = ConfirmRequestState::Failed;
    rememberTerminal(activeConfirmRequest, ConfirmRequestResult::FailedReplay);
  }

  void markConfirmRequestStaged() {
    assert(confirmRequestState == ConfirmRequestState::Running && responsePending);
    confirmRequestState = ConfirmRequestState::Staged;
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
    resetConfirmRequest();
    return true;
  }
};

struct OwnerFenceModel {
  bool ownerActive = false;
  std::string ownerSessionId;
  std::string ownerConnectionId;
  uint32_t ownerRemainingMs = 0;
  bool commandActive = false;
  bool commandSnapshotReady = true;
  Weigh command;
  bool responsePending = false;
  bool localFallback = false;

  void setOwner(
      const std::string& sessionId, const std::string& connectionId,
      uint32_t remainingMs) {
    ownerActive = remainingMs > 0;
    ownerSessionId = sessionId;
    ownerConnectionId = connectionId;
    ownerRemainingMs = remainingMs;
  }

  bool ownerAuthorizes(
      const std::string& sessionId, const std::string& connectionId) const {
    return ownerActive && ownerRemainingMs > 0 && !sessionId.empty() &&
      !connectionId.empty() && sessionId == ownerSessionId &&
      connectionId == ownerConnectionId;
  }

  bool ownerConnectionAuthorizes(const std::string& connectionId) const {
    return ownerActive && ownerRemainingMs > 0 && !connectionId.empty() &&
      connectionId == ownerConnectionId;
  }

  bool weigh(const Weigh& incoming) {
    if (responsePending) return false;
    if (!incoming.connectionId.empty() &&
        !ownerAuthorizes(incoming.sessionId, incoming.connectionId)) return false;
    commandActive = true;
    command = incoming;
    localFallback = false;
    commandSnapshotReady = true;
    return true;
  }

  bool actionable() const {
    return commandActive && !localFallback && (command.connectionId.empty() ||
      (commandSnapshotReady &&
       ownerAuthorizes(command.sessionId, command.connectionId)));
  }

  bool responseOperatorLocked() const {
    return responsePending && !localFallback;
  }

  bool detachForLocalInput() {
    if (!responsePending) return false;
    localFallback = true;
    return true;
  }

  bool clear(
      const std::string& sessionId, const std::string& connectionId,
      const std::string& commandId, bool pagehide = false) {
    const bool exactOwnerLifecycle = pagehide &&
      ownerAuthorizes(sessionId, connectionId);
    if (exactOwnerLifecycle && !commandActive) {
      localFallback = true;
      return true;
    }
    if (!commandActive) return false;
    const bool exactLifecycle = pagehide && !command.connectionId.empty() &&
      sessionId == command.sessionId && commandId == command.commandId &&
      connectionId == command.connectionId &&
      ((!ownerActive || ownerRemainingMs == 0) ||
       ownerAuthorizes(sessionId, connectionId));
    if (exactLifecycle) {
      localFallback = true;
      if (responsePending) return true;
      commandActive = false;
      command = {};
      return true;
    }
    if (exactOwnerLifecycle) {
      localFallback = true;
      return true;
    }
    if (responsePending) return false;
    if (!commandSnapshotReady && !command.connectionId.empty() &&
        !connectionId.empty()) {
      commandActive = false;
      command = {};
      commandSnapshotReady = true;
      return true;
    }
    const bool fullyLegacy = command.sessionId.empty() && command.commandId.empty();
    const bool sessionMatches = fullyLegacy || sessionId == command.sessionId;
    const bool commandMatches = command.commandId.empty()
      ? commandId.empty() : commandId == command.commandId;
    const bool connectionMatches = command.connectionId.empty() ||
      (!connectionId.empty() &&
       ((!ownerActive || ownerRemainingMs == 0) ||
        ownerConnectionAuthorizes(connectionId)));
    if (!sessionMatches || !commandMatches || !connectionMatches) return false;
    commandActive = false;
    command = {};
    return true;
  }

  void expireOwner() {
    ownerActive = false;
    ownerRemainingMs = 0;
    localFallback = true;
  }

  void expireOperatorLock() {
    localFallback = true;
  }

  void beginReconnect() {
    ownerActive = false;
    ownerRemainingMs = 0;
    commandSnapshotReady = false;
  }
};

struct OwnerTimestampModel {
  bool known = false;
  bool active = false;
  uint32_t timestamp = 0;

  bool receive(uint32_t nowEpoch, uint32_t incomingTimestamp) {
    constexpr uint32_t kClockSkewSec = 5;
    if (incomingTimestamp == 0 ||
        incomingTimestamp > nowEpoch + kClockSkewSec) {
      known = true;
      active = false;
      return false;
    }
    if (known && incomingTimestamp < timestamp) return false;
    known = true;
    active = true;
    timestamp = incomingTimestamp;
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
  assert(evaluateEnter(true, true, true) == EnterMode::RemoteCommit);
  assert(evaluateEnter(true, false, false) == EnterMode::BlockedNoCommand);
  assert(evaluateEnter(true, true, false) == EnterMode::BlockedNoCommand);
  assert(evaluateEnter(false, true, false) == EnterMode::BlockedOfflineCommand);
  assert(evaluateEnter(false, false, false) == EnterMode::StandaloneCommit);
  assert(evaluateEnter(true, true, false, true) == EnterMode::StandaloneCommit);
  assert(evaluateEnter(false, true, false, true) == EnterMode::StandaloneCommit);
}

void testModernCommandRequiresExactFreshOwner() {
  OwnerFenceModel model;
  Weigh command{
    "command-1", "session-1", "uuid-1", 10, 125.0f, "Milk",
    "connection-1"};

  assert(!model.weigh(command));
  model.setOwner("session-1", "connection-other", 30000);
  assert(!model.weigh(command));
  model.setOwner("session-1", "connection-1", 0);
  assert(!model.weigh(command));
  model.setOwner("session-1", "connection-1", 30000);
  assert(model.weigh(command));
  assert(model.actionable());

  model.expireOwner();
  assert(model.commandActive);
  assert(!model.actionable());

  // An exact heartbeat alone never relocks the physical operator. The Manager
  // must explicitly replay the same idempotent logical command.
  model.setOwner("session-1", "connection-1", 30000);
  assert(model.commandActive);
  assert(!model.actionable());
  assert(model.weigh(command));
  assert(model.actionable());

  model.commandActive = false;
  model.command = {};

  Weigh legacy = command;
  legacy.commandId.clear();
  legacy.connectionId.clear();
  model.expireOwner();
  assert(model.weigh(legacy));
  assert(model.actionable());
}

void testModernClearUsesCurrentOwnerConnectionFence() {
  OwnerFenceModel model;
  Weigh predecessor{
    "command-1", "session-1", "uuid-1", 10, 125.0f, "Milk",
    "connection-1"};
  model.setOwner("session-1", "connection-1", 30000);
  assert(model.weigh(predecessor));

  // A reload owns the scale now. Its clear may remove the predecessor, while
  // a late pagehide callback from the predecessor may not.
  model.setOwner("session-1", "connection-2", 30000);
  assert(!model.clear("session-1", "connection-1", "command-1"));
  assert(model.commandActive);
  assert(!model.clear("session-wrong", "connection-2", "command-1"));
  assert(!model.clear("session-1", "connection-2", "command-wrong"));
  assert(model.clear("session-1", "connection-2", "command-1"));
  assert(!model.commandActive);
}

void testRetainedClearCleansRawCommandAfterOfflineOwnerExpiry() {
  OwnerFenceModel model;
  Weigh command{
    "command-1", "session-1", "uuid-1", 10, 125.0f, "Milk",
    "connection-1"};
  model.setOwner("session-1", "connection-1", 30000);
  assert(model.weigh(command));

  model.expireOwner();
  assert(model.commandActive && !model.actionable());
  assert(!model.clear("session-wrong", "connection-1", "command-1"));
  assert(!model.clear("session-1", "connection-1", "command-wrong"));
  assert(!model.clear("session-1", "", "command-1"));
  assert(model.clear("session-1", "connection-1", "command-1"));
  assert(!model.commandActive);
}

void testReconnectRequiresFreshCommandSnapshotBeforeReactivation() {
  OwnerFenceModel model;
  Weigh command{
    "command-1", "session-1", "uuid-1", 10, 125.0f, "Milk",
    "connection-1"};
  model.setOwner("session-1", "connection-1", 30000);
  assert(model.weigh(command));
  assert(model.actionable());

  model.beginReconnect();
  model.setOwner("session-1", "connection-1", 30000);
  assert(model.commandActive);
  assert(!model.actionable());

  assert(model.weigh(command));
  assert(model.actionable());
}

void testReconnectClearSnapshotSupersedesDifferentRawFence() {
  OwnerFenceModel model;
  Weigh oldCommand{
    "command-old", "session-old", "uuid-old", 10, 125.0f, "Milk",
    "connection-old"};
  model.setOwner("session-old", "connection-old", 30000);
  assert(model.weigh(oldCommand));

  model.beginReconnect();
  model.setOwner("session-new", "connection-new", 30000);
  assert(!model.actionable());
  assert(model.clear("session-new", "connection-new", "command-new"));
  assert(!model.commandActive);
}

void testOwnerExpiryNeverDropsPendingResponse() {
  OwnerFenceModel model;
  Weigh command{
    "command-1", "session-1", "uuid-1", 10, 125.0f, "Milk",
    "connection-1"};
  model.setOwner("session-1", "connection-1", 30000);
  assert(model.weigh(command));
  model.responsePending = true;

  model.expireOwner();
  assert(model.commandActive);
  assert(model.responsePending);
  assert(!model.actionable());
  assert(model.localFallback);
  assert(!model.responseOperatorLocked());
  assert(evaluateEnter(true, true, false, model.localFallback) ==
    EnterMode::StandaloneCommit);
  assert(!model.clear("session-1", "connection-1", "command-1"));
}

void testLifecycleDetachReleasesOperatorWithoutDroppingResponse() {
  OwnerFenceModel model;
  Weigh command{
    "command-1", "session-1", "uuid-1", 10, 125.0f, "Milk",
    "connection-1"};
  model.setOwner("session-1", "connection-1", 30000);
  assert(model.weigh(command));
  model.responsePending = true;
  assert(model.responseOperatorLocked());

  assert(!model.clear(
    "session-1", "connection-wrong", "command-1", true));
  assert(!model.clear(
    "session-1", "connection-1", "command-1", false));
  assert(model.responsePending && model.commandActive);

  assert(model.clear(
    "session-1", "connection-1", "command-1", true));
  assert(model.responsePending && model.commandActive);
  assert(model.localFallback && !model.responseOperatorLocked());
  assert(!model.actionable());

  OwnerFenceModel stalePage;
  stalePage.setOwner("session-1", "connection-1", 30000);
  assert(stalePage.weigh(command));
  stalePage.responsePending = true;
  stalePage.setOwner("session-2", "connection-2", 30000);
  assert(!stalePage.clear(
    "session-1", "connection-1", "command-1", true));
  assert(stalePage.responseOperatorLocked());
}

void testResponseDeliveryLockIsBounded() {
  OwnerFenceModel model;
  Weigh command{
    "command-1", "session-1", "uuid-1", 10, 125.0f, "Milk",
    "connection-1"};
  model.setOwner("session-1", "connection-1", 30000);
  assert(model.weigh(command));
  model.responsePending = true;
  assert(model.responseOperatorLocked());
  model.expireOperatorLock();
  assert(model.responsePending);
  assert(!model.responseOperatorLocked());
  assert(evaluateEnter(true, true, false, model.localFallback) ==
    EnterMode::StandaloneCommit);
}

void testPhysicalInputImmediatelyReleasesResponseLock() {
  OwnerFenceModel model;
  Weigh command{
    "command-1", "session-1", "uuid-1", 10, 125.0f, "Milk",
    "connection-1"};
  model.setOwner("session-1", "connection-1", 30000);
  assert(model.weigh(command));
  model.responsePending = true;
  assert(model.responseOperatorLocked());

  // A physical TARE/ENTER intent detaches the UI lock immediately without
  // deleting or rewriting the already-staged durable response.
  assert(model.detachForLocalInput());
  assert(model.responsePending);
  assert(!model.responseOperatorLocked());
  assert(!model.actionable());
  assert(evaluateEnter(true, true, false, model.localFallback) ==
    EnterMode::StandaloneCommit);
}

void testDetachedUndoPrecedesDeferredLocalCommit() {
  std::string stackTop = "remote-a";
  bool undoPending = true;
  bool enterDeferred = false;

  // The detach path consumes/restores the pending undo before allowing the
  // physical ENTER to create a new local top entry.
  if (undoPending) {
    stackTop.clear();
    undoPending = false;
    enterDeferred = true;
  }
  assert(stackTop.empty());
  assert(enterDeferred);
  stackTop = "local-b";
  enterDeferred = false;

  assert(stackTop == "local-b");
  assert(!undoPending);
  assert(!enterDeferred);
}

void testMissingRemoteSnapshotsCannotBlockForever() {
  assert(evaluateFallback(
    true, false, false, false, false, false, false, false) ==
    FallbackDecision::Grace);
  assert(evaluateFallback(
    true, false, false, false, true, true, true, false) ==
    FallbackDecision::Grace);
  assert(evaluateFallback(
    true, true, true, true, true, true, false, false) ==
    FallbackDecision::Grace);
  assert(evaluateFallback(
    true, true, true, true, true, true, true, false) ==
    FallbackDecision::None);
  assert(evaluateFallback(
    true, true, true, false, true, true, true, false) ==
    FallbackDecision::Immediate);

  // Legacy commands never depended on the owner topic while connected.
  assert(evaluateFallback(
    true, false, false, false, true, false, true, false) ==
    FallbackDecision::None);
  // Any retained command is detached after a bounded offline grace.
  assert(evaluateFallback(
    false, false, false, false, true, false, false, false) ==
    FallbackDecision::Grace);
  assert(evaluateFallback(
    true, true, true, true, true, false, true, true, true) ==
    FallbackDecision::Immediate);
}

void testAsyncReconnectKeepsTransportOwnershipExclusive() {
  ConnectWorkerModel worker;
  assert(worker.mainOwnsTransport());
  assert(worker.start());
  assert(!worker.mainOwnsTransport());
  assert(!worker.start());

  // Aborting is cooperative: the application never touches the transport
  // until the worker has completed and its result has been consumed.
  assert(worker.requestAbort());
  worker.finish(true);
  assert(!worker.mainOwnsTransport());
  bool connected = true;
  bool aborted = false;
  assert(worker.consume(connected, aborted));
  assert(!connected);
  assert(aborted);
  assert(worker.mainOwnsTransport());

  assert(worker.start());
  worker.finish(true);
  assert(worker.consume(connected, aborted));
  assert(connected);
  assert(!aborted);
  assert(worker.mainOwnsTransport());
}

void testDetachedModeRejectsStaleRetainedReactivation() {
  assert(!fallbackReattachAllowed(
    true, "connection-1", 100, 100, "connection-1"));
  assert(fallbackReattachAllowed(
    true, "connection-1", 101, 100, "connection-1"));
  assert(fallbackReattachAllowed(
    true, "connection-2", 100, 100, "connection-1"));
  assert(fallbackReattachAllowed(
    true, "", 100, 100, "connection-1"));

  // A transport reset clears current freshness, not the last-seen evidence
  // captured as the fallback baseline.
  const uint32_t currentSnapshotTimestamp = 0;
  const uint32_t lastSeenTimestamp = 100;
  const std::string currentSnapshotConnection;
  const std::string lastSeenConnection = "connection-1";
  const uint32_t fallbackTimestamp = currentSnapshotTimestamp != 0
    ? currentSnapshotTimestamp : lastSeenTimestamp;
  const std::string fallbackConnection = !currentSnapshotConnection.empty()
    ? currentSnapshotConnection : lastSeenConnection;
  assert(!fallbackReattachAllowed(
    true, "connection-1", 100, fallbackTimestamp, fallbackConnection));
}

void testDetachedCommandDoesNotBlockSleep() {
  assert(commandBlocksSleep(true, false, false, false));
  assert(commandBlocksSleep(true, true, true, false));
  assert(!commandBlocksSleep(true, false, false, true));
  assert(!commandBlocksSleep(true, true, false, true));
}

void testTopBarSeparatesTransportFromManagerMode() {
  TopBarState state = evaluateTopBar(true, true, false);
  assert(state.mqttConnected);
  assert(state.managerAttached);

  state = evaluateTopBar(true, true, true);
  assert(state.mqttConnected);
  assert(!state.managerAttached);

  state = evaluateTopBar(true, false, false);
  assert(state.mqttConnected);
  assert(!state.managerAttached);

  state = evaluateTopBar(false, true, false);
  assert(!state.mqttConnected);
  assert(!state.managerAttached);
}

void testIdlePagehideDetachesExactOwner() {
  OwnerFenceModel model;
  model.setOwner("session-1", "connection-1", 30000);
  assert(!model.commandActive);
  assert(!model.clear(
    "session-1", "connection-old", "command-old", true));
  assert(model.clear(
    "session-1", "connection-1", "command-old", true));
  assert(model.localFallback);

  OwnerFenceModel legacy;
  legacy.setOwner("session-2", "connection-2", 30000);
  legacy.commandActive = true;
  legacy.command = Weigh{
    "", "", "uuid-legacy", 12, 50.0f, "Legacy", ""};
  assert(legacy.clear(
    "session-2", "connection-2", "command-current", true));
  assert(legacy.localFallback);
  assert(legacy.commandActive);
}

void testFutureOwnerTimestampCannotPoisonSnapshot() {
  OwnerTimestampModel model;
  constexpr uint32_t nowEpoch = 1784280000;
  assert(model.receive(nowEpoch, nowEpoch));
  assert(model.known && model.active && model.timestamp == nowEpoch);
  assert(!model.receive(nowEpoch, nowEpoch + 60));
  assert(model.known && !model.active && model.timestamp == nowEpoch);
  assert(model.receive(nowEpoch + 1, nowEpoch + 1));
  assert(model.active && model.timestamp == nowEpoch + 1);
  assert(!model.receive(nowEpoch, nowEpoch - 1));
  assert(model.active && model.timestamp == nowEpoch + 1);
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

  ProtocolModel connectionModel;
  Weigh connection = valid;
  connection.connectionId = "bad\"connection";
  assert(connectionModel.weigh(connection) == WeighResult::Invalid);
  connection.connectionId = "connection-1";
  assert(connectionModel.weigh(connection) == WeighResult::Activated);
  connection.connectionId = "bad\\connection";
  assert(connectionModel.weigh(connection) == WeighResult::Invalid);
  assert(connectionModel.command.connectionId == "connection-1");
}

void testRemoteConfirmRequestIsFencedAndIdempotent() {
  ProtocolModel model;
  Weigh command{"command-1", "session-1", "uuid-1", 10, 125.0f, "Milk"};
  command.connectionId = "connection-1";
  assert(model.weigh(command) == WeighResult::Activated);

  const ConfirmRequest request{
    "request-1", "session-1", "connection-1", "command-1", "uuid-1", 10};
  assert(model.confirmRequest(request) == ConfirmRequestResult::Accepted);
  assert(model.confirmRequest(request) == ConfirmRequestResult::AcceptedReplay);
  assert(!model.clear("session-1", "command-1"));
  assert(model.popConfirmRequest());
  assert(!model.popConfirmRequest());
  assert(model.confirmExecutions == 1);

  ConfirmRequest second = request;
  second.requestId = "request-2";
  assert(model.confirmRequest(second) == ConfirmRequestResult::RejectedBusy);

  model.failConfirmRequest();
  assert(model.confirmRequest(request) == ConfirmRequestResult::FailedReplay);
  assert(model.confirmRequest(second) == ConfirmRequestResult::RejectedBusy);
  ConfirmRequest third = request;
  third.requestId = "request-3";
  assert(model.confirmRequest(third) == ConfirmRequestResult::Accepted);
  assert(model.popConfirmRequest());
  assert(model.confirmExecutions == 2);
  assert(model.confirmRequest(request) == ConfirmRequestResult::FailedReplay);
  assert(model.confirmRequest(second) == ConfirmRequestResult::RejectedBusy);
  assert(model.confirmExecutions == 2);
}

void testRemoteConfirmStagedRequestCannotRunTwice() {
  ProtocolModel model;
  Weigh command{"command-1", "session-1", "uuid-1", 10, 125.0f, "Milk"};
  command.connectionId = "connection-1";
  assert(model.weigh(command) == WeighResult::Activated);
  const ConfirmRequest request{
    "request-1", "session-1", "connection-1", "command-1", "uuid-1", 10};
  assert(model.confirmRequest(request) == ConfirmRequestResult::Accepted);
  assert(model.popConfirmRequest());
  model.stageResponse("response-1", "immutable-confirm");
  model.markConfirmRequestStaged();

  assert(model.confirmRequest(request) == ConfirmRequestResult::StagedReplay);
  ConfirmRequest other = request;
  other.requestId = "request-2";
  assert(model.confirmRequest(other) == ConfirmRequestResult::RejectedBusy);
  assert(model.responsePayload == "immutable-confirm");
  assert(model.confirmExecutions == 1);
  assert(!model.responseAck("wrong-response"));
  assert(model.confirmRequest(request) == ConfirmRequestResult::StagedReplay);
  assert(model.responseAck("response-1"));
  assert(model.confirmRequest(request) == ConfirmRequestResult::RejectedStale);
  assert(model.confirmRequest(other) == ConfirmRequestResult::RejectedBusy);
}

void testRemoteConfirmConnectionRetargetDoesNotReuseRequest() {
  ProtocolModel model;
  Weigh command{"command-1", "session-1", "uuid-1", 10, 125.0f, "Milk"};
  command.connectionId = "connection-1";
  assert(model.weigh(command) == WeighResult::Activated);
  ConfirmRequest request{
    "request-1", "session-1", "connection-1", "command-1", "uuid-1", 10};
  assert(model.confirmRequest(request) == ConfirmRequestResult::Accepted);

  Weigh reload = command;
  reload.connectionId = "connection-2";
  assert(model.weigh(reload) == WeighResult::IgnoredPending);
  assert(model.command.connectionId == "connection-1");
  assert(model.popConfirmRequest());
  model.failConfirmRequest();
  assert(model.weigh(reload) == WeighResult::Duplicate);
  assert(model.command.connectionId == "connection-2");
  assert(model.confirmRequest(request) == ConfirmRequestResult::FailedReplay);

  request.connectionId = "connection-2";
  assert(model.confirmRequest(request) == ConfirmRequestResult::RejectedConflict);
  assert(model.confirmExecutions == 1);
}

void testRemoteConfirmTerminalRejectSurvivesFenceChange() {
  ProtocolModel model;
  Weigh command{"command-1", "session-1", "uuid-1", 10, 125.0f, "Milk"};
  command.connectionId = "connection-1";
  assert(model.weigh(command) == WeighResult::Activated);

  ConfirmRequest stale{
    "request-1", "session-1", "connection-2", "command-1", "uuid-1", 10};
  assert(model.confirmRequest(stale) == ConfirmRequestResult::RejectedStale);

  Weigh reload = command;
  reload.connectionId = "connection-2";
  assert(model.weigh(reload) == WeighResult::Duplicate);
  assert(model.confirmRequest(stale) == ConfirmRequestResult::RejectedStale);

  ConfirmRequest conflict = stale;
  conflict.uuid = "uuid-2";
  assert(model.confirmRequest(conflict) == ConfirmRequestResult::RejectedConflict);
  assert(model.confirmRequest(stale) == ConfirmRequestResult::RejectedStale);
  assert(model.confirmExecutions == 0);
}

void testRemoteConfirmTerminalCacheEvictsOnlyTheOldestRequest() {
  ProtocolModel model;
  Weigh command{"command-1", "session-1", "uuid-1", 10, 125.0f, "Milk"};
  command.connectionId = "connection-1";
  assert(model.weigh(command) == WeighResult::Activated);

  ConfirmRequest first{
    "request-1", "session-1", "connection-2", "command-1", "uuid-1", 10};
  ConfirmRequest second = first;
  second.requestId = "request-2";
  second.connectionId = "connection-3";
  ConfirmRequest third = first;
  third.requestId = "request-3";
  third.connectionId = "connection-4";
  assert(model.confirmRequest(first) == ConfirmRequestResult::RejectedStale);
  assert(model.confirmRequest(second) == ConfirmRequestResult::RejectedStale);
  assert(model.confirmRequest(third) == ConfirmRequestResult::RejectedStale);

  Weigh reload = command;
  reload.connectionId = "connection-2";
  assert(model.weigh(reload) == WeighResult::Duplicate);
  assert(model.confirmRequest(first) == ConfirmRequestResult::Accepted);
  assert(model.confirmRequest(second) == ConfirmRequestResult::RejectedStale);
  assert(model.confirmRequest(third) == ConfirmRequestResult::RejectedStale);
  assert(model.popConfirmRequest());
  assert(model.confirmExecutions == 1);
}

void testRemoteConfirmRejectsStaleOrInvalidFence() {
  ProtocolModel model;
  Weigh command{"command-1", "session-1", "uuid-1", 10, 125.0f, "Milk"};
  command.connectionId = "connection-1";
  assert(model.weigh(command) == WeighResult::Activated);
  ConfirmRequest request{
    "request-1", "session-1", "connection-1", "command-1", "uuid-1", 10};

  ConfirmRequest stale = request;
  stale.sessionId = "session-2";
  assert(model.confirmRequest(stale) == ConfirmRequestResult::RejectedStale);
  stale = request;
  stale.requestId = "request-2";
  stale.commandId = "command-2";
  assert(model.confirmRequest(stale) == ConfirmRequestResult::RejectedStale);
  stale = request;
  stale.requestId = "request-3";
  stale.uuid = "uuid-2";
  assert(model.confirmRequest(stale) == ConfirmRequestResult::RejectedStale);
  stale = request;
  stale.requestId = "request-4";
  stale.productId = 11;
  assert(model.confirmRequest(stale) == ConfirmRequestResult::RejectedStale);
  stale = request;
  stale.requestId = "short";
  assert(model.confirmRequest(stale) == ConfirmRequestResult::Invalid);
  assert(model.confirmExecutions == 0);
}

void testMaximumPayloadsFitBuffers() {
  const std::string commandId(64, 'c');
  const std::string sessionId(64, 's');
  const std::string uuid(36, 'u');
  const std::string responseId(24, 'r');
  const std::string connectionId(64, 'n');
  const std::string requestId(64, 'q');
  const std::string topic = "minu/scale/123456789abc/response";
  const std::string commandTopic = "minu/scale/123456789abc/command";
  const std::string ownerTopic = "minu/scale/123456789abc/owner";

  const std::string commandAck =
    "{\"type\":\"command_ack\",\"command_id\":\"" + commandId +
    "\",\"session_id\":\"" + sessionId + "\",\"connection_id\":\"" +
    connectionId + "\",\"uuid\":\"" + uuid +
    "\",\"product_id\":4294967295,\"state\":\"active\"}";
  const std::string confirm =
    "{\"type\":\"confirm\",\"uuid\":\"" + uuid +
    "\",\"product_id\":4294967295,\"session_id\":\"" + sessionId +
    "\",\"command_id\":\"" + commandId + "\",\"response_id\":\"" +
    responseId + "\",\"actual_weight\":-123456.7}";
  const std::string confirmRequest =
    "{\"type\":\"confirm_request\",\"request_id\":\"" + requestId +
    "\",\"session_id\":\"" + sessionId + "\",\"connection_id\":\"" +
    connectionId + "\",\"command_id\":\"" + commandId +
    "\",\"uuid\":\"" + uuid + "\",\"product_id\":4294967295}";
  const std::string confirmRequestAck =
    "{\"type\":\"confirm_request_ack\",\"request_id\":\"" + requestId +
    "\",\"command_id\":\"" + commandId + "\",\"connection_id\":\"" +
    connectionId +
    "\",\"state\":\"rejected\",\"reason\":\"context_changed\"}";
  const std::string owner =
    // 48 UTF-16 characters can occupy up to 144 UTF-8 bytes for valid BMP
    // names after the Manager strips control characters.
    "{\"user_id\":4294967295,\"user_name\":\"" + std::string(144, 'u') +
    "\",\"session_id\":\"" + sessionId + "\",\"connection_id\":\"" +
    connectionId + "\",\"lease_id\":\"" + std::string(64, 'l') +
    "\",\"timestamp\":4294967295}";

  assert(commandAck.size() < 384);
  assert(confirm.size() < 384);
  assert(confirmRequestAck.size() < 384);
  // MQTT_MAX_PACKET_SIZE also includes topic and packet framing.
  assert(topic.size() + commandAck.size() + 8 < 512);
  assert(topic.size() + confirm.size() + 8 < 512);
  assert(topic.size() + confirmRequestAck.size() + 8 < 512);
  assert(commandTopic.size() + confirmRequest.size() + 8 < 512);
  assert(ownerTopic.size() + owner.size() + 8 < 512);
}

} // namespace

int main() {
  testFencedClear();
  testEnterRequiresCommandOnlyWhileConnected();
  testModernCommandRequiresExactFreshOwner();
  testModernClearUsesCurrentOwnerConnectionFence();
  testRetainedClearCleansRawCommandAfterOfflineOwnerExpiry();
  testReconnectRequiresFreshCommandSnapshotBeforeReactivation();
  testReconnectClearSnapshotSupersedesDifferentRawFence();
  testOwnerExpiryNeverDropsPendingResponse();
  testLifecycleDetachReleasesOperatorWithoutDroppingResponse();
  testResponseDeliveryLockIsBounded();
  testPhysicalInputImmediatelyReleasesResponseLock();
  testDetachedUndoPrecedesDeferredLocalCommit();
  testMissingRemoteSnapshotsCannotBlockForever();
  testAsyncReconnectKeepsTransportOwnershipExclusive();
  testDetachedModeRejectsStaleRetainedReactivation();
  testDetachedCommandDoesNotBlockSleep();
  testTopBarSeparatesTransportFromManagerMode();
  testIdlePagehideDetachesExactOwner();
  testFutureOwnerTimestampCannotPoisonSnapshot();
  testDuplicateIsIdempotentAndReacked();
  testDuplicateNormalizesLongName();
  testPendingResponseIsByteImmutable();
  testCommandIdValidation();
  testRemoteConfirmRequestIsFencedAndIdempotent();
  testRemoteConfirmStagedRequestCannotRunTwice();
  testRemoteConfirmConnectionRetargetDoesNotReuseRequest();
  testRemoteConfirmTerminalRejectSurvivesFenceChange();
  testRemoteConfirmTerminalCacheEvictsOnlyTheOldestRequest();
  testRemoteConfirmRejectsStaleOrInvalidFence();
  testMaximumPayloadsFitBuffers();
  std::cout << "firmware MQTT protocol harness: OK\n";
  return 0;
}
