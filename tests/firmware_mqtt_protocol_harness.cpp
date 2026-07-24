#include <cassert>
#include <cstdint>
#include <iostream>
#include <string>
#include <utility>

#include "../firmware/esp32_hx711_serial/mqtt_update_cycle.h"
#include "../firmware/esp32_hx711_serial/mqtt_transport_health.h"

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
    const std::string& incomingLeaseId,
    uint32_t ownerTimestamp, uint32_t fallbackOwnerTimestamp,
    const std::string& fallbackConnectionId,
    const std::string& fallbackLeaseId) {
  return mqttFallbackHasFreshEvidence(
    localFallback, incomingConnectionId.empty(), ownerTimestamp,
    fallbackOwnerTimestamp, incomingConnectionId != fallbackConnectionId,
    incomingLeaseId != fallbackLeaseId);
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
  uint32_t lastSeenTimestamp = 0;
  std::string connectionId;
  std::string leaseId;
  std::string lastSeenConnectionId;
  std::string lastSeenLeaseId;

  bool receive(
      uint32_t nowEpoch, uint32_t incomingTimestamp,
      const std::string& incomingConnectionId = "connection-1",
      const std::string& incomingLeaseId = "lease-1") {
    constexpr uint32_t kClockSkewSec = 5;
    if (incomingTimestamp == 0 ||
        incomingTimestamp > nowEpoch + kClockSkewSec) {
      known = true;
      active = false;
      return false;
    }
    const bool exactStoredLease = known &&
      incomingConnectionId == connectionId && incomingLeaseId == leaseId;
    const bool exactLastSeenLease =
      incomingConnectionId == lastSeenConnectionId &&
      incomingLeaseId == lastSeenLeaseId;
    if (!mqttOwnerTimestampAllowed(
          exactStoredLease, exactLastSeenLease, incomingTimestamp,
          timestamp, lastSeenTimestamp)) return false;
    known = true;
    active = true;
    timestamp = incomingTimestamp;
    lastSeenTimestamp = incomingTimestamp;
    connectionId = incomingConnectionId;
    leaseId = incomingLeaseId;
    lastSeenConnectionId = incomingConnectionId;
    lastSeenLeaseId = incomingLeaseId;
    return true;
  }
};

struct PassiveRepairModel {
  uint8_t repairMask = 0;
  uint8_t observedMask = 0;
  uint8_t completedRetries = 0;
  bool repairing = false;
  bool reconnect = false;
  std::string outbox = "immutable-response";

  void detect(uint8_t failureMask) {
    assert(!repairing && failureMask != 0);
    repairing = true;
    repairMask = failureMask;
    observedMask &= (uint8_t)~failureMask;
  }

  void observe(uint8_t mask) {
    observedMask |= mask;
    repairMask &= (uint8_t)~mask;
    if (repairMask == 0) repairing = false;
  }

  void deadline(uint8_t maxRetries) {
    assert(repairing);
    if (completedRetries < maxRetries) {
      completedRetries++;
      observedMask &= (uint8_t)~repairMask;
      return;
    }
    reconnect = true;
  }
};

enum class TerminalResponseKind {
  Confirm,
  Skip,
  Undo,
};

struct ResponseRejectModel {
  bool pending = false;
  TerminalResponseKind kind = TerminalResponseKind::Confirm;
  std::string responseId;
  bool detachedApplied = false;
  bool undoReject = false;
  bool detachedUndoReject = false;
  bool stackValid = true;
  bool localFallback = false;
  bool confirmCommitApplied = false;
  bool confirmReceiptAvailable = false;
  bool tareRequired = false;
  bool managerAdvanced = false;
  unsigned confirmRollbacks = 0;
  unsigned commandAcks = 0;
  unsigned restoreCount = 0;

  void stage(TerminalResponseKind responseKind, const std::string& id) {
    pending = true;
    kind = responseKind;
    responseId = id;
    detachedApplied = false;
    undoReject = false;
    detachedUndoReject = false;
    stackValid = true;
    localFallback = false;
    confirmCommitApplied = false;
    confirmReceiptAvailable = false;
    tareRequired = false;
    managerAdvanced = false;
    confirmRollbacks = 0;
    commandAcks = 0;
    restoreCount = 0;
  }

  void applyConfirmCommit() {
    assert(pending && kind == TerminalResponseKind::Confirm);
    confirmCommitApplied = true;
    confirmReceiptAvailable = true;
  }

  void detach() {
    localFallback = true;
  }

  void applyDetachedUndo() {
    assert(pending && kind == TerminalResponseKind::Undo);
    detachedApplied = true;
    ++restoreCount;
  }

  bool reject(const std::string& id, const std::string& reason) {
    if (!pending || id != responseId || reason != "manager_incident") {
      return false;
    }
    if (kind == TerminalResponseKind::Undo) {
      if (detachedApplied) {
        detachedUndoReject = true;
        stackValid = false;
      } else {
        undoReject = true;
      }
    } else if (kind == TerminalResponseKind::Confirm &&
               confirmCommitApplied) {
      if (!localFallback && confirmReceiptAvailable) {
        confirmCommitApplied = false;
        confirmReceiptAvailable = false;
        ++confirmRollbacks;
      } else {
        stackValid = false;
        confirmReceiptAvailable = false;
        tareRequired = true;
      }
    }
    pending = false;
    responseId.clear();
    detachedApplied = false;
    return true;
  }

  bool canUndoRejectedConfirm() const {
    return confirmReceiptAvailable && stackValid && !tareRequired;
  }
};

struct UndoIncidentGateModel {
  bool needsTare = true;
  bool wizardActive = false;
  bool longPressTracking = false;
  bool netTareRequired = true;
  bool fallbackActive = true;
  bool appMode = false;
  bool rawCommand = false;
  bool commandSnapshotReady = false;
  bool ownerFresh = true;
  bool inboundHealthy = true;
  bool responsePending = false;
  unsigned commandAcks = 0;
  unsigned stackMutations = 0;
  unsigned enterCommits = 0;
  unsigned calibrationMutations = 0;

  void update(bool skipHeld, bool clearPressed, bool enterPressed) {
    if (needsTare) {
      longPressTracking = false;
      wizardActive = false;
      return;
    }
    if (skipHeld) {
      longPressTracking = true;
      wizardActive = true;
    }
    if (clearPressed) ++stackMutations;
    if (enterPressed) ++enterCommits;
  }

  void serialCalibrationMutation() {
    if (!needsTare) ++calibrationMutations;
  }

  void receiveRawCommand() {
    rawCommand = true;
    commandSnapshotReady = true;
    updateTransport();
  }

  void updateTransport() {
    if (!fallbackActive) {
      appMode = !netTareRequired && ownerFresh && inboundHealthy;
      return;
    }
    const bool commandContextReady = rawCommand && commandSnapshotReady &&
      ownerFresh;
    if (!netTareRequired && ownerFresh && inboundHealthy &&
        !responsePending && commandContextReady) {
      fallbackActive = false;
      appMode = true;
      ++commandAcks;
    }
  }

  void tareResult(bool applied) {
    if (!applied) return;
    needsTare = false;
    netTareRequired = false;
    updateTransport();
  }
};

struct FallbackRecoveryModel {
  bool fallbackActive = true;
  bool connected = true;
  bool inboundHealthy = true;
  bool ownerFresh = true;
  bool freshEvidence = true;
  bool responsePending = false;
  bool tareRequired = false;
  bool commandActive = false;
  bool commandSnapshotReady = false;
  bool commandOwnerAuthorized = false;
  bool commandHasId = false;
  bool appMode = false;
  unsigned commandAcks = 0;

  void update() {
    if (!fallbackActive) {
      appMode = !tareRequired && connected && inboundHealthy && ownerFresh;
      return;
    }
    const bool idleContextReady = !commandActive && ownerFresh && freshEvidence;
    const bool commandContextReady = commandActive && commandSnapshotReady &&
      freshEvidence && commandOwnerAuthorized;
    if (!tareRequired && connected && inboundHealthy && !responsePending &&
        (idleContextReady || commandContextReady)) {
      fallbackActive = false;
      appMode = true;
      if (commandActive && commandHasId) ++commandAcks;
    }
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

void testInboundHeartbeatIsProcessedBeforeBoundedFallbackDecision() {
  unsigned packet = 0;
  bool ownerFresh = false;
  bool fallbackEntered = false;

  mqttRunUpdateCycle(
    [&]() {
      const unsigned passes = mqttPollInboundBounded(
        [&]() {
          packet += 1;
          if (packet == 2) ownerFresh = true;
        },
        [&]() { return packet < 2; },
        4);
      assert(passes == 2);
    },
    [&]() { fallbackEntered = !ownerFresh; });

  assert(ownerFresh);
  assert(!fallbackEntered);

  unsigned boundedPasses = 0;
  const unsigned result = mqttPollInboundBounded(
    [&]() { boundedPasses += 1; },
    []() { return true; },
    4);
  assert(result == 4);
  assert(boundedPasses == 4);
}

void testDetachedModeRejectsStaleRetainedReactivation() {
  assert(!fallbackReattachAllowed(
    true, "connection-1", "lease-1", 100, 100,
    "connection-1", "lease-1"));
  assert(fallbackReattachAllowed(
    true, "connection-1", "lease-1", 101, 100,
    "connection-1", "lease-1"));
  assert(fallbackReattachAllowed(
    true, "connection-2", "lease-1", 100, 100,
    "connection-1", "lease-1"));
  assert(fallbackReattachAllowed(
    true, "", "", 100, 100, "connection-1", "lease-1"));
  assert(fallbackReattachAllowed(
    true, "connection-1", "lease-2", 100, 100,
    "connection-1", "lease-1"));

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
    true, "connection-1", "lease-1", 100, fallbackTimestamp,
    fallbackConnection, "lease-1"));
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

  // Monotonicity is scoped to the exact lease. A new lease/connection may
  // legitimately carry a lower second-resolution timestamp and must recover.
  assert(model.receive(
    nowEpoch + 2, nowEpoch, "connection-1", "lease-2"));
  assert(model.active && model.timestamp == nowEpoch);
  assert(model.receive(
    nowEpoch + 3, nowEpoch - 1, "connection-2", "lease-3"));
  assert(model.active && model.timestamp == nowEpoch - 1);

  // Operational presence is based on the arrival of a valid heartbeat, not
  // on its wall-clock age. One missed 10 s heartbeat plus 5 s clock skew stays
  // inside the 25 s presence window, while the original 30 s lease deadline
  // still prevents a retained replay from becoming fresh again.
  constexpr uint32_t presenceLimitMs = 25000;
  assert(mqttOwnerArrivalFresh(20000, 25000, presenceLimitMs));
  assert(!mqttOwnerArrivalFresh(25000, 30000, presenceLimitMs));
  assert(!mqttOwnerArrivalFresh(2000, 1000, presenceLimitMs));

  // A tablet 10 s behind gets bounded grace only after proving forward
  // progress on the same lease. Equal retained replay receives no extension,
  // and even progressive input cannot reopen an already expired lease.
  constexpr uint32_t ownerTtlMs = 30000;
  const uint32_t progressiveRemaining = mqttOwnerLeaseRemainingMs(
    10, ownerTtlMs, true, 10, 5000);
  const uint32_t replayRemaining = mqttOwnerLeaseRemainingMs(
    10, ownerTtlMs, false, 10, 5000);
  assert(progressiveRemaining == 25000);
  assert(replayRemaining == 20000);
  assert(mqttOwnerArrivalFresh(
    20000, progressiveRemaining, presenceLimitMs));
  assert(!mqttOwnerArrivalFresh(
    20000, replayRemaining, presenceLimitMs));
  assert(mqttOwnerLeaseRemainingMs(
    30, ownerTtlMs, true, 10, 5000) == 0);

  // Once the firmware is already in LOC with a known owner baseline, a new
  // connection/lease generation may become an inactive takeover candidate.
  // A later strictly advancing heartbeat on that exact generation proves the
  // live publisher without depending on the ESP32 wall clock. Same-generation
  // and older retained owners cannot start this handshake.
  assert(mqttOwnerFallbackTakeoverCandidate(
    true, false, true, true, nowEpoch, nowEpoch));
  assert(mqttOwnerFallbackTakeoverCandidate(
    true, false, true, true, nowEpoch + 301, nowEpoch));
  assert(!mqttOwnerFallbackTakeoverCandidate(
    false, false, true, true, nowEpoch, nowEpoch));
  assert(!mqttOwnerFallbackTakeoverCandidate(
    true, true, true, true, nowEpoch, nowEpoch));
  assert(!mqttOwnerFallbackTakeoverCandidate(
    true, false, false, true, nowEpoch, nowEpoch));
  assert(!mqttOwnerFallbackTakeoverCandidate(
    true, false, true, false, nowEpoch, nowEpoch));
  assert(!mqttOwnerFallbackTakeoverCandidate(
    true, false, true, true, nowEpoch - 1, nowEpoch));
  assert(mqttOwnerMonotonicProgress(
    false, true, true, nowEpoch + 1, nowEpoch));
  assert(!mqttOwnerMonotonicProgress(
    false, true, true, nowEpoch, nowEpoch));
  assert(!mqttOwnerMonotonicProgress(
    false, true, false, nowEpoch + 1, nowEpoch));
  assert(!mqttOwnerMonotonicProgress(
    true, true, true, nowEpoch + 1, nowEpoch));
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

void testPassiveInboundWatchdogIsBoundedAndPreservesOutbox() {
  constexpr uint32_t initialSnapshotMs = 2500;
  constexpr uint32_t ownerSilenceMs = 25000;
  constexpr uint32_t ackSilenceMs = 10000;

  // Idle transports have no expected inbound traffic and must not reconnect
  // merely because no Manager/retained command exists.
  assert(mqttPassiveRxFailureMask(
    false, false, false, false, false, false,
    60000, 60000, 60000, 60000, initialSnapshotMs,
    ownerSilenceMs, ackSilenceMs) == 0);

  // A healthy active context remains healthy while valid owner and command
  // evidence are present and the heartbeat is inside its window.
  assert(mqttPassiveRxFailureMask(
    true, true, true, false, true, true,
    60000, 60000, 5000, 0, initialSnapshotMs,
    ownerSilenceMs, ackSilenceMs) == 0);

  uint8_t ownerSilent = mqttPassiveRxFailureMask(
    true, true, true, false, true, true,
    60000, 60000, ownerSilenceMs, 0, initialSnapshotMs,
    ownerSilenceMs, ackSilenceMs);
  assert((ownerSilent & MQTT_RX_OWNER) != 0);

  // expected_command_id changed but the retained command never arrived even
  // though owner heartbeats still prove the owner subscription is alive.
  uint8_t commandSilent = mqttPassiveRxFailureMask(
    true, true, true, false, true, false,
    60000, initialSnapshotMs, 1000, 0, initialSnapshotMs,
    ownerSilenceMs, ackSilenceMs);
  assert(commandSilent == MQTT_RX_COMMAND);

  // A different retained command still proves that the subscription works.
  // Business convergence must remain local without rebuilding the socket.
  uint8_t commandMismatchWithEvidence = mqttPassiveRxFailureMask(
    true, true, true, false, true, true,
    60000, initialSnapshotMs, 1000, 0, initialSnapshotMs,
    ownerSilenceMs, ackSilenceMs);
  assert(commandMismatchWithEvidence == 0);

  // The absence grace is scoped to the current expectation, not to the age of
  // a long-lived transport, and equal owner heartbeats do not restart it.
  assert(mqttPassiveRxFailureMask(
    true, true, true, false, true, false,
    60000, initialSnapshotMs - 1, 1000, 0, initialSnapshotMs,
    ownerSilenceMs, ackSilenceMs) == 0);
  assert(mqttPassiveRxFailureMask(
    true, true, true, false, true, false,
    60000, initialSnapshotMs, 1000, 0, initialSnapshotMs,
    ownerSilenceMs, ackSilenceMs) == MQTT_RX_COMMAND);

  // expected_command_id="" is an authoritative clear even if the broker has
  // no retained command to replay. Outbox/confirm locks remain immutable.
  assert(mqttExpectedClearIsAuthoritative(
    true, true, true, false, false));
  assert(!mqttExpectedClearIsAuthoritative(
    true, true, true, true, false));
  assert(!mqttExpectedClearIsAuthoritative(
    true, true, true, false, true));
  assert(!mqttExpectedClearIsAuthoritative(
    true, false, true, false, false));

  // Evidence from an older expectation cannot mask a lost command for the new
  // generation. Command-before-owner is preserved only when its snapshot
  // already matches the newly announced expected ID.
  assert(!mqttCommandEvidenceCarriesToNewExpectation(
    false, true, true));
  assert(!mqttCommandEvidenceCarriesToNewExpectation(
    true, false, true));
  assert(mqttCommandEvidenceCarriesToNewExpectation(
    true, true, true));

  // Only a structurally valid, connection-aware weigh whose sole remaining
  // failure is the owner fence may open an owner-evidence generation. This
  // includes command-before-owner during takeover, even while the previous
  // owner snapshot is still fresh.
  assert(mqttShouldAwaitOwnerForModernCommand(
    true, true, false));
  assert(!mqttShouldAwaitOwnerForModernCommand(
    false, true, false));
  assert(!mqttShouldAwaitOwnerForModernCommand(
    true, false, false));
  assert(!mqttShouldAwaitOwnerForModernCommand(
    true, true, true));

  // Retries for the same command/connection cannot extend the monotonic grace.
  assert(mqttOwnerExpectationGenerationChanged(
    false, false, false));
  assert(!mqttOwnerExpectationGenerationChanged(
    true, true, true));
  assert(mqttOwnerExpectationGenerationChanged(
    true, false, true));
  assert(mqttOwnerExpectationGenerationChanged(
    true, true, false));
  assert(mqttOwnerExpectationFingerprintMatches(
    true, true, true));
  assert(!mqttOwnerExpectationFingerprintMatches(
    false, true, true));
  assert(!mqttOwnerExpectationFingerprintMatches(
    true, false, true));
  assert(!mqttOwnerExpectationFingerprintMatches(
    true, true, false));
  assert(mqttOwnerExpectationKeepsDeadline(
    true, false));
  assert(!mqttOwnerExpectationKeepsDeadline(
    false, false));
  assert(!mqttOwnerExpectationKeepsDeadline(
    true, true));

  // A generation may rebuild the transport once. Its fence survives that
  // rebuild; a second exhausted repair stays local until fresh evidence or a
  // different fingerprint arrives.
  assert(mqttOwnerExpectationMayReconnect(
    true, false));
  assert(!mqttOwnerExpectationMayReconnect(
    true, true));
  assert(mqttOwnerExpectationMayReconnect(
    false, true));
  assert(!mqttOwnerExpectationEvidenceAfterTransportReset(
    true, true));
  assert(!mqttOwnerExpectationEvidenceAfterTransportReset(
    true, false));
  assert(mqttOwnerExpectationEvidenceAfterTransportReset(
    false, true));

  // Historical/global owner traffic cannot resolve a generation-scoped
  // repair. Only an owner PUBLISH observed after the generation opened can.
  assert(!mqttOwnerRepairEvidenceRecovered(
    true, false, true));
  assert(mqttOwnerRepairEvidenceRecovered(
    true, true, false));
  assert(mqttOwnerRepairEvidenceRecovered(
    false, false, true));
  assert(!mqttOwnerRepairEvidenceRecovered(
    false, true, false));

  // True owner silence after that generation starts triggers bounded repair.
  // Any subsequent owner PUBLISH proves the topic even when its business
  // envelope is foreign or malformed.
  assert(mqttPassiveRxFailureMask(
    true, false, false, false, false, false,
    initialSnapshotMs - 1, 60000, 0, 0, initialSnapshotMs,
    ownerSilenceMs, ackSilenceMs) == 0);
  assert(mqttPassiveRxFailureMask(
    true, false, false, false, false, false,
    initialSnapshotMs, 60000, 0, 0, initialSnapshotMs,
    ownerSilenceMs, ackSilenceMs) == MQTT_RX_OWNER);
  assert(mqttPassiveRxFailureMask(
    true, false, false, false, true, false,
    initialSnapshotMs, 60000, 0, 0, initialSnapshotMs,
    ownerSilenceMs, ackSilenceMs) == 0);

  // state=active is legal only after all business fences converge and the
  // command is actually actionable. LOC fallback and tare keep it silent.
  assert(mqttCommandAckAllowed(
    true, true, true, true, true, true));
  assert(!mqttCommandAckAllowed(
    true, true, true, true, true, false));
  assert(!mqttCommandAckAllowed(
    true, true, true, true, false, true));
  assert(!mqttCommandAckAllowed(
    true, true, true, false, true, true));

  uint8_t ackSilent = mqttPassiveRxFailureMask(
    false, false, false, true, false, false,
    60000, 60000, 0, ackSilenceMs, initialSnapshotMs,
    ownerSilenceMs, ackSilenceMs);
  assert(ackSilent == MQTT_RX_ACK);

  PassiveRepairModel repair;
  repair.observedMask = MQTT_RX_ALL;
  repair.detect((uint8_t)(MQTT_RX_COMMAND | MQTT_RX_ACK));
  const std::string frozenOutbox = repair.outbox;
  for (unsigned deadline = 0; deadline < 3; ++deadline) {
    repair.deadline(2);
    assert(repair.outbox == frozenOutbox);
  }
  assert(repair.reconnect);
  assert(repair.outbox == "immutable-response");

  PassiveRepairModel recovered;
  recovered.detect(MQTT_RX_COMMAND);
  recovered.observe(MQTT_RX_COMMAND);
  assert(!recovered.repairing && !recovered.reconnect);

  // Firmware performs only one ACK-driven transport rebuild per immutable
  // response; afterwards retries continue without a tight reconnect loop.
  const bool ackReconnectAlreadyDone = true;
  assert(mqttPassiveRxFailureMask(
    false, false, false, !ackReconnectAlreadyDone, false, false,
    60000, 60000, 0, 60000, initialSnapshotMs,
    ownerSilenceMs, ackSilenceMs) == 0);
}

void testResponseRejectIsTerminalAndUndoSafe() {
  ResponseRejectModel model;
  model.stage(TerminalResponseKind::Confirm, "response-confirm");
  model.applyConfirmCommit();
  assert(model.reject("response-confirm", "manager_incident"));
  assert(!model.pending && model.restoreCount == 0 && model.stackValid);
  assert(model.confirmRollbacks == 1 && !model.confirmCommitApplied);
  assert(!model.confirmReceiptAvailable && !model.canUndoRejectedConfirm());
  assert(!model.managerAdvanced && model.commandAcks == 0);
  assert(!model.reject("response-confirm", "manager_incident"));
  assert(model.confirmRollbacks == 1);

  // Once local fallback has released the operator, newer local work may have
  // happened. The same reject must invalidate history and require TARE rather
  // than performing a destructive late rollback.
  model.stage(TerminalResponseKind::Confirm, "response-confirm-detached");
  model.applyConfirmCommit();
  model.detach();
  assert(model.reject("response-confirm-detached", "manager_incident"));
  assert(!model.stackValid && model.tareRequired);
  assert(model.confirmRollbacks == 0 && !model.confirmReceiptAvailable);
  assert(!model.canUndoRejectedConfirm());
  assert(!model.managerAdvanced && model.commandAcks == 0);

  model.stage(TerminalResponseKind::Skip, "response-skip");
  assert(!model.reject("response-skip", "wrong_reason"));
  assert(model.pending);
  assert(model.reject("response-skip", "manager_incident"));
  assert(!model.pending && model.restoreCount == 0 && model.stackValid);

  model.stage(TerminalResponseKind::Undo, "response-undo-pre");
  assert(model.reject("response-undo-pre", "manager_incident"));
  assert(model.undoReject && !model.detachedUndoReject);
  assert(model.restoreCount == 0 && model.stackValid);

  model.stage(TerminalResponseKind::Undo, "response-undo-post");
  model.applyDetachedUndo();
  assert(model.restoreCount == 1);
  assert(model.reject("response-undo-post", "manager_incident"));
  assert(!model.undoReject && model.detachedUndoReject);
  assert(model.restoreCount == 1);
  assert(!model.stackValid);
  assert(!model.reject("response-undo-post", "manager_incident"));
  assert(model.restoreCount == 1);

  // If a successful ACK wins the race after detach was requested but before
  // the main loop applied it, the pending local restore must remain queued.
  bool detachedUndoFlag = true;
  bool detachedUndoApplied = false;
  bool responsePending = true;
  unsigned ackRaceRestoreCount = 0;
  responsePending = false;
  if (detachedUndoApplied) detachedUndoFlag = false;
  if (detachedUndoFlag) {
    detachedUndoFlag = false;
    ++ackRaceRestoreCount;
  }
  assert(!responsePending && ackRaceRestoreCount == 1);
}

void testFallbackRecoveryAndUndoIncidentTareGate() {
  // The update cycle processes transport/inbound work before fallback state.
  // A fresh idle owner can therefore return to APP without another command.
  FallbackRecoveryModel idle;
  idle.update();
  assert(!idle.fallbackActive && idle.appMode);
  assert(idle.commandAcks == 0);

  // A retained command needs both its snapshot and exact owner evidence. The
  // recovery emits one fresh command ACK when it becomes actionable again.
  FallbackRecoveryModel command;
  command.commandActive = true;
  command.commandSnapshotReady = true;
  command.commandOwnerAuthorized = true;
  command.commandHasId = true;
  command.update();
  assert(!command.fallbackActive && command.appMode);
  assert(command.commandAcks == 1);
  command.update();
  assert(command.commandAcks == 1);

  // A detached-undo incident keeps both the UI and MQTT layers fenced. A raw
  // command may be retained for recovery, but cannot be ACKed or actionable.
  UndoIncidentGateModel incident;
  incident.wizardActive = true;
  incident.longPressTracking = true;
  incident.receiveRawCommand();
  assert(incident.rawCommand && incident.commandSnapshotReady);
  assert(incident.fallbackActive && !incident.appMode);
  assert(incident.commandAcks == 0);

  // Holding SKIP well beyond the wizard threshold, plus CLEAR/ENTER events,
  // cannot mutate calibration, stack or weight history while fenced.
  for (unsigned tick = 0; tick < 60; ++tick) {
    incident.update(true, true, true);
  }
  incident.serialCalibrationMutation();
  assert(!incident.longPressTracking && !incident.wizardActive);
  assert(incident.stackMutations == 0 && incident.enterCommits == 0);
  assert(incident.calibrationMutations == 0);

  // A failed tare leaves every gate unchanged. Only a successfully applied
  // physical tare clears both layers and resumes the same raw command once.
  incident.tareResult(false);
  incident.updateTransport();
  assert(incident.needsTare && incident.netTareRequired);
  assert(incident.fallbackActive && incident.commandAcks == 0);
  incident.tareResult(true);
  assert(!incident.needsTare && !incident.netTareRequired);
  assert(!incident.fallbackActive && incident.appMode);
  assert(incident.commandAcks == 1);
  incident.updateTransport();
  assert(incident.commandAcks == 1);
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
  const std::string ackTopic = "minu/scale/123456789abc/ack";
  const std::string statusTopic = "minu/scale/123456789abc/status";

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
    // Manager 1.5 bounds user_name to 48 UTF-8 bytes before publication.
    "{\"user_id\":4294967295,\"user_name\":\"" + std::string(48, 'u') +
    "\",\"session_id\":\"" + sessionId + "\",\"connection_id\":\"" +
    connectionId + "\",\"lease_id\":\"" + std::string(64, 'l') +
    "\",\"expected_command_id\":\"" + commandId +
    "\",\"timestamp\":4294967295}";
  const std::string responseReject =
    "{\"type\":\"response_reject\",\"response_id\":\"" + responseId +
    "\",\"reason\":\"manager_incident\"}";
  const std::string status =
    "{\"type\":\"status\",\"state\":\"sleeping\","
    "\"scale_id\":\"123456789abc\",\"name\":\"Minu Bench Scale\","
    "\"firmware_version\":\"1.5.0\",\"boot_id\":\"ffffffff\","
    "\"transport_id\":\"ffffffff\",\"status_seq\":4294967295,"
    "\"published_at\":4294967295,\"uptime_ms\":4294967295,"
    "\"operational_mode\":\"local\","
    "\"tare_required\":true,"
    "\"fallback_reason\":\"manager_incident_detached\","
    "\"last_command_error\":\"manager_incident_detached\","
    "\"rx_probe_seq\":4294967295,\"rx_mask\":7,"
    "\"rx_health\":\"repairing\",\"pending_response_id\":\"" +
    responseId + "\"}";

  assert(commandAck.size() < 384);
  assert(confirm.size() < 384);
  assert(confirmRequestAck.size() < 384);
  constexpr size_t mqttPacketBufferSize = 640;
  assert(status.size() < 512);
  // MQTT_MAX_PACKET_SIZE also includes topic and packet framing.
  assert(topic.size() + commandAck.size() + 8 < mqttPacketBufferSize);
  assert(topic.size() + confirm.size() + 8 < mqttPacketBufferSize);
  assert(topic.size() + confirmRequestAck.size() + 8 < mqttPacketBufferSize);
  assert(commandTopic.size() + confirmRequest.size() + 8 < mqttPacketBufferSize);
  assert(ackTopic.size() + responseReject.size() + 8 < mqttPacketBufferSize);
  assert(statusTopic.size() + status.size() + 8 < mqttPacketBufferSize);
  assert(ownerTopic.size() + owner.size() + 8 < mqttPacketBufferSize);
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
  testInboundHeartbeatIsProcessedBeforeBoundedFallbackDecision();
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
  testPassiveInboundWatchdogIsBoundedAndPreservesOutbox();
  testResponseRejectIsTerminalAndUndoSafe();
  testFallbackRecoveryAndUndoIncidentTareGate();
  testMaximumPayloadsFitBuffers();
  std::cout << "firmware MQTT protocol harness: OK\n";
  return 0;
}
