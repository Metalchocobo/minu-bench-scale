#pragma once

#include <stdint.h>

enum MqttRxTopicBit : uint8_t {
  MQTT_RX_OWNER = 1,
  MQTT_RX_COMMAND = 2,
  MQTT_RX_ACK = 4,
  MQTT_RX_ALL = MQTT_RX_OWNER | MQTT_RX_COMMAND | MQTT_RX_ACK
};

enum class MqttRxRepairAction : uint8_t {
  Wait = 0,
  Retry,
  Reconnect
};

inline bool mqttRxHealthy(uint8_t mask) {
  return (mask & MQTT_RX_ALL) == MQTT_RX_ALL;
}

inline MqttRxRepairAction mqttRxRepairAction(
    uint8_t mask, bool deadlineExpired, uint8_t completedRetries,
    uint8_t maxRetries) {
  if (mqttRxHealthy(mask) || !deadlineExpired) {
    return MqttRxRepairAction::Wait;
  }
  return completedRetries < maxRetries
    ? MqttRxRepairAction::Retry : MqttRxRepairAction::Reconnect;
}

inline bool mqttOwnerTimestampAllowed(
    bool exactStoredLease, bool exactLastSeenLease,
    uint32_t incomingTimestamp, uint32_t storedTimestamp,
    uint32_t lastSeenTimestamp) {
  if (exactStoredLease && incomingTimestamp < storedTimestamp) return false;
  if (exactLastSeenLease && incomingTimestamp < lastSeenTimestamp) return false;
  return true;
}

inline bool mqttOwnerArrivalFresh(
    uint32_t elapsedSinceArrivalMs, uint32_t leaseRemainingAtArrivalMs,
    uint32_t presenceLimitMs) {
  return leaseRemainingAtArrivalMs > 0 &&
    elapsedSinceArrivalMs < leaseRemainingAtArrivalMs &&
    elapsedSinceArrivalMs < presenceLimitMs;
}

inline uint32_t mqttOwnerLeaseRemainingMs(
    uint32_t ageSec, uint32_t ttlMs, bool progressiveSameLease,
    uint32_t progressiveMaxAgeSec, uint32_t progressiveGraceMs) {
  uint32_t ttlSec = ttlMs / 1000U;
  if (ageSec >= ttlSec) return 0;
  uint32_t remainingMs = ttlMs - ageSec * 1000U;
  if (progressiveSameLease && ageSec <= progressiveMaxAgeSec) {
    uint32_t roomMs = ttlMs - remainingMs;
    remainingMs += progressiveGraceMs < roomMs
      ? progressiveGraceMs : roomMs;
  }
  return remainingMs;
}

inline bool mqttFallbackHasFreshEvidence(
    bool fallbackActive, bool legacyCommand, uint32_t ownerTimestamp,
    uint32_t fallbackTimestamp, bool connectionChanged, bool leaseChanged) {
  return !fallbackActive || legacyCommand ||
    ownerTimestamp > fallbackTimestamp || connectionChanged || leaseChanged;
}

inline bool mqttExpectedClearIsAuthoritative(
    bool ownerActive, bool hasExpectedCommand, bool expectedCommandEmpty,
    bool responsePending, bool confirmPending) {
  return ownerActive && hasExpectedCommand && expectedCommandEmpty &&
    !responsePending && !confirmPending;
}

inline bool mqttCommandEvidenceCarriesToNewExpectation(
    bool expectationMatches, bool commandSnapshotReady,
    bool commandObservedOnTransport) {
  return expectationMatches && commandSnapshotReady &&
    commandObservedOnTransport;
}

inline bool mqttShouldAwaitOwnerForModernCommand(
    bool commandStructurallyValid, bool modernCommand,
    bool ownerAuthorized) {
  return commandStructurallyValid && modernCommand && !ownerAuthorized;
}

inline bool mqttOwnerExpectationGenerationChanged(
    bool generationTracked, bool sameCommandId,
    bool sameConnectionId) {
  return !generationTracked || !sameCommandId || !sameConnectionId;
}

inline bool mqttOwnerExpectationFingerprintMatches(
    bool generationTracked, bool sameCommandId,
    bool sameConnectionId) {
  return generationTracked && sameCommandId && sameConnectionId;
}

inline bool mqttOwnerExpectationKeepsDeadline(
    bool expectationPending, bool evidenceSatisfied) {
  return expectationPending && !evidenceSatisfied;
}

inline bool mqttOwnerExpectationMayReconnect(
    bool generationScopedRepair, bool reconnectAlreadyDone) {
  return !generationScopedRepair || !reconnectAlreadyDone;
}

inline bool mqttOwnerExpectationEvidenceAfterTransportReset(
    bool generationTracked, bool previousEvidence) {
  return generationTracked ? false : previousEvidence;
}

inline bool mqttOwnerRepairEvidenceRecovered(
    bool generationScopedRepair, bool generationEvidence,
    bool currentTopicEvidence) {
  return generationScopedRepair
    ? generationEvidence : currentTopicEvidence;
}

inline bool mqttCommandAckAllowed(
    bool commandActive, bool commandIdPresent, bool transportConnected,
    bool expectationMatches, bool ownerFenceSatisfied,
    bool commandActionable) {
  return commandActive && commandIdPresent && transportConnected &&
    expectationMatches && ownerFenceSatisfied && commandActionable;
}

inline uint8_t mqttPassiveRxFailureMask(
    bool ownerInputExpected, bool commandInputExpected,
    bool ownerActive, bool responsePending,
    bool ownerEvidence, bool commandEvidence,
    uint32_t ownerEvidenceAgeMs, uint32_t commandEvidenceAgeMs,
    uint32_t ownerSilenceMs,
    uint32_t ackWaitMs, uint32_t initialSnapshotMs,
    uint32_t ownerSilenceLimitMs, uint32_t ackSilenceLimitMs) {
  uint8_t failureMask = 0;
  if (ownerInputExpected && ownerEvidenceAgeMs >= initialSnapshotMs &&
      !ownerEvidence) {
    failureMask |= MQTT_RX_OWNER;
  }
  if (commandInputExpected &&
      commandEvidenceAgeMs >= initialSnapshotMs &&
      !commandEvidence) {
    failureMask |= MQTT_RX_COMMAND;
  }
  if (ownerActive && ownerSilenceMs >= ownerSilenceLimitMs) {
    failureMask |= MQTT_RX_OWNER;
  }
  if (responsePending && ackWaitMs >= ackSilenceLimitMs) {
    failureMask |= MQTT_RX_ACK;
  }
  return failureMask;
}
