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

inline uint8_t mqttPassiveRxFailureMask(
    bool modernCommand, bool ownerActive, bool responsePending,
    bool ownerEvidence, bool commandEvidence, bool commandSnapshotReady,
    uint32_t transportAgeMs, uint32_t ownerSilenceMs,
    uint32_t ackWaitMs, uint32_t initialSnapshotMs,
    uint32_t ownerSilenceLimitMs, uint32_t ackSilenceLimitMs) {
  uint8_t failureMask = 0;
  if (modernCommand && transportAgeMs >= initialSnapshotMs) {
    if (!ownerEvidence) failureMask |= MQTT_RX_OWNER;
    if (!commandEvidence || !commandSnapshotReady) {
      failureMask |= MQTT_RX_COMMAND;
    }
  }
  if (ownerActive && ownerSilenceMs >= ownerSilenceLimitMs) {
    failureMask |= MQTT_RX_OWNER;
  }
  if (responsePending && ackWaitMs >= ackSilenceLimitMs) {
    failureMask |= MQTT_RX_ACK;
  }
  return failureMask;
}
