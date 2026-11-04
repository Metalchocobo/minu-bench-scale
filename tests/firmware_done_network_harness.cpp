// check_done_network.py extracts the production DONE/outbox/ACK code.
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#define F(value) value
struct TestLog {
  void print(const char*) {}
  void println(const char*) {}
} ConsoleLog;
struct TestValue {
  const char* value;
  operator const char*() const { return value; }
  const char* operator|(const char* fallback) const { return value ? value : fallback; }
};
struct TestDoc {
  const char* id;
  const char* reason = nullptr;
  TestValue operator[](const char* key) const {
    return {strcmp(key, "response_id") == 0 ? id : reason};
  }
};
static uint32_t nowMs = 1000;
static uint32_t millis() { return nowMs; }
static uint32_t esp_random() { return 0xa1b2c3d4; }
static size_t strlcpy(char* target, const char* source, size_t size) {
  const size_t length = strlen(source);
  if (size > 0) {
    const size_t copied = length < size - 1 ? length : size - 1;
    memcpy(target, source, copied);
    target[copied] = '\0';
  }
  return length;
}
#include "done_network_enum.inc"
static bool attached = true, actionable = true, ownerExact = true, connected = true;
static uint32_t mqtt_ownerProductionId = 42, mqtt_cmdProductId = 42;
static bool mqtt_cmdActive = true, mqtt_ownerHasExpectedCommand = true;
static char mqtt_ownerExpectedCommandId[65] = "command-123";
static char mqtt_cmdCommandId[65] = "command-123";
static char mqtt_cmdUuid[37] = "ingredient-123";
static char mqtt_cmdSessionId[65] = "session-123";
static char mqtt_cmdConnectionId[65] = "connection-123";
static char mqtt_ownerSessionId[65] = "session-123";
static char mqtt_ownerConnectionId[65] = "connection-123";
static char mqtt_topicRsp[] = "minu/scale/123456789abc/response";
enum { MQTT_CONFIRM_REQUEST_NONE, MQTT_CONFIRM_REQUEST_QUEUED,
       MQTT_CONFIRM_REQUEST_RUNNING, MQTT_CONFIRM_REQUEST_STAGED, MQTT_CONFIRM_REQUEST_FAILED };
static int mqtt_confirmRequestState = MQTT_CONFIRM_REQUEST_NONE;
static bool mqtt_responsePending = false;
static MqttResponseKind mqtt_responseKind = MQTT_RESPONSE_NONE;
static char mqtt_responseId[25] = {}, mqtt_responsePayload[384] = {};
static uint32_t mqtt_responseLastPublishMs = 0, mqtt_responseStartedMs = 0, mqtt_responseCounter = 0;
static bool mqtt_responseAckReconnectDone = false, mqtt_statusDirty = false;
static uint8_t mqtt_doneAckStatus = 0;
static bool mqtt_doneRejectFlag = false;
static bool mqtt_detachedUndoFlag = false, mqtt_undoWasDetachedApplied = false;
static bool mqtt_undoAckFlag = false, mqtt_undoDetachedRejectFlag = false, mqtt_undoRejectFlag = false;
static bool mqtt_confirmRejectFlag = false, mqtt_confirmRejectRollbackSafe = false;
static char mqtt_confirmRejectResponseId[25] = {};
static bool mqtt_localFallbackActive = false;
static bool tareRequired = false;
enum { MQTT_RX_ACK, MQTT_COMMAND_ERROR_MANAGER_INCIDENT, MQTT_COMMAND_ERROR_MANAGER_INCIDENT_DETACHED };
static void mqttMarkInboundTopic(int) {}
static void mqttSetCommandError(int) {}
static void mqttSetTareRequired(bool value) { tareRequired = value; }
static void mqttResetCommand() { mqtt_cmdActive = false; }
static void mqttResetConfirmRequest() { mqtt_confirmRequestState = MQTT_CONFIRM_REQUEST_NONE; }
static bool isMqttManagerAttached() { return attached; }
static bool mqttCommandIsActionable() { return actionable; }
static bool mqttOwnerAuthorizes(const char*, const char*) { return ownerExact; }
static bool mqttClientConnectedForMain() { return connected; }
struct TestClient {
  bool result = true;
  std::vector<std::string> publications;
  bool publish(const char*, const uint8_t* payload, size_t length, bool) {
    publications.emplace_back(reinterpret_cast<const char*>(payload), length);
    nowMs += 70;  // A slow write must still space retries from completion.
    return result;
  }
} mqttClient;
#include "done_network_production.inc"

static void reset() {
  mqttResetPendingResponse();
  attached = actionable = ownerExact = connected = true;
  mqtt_ownerProductionId = mqtt_cmdProductId = 42;
  mqtt_cmdActive = mqtt_ownerHasExpectedCommand = true;
  strcpy(mqtt_ownerExpectedCommandId, "command-123");
  strcpy(mqtt_cmdCommandId, "command-123");
  mqtt_confirmRequestState = MQTT_CONFIRM_REQUEST_NONE;
  mqtt_doneAckStatus = 0;
  mqtt_doneRejectFlag = false;
  mqttClient.result = true;
  mqttClient.publications.clear();
  tareRequired = false;
}

int main() {
  reset();
  assert(isMqttDoneActionable());
  attached = false; assert(!mqttDoneActiveProduction(2)); attached = true;
  actionable = false; assert(!mqttDoneActiveProduction(2)); actionable = true;
  ownerExact = false; assert(!mqttDoneActiveProduction(2)); ownerExact = true;
  mqtt_ownerProductionId = 0; assert(!mqttDoneActiveProduction(2)); mqtt_ownerProductionId = 42;
  mqtt_cmdProductId = 43; assert(!mqttDoneActiveProduction(2)); mqtt_cmdProductId = 42;
  for (int state : {MQTT_CONFIRM_REQUEST_QUEUED, MQTT_CONFIRM_REQUEST_RUNNING, MQTT_CONFIRM_REQUEST_STAGED}) {
    mqtt_confirmRequestState = state;
    assert(!mqttDoneActiveProduction(2));
  }
  mqtt_confirmRequestState = MQTT_CONFIRM_REQUEST_NONE;
  assert(!mqttDoneActiveProduction(0));
  assert(!mqttDoneActiveProduction(1));
  assert(!mqttDoneActiveProduction(4));
  assert(!mqtt_responsePending);
  assert(mqttDoneActiveProduction(2));
  const std::string weighedPayload = mqtt_responsePayload;
  const std::string weighedId = mqtt_responseId;
  assert(weighedPayload.find("\"production_status\":2") != std::string::npos);
  assert(weighedPayload.find("\"command_id\":\"command-123\"") != std::string::npos);
  assert(weighedPayload.find("actual_weight") == std::string::npos);
  assert(mqttPopDoneAck() == 0);
  assert(!mqttDoneActiveProduction(3));
  assert(mqtt_responsePayload == weighedPayload);
  receiveAck({"foreign-id"}, "response_ack");
  assert(mqtt_responsePending && mqttPopDoneAck() == 0);
  // Context can detach while the immutable response keeps retrying.
  attached = false; mqtt_localFallbackActive = true;
  mqttPublishPendingResponse();
  assert(mqttClient.publications.back() == weighedPayload);
  assert(mqtt_responseLastPublishMs == nowMs);
  receiveAck({weighedId.c_str()}, "response_ack");
  assert(!mqtt_responsePending && mqttPopDoneAck() == 2 && mqttPopDoneAck() == 0);
  receiveAck({weighedId.c_str()}, "response_ack");
  assert(mqttPopDoneAck() == 0);

  reset(); mqtt_localFallbackActive = false;
  mqtt_cmdActive = false;
  assert(!isMqttDoneActionable());  // An expected weigh never becomes a recap.
  mqtt_ownerExpectedCommandId[0] = '\0';
  mqtt_ownerHasExpectedCommand = false; assert(!isMqttDoneActionable());
  mqtt_ownerHasExpectedCommand = true;
  assert(isMqttDoneActionable());
  mqttClient.result = false;
  assert(mqttDoneActiveProduction(3));  // Failed QoS0 publish still stages durable retry.
  const std::string finishedId = mqtt_responseId;
  const std::string finishedPayload = mqtt_responsePayload;
  assert(finishedPayload.find("\"production_status\":3") != std::string::npos);
  assert(finishedPayload.find("\"uuid\":\"\"") != std::string::npos);
  assert(finishedPayload.find("command_id") == std::string::npos);
  mqttPublishPendingResponse();
  assert(mqttClient.publications.back() == finishedPayload);
  receiveAck({finishedId.c_str(), "other_reason"}, "response_reject");
  assert(mqtt_responsePending && !mqttPopDoneReject());
  receiveAck({finishedId.c_str(), "manager_incident"}, "response_reject");
  assert(!mqtt_responsePending && mqttPopDoneAck() == 0);
  assert(mqttPopDoneReject() && !mqttPopDoneReject());
  assert(!tareRequired);  // DONE never mutates weight, zero or undo history.

  reset();
  assert(mqttDoneActiveProduction(3));
  const std::string successId = mqtt_responseId;
  receiveAck({successId.c_str()}, "response_ack");
  assert(mqttPopDoneAck() == 3 && mqttPopDoneAck() == 0);
  std::cout << "firmware DONE network: OK\n";
}
