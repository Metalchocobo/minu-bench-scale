// Production gesture helpers are extracted by check_done_controls.py.
#include <Arduino.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include "firmware/esp32_hx711_serial/keypad.h"
#include "firmware/esp32_hx711_serial/config/config_audio.h"

namespace Track = AudioConfig::Track;
static int errors = 0, successes = 0;
void buzzerError() { ++errors; }
void buzzerOk() { ++successes; }
namespace Audio {
static std::vector<uint16_t> requests;
void requestPlayMp3(uint16_t track) { requests.push_back(track); }
}
namespace CalWizard {
static bool active = false, longPress = false;
bool isActive() { return active; }
bool isLongPressInProgress() { return longPress; }
}
namespace Net {
static bool actionable = true, stageAllowed = true, pending = false;
static uint32_t productId = 42;
static std::string commandId = "command-1", connectionId = "connection-1";
static std::vector<uint8_t> stages;
static uint8_t ack = 0;
static bool rejected = false;
bool isMqttDoneActionable() { return actionable && !pending; }
uint32_t getMqttDoneProductId() { return productId; }
const char* getMqttDoneCommandId() { return commandId.c_str(); }
const char* getMqttDoneConnectionId() { return connectionId.c_str(); }
bool mqttDoneActiveProduction(uint8_t status) {
  if (!stageAllowed) return false;
  stages.push_back(status);
  pending = true;
  return true;
}
uint8_t mqttPopDoneAck() { uint8_t value = ack; ack = 0; return value; }
bool mqttPopDoneReject() { bool value = rejected; rejected = false; return value; }
}
enum EnterUiState { ENTER_UI_IDLE, ENTER_UI_ACQUIRING, ENTER_UI_SUCCESS };
static EnterUiState g_enterUiState = ENTER_UI_IDLE;
static bool g_syncIncidentNeedsTare = false, tareLocked = false, pressed = false;
static uint32_t g_donePressStartMs = 0, g_doneProductId = 0;
static char g_doneCommandId[65] = {}, g_doneConnectionId[65] = {};
static const uint32_t LONG_PRESS_MS = 2000;
bool isTareBehaviorLocked(uint32_t) { return tareLocked; }
bool keypad_is_pressed(KeyCode key) { return key == KEY_DONE && pressed; }
#include "done_controls_production.inc"

static unsigned checks = 0;
static void require(bool condition, const char* message) {
  ++checks;
  if (!condition) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}
static void reset() {
  errors = successes = 0;
  Audio::requests.clear(); Net::stages.clear();
  Net::actionable = Net::stageAllowed = true; Net::pending = false;
  Net::productId = 42; Net::commandId = "command-1"; Net::connectionId = "connection-1";
  Net::ack = 0; Net::rejected = false;
  CalWizard::active = CalWizard::longPress = false;
  g_enterUiState = ENTER_UI_IDLE;
  g_syncIncidentNeedsTare = tareLocked = false;
  g_donePressStartMs = 0; pressed = true;
}
static void shortRelease() { pressed = false; updateDoneKey(1200); }

int main() {
  reset(); startDoneKey(1000); updateDoneKey(2999);
  require(Net::stages.empty(), "held key must wait for threshold");
  pressed = false; updateDoneKey(2999);
  require(Net::stages == std::vector<uint8_t>{2}, "short release must stage Pesato only");
  require(Audio::requests.empty() && successes == 0, "staging must not announce success");
  Net::ack = 2; completeRemoteDone(); completeRemoteDone();
  require(Audio::requests == std::vector<uint16_t>{22} && successes == 1, "ACK must play 22 once");

  reset(); startDoneKey(1000); updateDoneKey(3000); updateDoneKey(5000);
  pressed = false; updateDoneKey(5100);
  require(Net::stages == std::vector<uint8_t>{3}, "long gesture must stage Fatto once, no Pesato");
  Net::ack = 3; completeRemoteDone(); completeRemoteDone();
  require(Audio::requests == std::vector<uint16_t>{23} && successes == 1, "ACK must play 23 once");

  reset(); startDoneKey(1000); pressed = false; updateDoneKey(3050);
  require(Net::stages == std::vector<uint8_t>{3}, "slow loop must preserve long gesture");

  // Each preflight blocker prevents arming and remains silent outside weighing.
  for (int blocker = 0; blocker < 7; ++blocker) {
    reset();
    if (blocker == 0) Net::actionable = false;
    if (blocker == 1) Net::pending = true;
    if (blocker == 2) g_syncIncidentNeedsTare = true;
    if (blocker == 3) tareLocked = true;
    if (blocker == 4) CalWizard::active = true;
    if (blocker == 5) CalWizard::longPress = true;
    if (blocker == 6) g_enterUiState = ENTER_UI_ACQUIRING;
    startDoneKey(1000); shortRelease();
    require(Net::stages.empty() && errors == 0, "blocked gesture must remain inactive");
  }

  // Context loss is sticky for this gesture, even if readiness later returns.
  for (int change = 0; change < 6; ++change) {
    reset(); startDoneKey(1000);
    if (change == 0) Net::actionable = false;
    if (change == 1) Net::pending = true;
    if (change == 2) ++Net::productId;
    if (change == 3) Net::commandId = "command-2";
    if (change == 4) Net::connectionId = "connection-2";
    if (change == 5) tareLocked = true;
    updateDoneKey(1100);
    Net::actionable = true; Net::pending = tareLocked = false;
    shortRelease();
    require(Net::stages.empty(), "context change must cancel until a fresh press");
  }

  reset(); Net::commandId.clear(); startDoneKey(1000); shortRelease();
  require(Net::stages == std::vector<uint8_t>{2}, "recap without ingredient must support DONE");
  reset(); Net::stageAllowed = false; startDoneKey(1000); shortRelease();
  require(errors == 1 && Audio::requests.empty(), "stage failure must only signal error");
  reset(); Net::rejected = true; completeRemoteDone(); completeRemoteDone();
  require(errors == 1 && Audio::requests.empty(), "rejection must never announce success");
  reset(); uint32_t start = UINT32_MAX - 999; startDoneKey(start); updateDoneKey(1000);
  require(Net::stages == std::vector<uint8_t>{3}, "gesture timing must survive millis rollover");
  require(AudioConfig::isMustPlayTrack(22) && AudioConfig::isMustPlayTrack(23),
          "DONE confirmations must survive another priority announcement");
  std::cout << "PASS: production DONE controls (" << checks << " checks)\n";
}
