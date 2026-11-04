// check_undo_controls.py extracts controller functions from the real sketch.
// Stack, tare, zero tracking and filter reset use their production modules.
#include <Arduino.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "firmware/esp32_hx711_serial/keypad.h"
#include "firmware/esp32_hx711_serial/scale_state.h"
#include "firmware/esp32_hx711_serial/weigh_stack.h"
#include "firmware/esp32_hx711_serial/config/config_audio.h"

#define ENABLE_MQTT 1
#define ConsoleLog Serial
using namespace ScaleConfig;
namespace Track = AudioConfig::Track;

ScaleTestSerial Serial;
static unsigned long testNowMs = 1000;
unsigned long millis() { return testNowMs; }
static int errorBeeps = 0;
static int okBeeps = 0;
static int keyBeeps = 0;
static int warningBeeps = 0;
void buzzerError() { ++errorBeeps; }
void buzzerOk() { ++okBeeps; }
void buzzerKeyClick() { ++keyBeeps; }
void buzzerWarn() { ++warningBeeps; }

namespace Audio {
static std::vector<uint16_t> requests;
static bool enabled = true;
static int toggles = 0;
void requestPlayMp3(uint16_t track) { requests.push_back(track); }
bool toggleEnabled() { ++toggles; enabled = !enabled; return enabled; }
bool isReady() { return true; }
bool hasSavedDiagnostic() { return false; }
}

namespace CalWizard {
static bool active = false;
static bool longPress = false;
bool isActive() { return active; }
bool isLongPressInProgress() { return longPress; }
}

namespace Net {
static bool pending = false;
static bool stageAllowed = true;
static bool undoAck = false;
static bool detachedUndo = false;
static bool detachedApplied = false;
static int stageCalls = 0;
static std::string stagedUuid, stagedConfirm, stagedSession;
static uint32_t stagedProduct = 0;
static std::string lastSession;
bool isMqttResponsePending() { return pending; }
const char* getMqttLastSessionId() { return lastSession.c_str(); }
bool mqttStageUndo(const char* uuid, uint32_t productId,
                   const char* undoOfResponseId, const char* sessionId) {
  ++stageCalls;
  if (!stageAllowed || pending) return false;
  stagedUuid = uuid;
  stagedProduct = productId;
  stagedConfirm = undoOfResponseId;
  stagedSession = sessionId;
  pending = true;
  return true;
}
bool mqttPopUndoAck() { bool value = undoAck; undoAck = false; return value; }
bool mqttPopDetachedUndo() { bool value = detachedUndo; detachedUndo = false; return value; }
void mqttReportDetachedUndoApplied(bool applied) { detachedApplied = applied; }
}

enum OverlayType { OVERLAY_NONE, OVERLAY_STACK_COMPARE, OVERLAY_AUDIO_STATUS,
                   OVERLAY_BATTERY };
enum EnterUiState { ENTER_UI_IDLE, ENTER_UI_ACQUIRING, ENTER_UI_SUCCESS,
                    ENTER_UI_FAILED_MOVING, ENTER_UI_FAILED };
enum InactivityStage { INACT_NONE, INACT_ZZZ };
static OverlayType g_overlayType = OVERLAY_NONE;
static EnterUiState g_enterUiState = ENTER_UI_IDLE;
static InactivityStage g_inactivitySleepStage = INACT_NONE;
static uint32_t g_inactivityStageStartMs = 0;
static uint32_t g_overlayStartMs = 0;
static int g_overlayCount = 0;
static bool g_overlayAudioEnabled = true;
static float gLiveEma = 0;
static bool gLiveEmaInit = false;
static bool gLiveEmaPrevStEnable = true;
static bool stEnable = true;
static uint32_t g_tareKeyLockUntilMs = 0;
static uint32_t g_lastKeyPressMs = 0;
static uint32_t lastOledMs = 0;
static bool g_manualTareArmed = false;
static bool g_syncIncidentNeedsTare = false;
static bool g_remoteUndoAwaitingAck = false;
static bool g_remoteUndoRestoredDetached = false;
static char g_remoteUndoConfirmId[WeighStack::RESPONSE_ID_CAP] = {};
static bool g_detachedUndoKeyGuard = false;
static uint32_t g_undoPressStartMs = 0;
static bool g_undoLongPressFired = false;
static const uint32_t LONG_PRESS_MS = 2000;
static bool undoPressed = false;
bool keypad_is_pressed(KeyCode key) { return key == KEY_UNDO && undoPressed; }
void ui_renderAudioStatus(bool, bool, bool) {}

#include "undo_controls_production.inc"

static const char* scenario = "initialization";
static unsigned int scenarios = 0;
static unsigned int assertions = 0;
static void require(bool condition, const char* message) {
  ++assertions;
  if (!condition) {
    std::cerr << "FAIL [" << scenario << "]: " << message << '\n';
    std::exit(1);
  }
}
static void start(const char* name) {
  scenario = name;
  ++scenarios;
  testNowMs += 5000;
  WeighStack::clear();
  WeighStack::clearReference();
  ScaleState::tareCancel(testNowMs);
  ScaleState::setTareUiActive(false);
  ScaleState::setScaleCpg(200.0f);
  ScaleState::setCalibrationZeroRaw(100000);
  ScaleState::setOffsetRaw(300000);
  ScaleState::setZtCounts(50);
  ScaleState::resetFiltersAndState();
  Audio::requests.clear();
  Audio::enabled = true;
  Audio::toggles = 0;
  errorBeeps = okBeeps = keyBeeps = warningBeeps = 0;
  Net::pending = Net::undoAck = Net::detachedUndo = Net::detachedApplied = false;
  Net::stageAllowed = true;
  Net::stageCalls = 0;
  Net::lastSession.clear();
  Net::stagedUuid.clear();
  Net::stagedConfirm.clear();
  Net::stagedSession.clear();
  Net::stagedProduct = 0;
  CalWizard::active = CalWizard::longPress = false;
  g_overlayType = OVERLAY_NONE;
  gLiveEma = 99.0f;
  gLiveEmaInit = true;
  g_enterUiState = ENTER_UI_IDLE;
  g_manualTareArmed = g_syncIncidentNeedsTare = false;
  g_remoteUndoAwaitingAck = g_remoteUndoRestoredDetached = false;
  g_remoteUndoConfirmId[0] = '\0';
  g_detachedUndoKeyGuard = false;
  g_undoPressStartMs = 0;
  g_undoLongPressFired = false;
  g_tareKeyLockUntilMs = 0;
  undoPressed = false;
}
static void pushLocal(long oldOffset = 200000, long oldZt = 50) {
  require(WeighStack::pushCommit(500, oldOffset, oldZt, "ingredient", 7, "original-session"),
          "commit can be pushed");
}
static void pushRemote() {
  pushLocal();
  require(WeighStack::setLastConfirmResponseId("original-confirm"), "remote receipt attached");
}
static void checkRestored() {
  require(WeighStack::count() == 0, "exactly one commit removed");
  require(ScaleState::getOffsetRaw() == 200000 && ScaleState::getZtCounts() == 50,
          "working offset and zero tracking both restored");
  const float displayed = static_cast<float>(300050 - ScaleState::effectiveOffsetCounts()) /
                          ScaleState::getScaleCpg();
  require(std::fabs(displayed - 500.0f) < 0.01f, "same plate load becomes visible as 500 grams");
  require(ScaleState::getCalibrationZeroRaw() == 100000, "persistent calibration reference unchanged");
  require(g_overlayType == OVERLAY_NONE && !gLiveEmaInit && gLiveEma == 0.0f,
          "weight display resumes immediately with live smoothing reset");
}

int main() {
  require(AudioConfig::Track::UNDO_SUCCESS == 21 && AudioConfig::isMustPlayTrack(21),
          "0021 success confirmation is queued instead of dropped behind another voice");
  start("local UNDO restores the previous tare and plays 0021 once");
  pushLocal();
  performUndoAction(testNowMs);
  checkRestored();
  require(Audio::requests == std::vector<uint16_t>{21}, "local success requests 0021 exactly once");
  require(errorBeeps == 0, "local success has no error beep");

  start("empty history gives only an error beep");
  performUndoAction(testNowMs);
  require(errorBeeps == 1 && Audio::requests.empty(), "empty stack has error beep and no voice");
  require(keyBeeps == 0 && okBeeps == 0 && warningBeeps == 0, "failed UNDO emits no success or key click");
  require(ScaleState::getOffsetRaw() == 300000, "empty history never changes tare");

  start("remote undo stages the exact receipt and waits for durable ACK");
  pushRemote();
  performUndoAction(testNowMs);
  require(Net::stageCalls == 1 && Net::stagedUuid == "ingredient" && Net::stagedProduct == 7 &&
          Net::stagedConfirm == "original-confirm" && Net::stagedSession == "original-session",
          "undo correlates the original ingredient, product, receipt and fallback session");
  require(WeighStack::count() == 1 && ScaleState::getOffsetRaw() == 300000 && Audio::requests.empty(),
          "staging does not restore tare or announce success");
  Net::pending = false;
  Net::undoAck = true;
  completeRemoteUndo(testNowMs);
  checkRestored();
  require(Audio::requests == std::vector<uint16_t>{21}, "remote ACK announces success once");
  completeRemoteUndo(testNowMs);
  require(Audio::requests.size() == 1 && WeighStack::count() == 0, "consumed ACK cannot repeat restore or audio");

  start("remote staging failure leaves state unchanged and gives error only");
  pushRemote();
  Net::stageAllowed = false;
  performUndoAction(testNowMs);
  require(WeighStack::count() == 1 && ScaleState::getOffsetRaw() == 300000,
          "failed staging preserves stack and tare");
  require(errorBeeps == 1 && Audio::requests.empty(), "failed staging gives error beep only");

  start("detached undo restores locally once and announces only at later ACK");
  pushRemote();
  performUndoAction(testNowMs);
  Net::detachedUndo = true;
  require(applyDetachedUndoNow(), "detach event consumed");
  checkRestored();
  require(Net::detachedApplied && Audio::requests.empty(), "detached restoration waits for Manager before voice");
  require(!applyDetachedUndoNow(), "detach event is consumed once");
  Net::pending = false;
  Net::undoAck = true;
  completeRemoteUndo(testNowMs);
  checkRestored();
  require(Audio::requests == std::vector<uint16_t>{21}, "late ACK plays voice once without restoring twice");
  completeRemoteUndo(testNowMs);
  require(Audio::requests.size() == 1, "late ACK feedback is consumed once");

  start("an ACK for a different top commit cannot restore or announce success");
  pushRemote();
  performUndoAction(testNowMs);
  require(WeighStack::pop(), "simulate invalidated local receipt");
  pushLocal(123000, 3);
  Net::pending = false;
  Net::undoAck = true;
  completeRemoteUndo(testNowMs);
  require(WeighStack::count() == 1 && ScaleState::getOffsetRaw() == 300000,
          "mismatched ACK leaves current stack and tare unchanged");
  require(Audio::requests.empty() && errorBeeps == 1, "mismatched ACK gives error only");

  start("quarantined-confirm rollback does not play undo success voice");
  pushRemote();
  require(restoreTopStackEntryAfterUndo(testNowMs), "shared restore helper succeeds");
  checkRestored();
  require(Audio::requests.empty(), "rollback helper has no UNDO success voice");

  start("short release runs one undo");
  pushLocal();
  g_undoPressStartMs = testNowMs;
  undoPressed = true;
  testNowMs += 100;
  updateUndoKey(testNowMs);
  require(WeighStack::count() == 1 && Audio::requests.empty(), "holding short press does not undo yet");
  undoPressed = false;
  updateUndoKey(testNowMs);
  checkRestored();
  require(Audio::requests == std::vector<uint16_t>{21}, "short release performs undo once");
  updateUndoKey(testNowMs);
  require(Audio::requests.size() == 1, "release is consumed once");

  start("two-second hold toggles audio once and never undoes on release");
  pushLocal();
  g_undoPressStartMs = testNowMs;
  undoPressed = true;
  testNowMs += 2000;
  updateUndoKey(testNowMs);
  require(Audio::toggles == 1 && !Audio::enabled, "long hold toggles audio off at threshold");
  testNowMs += 4000;
  updateUndoKey(testNowMs);
  require(Audio::toggles == 1, "continued hold does not toggle again");
  undoPressed = false;
  updateUndoKey(testNowMs);
  require(WeighStack::count() == 1 && ScaleState::getOffsetRaw() == 300000 && Audio::requests.empty(),
          "long release neither undoes nor plays success voice");

  start("release first observed after the long threshold cannot become an undo");
  pushLocal();
  g_undoPressStartMs = testNowMs;
  undoPressed = true;
  testNowMs += 1900;
  updateUndoKey(testNowMs);
  require(Audio::toggles == 0 && WeighStack::count() == 1,
          "last held sample below threshold has no action");
  undoPressed = false;
  testNowMs += 150;
  updateUndoKey(testNowMs);
  require(Audio::toggles == 1 && !Audio::enabled && g_undoPressStartMs == 0,
          "release observed at 2050ms selects and consumes the long action");
  require(WeighStack::count() == 1 && ScaleState::getOffsetRaw() == 300000 &&
          ScaleState::getZtCounts() == 50 && Audio::requests.empty(),
          "delayed release changes neither history nor tare and has no undo voice");
  updateUndoKey(testNowMs);
  require(Audio::toggles == 1 && WeighStack::count() == 1,
          "delayed release cannot repeat the audio toggle or undo later");

  for (unsigned int block = 0; block < 8; ++block) {
    start("release-time guards preserve state and give error only");
    pushLocal();
    switch (block) {
      case 0: g_syncIncidentNeedsTare = true; break;
      case 1: g_enterUiState = ENTER_UI_ACQUIRING; break;
      case 2: g_manualTareArmed = true; break;
      case 3: g_tareKeyLockUntilMs = testNowMs + 1000; break;
      case 4: CalWizard::active = true; break;
      case 5: CalWizard::longPress = true; break;
      case 6: Net::pending = true; break;
      case 7: ScaleState::tareStart(testNowMs); break;
    }
    g_undoPressStartMs = testNowMs - 100;
    // Active calibration consumes the key as "cancel" in the outer loop.
    // Test the action guard directly; its press tracker is intentionally reset.
    if (CalWizard::active) performUndoAction(testNowMs);
    else updateUndoKey(testNowMs);
    require(WeighStack::count() == 1 && ScaleState::getOffsetRaw() == 300000,
            "guard observed at release preserves history and tare");
    require(Audio::requests.empty() && errorBeeps == 1, "blocked undo emits error beep only");
  }

  start("calibration takes ownership of an already tracked UNDO press");
  pushLocal();
  g_undoPressStartMs = testNowMs - 100;
  CalWizard::active = true;
  updateUndoKey(testNowMs);
  require(g_undoPressStartMs == 0 && WeighStack::count() == 1 && Audio::requests.empty(),
          "wizard cancels normal key tracking without a weighing mutation");

  start("ACK racing a requested detach restores and announces only once");
  pushRemote();
  performUndoAction(testNowMs);
  Net::detachedUndo = true;
  Net::pending = false;
  Net::undoAck = true;
  completeRemoteUndo(testNowMs);
  checkRestored();
  require(applyDetachedUndoNow(), "stale detach event is consumed after ACK");
  checkRestored();
  require(Audio::requests == std::vector<uint16_t>{21}, "ACK/detach race cannot duplicate voice or restore");

  start("repeated local undo follows LIFO and restores each saved tare");
  pushLocal(100000, 11);
  pushLocal(200000, 22);
  performUndoAction(testNowMs);
  require(WeighStack::count() == 1 && ScaleState::getOffsetRaw() == 200000 &&
          ScaleState::getZtCounts() == 22, "first undo restores newest saved tare");
  testNowMs += TARE_OK_HOLD_MS + 1;
  performUndoAction(testNowMs);
  require(WeighStack::count() == 0 && ScaleState::getOffsetRaw() == 100000 &&
          ScaleState::getZtCounts() == 11, "second undo restores preceding saved tare");
  require(Audio::requests == std::vector<uint16_t>({21, 21}), "each successful LIFO undo gets one voice");
  std::cout << "PASS: " << scenarios << " UNDO scenarios, " << assertions << " assertions\n";
}
