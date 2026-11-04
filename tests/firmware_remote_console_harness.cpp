// Compile with production remote_console_state.cpp; no Arduino stubs required.
#include "../firmware/esp32_hx711_serial/remote_console_state.h"

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

using namespace RemoteConsoleCore;

static constexpr uint32_t EPOCH = 1793790000UL;
static const char* scenario = "initialization";
static unsigned int assertions = 0;
static unsigned int scenarios = 0;

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
}

static Request request(Action action, const char* session = "session-one",
                       const char* id = "request-one") {
  Request result;
  result.action = action;
  std::strcpy(result.requestId, id);
  std::strcpy(result.sessionId, session);
  std::strcpy(result.bootId, "1234abcd");
  result.expiresAt = EPOCH + 10;
  return result;
}

static Request exec(uint32_t sequence, const char* id = "exec-one",
                    const char* command = "cal status") {
  Request result = request(Action::Exec, "session-one", id);
  result.sequence = sequence;
  std::strcpy(result.command, command);
  return result;
}

static void expectError(const Result& result, const char* error) {
  require(result.outcome == Outcome::Rejected, "request rejected");
  require(std::strcmp(result.error, error) == 0, error);
  require(!result.queued, "rejected request never queues a command");
}

static void open(Session& session, uint32_t nowMs = 100) {
  session.reset("1234abcd");
  Result result = session.receive(request(Action::Open), nowMs, EPOCH);
  require(result.outcome == Outcome::Ok && result.opened, "new session opens");
}

static void testEnvelopeAndProbe() {
  start("capability probe has no session or sleep lease");
  Session session;
  session.reset("1234abcd");
  Request probe = request(Action::Probe, "");
  probe.bootId[0] = '\0';
  Result result = session.receive(probe, 100, EPOCH);
  require(result.outcome == Outcome::Ok, "probe accepts absent boot fence");
  require(!session.active() && !result.opened, "probe does not open a session");
  std::strcpy(probe.bootId, "deadbeef");
  expectError(session.receive(probe, 100, EPOCH), "boot_changed");

  start("untrusted envelope fences and bounded identifiers");
  Request invalid = request(Action::Open);
  invalid.requestId[0] = '\0';
  expectError(session.receive(invalid, 100, EPOCH), "invalid_request");
  invalid = request(Action::Open, "invalid/session");
  expectError(session.receive(invalid, 100, EPOCH), "invalid_request");
  invalid = request(Action::Invalid);
  expectError(session.receive(invalid, 100, EPOCH), "invalid_request");
  invalid = request(Action::Open);
  invalid.expiresAt = EPOCH - 1;
  expectError(session.receive(invalid, 100, EPOCH), "expired");
  invalid.expiresAt = EPOCH + 31;
  expectError(session.receive(invalid, 100, EPOCH), "invalid_expiry");
  invalid.expiresAt = EPOCH;
  expectError(session.receive(invalid, 100, EPOCH), "expired");
  expectError(session.receive(invalid, 100, 1000), "clock_unsynced");
  invalid = request(Action::Open);
  std::strcpy(invalid.bootId, "deadbeef");
  expectError(session.receive(invalid, 100, EPOCH), "boot_changed");
  invalid = request(Action::Open);
  invalid.bootId[0] = '\0';
  expectError(session.receive(invalid, 100, EPOCH), "boot_changed");
  Request bounded = request(Action::Open);
  std::memset(bounded.sessionId, 'a', ID_SIZE - 1);
  bounded.sessionId[ID_SIZE - 1] = '\0';
  bounded.expiresAt = EPOCH + 30;
  require(session.receive(bounded, 100, EPOCH).outcome == Outcome::Ok,
          "64-byte session and maximum expiry accepted");
}

static void testLeaseAndExclusion() {
  start("single lease excludes a second operator");
  Session session;
  open(session);
  Result result = session.receive(request(Action::Open, "session-two"), 200, EPOCH);
  require(result.outcome == Outcome::Busy && !result.opened, "second open is busy");
  require(std::strcmp(session.sessionId(), "session-one") == 0, "owner unchanged");
  expectError(session.receive(request(Action::Poll, "session-two"), 250, EPOCH),
              "session_closed");
  expectError(session.receive(request(Action::Close, "session-two"), 300, EPOCH),
              "session_closed");
  require(!session.tick(30099), "lease remains active before exact deadline");
  require(session.tick(30100) && !session.active(), "foreign traffic cannot renew lease");
  require(!session.tick(30101), "expiration transition emitted once");

  start("same-session open and poll renew without resetting sequence");
  open(session);
  result = session.receive(exec(1), 1000, EPOCH);
  require(result.queued, "first command queued");
  session.complete();
  result = session.receive(request(Action::Open), 30099, EPOCH);
  require(result.outcome == Outcome::Ok && !result.opened, "same-session open renews");
  require(!session.tick(30100), "renewed lease survives initial deadline");
  result = session.receive(request(Action::Poll), 60098, EPOCH);
  require(result.outcome == Outcome::Ok, "poll renews lease");
  require(!session.tick(90097), "poll lease lasts full duration");
  require(session.receive(exec(2, "exec-two"), 90097, EPOCH).queued,
          "renewals preserve execution sequence");
  session.complete();
  require(session.tick(90098), "execution does not renew lease");

  start("probe, rejected expiry, and monotonic wrap cannot extend lease");
  open(session, UINT32_MAX - 1000);
  require(!session.tick(28998), "wrapped lease active until 30 seconds");
  require(session.tick(28999), "wrapped lease expires at exact duration");
  open(session);
  require(session.receive(request(Action::Probe, ""), 30000, EPOCH).outcome == Outcome::Ok,
          "probe responds while lease still active");
  Request expiredPoll = request(Action::Poll);
  expiredPoll.expiresAt = EPOCH - 1;
  expectError(session.receive(expiredPoll, 30050, EPOCH), "expired");
  require(session.tick(30100), "probe and invalid poll cannot renew lease");

  start("close releases pending execution and next session starts fresh");
  open(session);
  require(session.receive(exec(1), 200, EPOCH).queued, "pending before close");
  require(session.receive(request(Action::Close), 300, EPOCH).outcome == Outcome::Ok,
          "owner can close");
  require(!session.active() && !session.pending(), "close releases all pending state");
  expectError(session.receive(request(Action::Open), 400, EPOCH), "session_closed");
  expectError(session.receive(exec(1), 450, EPOCH), "session_closed");
  require(session.receive(request(Action::Open, "session-two"), 500, EPOCH).opened,
          "a fresh session can open after close");
  Request newCommand = exec(1, "fresh-exec");
  std::strcpy(newCommand.sessionId, "session-two");
  require(session.receive(newCommand, 550, EPOCH).queued, "new session sequence starts at one");
  session.reset("deadbeef");
  expectError(session.receive(exec(1), 600, EPOCH), "boot_changed");
  require(!session.active(), "reset cannot preserve sleep lock");
}

static void testClosedSessionReplayFence() {
  start("closed-session tombstones prevent open plus execution replay");
  Session session;
  open(session);
  Request command = exec(1);
  require(session.receive(command, 150, EPOCH).queued, "original command executes");
  session.complete();
  require(session.receive(request(Action::Close), 200, EPOCH).outcome == Outcome::Ok,
          "original session closes");
  expectError(session.receive(request(Action::Open), 250, EPOCH), "session_closed");
  expectError(session.receive(command, 300, EPOCH), "session_closed");
  require(!session.active() && !session.pending(), "replay cannot resurrect sleep or command lock");
  expectError(session.receive(request(Action::Open), 31199, EPOCH), "session_closed");

  start("bounded tombstone cache never evicts a live replay fence");
  session.reset("1234abcd");
  const char* ids[] = {"session-a", "session-b", "session-c", "session-d"};
  for (uint32_t index = 0; index < 4; ++index) {
    const uint32_t openedAt = 100 + index * 200;
    require(session.receive(request(Action::Open, ids[index]), openedAt, EPOCH).opened,
            "next session opens while fence capacity remains");
    require(session.receive(request(Action::Close, ids[index]), openedAt + 100, EPOCH).outcome == Outcome::Ok,
            "next close records tombstone");
  }
  Result full = session.receive(request(Action::Open, "session-e"), 1000, EPOCH);
  require(full.outcome == Outcome::Busy && !full.opened && !session.active(),
          "cache full rejects new session without evicting fences");
  expectError(session.receive(request(Action::Open, ids[0]), 1100, EPOCH), "session_closed");
  expectError(session.receive(request(Action::Open, ids[3]), 1200, EPOCH), "session_closed");
  Request fresh = request(Action::Open, "session-e");
  fresh.expiresAt = EPOCH + 42;
  require(session.receive(fresh, 32000, EPOCH + 32).opened,
          "expired fences release capacity for fresh request");
  require(std::strcmp(session.sessionId(), "session-e") == 0, "fresh owner installed");
}

static void testExecutionReplayAndPrivacy() {
  start("execution replay cannot repeat configuration or mutate pending command");
  Session session;
  open(session);
  expectError(session.receive(exec(0), 200, EPOCH), "sequence_conflict");
  expectError(session.receive(exec(2), 200, EPOCH), "sequence_conflict");
  Request command = exec(1, "exec-one", "scale code 00-S");
  require(session.receive(command, 200, EPOCH).queued && session.pending(), "command queued once");
  require(std::strcmp(session.command(), command.command) == 0, "queued command preserved");
  Result replay = session.receive(command, 300, EPOCH);
  require(replay.outcome == Outcome::Busy && !replay.queued, "pending retry does not requeue");
  expectError(session.receive(exec(1, "exec-other"), 350, EPOCH), "sequence_conflict");
  expectError(session.receive(exec(1, "exec-one", "scale reset"), 350, EPOCH),
              "sequence_conflict");
  replay = session.receive(exec(2, "exec-two"), 400, EPOCH);
  require(replay.outcome == Outcome::Busy && !replay.queued, "new execution waits for pending one");
  session.complete();
  replay = session.receive(command, 500, EPOCH);
  require(replay.outcome == Outcome::Done && !replay.queued, "completed retry returns cached result");
  expectError(session.receive(exec(2, "exec-one"), 550, EPOCH), "sequence_conflict");
  require(session.receive(exec(2, "exec-two"), 600, EPOCH).queued, "next sequence is accepted");
  session.complete("domain_busy");
  expectError(session.receive(exec(2, "exec-two"), 650, EPOCH), "domain_busy");
  expectError(session.receive(command, 700, EPOCH), "sequence_conflict");

  start("secret-display and injected command rejection consumes definitive sequence");
  open(session);
  Request forbidden = exec(1, "secret-display", "wifi creds showpass 1");
  expectError(session.receive(forbidden, 200, EPOCH), "command_forbidden");
  expectError(session.receive(forbidden, 250, EPOCH), "command_forbidden");
  require(!session.pending(), "forbidden command cannot reach parser");
  require(session.receive(exec(2, "exec-two"), 300, EPOCH).queued,
          "definitive command rejection advances sequence");
  session.complete();
  require(!commandIsSafe("mqtt creds showpass"), "MQTT secrets blocked");
  require(!commandIsSafe("help\rscale reset"), "CR command injection blocked");
  require(!commandIsSafe("help\nscale reset"), "LF command injection blocked");
  require(commandIsSafe("wifi set 1 \"SSID\" \"tab\tvalue\""),
          "quoted configuration values retain USB parser compatibility");
  require(!commandIsSafe(""), "empty command blocked");
  std::string longest(COMMAND_SIZE - 1, 'x');
  require(commandIsSafe(longest.c_str()), "127-byte command accepted");
  longest += 'x';
  require(!commandIsSafe(longest.c_str()), "128-byte command rejected");
  require(commandIsSafe("wifi set 1 \"Local SSID\" \"private-value\""),
          "configuration may accept a password without displaying it");
  require(commandIsSafe("wifi set 1 \"showpass SSID\" \"showpass-secret\""),
          "USB-only option name remains valid inside configuration values");
}

static void append(LogRing& ring, const std::string& value) {
  ring.append(reinterpret_cast<const uint8_t*>(value.data()), value.size());
}

static void testLogCursorsAndUtf8() {
  start("bounded log polling advances cursor and reports overwritten bytes");
  LogRing ring;
  char output[32];
  uint32_t next = 99, oldest = 99, dropped = 99;
  require(ring.read(0, output, sizeof(output), next, oldest, dropped) == 0,
          "empty ring returns empty data");
  require(next == 0 && oldest == 0 && dropped == 0, "empty cursors initialized");
  append(ring, "first\nsecond\n");
  require(ring.read(0, output, 7, next, oldest, dropped) == 6, "bounded chunk size");
  require(std::strcmp(output, "first\n") == 0 && next == 6, "first chunk exact");
  require(ring.read(next, output, sizeof(output), next, oldest, dropped) == 7,
          "next poll reads only remaining data");
  require(std::strcmp(output, "second\n") == 0 && next == 13, "cursor tracks bytes");
  require(ring.read(1000, output, sizeof(output), next, oldest, dropped) == 0 &&
          next == ring.cursor() && dropped == 0, "future cursor clamps without underflow");
  ring.clear();
  std::string generated;
  for (size_t i = 0; i < LOG_SIZE * 3 + 17; ++i) generated += char('A' + i % 26);
  append(ring, generated);
  const uint32_t lost = static_cast<uint32_t>(generated.size() - LOG_SIZE);
  ring.read(0, output, sizeof(output), next, oldest, dropped);
  require(oldest == lost && dropped == lost, "overwrite reports exact loss");
  require(std::string(output) == generated.substr(lost, sizeof(output) - 1),
          "physical ring wrap preserves newest byte order");
  require(next == lost + sizeof(output) - 1, "next cursor includes loss and returned bytes");
  ring.clear();
  require(ring.read(0, nullptr, 0, next, oldest, dropped) == 0 && next == 0,
          "null/zero-capacity output is safe");

  start("UTF-8 chunks never split a codepoint and report overwritten fragments");
  const std::string euro = "\xe2\x82\xac";
  append(ring, std::string("A") + euro + "B");
  require(ring.read(0, output, 4, next, oldest, dropped) == 1 &&
          std::strcmp(output, "A") == 0 && next == 1, "chunk stops before partial UTF-8 codepoint");
  require(ring.read(next, output, 4, next, oldest, dropped) == 3 &&
          std::string(output) == euro && next == 4, "complete UTF-8 codepoint returned");
  require(ring.read(next, output, sizeof(output), next, oldest, dropped) == 1 &&
          std::strcmp(output, "B") == 0, "ASCII after codepoint preserved");
  ring.clear();
  append(ring, euro + std::string(LOG_SIZE - 2, 'x'));
  ring.read(0, output, sizeof(output), next, oldest, dropped);
  require(oldest == 1 && dropped == 3 && output[0] == 'x',
          "overwritten UTF-8 lead removes orphan continuation bytes");

  start("invalid log bytes are sanitized without changing byte cursor semantics");
  ring.clear();
  const uint8_t invalid[] = {'A', 0, 0xc0, 0xaf, 0xed, 0xa0, 0x80,
                             0xf4, 0x90, 0x80, 0x80, 'B'};
  ring.append(invalid, sizeof(invalid));
  require(ring.read(0, output, sizeof(output), next, oldest, dropped) == sizeof(invalid),
          "invalid bytes produce bounded printable replacements");
  require(std::string(output) == std::string("A") + std::string(10, '?') + "B",
          "NUL, overlong, surrogate, and above-Unicode-maximum bytes sanitized");
  require(next == sizeof(invalid) && dropped == 0, "sanitization consumes original bytes exactly");

  start("incomplete UTF-8 waits for later log writes instead of losing characters");
  ring.clear();
  append(ring, std::string("A\xe2", 2));
  require(ring.read(0, output, sizeof(output), next, oldest, dropped) == 1 &&
          std::strcmp(output, "A") == 0 && next == 1, "partial codepoint remains pending");
  append(ring, std::string("\x82", 1));
  require(ring.read(next, output, sizeof(output), next, oldest, dropped) == 0 && next == 1,
          "second byte alone cannot consume incomplete codepoint");
  append(ring, std::string("\xac") + "B");
  require(ring.read(next, output, sizeof(output), next, oldest, dropped) == 4 &&
          std::string(output) == euro + "B" && next == 5,
          "completed codepoint appears once with later bytes");
}

int main() {
  testEnvelopeAndProbe();
  testLeaseAndExclusion();
  testClosedSessionReplayFence();
  testExecutionReplayAndPrivacy();
  testLogCursorsAndUtf8();
  std::cout << "Remote console: " << scenarios << " scenarios, "
            << assertions << " assertions passed\n";
  return 0;
}
