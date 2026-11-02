// Compile together with production scale_identity.cpp and tests/identity_stubs.
#include "../firmware/esp32_hx711_serial/scale_identity.h"
#include <Preferences.h>

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

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
  IdentityTestNvs::state = {};
}

static void expectIdentity(const char* active, const char* saved) {
  require(std::strcmp(ScaleIdentity::code(), active) == 0, "active code");
  const std::string expectedName = std::string("Ronin ") + active;
  require(expectedName == ScaleIdentity::name(), "derived full name");
  require(std::strcmp(ScaleIdentity::savedCode(), saved) == 0, "saved code");
  require(IdentityTestNvs::state.opens == IdentityTestNvs::state.closes,
          "all successful NVS opens are closed");
}

static void seed(const char* value) {
  IdentityTestNvs::state.namespacePresent = true;
  IdentityTestNvs::state.keyPresent = true;
  IdentityTestNvs::state.value = value;
}

int main() {
  start("first boot with missing namespace");
  ScaleIdentity::load();
  expectIdentity("00", "00");
  require(IdentityTestNvs::state.writes == 0, "boot does not write flash");

  start("namespace exists without a code");
  IdentityTestNvs::state.namespacePresent = true;
  ScaleIdentity::load();
  expectIdentity("00", "00");

  start("save preserves zeroes and applies only after reboot");
  ScaleIdentity::load();
  const char* activeName = ScaleIdentity::name();
  require(ScaleIdentity::saveCode("00-S"), "valid code is saved");
  expectIdentity("00", "00-S");
  require(std::strcmp(activeName, "Ronin 00") == 0,
          "network reader keeps the same immutable active name");
  require(IdentityTestNvs::state.value == "00-S", "stored exact string");
  ScaleIdentity::load();
  expectIdentity("00-S", "00-S");

  start("maximum code remains complete");
  seed("Aa09_-012345");
  require(std::strlen("Aa09_-012345") == ScaleIdentity::CODE_MAX_LEN,
          "boundary fixture matches supported length");
  ScaleIdentity::load();
  expectIdentity("Aa09_-012345", "Aa09_-012345");
  require(std::strlen(ScaleIdentity::name()) == ScaleIdentity::NAME_MAX_LEN,
          "full maximum name fits with termination");

  start("invalid input cannot corrupt configuration or JSON");
  seed("01");
  ScaleIdentity::load();
  const char* invalid[] = {nullptr, "", "1234567890123", "00 S", "00\tS",
    "00\nS", "00\rS", "00/S", "00\\S", "00\"S", "00;S", "\xC3\xA0"};
  for (const char* value : invalid) {
    require(!ScaleIdentity::isValidCode(value), "invalid code is rejected");
    require(!ScaleIdentity::saveCode(value), "invalid code is not saved");
    expectIdentity("01", "01");
  }
  require(IdentityTestNvs::state.value == "01", "persistent value unchanged");
  require(IdentityTestNvs::state.writes == 0, "no invalid-input flash writes");
  require(ScaleIdentity::isValidCode("0"), "one-character code is valid");

  start("malformed or oversized stored values fail safely");
  const char* malformed[] = {"", "00 S", "00\"S", "1234567890123"};
  for (const char* value : malformed) {
    seed(value);
    ScaleIdentity::load();
    expectIdentity("00", "00");
  }
  require(IdentityTestNvs::state.writes == 0, "bad storage is not rewritten at boot");

  start("read-open failure falls back without persistence changes");
  seed("02");
  IdentityTestNvs::state.failReadBegin = true;
  ScaleIdentity::load();
  expectIdentity("00", "00");
  require(IdentityTestNvs::state.value == "02", "unread storage is preserved");

  start("read failure falls back safely");
  seed("02");
  IdentityTestNvs::state.failRead = true;
  ScaleIdentity::load();
  expectIdentity("00", "00");

  start("write-open failure preserves active and pending identity");
  seed("01");
  ScaleIdentity::load();
  require(ScaleIdentity::saveCode("02"), "stage valid pending code");
  IdentityTestNvs::state.failWriteBegin = true;
  require(!ScaleIdentity::saveCode("03"), "failed NVS open is reported");
  expectIdentity("01", "02");
  require(IdentityTestNvs::state.value == "02", "prior pending value survives");

  start("write failure preserves active and pending identity");
  seed("01");
  ScaleIdentity::load();
  require(ScaleIdentity::saveCode("02"), "stage valid pending code");
  IdentityTestNvs::state.failWrite = true;
  require(!ScaleIdentity::saveCode("03"), "failed commit is reported");
  expectIdentity("01", "02");
  require(IdentityTestNvs::state.value == "02", "prior committed value survives");

  start("reset becomes effective at the next boot");
  seed("00-S");
  ScaleIdentity::load();
  require(ScaleIdentity::saveCode(ScaleIdentity::DEFAULT_CODE), "reset is saved");
  expectIdentity("00-S", "00");
  ScaleIdentity::load();
  expectIdentity("00", "00");

  std::cout << "PASS: " << scenarios << " identity scenarios, "
            << assertions << " assertions\n";
}
