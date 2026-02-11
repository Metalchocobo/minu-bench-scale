# ANDREA'S AUTONOMOUS AGENT PROTOCOLS

## 1. CORE BEHAVIOR: ACT, DON'T ASK
- **Identity:** Senior Embedded DevOps for Gelateria Minu.
- **Language:** Italian (Output), English (Code/Comments).
- **Tone:** Direct. No apologies. No fluff.
- **Default action:** Do the work. Ask only when genuinely ambiguous.

## 2. USER CONSTRAINTS
- **User:** Andrea.
- Andrea decides scope. Don't expand scope without asking.
- If Andrea says "fix X", fix X. Don't refactor Y while you're at it.

## 3. DYNAMIC DOCUMENTATION (README.md)
- **Rule:** The README is a **state file**, not a log.
- **Auto-Update Trigger:** AFTER any code change that affects functionality.
- **Action:**
  - Scan `README.md`.
  - **Overwrite** outdated sections (Pinout, Libs, Logic) with the code actually written.
  - **Do NOT** append changelogs. Replace old info with new info.
  - Keep section numbering consistent (renumber if sections are added/removed).

## 4. BUILD VERIFICATION
- **Compile command** (verification only — Codex cannot flash hardware):
  ```
  arduino-cli compile --fqbn esp32:esp32:esp32 firmware/esp32_hx711_serial
  ```
- **Note:** Codex runs in a sandbox without hardware access. The actual flash is done by Andrea locally with:
  ```
  "C:\Users\shado\arduino-cli\arduino-cli.exe" compile --fqbn esp32:esp32:esp32 -u -p COM3 "D:\xampp\htdocs\bench-scale\minu-bench-scale\firmware\esp32_hx711_serial"
  ```
- After making firmware changes, **always verify the code compiles** if arduino-cli is available in the sandbox. If not, carefully check for syntax/type errors manually.
- Flash usage is tight: ~91% program space. Be mindful of string literals and large const arrays.

---

# PROJECT TECHNICAL KNOWLEDGE

## Architecture Overview

Two separate projects that communicate via MQTT:

1. **Firmware (ESP32):** `firmware/esp32_hx711_serial/`
   - Arduino IDE project, compiled with arduino-cli
   - Board: `esp32:esp32:esp32` (ESP32 core v3.3.5)
   - Flash: ~91% program, ~17% RAM (tight on flash, be mindful of string literals)

2. **Laravel gestionale:** External project at `D:\xampp\htdocs\minu\manager\` (not in this repo)
   - Laravel 11, Backpack 6, Tabler theme, horizontal layout
   - MQTT credentials in `.env` (MQTT_HOST, MQTT_WSS_PORT, MQTT_USERNAME, MQTT_PASSWORD)

## Communication: MQTT via Mosquitto

- **Broker:** `mqtt.gelateriaminu.it` (DigitalOcean VPS)
- **ESP32 port:** 8883 (MQTTS, TLS over TCP)
- **Browser port:** 8884 (WSS, WebSocket over TLS)
- **Auth:** username/password (stored in NVS on ESP32, in .env on Laravel)
- **TLS cert:** ISRG Root X1 (Let's Encrypt), hardcoded in firmware as C string concatenation (NOT raw string literal)
- **Scale ID:** ESP32 WiFi MAC address, lowercase, no separators (12 hex chars). Example: `841fe838c774`

### MQTT Topics
```
minu/scale/{scale_id}/status    Bilancia -> Browser  (retained, QoS 1)
minu/scale/{scale_id}/command   Browser -> Bilancia   (retained, QoS 1)
minu/scale/{scale_id}/response  Bilancia -> Browser   (not retained, QoS 1)
minu/scale/{scale_id}/owner     Browser -> Browser    (retained, QoS 1)
```

## Credential Storage Pattern (NVS)

All secrets follow the same pattern — **never hardcoded**, always in ESP32 NVS:

| Module | NVS Namespace | Serial Commands | Store File |
|--------|--------------|-----------------|------------|
| WiFi (2 slots) | `minu_wifi` | `wifi set <slot> "SSID" "PASS"`, `wifi creds`, `wifi apply` | `wifi_store.h/.cpp` |
| OTA | `minu_ota` | `ota set "PASS"`, `ota status`, `ota clear` | `ota_store.h/.cpp` |
| MQTT | `minu_mqtt` | `mqtt set "host" "user" "pass"`, `mqtt creds`, `mqtt apply` | `mqtt_store.h/.cpp` |

**Pattern:** Each store has `load()`, `save()`, `clear()`, `isConfigured()`. Uses ESP32 `Preferences` library. Data persists across OTA/USB uploads, lost only on full flash erase.

**CRITICAL:** Never put passwords, API keys or credentials in source code. If you need to add a new secret, create a new `*_store.h/.cpp` pair following the existing pattern.

## ESP32 Gotchas (MUST READ before editing firmware)

### TLS / MQTT Connection
- **SNI required:** PubSubClient resolves DNS and passes IP to WiFiClientSecure, losing the hostname needed for SNI. **Fix:** Manual TLS connect with hostname BEFORE PubSubClient.connect():
  ```cpp
  mqttWifiClient.connect(mqtt_host, MQTT_PORT);  // TLS with hostname (SNI)
  mqttClient.connect(clientId, user, pass, ...);  // MQTT over existing TLS
  ```
- **PEM certificates on Windows:** Raw string literals `R"EOF(...)EOF"` introduce `\r` characters that break PEM parsing. **Fix:** Use C string concatenation with explicit `\n`:
  ```cpp
  static const char CERT[] =
    "-----BEGIN CERTIFICATE-----\n"
    "MIIFaz...\n"
    "-----END CERTIFICATE-----\n";
  ```
  NEVER use `R"EOF(...)EOF"` for certificates in this project.
- **PROGMEM not needed on ESP32:** Unlike AVR, ESP32 stores const in flash by default. Do NOT add `PROGMEM` to cert strings or large const arrays.
- **NTP sync required before TLS:** ESP32 boots with epoch 1970. Cert validation fails without correct time. `configTime()` must be called before any TLS connection attempt. Check `time(nullptr) > 1700000000` before connecting.

### Watchdog Timer (WDT)
- **8-second WDT** configured in firmware. If loop blocks > 8s, ESP32 reboots.
- **TLS handshake can take > 8s.** Must disable WDT during handshake:
  ```cpp
  esp_task_wdt_delete(NULL);  // before handshake
  // ... TLS handshake ...
  esp_task_wdt_add(NULL);     // after handshake
  ```
- **Same for OTA uploads:** WDT disabled in `onStart`, re-enabled in `onEnd`/`onError`.
- **Any new blocking operation > 2s** must handle WDT. Either disable/re-enable or break into non-blocking chunks.

### Light Sleep
- **Do NOT call `esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER)`** if timer wakeup was never enabled. On ESP32 core v3.3.5 this causes a fatal error (`Incorrect wakeup source (4) to disable`). Just call `esp_sleep_enable_gpio_wakeup()` directly.

### PubSubClient
- **Library:** PubSubClient v2.8
- **MQTT_MAX_PACKET_SIZE** must be defined BEFORE including PubSubClient.h (set to 512 in `net_ota_cloud.h`). If you need larger packets, change it there.
- **ArduinoJson** v7.3.0

### General ESP32
- **Flash is 91% full.** Every `F()` macro, every string literal counts. Avoid verbose log messages. Reuse format strings where possible.
- **Loop must be non-blocking.** No `delay()` longer than ~50ms in the main loop. Use state machines and timestamp-based logic.
- **GPIO34-39 are input-only** and have no internal pull-up/pull-down. If using these for digital input, add external pull resistors.

## Firmware Module Structure

```
firmware/esp32_hx711_serial/
├── esp32_hx711_serial.ino    # Main loop + serial commands + key handlers (~1600 lines)
├── config/
│   ├── config_pins.h         # GPIO assignments
│   ├── config_audio.h        # DFPlayer timing, tracks, volume
│   ├── config_scale.h        # Filters, states, zero-tracking, display params
│   └── config_battery.h      # Voltage thresholds, shutdown timing
├── audio.h / .cpp            # Audio:: (DFPlayer FIFO, priority, power-gating)
├── scale_filters.h / .cpp    # ScaleFilters:: (median, MA, spike guard, history)
├── scale_state.h / .cpp      # ScaleState:: (offset, tare, states, ZT, display)
├── net_ota_cloud.h / .cpp    # Net:: (WiFi/OTA/MQTT, TLS, reconnection)
├── mqtt_store.h / .cpp       # MqttStore:: (MQTT credentials in NVS)
├── wifi_store.h / .cpp       # WifiStore:: (WiFi credentials in NVS)
├── ota_store.h / .cpp        # OtaStore:: (OTA password hash in NVS)
├── hx711_driver.h / .cpp     # HX711 low-level (SCK/DOUT, read)
├── hx_health.h / .cpp        # HxHealth:: (OK/WARN/ERROR/ERROR_HARD)
├── battery_monitor.h / .cpp  # BatteryMonitor:: (INA219, levels, charging)
├── ui_display.h / .cpp       # UI:: (OLED SSD1322 256x64, icons, layouts)
├── keypad.h / .cpp           # Keypad:: (4x2, debounce, one-shot)
├── buzzer.h / .cpp           # Buzzer:: (beep, tones)
├── dfplayer_driver.h / .cpp  # DFPlayer:: (UART, base commands)
└── calibration_wizard.h/.cpp # CalWizard:: (on-display calibration wizard)
```

### Key architectural patterns
- **Namespaces:** Each module uses a C++ namespace (`Audio::`, `ScaleFilters::`, `ScaleState::`, `Net::`, `MqttStore::`, etc.)
- **Config headers:** All tunable parameters are in `config/config_*.h`. Change thresholds there, not in implementation files.
- **Serial commands:** Parsed in `parseCommand()` in the .ino. Pattern: `if (strncmp(cmd, "prefix ", N) == 0)` with nested arg parsing. Add new commands following the existing style.
- **Display icons:** All in `ui_display.cpp`. Pixel-level drawing with U8g2. OLED is 256x64 monochrome (SSD1322).

## Integration Spec Files

- `minu-scale-integration-spec.md` — Overall architecture spec (firmware + browser + Laravel)
- `prompt-fase1-firmware-mqtt.md` — Fase 1 prompt (COMPLETED: firmware MQTT client)
- `prompt-fase2-3-browser-laravel.md` — Fase 2-3 prompt (READY: browser JS + Laravel CRUD)

## Mosquitto Server Config

On DigitalOcean VPS, config at `/etc/mosquitto/conf.d/minu.conf`:
- Port 8883: MQTT over TLS (for ESP32)
- Port 8884: WebSocket over TLS (for browser)
- `tls_version tlsv1.2` required for ESP32 compatibility
- Do NOT set `cafile` to the same file as `certfile` (causes Mosquitto error)

## Current State (as of 2026-02-11)

### Completed
- Fase 1: ESP32 firmware MQTT client (weigh/clear commands, confirm/skip responses, TLS, LWT, backoff reconnection, OLED icons, key handlers)
- Credential storage in NVS for WiFi, OTA, MQTT (no secrets in source code)
- Calibration wizard (on-display + serial)
- Battery monitoring with shutdown protection
- HX711 health monitoring

### Not Started
- Fase 2: ScaleMqttClient JavaScript module (browser-side MQTT via mqtt.js)
- Fase 3: Laravel integration (scales CRUD, user-scale association, REST endpoints, weigh view integration)
