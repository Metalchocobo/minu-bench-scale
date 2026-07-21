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

## 3. DOCUMENTATION MAINTENANCE
- **Rule:** Documentation is a **state file**, not a log. It must describe the current system, not the history of interventions.
- **Default:** Agents must keep documentation synchronized with the code/config they change in the same task.
- **Read before editing:** For firmware behavior, pinout, wiring, commands, MQTT, power, calibration, or build flow, scan the relevant sections of `README.md`, `docs/WIRING.md`, and `firmware/esp32_hx711_serial/README.md` if present.
- **Auto-Update Trigger:** AFTER any code/config change that affects:
  - pinout, wiring, hardware modules, voltage thresholds, battery/sleep behavior
  - firmware logic, state machines, filters, calibration, serial commands, keypad/OLED/audio behavior
  - WiFi/OTA/MQTT topics, payloads, credentials/storage, TLS/NTP/reconnect behavior
  - build/upload commands, library requirements, module/file structure
- **Documentation targets:**
  - `README.md`: project current state, user workflows, firmware behavior, commands, MQTT, pinout, libraries.
  - `docs/WIRING.md`: physical wiring, pin maps, power paths, hardware caveats.
  - `firmware/esp32_hx711_serial/README.md`: firmware-local quick reference when code changes diverge from it.
  - `AGENTS.md`: durable cross-agent rules only.
  - `CLAUDE.md`: Claude-specific guardrails only, when they must differ from `AGENTS.md`.
- **Action:**
  - **Overwrite** outdated sections with the code actually written.
  - **Do NOT** append changelogs. Replace old info with new info.
  - Keep section numbering consistent (renumber if sections are added/removed).
  - Do not duplicate the same state in multiple docs unless each doc needs it for its audience.
  - Never document secrets or real passwords. Document storage locations/patterns only.
  - If no documentation update is needed after a code change, state that explicitly in the final response.

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
- Flash usage is tight: ~94% program space. Be mindful of string literals and large const arrays.

---

# PROJECT TECHNICAL KNOWLEDGE

## Architecture Overview

Two separate projects that communicate via MQTT:

1. **Firmware (ESP32):** `firmware/esp32_hx711_serial/`
   - Arduino IDE project, compiled with arduino-cli
   - Board: `esp32:esp32:esp32` (ESP32 core v3.3.5)
   - Flash: ~94% program, ~20% RAM (tight on flash, be mindful of string literals)

2. **Laravel gestionale:** External project at `D:\xampp\htdocs\minu\manager\` (not in this repo)
   - Laravel 11, Backpack 6, Tabler theme, vertical layout
   - MQTT credentials in `.env` (MQTT_HOST, MQTT_WSS_PORT, MQTT_USERNAME, MQTT_PASSWORD)

## Communication: MQTT via Mosquitto

- **Broker:** `mqtt.gelateriaminu.it` (DigitalOcean VPS)
- **ESP32 port:** 8883 (MQTTS, TLS over TCP)
- **Browser port:** 8884 (WSS, WebSocket over TLS)
- **Auth:** username/password (stored in NVS on ESP32, in .env on Laravel)
- **TLS cert:** ISRG Root X1 (Let's Encrypt), hardcoded in firmware as C string concatenation (NOT raw string literal)
- **Scale ID:** MAC STA letto dall'eFuse ESP32, lowercase, senza separatori (12 hex chars). È disponibile anche con WiFi OFF; `000000000000` è invalido e blocca MQTT. Esempio: `841fe838c774`

### MQTT Topics

| Topic | Direzione | Retain | QoS effettivo |
|---|---|---|---|
| `minu/scale/{scale_id}/command` | Browser → bilancia | `weigh`/`clear` sì; `confirm_request` no | 1 |
| `minu/scale/{scale_id}/response` | Bilancia → browser | no | 0; `command_ack`/`confirm_request_ack` transitori, response durable con retry applicativo |
| `minu/scale/{scale_id}/ack` | Browser → bilancia | no | 1 |
| `minu/scale/{scale_id}/status` | Bilancia → browser | sì | online/sleeping 0; LWT offline 1 |
| `minu/scale/{scale_id}/owner` | Browser → browser + bilancia | sì | 1 |

Invarianti del protocollo corrente:

- Ogni scheda mantiene il proprio `session_id` durante reload/reconnect. Un Web Lock esclusivo, con fallback cross-tab fail-safe, rileva una copia live dello stesso `sessionStorage` e ruota l'ID della scheda duplicata. Ogni attivazione logica `weigh` usa inoltre un `command_id` univoco di 8–64 caratteri `[A-Za-z0-9_-]`, conservato nei retry soltanto a parità di sessione, UUID, prodotto, nome e target; ogni variazione semantica genera un ID nuovo e il valore vuoto è solo legacy.
- Ogni documento browser usa un `connection_id` distinto e ogni acquisizione owner un `lease_id`; il controllo è valido solo dopo l'eco esatta di utente, sessione, connessione e lease. Il browser non usa un LWT che muti `command`, invalida lo status a ogni reconnect e considera la bilancia pronta solo dopo tutti i SUBACK, status fresco, ownership, retained command coerente e `command_ack`. Da firmware 1.4 il replay dello stesso comando aggiorna solo la connessione e l'ACK la riecheggia, senza ruotare il `command_id`. Da 1.4.1 il firmware sottoscrive anche `owner`: un `weigh` connection-aware è operativo solo con owner fresco ed esatto e, dopo reconnect, dopo lo snapshot retained `command`. Da 1.4.2 il lease di takeover resta 30 s, ma la presenza operativa firmware richiede un heartbeat entro 15 s (heartbeat browser 10 s). La perdita owner o l'assenza prolungata degli snapshot attiva una modalità locale sticky: comando/ingrediente non sono visibili e TARE/ENTER fisici non restano dipendenti dal browser. Un retained `weigh` della connessione sganciata non basta a riattivarla; serve un heartbeat più nuovo o una nuova connessione, seguito da un ACK nuovo. Da 1.4.3 le frecce OLED descrivono soltanto il trasporto MQTT e il badge fisso `APP`/`LOC` descrive separatamente la presenza del Manager. Da 1.4.4 il firmware elabora un burst limitato a quattro pacchetti già disponibili prima di valutare il fallback; il Manager ritenta l'intera coorte se manca un SUBACK o lo status fresco senza rinviare l'heartbeat e, al ritorno in primo piano, rinnova il lease solo dopo avere riletto l'owner retained esatto.
- Senza outbox pending un `clear` live può annullare un comando moderno soltanto quando coincidono `session_id` e `command_id` e il `connection_id` è non vuoto. Con owner fresco deve appartenere alla sua connessione, così i callback tardivi del predecessore vengono respinti; senza owner fresco il retained clear esatto è un tombstone autorevole. Al reconnect, prima di riaprire il gate command snapshot, un retained clear moderno pulisce il raw RAM anche se la coppia broker è già avanzata. Un comando v1.2 senza ID ma session-aware richiede un clear senza ID della stessa sessione. Un comando realmente legacy senza entrambi gli ID accetta un clear senza command ID anche se il Manager nuovo aggiunge la propria sessione; un clear con command ID non cancella mai un comando legacy.
- Per ogni `weigh` tokenizzato attivato il firmware pubblica sul topic `response` un `command_ack` transitorio con `command_id`, `session_id`, `connection_id`, UUID, `product_id` e `state=active`. Un duplicato identico con lo stesso ID non reinizializza il comando e riemette l'ACK; lo stesso ID con contenuto conflittuale viene ignorato senza ACK e non sovrascrive lo stato attivo.
- Il pulsante Conferma del Manager 1.4 pubblica una `confirm_request` QoS 1 non retained con `request_id`, sessione, connessione, comando, UUID e prodotto. Il firmware la accoda e usa lo stesso percorso e lock TARE del tasto ENTER. Retry dello stesso ID riemettono `confirm_request_ack` senza un secondo commit; dopo reconnect il Manager riattiva prima lo stesso `weigh` e attende un `command_ack` fresco. Una cache RAM conserva gli ultimi due `failed`/`rejected`, impedendo la riesecuzione finché l'ID resta in uno dei due slot. Un altro ID durante acquisizione/outbox è rejected. `accepted`/`staged` non avanzano la pagina: solo il `confirm` durabile completato da Laravel, clear e `response_ack` cambia ingrediente. Se dopo `staged` cade il trasporto o viene revocato l'owner esatto, il browser sblocca il manuale dopo 10 secondi senza ripetere ENTER e conserva la fingerprint per una response tardiva.
- I `confirm`/`skip` v1.3 riportano il `command_id` originale quando presente, oltre a UUID ingrediente, `product_id`, `session_id` e `response_id`. `undo` non inventa un command ID: usa `undo_of_response_id` come riferimento durabile.
- CLEAR breve è un undo LIFO reale: lo stack conserva offset/zero-tracking precedenti e la provenienza remota. Per una voce session-aware il firmware emette `undo` con nuovo `response_id` e `undo_of_response_id`, poi applica pop e ripristino locale all'ACK. Se l'undo diventa detached, il restore locale viene applicato subito una volta sola mentre la response continua in background; l'ACK tardivo non lo ripete. Una voce senza receipt viene annullata subito e solo localmente. CLEAR lungo svuota soltanto lo stack locale. Con un outbox già pending ulteriori CLEAR restano serializzati.
- PubSubClient pubblica le response a QoS 0: il firmware conserva una response durable in RAM e la ritenta ogni secondo finché riceve il `response_ack` esatto sul topic `ack`. Il `command_ack` non usa questo outbox.
- Il browser passa prima la response a Laravel. Receipt e mutazione prodotto vengono salvate atomicamente; replay identico = successo idempotente, stesso ID con azione immutabile diversa = conflitto.
- Solo dopo commit/replay Laravel il browser sostituisce il retained `weigh` con clear e attende il PUBACK QoS 1, poi pubblica `response_ack` e attende il relativo PUBACK. Su errore REST non invia né clear né ACK.
- Il PUBACK broker non prova che il firmware abbia ricevuto `response_ack`: uno status non-online invalida la consegna non conclusa e un successivo replay firmware ripete clear + ACK senza ripetere REST o callback UI. Una response dello stesso prodotto riconcilia la pagina anche se UUID/comando sono già avanzati o assenti; un orphan di un altro prodotto non cancella il comando corrente.
- Cambio e disassociazione bilancia sono bloccati durante una `confirm_request` attiva o response in corso; una request staged scaduta lato browser sblocca il manuale/cambio senza poter rieseguire ENTER. Se la response parte durante il PUT di selezione, la transizione MQTT attende i coordinator idle. Solo l'owner esatto può pubblicare clear o release durante disconnect/cambio. I passaggi interni tra ingredienti/modalità e i form che rientrano nella pesatura usano un handoff senza `lifecycle=pagehide`; il detach resta riservato alla vera uscita dal documento.
- La coppia (`scale_id`, `response_id`) identifica univocamente la receipt. La fingerprint confronta `scale_id`, prodotto, UUID ingrediente, azione, peso e riferimento undo; esclude la `session_id`, che resta routing/audit.
- I payload legacy privi di `session_id` restano compatibili, senza outbox/ACK applicativo.
- L'outbox è totalmente immutabile: durante un pending ogni `weigh` e clear normale, incluso il retry dello stesso comando, viene ignorato senza `command_ack` e non può cambiare payload, sessione, command ID o response ID. Il clear speciale `lifecycle=pagehide`, se autorizzato dall'esatta sessione/connessione owner, non chiude l'outbox ma sgancia il lock fisico anche senza comando; cancella il raw soltanto con fence comando esatta. Il retry della `confirm_request` già staged riemette soltanto lo stato; richieste diverse sono busy. Solo il `response_ack` corrispondente chiude l'outbox.
- Il lock operatore dell'outbox dura al massimo 10 secondi e termina prima su pagehide esatto, owner scaduto, indisponibilità remota prolungata o alla prima pressione fisica di TARE/ENTER; quell'input non viene scartato e prosegue localmente. Owner/snapshot mai ricevuti usano una grace di 10 secondi. DNS/TCP/TLS/MQTT CONNECT girano in un worker FreeRTOS a proprietà esclusiva: il loop tastiera/HX711 non attende mai la rete e prende il client soltanto dopo l'handoff atomico. Dopo il detach TARE/ENTER fisici sono locali e non creano un secondo outbox; SKIP/CLEAR remoto, long-press e mutazioni del wizard/seriale restano serializzati. In fallback anche il light-sleep è ammesso: outbox RAM e retry riprendono al wake.
- Con un owner vivo, ENTER richiede un comando `weigh` attivo. Senza consumer remoto, oppure dopo il timeout operatore dell'outbox, ENTER usa esplicitamente il commit standalone anche con MQTT connesso; un nuovo `weigh` valido riattiva la modalità gestita.

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
- **Reconnect MQTT asincrono:** DNS/TCP/TLS/MQTT CONNECT e SUBSCRIBE devono restare nel worker dedicato, che non è registrato al WDT del loop. Il main task non deve accedere a `mqttWifiClient`/`mqttClient` mentre il worker è `RUNNING`; abort e handoff sono cooperativi.
- Dopo CONNECT il timeout read/write TLS è 500 ms e PubSubClient usa socket timeout 1 s. Il timestamp di retry outbox va aggiornato dopo ogni `publish`, così una write lenta non concatena immediatamente il tentativo successivo sul loop.
- **OTA upload:** il task viene rimosso dal WDT in `onStart` e riaggiunto/ripristinato in `onEnd` e `onError`.
- **Recovery coerente:** dopo WDT non ripristinare soltanto offset/zero-tracking da RTC. Stack, riferimento, zero runtime e filtri vengono invalidati insieme; la UI informa del reset e avvia una nuova Auto-TARE robusta. Il rumore non blocca il recupero, mentre campioni insufficienti o calibrazione invalida richiedono un retry TARE. La calibrazione NVS resta valida.
- **Ogni nuova operazione bloccante >2 s** deve essere spezzata in step non bloccanti, alimentare esplicitamente il WDT o usare la stessa reconfiguration temporanea con ripristino garantito.

### Light Sleep
- **Do NOT call `esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER)`** if timer wakeup was never enabled. On ESP32 core v3.3.5 this causes a fatal error (`Incorrect wakeup source (4) to disable`). Just call `esp_sleep_enable_gpio_wakeup()` directly.
- Prima del DISCONNECT pulito MQTT pubblicare retained `state=sleeping`; il LWT `offline` copre solo le disconnessioni impreviste.
- Un comando operativo o una response outbox ancora agganciata al Manager bloccano lo sleep per inattività; dopo il fallback locale non lo bloccano più e il retry RAM riprende al wake. Ogni comando accettato resetta il timer.
- Il tasto che provoca il wake deve restare soppresso fino al rilascio, senza generare anche un'azione applicativa.

### PubSubClient
- **Library:** PubSubClient v2.8
- Il define locale `MQTT_MAX_PACKET_SIZE` resta impostato a 512 prima dell'include, ma non garantisce da solo la dimensione del buffer della libreria compilata separatamente.
- `mqttClient.setBufferSize(512)` deve essere chiamato e verificato a runtime in setup e dopo il reload delle credenziali. Se serve una dimensione diversa, aggiornare coerentemente define, chiamata runtime e payload buffer.
- **ArduinoJson** v7.3.0

### General ESP32
- **Flash is 94% full.** Every `F()` macro, every string literal counts. Avoid verbose log messages. Reuse format strings where possible.
- **Loop must be non-blocking.** No `delay()` longer than ~50ms in the main loop. Use state machines and timestamp-based logic.
- **GPIO34-39 are input-only** and have no internal pull-up/pull-down. If using these for digital input, add external pull resistors.
- INA219: soltanto letture I2C riuscite, finite, plausibili e fresche possono aggiornare filtri, charging o protezione sleep; la soglia hard-low richiede conferma temporale.
- Il peso reale oltre il campo ±16 kg entra in **SOVRACCARICO** con isteresi e blocca ENTER/TARE; non registrare mai il valore clamped come una pesata valida.
- SKIP breve scatta al rilascio; SKIP tenuto 5 s apre la calibrazione senza emettere prima uno skip MQTT.
- Auto-TARE e TARE manuale usano trimmed mean su 64 campioni: il range dovuto a vibrazioni ambientali è diagnostico, non un veto. Al boot bastano 32 campioni validi per avviare e lasciare allo zero-tracking la rifinitura nelle sole finestre quiete; la manuale rifiuta soltanto una deriva sostenuta oltre 10 g/s. Il buzzer segnala sempre un errore reale anche senza DFPlayer.

## Firmware Module Structure

```
firmware/esp32_hx711_serial/
├── esp32_hx711_serial.ino    # Main loop + serial commands + key handlers
├── config/
│   ├── config_pins.h         # GPIO assignments
│   ├── config_audio.h        # DFPlayer timing, tracks, volume
│   ├── config_scale.h        # Filters, states, zero-tracking, display params
│   └── config_battery.h      # Voltage thresholds, shutdown timing
├── audio.h / .cpp            # Audio:: (DFPlayer FIFO, priority, power-gating)
├── scale_filters.h / .cpp    # ScaleFilters:: (median, MA, spike guard, history)
├── scale_state.h / .cpp      # ScaleState:: (offset, tare, states, ZT, display)
├── net_ota_cloud.h / .cpp    # Net:: (WiFi/OTA/MQTT, TLS, status, response outbox)
├── mqtt_store.h / .cpp       # MqttStore:: (MQTT credentials in NVS)
├── wifi_store.h / .cpp       # WifiStore:: (WiFi credentials in NVS)
├── ota_store.h / .cpp        # OtaStore:: (OTA password hash in NVS)
├── hx711_driver.h / .cpp     # HX711 low-level (SCK/DOUT, read)
├── hx_health.h / .cpp        # HxHealth:: (OK/WARN/ERROR/ERROR_HARD)
├── battery_monitor.h / .cpp  # BatteryMonitor:: (INA219, validity/freshness, charging)
├── ui_display.h / .cpp       # UI:: (OLED SSD1322 256x64, icons, layouts)
├── keypad.h / .cpp           # Keypad:: (4x2, debounce, one-shot, wake suppression)
├── buzzer.h / .cpp           # Buzzer:: (beep, tones)
├── dfplayer_driver.h / .cpp  # DFPlayer:: (UART, base commands)
├── weigh_stack.h / .cpp      # WeighStack:: (reversible LIFO + response metadata)
└── calibration_wizard.h/.cpp # CalWizard:: (on-display calibration wizard)
```

### Key architectural patterns
- **Namespaces:** Each module uses a C++ namespace (`Audio::`, `ScaleFilters::`, `ScaleState::`, `Net::`, `MqttStore::`, etc.)
- **Config headers:** All tunable parameters are in `config/config_*.h`. Change thresholds there, not in implementation files.
- **Serial commands:** Parsed in `parseCommand()` in the .ino. Pattern: `if (strncmp(cmd, "prefix ", N) == 0)` with nested arg parsing. Add new commands following the existing style.
- **Display icons:** All in `ui_display.cpp`. Pixel-level drawing with U8g2. OLED is 256x64 monochrome (SSD1322).

## Integration Spec Files

La specifica end-to-end e la documentazione operativa Laravel sono nel repository Manager esterno:

- `D:\xampp\htdocs\minu\manager\minu-scale-integration-spec.md`
- `D:\xampp\htdocs\minu\manager\docs\agent-knowledge\06-scale-mqtt.md`

I file `prompt-fase*.md` del Manager sono riferimenti superati e non definiscono il protocollo corrente.

## Mosquitto Server Config

On DigitalOcean VPS, config at `/etc/mosquitto/conf.d/minu.conf`:
- Port 8883: MQTT over TLS (for ESP32)
- Port 8884: WebSocket over TLS (for browser)
- `tls_version tlsv1.2` required for ESP32 compatibility
- Do NOT set `cafile` to the same file as `certfile` (causes Mosquitto error)

## Current State (as of 2026-07-21)

- Firmware ESP32 operativo: TLS/NTP, reconnect MQTT su worker non bloccante, status `online/sleeping/offline`, backoff, `weigh`/`clear` fenced da `command_id`, ENTER remoto idempotente, `command_ack`, `confirm`/`skip`/`undo`, outbox immutabile con retry/ACK applicativo, fallback locale con lock operatore limitato a 10 secondi e badge OLED fisso `APP`/`LOC` separato dall'icona trasporto MQTT.
- Credenziali WiFi, OTA e MQTT persistite in NVS; nessun segreto nel sorgente.
- Auto-TARE fail-soft sul rumore e TARE manuale tollerante alle vibrazioni, INA219 validato/fresh, hard-low debounced, sovraccarico esplicito, reset WDT coerente, wake key consumato e stack undo LIFO operativi.
- Browser MQTT operativo nel Manager con identità sessione/connessione separata, owner lease esatto, refresh `weigh`/ACK a ogni eco owner 1.4.2, clear `lifecycle=pagehide` owner-only anche senza comando o durante response, REST-before-ACK, deduplica response e compatibilità legacy.
- Laravel operativo con CRUD bilance, associazione utente-bilancia, discovery MQTT, pagina pesatura e receipt idempotenti per confirm/skip/undo.
- `MQTT_FW_VERSION` corrente: `1.4.4`.
