# Minù Bench Scale — ESP32 + HX711 + OLED SSD1322 + INA219 (SLA 6 V) · v2026-01-01

Bilancia da banco per uso interno in laboratorio (gelato/pasticceria), pensata per guidare e rendere affidabili le pesate ingredienti (non “pesatura legale”).

Hardware di riferimento:
- **ESP32 DevKit**
- **HX711** (ADC 24 bit) per cella di carico con eccitazione a **5 V**
- **OLED SSD1322** 256×64 (3,12", SPI)
- **Batteria piombo SLA 6 V** con modulo caricatore/protezione **CTK3S**
- **INA219** (CJMCU-219) per tensione/corrente batteria (high-side)
- **Tastierino 4×2** + **buzzer**

---

## 0) Stato e obiettivo

Target:
- Portata: **20 kg**
- Lettura visualizzata: **1 g** (clamp display a ±16 kg per sicurezza)

Funzioni principali (firmware HX711):
- Lettura HX711 con filtri + stati **STABLE / UNSTABLE / LIVE**
- **Zero-tracking** vicino allo zero + **snap-to-zero** allo scarico
- **TARA** con UI dedicata (testo + barra), blocco pesata durante l’operazione
- Monitor batteria con **tacche** + stato **charging** (stabilizzato)
- Protezione “batteria scarica” con **avviso + beep + light-sleep**, anti-flapping (debounce 10 s, recovery stabile 30 s) e cooldown avvisi 5 min
- **HX health** runtime: stati **OK / WARN / ERROR / ERROR HARD**, badge **"Errore Cella"** (testo piccolo + triangolino) in alto a destra in WARN, schermata error bloccante (senza mostrare l'ultimo peso), blocco tara/calib, audio 0015.mp3 una sola volta all’ingresso in ERROR

### HX health (runtime, HX711)

Scopo: gestire errori runtime dell’HX711 **senza blocchi in loop** (logica solo a timestamp) e senza impattare feature non correlate.

**Soglie (ms)**
- WARN se nessun campione valido per **≥ 500 ms**
- ERROR se nessun campione valido per **≥ 3.000 ms**
- ERROR HARD se nessun campione valido per **≥ 30.000 ms** oppure **mai** registrato un valore valido
- Ritorno a OK solo dopo campioni validi “OK stabile” per **≥ 800 ms**

**Cosa conta come “campione valido”**
- Valido quando nel loop: `hx711_is_ready()` è true e `hx711_read()` completa.
- Mitigazione “DOUT mascherato” (es. partitore/pull-down): se il raw resta inchiodato (±1 count) per **> 1500 ms**, quei campioni vengono trattati come **non validi** per HX health (non aggiornano `lastRawMs`). Le soglie WARN/ERROR si conteggiano dall’inizio della flatline.
- `hx711_is_ready() == true` e lettura `hx711_read()` completata nel normale loop (niente medie bloccanti).

**Nota importante (cablaggio DOUT su ESP32)**
- Se **DOUT è su GPIO34..39** (es. **GPIO35**), l’ESP32 **non ha pull-up interni**: a modulo scollegato il pin può fluttuare e risultare “ready” a caso.
- In quel caso la rilevazione “modulo non trovato” non è affidabile solo via firmware.
- Soluzioni robuste: spostare DOUT su un GPIO con pull-up (es. GPIO22) oppure aggiungere una pull-up esterna verso 3V3.

**Stati e comportamento**

| Stato | Quando | UI | Audio | Azioni |
|---|---|---|---|---|
| OK | campioni regolari | UI normale | nessuno | tara/calib abilitate |
| WARN | assenza campioni ≥ 500 ms | triangolino warning alto a destra | nessuno | nessun blocco UI/tasti |
| ERROR | assenza campioni ≥ 3.000 ms | schermata ERROR bloccante, mostra “Ultimo valore valido” se non più vecchio di 30 s | 0015.mp3 **una sola volta** all’ingresso | **tara/calib disabilitate** |
| ERROR HARD | assenza campioni ≥ 30.000 ms o nessun valore valido mai registrato | schermata ERROR bloccante, **non** mostra valore | 0015.mp3 una sola volta all’ingresso | **tara/calib disabilitate** |

---

## 1) Perché HX711 (e perché non NAU7802)

Vincolo emerso sul campo: la cella di carico è stabile/precisa solo se eccitata a **~5 V**.  
Con NAU7802 si è provato a lavorare a 5 V, ma in un setup reale si è verificato **backfeed verso GPIO ESP32** (tensione logica troppo alta) con danni hardware.

Scelta attuale:
- HX711 alimentato a **5 V**
- ESP32 resta a **3,3 V sui GPIO**
- La linea **DOUT va resa 3,3 V safe** (vedi cablaggio)

Firmware disponibili nel repo:
- ✅ `firmware/esp32_hx711_serial/` → corrente
- ⚠️ `archive/esp32_nau7802_serial/` → storico (non consigliato)
- 🧱 eventuali cartelle “legacy” → archivio, non usare per nuove modifiche

---

## 2) Alimentazione (architettura)

Schema logico:
1. Alimentatore → ingresso PV CTK3S  
2. CTK3S BAT → batteria SLA 6 V  
3. CTK3S LOAD → **buck 5 V**  
4. Rail 5 V dal buck alimenta: **ESP32 (VIN/5V)**, **HX711**, **OLED** (se il tuo modulo accetta 5 V)

Motivo: il CTK3S protegge la batteria staccando **LOAD** in undervoltage.

Condensatori consigliati (stabilità rail 5 V):
- Ingresso buck: **220 µF** + **100 nF**
- Uscita buck (rail 5 V): **470 µF** + **100 nF**
- Vicino ESP32 (VIN↔GND): **10–47 µF** + **100 nF**

---

## 3) Pinout (firmware HX711)

| Funzione | Modulo | GPIO ESP32 | Note |
|---|---|---:|---|
| I2C SDA | INA219 | 32 | attenzione a non invertire con SCL |
| I2C SCL | INA219 | 33 | |
| HX711 SCK | HX711 | 16 | spesso serigrafato **RX2** |
| HX711 DOUT | HX711 | 35 | input-only, scelto per evitare strap pin |
| SPI SCK | OLED SSD1322 | 18 | VSPI |
| SPI MOSI | OLED SSD1322 | 23 | |
| SPI CS | OLED SSD1322 | 25 | |
| SPI DC | OLED SSD1322 | 26 | |
| SPI RST | OLED SSD1322 | 27 | |
| Tastiera R1 | Keypad | 17 | |
| Tastiera R2 | Keypad | 5 | |
| Tastiera R3 | Keypad | 13 | |
| Tastiera R4 | Keypad | 14 | |
| Tastiera C1 | Keypad | 19 | |
| Tastiera C2 | Keypad | 21 | |
| Buzzer | buzzer passivo | 22 | LEDC |

---

## 4) Cablaggio HX711 (importante)

### 4.1 Cella di carico → HX711
Collega i 4 fili della cella su **E+, E-, A+, A-** (colori variabili: verifica lo schema della tua cella).

### 4.2 HX711 → ESP32 (logica)

**Alimentazioni**
- HX711 **VCC → 5 V**
- HX711 **GND → GND comune**

**SCK (clock)**
- ESP32 GPIO16 → HX711 SCK **diretto (3,3 V va bene)**  
- Consigli pratici:
  - **220 Ω in serie** sul filo SCK vicino all’ESP32 (anti-ringing/spike)
  - **100 kΩ pulldown** tra SCK e GND vicino al modulo HX711 (tiene SCK LOW durante boot/reset)

> Non usare i classici level shifter bidirezionali “I2C-style” (BSS138) su **SCK**: possono tenere la linea alta e mandare HX711 in **power-down**.

**DOUT (data)**
Se HX711 è a 5 V, **DOUT può andare a 5 V**: va reso “3,3 V safe”.

Soluzione consigliata: **partitore resistivo**
- DOUT (HX711) → **10 kΩ** → GPIO35 (ESP32)
- GPIO35 (ESP32) → **20 kΩ** → GND

(Altri valori equivalenti vanno bene, l’obiettivo è restare ~≤3,3 V sul pin ESP32).

Nota pratica:
- Se colleghi DOUT diretto, prima **misura** con multimetro: se HIGH >3,6 V, è rischioso (nel tempo può rovinare l’ESP32).

---

## 5) HX711: 10 SPS vs 80 SPS

La velocità **non** si imposta da software sull’HX711: dipende dal pin/ponte **RATE** del breakout.

- Tipico default: **10 SPS**
- Per passare a **80 SPS**: modifica la pista/ponticello “RATE” sul retro del modulo (dipende dal modello)

---

## 6) Firmware: comportamento “80 Hz senza rumore a display”

Con HX711 a **80 SPS**:
- il firmware legge tutti i campioni,
- ma **decima/filtra** diversamente per WORK vs LIVE.

Concetto:
- **WORK**: più stabile (decisioni di stato, zero-tracking, ecc.)
- **LIVE**: più reattivo a display, con anti-flicker (e smoothing leggero EMA sul float)

Impostazioni attuali (HX711=80 SPS):
- WORK: decimazione **N=5**
- LIVE: decimazione **N=3** + **EMA veloce** sul valore in grammi prima della quantizzazione (alpha ~0.45)


### Display: anti-flicker (novità)
In LIVE e WORK la visualizzazione è **quantizzata a grammo** ma con due accorgimenti per evitare tremolii:
1) **Banda zero con isteresi** (enter/exit):
   - **LIVE**: 0 finché `|g| < 1.0 g` (entry). Dopo che sei entrato, torna a 0 solo sotto ~`0.8 g` (exit).
   - **WORK**: 0 finché `|g| < 1.3 g` (entry). Rientro a 0 solo sotto ~`0.9 g` (exit).
2) **Isteresi tra interi**: riduce il flicker tra N e N+1 mentre pesi (senza introdurre filtri “lenti” sul segnale).
3) **Reversal lock (solo LIVE)**: quando il display cambia valore, per una finestra breve ignora il rimbalzo immediato al valore precedente.
   Effetto: elimina il flicker A↔B senza ritardare l’incremento progressivo mentre versi.
   Eccezione: 0 → ±1 è **immediato** appena tocchi 1 g (anche per un istante).

### Anti-spike guard (RAW, novità)
Taglia i glitch singoli (es. un salto momentaneo a valori negativi/assurdi): se un campione fa un salto > **150 g**,
viene scartato e accettato solo se il campione successivo conferma (oppure se anche il successivo resta oltre soglia,
così non rallentiamo i carichi rapidi).

Nota: la guard scarta solo **frame singoli** (o confermati) e quindi non cambia la logica WORK/zt/stati, se non in presenza di glitch reali.

### TARE (novità)
- Rimossa la fallback **blocking**: se per qualsiasi motivo arrivano pochi campioni, l’offset viene applicato con quelli disponibili senza congelare UI/audio.

### Auto-TARE al boot (novità)
- Approccio “robusto”: discard iniziale + fino a **64 campioni** e **trimmed-mean** (taglia outlier) per ridurre i casi di “0 → +1 g” dopo l’avvio.

I parametri (N di media, isteresi, ecc.) sono nel firmware e sono pensati per essere ritoccati in base al tuo rumore reale.

---

## 7) TARA (UI e logica)

Tasto **TARE**:
- avvia tara e mostra “**- TARA -**” + barra di stabilizzazione
- durante la tara la pesata è “bloccata” a display (l’utente non deve pesare)
- al termine torna alla UI normale

Obiettivo: evitare tara che finisce a **±1 g** a causa di rumore/assestamento.

---

## 8) Batteria SLA 6 V (INA219)

### 8.1 Collegamenti INA219
- VCC → **3V3**
- GND → GND
- SDA → GPIO32
- SCL → GPIO33
- VIN+/VIN−: high-side sulla linea batteria (vedi schema del tuo cablaggio)

Se INA non viene trovato:
- prima cosa: controlla **SDA/SCL non invertiti**.

### 8.2 Soglie tacche (default firmware, tensione filtrata)
Soglie “pratiche” (dipendono da carico/temperatura). Sono tarate per **usabilità UI** (tacche), non per SoC perfetto:
- **4 tacche (FULL)**: ≥ **6.20 V**
- **3 tacche (GOOD)**: ≥ **6.08 V**
- **2 tacche (LOW)**:  ≥ **5.95 V**
- **1 tacca (CRITICAL)**: ≥ **5.85 V**
- **0 tacche (EMPTY)**: < **5.85 V**

Nota: c'è un **cuscinetto** tra 0 tacche e lo stacco per batteria. La sequenza di countdown parte sotto **5.80 V**.

### 8.3 Rilevamento “in carica” (stabilizzato)
L’icona charging è basata sulla **corrente** (negativa = entra in batteria) con:
- isteresi (start/stop),
- debounce temporale,
- **min-on time** (per evitare flicker quando il caricatore/PWM stacca a impulsi).

Default firmware:
- entra in carica se **I < −80 mA**
- esce da carica se **I > −20 mA**
- debounce ingresso **1,5 s**, uscita **10 s**
- min-on **20 s**

---

## 9) Batteria scarica: avviso + light-sleep (protezione ESP)

La protezione qui è pensata per l’ESP32 (evitare latenze/reset quando il buck 5V perde margine).

Comportamento:
- **0 tacche (EMPTY)**: icona batteria **lampeggiante** + avviso sonoro
  - **0011.mp3** (batteria bassa) con **cooldown 5 min**
  - beep buzzer con **cooldown 5 min**
  - Nota: l'avviso 0011 è agganciato al livello **EMPTY (0 tacche)**, non a una finestra fissa di volt.
- Se V scende sotto **5,80 V** per ≥ **5 s**:
  - schermo “batteria scarica, collega alimentatore”
  - **0012.mp3** (batteria critica) **subito all'ingresso countdown** e **di nuovo a metà** (a ~60 s)
  - beep ogni **10 s** per **120 s**
  - poi entra in **LIGHT-SLEEP**
- Prima del light-sleep per batteria scarica: DFPlayer viene **spento** via MOSFET (zero consumo audio in sleep).
- Anti-troncamento audio (DFPlayer): applicato un gap ~200 ms tra stop → play e ignorati i primi ~300 ms di BUSY=idle dopo il play (evita tagli tipo 0001→0002).
  - Nota HW: BUSY su GPIO39 non ha pull-up interno; se il BUSY resta instabile, aggiungere pull-up esterno verso 3V3 (10k..47k).
- Wake: premendo un tasto (la tastiera risveglia, poi l’ESP32 riparte con reboot pulito)

---


---

## 10) Risparmio energetico per inattività (5 minuti)
Se per **5 minuti** non viene premuto alcun tasto **e il peso visualizzato resta fermo entro ±5 g**, la bilancia entra in **LIGHT-SLEEP**.

Caratteristiche:
- Wake: **qualsiasi tasto**.
- L'inattività viene azzerata anche da una variazione del peso visualizzato **> 5 g** (in LIVE o WORK).
- **Nessun reset** dello stato/pesata: riprende esattamente dove era.
- WiFi/OTA vengono sospesi prima dello sleep e riattivati dopo il wake **solo se l'utente ha lasciato il WiFi ON**.
- **Tasto SLEEP**: forza la sequenza di standby (schermata **Zzz...** per ~5 s + audio 0003), poi light-sleep.

### Indicatore esterno sleep (LED)
Quando il display è spento, per capire che la bilancia è in sleep, usa un LED su:
- **GPIO15** → **resistenza 330–2.2kΩ** → **anodo LED** → catodo a **GND** (HIGH = LED acceso).

## 11) WiFi + OTA (opzionale)

Il firmware HX711 supporta:
- WiFi **non bloccante** (background)
- OTA via Arduino IDE (porta di rete)

Dove si configura:
- Le credenziali **non** stanno nel codice e **non** vanno versionate.
- Si impostano via **seriale** e vengono salvate in **NVS** (persistono tra aggiornamenti firmware).

Note pratiche:
- di default è attivo **ENABLE_WIFI_OTA = 1**
- l’icona WiFi sul display segue lo stato di connessione (se WiFi è abilitato a compile-time)

Configurazione credenziali (max 2 reti, via seriale):
- `wifi set 1 "SSID" "PASS"`
- `wifi set 2 "SSID" "PASS"`
- `wifi creds` (lista SSID con password mascherata)
- `wifi creds showpass 1` / `wifi creds showpass 2` (recupero: stampa password in chiaro)
- `wifi clear 1` / `wifi clear 2` / `wifi clear all`
- `wifi apply` (ricarica credenziali e riavvia i tentativi senza reboot)

Password OTA (consigliata):
- `ota status`
- `ota set "PASS" [reboot]`
- `ota clear [reboot]`
- `ota reboot`

Note: la password non viene salvata in chiaro: in NVS resta solo un hash (MD5). Per rendere effettivi set/clear serve un reboot.

Persistenza:
- le credenziali restano in NVS anche dopo OTA/upload standard.
- si perdono solo con un **erase flash** completo o modifiche invasive alle partizioni.

Tasti:
- **WIFI**: abilita/disabilita il WiFi in modo **persistente** (salvato in NVS).
  - Feedback immediato al tasto: **bip** (buzzer). La connessione vera e propria avviene in background.
  - All’avvio il WiFi segue la preferenza utente, non l’ultimo stato momentaneo (es. WiFi spento in sleep).
  - Durante il light-sleep per inattività il WiFi viene comunque spento per consumi, ma al wake viene riattivato **solo** se la preferenza è ON.
  - Audio:
    - **0005.mp3**: Wi‑Fi modulo ON (toggle manuale)
    - **0006.mp3**: Wi‑Fi modulo OFF (toggle manuale)
    - **0008.mp3**: connessione Wi‑Fi avvenuta (anche in background)
    - **0007.mp3**: errore connessione Wi‑Fi (connect failed / SSID non disponibile, con cooldown)
    - Dopo un wake da light-sleep la riconnessione **non** viene annunciata (comportamento come prima).

- **MODE**: cambio modalità WORK/LIVE con **bip** immediato (buzzer) + audio **0017/0018.mp3**.

Se non ti serve OTA/WiFi:
- imposta `#define ENABLE_WIFI_OTA 0` per ridurre consumi e complessità

---

## 12) Troubleshooting rapido (i classici)

**DOUT sempre HIGH / letture 0 fisse**
- spesso SCK resta HIGH → HX711 in power-down
- verifica pulldown 100 kΩ su SCK e che non ci sia un level shifter “I2C” su SCK

**INA219 “non trovato”**
- SDA/SCL invertiti (errore più comune)
- indirizzo diverso da 0x40 (raro, ma possibile)

**Rumore alto a 80 SPS**
- normale: si gestisce con decimazione/filtri e con layout/cavi puliti (cella, massa, schermature se serve)

---

## 13) Firmware e cartelle

Firmware corrente (HX711):
- `firmware/esp32_hx711_serial/`

Archivio (legacy, non mantenuto):
- `archive/esp32_nau7802_serial/`

> Nota: la documentazione operativa (cablaggi, comandi seriali, DFPlayer, sleep, debug) è in questo README.

### Architettura modulare

Il firmware è organizzato in moduli con namespace C++ per una migliore manutenibilità:

```
firmware/esp32_hx711_serial/
├── esp32_hx711_serial.ino   # Main loop (~1,400 righe)
├── config/
│   ├── config_pins.h        # Pin hardware (HX711, I2C, SPI, keypad, buzzer, DFPlayer)
│   ├── config_audio.h       # AudioConfig:: (volume, timing, tracce MP3)
│   └── config_scale.h       # ScaleConfig:: (filtri, stati, ZT, display)
├── audio.h / audio.cpp      # Audio:: (DFPlayer: play, coda, power-gating)
├── scale_filters.h / .cpp   # ScaleFilters:: (mediana, MA, spike guard, storico)
├── scale_state.h / .cpp     # ScaleState:: (offset, tara, stati, ZT, display)
├── dfplayer_driver.h / .cpp # DFPlayer:: (UART, comandi base)
├── buzzer.h / .cpp          # Buzzer:: (beep, toni)
├── hx711_driver.h / .cpp    # HX711 low-level (SCK/DOUT, read)
├── keypad.h / .cpp          # Keypad:: (4x2, debounce, one-shot)
├── battery_monitor.h / .cpp # BatteryMonitor:: (INA219, tacche, charging)
├── hx_health.h / .cpp       # HxHealth:: (OK/WARN/ERROR/ERROR_HARD)
├── ui_display.h / .cpp      # UiDisplay:: (OLED SSD1322, layout, icone)
├── scale_core.h / .cpp      # ScaleCore:: (calibrazione, NVS)
└── net_ota_cloud.h          # WiFi/OTA (opzionale)
```

**Namespace principali:**
- `Audio::` — gestione DFPlayer (coda FIFO, priorità, power-gating, anti-troncamento)
- `ScaleFilters::` — filtri segnale (mediana 3, media mobile, spike guard, storico range/slope)
- `ScaleState::` — macchina a stati (STABLE/UNSTABLE/LIVE), zero-tracking, tara, quantizzazione display
- `ScaleConfig::` / `AudioConfig::` — parametri configurabili (soglie, timing, tracce)

### DFPlayer Mini (audio eventi) + power-gating solo in standby
Il firmware può suonare file MP3 (es. avviso sleep). Per evitare click e stati strani, **non fa power-cycle a fine brano**.

Comportamento attuale:
- DFPlayer viene portato in stato **pronto** automaticamente **all’avvio** e a ogni **wake** (alimentazione ON + UART init + volume), così un suono può partire subito.
- La riproduzione MP3 è progettata per essere **non bloccante**: mentre l’audio suona, la bilancia continua a leggere HX711 e ad aggiornare UI/log.
- **Gestione priorità audio**: alcuni MP3 sono **non interrompibili** e, se richiesti mentre un altro non interrompibile sta suonando, vengono messi in **coda FIFO** (solo fra loro). Tutti gli altri MP3 sono **interrompibili**: interrompono l’audio interrompibile in corso e ripartono subito, **senza accodarsi**. Se è in corso un non interrompibile, le richieste interrompibili vengono ignorate (resta il **bip** del buzzer sui tasti).
  - Non interrompibili: **0002, 0007, 0008, 0012, 0013, 0014, 0015, 0016**.
- Durante l’uso resta alimentato.
- Viene spento **quando la bilancia entra in standby/light-sleep per inattività** e anche **prima del light-sleep per batteria scarica**.

Nota: il firmware mantiene comunque il percorso “cold-start” sul primo `mp3` (utile se in futuro vuoi tornare all’accensione on-demand).

### Collegamenti minimi
- **ESP32 GPIO4 (TX1)** → **1kΩ in serie** → **DFPlayer RX**
- **DFPlayer TX** → **ESP32 GPIO34 (RX1)** (opzionale, ma consigliato)
- **DFPlayer BUSY** → **ESP32 GPIO39**
  - BUSY è tipicamente **3.3V** (ok per ESP32)
  - GPIO39 non ha pull interni: se a riposo il segnale risulta instabile/flottante, aggiungi **pull-up 10k..47k a 3V3**
  - evita il pulldown a GND (rischi di leggere "busy" fisso)
- **Altoparlante**: usa **SPK1/SPK2** (8Ω ok)

### Power-gating (high-side) consigliato
- **ESP32 GPIO2** comanda l'alimentazione DFPlayer (HIGH=ON) tramite **NPN + P-MOSFET high-side**.

> Nota: evitiamo di usare il comando "sleep" interno del DFPlayer (0x0A) perché su molti cloni non si risveglia in modo affidabile; per risparmio batteria serio è meglio tagliare VCC.

### Bypass per test (senza MOSFET)
Per provare oggi:
- collega **DFPlayer VCC direttamente a +5V** e **GND a GND**
- lascia GPIO2 non connesso (o connesso ma senza circuito), il firmware funziona lo stesso

> Nota: non usiamo più il comando DFPlayer "sleep" (0x0A) durante i test perché su molti cloni non si risveglia in modo affidabile.
> Per il risparmio energetico vero, la strada solida è tagliare VCC con MOSFET high-side.

### Comandi seriale
- `mp3 1` (suona `/MP3/0001.mp3`)
- `mp3 1 5` (cap a 5s, solo come paracadute)
- `stop` (stop della riproduzione; **non** spegne il DFPlayer)
- `vol 20`
- `mp3 status`


### Legenda MP3 eventi (cartella /MP3)
Metti i file in **SD:/MP3/** con nome a 4 cifre (es. `0001.mp3`).

| File | Evento |
|---:|---|
| 0001.mp3 | Avvio bilancia |
| 0002.mp3 | Boot completato |
| 0003.mp3 | Entrata risparmio energetico (inattività: schermata Zzz... 5s prima del light-sleep) |
| 0004.mp3 | Uscita risparmio energetico (wake) |
| 0005.mp3 | Wi‑Fi **modulo ON** (toggle manuale) |
| 0006.mp3 | Wi‑Fi **modulo OFF** (toggle manuale) |
| 0007.mp3 | Errore connessione Wi‑Fi (connect failed / SSID non disponibile, con cooldown) |
| 0008.mp3 | Connessione Wi‑Fi avvenuta (anche in background) |
| 0011.mp3 | Batteria bassa (una sola volta all'ingresso in 0 tacche, beep resta attivo) |
| 0012.mp3 | Batteria critica (una sola volta all'ingresso fase critica, beep resta attivo) |
| 0013.mp3 | Standby pre‑sleep per batteria scarica (schermata Zzz... 5s prima del light-sleep) |
| 0014.mp3 | Errore lettura batteria (INA) |
| 0015.mp3 | Errore sensore peso (HX) |
| 0016.mp3 | Errore TARA al boot (Auto‑TARE fallita, richiede ACK) |
| 0017.mp3 | Modalità WORK |
| 0018.mp3 | Modalità LIVE |


### Debug seriale HX711 (opzionale)
Di default il log continuo del sensore è disattivato.

Nota: il parser comandi seriali è **non bloccante** (nessun timeout/attesa). I comandi vengono eseguiti solo quando invii una riga completa (terminata da invio). 
Comandi:
- `hxlog on` / `hxlog off`
- `hxlog ?`
- `hxlog rate <ms>` (50..5000)


### Tastiera: debounce + one-shot
La tastiera ha un debounce software (40 ms) e genera eventi **one-shot**: un tasto premuto produce **un solo evento**, anche se lo tieni premuto.

Se in futuro ti serve una logica di "long press" o autorepeat (freccia UP), va aggiunta esplicitamente.
