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
- Protezione “batteria scarica” con **avviso + beep + light-sleep**

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
- **LIVE**: più reattivo a display, con anti-flicker

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
Soglie “pratiche” (dipendono da carico/temperatura):
- **4 tacche (FULL)**: ≥ **6.35 V**
- **3 tacche (GOOD)**: ≥ **6.20 V**
- **2 tacche (LOW)**:  ≥ **6.05 V**
- **1 tacca (CRITICAL)**: ≥ **5.90 V**
- **0 tacche (EMPTY)**: < **5.90 V**

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
- **0 tacche (EMPTY)**: beep di avviso ogni **60 s**, UI normale
- Se V scende sotto **5,80 V** per ≥ **5 s**:
  - schermo “batteria scarica, collega alimentatore”
  - beep ogni **10 s** per **60 s**
  - poi entra in **LIGHT-SLEEP**
- Prima del light-sleep per batteria scarica: DFPlayer viene **spento** via MOSFET (zero consumo audio in sleep).
- Wake: premendo un tasto (la tastiera risveglia, poi l’ESP32 riparte con reboot pulito)

---


---

## 10) Risparmio energetico per inattività (5 minuti)
Se per **5 minuti** non viene premuto alcun tasto, la bilancia entra in **LIGHT-SLEEP**.

Caratteristiche:
- Wake: **qualsiasi tasto**.
- **Nessun reset** dello stato/pesata: riprende esattamente dove era.
- WiFi/OTA vengono sospesi prima dello sleep e riattivati dopo il wake.

### Indicatore esterno sleep (LED)
Quando il display è spento, per capire che la bilancia è in sleep, usa un LED su:
- **GPIO15** → **resistenza 330–2.2kΩ** → **anodo LED** → catodo a **GND** (HIGH = LED acceso).

## 11) WiFi + OTA (opzionale)

Il firmware HX711 supporta:
- WiFi **non bloccante** (background)
- OTA via Arduino IDE (porta di rete)

Dove si configura:
- `firmware/esp32_hx711_serial/net_ota_cloud.h`

Note pratiche:
- di default è attivo **ENABLE_WIFI_OTA = 1**
- devi sostituire SSID/PASS con quelli reali
- l’icona WiFi sul display segue lo stato di connessione (se WiFi è abilitato a compile-time)

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

### DFPlayer Mini (audio eventi) + power-gating solo in standby
Il firmware può suonare file MP3 (es. avviso sleep). Per evitare click e stati strani, **non fa power-cycle a fine brano**.

Comportamento attuale:
- DFPlayer viene portato in stato **pronto** automaticamente **all’avvio** e a ogni **wake** (alimentazione ON + UART init + volume), così un suono può partire subito.
- La riproduzione MP3 è progettata per essere **non bloccante**: mentre l’audio suona, la bilancia continua a leggere HX711 e ad aggiornare UI/log.
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
| 0005.mp3 | Wi‑Fi connesso |
| 0006.mp3 | Wi‑Fi disconnesso |
| 0007.mp3 | Errore connessione Wi‑Fi (connect failed / SSID non disponibile, con cooldown) |
| 0011.mp3 | Batteria bassa (una sola volta all'ingresso in 0 tacche, beep resta attivo) |
| 0012.mp3 | Batteria critica (una sola volta all'ingresso fase critica, beep resta attivo) |
| 0013.mp3 | Standby pre‑sleep per batteria scarica (schermata Zzz... 5s prima del light-sleep) |
| 0014.mp3 | Errore lettura batteria (INA) |
| 0015.mp3 | Errore sensore peso (HX) |
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
