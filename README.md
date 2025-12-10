# Bilancia ESP32 + NAU7802 + OLED SSD1322 + INA219 · Prototipo 6 V (v2025-12-10)

Prototipo di **bilancia da banco** per laboratorio (gelato / pasticceria) basata su:

- ESP32 DevKit
- ADC 24 bit **NAU7802** per la cella di carico
- Display **OLED SSD1322** 256×64 (3,12", SPI)
- Alimentazione a **batteria piombo 6 V** con modulo CTK3S
- Monitor batteria con **INA219** (CJMCU-219, lato alto)

> Progetto pensato per uso interno in laboratorio, non per pesate legali a fini commerciali.

---

## 0. Scopo e stato del progetto

- Portata target: **20 kg**  
- Risoluzione visibile: **1 g** (clamp a ±16 kg per sicurezza)
- Uso tipico: pesate di ingredienti in laboratorio (gelateria Minù), con focus su stabilità e lettura “ferma”.
- Piattaforma: **ESP32 DevKit** + firmware in C++ stile Arduino.

Stato attuale:

- ✅ Lettura affidabile tramite NAU7802 con filtri (mediana + media mobile)
- ✅ Macchina di stati **STABLE / UNSTABLE / LIVE**
- ✅ **Zero-tracking** vicino a zero + **snap-to-zero** allo scarico
- ✅ Auto-tare opzionale all’avvio
- ✅ UI base su OLED (peso grande + indicatore di stato)
- ✅ Tastierino frontale 4×2: **TARE** e **MODE** già operativi
- ✅ Monitor batteria con INA219: 4 livelli (FULL / GOOD / LOW / CRITICAL) + flag *charging*
- ✅ Buzzer di sistema (suono di boot + suoni UI gestiti da modulo dedicato)

Cose previste / TODO:

- Menù di calibrazione e setup **solo da tastiera**
- Icona batteria su OLED (4 tacche + simbolo “fulmine” in carica)
- Migliore gestione errori (ADC, I2C, out-of-range, ecc.)
- Eventuale logging avanzato via seriale o Wi-Fi

---

## 1. Hardware

### 1.1 Moduli principali

- **ESP32 DevKit** (form factor tipo NodeMCU / JOY-IT)
- **Cella di carico 4 fili** (20–30 kg)
- **Breakout NAU7802** (ADC 24 bit, alimentato a 5 V, I2C a 3,3 V)
- **OLED 3,12" SSD1322 256×64**, interfaccia SPI 4-wire
- **INA219 CJMCU-219** per tensione/corrente batteria (alto lato)
- **Batteria piombo 6 V** (SLA/AGM)
- **Modulo CTK3S** per:
  - carica da alimentatore 12 V
  - protezione da sovrascarica tramite uscita LOAD (LVD integrato)
- **Buck step-down 5 V**, regolabile, per alimentare logica e sensori
- **Tastierino 4×2 (8 tasti)** pannello frontale
- **Buzzer passivo 2 pin** su GPIO ESP32

### 1.2 Architettura di alimentazione

Schema logico:

1. **Alimentatore 12 V** → ingresso **PV** del CTK3S.
2. Morsetti **BAT+ / BAT−** del CTK3S → direttamente alla batteria 6 V.
3. Uscita **LOAD+ / LOAD−** del CTK3S → ingresso del buck step-down 5 V.
4. Uscita **5 V** del buck → rail 5 V che alimenta:
   - VIN/5V dell’ESP32
   - VCC del NAU7802
   - VCC dell’OLED SSD1322
5. **INA219** montato dal lato batteria (alto lato):
   - BAT+ → fusibile → VIN+ INA219 → VIN− INA219 → B+ CTK3S
   - BAT− → B− CTK3S → GND comune.

Motivazione uso di LOAD e non BAT:

- L’LVD interno del CTK3S agisce solo sulla porta **LOAD**.
- Prelevando corrente da LOAD si evita la scarica profonda della batteria:
  - sotto soglia la porta LOAD viene scollegata
  - viene riattivata solo dopo risalita sopra una tensione di hysteresis.

Condensatori consigliati:

- **Ingresso buck (da LOAD CTK3S)**
  - 220 µF elettrolitico (≥ 16 V)
  - 100 nF ceramico
- **Uscita buck (rail 5 V)**
  - 470 µF elettrolitico (≥ 10–16 V)
  - 100 nF ceramico
- **Vicino all’ESP32 (VIN/5V ↔ GND)**
  - 10–47 µF elettrolitico
  - 100 nF ceramico

Fusibili tipici:

- 2 A sul ramo **12 V → PV CTK3S**
- 1–2 A fra **BAT+ → VIN+ INA219**
- 2 A fra **LOAD+ → ingresso buck 5 V**
- opzionale 1 A sul **5 V** dopo il buck per protezione locale

### 1.3 Pinout ESP32 (riassunto)

| Funzione                 | Modulo          | GPIO ESP32 | Note                                          |
|--------------------------|-----------------|-----------:|----------------------------------------------|
| I2C SDA                  | NAU7802 + INA   |       32   | bus I2C comune per ADC e monitor batteria    |
| I2C SCL                  | NAU7802 + INA   |       33   |                                              |
| SPI SCK                  | OLED SSD1322    |       18   | bus VSPI                                     |
| SPI MOSI                 | OLED SSD1322    |       23   |                                              |
| SPI CS                   | OLED SSD1322    |       25   |                                              |
| SPI DC                   | OLED SSD1322    |       26   |                                              |
| SPI RST                  | OLED SSD1322    |       27   |                                              |
| Tastiera R1              | Keypad 4×2      |        4   | fila 1                                       |
| Tastiera R2              | Keypad 4×2      |        5   | fila 2                                       |
| Tastiera R3              | Keypad 4×2      |       13   | fila 3                                       |
| Tastiera R4              | Keypad 4×2      |       14   | fila 4                                       |
| Tastiera C1              | Keypad 4×2      |       19   | colonna 1                                    |
| Tastiera C2              | Keypad 4×2      |       21   | colonna 2                                    |
| Buzzer                   | Buzzer passivo  |       22   | pilotato con LEDC                            |
| 5 V (VIN)                | Rail 5 V buck   |      VIN   | alimentazione logica                         |
| 3V3                      | Rail 3V3 ESP32  |      3V3   | alimentazione INA219                         |
| GND                      | Massa comune    |      GND   | in comune con CTK3S, buck, NAU, OLED, INA    |

> Verificare sempre la serigrafia reale del DevKit (D18, D19, ecc.) per evitare confusioni fra “Dx” e GPIO.

### 1.4 Cablaggio dettagliato

#### Cella di carico ↔ NAU7802

Corrispondenza tipica (controllare colori reali del cavo):

- E+ / EXC+ → alimentazione ponte + (di solito **rosso**)
- E− / EXC− → alimentazione ponte − (di solito **nero**)
- A+ / SIG+ → segnale + (tipicamente **verde**)
- A− / SIG− → segnale − (tipicamente **bianco**)

Consigli:

- Usare cavo twistato e schermato.
- Collegare la **calza di schermatura a GND lato scheda**.

#### NAU7802 ↔ ESP32 (I2C)

- SDA → GPIO32
- SCL → GPIO33
- VCC → 5 V (dal buck)
- GND → GND comune

Nel codice:

```cpp
const int I2C_SDA = 32;
const int I2C_SCL = 33;

Wire.begin(I2C_SDA, I2C_SCL);
Wire.setClock(400000);
```

#### OLED SSD1322 ↔ ESP32 (SPI)

Bus VSPI:

- SCK → GPIO18
- MOSI → GPIO23
- (MISO non usato)

Segnali di controllo:

- CS  → GPIO25
- DC  → GPIO26
- RST → GPIO27

Alimentazione display:

- VCC → 5 V (stessa rail dell’ESP32)
- GND → GND comune

Inizializzazione tipica (U8g2):

```cpp
#include <U8g2lib.h>

static const int OLED_CS  = 25;
static const int OLED_DC  = 26;
static const int OLED_RST = 27;

U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI oled(
  U8G2_R0,
  OLED_CS,
  OLED_DC,
  OLED_RST
);

// SPI
SPI.begin(18, -1, 23, OLED_CS);
```

#### INA219 ↔ batteria + ESP32

Lato potenza:

- BAT+ → fusibile → VIN+ INA219
- VIN− INA219 → B+ CTK3S
- BAT− → B− CTK3S → GND comune

Lato logica:

- VCC → 3V3 ESP32
- GND → GND comune
- SDA → GPIO32 (I2C)
- SCL → GPIO33 (I2C)

Indirizzo I2C di default: `0x40`.

#### Tastierino 4×2

Tasti fisici: `TARE`, `ENTER`, `ZERO`, `UP`, `UNIT`, `SET`, `CALI`, `MODE`.

Collegamento fili → GPIO:

- Filo 1 (R1) → GPIO17 (tx2)
- Filo 2 (R2) → GPIO5
- Filo 3 (R3) → GPIO13
- Filo 4 (R4) → GPIO14
- Filo 5 (C1) → GPIO19
- Filo 6 (C2) → GPIO21

Mappa tasti (R = riga, C = colonna):

- R1-C1 → TARE
- R1-C2 → ENTER
- R2-C1 → ZERO
- R2-C2 → UP
- R3-C1 → UNIT
- R3-C2 → SET
- R4-C1 → CALI
- R4-C2 → MODE

Comportamento **attuale**:

- `TARE`: esegue una tara completa, come il comando seriale `t`.
- `MODE`: alterna fra:
  - modalità WORK / normal (`setMode("work")`)
  - modalità FINE / live (`setMode("live")`)
- gli altri tasti sono letti ma non assegnati: pronti per menù e calibrazione.

#### Buzzer

- `+` buzzer → GPIO22
- `−` buzzer → GND comune

Pilotaggio:

- buzzer passivo pilotato a 3,3 V tramite GPIO e periferica LEDC
- nessun componente in serie; eventuali condensatori sono solo sulle rail 3V3/5V.

È normale un leggero fruscio di fondo dovuto al rumore di alimentazione; soluzioni con R+C direttamente sul buzzer lo riducono ma attenuano troppo il volume. Un miglioramento futuro valuta driver a transistor o buzzer attivo.

---

## 2. Firmware · concetti chiave

### 2.1 Counts, offset e scala

- Il NAU7802 fornisce un valore grezzo `raw` proporzionale alla forza sulla cella.
- Dopo una TARE, il valore medio a vuoto viene salvato come `OFFSET_RAW`.
- La scala in **counts per grammo** è `SCALE_CPG`:

```txt
SCALE_CPG = (raw_con_peso - OFFSET_RAW) / peso_grammi
```

- La conversione in grammi (dopo i filtri) usa:

```txt
gLive = (rawAvg - (OFFSET_RAW + zero_track_counts)) / SCALE_CPG
```

### 2.2 Stati STABLE / UNSTABLE / LIVE

Modalità con **ST attivo**:

- `STABLE`:
  - il peso è “fermo”
  - il display mostra un valore agganciato `gLatch`
- `UNSTABLE`:
  - il carico si sta muovendo
  - il display segue `gLive` con una deadband per non tremare

Transizioni principali:

- UNSTABLE → STABLE quando il range in una finestra (~1,2 s) scende sotto `ST_ENTER_RANGE_G`
- STABLE → UNSTABLE quando `|gLive − gLatch| ≥ ST_LEAVE_DELTA_G`

Modalità con **ST disattivato**:

- stato unico `LIVE`: il display segue la misura filtrata senza latch.

### 2.3 Zero-tracking (ZT)

- Attivo solo vicino a zero, in una finestra `±ZT_WINDOW_G` (tipicamente 1–1,5 g).
- Controlla che la bilancia sia “silenziosa”:
  - range dei campioni basso
  - pendenza (slope) molto piccola
- Ogni `ZT_PERIOD_MS` corregge lentamente `zero_track_counts` per riportare la lettura verso 0 (passi tipici ~0,02 g).
- La correzione totale è limitata a `±ZT_MAX_G` (tipicamente ~3 g) per non spostare la calibrazione.

### 2.4 Snap-to-zero allo scarico

- Rileva che stai **scaricando rapidamente** la bilancia:
  - pendenza negativa sotto una soglia (`UNLOAD_SLOPE_GPS_NEG`)
  - |g| entro una finestra attorno a zero (`UNLOAD_CROSS_WIN_G`)
- In questo caso forza uno “snap” veloce a zero, regolando `zero_track_counts` fino ad un massimo `UNLOAD_SNAP_MAX_G`.

### 2.5 Deadband display

- In `UNSTABLE` / `LIVE` il numero a display viene aggiornato solo se la differenza supera `deadbandUnstable`.
- Tipici:
  - 0,10–0,20 g in modalità normale
  - 0,05 g in modalità fine/micro.

---

## 3. Monitor batteria (INA219)

Il monitor batteria vive in due file dedicati:

- `battery_monitor.h`
- `battery_monitor.cpp`

Strutture principali:

```cpp
enum BatteryLevel {
  BATT_LEVEL_FULL = 0,
  BATT_LEVEL_GOOD,
  BATT_LEVEL_LOW,
  BATT_LEVEL_CRITICAL
};

struct BatteryStatus {
  float        voltage_V;   // V batteria filtrati
  float        current_mA;  // mA filtrati: >0 = scarica, <0 = carica
  BatteryLevel level;       // 4 livelli batteria
  bool         charging;    // true = in carica
};
```

API:

```cpp
void          battery_init();                  // da chiamare in setup()
void          battery_update(uint32_t nowMs);  // da chiamare nel loop
BatteryStatus battery_get_status();            // ultimo stato calcolato
void          battery_debug_print(const BatteryStatus &st); // stampa su Serial
```

Integrazione tipica:

```cpp
#include "battery_monitor.h"

void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  // init NAU, OLED, ecc...

  battery_init();
}

void loop() {
  uint32_t now = millis();

  battery_update(now);

  // resto logica bilancia...
}
```

### 3.1 Livelli batteria (4 step)

Le soglie usano la tensione della batteria filtrata:

- `FULL`     se V ≥ 6,40 V
- `GOOD`     se 6,20 V ≤ V < 6,40 V
- `LOW`      se 6,00 V ≤ V < 6,20 V
- `CRITICAL` se V < 6,00 V

Sono soglie conservative per una 6 V piombo, per evitare scariche profonde.

### 3.2 Rilevamento carica / scarica

Si usa il segno della corrente misurata dall’INA219:

- **Scarica**: la bilancia consuma dalla batteria → `current_mA > 0`
- **Carica**: il CTK3S spinge corrente nella batteria → `current_mA < 0`

Logica con isteresi:

- `charging = true` se `current_mA < -30 mA`
- `charging = false` quando la corrente risale sopra circa `-15 mA`

In debug tipicamente si vedono righe del tipo:

- Scarica: `[BATT] V=6.33 V I=76 mA lvl=GOOD charging=NO`
- Carica:  `[BATT] V=6.84 V I=-187 mA lvl=FULL charging=YES`

---

## 4. UI su OLED

UI attuale (minimal):

- peso in grande al centro (solo numero, senza unità)
- indicatore di stato in alto a sinistra:
  - `STABLE`
  - `UNSTABLE`
  - `LIVE` (se ST disattivato)

Aggiornamento:

- refresh ogni `OLED_UPDATE_MS` (tipico ~120 ms)
- valore mostrato: `gDisp` (filtrato + deadband + eventuale latch STABLE).

Integrazione prevista con monitor batteria:

- icona a 4 tacche in base a `BatteryStatus.level`
- simbolo “fulmine” in caso di `BatteryStatus.charging == true`.

---

## 5. Avvio e calibrazione

Procedura consigliata:

1. **Avvio a vuoto**
   - piattaforma completamente scarica
   - accendi; se `AUTO_TARE_ON_BOOT = true` viene eseguita una tara iniziale dopo `AUTO_TARE_SETTLE_MS`.

2. **Verifica zero**
   - tramite Serial Monitor, controlla che `gDisp` sia vicino a 0 g
   - se non lo è, invia `t` per una TARE manuale.

3. **Calibrazione con peso noto**
   - con bilancia a zero, metti un peso noto (es. 2000 g)
   - su Serial Monitor (115200 baud) invia:

     ```txt
     c 2000
     ```

   - lo sketch esegue una serie di letture e ricalcola `SCALE_CPG`.

4. **Salvataggio in NVS**
   - quando OFFSET e SCALE sono stabili, invia:

     ```txt
     s
     ```

   - i parametri di calibrazione vengono salvati in NVS.

5. **Test**
   - togli e rimetti il peso:
     - a vuoto: ~0 g
     - con peso: valore vicino al nominale (es. ~2000 g)

---

## 6. Comandi seriali principali

Baud rate tipico: **115200 8N1**.

- `t`  
  Esegue la **TARE**: salva il raw medio a vuoto come `OFFSET_RAW` e azzera `zero_track_counts`.

- `c <g>`  
  Calibrazione con peso noto.  
  Esempio: `c 2000` per un peso da 2000 g.  
  Procedura: TARE → posiziona il peso → `c 2000`.

- `p`  
  Stampa stato e parametri principali:
  - `OFFSET_RAW`, `SCALE_CPG`, `REF_G`
  - `MA`, `DB_UNSTABLE`, stato ST on/off, ZT on/off
  - `zero_track_counts` in counts e grammi.

- `s`  
  Salva in NVS `offset`, `scale`, `ref_g`.

- `m work` / `m normal`  
  Preset “normale”:
  - `MA = 6`, `deadband ≈ 0,10 g`
  - `ST = on`, `ZT = on`.

- `m fine` / `m live`  
  Preset “fine/micro”:
  - `MA = 4`, `deadband ≈ 0,05 g`
  - `ST = off` (stato LIVE), `ZT = on`.

- `st on` / `st off` / `st ?`  
  Abilita/disabilita la macchina STABLE/UNSTABLE e ne mostra lo stato.

- `zt on` / `zt off` / `zt reset` / `zt ?`  
  Gestione zero-tracking + snap allo scarico:
  - `zt reset` azzera `zero_track_counts`
  - `zt ?` mostra stato e valore di ZT (counts e grammi).

In futuro può essere aggiunto un comando dedicato al debug batteria; per ora si usa `battery_debug_print()` richiamato dal loop.

---

## 7. Tuning pratico

Linee guida di massima:

- **Micro dosi / ingredienti piccoli**
  - preset `m fine`
  - deadband 0,05 g
  - `ST off` se vuoi una risposta più “live”.

- **Lavoro normale in laboratorio**
  - preset `m work` / `m normal`
  - `ST on`, `ZT on`
  - deadband fra 0,10 e 0,20 g.

- **Monitor batteria**
  - `LOW`: inizia a programmare una ricarica
  - `CRITICAL`: finisci la pesata e ricarica
  - flag `charging` per capire se stai lavorando in carica o solo a batteria.

---

## 8. Buzzer di sistema

### 8.1 Hardware

- Buzzer passivo 2 pin, marcato “SPEAKER”.
- Collegato direttamente al GPIO22 dell’ESP32.
- Pilotato tramite periferica LEDC (PWM) a 3,3 V.

### 8.2 Firmware

Modulo dedicato:

- `buzzer.h`
- `buzzer.cpp`

Elementi principali:

- definizione di `BUZZER_PIN`
- uso delle API LEDC:
  - `ledcAttach(pin, freq, resolutionBits)`
  - `ledcWrite(pin, duty)`
  - `ledcWriteTone(pin, freq)`
  - `ledcDetach(pin)`
- sequenze di note modellate con struttura:

  ```cpp
  struct Note {
    uint16_t freqHz; // 0 = pausa
    uint16_t durMs;  // durata in millisecondi
  };
  ```

Pattern tipici:

- suono di boot
- click conferma
- errore / avviso.

---

## 9. Build del firmware

> Nota: questa sezione è generica; verificare sempre i nomi delle cartelle e degli sketch nel repository.

1. Apri la cartella `firmware` nel tuo IDE (Arduino IDE oppure PlatformIO).
2. Se usi Arduino IDE:
   - seleziona scheda **ESP32 Dev Module** (pacchetto Espressif per Arduino).
   - imposta la porta seriale corretta.
3. Installa le librerie richieste (controlla gli `#include` nello sketch principale), tipicamente:
   - `U8g2` per il display SSD1322
   - libreria per `INA219` (es. Adafruit INA219)
   - libreria per `NAU7802` (es. SparkFun NAU7802)  
   - eventuale libreria per il tastierino (se non gestito “a mano”).
4. Compila e carica il firmware.
5. Apri Serial Monitor a **115200 baud** per seguire:
   - messaggi di boot
   - debug calibrazione
   - logging del monitor batteria (se abilitato).

---

## 10. Note finali

- Questo README descrive la **configurazione di riferimento** del prototipo 6 V con CTK3S e INA219.
- Se cambi DevKit, cella di carico o moduli, aggiorna **pinout** e **parametri di calibrazione** di conseguenza.
- La parte elettrica (fusibili, dimensionamento cavi, isolamento meccanico della cella, ecc.) va sempre verificata rispetto all’hardware reale.

Per qualsiasi modifica importante della struttura (nuovi moduli, nuove rail, seconda cella di carico, ecc.) conviene aggiornare questo README e aggiungere schemi nella cartella `docs/`.
