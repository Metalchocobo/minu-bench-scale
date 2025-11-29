# Bilancia ESP32 + NAU7802 + OLED SSD1322 + INA219 — Prototipo 6 V (v2025-11-27)

Prototipo di bilancia da banco con:

- ESP32 DevKit
- NAU7802 (ADC 24 bit) per la cella di carico
- OLED 3.12″ SSD1322, 256×64 per la UI (SPI)
- Alimentazione da batteria piombo 6 V con CTK3S + buck 5 V
- **INA219 (CJMCU-219) per monitor batteria (tensione, corrente, livello, stato di carica)**

Firmware “light” con:

- Stati STABLE / UNSTABLE / LIVE
- Filtro mediana + media mobile
- Zero-tracking vicino a 0 (micro correzione lenta)
- Snap-to-zero allo scarico
- UI base su OLED (peso in grande + stato, pronto per icona batteria)
- Comandi via Serial Monitor (calibrazione, tare, preset filtri)

---

## 1. Architettura elettrica attuale (batteria 6 V + CTK3S + buck 5 V + INA219)

Schema logico:

- Batteria piombo 6 V (SLA/AGM)
- CTK3S:
  - ingresso PV da alimentatore 12 V (caricabatterie)
  - morsetti BAT+ / BAT− collegati direttamente alla batteria
  - uscita LOAD+ / LOAD− verso l’impianto
- Buck step-down 5 V:
  - ingresso dal LOAD del CTK3S
  - uscita regolata a 5,05–5,10 V verso ESP32 + NAU + OLED
- **INA219 ad alta side sul ramo BAT+ → CTK3S B+**:
  - BAT+ → fusibile → VIN+ INA219 → VIN− INA219 → B+ CTK3S
  - misura tensione batteria e corrente che entra/esce dalla batteria

Perché usare LOAD e non BAT:

- L’LVD del CTK3S lavora solo sulla porta LOAD
- Se si preleva da BAT, l’LVD non interviene e rischi scarica profonda della batteria
- Così invece:
  - quando la batteria scende sotto soglia (~5,3–5,4 V), il CTK3S stacca LOAD
  - riattacca solo dopo risalita (~5,6–5,9 V, da verificare sul tuo esemplare)

Condensatori e disposizione fisica:

- Ingresso buck (da LOAD CTK3S)
  - 220 µF elettrolitico (≥16 V)
  - 100 nF ceramico
  - in parallelo, montati vicino ai morsetti IN del buck
- Uscita buck (rail 5 V)
  - 470 µF elettrolitico (≥10–16 V)
  - 100 nF ceramico
  - in parallelo, montati vicino ai morsetti OUT
- Vicino all’ESP32 (VIN/5V ↔ GND)
  - 10–47 µF elettrolitico
  - 100 nF ceramico
  - il più vicino possibile ai pin VIN/5V e GND (decoupling locale per i picchi Wi-Fi)

Fusibili consigliati:

- Tra alimentatore 12 V ↔ CTK3S (PV+): 2 A rapido/medio
- Tra BAT+ ↔ VIN+ INA219 (verso CTK3S): 1–2 A rapido
- Tra LOAD+ ↔ buck 5 V: 2 A rapido
- Sul 5 V dopo il buck (opzionale): 1 A rapido per protezione locale

Note operative:

- Regola il buck a 5,05–5,10 V sotto carico leggero
- In carica la batteria può stare a 7,2–7,5 V: è normale
- Nessun LVD esterno necessario finché il buck resta collegato a LOAD

---

## 2. Concetti chiave firmware (NAU7802 + stati + ZT)

- **Counts (raw)**  
  Il NAU7802 fornisce un valore grezzo (`raw`) proporzionale alla forza sulla cella.

- **OFFSET_RAW**  
  Valore di `rawAvg` a vuoto dopo una TARE. È la baseline in counts.

- **SCALE_CPG (counts per grammo)**  
  Si ricava con un peso noto:  
  `SCALE_CPG = (raw_con_peso - OFFSET_RAW) / peso_grammi`

- **Conversione in grammi**  
  Dopo i filtri, usiamo:  
  `gLive = (rawAvg - (OFFSET_RAW + zero_track_counts)) / SCALE_CPG`

- **Stati STABLE / UNSTABLE / LIVE**
  - In modalità ST on:
    - STABLE: il peso è fermo e il display mostra un valore “agganciato” (`gLatch`)
    - UNSTABLE: stai muovendo il carico; il display segue `gLive` con una deadband (per non tremare)
    - passaggio UNSTABLE→STABLE quando il range in una finestra (~1,2 s) scende sotto `ST_ENTER_RANGE_G`
    - uscita da STABLE quando \|gLive − gLatch\| ≥ `ST_LEAVE_DELTA_G`
  - In modalità ST off: stato “LIVE”, il display segue la misura filtrata senza latch.

- **Zero-tracking (ZT) vicino a 0**
  - Attivo solo se \|g\| ≤ `ZT_WINDOW_G` (tipicamente 1–1,5 g)
  - Il sistema controlla che la bilancia sia “silenziosa” (range basso + slope bassa)
  - Ogni `ZT_PERIOD_MS` sposta lentamente `zero_track_counts` verso azzerare la lettura (passi di ~0,02 g)
  - Il totale della correzione è limitato (±`ZT_MAX_G`, tipicamente 3 g) per non far “scappare” la calibrazione.

- **Snap-to-zero allo scarico**
  - Se stai scaricando velocemente (pendenza ≤ `UNLOAD_SLOPE_GPS_NEG` e \|g\| ≤ `UNLOAD_CROSS_WIN_G`),
    il firmware fa uno “snap” rapido a zero regolando `zero_track_counts` fino a un massimo di `UNLOAD_SNAP_MAX_G`.

- **Deadband display**
  - In UNSTABLE/LIVE si aggiorna il numero solo se la differenza supera `deadbandUnstable`
  - Tipicamente 0,10–0,20 g in fine/normal per ridurre il tremolio visivo.

---

## 3. Hardware e cablaggio

### Moduli principali

- ESP32 DevKit (tipo NodeMCU ESP32 / JOY-IT)
- NAU7802 breakout (alimentato a 5 V, I2C a 3,3 V)
- Cella di carico 4 fili (20–30 kg, secondo modello)
- OLED 3,12" SSD1322 256×64 (WAVGAT / compatibile NHD, SPI 4-wire)
- **CJMCU-219 / INA219 per monitor batteria (I2C)**

### Collegamento cella di carico ↔ NAU7802

Dipende dal breakout, ma in generale:

- E+ / EXC+ → filo rosso (alimentazione ponte +)
- E− / EXC− → filo nero (alimentazione ponte −)
- A+ / SIG+ → filo verde (segnale +)
- A− / SIG− → filo bianco (segnale −)

Cavo cella:

- meglio se twistato e schermato, con la calza collegata a GND lato scheda.

### Collegamento NAU7802 ↔ ESP32 (I2C)

Mappatura attuale “definitiva” (cluster da 5 pin):

- SDA → D32 (GPIO32)
- SCL → D33 (GPIO33)
- VCC → 5 V (dal buck)
- GND → GND comune

In codice:

```cpp
const int I2C_SDA = 32;  // D32
const int I2C_SCL = 33;  // D33

Wire.begin(I2C_SDA, I2C_SCL);
Wire.setClock(400000);
```

### Collegamento OLED SSD1322 ↔ ESP32 (SPI)

Bus SPI hardware (VSPI):

- SCK → D18 (GPIO18)
- MOSI → D23 (GPIO23)
- MISO non usato

Segnali di controllo (cluster da 3 pin contigui):

- CS → D25 (GPIO25)
- DC → D26 (GPIO26)
- RST → D27 (GPIO27)

Alimentazione:

- VCC → 5 V (dal buck, stessa rail dell’ESP32)
- GND → GND comune

In codice (U8g2):

```cpp
#include <U8g2lib.h>
static const int OLED_CS  = 25;  // D25
static const int OLED_DC  = 26;  // D26
static const int OLED_RST = 27;  // D27

U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI oled(
  U8G2_R0,
  OLED_CS,
  OLED_DC,
  OLED_RST
);

// SPI
SPI.begin(18, -1, 23, OLED_CS); // SCK=18, MISO unused, MOSI=23, SS=OLED_CS
```

### Collegamento INA219 ↔ batteria + ESP32

**Lato potenza (alto lato batteria 6 V):**

- BAT+ → fusibile 1–2 A → VIN+ INA219
- VIN− INA219 → B+ CTK3S
- BAT− → B− CTK3S (massa comune)

**Lato logica (I2C + alimentazione):**

- VCC → 3V3 ESP32
- GND → GND comune
- SDA → D32 (GPIO32, stesso bus NAU)
- SCL → D33 (GPIO33, stesso bus NAU)

Indirizzo I2C:

- default 0x40 (CJMCU-219 standard).

### Tastierino frontale 4x2 (8 tasti)
  Fisicamente: TARE, ENTER, ZERO, UP, UNIT, SET, CALI, MODE

  Collegamento fili -> GPIO:

  - Filo 1 (R1) = GPIO4
  - Filo 2 (R2) = GPIO5
  - Filo 3 (R3) = GPIO13
  - Filo 4 (R4) = GPIO14
  - Filo 5 (C1) = GPIO19
  - Filo 6 (C2) = GPIO21

  Mappa tasti (R = riga, C = colonna):

    R1-C1 (filo1+filo5) -> TARE
    R1-C2 (filo1+filo6) -> ENTER
    R2-C1 (filo2+filo5) -> ZERO
    R2-C2 (filo2+filo6) -> UP
    R3-C1 (filo3+filo5) -> UNIT
    R3-C2 (filo3+filo6) -> SET
    R4-C1 (filo4+filo5) -> CALI
    R4-C2 (filo4+filo6) -> MODE

Comportamento attuale dei tasti:
- TARE:
    Esegue la tara, usando la stessa logica del comando seriale "t".
- MODE:
    Alterna tra:
      * modalità WORK / normal (setMode("work"))
      * modalità FINE / live (setMode("live"))
    mantenendo allineati MA, deadband e stato ST (ST on in WORK, ST off in LIVE).

- ENTER, ZERO, UP, UNIT, SET, CALI:
    Già mappati e letti dal firmware, ma al momento non eseguono azioni:
    sono lasciati liberi per menu, calibrazioni o altre funzioni future.


---

## 4. Monitor batteria (INA219)

Il monitor batteria è implementato in due file dedicati:

- `firmware/.../battery_monitor.h`
- `firmware/.../battery_monitor.cpp`

Si appoggia alla libreria **Adafruit INA219**.

### API e struttura dati

```cpp
enum BatteryLevel {
  BATT_LEVEL_FULL = 0,
  BATT_LEVEL_GOOD,
  BATT_LEVEL_LOW,
  BATT_LEVEL_CRITICAL
};

struct BatteryStatus {
  float voltage_V;     // Tensione batteria filtrata (V)
  float current_mA;    // Corrente filtrata (mA): >0 = scarica, <0 = carica
  BatteryLevel level;  // 4 livelli batteria
  bool charging;       // true = batteria in carica
};

// Inizializzazione (dopo Wire.begin)
void battery_init();

// Da chiamare nel loop principale con now = millis()
void battery_update(uint32_t nowMs);

// Restituisce l'ultimo stato calcolato
BatteryStatus battery_get_status();

// Stampa su Serial una riga di debug leggibile
void battery_debug_print(const BatteryStatus &st);
```

### Integrazione nel firmware

Nel file principale:

```cpp
#include "battery_monitor.h"

void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  // init NAU, OLED, ecc...

  battery_init();  // inizializza INA219 (addr 0x40, calib 32V/2A)
}

void loop() {
  uint32_t now = millis();

  battery_update(now);

  // ... resto della logica bilancia ...
}
```

Debug opzionale ogni X secondi:

```cpp
static uint32_t lastBattDebug = 0;

void loop() {
  uint32_t now = millis();

  battery_update(now);

  if (now - lastBattDebug > 3000) {
    lastBattDebug = now;
    BatteryStatus st = battery_get_status();
    battery_debug_print(st);
  }

  // ...
}
```

### Logica di livello batteria (4 step)

Soglie basate sulla **tensione filtrata** della batteria:

- `BATT_LEVEL_FULL`    se `V ≥ 6.40 V`
- `BATT_LEVEL_GOOD`    se `6.20 V ≤ V < 6.40 V`
- `BATT_LEVEL_LOW`     se `6.00 V ≤ V < 6.20 V`
- `BATT_LEVEL_CRITICAL` se `V < 6.00 V`

Queste soglie sono conservative per una 6 V piombo: non spremono la batteria in scarica profonda.

### Rilevamento stato di carica

Il segno della corrente è dato dall’INA219:

- **Scarica**: la bilancia consuma dalla batteria → `current_mA > 0`
- **Carica**: il CTK3S spinge corrente nella batteria → `current_mA < 0`

La logica in `battery_monitor.cpp` usa una soglia:

- `charging = true` se `current_mA < -30 mA` (corrente verso la batteria, sopra rumore)
- `charging = false` quando la corrente risale sopra ~`-15 mA` (isteresi per evitare lampeggi)

In debug vedrai, ad esempio:

- in scarica:
  - `[BATT] V=6.335 V  I=76 mA  lvl=GOOD  charging=NO`
- in carica:
  - `[BATT] V=6.845 V  I=-187 mA  lvl=FULL  charging=YES`

---

## 5. Parametri principali (nel codice)

I parametri di tuning sono tutti all’inizio dello sketch principale e nei file dedicati:

- **Default calibrazione NAU**
  - `DEFAULT_REF_RAW`, `DEFAULT_ZERO_RAW`, `DEFAULT_REF_G`, `DEFAULT_CPG`
  - solo valori seed: dopo CAL + SAVE userai quelli salvati in NVS.

- **Auto-TARE all’avvio**
  - `AUTO_TARE_ON_BOOT`: `true` per fare una tara semplice al boot.
  - `AUTO_TARE_SAMPLES`: quanti campioni medi per la tara iniziale.
  - `AUTO_TARE_SETTLE_MS`: attesa prima di iniziare la tara.

- **Limite visibile**
  - `MAX_DISPLAY_G = 16000.0f` → clamp a ±16 kg anche se la cella è da 30 kg.

- **Filtri display**
  - `MA_DEFAULT` / `MA_FINE` (normal vs fine/live)
  - `DB_UNSTABLE_N`, `DB_UNSTABLE_F` (deadband in g).

- **Stati STABLE/UNSTABLE**
  - `ST_LEAVE_DELTA_G`: soglia per uscire da STABLE
  - `ST_ENTER_RANGE_G`: range massimo per rientrare STABLE
  - `ST_TO_STABLE_MS`: finestra temporale per valutare il rientro.

- **Zero-tracking (ZT)**
  - `ZT_WINDOW_G`, `ZT_QUIET_MS`, `ZT_QUIET_RANGE_G`, `ZT_QUIET_SLOPE_GPS`
  - `ZT_PERIOD_MS`, `ZT_STEP_G`, `ZT_MAX_G`

- **Snap allo scarico**
  - parametri `UNLOAD_*` (range, pendenza, cooldown, snap massimo)

---

## 6. UI attuale su OLED

UI minimal, pensata per pesate rapide:

- Peso in grande al centro (intero, senza unità)
- Piccola scritta in alto a sinistra con lo stato:
  - `STABLE`
  - `UNSTABLE`
  - `LIVE` (se ST disattivato)

Aggiornamento:

- OLED aggiornato ogni ~`OLED_UPDATE_MS` (tipicamente 120 ms)
- Il valore mostrato è `gDisp` (filtrato + deadband + eventuale latch STABLE)

Integrazione prevista con il monitor batteria:

- icona batteria a 4 tacche basata su `BatteryStatus.level`
- simbolo “fulmine” se `BatteryStatus.charging == true`

---

## 7. Procedura di avvio e calibrazione

1. **Avvio a vuoto**

   - Bilancia scarica, piattaforma senza peso
   - Accendi: se `AUTO_TARE_ON_BOOT=true` fa una tara semplice al boot dopo `AUTO_TARE_SETTLE_MS`.

2. **Verifica a zero**

   - Controlla su Serial Monitor che `gDisp` sia vicino a 0 g
   - Se non lo è, puoi fare una `TARE` manuale (`t`).

3. **Calibrazione con peso noto (es. 2000 g)**

   - Bilancia a zero, `t` se necessario
   - Metti il peso noto (es. 2 kg) sulla piattaforma
   - Su Serial Monitor (115200 baud) invia:

     ```text
     c 2000
     ```

   - Lo sketch fa 15 letture medie e ricalcola `SCALE_CPG`
   - Il log ti mostra qualcosa tipo:

     ```text
     [CAL] Riferimento: 2000 g (fai TARE, metti il peso noto, poi c)
     [CAL] SCALE_CPG=xxx.xxxxxx
     ```

4. **Salvataggio in NVS**

   - Quando sei soddisfatto di OFFSET e SCALE, esegui:

     ```text
     s
     ```

   - Lo sketch salva OFFSET/SCALE/REF in NVS.

5. **Test**

   - Togli e rimetti il peso:
     - a vuoto: ~0 g
     - con il peso: ~valore nominale (es. ~2000 g)

---

## 8. Comandi seriali supportati (versione light)

- `t`  
  Tare: acquisisce il raw a vuoto, aggiorna `OFFSET_RAW` e azzera `zero_track_counts`.

- `c <g>`  
  Calibrazione con peso noto.  
  Esempio: `c 2000` per un peso da 2000 g.  
  Procedura: TARE → metti il peso → `c 2000`.

- `p`  
  Stampa stato e parametri principali:

  - `OFFSET_RAW`, `SCALE_CPG`, `REF_G`,
  - `MA`, `DB_UNSTABLE`, ST on/off, ZT on/off, `zero_track_counts` in counts e in g.

- `s`  
  Salva in NVS: `offset`, `scale`, `ref_g`.

- `m work` / `m normal`  
  Preset “normale”:

  - `MA=6`, `deadband=0,10 g`
  - `ST=on`, `ZT=on`

- `m fine` / `m live`  
  Preset “fine/micro”:

  - `MA=4`, `deadband=0,05 g`
  - `ST=off` (stato LIVE), `ZT=on`

- `st on` / `st off` / `st ?`

  - Abilita/disabilita la macchina STABLE/UNSTABLE
  - `st off` → modalità LIVE (nessun latch)

- `zt on` / `zt off` / `zt reset` / `zt ?`

  - Attiva/disattiva zero-tracking + snap allo scarico
  - `zt reset` azzera `zero_track_counts`
  - `zt ?` mostra stato e valore di ZT in counts e in g

*(in futuro si può aggiungere un comando dedicato per il debug batteria, ma al momento il logging usa solo `battery_debug_print()` richiamata dal loop.)*

---

## 9. Tuning pratico

Alcune linee guida, da combinare con i parametri nel codice:

- **Micro dosi / ingredienti piccoli**
  - `m fine` (MA=4, deadband 0,05)
  - `ST off` se vuoi un comportamento più “live”

- **Lavoro normale in laboratorio**
  - `m work` o `m normal`
  - ST on, ZT on, deadband 0,10–0,20 g

- **Monitor batteria**
  - Considera `LOW` come “inizia a pensare alla ricarica”
  - Considera `CRITICAL` come “finisci la pesata e ricarica”
  - Usa `charging=YES` per capire al volo, dalla UI, se sei in carica o stai lavorando solo a batteria.
