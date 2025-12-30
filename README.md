# Bilancia ESP32 + HX711 + OLED SSD1322 + INA219 · Prototipo 6 V (v2025-12-28)

Prototipo di **bilancia da banco** per laboratorio (gelato / pasticceria) basata su:

- ESP32 DevKit
- ADC 24 bit **HX711** per la cella di carico (eccitazione ponte a 5 V)
- Display **OLED SSD1322** 256×64 (3,12", SPI)
- Alimentazione a **batteria piombo 6 V** con modulo CTK3S
- Monitor batteria con **INA219** (CJMCU-219, lato alto)

> Progetto pensato per uso interno in laboratorio, non per pesate legali a fini commerciali.

## 0. Scopo e stato del progetto

- Portata target: **20 kg**
- Risoluzione visibile: **1 g** (clamp a ±16 kg per sicurezza)
- Uso tipico: pesate ingredienti in laboratorio, con focus su stabilità e lettura “ferma”.

Stato attuale:

- ✅ Lettura tramite **HX711** con filtri (mediana + media mobile)
- ✅ Macchina di stati **STABLE / UNSTABLE / LIVE**
- ✅ **Zero-tracking** vicino a zero + **snap-to-zero** allo scarico
- ✅ Auto-tare opzionale all’avvio
- ✅ UI base su OLED (peso grande + indicatore di stato)
- ✅ Tastierino frontale 4×2: **TARE** e **MODE** operativi
- ✅ Monitor batteria con INA219: 4 livelli (FULL / GOOD / LOW / CRITICAL) + flag charging
- ✅ Buzzer di sistema (boot + click/ok)

## 1. Perché HX711 (e cosa è successo con il NAU7802)

Per questa bilancia è emerso un vincolo pratico: la cella di carico (ponte estensimetrico) rende bene solo se **eccitata a 5 V**.

Con il NAU7802 si è provato a lavorare a 5 V, ma in un setup reale si è verificato backfeed/tensione troppo alta verso l’ESP32 sui pin logici, con danni hardware.

Quindi:

- Il firmware “attuale” è su HX711, alimentato a 5 V.
- L’ESP32 resta **sempre a 3,3 V** sui GPIO.
- Se HX711 è alimentato a 5 V, la linea **DOUT non deve mai andare diretta a un GPIO**.

Firmware disponibili:

- ✅ `firmware/esp32_hx711_serial/` (corrente)
- ⚠️ `firmware/esp32_nau7802_serial/` (storico, non consigliato)
- 🧱 `firmware/esp32_hx711_serial_legacy_nau/` (file vecchio che in realtà era NAU, tenuto solo come archivio)

## 2. Hardware

### 2.1 Architettura di alimentazione

Schema logico:

1. Alimentatore 12 V → ingresso PV CTK3S.
2. BAT+ / BAT− CTK3S → batteria piombo 6 V.
3. LOAD+ / LOAD− CTK3S → ingresso buck step-down 5 V.
4. Uscita 5 V del buck → rail 5 V che alimenta:
   - VIN/5V dell’ESP32
   - VCC dell’OLED SSD1322
   - VCC dell’HX711
   - Eccitazione cella (tramite HX711)
5. INA219 montato dal lato batteria (alto lato) per misurare tensione/corrente.

Motivo uso LOAD (e non BAT): il CTK3S stacca solo LOAD in undervoltage (LVD), proteggendo la batteria.

Condensatori consigliati:

- Ingresso buck (da LOAD CTK3S): 220 µF elettrolitico (≥ 16 V) + 100 nF ceramico
- Uscita buck (rail 5 V): 470 µF elettrolitico (≥ 10–16 V) + 100 nF ceramico
- Vicino all’ESP32 (VIN/5V ↔ GND): 10–47 µF elettrolitico + 100 nF ceramico

### 2.2 Pinout ESP32 (riassunto, firmware HX711)

| Funzione | Modulo | GPIO ESP32 | Note |
|---|---|---:|---|
| I2C SDA | INA219 | 32 | bus I2C batteria |
| I2C SCL | INA219 | 33 | |
| HX711 SCK | HX711 | 16 | uscita ESP32 3,3 V |
| HX711 DOUT | HX711 | 35 | ingresso ESP32 3,3 V (input-only) (vedi level shifting sotto) |
| SPI SCK | OLED SSD1322 | 18 | VSPI |
| SPI MOSI | OLED SSD1322 | 23 | |
| SPI CS | OLED SSD1322 | 25 | |
| SPI DC | OLED SSD1322 | 26 | |
| SPI RST | OLED SSD1322 | 27 | |
| Tastiera R1 | Keypad 4×2 | 17 | fila 1 |
| Tastiera R2 | Keypad 4×2 | 5 | fila 2 |
| Tastiera R3 | Keypad 4×2 | 13 | fila 3 |
| Tastiera R4 | Keypad 4×2 | 14 | fila 4 |
| Tastiera C1 | Keypad 4×2 | 19 | colonna 1 |
| Tastiera C2 | Keypad 4×2 | 21 | colonna 2 |
| Buzzer | buzzer passivo | 22 | pilotato con LEDC |

Nota: abbiamo messo **DOUT su GPIO35** (input-only) proprio per evitare i **pin di strap** e ridurre i rischi di boot.

## 3. Cablaggio dettagliato

### 3.1 Cella di carico ↔ HX711

Colori tipici (verifica sempre il tuo cavo):

- E+ / EXC+ (eccitazione +) → rosso
- E− / EXC− (eccitazione −) → nero
- A+ / SIG+ (segnale +) → verde
- A− / SIG− (segnale −) → bianco

### 3.2 HX711 ↔ ESP32 (logica)

Alimentazione HX711:

- VCC → 5 V (rail buck)
- GND → GND comune

Segnali:

- SCK (clock) → GPIO16 (diretto, 3,3 V è ok)
- DOUT (data) → GPIO35 **solo dopo conversione a 3,3 V**

#### Opzione consigliata: level shifter (quello che hai già)

Hai i moduli: **Logic Level Converter Bi-Directional 4 canali (3,3 V ↔ 5 V)**.

Collega così:

- HV → 5 V
- LV → 3V3
- GND → GND comune
- DOUT HX711 sul lato HV (canale 1) → corrispondente LV → GPIO35
- (opzionale ma ok) SCK ESP32 GPIO16 lato LV → corrispondente HV → SCK HX711

Pro: è la soluzione più “a prova di errore”.

#### Opzione alternativa: partitore resistivo (solo su DOUT)

Se vuoi evitare il level shifter:

- Metti un partitore su DOUT (5 V → 3,3 V). Valori tipici:
  - R alto 20 kΩ (DOUT→GPIO)
  - R basso 10 kΩ (GPIO→GND)

SCK può restare diretto (3,3 V verso HX711 è ok).

Contro: più facile sbagliare cablaggio; meno robusto.

### 3.3 INA219 ↔ batteria + ESP32

Lato potenza (alto lato):

- BAT+ → fusibile → VIN+ INA219
- VIN− INA219 → B+ CTK3S
- BAT− → B− CTK3S → GND comune

Lato logica:

- VCC → 3V3 ESP32
- GND → GND comune
- SDA → GPIO32
- SCL → GPIO33

Indirizzo I2C di default: `0x40`.

### 3.4 OLED SSD1322 ↔ ESP32 (SPI)

Bus VSPI:

- SCK → GPIO18
- MOSI → GPIO23

Controlli:

- CS  → GPIO25
- DC  → GPIO26
- RST → GPIO27

Alimentazione display:

- VCC → 5 V
- GND → GND

### 3.5 Tastierino 4×2

Collegamento fili → GPIO (coerente col firmware):

- Filo 1 (R1) → GPIO17
- Filo 2 (R2) → GPIO5
- Filo 3 (R3) → GPIO13
- Filo 4 (R4) → GPIO14
- Filo 5 (C1) → GPIO19
- Filo 6 (C2) → GPIO21

Comportamento attuale:

- `TARE`: tara completa
- `MODE`: alterna WORK/normal ↔ LIVE/fine

## 4. Firmware

Firmware corrente:

- `firmware/esp32_hx711_serial/esp32_hx711_serial.ino`

Note:

- La logica di STABLE/UNSTABLE/LIVE e ZT è stata mantenuta identica rispetto al ramo NAU.
- L’unica sostituzione è il driver di lettura (HX711 al posto del NAU).
