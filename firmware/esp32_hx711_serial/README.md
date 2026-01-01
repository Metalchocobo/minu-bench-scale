# ESP32 + HX711 (Serial + OLED) — esp32_hx711_serial (v2 + parity merge)

Questa cartella contiene il firmware **HX711** per la bilancia “Minù Bench Scale”.
Obiettivo: sostituire NAU7802 con HX711 **senza cambiare la logica utente** (stabilità STABLE/UNSTABLE, zero-tracking, beep/tasti, OLED).

## Collegamenti consigliati (HX711 a 5 V)

### Alimentazioni
- **HX711 VCC → 5V**
- **HX711 GND → GND** (massa comune con ESP32)

### Cella di carico (verso HX711)
Dipende dal colore dei fili della cella: collega i 4 fili ai pin **E+, E-, A+, A-** del modulo HX711 seguendo lo schema della tua cella.

### Segnali HX711 ↔ ESP32
Pin usati in questo firmware:
- **HX711 SCK → ESP32 GPIO16 (RX2)**  
- **HX711 DOUT → ESP32 GPIO35**

#### Protezione 3.3 V su DOUT (obbligatoria se HX711 è a 5 V)
Porta **DOUT a 3.3 V** con un **partitore** (consigliato):
- DOUT (HX711) → **10 kΩ** → GPIO35 (ESP32)
- GPIO35 (ESP32) → **20 kΩ** → GND

(Valori equivalenti vanno bene: l’importante è che il rapporto riporti ~3.3 V massimo sul pin ESP32).

#### SCK: niente level shifter “I2C-style”
- **SCK deve essere 3.3 V** dall’ESP32.  
- Evita i classici level shifter a MOSFET (BSS138) su SCK: possono tenere la linea alta e mandare l’HX711 in power-down.

Consigli pratici:
- **220 Ω in serie** sul filo SCK vicino all’ESP32 (riduce ringing/spike, non è un “convertitore di livello”).
- **100 kΩ pulldown** tra SCK e GND vicino al modulo HX711 (tiene SCK LOW durante il boot/reset dell’ESP32).

## 10 SPS vs 80 SPS (hardware)
La velocità (10 Hz o 80 Hz) **non si imposta da software** sull’HX711.
Dipende dal pin/ponte **RATE** del breakout:
- di solito **default = 10 SPS**
- per **80 SPS**: modificare la connessione “RATE” sul retro del modulo (taglio pista / saldatura ponticello, dipende dal modello)

## Batteria SLA 6V (INA219)
Questo firmware usa **INA219** su I2C per:
- icona batteria (tacche)
- rilevare stato **in carica** (corrente negativa)

**Gestione batteria SLA 6V (safety ESP):**
- **0 tacche (EMPTY):** beep di avviso ogni **60s** (UI peso normale).
- **Fase di stacco (V ≤ 5.80 V per ≥ 5s):** mostra solo avviso + beep ogni **10s** per **60s**, poi entra in **LIGHT-SLEEP**.
- **Wake:** solo da tastiera (qualsiasi tasto). Nessun wake automatico.
- **Isteresi:** annulla countdown se V ≥ 5.90 V o se `charging=true`.

Pin I2C usati:
- **SDA = GPIO32**
- **SCL = GPIO33**

### Soglie tacche (SLA 6V)
Sono soglie “pratiche” (dipendono da carico/temperatura):
- **4 tacche (FULL):** ≥ **6.35 V**
- **3 tacche (GOOD):** ≥ **6.20 V**
- **2 tacche (LOW):**  ≥ **6.05 V**
- **1 tacca (CRITICAL):** ≥ **5.90 V**
- **0 tacche (EMPTY):** < **5.90 V**

### Spegnimento di sicurezza (protezione ESP)
Se **non** è in carica e la tensione filtrata scende sotto:
- **5.80 V per almeno 5 s**

Il display mostra **“BATTERIA SCARICA”** e l’ESP32 entra in **deep-sleep**.
Wake-up automatico: ogni **30 s** (per verificare se la tensione è tornata ok / se hai collegato il caricatore).

## Cosa è stato cambiato rispetto a v2 “pura”
- Driver HX711 **separato** in `hx711_driver.cpp/.h` (stessa lettura 24 bit, stesso gain).
- Cadenzamento campionamento: se HX711 **non è pronto**, il firmware **non avanza** il timebase (range/slope/stability restano coerenti).
- Seed calibrazione di default aggiornati (valori realistici):
  - `DEFAULT_ZERO_RAW = 51471`
  - `DEFAULT_REF_RAW  = 265290` con `DEFAULT_REF_G = 2000 g`
- Auto-tare “smart”:
  - minimo 10 campioni
  - continua finché la finestra rientra sotto soglia per un tempo minimo
  - massimo 25 campioni

## Note importanti
- Se vedi letture “0 fisse” o DOUT che non scende: è quasi sempre un problema di **pull-up/level shifter** o SCK che resta HIGH.
- Con cella scollegata è normale vedere valori instabili/oscillanti.

## HX711 a 80 SPS: filtro e reattività

Se il tuo HX711 è impostato a **80 SPS**, il firmware legge ogni campione ma decima:
- WORK: media di 5 campioni (≈16 Hz) per stabilità/zero-tracking.
- LIVE: media di 2 campioni (≈40 Hz) solo in modalità LIVE, con rounding a isteresi (anti-flicker).
