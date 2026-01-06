# Wiring (HX711 + OLED + INA219)

**Moduli**: ESP32 DevKit, HX711 (alimentato a 5 V), cella 4 fili, OLED SSD1322 SPI, INA219 I2C.

---

## 1) Cella → HX711
- Rosso → **E+**
- Nero  → **E-**
- Verde → **A+**
- Bianco→ **A-**

**Perché**: cablaggio standard load cell 4 fili su canale A dell’HX711.

---

## 2) HX711 → ESP32 (alimentazione)
- HX711 **VCC → 5V** (rail buck / 5V stabile)
- HX711 **GND → GND comune**

**Perché**:
- la cella rende meglio e più pulita a 5 V;
- massa comune obbligatoria per avere riferimenti corretti sui segnali.

---

## 3) HX711 → ESP32 (segnali)

### SCK (clock)
- ESP32 **GPIO16 → 220 Ω in serie → SCK (HX711)**

**Perché**:
- HX711 accetta HIGH a 3.3 V, quindi niente 5 V su SCK;
- la **220 Ω in serie** smorza spike/ringing e riduce disturbi senza alterare la logica.

> Nota: niente level shifter su SCK. I classici bidirezionali a MOSFET (tipo BSS138) sono pensati per I2C/open-drain e possono tenere la linea alta, causando power-down o letture instabili.

### DOUT / DT (data ready + dati)
**Partitore resistivo (protezione da 5 V)**:
- **DOUT (HX711) → 10 kΩ → GPIO35 (ESP32)**
- **GPIO35 (ESP32) → 20 kΩ → GND**

**Perché**:
- evita che arrivino 5 V sul GPIO (se DOUT fosse a 5 V, su GPIO35 arrivano ~3.33 V);
- soluzione semplice e stabile, senza i problemi dei level shifter MOSFET.

**Limite accettato**:
- con questo partitore, se DOUT/HX si scollega il pin tende a stare basso (pull-down), quindi la detection “HX scollegato” non è affidabile al 100%. Abbiamo deciso di accettarlo per non cambiare l’hardware.

---

## 4) OLED SSD1322 (SPI) → ESP32
- **SCK = GPIO18**
- **MOSI = GPIO23**
- **CS = GPIO27**
- **DC = GPIO25**
- **RST = GPIO26**

**Perché**: SPI hardware affidabile e pin già consolidati nel firmware.

---

## 5) INA219 (I2C) → ESP32
- **SDA = GPIO32**
- **SCL = GPIO33**
- VCC → 3.3 V (o come da modulo INA219, di solito 3.3 V)
- GND → GND comune

**Perché**:
- 32/33 sono risultati corretti dopo il test di inversione;
- lettura batteria/charging stabilizzata lato software (debounce + isteresi).

---

## Cosa NON usare (decisioni prese)
- **No level shifter MOSFET 3.3↔5V su SCK**: è pensato per I2C/open-drain e può tenere SCK alto, causando power-down o comportamenti strani dell’HX711.
- **No 5 V diretto su GPIO35**: DOUT deve essere sempre limitato a livelli sicuri (qui tramite partitore 10k/20k).
