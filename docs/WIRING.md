# Cablaggio hardware

Stato corrente del cablaggio usato dal firmware `firmware/esp32_hx711_serial`.

Moduli: ESP32 DevKit, HX711 alimentato a 5 V, cella di carico a 4 fili, OLED SSD1322 SPI, INA219 I2C, tastiera 4x2, buzzer, DFPlayer Mini e LED di standby.

## 1) Cella di carico → HX711

- Rosso → **E+**
- Nero → **E-**
- Verde → **A+**
- Bianco → **A-**

Il colore dei fili può variare tra produttori: se la cella non segue questa convenzione, verificare il datasheet prima di alimentarla.

## 2) HX711 → ESP32

### Alimentazione

- HX711 **VCC → 5 V** stabile
- HX711 **GND → GND comune**

### SCK

- ESP32 **GPIO16 → resistenza 220 Ω in serie → SCK HX711**

L'HX711 accetta il livello HIGH a 3,3 V. Non usare un level shifter bidirezionale a MOSFET su SCK: i moduli per I2C/open-drain possono mantenere la linea alta e causare power-down o letture instabili.

### DOUT / DT

Il cablaggio corrente usa un partitore resistivo:

- **DOUT HX711 → 10 kΩ → GPIO35 ESP32**
- **GPIO35 ESP32 → 20 kΩ → GND**

Il partitore limita a circa 3,3 V un eventuale livello DOUT a 5 V. GPIO35 è input-only e non dispone di pull-up/pull-down interno. Con questo partitore, se DOUT o l'HX711 vengono scollegati, il pin tende a restare basso: il rilevamento del modulo scollegato non può quindi essere garantito in ogni condizione.

## 3) OLED SSD1322 SPI → ESP32

- **SCK → GPIO18**
- **MOSI → GPIO23**
- **CS → GPIO25**
- **DC → GPIO26**
- **RST → GPIO27**
- Alimentazione e livelli logici secondo le specifiche del modulo OLED usato
- **GND → GND comune**

## 4) INA219 I2C → ESP32

- **SDA → GPIO32**
- **SCL → GPIO33**
- **VCC → 3,3 V**, salvo diversa indicazione del modulo
- **GND → GND comune**

L'INA219 misura la batteria e lo stato di carica. Il percorso di potenza e lo shunt devono rispettare lo schema del modulo specifico; SDA/SCL non trasportano la corrente del carico.

## 5) Tastiera 4x2 → ESP32

- **R1 → GPIO17**
- **R2 → GPIO5**
- **R3 → GPIO13**
- **R4 → GPIO14**
- **C1 → GPIO19**
- **C2 → GPIO21**

La matrice è gestita direttamente dal firmware; non collegare tensioni esterne alle righe o alle colonne.

## 6) Buzzer → ESP32

- Segnale buzzer → **GPIO22**
- Massa → **GND comune**

GPIO22 è riservato al buzzer e non è disponibile come alternativa per DOUT HX711 nel cablaggio corrente. Se il buzzer richiede più corrente di quella ammessa dal GPIO, pilotarlo tramite transistor.

## 7) DFPlayer Mini → ESP32

- **GPIO4 (TX ESP32) → resistenza 1 kΩ in serie → RX DFPlayer**
- **TX DFPlayer → GPIO34 (RX ESP32)**, collegamento opzionale
- **BUSY DFPlayer → GPIO39**
- **GPIO2 → circuito di power-gate**, `HIGH = ON`
- Altoparlante → **SPK1 / SPK2** del DFPlayer
- Alimentazione e condensatori secondo le specifiche del modulo
- **GND → GND comune**

GPIO34 e GPIO39 sono input-only e non hanno pull-up interno. Sul segnale BUSY è consigliata una pull-up esterna da **10–47 kΩ verso 3,3 V** se il modulo non garantisce un livello definito durante spegnimento o reset.

Il power-gate previsto dal firmware è un high-side con P-MOSFET comandato tramite NPN; non alimentare il DFPlayer direttamente da GPIO2.

## 8) LED di standby → ESP32

- **GPIO15 → resistenza 330 Ω–2,2 kΩ → anodo LED**
- Catodo LED → **GND**

Il firmware usa logica active-high: LED acceso durante standby/light sleep.

## 9) Alimentazione e masse

- Tutti i moduli che scambiano segnali devono condividere la massa con l'ESP32.
- Non applicare mai 5 V direttamente a un GPIO ESP32.
- Dimensionare buck, piste, cavi e protezioni per la corrente combinata di ESP32, OLED, HX711, INA219, buzzer e DFPlayer.
- Tenere i cavi della cella e dell'HX711 lontani dal DFPlayer, dall'altoparlante e dai percorsi di potenza per ridurre il rumore sulla misura.

## 10) Collegamenti da non usare

- Nessun level shifter MOSFET 3,3↔5 V su SCK HX711.
- Nessun collegamento diretto a 5 V su GPIO35.
- Nessun DOUT HX711 su GPIO22 senza una modifica coordinata di buzzer, firmware e documentazione.
- Nessun carico DFPlayer alimentato direttamente da GPIO2.
