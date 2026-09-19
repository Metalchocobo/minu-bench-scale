# Cablaggio hardware

Stato corrente del cablaggio usato dal firmware `firmware/esp32_hx711_serial`. Alimentazione montata: batteria USB-C NASTIMA LiFePO4 e Mini360 MP1482DS regolato a **5,11 V**, valore riferito da Andrea. Cablaggio completato e primo funzionamento apparentemente regolare riferiti il 18 settembre 2026; le tacche firmware usano fasce indicative LiFePO4, mentre protezioni e rilevamento ricarica conservano la configurazione precedente.

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

L'INA219 misura la tensione ai morsetti batteria e la corrente diretta al Mini360. **VIN+ riceve il positivo dopo F1; VIN− va a S1 e poi a IN+ del Mini360. VIN− è ancora il positivo, non GND.** SDA/SCL non trasportano la corrente del carico. La ricarica USB interna del pacco non attraversa questo shunt: il sensore non misura la corrente netta delle celle e l'icona firmware `charging` non certifica la ricarica USB.

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

### Percorso montato

- **Batteria scelta:** NASTIMA **BK06-LF60-NATC**, LiFePO4 **6,4 V nominali / 6 Ah**, caricatore USB-C e BMS integrati. Il riferimento commerciale «6 V» non indica una batteria al piombo.
- **Ricarica:** alimentatore USB esterno → collegamento USB-C da scocca → ingresso USB-C del pacco, dichiarato **5 V / 1,5 A**. Il caricatore è dentro il pacco; la prolunga non è un caricatore.
- **Positivo di potenza:** batteria + → **F1 vicino alla batteria → INA219 VIN+ → VIN− → S1 → Mini360 IN+**.
- **Negativo:** batteria −, Mini360 IN− e OUT−, ESP32 e periferiche condividono GND.
- **Uscita:** Mini360 OUT+ → linea nominale 5 V, regolata sul montaggio a **5,11 V** → ESP32 **VIN/5V**, HX711, OLED compatibile e power-gate audio esistente. Non alimentare il pin 3V3 con questa linea.
- **S1:** interruttore semplice dopo VIN− dell'INA219 e prima di IN+ e dei condensatori d'ingresso. In OFF disalimenta il Mini360 dalla batteria; non interrompe la ricarica interna del pacco. Gli ingressi INA219 possono restare sul bus batteria con VCC spento, come ammesso dal [datasheet TI, §8.3.1](https://www.ti.com/lit/ds/symlink/ina219.pdf).

F1 **T2 A** resta il valore proposto nello schema per fili corti in rame ≥ 0,5 mm²; non sono stati riferiti valore effettivamente montato o misure degli spunti. CTK3S e SLA non sono componenti del percorso corrente.

### Condensatori e collocazione

| Posizione | Elettrolitico | Ceramico | Collegamento |
|---|---|---|---|
| Ingresso Mini360, dopo S1 | C1: **220 µF / 16 V** | C2: **100 nF** | Entrambi tra **IN+ e IN−** |
| Uscita Mini360 | C3: **470 µF / 10 V** | C4: **100 nF** | Entrambi tra **OUT+ e OUT−** |

Ogni coppia è in parallelo all'alimentazione. Saldare vicino ai rispettivi pad, direttamente sulla schedina se c'è spazio oppure subito accanto con collegamenti corti. Andrea ha montato il ceramico di uscita più vicino al Mini360 e poi l'elettrolitico: disposizione coerente con il cablaggio. Il + dell'elettrolitico va al + della relativa coppia; la banda − va a IN−/OUT−. I ceramici non hanno polarità. Restano previsti i condensatori locali **10–47 µF + 100 nF** presso VIN/5V e GND dell'ESP32.

### Stato delle verifiche e monitoraggio

Il montaggio risulta funzionante dalla prima prova riferita da Andrea. **5,11 V** è la regolazione comunicata, non una registrazione di stabilità su tutti i carichi. Autonomia, cadute con Wi-Fi/audio, temperatura, transitori USB e riavvio dopo stacco BMS non hanno misure riportate.

Le tacche firmware usano fasce indicative LiFePO4; avvisi sonori e soglie di sleep restano indipendenti dalla mappa delle tacche. Il rilevamento charging usa ancora la corrente negativa: vedere [README, monitoraggio batteria](../README.md#8-monitoraggio-batteria-ina219). Il light-sleep non è uno stacco elettrico del pacco.

L'USB del PC sull'ESP32 è distinta dall'USB di ricarica del pacco. OUT+ rimane collegato all'ESP32 anche con S1 OFF: se la scheda riporta la tensione USB su VIN/5V, può rialimentare il Mini360. La sola rialimentazione non dimostra un danno; questo caso non è stato caratterizzato sulla scheda specifica. Tavole, pinout del retro e dettagli: [Mini360](../artifacts/mini360/README.md).

### Masse e segnali

- Tutti i moduli che scambiano segnali devono condividere la massa con l'ESP32.
- Non applicare mai 5 V direttamente a un GPIO ESP32.
- Dimensionare buck, piste, cavi e protezioni per la corrente combinata di ESP32, OLED, HX711, INA219, buzzer e DFPlayer.
- Tenere i cavi della cella e dell'HX711 lontani dal DFPlayer, dall'altoparlante e dai percorsi di potenza per ridurre il rumore sulla misura.

## 10) Collegamenti da non usare

- Nessun level shifter MOSFET 3,3↔5 V su SCK HX711.
- Nessun collegamento diretto a 5 V su GPIO35.
- Nessun DOUT HX711 su GPIO22 senza una modifica coordinata di buzzer, firmware e documentazione.
- Nessun carico DFPlayer alimentato direttamente da GPIO2.
