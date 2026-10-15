# Cablaggio hardware

Fonti del cablaggio: documentazione HX711 confermata da Andrea e due ritagli di schema forniti per gli altri moduli. Il primo indica **OLED VCC e INA219 VCC da ESP32 3V3**; il secondo indica **source IRF9540 sulla linea VIN/5V**, il comando audio e la **resistenza LED da 1 kΩ**. I dettagli ancora da verificare sono elencati nella sezione finale.

Stato corrente del cablaggio usato dal firmware `firmware/esp32_hx711_serial`. Alimentazione montata: batteria USB-C NASTIMA LiFePO4 e Mini360 MP1482DS regolato a **5,11 V**, valore riferito da Andrea. Cablaggio completato e primo funzionamento apparentemente regolare riferiti il 18 settembre 2026; le tacche firmware usano fasce indicative LiFePO4 e l'icona di ricarica usa la tensione filtrata, senza modificare soglie e tempi delle protezioni.

Moduli: ESP32 DevKit, HX711 alimentato a 5 V, cella di carico a 4 fili, OLED SSD1322 SPI, INA219 I2C, tastiera 4x2, buzzer, DFPlayer Mini e LED di standby.

La [guida illustrata del cablaggio corrente](../artifacts/wiring-current/README.md) raccoglie tavole e istruzioni per ricostruire i collegamenti. **Schema generale con tutti i fili:** [PDF A2, pagina singola](../artifacts/wiring-current/cablaggio-generale.pdf), [PNG](../artifacts/wiring-current/00-wiring-complete.png) / [SVG](../artifacts/wiring-current/00-wiring-complete.svg). I componenti mostrati nello schema dell'audio e i valori riferiti da Andrea sono distinti dalle raccomandazioni non confermate sul montaggio.

### ESP32 montato: orientamento e terminali

La vista fornita da Andrea identifica una scheda **a 30 pin, con 15 terminali per fila**. Leggere le due file **dall'alto verso il basso**, mantenendo **3V3 in alto a sinistra e VIN in alto a destra**; destra e sinistra si riferiscono a questa vista, non al retro della scheda.

| Posizione dall'alto | Fila sinistra | Fila destra |
|---|---|---|
| 1 | **3V3** | **VIN** |
| 2 | **GND** | **GND** |
| 3 | **GPIO15** | **GPIO13** |
| 4 | **GPIO2** | **GPIO12** |
| 5 | **GPIO4** | **GPIO14** |
| 6 | **RX2 / GPIO16** | **GPIO27** |
| 7 | **TX2 / GPIO17** | **GPIO26** |
| 8 | **GPIO5** | **GPIO25** |
| 9 | **GPIO18** | **GPIO33** |
| 10 | **GPIO19** | **GPIO32** |
| 11 | **GPIO21** | **GPIO35** |
| 12 | **RX0 / GPIO3** | **GPIO34** |
| 13 | **TX0 / GPIO1** | **GPIO39 / VN** |
| 14 | **GPIO22** | **GPIO36 / VP** |
| 15 | **GPIO23** | **EN** |

L'immagine riporta anche SDA accanto a GPIO21 e SCL accanto a GPIO22: sono alias del pinout della scheda. Nel cablaggio corrente questi GPIO servono rispettivamente tastiera e buzzer; **INA219 SDA va a GPIO32 e SCL a GPIO33**. GPIO12, GPIO36/VP ed EN non hanno un collegamento esterno assegnato dal progetto; RX0/TX0 restano dedicati alla seriale. La vista identifica le posizioni dell'ESP32, non l'orientamento fisico dei piedini del NPN **2N3904 H331** né il modello dell'OLED o del breakout HX711.

## 1) Cella di carico → HX711

- Rosso → **E+**
- Nero → **E-**
- Verde → **A+**
- Bianco → **A-**

Il colore dei fili può variare tra produttori: se la cella non segue questa convenzione, verificare il datasheet prima di alimentarla.

## 2) HX711 → ESP32

### Alimentazione

Il **modulo HX711 è alimentato dalla linea nominale 5 V del Mini360**, regolato a **5,11 V**, perché la cella di carico usata richiede un'eccitazione di **circa 5 V** per mantenere stabilità e precisione. La cella è collegata a **E+ / E− del modulo HX711**; tutte le masse sono comuni. I GPIO dell'ESP32 restano a **3,3 V**. La tensione effettiva E+/E− dipende dal breakout e non è stata misurata.

- HX711 **VCC → 5 V** stabile
- HX711 **GND → GND comune**

### SCK

- ESP32 **GPIO16 → resistenza 220 Ω in serie → SCK HX711**
- **100 kΩ pull-down da prevedere nella replica** fra **SCK e GND**, dopo la 220 Ω sul lato HX711 e vicino al modulo. Mantiene SCK LOW quando GPIO16 non è pilotato durante avvio/reset: è una scelta di robustezza, non un obbligo del datasheet AVIA né un level shifter.
- Andrea ritiene che la 100 kΩ sia già montata: presenza attuale **probabile, da confermare**. Il rilievo fisico resta distinto dalla decisione di includerla nella replica.

Il collegamento HX711 confermato usa il segnale del GPIO16 a 3,3 V attraverso la resistenza da 220 Ω. Non usare un level shifter bidirezionale a MOSFET su SCK: i moduli per I2C/open-drain possono mantenere la linea alta e causare power-down o letture instabili.

### DOUT / DT

Il cablaggio corrente usa un partitore resistivo:

- **DOUT HX711 → 10 kΩ → GPIO35 ESP32**
- **GPIO35 ESP32 → 20 kΩ → GND**

Il rapporto ideale del partitore è 2/3: **3,33 V** con DOUT a 5 V, **3,41 V** con DOUT a 5,11 V, prima delle tolleranze; i livelli effettivi del breakout non sono stati misurati. GPIO35 è input-only e non dispone di pull-up/pull-down interno. Con questo partitore, se DOUT o l'HX711 vengono scollegati, il pin tende a restare basso: il rilevamento del modulo scollegato non può quindi essere garantito in ogni condizione.

## 3) OLED SSD1322 SPI → ESP32

- **SCK → GPIO18**
- **MOSI → GPIO23**
- **CS → GPIO25**
- **DC → GPIO26**
- **RST → GPIO27**
- **VCC → ESP32 3V3**, come indicato nel primo ritaglio di schema
- **GND → GND comune**

## 4) INA219 I2C → ESP32

- **SDA → GPIO32**
- **SCL → GPIO33**
- **VCC → ESP32 3V3**, come indicato nel primo ritaglio di schema
- **GND → GND comune**

L'INA219 misura la tensione ai morsetti batteria e la corrente diretta al Mini360. **VIN+ riceve il positivo dopo F1; VIN− va a S1 e poi a IN+ del Mini360. VIN− è ancora il positivo, non GND.** SDA/SCL non trasportano la corrente del carico. La ricarica USB interna del pacco non attraversa questo shunt: il sensore non misura la corrente netta delle celle. L'icona firmware `charging` è una stima dalla tensione e non certifica la ricarica USB.

## 5) Tastiera 4x2 → ESP32

La mappa corrente è stata rilevata dagli otto tasti del montaggio e definisce il firmware operativo `esp32_hx711_serial`. I sei fili restano sui GPIO indicati; **TX2 è GPIO17**, distinto da TX0/GPIO1 della seriale USB. Le sigle **R1–R6** del ritaglio di schema non definiscono l'ordine fisico dei contatti sul connettore: identificare le linee tramite le coppie GPIO dei tasti.

| Linea GPIO | Tasti collegati | Ruolo nella scansione |
|---|---|---|
| **GPIO13** | SKIP, SLEEP | riga / uscita |
| **GPIO14** | ENTER, CLEAR | riga / uscita |
| **GPIO19** | WIFI, TOTAL | riga / uscita |
| **GPIO21** | MODE, TARE | riga / uscita |
| **TX2 / GPIO17** | SKIP, ENTER, WIFI, MODE | colonna / ingresso pull-up |
| **GPIO5** | SLEEP, CLEAR, TOTAL, TARE | colonna / ingresso pull-up |

Righe e colonne sono il raggruppamento logico usato dalla scansione, non la posizione del connettore o dei tasti sul pannello. La matrice è gestita direttamente dal firmware; non collegare le sei linee direttamente a 3V3, 5 V o GND.

| Riga scansione | Colonna GPIO17 | Colonna GPIO5 |
|---|---|---|
| GPIO13 | SKIP | SLEEP |
| GPIO14 | ENTER | CLEAR |
| GPIO19 | WIFI | TOTAL |
| GPIO21 | MODE | TARE |

Il firmware attuale pilota la riga selezionata LOW e le altre HIGH; diodi o resistenze serie non sono documentati nella matrice. Con contatti diretti, due tasti della stessa colonna su righe diverse mettono in contesa le uscite. La combinazione prevista WIFI + TOTAL condivide la riga GPIO19. In light-sleep tutte le righe GPIO13, 14, 19 e 21 sono LOW e le colonne GPIO17 e 5 rilevano il wake; il tasto di risveglio viene consumato fino al rilascio.

Per verificare una replica, lo [sketch di test tastiera](../firmware/keypad_wiring_test/README.md) rileva tutte le coppie GPIO senza assegnare preventivamente righe o colonne. Raccogliere gli otto risultati prima di modificare la mappa operativa.

## 6) Buzzer → ESP32

- Segnale buzzer → **GPIO22**
- Massa → **GND comune**

GPIO22 è riservato al buzzer e non è disponibile come alternativa per DOUT HX711 nel cablaggio corrente. Se il buzzer richiede più corrente di quella ammessa dal GPIO, pilotarlo tramite transistor.

## 7) DFPlayer Mini → ESP32

- **GPIO4 (TX ESP32) → resistenza 1 kΩ in serie → RX DFPlayer**
- **TX DFPlayer → GPIO34 (RX ESP32)**, collegato nello schema; opzionale per il firmware
- **BUSY DFPlayer → GPIO39**, diretto nello schema
- **GPIO2 → circuito di power-gate**, `HIGH = ON`
- Altoparlante → **SPK1 / SPK2** del DFPlayer
- **VCC → drain IRF9540**, come dettagliato sotto
- **GND → GND comune**
- Presso il DFPlayer sono confermati un **elettrolitico da 470 µF / 16 V** e un **ceramico marcato 104, cioè 100 nF**. Quantità complessiva, collegamenti, polarità e collocazione esatta restano da rilevare sul circuito. Nelle tavole i collegamenti tratteggiati dei condensatori fra **VCC audio e GND** sono un riferimento **da confermare**, non la verifica del montaggio. Il 470 µF / 16 V audio è distinto dal **470 µF / 10 V del Mini360**.

GPIO34 e GPIO39 sono input-only e non hanno pull-up interno. Il collegamento BUSY mostrato è diretto: una pull-up esterna da **10–47 kΩ verso 3,3 V** è una raccomandazione condizionale se il modulo non garantisce un livello definito durante spegnimento o reset; non è confermata sul montaggio. Non confonderla con le due resistenze da 100 kΩ del power-gate.

### Power-gate montato: IRF9540 + NPN 2N3904

| Collegamento | Destinazione |
|---|---|
| IRF9540 **source (S)** | Linea **VIN/5V del Mini360**, regolata a **5,11 V**, come indicato nel ritaglio audio; non 3V3 |
| IRF9540 **drain (D)** | **VCC DFPlayer** |
| IRF9540 **gate (G)** | **Collettore NPN** |
| Resistenza **100 kΩ** | Fra **gate e source IRF9540** |
| **GPIO2 → 10 kΩ** | **Base NPN** |
| Resistenza **100 kΩ** | Fra **base NPN e GND** |
| **Emettitore NPN** | **GND comune** |

Con GPIO2 HIGH il NPN porta il gate verso GND e accende il P-MOSFET; con GPIO2 LOW la resistenza gate-source spegne il P-MOSFET. I modelli montati identificati da Andrea sono **IRF9540** e **2N3904**, quest'ultimo con marcatura **H331**. Il 2N3904 è adatto al comando del gate; H331 è riportato senza attribuirgli produttore o data. L'orientamento fisico dei piedini del transistor reale resta da verificare: le lettere B/C/E indicano funzioni, non una disposizione fisica universale dei piedini. Il [datasheet onsemi 2N3904](https://www.onsemi.com/download/data-sheet/pdf/2n3904-d.pdf) indica **pin 1 = E, 2 = B, 3 = C** per i suoi package TO-92: è un riferimento del produttore, non l'identificazione del package o del pinout dell'esemplare montato.

Il firmware chiude UART e imposta GPIO4 e GPIO34 come INPUT prima di portare GPIO2 LOW. Non alimentare il DFPlayer direttamente da GPIO2. La topologia coincide con il comando firmware `HIGH = ON`; caduta di tensione e temperatura dell'IRF9540 durante l'audio non sono state misurate.

## 8) LED di standby → ESP32

- **GPIO15 → resistenza 1 kΩ → anodo LED**
- Catodo LED → **GND**

Il LED montato è un normale **LED rosso del kit Arduino**. La resistenza **1 kΩ** è identificata nel ritaglio di schema fornito da Andrea.

Il firmware usa logica active-high: LED acceso durante standby/light sleep.

## 9) Alimentazione e masse

### Percorso montato

- **Batteria scelta:** NASTIMA **BK06-LF60-NATC**, LiFePO4 **6,4 V nominali / 6 Ah**, caricatore USB-C e BMS integrati. Il riferimento commerciale «6 V» non indica una batteria al piombo.
- **Ricarica:** alimentatore USB esterno → collegamento USB-C da scocca → ingresso USB-C del pacco, dichiarato **5 V / 1,5 A**. Il caricatore è dentro il pacco; la prolunga non è un caricatore.
- **Positivo di potenza:** batteria + → **F1 vicino alla batteria → INA219 VIN+ → VIN− → S1 → Mini360 IN+**.
- **Negativo:** batteria −, Mini360 IN− e OUT−, ESP32 e periferiche condividono GND.
- **Uscita:** Mini360 OUT+ → linea nominale 5 V, regolata sul montaggio a **5,11 V** → ESP32 **VIN/5V**, HX711 **VCC** e IRF9540 **S** del power-gate audio. ESP32 **3V3 → OLED VCC e INA219 VCC**. Non applicare la linea 5 V al pin 3V3.
- **S1:** interruttore semplice dopo VIN− dell'INA219 e prima di IN+ e dei condensatori d'ingresso. In OFF disalimenta il Mini360 dalla batteria; non interrompe la ricarica interna del pacco. Gli ingressi INA219 possono restare sul bus batteria con VCC spento, come ammesso dal [datasheet TI, §8.3.1](https://www.ti.com/lit/ds/symlink/ina219.pdf).

F1 **T2 A** resta il valore proposto nello schema per fili corti in rame ≥ 0,5 mm²; non sono stati riferiti valore effettivamente montato o misure degli spunti. CTK3S e SLA non sono componenti del percorso corrente.

### Condensatori e collocazione

| Posizione | Elettrolitico | Ceramico | Collegamento |
|---|---|---|---|
| Ingresso Mini360, dopo S1 | C1: **220 µF / 16 V** | C2: **100 nF** | Entrambi tra **IN+ e IN−** |
| Uscita Mini360 | C3: **470 µF / 10 V** | C4: **100 nF** | Entrambi tra **OUT+ e OUT−** |

Ogni coppia è in parallelo all'alimentazione. Saldare vicino ai rispettivi pad, direttamente sulla schedina se c'è spazio oppure subito accanto con collegamenti corti. Andrea ha montato il ceramico di uscita più vicino al Mini360 e poi l'elettrolitico: disposizione coerente con il cablaggio. Il + dell'elettrolitico va al + della relativa coppia; la banda − va a IN−/OUT−. I ceramici non hanno polarità.

Sulla **PCB custom fra Mini360 ed ESP32** è montata una **coppia aggiuntiva 47 µF / 16 V elettrolitico + ceramico 104 = 100 nF**, in parallelo fra **linea 5 V e GND**. Il + dell'elettrolitico va alla linea 5 V e il − a GND; il ceramico non ha polarità. La coppia è identificata e distinta da quelle Mini360 e presso l'ESP32.

**Sono montati anche condensatori esterni presso l'ESP32 fra VIN/5V e GND**; i terminali sono confermati, mentre i valori fisici di questa coppia non sono stati letti. Per la sua replica sono consigliati **47 µF / 16 V elettrolitico + 100 nF ceramico (104)**, entrambi in parallelo fra VIN/5V e GND e vicini alla scheda. Il **+ dell'elettrolitico va a VIN/5V**, il **− va a GND**; il ceramico non ha polarità. I valori sono consigliati per questa coppia, non letti sui componenti presenti. Non fondere la coppia presso l'ESP32 con quella aggiuntiva sulla PCB custom.

| Gruppo distinto | Elettrolitico | Ceramico | Stato |
|---|---|---|---|
| Ingresso Mini360 | **220 µF / 16 V** | **100 nF / 104** | Cablaggio documentato, fra IN+ e IN− |
| Uscita Mini360 | **470 µF / 10 V** | **100 nF / 104** | Cablaggio documentato, fra OUT+ e OUT− |
| PCB custom fra Mini360 ed ESP32 | **47 µF / 16 V** | **100 nF / 104** | Coppia aggiuntiva identificata, fra 5 V e GND |
| Presso ESP32 | Valore fisico ignoto; **47 µF / 16 V consigliato per replica** | Valore fisico ignoto; **100 nF / 104 consigliato per replica** | Coppia montata distinta, fra VIN/5V e GND |
| Audio DFPlayer | **470 µF / 16 V** | **100 nF / 104** | Valori identificati; collegamenti e polarità restano da verificare |

### Stato delle verifiche e monitoraggio

Il montaggio risulta funzionante dalla prima prova riferita da Andrea. **5,11 V** è la regolazione comunicata, non una registrazione di stabilità su tutti i carichi. Autonomia, cadute con Wi-Fi/audio, temperatura, transitori USB e riavvio dopo stacco BMS non hanno misure riportate.

Le tacche firmware usano fasce indicative LiFePO4. L'icona `charging` si accende a tensione filtrata ≥ **6,70 V per 5 s** e si spegne a ≤ **6,68 V per 10 s**; nella banda intermedia conserva lo stato. Sono soglie indicative ricavate dalle letture riferite, non un rilevamento della presenza USB: a batteria bassa la ricarica può non raggiungerle. Avvisi sonori e protezioni di sleep restano attivi indipendentemente dalle tacche e dall'icona; campioni invalidi/scaduti non alimentano il rilevamento. Dettagli nel [README, monitoraggio batteria](../README.md#8-monitoraggio-batteria-ina219). Il light-sleep non è uno stacco elettrico del pacco.

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

## 11) Dati da rilevare per una replica esatta

La guida riporta i collegamenti dalle fonti indicate, non un collaudo elettrico completo. Sono identificati **2N3904 H331**, **condensatori audio 470 µF / 16 V + ceramico 104 = 100 nF**, **coppia aggiuntiva PCB custom 47 µF / 16 V + 104** e **resistenza LED 1 kΩ**. Restano da verificare **orientamento fisico e pinout del NPN**, **versione e ordine fisico dei terminali del modulo OLED**, **quantità, collegamenti, polarità e collocazione dei condensatori audio**, **valori fisici della distinta coppia presso l'ESP32** e **valore del fusibile F1**. L'alimentazione OLED e INA219 da **3V3** è indicata nel primo ritaglio; il source IRF9540 su **VIN/5V** è indicato nel secondo. La coppia presso ESP32 è confermata fra **VIN/5V e GND**; per la sua replica si adottano i valori consigliati **47 µF / 16 V + 100 nF**, senza presentarli come letti sul montaggio e senza confonderli con la coppia PCB custom identificata. La **100 kΩ SCK–GND è da prevedere nella replica**; soltanto la sua presenza sul montaggio attuale resta **probabile, da confermare**. La pull-up BUSY resta una raccomandazione condizionale non confermata sul montaggio.
