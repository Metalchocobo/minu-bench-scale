# Guida illustrata al cablaggio della bilancia montata

Le fonti della guida sono la documentazione HX711 confermata da Andrea, le coppie GPIO rilevate per la tastiera montata e i due ritagli di schema forniti per gli altri moduli. **OLED e INA219 VCC arrivano da ESP32 3V3; HX711 VCC e IRF9540 S dalla linea Mini360 VIN/5V**. La resistenza LED è **1 kΩ** nello schema; la **100 kΩ SCK–GND è da prevedere nella replica**, mentre soltanto la sua presenza sulla bilancia attuale resta probabile, da confermare. I dettagli fisici ancora da completare sono elencati in fondo.

Questa guida raccoglie i collegamenti della bilancia **NASTIMA + Mini360**, con ESP32, HX711, OLED SSD1322, INA219, tastiera, buzzer, DFPlayer Mini, LED di standby e rilevatore presenza USB **PC817 DIP-4**. Il PC817 fa parte del cablaggio per la replica; il suo montaggio fisico sull'esemplare attuale non è confermato. La guida serve a riconoscere i fili e preparare una replica seguendo le fonti indicate; i dettagli che richiedono la bilancia sotto mano sono indicati esplicitamente.

Il **MOSFET IRF9540 resta il componente scelto**. La tastiera usa la mappa firmware ricavata dalle coppie GPIO rilevate sul cablaggio montato. Sleep e LED conservano il loro funzionamento. Questa guida non prescrive una nuova variante di alimentazione.

**Per una replica identica occorre completare i dettagli fisici elencati in fondo**, soprattutto orientamento dei piedini del NPN 2N3904, collegamenti dei condensatori audio e identificazione del fusibile montato. **Per la replica F1 è scelto: F2A rapido, ceramico, 5×20 mm**, con requisiti DC dichiarati nella [specifica F1](../../docs/WIRING.md#fusibile-f1-per-la-replica). Le illustrazioni mostrano dove collegare i fili; sui moduli di cui manca il modello esatto conta la scritta del terminale, non la sua posizione nel disegno.

## 1. Tavole da consultare

Lo [schema generale in PDF A2, pagina singola](./cablaggio-generale-pc817.pdf) mostra **tutti i fili e tutti i 30 terminali dell'ESP32 nelle posizioni della vista fornita da Andrea**. La [guida navigabile](./index.html) riunisce schema generale e tavole di dettaglio; il [PDF della guida illustrata](./guida-illustrata.pdf) raccoglie le sette tavole di dettaglio in sette pagine A3 orizzontali. Ogni tavola è disponibile come PNG e come SVG ingrandibile.

| Tavola | Cosa mostra | File |
|---|---|---|
| 00 — Schema generale | Tutti i collegamenti, componenti e pin dell'ESP32 nell'ordine fisico fornito | [PDF A2](./cablaggio-generale-pc817.pdf) · [PNG](./00-wiring-complete.png) · [SVG](./00-wiring-complete.svg) |
| 01 — Vista completa | Tutti i moduli e i gruppi di fili | [PNG](./01-wiring-overview.png) · [SVG](./01-wiring-overview.svg) |
| 02 — Alimentazione | Batteria, ricarica, fusibile, INA219, interruttore, Mini360 e condensatori | [PNG](./02-power.png) · [SVG](./02-power.svg) |
| 03 — Cella e HX711 | I quattro fili della cella, SCK e partitore DOUT | [PNG](./03-hx711.png) · [SVG](./03-hx711.svg) |
| 04 — Audio | DFPlayer, altoparlante, IRF9540, NPN e resistenze | [PNG](./04-audio.png) · [SVG](./04-audio.svg) |
| 05 — Comandi | Tastiera, buzzer e LED | [PNG](./05-controls.png) · [SVG](./05-controls.svg) |
| 06 — Display e misura batteria | OLED e collegamenti di alimentazione/segnale INA219 | [PNG](./06-display-ina.png) · [SVG](./06-display-ina.svg) |
| 07 — Presenza USB | PC817 DIP-4, ramo USB del caricatore, resistenze e ingresso GPIO36 / VP | [PNG](./07-usb-pc817.png) · [SVG](./07-usb-pc817.svg) |

Per i **pad reali del Mini360 visto dal retro**, consultare anche la [tavola di dettaglio esistente](../mini360/02-pad-regolazione-ina219.png). Le sue quattro posizioni sono ricavate dalla vista del venditore: scritta MINI-360 dritta, freccia a sinistra; **OUT− alto a sinistra, OUT+ basso a sinistra, IN− alto a destra, IN+ basso a destra**. Questa indicazione vale per il modulo DAOKAI identificato nel progetto.

## 2. Come leggere i collegamenti

- **5 V** indica la linea positiva in uscita dal Mini360. Sul montaggio Andrea ha riferito una regolazione di **5,11 V**.
- **USB +5 V / USB−** sono i due fili dell'ingresso USB del caricatore della batteria: il +5 V USB è distinto da Mini360 OUT+. Il rilevatore PC817 non aggiunge un ponte fra USB− e GND della bilancia.
- **GND** indica il negativo comune: tutti i GND della bilancia sono collegati fra loro.
- **3V3** è un'altra uscita dell'ESP32, a 3,3 V. Alimenta **VCC dell'OLED, VCC dell'INA219 e + del buzzer passivo**; i GPIO scambiano segnali a 3,3 V.
- **GPIO35**, per esempio, significa il pin con quel numero sull'ESP32. Individuarlo dalla scritta sulla scheda; non contare i piedini partendo da un angolo.
- **In serie** significa che il filo arriva a una resistenza e riparte dall'altro capo della resistenza.
- **Tra due punti** significa che ciascun capo della resistenza o del condensatore va a uno di quei punti. Non interrompere il filo principale per inserire un condensatore.
- Un **punto comune** collega tutti i fili che vi arrivano. Un incrocio di linee senza collegamento esplicito non indica una giunzione.

I colori delle illustrazioni aiutano a seguire i collegamenti; non certificano il colore dei fili già presenti nella bilancia. Leggere sempre le etichette alle due estremità.

Le bande sulle resistenze disegnate sono illustrative: usare il **valore scritto accanto**, non il colore delle bande. Il PDF generale contiene **una pagina A2 orizzontale**; il PDF della guida contiene sette pagine **A3 orizzontali**, ingrandibili sullo schermo. Per una stampa A4 selezionare “adatta alla pagina”.

### Orientamento fisico dell'ESP32 montato

Usare la vista fornita da Andrea: **3V3 in alto a sinistra, VIN in alto a destra**, con **15 terminali su ciascuna fila**. Il primo terminale della tabella è quello in alto, il quindicesimo quello in basso. Non applicare questo ordine al retro della scheda o a una diversa variante ESP32.

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

Nell'immagine GPIO21 porta anche l'etichetta SDA e GPIO22 l'etichetta SCL. **Per l'INA219 della bilancia usare GPIO32 SDA e GPIO33 SCL**: GPIO21 è la riga R4 della tastiera e GPIO22 pilota il buzzer. RX2/TX2 sono qui GPIO16/GPIO17, usati per HX711 SCK e tastiera C1, non per l'audio. **GPIO36 / VP è riservato all'ingresso presenza USB del PC817**, con pull-up esterna da 10 kΩ a 3V3. GPIO12 ed EN non hanno fili esterni assegnati; RX0/TX0 restano alla seriale.

## 3. Alimentazione: dal pacco batteria ai moduli

Componenti identificati: batteria **NASTIMA BK06-LF60-NATC**, LiFePO4 6,4 V nominali / 6 Ah con caricatore USB-C e BMS interni; Mini360 **DAOKAI B0B82GL5XN**, con chip **MP1482DS**.

| Da | A | Dettaglio |
|---|---|---|
| Batteria, morsetto **+** | Fusibile **F1**, poi **VIN+ dell'INA219** | Replica: **F2A rapido ceramico 5×20 mm**, vicino alla batteria; dati DC dichiarati. Marcatura montata da rilevare |
| INA219 **VIN−** | Ingresso dell'interruttore **S1** | VIN− è ancora sul positivo della batteria |
| Uscita di **S1** | Mini360 **IN+** | Alimentazione positiva del convertitore |
| Batteria, morsetto **−** | Mini360 **IN−** e GND comune | Negativo |
| Mini360 **OUT−** | GND comune | Stesso negativo dei moduli |
| Mini360 **OUT+** | Linea 5 V | Uscita regolata a 5,11 V sul montaggio |
| Linea 5 V | ESP32 **VIN/5V** | Leggere il nome del pin; non collegarla al pin 3V3 |
| Linea 5 V | HX711 **VCC** | Alimentazione della misura |
| ESP32 **3V3** | OLED **VCC**, INA219 **VCC** e **+ del buzzer passivo** | OLED/INA219 dal primo ritaglio; buzzer con pilotaggio BC337-40 |
| Linea **VIN/5V** | IRF9540 **S** | Indicata nel ritaglio audio; non 3V3; vedi tavola 04 |
| USB-C femmina della scocca | USB-C della batteria | Prolunga di ricarica verso il caricatore interno al pacco |

**VIN− dell'INA219 non va collegato a GND.** VIN+ e VIN− sono i due terminali attraverso cui passa il positivo prima di S1 e del Mini360; VCC/GND alimentano invece la piccola elettronica del sensore.

S1 interrompe il positivo fra INA219 e Mini360. La ricarica USB-C interna alla batteria rimane indipendente. L'USB di programmazione dell'ESP32 è una connessione diversa dalla ricarica del pacco; il comportamento quando è collegata al PC è descritto nella [guida Mini360](../mini360/README.md).

### Condensatori del Mini360

| Posizione | Componente | Primo terminale | Secondo terminale |
|---|---|---|---|
| Ingresso, dopo S1 | **220 µF / 16 V**, elettrolitico | **+** a IN+ | **−** a IN− |
| Ingresso, dopo S1 | **100 nF**, ceramico | IN+ | IN− |
| Uscita | **470 µF / 10 V**, elettrolitico | **+** a OUT+ | **−** a OUT− |
| Uscita | **100 nF**, ceramico | OUT+ | OUT− |

Ogni coppia è collegata in parallelo, vicino ai relativi pad del convertitore e con fili corti. Andrea ha riferito il ceramico di uscita più vicino al Mini360, seguito dall'elettrolitico: entrambi arrivano agli stessi OUT+ e OUT−.

Sugli elettrolitici la banda con **−** identifica il terminale negativo. Il ceramico non ha verso. **100 nF = 0,1 µF**, spesso marcato **104**. La [tavola 03 Mini360 esistente](../mini360/03-condensatori-mini360.png) ingrandisce questi quattro collegamenti.

### Coppia aggiuntiva sulla PCB custom

Sulla **PCB custom fra Mini360 ed ESP32** è montata una coppia **aggiuntiva** identificata da Andrea: **47 µF / 16 V elettrolitico + ceramico 104 = 100 nF**, in parallelo fra linea 5 V e GND. È distinta sia dalle due coppie Mini360 sia dai condensatori presso l'ESP32.

| Componente identificato sulla PCB custom | Primo terminale | Secondo terminale |
|---|---|---|
| **47 µF / 16 V**, elettrolitico | **+** alla linea 5 V | **−** a GND |
| **100 nF**, ceramico marcato **104** | Linea 5 V | GND |

### Coppia distinta presso l'ESP32

**Sono montati anche condensatori esterni presso l'ESP32 fra VIN/5V e GND**, distinti dalla coppia PCB custom e dai componenti integrati nella scheda. I terminali sono confermati; i valori fisici di questa coppia presso l'ESP32 non sono stati letti. **Per la sua replica sono consigliati i seguenti valori**, con collegamenti corti vicino alla scheda:

| Componente consigliato per la replica | Primo terminale | Secondo terminale |
|---|---|---|
| **47 µF / 16 V**, elettrolitico | **+** a ESP32 VIN/5V | **−** a GND |
| **100 nF**, ceramico marcato **104** | ESP32 VIN/5V | GND |

I due condensatori sono in parallelo: entrambi arrivano agli stessi VIN/5V e GND. **47 µF / 16 V e 100 nF sono una scelta per la replica, non la lettura dei componenti montati.** La scelta di 47 µF sul VIN a 5 V della DevKit non è una prescrizione del produttore della scheda. Le [linee guida Espressif](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32/pcb-layout-design.html#power-supply) riguardano il disaccoppiamento del chip sulle linee a 3,3 V; il riferimento qui serve alla collocazione vicino all'alimentazione, senza attribuire loro il valore di 47 µF su VIN.

### I cinque gruppi da tenere separati

| Dove | Elettrolitico | Ceramico | Collegamento / stato |
|---|---|---|---|
| Ingresso Mini360 | **220 µF / 16 V** | **104 = 100 nF** | IN+ / IN−, dopo S1 |
| Uscita Mini360 | **470 µF / 10 V** | **104 = 100 nF** | OUT+ / OUT− |
| PCB custom fra Mini360 ed ESP32 | **47 µF / 16 V** | **104 = 100 nF** | Coppia aggiuntiva identificata, linea 5 V / GND |
| Presso ESP32 | Valore fisico ignoto; **47 µF / 16 V consigliato per replica** | Valore fisico ignoto; **104 / 100 nF consigliato per replica** | Coppia distinta presente, VIN/5V / GND |
| Presso DFPlayer | **470 µF / 16 V** | **104 = 100 nF** | Valori identificati; fili e polarità da confermare |

## 4. Cella di carico e HX711

Il modulo HX711 usa la linea 5 V; la cella riceve l'eccitazione dai terminali E+ ed E− del modulo. Non collegare la cella direttamente ai GPIO dell'ESP32.

| Filo o terminale di partenza | Arrivo | Componente intermedio |
|---|---|---|
| Cella, filo **rosso** | HX711 **E+** | Nessuno |
| Cella, filo **nero** | HX711 **E−** | Nessuno |
| Cella, filo **verde** | HX711 **A+** | Nessuno |
| Cella, filo **bianco** | HX711 **A−** | Nessuno |
| Linea 5 V | HX711 **VCC** | Nessuno |
| GND comune | HX711 **GND** | Nessuno |
| ESP32 **GPIO16**, spesso scritto **RX2** | HX711 **SCK / CLK** | **220 Ω in serie**, come indicato in WIRING |
| HX711 **SCK / CLK**, dopo la 220 Ω | GND comune | **100 kΩ pull-down da prevedere nella replica**, vicino all'HX711 |
| HX711 **DOUT / DT** | Punto comune collegato a ESP32 **GPIO35** | **10 kΩ in serie** |
| Punto comune di **GPIO35** | GND comune | **20 kΩ** |

La **100 kΩ fra SCK e GND va prevista nella replica**, dopo la 220 Ω sul lato HX711 e vicino al modulo. Mantiene SCK LOW quando GPIO16 non è pilotato durante avvio/reset: è una scelta di robustezza, non un obbligo del datasheet AVIA né un level shifter. Andrea ritiene che sia già montata: la presenza attuale è **probabile, da confermare**, ma questo riguarda soltanto il rilievo fisico. I collegamenti SCK 220 Ω e DOUT 10 kΩ / 20 kΩ seguono la documentazione HX711 confermata, senza level shifter.

Le ultime due righe formano un partitore: il punto che arriva al GPIO35 è **dopo la 10 kΩ** e collega anche un capo della **20 kΩ**. L'altro capo della 20 kΩ va a GND. La 20 kΩ non è in serie sul filo che arriva all'ESP32.

I colori della cella sono la convenzione presente nei documenti del progetto. Su una cella di altro produttore verificare le funzioni dei quattro fili prima di usare questa corrispondenza. Il modello e l'impostazione fisica **RATE 80 SPS** del modulo vanno verificati per una replica identica; la velocità non viene selezionata da un comando del firmware.

## 5. DFPlayer Mini e accensione audio

Il comando di alimentazione montato usa **P-MOSFET IRF9540** e transistor **NPN 2N3904**, con marcatura **H331** riferita da Andrea. Il 2N3904 è adatto a comandare il gate dell'IRF9540. La marcatura H331 è riportata senza attribuirle produttore o data; l'orientamento fisico dei piedini dell'esemplare resta da verificare.

### Segnali, alimentazione e altoparlante

| Da | A | Componente intermedio |
|---|---|---|
| ESP32 **GPIO4** | DFPlayer **RX** | **1 kΩ in serie** |
| DFPlayer **TX** | ESP32 **GPIO34** | Nessuno; collegamento presente nell'appunto |
| DFPlayer **BUSY** | ESP32 **GPIO39 / VN** | Nessuno nell'appunto |
| IRF9540 **D** | DFPlayer **VCC** | Nessuno |
| DFPlayer **GND** | GND comune | Nessuno |
| DFPlayer **SPK1** | Primo terminale dell'altoparlante | Nessuno |
| DFPlayer **SPK2** | Secondo terminale dell'altoparlante | Nessuno |

L'altoparlante è collegato **fra SPK1 e SPK2**: nessuno dei suoi due fili va a GND. L'impedenza e la potenza dell'altoparlante montato restano da rilevare. I documenti ammettono un altoparlante da 8 Ω, ma questo non identifica quello presente sulla bilancia.

### IRF9540, NPN e resistenze dell'appunto

| Da | A | Componente intermedio |
|---|---|---|
| Linea **VIN/5V** | IRF9540 **S** | Nessuno; confermato nel ritaglio audio, non 3V3 |
| IRF9540 **D** | DFPlayer **VCC** | Nessuno |
| IRF9540 **G** | NPN **C** | Nessuno; stesso punto |
| IRF9540 **G** | IRF9540 **S / linea 5 V** | **100 kΩ** |
| ESP32 **GPIO2** | NPN **B** | **10 kΩ in serie** |
| NPN **B** | GND comune | **100 kΩ** |
| NPN **E** | GND comune | Nessuno |

I **due resistori da 100 kΩ del power-gate audio sono distinti**: uno collega G a S del MOSFET, l'altro collega B del transistor a GND. Sono separati dalla terza 100 kΩ prevista fra SCK HX711 e GND. Il cerchio chiamato **GATE** nell'appunto è lo stesso punto elettrico del piedino G, non un componente da aggiungere.

GPIO2 dà il comando: **HIGH accende l'audio**. Il NPN porta il punto G verso GND e il MOSFET lascia passare l'alimentazione da S a D. La corrente che alimenta il DFPlayer passa attraverso il MOSFET; GPIO2 non alimenta direttamente il modulo.

Prima di saldare, identificare i piedini dei componenti effettivamente usati. Le lettere **G/D/S** e **C/B/E** indicano la funzione; la loro posizione fisica dipende dal componente. Per l'IRF9540 TO-220 documentato, visto dalla faccia con la scritta e con i piedini verso il basso, l'ordine è **G–D–S**; la parte metallica è collegata a D. La sigla del NPN montato è **2N3904 H331**, ma il suo orientamento fisico va ancora verificato. Il [datasheet onsemi 2N3904](https://www.onsemi.com/download/data-sheet/pdf/2n3904-d.pdf) indica **pin 1 = E, 2 = B, 3 = C** per i suoi package TO-92: questo riferimento non attribuisce produttore, package o disposizione dei piedini all'esemplare presente sulla bilancia.

**Condensatori audio:** presso il DFPlayer sono confermati un **elettrolitico da 470 µF / 16 V** e un **ceramico marcato 104, cioè 100 nF**. La quantità complessiva, la polarità, i collegamenti e la posizione dei componenti devono essere rilevati sul circuito; l'appunto fornito non li mostra. Nelle tavole **00 e 04**, i fili tratteggiati dei condensatori fra **VCC audio e GND** sono un riferimento **da confermare**, non la verifica dei collegamenti montati. Il 470 µF / 16 V audio è distinto dal **470 µF / 10 V del Mini360**. Non aggiungere componenti per ricostruire un dettaglio ancora da osservare.

## 6. Tastiera, buzzer e LED

### Tastiera 4 × 2

La tastiera è una matrice con sei collegamenti. Le coppie GPIO sono state rilevate premendo i singoli tasti sul cablaggio montato. **R1-R4 e C1-C2 indicano la matrice logica corrente del firmware**, non l'ordine fisico del connettore né la posizione dei tasti sul frontale.

| Collegamento tastiera | ESP32 | Tasti collegati |
|---|---|---|
| **R1** | **GPIO13** | SKIP, SLEEP |
| **R2** | **GPIO14** | ENTER, CLEAR |
| **R3** | **GPIO19** | WIFI, TOTAL |
| **R4** | **GPIO21** | MODE, TARE |
| **C1** | **GPIO17** | SKIP, ENTER, WIFI, MODE |
| **C2** | **GPIO5** | SLEEP, CLEAR, TOTAL, TARE |

| Riga logica | C1 (GPIO17) | C2 (GPIO5) |
|---|---|---|
| R1 (GPIO13) | SKIP | SLEEP |
| R2 (GPIO14) | ENTER | CLEAR |
| R3 (GPIO19) | WIFI | TOTAL |
| R4 (GPIO21) | MODE | TARE |

Le etichette **R1-R6 del ritaglio di schema precedente non definiscono la mappa corrente**. Non numerare i pin del connettore come fili 1-6 senza un orientamento verificato. Per una replica, usare le coppie GPIO della tabella o rilevare la continuità a tastiera scollegata.

Non collegare 5 V, 3V3 o GND alle sei linee della tastiera: ciascuna arriva soltanto al GPIO assegnato.

### Buzzer e LED

Il **buzzer passivo** è adatto all'alimentazione a **3,3 V** e al circuito di pilotaggio.

| Da | A | Dettaglio |
|---|---|---|
| ESP32 **GPIO22** | **Base (B) BC337-40 NPN** | **220 Ω / ¼ W in serie** |
| BC337-40 **emettitore (E)** | GND comune | Nessuno |
| BC337-40 **collettore (C)** | Buzzer, terminale **−** | Nessuno |
| ESP32 **3V3** | Buzzer, terminale **+** | Alimentazione 3,3 V |
| **1N5819, catodo con fascetta** | Buzzer **+ / 3V3** | Diodo in parallelo al buzzer |
| **1N5819, anodo senza fascetta** | Buzzer **− / collettore** | Altro capo dello stesso diodo |
| ESP32 **GPIO15** | **Anodo del LED**, attraverso la resistenza in serie | **1 kΩ**, identificata nello schema |
| **Catodo del LED** | GND comune | Identificare la polarità del LED |

**In parallelo** significa che il diodo collega gli stessi due terminali del buzzer: su ciascun terminale arrivano il filo del circuito e un capo del diodo. La **220 Ω limita la corrente della base**, non è in serie al buzzer. Il firmware resta invariato: PWM LEDC su GPIO22 attivo HIGH e silenzio LOW.

Il LED montato è un normale **LED rosso del kit Arduino**, con **1 kΩ in serie da GPIO15**, come indicato nel ritaglio di schema. La guida conserva i collegamenti e il comportamento di sleep esistenti.

## 7. Display, misura batteria e presenza USB

### OLED SSD1322, SPI, 256 × 64, 3,12 pollici

| Da | A sul display | Dettaglio |
|---|---|---|
| ESP32 **GPIO18** | **SCK / CLK** | Clock |
| ESP32 **GPIO23** | **MOSI / DIN** | Dati verso il display |
| ESP32 **GPIO25** | **CS** | Selezione display |
| ESP32 **GPIO26** | **DC** | Dati/comandi |
| ESP32 **GPIO27** | **RST / RES** | Reset |
| ESP32 **3V3** | **VCC** | Alimentazione indicata nel primo ritaglio di schema |
| GND comune | **GND** | Negativo comune |

Il modello preciso dell'OLED posseduto non è identificato nei documenti: la famiglia SSD1322 non basta per assegnare un ordine fisico ai sette terminali. Seguire la serigrafia del modulo. Il riferimento KS Display citato nel progetto meccanico è soltanto un esempio d'ingombro, non l'identificazione del display montato.

### INA219 / CJMCU-219

| Da | A sul modulo | Dettaglio |
|---|---|---|
| ESP32 **3V3** | **VCC** | Alimentazione indicata nel primo ritaglio di schema |
| GND comune | **GND** | Negativo comune |
| ESP32 **GPIO32** | **SDA** | Segnale dati |
| ESP32 **GPIO33** | **SCL** | Segnale clock |
| Fusibile F1, lato uscita | **VIN+** | Positivo batteria |
| **VIN−** | S1, poi Mini360 IN+ | Positivo verso il carico, non GND |

I collegamenti VCC/SDA/SCL non trasportano la corrente della bilancia. L'INA219 misura attraverso VIN+/VIN− la corrente diretta al Mini360 e la tensione del ramo batteria. La ricarica USB interna al pacco non passa attraverso questo sensore. L'indicatore USB `charging` usa invece il PC817 e resta indipendente dalle letture INA219.

La documentazione non specifica resistenze I2C esterne aggiuntive. Presenza, valori e collegamenti delle eventuali resistenze già sulla schedina si verificano sul modello effettivo; non sono disegnati come componenti esterni montati.

### Presenza USB: PC817 DIP-4, tavola 07

Il rilevatore PC817 si collega **in parallelo all'ingresso USB del caricatore della batteria**: il percorso di ricarica resta continuo e si aggiunge un ramo per il LED interno del PC817. Individuare **USB +5 V e USB− prima del caricatore**, non l'uscita Mini360 OUT+ / OUT−. Il 5 V USB e il 5 V della bilancia sono linee distinte.

| Da | A | Componente / dettaglio |
|---|---|---|
| **USB +5 V**, ramo parallelo al caricatore | PC817 **pin 1, anodo** | **560 Ω / ¼ W in serie** |
| PC817 **pin 2, catodo** | **USB−** | Lato USB del rilevatore |
| PC817 **pin 3, emettitore** | **GND ESP32** | Lato bilancia |
| PC817 **pin 4, collettore** | **GPIO36 / VP** | Ingresso digitale presenza USB |
| **GPIO36 / VP** | **ESP32 3V3** | **10 kΩ di pull-up esterna** |

I due collegamenti che arrivano a GPIO36 sono sullo stesso punto: il collettore del PC817 e un capo della 10 kΩ. **L'altro capo della 10 kΩ va a 3V3**, non a GND. GPIO36 è input-only e non ha pull-up/pull-down interni, quindi la resistenza esterna è necessaria. Identificare il riferimento del pin 1 sul DIP-4 e seguire il [pinout Sharp](https://global.sharp/products/device/lineup/data/pdf/datasheet/PC817XxNSZ1B_e.pdf): **1 anodo, 2 catodo, 3 emettitore, 4 collettore**.

**Non aggiungere un ponte USB− / GND bilancia nel rilevatore.** Il lato USB accende il LED interno e il lato ESP32 legge il fototransistor; il collegamento fra i due è ottico. A USB nominale 5 V la corrente LED calcolata con 560 Ω è circa **6–7 mA**, non una misura sull'esemplare.

Con ESP32 alimentato, **USB presente = LOW** e **USB assente = HIGH**. Il firmware conferma sia collegamento sia scollegamento dopo **250 ms di livello stabile**. L'icona animata `charging` indica USB presente anche a batteria piena; resta indipendente da INA219 assente, errori I2C o letture invalide/scadute. Il segnale non misura corrente di ricarica, carica residua o fine carica e non disabilita le protezioni per tensione bassa. Il PC817 fa parte del cablaggio per la replica; il montaggio fisico sull'esemplare attuale non è confermato e non sono documentate prove hardware del rilevatore.

## 8. Resistenze e dettagli da distinguere

### Valori del cablaggio di riferimento e della replica

| Valore | Quantità per i collegamenti descritti | Dove |
|---|---:|---|
| **220 Ω** | 2 | Una fra GPIO16 e SCK HX711; una **¼ W** fra GPIO22 e base BC337-40 |
| **10 kΩ** | 1 | DOUT HX711 → punto GPIO35 |
| **20 kΩ** | 1 | Punto GPIO35 → GND |
| **100 kΩ** | 1 | Pull-down SCK HX711 → GND, dopo la 220 Ω e vicino al modulo; da prevedere nella replica |
| **1 kΩ** | 1 | GPIO4 → RX DFPlayer |
| **10 kΩ** | 1 | GPIO2 → B del NPN |
| **100 kΩ** | 1 | G → S del MOSFET |
| **100 kΩ** | 1 | B del NPN → GND |
| **1 kΩ** | 1 | Resistenza in serie al LED su GPIO15, identificata nello schema |
| **560 Ω / ¼ W** | 1 | USB +5 V → pin 1 anodo PC817 |
| **10 kΩ** | 1 | Pull-up GPIO36 / collettore PC817 → ESP32 3V3 |

Questa tabella raccoglie i valori riportati nelle fonti e la 100 kΩ SCK–GND prevista per la replica: servono **tre resistenze da 100 kΩ in totale**, una per l'HX711 e due per l'audio. Non sostituisce l'ispezione dell'esemplare per certificare le sigle e la collocazione fisica dei componenti.

Il pilotaggio del buzzer comprende inoltre **un transistor NPN BC337-40** e **un diodo 1N5819**, collegati come descritto nella sezione 6. Il BC337-40 del buzzer è distinto dal 2N3904 del power-gate DFPlayer.

Il rilevatore presenza USB comprende **un optoisolatore PC817 DIP-4**, oltre alla **560 Ω / ¼ W** e alla **10 kΩ** già elencate nella tabella. Questi tre componenti fanno parte della replica; la loro installazione fisica sulla bilancia attuale non è confermata.

### Componenti da completare e raccomandazioni da verificare

| Componente | Indicazione esistente | Stato per questa guida |
|---|---|---|
| Resistenza BUSY → 3V3 | **10–47 kΩ**, raccomandata soltanto in caso di segnale instabile | Assente dall'appunto audio; presenza fisica da verificare |
| Coppia presso ESP32, distinta dalla PCB custom | Montata fra **VIN/5V e GND**; **47 µF / 16 V + 100 nF (104) consigliati per la replica** | Terminali confermati, valori fisici di questa coppia non letti; la coppia PCB custom 47 µF / 16 V + 104 è invece già identificata |
| Fusibile F1 | Replica: **F2A rapido, ceramico, 5×20 mm**; fili corti in rame almeno **0,5 mm²**, portafusibile isolato e dati DC dichiarati | Marcatura montata, sezione effettiva e spunti da verificare. **T2A** valutabile soltanto per spunti normali confermati; [specifica F1](../../docs/WIRING.md#fusibile-f1-per-la-replica) |

**Diodi di alimentazione:** la nuova bilancia deve ancora essere cablata; **la replica NASTIMA non richiede la coppia di selezione delle sorgenti del progetto Li-ion 2S**. Quel progetto prevedeva due 1N5822 per unire alimentatore 12 V e batteria sull'ingresso del buck; qui il Mini360 riceve dal solo ramo batteria e l'USB-C ricarica il pacco internamente. Le tavole mantengono **S1 → IN+** e **OUT+ → linea 5 V** diretti. Sul DFPlayer non è previsto un diodo aggiuntivo nel circuito base: usare il riferimento **5,00 V** per la nuova replica e verificare VCC dopo l'IRF9540. Il possibile ritorno dall'USB del PC verso il Mini360 resta una verifica separata; vedere [dettaglio tecnico](../../docs/WIRING.md#due-diodi-presso-il-mini360) e [procedura USB](../mini360/README.md#collegamenti-del-ramo-di-potenza). Il diodo 1N5819 del buzzer resta previsto.

## 9. Dati da completare con la bilancia sotto mano

Quando il circuito sarà disponibile, rilevare questi dati e integrare la stessa guida senza cambiare il cablaggio per adattarlo al disegno:

1. **Transistor NPN 2N3904 H331:** faccia con la scritta, orientamento e ordine dei piedini C/B/E e fili effettivamente collegati. La sigla è identificata; il pinout fisico dell'esemplare resta da verificare.
2. **Condensatori presso il DFPlayer:** sono identificati **470 µF / 16 V** per l'elettrolitico e **104 = 100 nF** per il ceramico; rilevare quantità complessiva, polarità, terminali collegati e posizione fisica. Il ritaglio dell'appunto non permette di ricostruire questi dettagli.
3. **Fusibile F1 montato:** rilevare marcatura completa, tipo e portafusibile installati. La replica prevede **F2A rapido ceramico 5×20 mm**; verificare sezione dei fili, dati DC e spunti senza confondere la scelta progettuale con il componente dell'esemplare precedente.
4. **INA219:** modello effettivo ed eventuali resistenze presenti sulla schedina; il VCC da **ESP32 3V3** è già indicato nel primo ritaglio.
5. **Moduli:** foto leggibile delle serigrafie di OLED e HX711 e impostazione RATE dell'HX711. L'ordine fisico dei 30 terminali ESP32 è quello della vista fornita da Andrea; l'OLED è alimentato da **3V3**, mentre il modello e l'ordine fisico dei suoi terminali restano da identificare.
6. **Coppia distinta presso ESP32:** presenza e collegamento fra **VIN/5V e GND** sono confermati; i valori fisici di questa coppia restano da leggere. Per la replica guidata sono consigliati **47 µF / 16 V + 100 nF (104)**. Non confonderla con la coppia aggiuntiva sulla PCB custom, già identificata come **47 µF / 16 V + 104**.
7. **Altri dettagli:** impedenza/potenza altoparlante, presenza fisica della **100 kΩ SCK–GND** sulla bilancia attuale ed eventuale pull-up BUSY. La 100 kΩ SCK–GND è già prevista per la replica; sul montaggio attuale resta probabile, da confermare. La **resistenza LED da 1 kΩ** è già identificata nello schema.
8. **Rilevatore USB PC817:** fa parte del cablaggio per la replica; l'installazione fisica sull'esemplare attuale non è confermata. Non sono documentate misure o prove hardware del rilevatore.
9. **Eventuali diodi sul montaggio precedente:** identificarli soltanto se si vuole ricostruire quell'esemplare. La coppia 1N5822 del progetto Li-ion 2S non è richiesta per la selezione delle sorgenti nella replica NASTIMA e non va aggiunta sulla base del ricordo.

I punti principali del collegamento sono già descritti; questi rilievi completano la **replica fisica identica**. Dove un valore o una posizione manca, la guida lo lascia indicato come da confermare.

## 10. Riferimenti

- [Cablaggio tecnico corrente](../../docs/WIRING.md).
- [README del progetto e pinout firmware](../../README.md#3-pinout-firmware-hx711).
- [Alimentazione Mini360: pad, condensatori e dettagli](../mini360/README.md).
- [GPIO definiti nel firmware](../../firmware/esp32_hx711_serial/config/config_pins.h).
- [Mappa effettiva della tastiera](../../firmware/esp32_hx711_serial/keypad.cpp).
- [Datasheet Vishay IRF9540, piedini del TO-220](https://www.vishay.com/docs/91078/91078.pdf).
- [Datasheet onsemi 2N3904, riferimento tecnico e package del produttore](https://www.onsemi.com/download/data-sheet/pdf/2n3904-d.pdf).
- [Datasheet Sharp PC817, pinout DIP-4 e caratteristiche del componente](https://global.sharp/products/device/lineup/data/pdf/datasheet/PC817XxNSZ1B_e.pdf).

## 11. Sorgenti delle tavole

`draw_wiring.py` genera le tavole di dettaglio 01-06; `draw_pc817.py` genera la tavola 07 con il rilevatore USB. `draw_complete.py` genera la tavola generale con i pin ESP32 nella posizione della mappa fornita. Dopo questi tre generatori, `render.cjs` produce i PNG, `build_pdf.py` compone il PDF dei sette dettagli e `build_complete_pdf.py` esporta il PDF generale A2. Dopo una modifica, rigenerare in quest'ordine e controllare visivamente fili, giunzioni, polarità ed etichette prima di distribuire i PDF. `index.html` consulta gli SVG locali e offre ingrandimento senza dipendenze di rete.
