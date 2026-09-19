# Alimentazione montata: batteria USB-C e Mini360 DAOKAI

Configurazione montata con batteria USB-C NASTIMA e convertitore DAOKAI, Amazon ASIN **B0B82GL5XN**, con integrato **MP1482DS identificato nella foto dell'esemplare**. Andrea conferma cablaggio completato e primo funzionamento apparentemente regolare il **18 settembre 2026**. Uscita regolata a **5,11 V**, valore ottenuto e riferito da Andrea; il riferimento nominale resta una linea a 5 V. Nessuna modifica o caricamento del firmware eseguiti per questo montaggio.

Il task di cablaggio e documentazione è completato. La conferma di funzionamento è una prova pratica iniziale; non sono riportate misure di autonomia, stabilità con Wi-Fi/audio, temperatura, transitori USB o recupero dopo intervento del BMS.

## Tavole

- [1 — Alimentazione, positivo/GND, fusibile e condensatori (PNG)](./01-alimentazione-mini360.png) · [SVG ingrandibile](./01-alimentazione-mini360.svg)
- [2 — Pad reali del retro, regolazione e INA219 (PNG)](./02-pad-regolazione-ina219.png) · [SVG ingrandibile](./02-pad-regolazione-ina219.svg)
- [3 — Condensatori sui morsetti di ingresso e uscita (PNG)](./03-condensatori-mini360.png) · [SVG ingrandibile](./03-condensatori-mini360.svg)

Le tre tavole rappresentano gli stessi componenti. I blocchi delle tavole 1 e 3 sono funzionali, non viste fisiche dei piedini. La tavola 2 riproduce l'orientamento del **retro** mostrato nell'immagine originale del venditore: scritta MINI-360 dritta, freccia a sinistra; OUT− in alto a sinistra, OUT+ in basso a sinistra, IN− in alto a destra, IN+ in basso a destra. La tavola 3 ingrandisce soltanto i collegamenti dei quattro condensatori.

## Batteria e ricarica

La batteria USB scelta per il montaggio è la NASTIMA **BK06-LF60-NATC**, ASIN **B0FDB3VWDF**, LiFePO4 **6,4 V nominali / 6 Ah**, con BMS e caricatore USB-C interni. «6 V» è la denominazione commerciale del pacco; la configurazione montata non usa la precedente SLA con CTK3S.

La ricarica usa l'ingresso USB-C del pacco attraverso il collegamento femmina da scocca → maschio interno. Il limite dichiarato dell'ingresso è **5 V / 1,5 A (7,5 W)**; un alimentatore con maggiore potenza nominale non aumenta questo limite. Il ramo di potenza esterno usa i morsetti del pacco. Non applicare direttamente i 5 V USB ai morsetti batteria. La prova iniziale riferita non documenta separatamente il funzionamento durante ricarica o l'avvio dopo stacco del BMS.

## Collegamenti del ramo di potenza

| Da | A |
|---|---|
| Morsetto positivo batteria | F1 vicino alla batteria → VIN+ INA219 |
| VIN− INA219 | Interruttore semplice S1 → IN+ Mini360 e positivo di C1/C2 |
| Morsetto negativo batteria | IN− Mini360 e negativo comune |
| OUT+ Mini360 | Linea 5 V di tutti i carichi |
| OUT− Mini360 | Negativo comune/GND |
| Linea 5 V | ESP32 VIN/5V, HX711 VCC, OLED compatibile 5 V e circuito audio esistente |
| GND comune | ESP32, HX711, OLED, INA219 e circuito audio |
| VCC INA219 | 3V3 ESP32 |
| SDA INA219 | GPIO32 ESP32 |
| SCL INA219 | GPIO33 ESP32 |

**VIN− dell'INA219 è ancora sul positivo; non è GND.** Il suo shunt è prima del buck, così la tensione misurata resta quella del ramo batteria. Il circuito audio conserva il power-gate esistente: GPIO2 non alimenta direttamente il DFPlayer.

**S1 è l'interruttore semplice ON/OFF, posto sul positivo dopo VIN− dell'INA219 e prima di IN+ del Mini360 e di C1/C2.** In OFF interrompe l'alimentazione del buck dalla batteria. OUT+ rimane collegato alla linea 5 V dell'ESP32 e delle periferiche; la massa resta comune. Gli ingressi di misura dell'INA219 restano sul ramo batteria anche quando il suo VCC è spento: il datasheet TI ammette questa condizione (§8.3.1). La ricarica USB interna del pacco rimane indipendente da S1. I contatti di S1 devono avere una portata DC adatta alla corrente del ramo batteria.

| Posizione di un interruttore semplice | Effetto in OFF |
|---|---|
| Prima del Mini360, come disegnato | Toglie il consumo del buck dalla batteria; lascia la sua uscita collegata all'ESP32. |
| Dopo il Mini360, sul positivo di tutti i carichi | Separa il buck dalla linea ESP32/USB; lascia il buck alimentato dalla batteria. |

**USB del PC collegata all'ESP32:** se i 5 V USB raggiungono il pin VIN/5V della scheda, raggiungono anche OUT+ del Mini360. Con S1 OFF possono rialimentare internamente il convertitore e il suo ingresso; S1 aperto impedisce a questo percorso di raggiungere il positivo della batteria. Il circuito USB/VIN dell'esatta scheda ESP32 e il comportamento del Mini360 in questa condizione non sono stati verificati. OFF, da solo, non garantisce quindi isolamento dall'USB del PC. Con S1 ON, l'interruttore non separa le due sorgenti di alimentazione.

Applicare 5 V all'uscita non costituisce di per sé una sovratensione su un'uscita regolata a 5 V. L'eventuale ritorno di corrente non implica automaticamente un danno: l'effetto dipende dal circuito e dalla corrente. Per isolare il Mini360 durante la programmazione senza dipendere dalle protezioni non verificate, occorre separare fisicamente OUT+ dalla linea 5 V dei carichi, a circuito disalimentato. Il semplice cablaggio disegnato non realizza tale separazione automaticamente.

L'USB di ricarica del pacco è un collegamento diverso dall'USB del PC sull'ESP32: entra nel caricatore interno della batteria. In OFF viene eliminato l'assorbimento del Mini360 dalla batteria; restano le funzioni interne al pacco (BMS e caricatore). Non si dichiara quindi consumo totale nullo della batteria.

F1 **T2 A** è il valore progettuale proposto per cablaggio corto di potenza in rame di almeno **0,5 mm²**; valore effettivamente montato e spunti non sono stati riferiti separatamente. Usare portafusibile isolato e rating DC idoneo. Non è una misura della corrente della bilancia o una garanzia di protezione dei semiconduttori del buck.

Condensatori ripresi dalle indicazioni già presenti nel README del progetto:

| Lato del Mini360 | Componente | Collegamenti |
|---|---|---|
| **Ingresso** | C1, elettrolitico **220 µF / 16 V** | + a IN+, − a IN− |
| **Ingresso** | C2, ceramico **100 nF** | Tra IN+ e IN−, in parallelo a C1; senza polarità |
| **Uscita** | C3, elettrolitico **470 µF / 10 V** | + a OUT+, − a OUT− |
| **Uscita** | C4, ceramico **100 nF** | Tra OUT+ e OUT−, in parallelo a C3; senza polarità |

**Saldare ogni coppia il più vicino possibile ai rispettivi pad del Mini360**, direttamente sulla schedina se c'è spazio oppure subito accanto con collegamenti corti. In particolare, il ceramico deve avere un percorso corto fra positivo e negativo. Le distanze nelle tavole servono solo alla leggibilità: non indicano di collocare C1/C2 presso l'INA219 o C3/C4 presso l'audio. I condensatori sono in parallelo all'alimentazione, mai in serie sul filo positivo. C1/C2 sono entrambi dopo S1.

100 nF = 0,1 µF, spesso marcato **104**. Sugli elettrolitici la banda con il simbolo − indica il terminale negativo. Non rimuovere i condensatori già montati sul Mini360. Riutilizzare gli eventuali condensatori esterni già presenti se idonei, senza duplicarli automaticamente.

Sul montaggio Andrea ha posizionato **prima il ceramico di uscita, più vicino al Mini360, poi l'elettrolitico**. È coerente con lo schema: entrambi sono collegati agli stessi OUT+ e OUT−, in parallelo.

Vicino all'ESP32 rimangono i condensatori locali già previsti: **10–47 µF + 100 nF** fra VIN/5V e GND. Servono nel punto di alimentazione dell'ESP32 e non sono ripetuti come nuovi componenti nelle tavole.

## Regolazione del convertitore

**Regolazione del montaggio: 5,11 V riferiti da Andrea.** Non è stato riferito un collaudo strumentale delle variazioni a vuoto, con Wi-Fi/audio e durante le commutazioni. La procedura seguente mantiene il riferimento nominale di 5,00 V per una futura regolazione; non richiede di modificare il montaggio funzionante per inseguire il centesimo.

1. Con PC scollegato, lasciare il filo **OUT+ → linea 5 V dei carichi fisicamente staccato e isolato**. Il carico non deve essere collegato al buck ancora da regolare.
2. Collegare correttamente IN+ e IN− tramite il ramo F1 → INA219 → S1, poi portare S1 su ON per alimentare il convertitore.
3. Misurare direttamente OUT+ rispetto a OUT− con tester in volt DC e ruotare a piccoli passi il trimmer fino a **5,00 V**.
4. Portare S1 su OFF e scollegare la batteria prima di completare la saldatura del filo OUT+ verso i carichi. Nessuna saldatura a circuito alimentato.
5. Ricollegare la batteria, chiudere S1 e verificare la tensione anche con Wi-Fi e audio attivi. Se la tensione scende perché manca margine in ingresso, non compensare aumentando arbitrariamente il trimmer.
6. Fissare il modulo con supporto isolante; nessun pad o saldatura deve toccare la scocca metallica.

È un **buck**, non un buck-boost: non può mantenere 5 V regolati quando l'ingresso si avvicina a 5 V. Il chip **MP1482DS**, leggibile sui moduli fotografati, ha un ingresso operativo di **4,75–18 V** e una corrente d'uscita nominale dell'integrato di **2 A**. La minima tensione d'ingresso non garantisce 5 V in uscita a quel valore. Le specifiche commerciali generiche 23 V / 3 A non qualificano questo esemplare; i 2 A del chip non certificano la corrente continua sostenibile dalla schedina nel vano. Restano da misurare margine d'ingresso e comportamento termico con il carico reale. I limiti del chip non sostituiscono quelli degli altri componenti, inclusi i condensatori esterni.

## Monitoraggio quando la ricarica è interna al pacco

Con la NASTIMA, la corrente USB entra nel pacco internamente e non attraversa l'INA219 posto sui morsetti di uscita: questo cablaggio misura **la corrente diretta al buck**, non la corrente netta delle celle. Il firmware ricava `charging` da corrente negativa; il countdown parte a ≤ 5,80 V dopo 5 s e dura 120 s, mentre il light-sleep anticipato interviene a ≤ 5,70 V per 3 s. **Le tacche usano fasce indicative LiFePO4, non percentuali calibrate, e l'icona charging non certifica la ricarica USB.** Avvisi sonori e protezioni sono indipendenti dalla mappa delle tacche. Il light-sleep non scollega fisicamente il Mini360. Soglie e comportamento corrente sono nel [README principale](../../README.md#8-monitoraggio-batteria-ina219).

## Analisi e verifica del ritorno dall'USB del PC

**Identificazione:** nella foto fornita da Andrea si legge **MP1482DS** sui moduli centrale e destro. La marcatura corrisponde a quella prevista dal datasheet MPS. La foto mostra il fronte, con integrato, induttore, trimmer e passivi; non costituisce una verifica della continuità completa delle piste o del lato posteriore.

Nel datasheet originale MP1482, pagina 2, i massimi assoluti sono **21 V sul nodo SW** e **20 V su VIN**. Applicare 5 V all'uscita non dimostra di per sé il superamento di questi limiti. Il limite massimo operativo di VIN è invece 18 V: un massimo assoluto non è una tensione d'impiego.

Nel circuito applicativo MPS, OUT è collegato a SW attraverso l'induttore. La struttura del MOSFET superiore permette il percorso `OUT+ → induttore → SW → diodo interno → IN+`. IN aperto non è fissato a zero: può caricarsi dal lato OUT. La soglia di avvio per sottotensione è **4,10 V tipici** (3,80–4,40 V), pagina 3; il circuito applicativo prevede una resistenza di pull-up da 100 kΩ su EN. Da questi dati si deduce che una sorgente esterna sull'uscita può anche riattivare il regolatore. È un'inferenza circuitale, non una misura sull'esemplare. Il datasheet non garantisce questa modalità né consente di concludere che un guasto sia inevitabile. La limitazione di corrente del MOSFET basso non equivale all'isolamento OUT/IN.

**Rilevanza pratica:** la sola rialimentazione del Mini360 non è un difetto. Se tensioni, corrente e temperatura rimangono accettabili, il suo assorbimento dal PC può essere irrilevante per l'uso previsto. Va distinto questo caso dal trasferimento attivo di energia verso IN. La nota TI SLYT689, sezione 6, descrive quest'ultimo nei buck in PWM forzato quando l'uscita esterna supera il riferimento e il lato ingresso non assorbe l'energia. L'analisi del controllo MP1482 rende questo comportamento pertinente da verificare, senza dimostrarlo sull'esemplare.

Le fonti consultate includono una prova strumentale del DSN-MINI-360 con MP1482DS pubblicata da Gough: misura un assorbimento a vuoto di circa 3–12,5 mA nelle condizioni provate, alimentando IN. Non misura il ritorno alimentando OUT, quindi quei numeri non sono una previsione dell'assorbimento dall'USB. Sul forum ufficiale MPS, una discussione sul Nano Every riporta 4,65 V applicati a OUT e 5,46 V misurati a IN: è un esempio concreto di rialimentazione attiva, ma riguarda MPM3610 e non MP1482. Nelle fonti pubbliche consultate non è emersa una prova specifica di MP1482 con OUT a 5 V e IN aperto che ne quantifichi corrente e transitori.

**Prima misura, sul solo ESP32:**

1. Con tutte le sorgenti scollegate, separare la batteria e il filo OUT+ del buck dalla linea 5 V della scheda. Lasciare l'ESP32 senza altre alimentazioni esterne.
2. Alimentare l'ESP32 soltanto dalla sua USB.
3. Usare il multimetro in volt DC, puntale nero su GND e rosso sul pin VIN/5V, senza cortocircuitare pin adiacenti.
4. Una lettura prossima a 5 V dimostra la presenza della tensione USB su quel pin nelle condizioni della prova. Collegare il buck a quel pin esporrebbe anche OUT+ a tale tensione. La misura non caratterizza il buck né i transitori; una lettura nulla non certifica da sola ogni stato della scheda.

Per caratterizzare il Mini360 servono modulo isolato, sorgente da banco con limitazione di corrente e osservazione delle tensioni e della corrente, incluse le transizioni. La sola limitazione di corrente non esclude una sovratensione su IN generata da un convertitore sincrono. Una prova di accensione prolungata dall'uscita, usando direttamente il PC, non è un criterio di validazione. Il multimetro può rilevare il ritorno a regime; per escludere picchi e commutazioni indesiderate serve anche un oscilloscopio. Non sono riportate queste misure specifiche; la prima prova positiva della bilancia non viene estesa a una caratterizzazione del ritorno USB.

## Fonti e riproduzione

- [Prodotto indicato da Andrea](https://www.amazon.it/dp/B0B82GL5XN).
- [Immagine originale DAOKAI con i quattro pad e la vista del retro](https://m.media-amazon.com/images/I/71XIQphK08L._US500_.jpg).
- [Descrizione DAOKAI e galleria riportate dal distributore](https://snapklik.com/en-ca/product/daokai-10pcs-mini-360-power-supply-buck-module-dc-to-dc-adjustable-buck-converter-4-75-23v-to-1-0-17v-ultra-small-step-down-voltage-regulator-3a-for-aeronautical-model/0PSL4PO7K3085).
- [Manuale originale della NASTIMA USB-C](https://m.media-amazon.com/images/I/91H3v%2BEZy2L.pdf).
- [Texas Instruments: effetti generali dell'alimentazione applicata all'uscita di un buck spento](https://www.ti.com/document-viewer/lit/html/SSZTAG7/GUID-A966A819-DB0C-4D5D-AA2C-345810AA0674). Non è una caratterizzazione del Mini360 specifico.
- [Foto dell'esemplare fornita da Andrea](../../.codex-remote-attachments/01a0916e-df51-71a3-a685-71b7142ab0dd/86ad795f-8b1d-4ca9-9806-ef48b400f0ee/1-Photo-1.jpg): marcatura MP1482DS.
- [Datasheet originale MPS MP1482, revisione 1.31](https://www.monolithicpower.com/en/documentview/productdocument/index/version/2/document_type/Datasheet/lang/en/sku/MP1482/document_id/344/), pagine 2–6 e circuito applicativo a pagina 11.
- [Gough: prova strumentale DSN-MINI-360 con MP1482DS](https://goughlui.com/2025/09/30/tested-dsn-mini-360-mp1482-based-2a-buck-converter-module-4-5-18v-in/): funzionamento normale, non prova del ritorno da OUT.
- [MPS: MPM3610 su Arduino Nano Every alimentato da USB](https://forum.monolithicpower.com/t/mpm3610-in-arduino-nano-every/1284): caso analogo con integrato diverso.
- [TI SLYT689: alimentare l'uscita senza alimentazione all'ingresso](https://www.ti.com/lit/an/slyt689/slyt689.pdf): sezioni 2, 3 e 6 distinguono assorbimento e funzionamento inverso attivo.
- [Pinout attuale della bilancia](../../docs/WIRING.md).
- [Datasheet Texas Instruments INA219](https://www.ti.com/lit/ds/symlink/ina219.pdf), §8.3.1: tensione sul bus ammessa anche con alimentazione VCC spenta.

Rigenerare con `python artifacts/mini360/draw_wiring.py`, poi `node artifacts/mini360/render.cjs`. Il renderer usa Sharp dal runtime locale configurato; aggiornare il percorso se il runtime cambia.
