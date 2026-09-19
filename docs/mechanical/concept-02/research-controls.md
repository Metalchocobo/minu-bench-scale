# Concept 02 - comandi, USB-C e display esistente

Stato: ricerca per il prototipo meccanico, verificata il 7 settembre 2026. Nessun componente acquistato. Le quote di catalogo, le quote progettate e gli ingombri ancora da misurare sono distinti; questi ultimi non costituiscono un attacco produttivo validato.

## 1. Vincoli derivati dal firmware

Il progetto documenta un **OLED SSD1322 SPI da 3,12 pollici, 256 x 64** (`README.md`, elenco hardware), ma non indica produttore, codice del modulo, interassi o profondita del connettore. Si riutilizza il display di Andrea; la distinta non ne prevede l'acquisto.

La tastiera attuale e una matrice passiva **4 righe x 2 colonne**: R1-R4 sui GPIO 17, 5, 13, 14 e C1-C2 sui GPIO 19, 21. Le otto funzioni sono definite in `firmware/esp32_hx711_serial/keypad.cpp`: SKIP, WIFI, SLEEP, CLEAR nella prima colonna; ENTER, TOTAL, MODE, TARE nella seconda. Due tastiere 1 x 4 con un comune indipendente ciascuna possono realizzare la stessa topologia, unendo soltanto le quattro linee dei singoli tasti. Questa e una proposta di cablaggio, da provare sul campione, senza modifica firmware nel presente lavoro. Nessuna alimentazione esterna su righe o colonne.

## 2. Tastiera: otto tasti su due membrane adesive

**Selezione d'acquisto: 2 x BerryBase B-SM4T senza numeri**, produzione Cina. Ingombro dichiarato **70 x 20 x 1 mm**, cavo compreso connettore **88 mm**, cinque poli passo **2,54 mm**. La pagina visitata mostra EUR 1,60 e otto pezzi disponibili; prezzo e giacenza restano da rivedere al momento dell'ordine. Il fissaggio e adesivo e non richiede viti. Le etichette funzionali sono da stampare per il prototipo. L'ordine elettrico dei pin non e pubblicato: si rileva con una prova di continuita prima del collegamento. [BerryBase B-SM4T](https://www.berrybase.de/en/membrane-keypad-4-keys-without-labelling-with-adhesive-layer).

**Alternativa dimensionale e riferimento per la matrice: Adafruit PID 1332.** Ogni membrana misura 69,14 x 20,07 mm; coda e connettore misurano 87,31 x 14,28 mm; connettore femmina a cinque poli, passo 2,54 mm. Il produttore pubblica il pinout: pin 1 comune, pin 2 tasto 2, pin 3 tasto 1, pin 4 tasto 4, pin 5 tasto 3. I quattro tasti sono blu e numerati. La pagina diretta risulta esaurita; un risultato indicizzato riportava una giacenza contraddittoria e non e stato usato come conferma di disponibilita. Lo spessore di questa alternativa non e quotato nella pagina. **Il suo pinout non va trasferito automaticamente alla BerryBase.** [Adafruit 1332](https://www.adafruit.com/product/1332), [pinout ufficiale](https://learn.adafruit.com/matrix-keypad?view=all).

Il riferimento si trova anche dal fornitore cinese **ODSEVEN / Shenzhen Xuan Yao**, che dichiara lo stesso ingombro 69,14 x 20,07 mm. La pagina indica MOQ 10 e fascia USD 0,90-1,24: e una richiesta d'offerta, non una disponibilita confermata per due pezzi. Il codice fornitore e le quote della coda vanno riconfermati. [ODSEVEN su Made-in-China](https://xuanyao.en.made-in-china.com/product/ywxmLVDcCpWX/China-Membrane-1X4-Keypad-Extras-Wholesale.html).

Innesto progettato per entrambi: due sedi lisce **71 x 21 mm**, separate, con bordo e raccordi privi di spigoli taglienti; due passacavo nominali **16 x 6 mm** per far transitare il connettore senza smontarlo. Il posizionamento del passacavo rispetto al punto di uscita della coda e ancora una **quota da campione**. Le foto ufficiali BerryBase e Adafruit mostrano la coda centrata sul bordo lungo superiore, fra i due tasti centrali; la foto del retro Adafruit mostra la giunzione a pochi millimetri dal bordo. La larghezza esterna e lo spessore del connettore BerryBase non sono quotati: il suo passaggio nello slot 16 x 6 resta da verificare sul campione. Non collocare il passacavo al centro dell'intera membrana o sotto i centri di pressione. Il CAD prevede due ponti copricoda **24 x 11 x 2,4 mm**, fissati soltanto all'esterno dell'impronta della membrana con ritagli adesivi spessi 0,13 mm. La loro superficie inferiore lascia libera la curva della coda e non preme i tasti. La membrana e le coperture non attestano una classe IP dell'intera bilancia.

L'helper `controls_cable_details.py` include due nastri con centrolinea curva di 76 mm e connettori terminali lunghi 12 mm, per rappresentare l'ingombro totale di 88 mm pubblicato. Entrambi piegano nel piano YZ, senza curve laterali nel piano del nastro: il tratto inferiore della prima coda e 5 mm sotto quello della seconda, conservando lo sviluppo totale. **La suddivisione 76+12, lo spessore 0,3, la larghezza 14,28, i raggi 2/4 e i corpi dei connettori sono assunzioni CAD da verificare.** I due pettini 1 x 5 alimentano un adattatore cablato passivo verso una terminazione a sei fili sopra il vassoio elettronica. La terminazione non pretende di coincidere con una presa esistente sulla scheda ESP32 posseduta.

Per le coperture e proposto nastro di trasferimento **3M 468MP**, nominalmente 0,13 mm: l'adesione sul PA12 stampato e finito va provata. La scheda 3M riguarda il nastro, non approva questa specifica giunzione; si acquista un piccolo foglio o una striscia convertita per ricavare sei ritagli, con formato e prezzo da fornitore. [Fonte 3M](https://www.3m.com/3M/en_US/p/dc/v100808810/).

L'adattatore e un **cablaggio su misura**, costituito da due rami a cinque conduttori e una terminazione a sei segnali. Materiali da acquistare:

- **Due pettini 1 x 5:** si tagliano da una strip passo 2,54. Una [BerryBase PINH-1X20P](https://www.berrybase.de/en/male-connector-1x-20-pin-rm-2.54-straight), EUR 0,20 e oltre 100 pezzi visualizzati, e sufficiente. La [strip PINH-1X40P](https://www.berrybase.de/en/male-connector-1x-40-pin-rm-2.54-straight) resta un riferimento esaurito. Per acquisto cinese e disponibile [ZHOURI 2.54-1*40, LCSC C2977586](https://www.lcsc.com/product-detail/C2977586.html), MOQ cinque strip: ne viene consumata una, ricavando i due pettini. I dieci contatti metallici nel CAD sono inclusi nei pettini e non vanno ordinati nuovamente.
- **Un housing e sei contatti:** [BerryBase DUPH-1X06](https://www.berrybase.de/en/dupont-housing-1x6-pin), EUR 0,10, e sei [DUPCP-F singoli per AWG22-28](https://www.berrybase.de/en/dupont-crimp-contact-for-cable-awg-22-28-female), EUR 0,05 ciascuno, entrambi indicati disponibili. Sono la stessa famiglia generica proposta dallo stesso venditore; non e una coppia certificata da un fabbricante identificato. Verificare ritenuta del contatto e crimpatura dell'isolamento sul campione. Gli inviluppi dei contatti sono semplificati. Le quattro giunzioni delle righe vanno realizzate a monte: non inserire due conduttori in un crimp non dichiarato per tale uso.
- **Filo per i due rami:** [Adafruit 6182](https://www.adafruit.com/product/6182), nastro separabile a dieci conduttori AWG26, un metro, isolamento silicone e diametro esterno del singolo filo 1,3 mm. Prezzo USD 2,50; 65 pezzi visualizzati. Da un tratto nominale di 200 mm si ricavano dieci fili, riuniti in due rami da cinque; il metro fornito lascia materiale per le giunzioni. Il CAD usa fasci diametro **4,2 mm** come inviluppi di instradamento, non replica del fascio finito. I percorsi nominali sono lunghi 152,339 e 139,017 mm; la lunghezza di taglio si rifinisce sul montaggio.

Cablaggio proposto usando il pinout Adafruit:

| Linea | Membrana A | Membrana B | ESP32 |
|---|---|---|---|
| Comune A | pin 1 | - | C1 / GPIO19 |
| Comune B | - | pin 1 | C2 / GPIO21 |
| R1 | pin 3, SKIP | pin 3, ENTER | GPIO17 |
| R2 | pin 2, WIFI | pin 2, TOTAL | GPIO5 |
| R3 | pin 5, SLEEP | pin 5, MODE | GPIO13 |
| R4 | pin 4, CLEAR | pin 4, TARE | GPIO14 |

Realizzare le giunzioni in un cavetto adattatore a sei fili verso la scheda, con due ingressi a cinque pin; isolare singolarmente saldature e giunzioni prima di riunire il fascio. Verificare a circuito spento continuita e corrispondenza delle otto funzioni prima del collegamento. Il firmware e progettato per l'uso di un tasto alla volta.

## 3. Presa USB-C: connettore commerciale completo

Selezionato **Premier Cable PCM-0726, 0,3 m**, reperibile come **Adafruit 6069**. E una prolunga passiva USB-C femmina da pannello / USB-C maschio interno. Il disegno Adafruit riporta lo stesso codice PCM-0726 del produttore cinese. Include corpo, dado ABS, O-ring e tappo antipolvere con linguetta; la sua funzione nel progetto e portare la presa di ricarica alla parete. [Adafruit 6069](https://www.adafruit.com/product/6069), [acquisto diretto Premier Cable, Cina](https://www.premier-cable.store/products/panel-mount-usb-3-1-type-c-waterproof-cable).

Quote documentate dal produttore: filetto **M12 x 1** lungo **15 mm**, doppio piano **11,40 mm**, flangia **diametro 18 x 6 mm**, retro cilindrico **diametro 11 x 10 mm**. Il corpo e lungo 31 mm, somma di 15 e 16 mm del disegno. Cavo diametro **4,8 mm**, lunghezza libera **300 +/- 15 mm**. L'O-ring e indicato **11 x 1,5 x 14 mm** (interno, sezione, esterno). La pagina Adafruit ammette pareti fino a 13 mm.

La flangia rimane **all'interno**; il tratto filettato attraversa il pannello e riceve dado e tappo all'esterno. Per il prototipo: foro CAD **diametro 12,4 mm** in un pannello spesso **3 mm**, quote scelte dal progetto rispetto al foro nominale diametro 12 del produttore. Prevedere 16 mm interni per il corpo e almeno 25 mm di spazio aggiuntivo per instradare il cavo; quest'ultimo e un ingombro di progetto, non il raggio minimo certificato del cavo. Dado modellato come ingombro diametro 20 x 4 e tappo diametro 18 x 4: **dimensioni provvisorie**, non pubblicate. Usare il dado fornito, senza stampare un dado sostitutivo.

I PDF e il dettaglio quotato sono conservati in `sources/adafruit-6069-drawing.pdf` e `sources/adafruit-6069-dimensions.jpg`. [Disegno completo](https://cdn-shop.adafruit.com/product-files/6069/P6069+PCM-0726_TYPE_C___________F-M_L300MM.pdf), [dettaglio del corpo](https://cdn-shop.adafruit.com/product-files/6069/P6069_____________20240514182020______________%281%29.jpg).

Prezzi letti: Adafruit USD 4,50; Premier USD 16,00 senza IVA, vendita senza MOQ dichiarata. Trasporto e imposte non inclusi nel confronto. La selezione definisce l'interfaccia meccanica, **non il circuito di ricarica**: la prolunga USB deve terminare sull'ingresso del caricatore previsto. Il CAD rappresenta la variante Waveshare, non installata; il cablaggio corrente con batteria USB NASTIMA e Mini360 e descritto in [WIRING](../../WIRING.md). Il caricatore, la gestione CC e la tensione di alimentazione devono appartenere allo schema di alimentazione definito separatamente. Il disegno del cavo mostra la continuita dei contatti CC; la compatibilita effettiva del sistema di ricarica della variante Waveshare resta da provare. Per questa variante proposta nel concept, ingresso USB **5 V / 3 A**, la portata continua della prolunga **non e pubblicata nelle fonti consultate**: va confermata dal fornitore e poi verificata con misura della caduta di tensione e del riscaldamento al carico previsto. La sezione 24 AWG riportata nel disegno non sostituisce la portata nominale dell'assieme di cavo e contatti. Il blocco CAD di terminazione dell'uscita **5 V** indica il punto di collegamento all'elettronica posseduta: non identifica un morsetto commerciale selezionato o gia compatibile.

## 4. Display posseduto: carrier sostituibile

La famiglia e nota, il modulo preciso no. Un esempio plausibile della stessa famiglia e il **KS Display KSE25664-31M**, dichiarato 100,5 x 33,5 x 11,3 mm, area attiva 76,78 x 19,18 mm, sette pin. Serve esclusivamente a impostare un ingombro preliminare; **non identifica il display posseduto**. [KS Display, specifica del modulo](https://ksdisplay.com/mono-oled-display/3.12-inch-oled-display-module-256x64-ssd1322-kse25664-31m/).

Il CAD usa un carrier stampato smontabile con tasca nominale **101,5 x 34,5 mm**, finestra ottica **80 x 22 mm** e vetrino separato **82 x 24 x 1,5 mm**. La geometria del carrier e una scelta del progetto. Quattro viti M3 e dadi lo fissano alla scocca; due traversini con altre quattro viti M3 trattengono posteriormente l'inviluppo del display. Le zone reali di appoggio, la posizione della superficie attiva, l'altezza PCB, i componenti sul retro e l'uscita del connettore restano **da misurare prima della stampa funzionale**. Non usare interassi inventati come se provenissero dal display di Andrea e non applicare pressione sul vetro.

La BOM non deve confondere questi punti: display gia posseduto, carrier stampato da produrre, minuteria del carrier da acquistare. Le viti del display non possono essere confermate finche non si identifica il suo PCB. Un fissaggio a clip regolabili e possibile solo sui bordi meccanicamente liberi del PCB, che al momento non sono noti.

## 5. Minuteria pertinente ai controlli

| Voce | Quantita proposta | Stato |
|---|---:|---|
| Membrane 1 x 4 | 2 | Acquisto, quote esterne pubblicate |
| Connettori maschio 1 x 5 passo 2,54 | 2 | Da acquistare per BerryBase; inclusi nell'alternativa Adafruit |
| Cablaggio adattatore tastiera 2 x 5 verso 1 x 6 | 1 | Da assemblare e collaudare |
| Ponti copricoda | 2 | Stampa, ingombro 24 x 11 x 2,4 |
| Ritagli adesivi copricoda 3M468MP | 6 | Due 24 x 2 e quattro 2 x 6, spessore 0,13 |
| Housing 1 x 6 e contatti compatibili | 1 + 6 | Terminazione del cablaggio adattatore |
| Presa PCM-0726 completa | 1 | Corpo, dado, O-ring e tappo compresi |
| Carrier display | 1 | Stampa, interfaccia modulo da confermare |
| Viti M3 carrier verso scocca | 4 | Svasate M3 x 10 con dadi |
| Viti M3 traversini posteriori | 4 | Cilindriche M3 x 10 con dadi |
| Viti/distanziatori modulo OLED verso carrier | da definire | Dipendono dal display posseduto, nessuna quota inventata |

Le fonti alternative non sono automaticamente intercambiabili. Restano da chiudere soltanto le quote esplicitamente segnate come provvisorie, i cablaggi e le prove sull'assieme; questa ricerca non certifica resistenza, grado IP o accuratezza della bilancia.
