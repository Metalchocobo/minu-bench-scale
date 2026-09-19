# Componenti della struttura - Concept 02

Ricerca del 7 settembre 2026. Scopo: definire interfacce reali per un prototipo con **20 kg di carico utile**, piatto plastico e interno libero da nervature strutturali. Nessun ordine o preventivo trasmesso.

## 1. Selezione acquistabile

| Componente | Scelta | Quantita | Fonte e stato |
|---|---|---:|---|
| Cella single point | **Zemic L6D-C3-30kg-0.4B** | 1 | [Essmann, articolo 11250496](https://www.essmann-shop.com/en/single-point-load-cell-zemic-l6d-c3-30kg-0.4b/11250496): pagina consultata, EUR 56,60 esclusi IVA e trasporto, disponibilita indicata 2-5 giorni. |
| Piedi regolabili antiscivolo | **Elesa LX.30-SW17-AS-M6x22**, codice **531051** | 4 | [Elesa+Ganter](https://www.elesa-ganter.nl/p/lx-30-sw17-as-m6x22/127394/): prodotto esatto indicizzato; apertura diretta 403, prezzo e stock non confermati. Quote verificate sul PDF del produttore. |
| Viti cella, lato fisso | M6 x 25, ISO 4762, acciaio 8.8 zincato, **Keller 240912625** | 2 | [Keller & Kalmbach](https://www.keller-kalmbach.com/products/fasteners/screws/hexagon-socket-cap-screws/iso-4762--din-912-cap-screw/p/240912625): prodotto e quote verificati; vendita B2B, prezzo con account. |
| Viti cella, lato mobile | M6 x 45, **DIN 7991**, acciaio 10.9 zincato, **Accu SSK-M6-45-10.9-Z** | 2 | [Accu](https://www.accu.co.uk/countersunk-socket-head-screws/495060-SSK-M6-45-10-9-Z): prodotto e quote verificati, confezione da un pezzo; prezzo non rilevato. |
| Rondelle lato fisso | M6 ISO 7089, acciaio 200 HV | 2 | [Westfield Fasteners, variante 200 HV](https://www.westfieldfasteners.co.uk/BZP-Washer-M6.html), quote 6,4 x 12 x 1,6 mm. |
| Controdadi dei piedi | M6 DIN 934 acciaio zincato, **Essentra 051060040952 / 10180143** | 4 | [Essentra](https://www.essentracomponents.com/en-gb/p/standard-hex-nuts-metal/051060040952), chiave 10, altezza 5 mm; confezione industriale da 250. Si puo acquistare lo stesso standard da ferramenta in quantita minore. |
| Fermi di sovraccarico | M6 x 60 DIN 913, acciaio 45H, **RW Products D913V060060** | 4 | [RW Products](https://www.rwproducts.nl/din-913-stelschroef-binnenzeskant-met-afschuining-elvz-m6x60), chiave 3, passo 1; disponibilita dichiarata presso fornitore 2-5 giorni. |
| Controdadi dei fermi | M6 DIN 934, stesso articolo sopra | 4 | Stessa fonte Essentra; in totale otto dadi M6 tra piedi e fermi. |

Le quantita riguardano questo sottogruppo. La distinta generale del CAD comprende anche viti di scocca, piatto, elettronica e connettori.

## 2. Interfaccia della cella

Fonte primaria: **Zemic, L6D Rev12, dicembre 2022**, copia del produttore fornita da Essmann: [PDF online](https://www.essmann-shop.com/media/eb/a6/ea/1752248218/L6D_Datasheet.pdf?ts=1752248218), [copia locale](sources/Zemic-L6D-Rev12.pdf). Pagina 3 renderizzata e letta visivamente.

- Corpo **130 x 30 x 22 mm**; quattro filetti passanti **M6-6H** su rettangolo **106 x 15 mm**; centri a 12 mm dalle estremita longitudinali.
- Zone di appoggio terminali **25 x 30 mm**. Lato cavo fisso; lato freccia carico mobile. Riservare l'uscita cavo sia alta sia a mezza altezza.
- Piattaforma massima dichiarata **250 x 350 mm**. **Il PDF non associa le due dimensioni agli assi della cella**: l'orientamento del piatto 280 x 200 rispetto alla cella resta da confermare con Zemic.
- Portata nominale 30 kg, deflessione dichiarata 0,6-0,8 mm per questa fascia. Non usare il sovraccarico ammissibile come portata operativa.
- Il produttore indica viti M6 classe 8.8 lubrificate. La tabella di coppia distingue `<30 kg` e `>30 kg`, lasciando ambiguo il caso esatto **30 kg**. Confermare coppia e approvazione del fissaggio mobile svasato 10.9 prima del serraggio definitivo.

**Una divisione visualizzata di 1 g non equivale ad accuratezza di 1 g.** Il solo errore combinato C3 dichiarato, 0,020% del fondo scala, equivale qui a 6 g; a questo si aggiungono gli effetti dell'assemblaggio. La cella e il sistema vanno verificati con masse note e carichi eccentrici.

## 3. Piatto e supporti: produzione su disegno

La ricerca non ha individuato un telaio commerciale economico con disegno quotato completo e interfacce comprovate per questa cella. I bracket generici trovati su marketplace non vengono dichiarati compatibili. Non basta il testo "30 kg" per inserirli nel CAD come componenti pronti al montaggio.

La soluzione proposta mantiene **piatto esterno e scocca in plastica**, con quattro semplici parti in alluminio tagliato/lavorato. Il carico segue: piatto -> carrier mobile -> distanziale mobile -> cella -> distanziale fisso -> basamento -> piedi.

| Parte su misura | Dimensioni di progetto iniziali | Lavorazione |
|---|---|---|
| Basamento liscio | 300 x 320 x 6 mm | Alluminio 6061-T6, contorno raccordato, foratura e filetti secondo CAD. La superficie superiore resta piana salvo attacchi funzionali. |
| Carrier del piatto | 276 x 196 x 8 mm | Alluminio 6061-T6, fori svasati per fissaggio alla cella, fissaggi del piatto plastico; sbalzo plastico perimetrale nominale 2 mm. |
| Distanziale fisso | 25 x 30 x 6 mm | Alluminio, due passaggi viti sul passo 15 mm. |
| Distanziale mobile | 25 x 30 x 22 mm | Alluminio, due passaggi viti sul passo 15 mm. |
| Copertura/piatto plastico | Superficie 280 x 200 mm, pelle nominale 4 mm | PA12 MJF/SLS; geometria di aggancio e sede inferiore definite nel CAD. |

Queste dimensioni sono **scelte progettuali, non dimensioni di prodotti a catalogo**. La geometria effettiva da fabbricare e la sua distinta vengono generate dal CAD finale. L'altezza dei distanziali lascia libera la parte deformabile della cella.

Fornitori concreti: [JLCCNC - fresatura CNC](https://jlccnc.com/) per basamento, carrier e distanziali; [JLC3DP - stampa MJF/SLS](https://jlc3dp.com/) per piatto e scocca. Sono servizi cinesi di produzione su file, **non ricambi standard**. Il prezzo richiede caricamento dei file e conferma delle lavorazioni. Per una produzione locale gli stessi STEP e disegni possono essere forniti a un'officina CNC.

## 4. Innesti e quote ancora da validare

- Piedi: la scheda [Elesa LX, novembre 2025](https://www.elesa.com/siteassets/PDF/PDF_EN/LX.pdf), pagina 3, indica D30, corpo l2=12, L35 e L1=36 con pattino SBR. Quindi l'inviluppo inferiore con pattino e circa 13 mm. Una diversa edizione USA riporta stud piu corto di 1 mm: nel CAD si riserva l'inviluppo maggiore e si verifica il campione.
- Due viti fisse M6x25 attraversano rondella 1,6 + base 6 + distanziale 6: ingaggio nominale nella cella **11,4 mm**. Due svasate M6x45 attraversano carrier 8 + distanziale 22: ingaggio **15 mm**. Le svasate si misurano testa compresa; non aggiungere una rondella sotto la testa conica.
- I quattro piedi si regolano nei filetti **M6x1 del basamento**, poi si bloccano con i dadi superiori. Un solo dado superiore con foro libero non consentirebbe di regolare la quota sotto carico. I fermi M6x60 devono impegnare tutti i 6 mm del basamento.
- Svasata selezionata: DIN 7991, testa D12, altezza 3,3, cono 90 gradi, chiave 4. Non sostituirla con un generico ISO 10642 senza ricontrollare la testa e la sede.
- I fermi di sovraccarico sono parti del basamento, normalmente separati dal carrier. La loro regolazione richiede la deflessione reale della cella montata e le tolleranze dell'insieme; un gioco disegnato non prova che i fermi intervengano al carico corretto.
- 20 kg e l'obiettivo di carico **utile**. Il peso del sottogruppo mobile va sottratto al margine della cella da 30 kg. La resistenza complessiva, il creep, l'assenza di contatti parassiti e la stabilita sono verifiche del prototipo completo.

## 5. Evidenza disponibile

Il file [component-specs-load.json](component-specs-load.json) distingue quote del produttore (`CERTAIN`), scelte di progetto (`DESIGNED`) e dati mancanti o da verificare (`UNCONFIRMED`). Sono conservati i due PDF originali e le pagine quotate renderizzate sotto `sources/`. Nessuna modifica a firmware, pinout o hardware installato.

