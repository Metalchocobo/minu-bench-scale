# Note di lavorazione CNC - Concept 02

**Tutte le quote sono nominali, in millimetri.** Queste note completano i quattro STEP metallici: il prototipo richiede conferma delle tolleranze e delle giunzioni della cella prima dell'ordine. Materiale proposto: alluminio 6061-T6, da confermare nella richiesta di lavorazione.

## 1. Interpretazione dei file

I fori filettati sono rappresentati nello STEP come cilindri al **diametro nominale del filetto**, senza elica. Un cilindro D3 contrassegnato M3 non va fabbricato semplicemente con una punta da 3; lo stesso vale per D6/M6.

| Lavorazione richiesta | Preforo indicativo per maschiatura ad asportazione | Filetto finale |
|---|---:|---|
| M3 passante | D2,5 | M3 x 0,5 - 6H |
| M6 passante | D5,0 | M6 x 1 - 6H |
| Passaggio libero M3 | D3,4 | Nessun filetto |
| Passaggio libero M6 | D6,4 | Nessun filetto |

I prefori per maschiatura a deformazione possono essere diversi e vanno definiti dall'officina per il proprio utensile. Riferimento del produttore: [TR Fastenings - tapping sizes and clearance holes](https://www.trfastenings.com/Knowledge-Base/Engineering-Data/tapping-sizes-and-clearance-holes). **Le indicazioni di filetto in queste note prevalgono sul diametro cilindrico semplificato del modello.**

Origine delle coordinate globali: centro del basamento in pianta. X e la larghezza; Y positivo e il retro; Z e verso l'alto. Gli STEP conservano la posizione dell'assieme. Nelle tabelle locali di distanziali e carrier viene esplicitata anche l'origine del singolo pezzo.

Le superfici di appoggio cella devono essere piane, pulite e prive di bave. Non introdurre conicita o vernice sulle due superfici di serraggio. La scelta di tolleranze per planarita, parallelismo e posizione va concordata con officina e fornitore della cella; la sola precisione di stampa della scocca non definisce quella dei fissaggi metallici.

## 2. Basamento

File: `step/base_aluminium.step`. Un pezzo, **300 x 320 x 6**, angoli R18, Z13..19. Tutti i fori attraversano i 6 mm di spessore. Sono presenti **30 fori**, ripartiti come segue.

| Gruppo | Quantita | Coordinate globali XY | Lavorazione finale |
|---|---:|---|---|
| Chiusura scocca laterale | 6 | X=-142,+142; per ciascun X, Y=-118,-10,+105 | D3,4 passante |
| Chiusura scocca frontale | 2 | (-88,-152), (+88,-152) | D3,4 passante |
| Cella lato fisso | 2 | (-106;42,5), (-106;57,5) | D6,4 passante |
| Piedi regolabili | 4 | (-132,-142), (-132,+142), (+132,-142), (+132,+142) | **M6x1-6H passante**, preforo D5 |
| Fermi di sovraccarico | 4 | (-112,-20), (-112,120), (+112,-20), (+112,120) | **M6x1-6H passante**, preforo D5 |
| Vassoio potenza | 4 | (45,22), (45,88), (118,22), (118,88) | **M3x0,5-6H passante**, preforo D2,5 |
| Vassoio elettronica libero | 4 | (-107,-53), (-107,-3), (26,-53), (26,-3) | **M3x0,5-6H passante**, preforo D2,5 |
| Fermacavi | 4 | (126,-28), (138,-28), (16,18), (28,18) | **M3x0,5-6H passante**, preforo D2,5 |

I filetti dei piedi consentono di regolarne la quota; i controdadi superiori bloccano la regolazione. Non sostituire questi quattro filetti con semplici fori liberi mantenendo un solo dado superiore.

I grani arresto M6x60 attraversano tutto il filetto del basamento. La quota nominale della punta e Z67,5, che lascia 1,5 mm sotto il carrier; il lato inferiore e Z7,5. I quattro controdadi dei grani sono sopra il basamento. La scocca plastica dispone di quattro passaggi D8 allineati, che non si devono confondere con i fori filettati della base.

## 3. Carrier mobile del piatto

File: `step/plate_carrier_aluminium.step`. Un pezzo, **276 x 196 x 8**, angoli R16, centro globale XY(0,50), Z69..77. Origine locale al centro del pezzo in pianta.

| Gruppo | Quantita | Coordinate locali XY | Coordinate globali XY | Lavorazione finale |
|---|---:|---|---|---|
| Fissaggio cella mobile | 2 | (0;-7,5), (0;+7,5) | (0;42,5), (0;57,5) | D6,4 passante + svasatura superiore **90 gradi, diametro nominale D12,4** |
| Fissaggio piatto plastico | 6 | X=-113,+113; per ogni X, Y=-74,0,+74 | X=-113,+113; per ogni X, Y=-24,50,124 | **M3x0,5-6H passante**, preforo D2,5 |

Con D6,4 e D12,4 la profondita geometrica della svasatura a 90 gradi e `(12,4-6,4)/2 = 3,0 mm`; restano 5,0 mm di tratto cilindrico. La vite selezionata e **M6x45 DIN7991**, testa nominale D12 e altezza di catalogo 3,3 mm: questa altezza non definisce da sola l'angolo del cono. Verificare che la testa sia a filo o leggermente incassata e non sollevi il piatto plastico. Non sostituire il modello di vite senza confrontare testa e sede.

Il piatto plastico 280x200 sporge nominalmente 2 mm dal carrier e si avvita ai sei filetti M3. Non aggiungere rondelle sotto le teste coniche delle viti della cella.

## 4. Distanziale fisso

File: `step/cell_fixed_spacer.step`. Un pezzo, **25 x 30 x 6**, raccordi verticali R1, centro globale XY(-105,5;50), Z19..25.

| Quantita | Coordinate locali dal centro del pezzo | Coordinate globali | Lavorazione |
|---:|---|---|---|
| 2 fori | (-0,5;-7,5), (-0,5;+7,5) | (-106;42,5), (-106;57,5) | D6,4 passante, senza filetto |

L'offset locale di 0,5 mm e intenzionale: fa coincidere l'appoggio con il blocco terminale della cella e i fori con il suo disegno. Le due M6x25 si inseriscono dal basso attraverso rondella 1,6 mm, basamento 6 mm e distanziale 6 mm; il filetto della cella viene impegnato nominalmente per 11,4 mm.

## 5. Distanziale mobile

File: `step/cell_moving_spacer.step`. Un pezzo, **25 x 30 x 22**, raccordi verticali R1, centro globale XY(-0,5;50), Z47..69.

| Quantita | Coordinate locali dal centro del pezzo | Coordinate globali | Lavorazione |
|---:|---|---|---|
| 2 fori | (+0,5;-7,5), (+0,5;+7,5) | (0;42,5), (0;57,5) | D6,4 passante, senza filetto |

Le due M6x45 svasate si inseriscono dall'alto del carrier e attraversano 8+22 mm di alluminio; i 45 mm includono la testa. L'ingaggio nominale nella cella e 15 mm, senza raggiungere il lato inferiore del blocco da 22 mm.

## 6. Vincoli di montaggio da conservare

- Il basamento, la scocca e i fermi restano fissi. Carrier, piatto e distanziale mobile devono poter muoversi senza contatto con la scocca.
- I fermi non vanno precaricati contro il carrier. La quota 1,5 mm e provvisoria: regolarli sulla cella montata dopo misura di flessione e carichi eccentrici.
- Il datasheet L6D riporta filetti M6 passanti e viti 8.8 lubrificate, ma la coppia per la capacita esatta 30 kg e ambigua. Confermare con Zemic la coppia e la giunzione mobile con viti svasate 10.9.
- Confermare con Zemic l'orientamento del suo limite piattaforma 250x350 rispetto all'asse della cella. Il CAD sceglie la cella lungo X, parallelamente al lato 280 mm del piatto.

Queste note descrivono i quattro pezzi metallici del prototipo nominale. Non costituiscono una dichiarazione di portata, accuratezza o fit fisico gia collaudato.

