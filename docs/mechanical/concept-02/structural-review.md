# Revisione meccanica indipendente - Concept 02

Stato: **screening analitico e revisione geometrica del prototipo; portata fisica e precisione non ancora verificate**. Non e una FEA. I numeri servono a individuare le verifiche necessarie prima dell'uso con 20 kg.

## 1. Percorso del carico

Il carico attraversa piatto plastico, carrier in alluminio, distanziale mobile, cella, distanziale fisso, basamento in alluminio e quattro piedi. La scocca resta separata dal sottogruppo mobile. La parte plastica non e piu chiamata a portare 20 kg attraverso montanti alti e isolati.

| Parte | Geometria nominale letta dal CAD |
|---|---|
| Basamento | 300 x 320 x 6, raccordi R18, Z13..19 |
| Carrier | 276 x 196 x 8, R16, centro XY(0,50), Z69..77 |
| Piatto plastico | 280 x 200 x 4, R20, centro XY(0,50), Z77..81 |
| Appoggio mobile | 25 x 30 x 22, centro XY(-0,5;50), Z47..69 |
| Cella | Corpo 130 x 30 x 22, centro XY(-53;50), Z25..47 |
| Appoggio fisso | 25 x 30 x 6, centro XY(-105,5;50), Z19..25 |
| Fori cella | Lato fisso X=-106; mobile X=0; Y=42,5 e 57,5 |
| Piedi | Quattro centri X=+-132, Y=+-142, pattini nominali D30 |

La superficie portante sotto la plastica e quasi continua; lo sbalzo perimetrale plastico e nominalmente 2 mm. I distanziali toccano soltanto i blocchi terminali della cella. Il supporto mobile attraversa un'apertura della scocca; non e appoggiato al tetto.

## 2. Stabilita con carico eccentrico

Con quattro pattini circolari D30 tutti aderenti a un piano, il poligono effettivo di appoggio e l'inviluppo convesso dei dischi, cioe un rettangolo raccordato. Gli estremi raggiungono X=+-147 e Y=+-157.

Il contorno reale del piatto, **raccordato R20**, e interamente compreso nell'inviluppo e mantiene **7 mm di margine geometrico minimo**. La verifica campiona anche gli archi. Il margine non va confuso con un rettangolo di appoggio pieno: per l'angolo teorico (140,150) del rettangolo d'ingombro, che non appartiene al piatto raccordato, sarebbe soltanto `15 - sqrt(8^2+8^2) = 3,69 mm`.

Una seconda verifica tratta prudentemente i piedi come **punti nei loro centri**, rinunciando al contributo della larghezza dei pattini. Il piatto supera allora il rettangolo dei centri di 8 mm a destra e dietro, ma il peso proprio sposta la risultante verso l'interno.

Dai volumi nominali dei quattro pezzi custom in alluminio si ottengono circa **2,756 kg**, assumendo densita uniforme 2,70 g/cm3. Il baricentro ricavato dai solidi CAD di questi soli pezzi e circa X=-0,41, Y=21,96 mm. Non e il peso o il baricentro misurato della bilancia completa.

| Carico utile di 20 kg | Risultante con i soli quattro pezzi in alluminio | Margine rispetto ai centri piedi |
|---|---|---|
| Bordo destro, XY(140;50) | Circa (122,99;46,60) | 9,01 mm a destra |
| Bordo posteriore, XY(0;150) | Circa (-0,05;134,49) | 7,51 mm dietro |
| Angolo del rettangolo d'ingombro, XY(140;150) | Circa (122,99;134,49) | 9,01 mm a destra e 7,51 mm dietro |

Nell'ultima idealizzazione il rapporto fra momento stabilizzante dei quattro pezzi e momento ribaltante del carico oltre i centri piedi e circa **2,28 lateralmente** e **2,07 posteriormente**. Sono rapporti di questo specifico equilibrio statico, non fattori di sicurezza certificati dell'intera bilancia.

Questi risultati richiedono piedi livellati, contatto dei quattro appoggi e carico verticale statico. Un contenitore molto alto, una spinta orizzontale, un urto o un piede non aderente richiedono un'altra verifica. La regolazione reale avviene nei quattro filetti M6 del basamento, con controdadi superiori.

## 3. Ipotesi dei calcoli di rigidezza

Si assume alluminio **6061-T6**, modulo elastico **68,3 GPa** e snervamento tipico **276 MPa**, tratti dalla [scheda tecnica Kaiser 6061](https://online.kaiseraluminum.com/depot/PublicProductInformation/Document/1015/Kaiser_Aluminum_6061_Sheet_Coil_and_Plate.pdf). Sono proprieta tipiche di riferimento, non il certificato del materiale acquistato.

Il carico di screening e **22 kg equivalenti = 215,75 N**: 20 kg utili piu un'allocazione di 2 kg per l'insieme mobile. Il peso effettivo del piatto completo dovra rientrare in questa allocazione oppure i conti vanno aggiornati. L'allocazione non cambia la portata nominale da 30 kg della cella.

Calcoli riproducibili: [structural_screening.py](structural_screening.py); risultati: [structural-screening.json](structural-screening.json). Il codice usa equazioni di trave di Euler-Bernoulli, senza elementi finiti. I vincoli ideali non comprendono gioco dei fissaggi, rotazione della cella, deformabilita dei piedi, creep o risposta agli urti.

## 4. Carrier in alluminio da 8 mm

Per rendere visibile la sensibilita alla distribuzione del carico si confrontano una striscia larga quanto tutto il carrier e una striscia larga quanto l'appoggio. Sono modelli distinti: **non costituiscono un intervallo garantito della deformazione reale della piastra**. Entrambi assumono incastro ideale alla zona centrale.

Per una striscia di larghezza `b`, spessore `h`, sbalzo `L` e carico terminale `F`:

`I = b h^3 / 12; sigma = 6 F L / (b h^2); delta = F L^3 / (3 E I)`.

| Caso a 22 kg equivalenti | Larghezza efficace | Sbalzo | Tensione nominale | Freccia terminale |
|---|---:|---:|---:|---:|
| Bordo X, partecipazione larga | 196 mm | 128 mm | 13,21 MPa | 0,264 mm |
| Bordo X, striscia dell'appoggio | 30 mm | 128 mm | 86,30 MPa | 1,725 mm |
| Bordo Y, partecipazione larga | 276 mm | 85 mm | 6,23 MPa | 0,055 mm |
| Bordo Y, striscia dell'appoggio | 25 mm | 85 mm | 68,77 MPa | 0,606 mm |

Le tensioni nominali di questi modelli sono inferiori allo snervamento tipico. Il caso piu stretto raggiunge circa il 31% del valore tipico; questo **non dimostra** la resistenza dei fori, delle svasature, della cella o dell'assemblaggio. La dispersione delle frecce indica che la rigidezza al carico eccentrico e una verifica concreta da fare sul piatto montato.

I due casi X includono anche il punto a X=112 degli arresti: la sola freccia di striscia a quella coordinata e circa **0,179 mm** oppure **1,168 mm**. Non va sommata ciecamente ad altre frecce per produrre una previsione definitiva, ma serve a controllare il rischio di contatto prematuro dei fermi.

## 5. Basamento in alluminio da 6 mm

Il carico non entra nella base al centro del piatto: entra a **X=-106 mm**, portando anche il momento generato dalla distanza del carico. Per esempio, con il carico al bordo destro X=140, il momento applicato al fissaggio e `215,75 x (140+106) = 53,07 N m`. Omettere questo momento sottostimerebbe la sollecitazione della base.

Lo screening seguente rappresenta la base come trave larga, appoggiata su due **linee continue** corrispondenti alle coordinate dei piedi. Questa idealizzazione non riproduce la flessione/torsione bidimensionale fra i quattro piedi reali.

| Caso | Luce e larghezza | Momento massimo interno | Tensione nominale massima | Freccia massima del modello |
|---|---|---:|---:|---:|
| X, carico al centro piatto | Luce 264, larghezza 320 mm | 25,67 N m | 13,37 MPa | 0,311 mm |
| X, carico al bordo destro | Luce 264, larghezza 320 mm | 52,90 N m | 27,55 MPa | 0,640 mm |
| Y, carico al bordo posteriore | Luce 284, larghezza 300 mm | 20,41 N m | 11,34 MPa | 0,097 mm |

Il modello isola il carico mobile e per questo alcune reazioni sono negative: per l'equilibrio completo serve anche il peso proprio, trattato nella sezione sulla stabilita. Non si deve scambiare questa idealizzazione con una prova che tutti i piedi restino caricati allo stesso modo.

Il materiale pieno e gli spessori proposti rendono plausibile una struttura metallica compatta con pavimento interno liscio. **Il basamento reale su quattro appoggi, il fissaggio stretto della cella e la torsione sotto carico d'angolo restano da misurare o analizzare con un modello di piastra appropriato.**

## 6. Fissaggi, arresti e tolleranze

| Fissaggio | Catena nominale | Ingaggio risultante |
|---|---|---:|
| Cella fissa, M6x25 ISO 4762 | Rondella 1,6 + base 6 + distanziale 6 | 11,4 mm nella cella |
| Cella mobile, M6x45 DIN 7991 | Carrier 8 + distanziale 22; lunghezza testa compresa | 15 mm nella cella |
| Piedi M6 | Filetto passante nella base e dado superiore | 6 mm nella base |
| Grani arresto M6x60 | Base Z13..19, grano da Z7,5 a 67,5 | 6 mm nella base |

Le viti della cella restano entro i 22 mm di spessore del blocco. Le teste mobili sono svasate per lasciare piano il supporto del piatto. Le svasature devono avere cono **90 gradi**, con lavorazione definita dal diametro; non ricavare l'angolo dividendo semplicemente tutte le quote di testa del catalogo.

La cella richiede zone di appoggio piane e un serraggio coerente. Il suo datasheet lascia ambiguo il valore di coppia per la capacita esatta di 30 kg e specifica viti 8.8; la giunzione mobile svasata 10.9 necessita conferma della coppia appropriata. Gli ingaggi geometrici non sostituiscono questa verifica.

I quattro arresti hanno **1,5 mm di gioco nominale**, attraversano appositi passaggi D8 nel tetto e devono restare senza contatto con il carrier durante la misura. Il grano presenta la punta piana verso il piatto e la presa esagonale verso il basso. La regolazione richiede accesso inferiore a bilancia sollevata, poi blocco del controdado.

L'[audit geometrico indipendente](mechanical-audit.json) rileva volume di intersezione nullo fra scocca e carrier, distanziale mobile e ciascuno dei quattro ingombri massimi D6 dei grani. L'audit comprende i quattro solidi CNC e queste interfacce nominali; non comprende tolleranze, deformazione sotto carico o tutte le parti dell'assieme. Va rigenerato se cambia questa geometria. La distinta delle lavorazioni e in [manufacturing-notes.md](manufacturing-notes.md).

La cella dichiara 0,6-0,8 mm di deflessione alla portata nominale in questa fascia. Una semplice proporzione lineare a 22 kg darebbe circa 0,44-0,59 mm: e un'ipotesi, non una misura della cella scelta. In combinazione con la flessione del carrier e della base, **il gioco 1,5 mm non e ancora dimostrato sufficiente per 20 kg eccentrici, ne e dimostrato che limiti il sovraccarico al valore corretto**. I fermi si regolano dopo la caratterizzazione dell'assieme, controllando tutti i bordi. Un fermo che tocca troppo presto sottrae peso alla cella e falsifica la lettura.

## 7. Esito e prova del prototipo

La geometria offre una base credibile per il prototipo: appoggi ampi e periferici, contorno piatto dentro l'impronta, carico attraverso elementi metallici identificati, piatto plastico quasi completamente sostenuto, interno senza nervature portanti.

La prima verifica fisica deve misurare ritorno allo zero, freccia e giochi con masse crescenti al centro, sui quattro bordi e nelle zone d'angolo, fino ai 20 kg utili dopo la verifica preliminare dei fissaggi. Va ripetuta dopo permanenza sotto carico e dopo il livellamento dei piedi. Annotare eventuali contatti con scocca/arresti e deformazioni residue. Non usare urti come primo test.

Una lettura da **1g** non prova accuratezza di 1 g: per la cella C3 selezionata il solo errore combinato dichiarato equivale a 6 g sul fondo scala 30 kg. Il pacchetto CAD non certifica precisione, portata, tenuta o comportamento del prodotto stampato.

