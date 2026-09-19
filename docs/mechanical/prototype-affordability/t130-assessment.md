# Gruppo T130: supporti commerciali e plastiche personalizzate

Valutazione del 7 settembre 2026 dalle quattro immagini fornite da Andrea, conservate in [sources](sources/). Sono dati del venditore, non prove indipendenti. La sezione automatica «Panoramica prodotto con IA» non è usata come specifica tecnica.

**Questa è la prima architettura da sviluppare per il prototipo da 20 kg utili.** La bilancia donatrice resta un'alternativa economica di confronto. Il gruppo rende plausibile eliminare i quattro pezzi metallici custom del Concept 02; prezzo completo e compatibilità degli innesti non sono ancora verificati.

## Quote disponibili

| Dato | Lettura | Fonte e limite |
|---|---|---|
| Staffa media | 220 × 125 mm | [Immagine 2](sources/t130-02-brackets.png): ingombri, non interassi degli appoggi |
| Staffa grande | 259 × 169 mm | Immagine 2 e [disegno](sources/t130-03-drawing.png) |
| Fori esterni della grande | Ø10, interassi 240 × 150 mm | Disegno: non sono indicati come filetti M10 |
| Attacchi interni | Indicazioni M5, Ø3,5, Ø5,3, Ø6,3, Ø7,5 | Non assumere quattro attacchi uguali; le coppie interne hanno quote differenti 175,1 e 167,7 mm |
| Cella | Famiglia T130, lunghezza pubblicizzata 130 mm | [Immagine 1](sources/t130-01-offer.png); mancano disegno completo e interassi della variante |
| Piattaforma consigliata | 250 × 350 mm | [Scheda](sources/t130-04-specs.png): il piatto 280 × 200 rientra nell'inviluppo con assi coerenti |
| Altezza totale del gruppo | Non determinata | Sezioni tagliate: non sommare quote locali per inventare l'altezza completa |

Le fotografie fanno ritenere plausibile la staffa media mobile superiore e quella grande fissa inferiore, collegate alle estremità opposte della cella. È un'inferenza da confermare per la combinazione acquistata. La lega di alluminio è dichiarata nel titolo; lega esatta, processo produttivo e portata delle staffe non sono documentati dalle immagini.

## Struttura proposta

**Piatto plastico nervato sotto → staffa media mobile → cella → staffa grande fissa → supporti corti sotto il fondo → piedi allargati.**

La staffa superiore distribuisce il carico fra più appoggi del piatto. Non rende automaticamente sufficiente una lastra plastica piatta e sottile: servono nervature inferiori e verifica del carico locale. Rispetto all'ingombro della staffa media centrata, gli sbalzi del piatto sarebbero 30 e 37,5 mm per lato; le distanze dagli appoggi effettivi saranno maggiori e richiedono quote complete.

Il fondo può avere il piano interno liscio e i rinforzi sul lato inferiore. La plastica fra staffa fissa e piedi resta strutturale localmente, perché deve trasferire forze e momenti. Il guscio svolge principalmente la funzione di involucro. Viti, dadi, rondelle, boccole e piedi saranno standard e rappresentati nel CAD, dopo il dimensionamento.

### Stabilità dei piedi

I fori 240 × 150 mm non sono una disposizione automaticamente adatta ai piedi. Un piatto 280 × 200 mm sporge di 20 e 25 mm oltre quelle linee di appoggio.

Esempio limite: piedi puntuali a Y = ±75 mm, carico puntuale 20 kg a Y = +100 mm e massa propria centrata. L'equilibrio rispetto ai piedi anteriori richiede una massa propria superiore a 20 × 25 / 75 = **6,67 kg**. Il modello non include larghezza dei piedi, impronta del pacco, urti e baricentro reale: serve a escludere una promessa di stabilità ai bordi fondata soltanto sulla presenza della staffa.

La proposta preliminare allarga i centri piedi verso **X = ±140, Y = ±125 mm**, ad esempio su fondo 310 × 300 mm. È una quota di progetto, non del prodotto acquistato. Le campate dalla staffa ai piedi diventano corte, con spostamento in pianta circa 20 × 50 mm. Le nervature inferiori vanno dimensionate. Un fondo da 310 mm richiede una macchina adeguata o una revisione di ingombro; non è già compatibile con ogni servizio ABS da 300 × 300 mm.

### Portata e accuratezza

Per **20 kg utili** si valuta una cella **30 kg**, se disponibile con caratteristiche e attacchi adatti: la 20 kg selezionata nella schermata deve sostenere anche piatto, staffa mobile e fissaggi. La tara effettiva e i carichi eccentrici vanno verificati.

La scheda riporta sovraccarico sicuro 120% FS e limite estremo 150% FS: non aumentano la portata nominale d'uso. Le battute richiedono gioco regolato sulla corsa reale, senza contatti parassiti durante la misura.

La riga «Quadrangular erro 0.02% loading value/100mm», letta letteralmente, corrisponde a 4 g su 20 kg per 100 mm. La formulazione è incompleta e non supporta una garanzia di accuratezza complessiva di 1 g. Mancano sensibilità, non linearità, isteresi, creep e ripetibilità della variante. Le indicazioni di serraggio senza unità leggibili non vengono trasformate in coppie di montaggio.

## Ordine singolo e prezzo

La [schermata di acquisto](sources/t130-01-offer.png) mostra **17,39 € con «20kg load cell» selezionato**, IVA indicata inclusa e dazi al checkout. «Large bracket», «Medium bracket» e «Small stand» sono opzioni distinte: **17,39 € non è un prezzo verificato per due staffe e cella insieme**.

Da quotare: una cella 30 kg candidata, una staffa grande, una media, eventuali distanziali originali, piedi e minuteria standard. Il contenuto di ciascuna confezione, MOQ, prezzo delle staffe e spedizione non sono visibili. I pezzi plastici vanno quotati in una copia ciascuno sul CAD definitivo. Otto tasti, OLED posseduto e ricarica USB restano requisiti della nuova scocca; il kit non risolve l'alimentazione.

Andrea indica anche la [pagina Alibaba 1601276726922](https://www.alibaba.com/product-detail/Universal-Aluminum-Alloy-Load-Cell-Mounting_1601276726922.html) come lo stesso prodotto. Il lettore web non ne ha restituito il contenuto. L'identità esatta della variante, il minimo d'ordine, il contenuto e il prezzo di quella inserzione non sono verificati: non trasferire dati fra le due inserzioni.

Prima del CAD assemblabile mancano: disegni completi di cella/staffe/distanziali; limiti meccanici e deformazione del gruppo; contenuto e costo dell'ordine singolo. Il Concept 02 non rappresenta questo gruppo. Nessun nuovo CAD, acquisto o preventivo di stampa; firmware e alimentazione installata non sono stati modificati.
