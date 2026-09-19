# Minù — Scocca Concept 01

Proposta preliminare per una bilancia da banco con scocca, basamento e piatto in plastica. **20 kg è il requisito di progetto; la portata dell'insieme non è ancora verificata.** Il concept permette di valutare forma, ingombri e disposizione delle nervature. Gli attacchi della cella e le giunzioni tra pezzi devono essere definiti prima di realizzare una bilancia funzionante.

## Forma e ingombri

- Larghezza × profondità × altezza complessiva: **280 × 300 × 110 mm**, con piedini indicativi.
- Piatto: **280 × 200 mm**, nella zona posteriore.
- Angoli raccordati, frontale inclinato liscio, basamento arretrato e piatto separato dalla scocca.
- Nessun display, tastiera o connettore definito in questa fase.
- Nessun telaio generale in alluminio. La cella e futuri inserti o viti locali restano componenti separati.

Le dimensioni sono scelte di questo concept, ispirate agli ingombri della [EVA 9901 descritti nella scheda commerciale METRO](https://www.metro.it/marketplace/product/8e3196b6-624e-4b28-9775-db98ee96afa2). Non costituiscono un rilievo della bilancia di Andrea. La scheda attuale Kaufgut presenta campi dimensionali diversi; per questa proposta si usa esplicitamente il riferimento commerciale 280 × 300 × 110 mm.

## Struttura proposta

Tre parti plastiche separate:

1. **Basamento nervato:** collega la zona di appoggio fissa della futura cella alle zone dei quattro piedi.
2. **Scocca:** protegge il volume interno e forma il frontale. Non deve sostenere direttamente il piatto durante la pesata.
3. **Piatto nervato:** distribuisce il carico verso la zona mobile della futura cella. La faccia superiore resta piana.

Il percorso previsto del peso è **piatto → attacco mobile → cella → attacco fisso → basamento → piedi**. La cella single-point è un'ipotesi di architettura; il suo volume nel CAD è soltanto una riserva di spazio. Non è stata selezionata una cella e non sono definiti interassi, distanziali, coppie di serraggio o arresti di sovraccarico. Il piatto non deve appoggiarsi alla scocca: un contatto parassita aggirerebbe il sensore.

Quote effettive del modello:

| Elemento | Geometria preliminare |
|---|---|
| Pelle superiore del piatto | 6 mm |
| Nervature del piatto | 6 mm di spessore × 30 mm di altezza, passo 40 mm |
| Isola mobile del piatto | 80 × 64 mm, centrata sotto il piatto |
| Fondo basamento | 6 mm |
| Nervature principali basamento | 4 mm di spessore × 25 mm di altezza |
| Pareti scocca | 3,2 mm nominali |
| Riserva indicativa cella | 38 × 100 × 22 mm, senza interassi |

La riserva cella non interseca i tre solidi plastici a riposo. Le isole interne hanno aperture di alleggerimento e accesso per la rimozione della polvere; tali aperture non sono fori di montaggio della cella.

Il controllo preliminare delle sezioni è in [structural-screening.md](structural-screening.md). Serve per scegliere proporzioni plausibili delle nervature; non sostituisce il calcolo dell'assemblato né le prove sotto carico.

## File

- `concept-estetico.png`: immagine illustrativa generata con ImageGen. Mostra la direzione estetica; le superfici non sono un rendering metrico del CAD.
- `generate_concept.py`: modello parametrico in CadQuery, commentato in inglese.
- `step/`: solidi modificabili, singoli e assemblato con riferimenti separati.
- `stl-forma/`: tre parti triangolate per valutare e stampare la geometria preliminare. Nessuna cella o piedino di riferimento incluso.
- `preview-mesh.json`: triangolazione alleggerita dello stesso CAD per l'anteprima.
- `verification.json`: controlli geometrici effettivamente eseguiti e parametri del modello.
- `imagegen-prompt.txt`: prompt del concept estetico; generazione tramite strumento integrato, senza CLI/API esterna.

Tutte le quote sono in **millimetri**. Gli STL non contengono l'unità: impostare mm nel software di stampa. Le parti mantengono le coordinate dell'assemblato; il service deve orientare e posizionare ciascuna parte sul piano di costruzione.

Verifica locale eseguita: ciascun pezzo è un solido CAD valido; ogni STL è una mesh chiusa con orientamento coerente e un solo componente. Non risultano volumi di intersezione fra le tre parti plastiche, né fra esse e il riferimento cella. Questi controlli verificano la geometria, non la resistenza sotto carico.

## Stampa presso terzi

Per lo sviluppo funzionale si propone **PA12 con MJF o SLS**, previa verifica di materiale, processo, tolleranze e orientamento con il service. HP indica il PA12 fra i materiali adatti a scocche e componenti tecnici: [materiali HP](https://www.hp.com/us-en/printers/3d-printers/materials.html). La scelta di processo fa parte del dimensionamento; una stampa di forma in un materiale diverso non dimostra il comportamento della futura versione funzionale.

Il service può usare gli STL per un preventivo o un campione di forma. **Non presentare questo pacchetto come progetto già verificato per sostenere 20 kg in esercizio.** Prima della stampa funzionale vanno chiusi gli attacchi della cella, le giunzioni, i piedi reali e i finecorsa, quindi ricontrollate deformazioni e interferenze. Le prove dovranno comprendere carico centrato, ai bordi e agli angoli, ritorno a zero e permanenza del peso nel tempo.

Per le interfacce della cella, il produttore richiede appoggi rigidi e piani e indica la protezione meccanica dai sovraccarichi: [manuale A&D LCB27](https://www.aandd.jp/products/manual/loadcells/lcb27.pdf). È una fonte per l'architettura, non una selezione di quel modello.

## Rigenerazione

Usare Python compatibile con CadQuery e installare le dipendenze in un ambiente esterno al repository:

```sh
python -m pip install cadquery==2.8.0 trimesh
python generate_concept.py
```

Questo progetto meccanico non modifica firmware, cablaggio o calibrazione della bilancia esistente.
