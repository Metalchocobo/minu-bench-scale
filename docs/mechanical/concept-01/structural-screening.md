# Controllo preliminare della rigidezza — concept 01

Il requisito di progetto è **20 kg di carico utile**. Il CAD rappresenta un piatto e un basamento portanti in plastica; la scocca esterna non deve sostenere il piatto. Questo controllo orienta le proporzioni delle nervature: **non dimostra ancora la portata dell'assemblato**.

## Geometria considerata

Quote lette in `generate_concept.py`, in millimetri:

- Piatto 280 × 200, pelle superiore 6, superficie a Z = 110.
- Nervature monolitiche 6 × 30, passo 40, da Z = 74 a Z = 104.
- Isola mobile 80 × 64, centrata in X = 0, Y = 50; fondo, pilastri e setti ancora senza interfaccia con una cella selezionata.
- Traverso a Y = 30: bordo laterale a X = 136 e bordo dell'isola a X = 40; tratto esterno **L = 96**.
- Nervatura longitudinale a X = 20: estremità anteriore Y = −47 e bordo isola Y = 18; tratto esterno **L = 65**.
- Distanza geometrica nominale fra fondo nervature e tetto della scocca: 3.
- Basamento: pavimento 6 e nervature 4 × 25. Il presente calcolo non verifica il basamento.

## Modello semplificato

Si isola una nervatura con una striscia della pelle larga **24 mm**: questa sezione a T esiste nel CAD lungo i due tratti considerati. La larghezza ridotta mantiene la striscia entro la proiezione dell'isola alla radice del traverso Y = 30. Il resto della griglia, il bordo e la restante pelle non contribuiscono al modello.

Alla radice si assume un **incastro perfettamente rigido al bordo dell'isola**. L'isola reale, la sua connessione alla cella e il basamento non sono incastri ideali: la loro deformazione è esclusa. Si applica all'estremità della singola nervatura l'intero carico verticale statico di 20 kg, senza ripartirlo arbitrariamente fra più nervature.

Omettere il resto della griglia penalizza la nervatura isolata; assumere la radice rigida favorisce il risultato. Per questo **il calcolo non è un limite superiore garantito della deformazione del piatto completo**. Non copre un carico all'angolo, la flessione locale della pelle fra nervature, il breve bordo oltre la nervatura o una torsione dell'insieme.

## Materiale e formule

Ipotesi di screening: PA12, comportamento elastico lineare isotropo, **E = 1500 N/mm²**. È il modulo di flessione tipico X dichiarato per EOS PA 2200 asciutto ed è inferiore al suo modulo di trazione tipico di 1650 N/mm². Non è un minimo garantito per qualsiasi PA12, orientamento, condizionamento o service. [Scheda primaria EOS PA 2200](https://www.eos.info/es/soluciones-de-polimeros/materiales-polimeros/fichas-tecnicas/mds-pa-2200).

La sezione comprende anima 6 × 30 e ala 24 × 6. Misurando Z dal fondo dell'anima:

- area anima = 180 mm²; baricentro a 15 mm;
- area ala = 144 mm²; baricentro a 33 mm;
- baricentro complessivo = 23 mm;
- momento d'inerzia **I = 39 852 mm⁴**;
- massima distanza dal baricentro **c = 23 mm**.

Con F = 20 × 9,81 = **196,2 N**:

```text
z_bar = sum(A_i * z_i) / sum(A_i)
I = sum(b_i * h_i^3 / 12 + A_i * (z_i - z_bar)^2)
M_root = F * L
sigma_max = M_root * c / I
delta_bending = F * L^3 / (3 * E * I)
```

I tratti sono relativamente corti rispetto alla loro altezza. Si aggiunge quindi una stima del contributo di taglio, senza presentare la sola flessione come spostamento totale. Assumendo Poisson ν = 0,40, G = E / [2(1 + ν)] = 535,7 N/mm² e una sezione resistente a taglio pari alla sola anima A_web = 180 mm²:

```text
k = 5 / 6
delta_shear = F * L / (k * G * A_web)
delta_section_estimate = delta_bending + delta_shear
```

ν e il fattore di taglio sono assunzioni del modello, non proprietà misurate sul pezzo. La stima non include concentrazioni di tensione nei raccordi e nelle giunzioni.

## Risultati

| Tratto isolato | Carico | Tensione normale massima nominale | Freccia da flessione | Contributo stimato di taglio | Somma stimata |
|---|---:|---:|---:|---:|---:|
| Laterale, L = 96 mm | 20 kg | 10,87 MPa | 0,968 mm | 0,234 mm | **1,20 mm** |
| Anteriore/posteriore, L = 65 mm | 20 kg | 7,36 MPa | 0,300 mm | 0,159 mm | **0,46 mm** |

I risultati descrivono una singola sezione con i vincoli sopra indicati. Rendono plausibile l'impiego di nervature profonde in plastica per il requisito di 20 kg e motivano la centratura dell'isola mobile. **Non costituiscono un esito di verifica dell'assemblato né un margine di sicurezza certificato.** Non si può sottrarre semplicemente 1,20 mm dal gioco nominale di 3 mm e dichiarare garantita l'assenza di contatto: mancano deformazioni dell'isola/base/cella, tolleranze e condizioni di carico fuori da questo modello.

## Verifiche necessarie per un prototipo funzionale da 20 kg

- Definire la cella e i suoi fissaggi, considerando anche il peso del piatto e dei componenti mobili oltre ai 20 kg utili. La sola riserva volumetrica non chiude il percorso del carico.
- Verificare basamento, isole, giunzioni e fissaggi; la flessione del sensore deve avvenire senza contatti con scocca o arresti durante la pesatura normale.
- Verificare piatto intero con carico centrale, laterale e ai quattro angoli, incluse deformazioni locali e distribuzione effettiva del carico. Un'analisi numerica deve usare la geometria dell'assemblato e dati del processo scelto.
- Verificare il pezzo stampato con carico noto, scarico/ritorno a zero, ripetibilità, carico eccentrico e permanenza sotto carico nelle condizioni d'uso. Impatti e arresti di sovraccarico richiedono una verifica separata.
- Confermare materiale, orientamento, qualità e tolleranze con il service. Il comportamento sotto carico prolungato non deriva dal solo modulo elastico: prove su PA12 SLS documentano componenti elastiche, viscoelastiche e irreversibili della deformazione. [Krönert et al., ricerca originale sul creep del PA12 SLS](https://link.springer.com/article/10.1007/s00170-022-09446-z).

**Stato:** geometria stampabile e proporzioni delle nervature controllate preliminarmente; cella/assemblaggio e portata funzionale da verificare. Nessuna FEA o prova fisica eseguita.
