# Minù — Proposta meccanica 02

**Stato economico:** questo assieme richiede quattro parti in alluminio su misura e non è la soluzione consigliata per il singolo prototipo economico. La [valutazione corrente](../prototype-affordability/README.md) propone un gruppo di pesatura commerciale e nuove plastiche FDM; i suoi innesti non sono rappresentati negli STEP/STL di questa cartella o nello ZIP Concept 02.

**Alimentazione installata:** batteria USB NASTIMA BK06-LF60-NATC LiFePO4 6,4 V / 6 Ah e Mini360 MP1482DS, con uscita regolata da Andrea a 5,11 V e primo funzionamento riferito positivo. Il gruppo Waveshare rappresentato in questo CAD resta una variante meccanica non installata. Per il cablaggio corrente vedere [WIRING](../../WIRING.md) e per il comportamento firmware il [README principale](../../../README.md).

Prototipo parametrico per una bilancia con **20 kg di carico utile di progetto**, scocca e piatto esterno in plastica, fondo interno piano e supporto del piatto in alluminio. Il pacchetto contiene l'assieme STEP con componenti separati, i pezzi custom STEP/STL, la distinta di acquisto e le note di montaggio. È una base concreta per il prototipo: la portata, gli accoppiamenti stampati e il funzionamento della ricarica non sono ancora collaudati fisicamente.

## Geometria e materiali

| Elemento | Scelta di progetto |
|---|---|
| Corpo | **300 × 320 × 81 mm**, altezza con piedi nella posizione nominale; la presa USB porta la larghezza complessiva a circa **312 mm** |
| Piatto esterno | **280 × 200 × 4 mm**, plastica PA12 proposta, spigoli raccordati R20 |
| Portapiatto | **276 × 196 × 8 mm**, alluminio 6061-T6, supporto quasi continuo della plastica |
| Fondo | **300 × 320 × 6 mm**, alluminio 6061-T6, superficie superiore piana |
| Guscio | PA12 MJF/SLS, pareti nominali 3 mm; rimovibile dal basamento |
| Cella | Zemic L6D-C3-30kg-0.4B, corpo 130 × 30 × 22 mm; capacità nominale 30 kg per lasciare margine al piatto e al supporto |
| Piedi | 4 Elesa D30, centri X ±132 / Y ±142 mm, regolazione M6 e controdado |
| Comandi | 2 tastierini adesivi BerryBase B-SM4T da 4 tasti, totale **8 tasti** |
| Display | Riutilizzo dell'**OLED SSD1322 SPI 256×64, 3,12 pollici** documentato nel progetto; cornice separata e sostituibile |
| Ricarica proposta | USB-C laterale Premier PCM-0726, verso modulo Waveshare UPS HAT(D) con due 21700 |

Il fondo non ha una griglia di nervature. Sono presenti soltanto i fissaggi funzionali: cella, piedini, fermi di sovraccarico e vassoi rimovibili. La scocca ha sedi per dadi lungo il perimetro. Le schede si dispongono sopra un vassoio isolante piano; non devono appoggiare direttamente sull'alluminio.

Il percorso del carico è **piatto plastico → portapiatto metallico → distanziale mobile → cella → distanziale fisso → fondo metallico → piedi**. Il guscio e i cavi devono rimanere separati dal piatto mobile. Gli arresti regolabili sono previsti per intervenire oltre il campo di misura; il gioco corretto deve essere determinato sul prototipo.

## File da aprire

- [Schema quotato di ingombro e percorso del carico](drawings/layout-and-load-path.svg): pianta e disposizione del gruppo di pesatura.
- [Assieme STEP completo](step/MINU_concept_02_COMPLETE_NOMINAL_ASSEMBLY.step): ogni componente è un solido nominato, compresi viti, dadi, rondelle e i sottopezzi del modello ufficiale del gruppo UPS. Filetti standard semplificati nei pezzi generati; non sono eliche da stampare.
- [Distinta con link ai fornitori](distinta-acquisti.md) e [CSV](distinta-acquisti.csv): acquisti, componenti inclusi nei kit e parti da produrre sono distinti.
- [Note di lavorazione CNC](manufacturing-notes.md): coordinate dei fori, filetti e svasature. Queste specifiche accompagnano obbligatoriamente gli STEP nominali.
- [Revisione strutturale](structural-review.md): equilibrio statico, screening analitico degli spessori e limiti dei calcoli.
- [Revisione comandi e passaggi](review-controls.md): controlli degli innesti e misure ancora da confermare.
- [Verifica geometrica automatica](verification.json): **444 solidi**, STEP reimportato valido e **15 STL** chiusi con orientamento coerente. Il [controllo delle interferenze](interference-audit.json) riporta zero intersezioni inattese nelle 1.004 coppie candidate: verifica completa, seguita dal ricalcolo dei tre pezzi modificati nell'ultimo percorso USB-C. I contatti interni al kit ufficiale UPS sono esclusi; gli innesti elettrici intenzionali sono elencati separatamente.

I file della cartella `stl/` sono soltanto parti in plastica da stampare; i metalli e la viteria si acquistano o lavorano separatamente. Gli STL conservano le coordinate dell'assieme: lo stampatore deve orientare e posare ogni pezzo sul piano. Le quote nominali non incorporano una compensazione specifica della macchina o del materiale.

## Fornitori e parti custom

La cella è disponibile tramite [Essmann](https://www.essmann-shop.com/en/single-point-load-cell-zemic-l6d-c3-30kg-0.4b/11250496); i tastierini tramite [BerryBase](https://www.berrybase.de/en/membrane-keypad-4-keys-without-labelling-with-adhesive-layer); il passapannello tramite [Adafruit](https://www.adafruit.com/product/6069) oppure direttamente [Premier Cable, Cina](https://www.premier-cable.store/products/panel-mount-usb-3-1-type-c-waterproof-cable). Per l'alimentazione è indicato il [kit Waveshare cinese 25507](https://www.waveshare.net/shop/UPS-HAT-D.htm), comprendente le batterie.

**Piatto, guscio, cornice e supporti non sono ricambi generici da marketplace:** sono i pezzi a disegno allegati. [JLC3DP](https://jlc3dp.com/) è un fornitore cinese per la stampa PA12; [JLCCNC](https://jlccnc.com/) per i quattro pezzi in alluminio e la valutazione delle lavorazioni custom. I file possono essere forniti anche a un'officina e a uno stampatore locali. Non sono stati caricati presso terzi, né sono stati effettuati ordini.

## Sequenza di montaggio prevista

1. Lavorare e sbavare basamento, portapiatto e distanziali secondo le note CNC. Controllare la planarità delle zone che stringono la cella e del piano che sostiene la plastica.
2. Avvitare i quattro piedini nei filetti M6 del fondo e montare i controdadi. Inserire i quattro grani di arresto; tenerli inizialmente arretrati per non influenzare la prima caratterizzazione.
3. Fissare il lato cavo della cella al distanziale da 6 mm e al fondo con due M6×25 e rondelle. Collegare il lato mobile al distanziale da 22 mm. Serrare alla coppia confermata per la cella e per la specifica giunzione.
4. Montare i vassoi isolanti e il gruppo UPS con i suoi ritegni. Disporre le schede esistenti sul vassoio elettronica; definire i loro attacchi dopo aver misurato i PCB posseduti. Fermare i cavi sul fondo, lasciando libero il corpo deformabile della cella.
5. Assemblare la cornice OLED, il vetrino e il supporto posteriore. Adattare il carrier al display reale, senza serrare componenti o connettori del PCB. Incollare le due membrane nelle sedi e far passare le code attraverso i passaggi protetti.
6. Inserire la USB-C dall'interno, con flangia e O-ring interni e dado esterno. Nella variante Waveshare rappresentata dal CAD, collegare la prolunga all'ingresso USB-C dell'UPS; l'uscita USB-A alimenta il cablaggio 5 V. Questa variante richiede la verifica elettrica e l'adattamento firmware descritti sotto.
7. Inserire i dadi M3 nelle sedi captive del guscio, chiudere la scocca e fissarla dal fondo. Collegare il portapiatto al distanziale mobile con due M6×45 svasate, quindi fissare il piatto plastico con sei M3×10 svasate.
8. Livellare i piedi. Con masse note crescenti, misurare il comportamento al centro e ai bordi, i giochi verso scocca e arresti e il ritorno allo zero. Regolare gli arresti sulla deformazione reale; bloccare i controdadi senza precaricare il piatto.

L'accesso alle schede richiede lo smontaggio di piatto/portapiatto e guscio. Non sollevare la bilancia afferrando il piatto: la cella riceverebbe un carico di segno opposto.

## Cosa resta da confermare prima della produzione funzionale

1. **Display e schede possedute.** Nel repository sono identificati i moduli elettrici, non tutti gli ingombri e interassi dei PCB. La cornice OLED e il vassoio sono predisposizioni sostituibili; i singoli attacchi non possono essere dichiarati compatibili con moduli non misurati.
2. **Campioni commerciali e stampa.** Verificare uscita/pinout delle code BerryBase, sagoma reale di dado e tappo USB, giochi delle sedi captive e adesione delle membrane sul PA12 scelto. Il percorso USB-C misura circa 303,7 mm, rispetto ai 300 ±15 mm dichiarati: deve essere adattato alla lunghezza effettiva del campione, specialmente se corta. Scegliere compensazioni di stampa con il fornitore.
3. **Cella e carico.** Confermare con Zemic orientamento ammesso del piatto 280×200 e coppia di serraggio per 30 kg, quindi collaudare i 20 kg utili, anche eccentrici. Una lettura da 1 g non equivale ad accuratezza da 1 g; il solo errore combinato dichiarato della cella C3 equivale a 6 g sul fondo scala.
4. **Variante batteria nel CAD.** Il modello presenta il gruppo Waveshare con due 21700, non la NASTIMA e il Mini360 già installati. I suoi ingombri e fissaggi non attestano la compatibilità con la batteria posseduta. Il firmware attuale usa tacche indicative LiFePO4 2S e protezioni a 5,80/5,70 V: l'eventuale adozione della variante Waveshare richiede un intervento elettrico/firmware separato. Vedere [ricarica e alimentazione](research-power.md), compreso il limite di corrente non pubblicato della prolunga USB-C.

La validità del CAD e degli STL non dimostra la portata fisica o la precisione della bilancia. Gli accoppiamenti verificati sono nominali e i cablaggi flessibili rappresentano percorsi di progetto.

## Riproducibilità

Generatore [generate_assembly.py](generate_assembly.py), unità millimetri, assi X larghezza, Y profondità (fronte negativo), Z verticale. Ambiente verificato: Python 3.12, CadQuery 2.8, trimesh. Il modello originale Waveshare resta sotto `sources/`.

```powershell
python -m pip install cadquery==2.8.0 trimesh
python docs/mechanical/concept-02/generate_assembly.py
python docs/mechanical/concept-02/build_bom.py
python docs/mechanical/concept-02/structural_screening.py
```

L'anteprima leggera usa `build_preview.py` e richiede inoltre `fast-simplification`. Riduce le triangolazioni e omette i componenti elettronici vendor più piccoli soltanto per la visualizzazione; non modifica STEP e STL. Nessuna modifica al firmware, al pinout o alla configurazione elettrica installata in questa proposta.
