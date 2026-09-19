# Bilance donatrici per un solo prototipo economico

Ricerca del 7 settembre 2026; prezzi e disponibilità letti sui siti dei venditori, senza acquisti, contatti o inserimento di un indirizzo di consegna. Obiettivo: riutilizzare il gruppo di pesatura di una bilancia completa e progettare soltanto le plastiche necessarie. Nessun nuovo CAD prodotto in questa ricerca.

**Prima scelta: Steinberg SBS-PW-301CA nuova, 43 € IVA e spedizione incluse.** Ha ingombri vicini alla EVA 9901 e costa meno della sola cella del Concept 02. È una candidata concreta da aprire e misurare: le fonti identificano ricambi separati, ma non pubblicano gli innesti necessari per disegnare già adesso una scocca compatibile.

## Confronto per un esemplare

| Donatrice e fonte diretta | Prezzo IVA inclusa | Spedizione pubblicata | Totale leggibile | Disponibilità | Dimensioni dichiarate e interesse |
|---|---:|---:|---:|---|---|
| [Steinberg SBS-PW-301CA, 30 kg, articolo 10030317 — Expondo](https://www.expondo.it/steinberg-systems-selezionatrice-ponderale-30-kg-1-g-28-8-x-21-8-cm-lcd-10030317) | **43,00 €**, aliquota 22%; nuova | Gratuita | **43,00 €** | Disponibile, ordine da 1 pezzo; consegna indicativa 15 settembre | Piatto inox removibile **288 × 218 mm**; bilancia **285 × 275 × 130 mm**. Quattro piedini inclusi. Divisione pubblicizzata 1 g. Miglior compromesso fra prezzo e formato. |
| [EVA 033289, 30 kg — RagStore](https://www.ragstore.it/professionali/24753-bilancia-digitale-pesapacchi-eva-kg-25-8004537332896.html) | **25,00 €** | Corriere espresso **7,30 €** mostrati dalla pagina | **32,30 €**, da riconfermare con destinazione reale | Ultimi articoli in magazzino, ordine da 1 pezzo | Ingombro descritto **240 × 155 × 34 mm**; divisione 2 g. È economica, ma molto più piccola della 9901: richiede accettare un piatto più contenuto. |
| [EVA 9901 / 033291, 30 kg — Cose&Cose](https://www.cosecose.com/articolo/0021678/bilancia-elettronica-pesapacchi-30kg) | **134,30 €**, variante 7091308 | Minimo **7,90 €**; costo effettivo calcolato per peso/volume | **Almeno 142,20 €**; totale effettivo non pubblicato | Pronta consegna; confezione da 1 pezzo | Piatto **280 × 200 mm**; ingombro di riferimento **280 × 300 × 110 mm**. È la donatrice più fedele al formato richiesto, ma non la più conveniente. |

La portata e la divisione sono quelle pubblicizzate per la bilancia completa: **1 g sul display non dimostra un errore massimo di 1 g** dopo il cambio di elettronica o di piatto. La quantità disponibile può cambiare; i prezzi promozionali non sono un preventivo prenotato.

Per Cose&Cose, l'IVA è inclusa secondo le [condizioni di vendita](https://www.cosecose.com/condizioni-di-vendita); il minimo di spedizione deriva dalla [pagina consegne](https://www.cosecose.com/spedizioni-e-consegna). Il prezzo di 7 € presente nella pagina prodotto riguarda il solo accessorio di ricarica, non la bilancia. L'ingombro della 9901 viene dalla [scheda Kaufgut del modello, pubblicata da ROS](https://ros.bergamo.it/a/0189c84f-15d5-459b-8975-dcb0aba4d424).

Per EVA 033289, EAN 8004537332896, la [scheda del marchio Kaufgut](https://www.kaufgut.it/it/bilancia-pesapacchi-digitale-inox-30kg-2g-eva/202310033289) ripete 240 × 155 × 34 mm nella descrizione ma riporta 285 × 165 × 50 mm nei campi dimensionali: non è esplicitato se questi ultimi siano l'imballo. Misurare l'esemplare; non usare il secondo terzetto come quota CAD certa. La superficie utile del piatto non è quotata separatamente.

## Cosa sappiamo realmente della Steinberg proposta

Il [catalogo ricambi ufficiale per 10030317](https://manuals.expondo.com/spare-parts/10030317), nella versione verificata datata 7 settembre 2026, identifica separatamente:

- sensore estensimetrico **19012828**;
- piatto plastico **19006733**, descritto come 22 × 29 cm;
- guscio inferiore **19009893**, guscio superiore **19006267**;
- piedini **19001056** e **19013691**.

Questo supporta la scelta di un prodotto smontabile con parti sostituibili. Il catalogo non quota sensore, supporti, fori o geometria del gruppo portante; non dimostra un telaio metallico completo già estraibile. I ricambi non hanno un prezzo pubblico nel PDF e la giacenza riportata non è una conferma d'ordine. Comprare singolarmente cella, fondo e portapiatto non ha quindi un costo dimostrato inferiore ai 43 € della bilancia completa.

## Come ridurre davvero il costo di costruzione

Per il primo esemplare propongo di conservare **fondo originale, cella, collegamenti portanti, portapiatto e piedini come gruppo**, mantenendo gli appoggi con cui il prodotto sostiene il peso. La nuova scocca può rivestire questo insieme e portare il display già posseduto, gli otto tasti e la USB. Il nuovo piatto in plastica dovrà appoggiarsi al portapiatto originale, con quota, corsa libera e fissaggi ricavati dal campione. Non occorre acquistare un altro display.

Questo evita i quattro pezzi CNC dedicati del Concept 02. Come riferimento economico, la [cella Zemic L6D 30 kg selezionata in quel concept](https://www.essmann-shop.com/en/single-point-load-cell-zemic-l6d-c3-30kg-0.4b/11250496) era quotata **56,60 € senza IVA**, cioè **69,05 € applicando il 22%**, prima di spedizione, supporti e piedini. È il riferimento documentato del concept, non un nuovo preventivo: la donatrice Steinberg intera costa circa 26 € meno di quella sola cella. Non viene attribuito alcun credito ipotetico agli LCD o all'elettronica della donatrice, che non servono al progetto.

Prima di congelare gli innesti della nuova plastica servono il campione aperto e poche misure decisive: appoggi del portapiatto, posizione e fissaggi della cella, piedi/fondo, corsa sotto carico e spazio per le schede. Va identificato il collegamento elettrico del sensore per verificarne l'uso con HX711. Nessuna delle tre fonti permette di garantire oggi questi accoppiamenti o la rigidità del nuovo piatto a 20 kg. Le verifiche a carico centrale ed eccentrico riguarderanno il prototipo assemblato.

Se una bilancia già posseduta è disponibile per essere destinata al prototipo, usare quella può azzerare la spesa per il donatore. La proposta da 43 € serve quando occorre acquistare un esemplare dedicato; **non è il costo totale della bilancia personalizzata**, che comprende ancora stampa, tastierini, alimentazione e minuteria aggiunta.
