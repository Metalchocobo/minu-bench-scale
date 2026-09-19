# Un prototipo economico: stampa FDM e confronto Weerg

Stato: ricerca di fattibilità economica, verificata il 7 settembre 2026. Quantità richiesta: **un solo esemplare**. Nessun file è stato caricato presso fornitori, nessun venditore contattato e nessun preventivo del progetto ottenuto. Il CAD concept-02 resta un assieme nominale ibrido; questa nota non lo trasforma in un progetto integralmente stampabile né ne verifica la portata di 20 kg.

## Scelta pratica

La strada da approfondire è **3–5 componenti FDM in ABS o PETG, minuteria acquistabile e niente quattro lavorazioni CNC su misura**. ASA è un'alternativa se servono resistenza agli UV o specifiche termiche. Per il singolo prototipo la convenienza va confrontata sul prezzo del lotto completo, con finitura grezza e tempi non urgenti. Il PA12 industriale non è il materiale predefinito di questa proposta.

ABS è un candidato concreto, anche presso Weerg. L'aspetto della plastica delle bilance commerciali non prova però l'equivalenza fra una scocca stampata a iniezione e la stessa forma prodotta a filamento: orientamento degli strati e geometria dei collegamenti devono far parte del progetto.

## Che cosa offre Weerg

Il catalogo pubblico distingue chiaramente **stampa MJF/FDM/resina**, **lavorazione CNC dell'alluminio** e **taglio laser**. Non è stata trovata un'offerta Weerg verificabile di stampa metallica SLM/LPBF. Il riferimento dell'utente è quindi utile per ottenere componenti in alluminio online, ma non prova che siano stampati. [Catalogo stampa Weerg](https://www.weerg.com/it/stampa-3d-online), [alluminio CNC Weerg](https://www.weerg.com/it/materiali-lavorazioni-cnc/alluminio).

| Processo | Che cosa produce | Implicazione per il prototipo |
|---|---|---|
| FDM | Termoplastico depositato per strati, fra cui ABS/ASA e PETG secondo il servizio | Prima opzione da quotare per scocca, base e piatto riprogettati |
| MJF | Parti plastiche da polvere; Weerg propone, fra gli altri, PA12 e PA11 | Alternativa industriale da confrontare a preventivo, senza presumerla necessaria |
| SLS polimerico | Polvere plastica consolidata con laser | Processo diverso da FDM e da stampa dell'alluminio; non identificare automaticamente SLS e MJF |
| SLM/LPBF metallico | Metallo ottenuto da polvere mediante laser | È una tecnologia esistente per alluminio, ma non un servizio Weerg confermato da questa ricerca |
| CNC alluminio | Materiale asportato da un grezzo | Produce il pezzo disegnato su misura; non è un supporto già pronto a catalogo |

Riferimenti tecnici per la distinzione: [SLS secondo Formlabs](https://formlabs.com/blog/what-is-selective-laser-sintering/), [alluminio additivo EOS](https://www.eos.info/metal-solutions/metal-materials/aluminium), [LPBF secondo EOS](https://3d.eos.info/-eos-additive-manufacturing-v2).

**Quantità minima:** Weerg accetta un singolo pezzo sia stampato sia CNC. La FAQ descrive per CNC un caricamento STEP e una richiesta di offerta valutata da un esperto; non bisogna promettere che qualsiasi configurazione CNC riceva subito un prezzo automatico. Non è stato verificato un importo minimo generale dell'ordine: quantità minima 1 non significa minimo monetario zero. [FAQ prototipi e quantità](https://www.weerg.com/faq/order-prototypes-and-large-runs).

**Preventivo:** per la stampa servono file, materiale, finitura e data; STEP/STP/STL sono accettati fino a 150 MB. Il prezzo reale richiede l'invio della geometria. Non esiste in questa ricerca un prezzo Weerg dimostrato per la bilancia né un tariffario pubblico universale €/g applicabile al progetto. [Procedura di ordine](https://www.weerg.com/it/faq/come-si-realizza-un-ordine-su-weerg), [preventivazione stampa](https://www.weerg.com/it/stampa-3d-online).

### ABS e ASA Weerg: formato e tolleranze da risolvere

La pagina **ABS Alimentare** indica FDM, ingombro massimo **300×300×300 mm** e tolleranze **±0,60 mm sotto 100 mm; ±0,75% sopra 100 mm**. A titolo aritmetico, 0,75% di 280 mm è 2,1 mm: gli accoppiamenti nominali stretti del concept-02 non sono quindi già convalidati per questa offerta. Il nome alimentare non certifica automaticamente l'assieme finito. [Pagina ABS Weerg](https://www.weerg.com/it/materiali-stampa-3d/abs/abs-alimentare-food).

Anche la pagina **ASA** riporta 300×300×300 mm e le stesse tolleranze. La tabella tecnica generale pubblicata al percorso rev4.0 riporta invece per ABS un formato teorico 300×300×400 mm: è una discordanza delle fonti, da risolvere nel configuratore o con il servizio tecnico prima di progettare intorno al limite maggiore. [Pagina ASA](https://www.weerg.com/it/materiali-stampa-3d/abs/asa), [tabella tecnica Weerg](https://www.weerg.com/hubfs/Datasheets/Weerg%20Scheda%20Tecnica%20Materiali%20rev4.0-ITA.pdf).

Il corpo attuale 300×320 mm **non entra piatto** su un piano 300×300 mm. La possibilità di ruotarlo nello spazio non dimostra che quell'orientamento sia economico, privo di supporti o strutturalmente adatto. Per Weerg ABS occorre verificare orientamento e formato effettivo, oppure ridurre l'ingombro con un nuovo progetto; non dividere la base portante solo per aggirare il limite macchina.

## Servizi FDM per un solo esemplare

Le dimensioni dichiarate sono capacità del servizio, non approvazioni del nostro file. Materiale, colore e macchina disponibili insieme richiedono comunque verifica del preventivo.

| Servizio e fonte primaria | Evidenza quantità/materiale/formato | Prezzo pubblico e limite della verifica |
|---|---|---|
| [Weerg, Italia](https://www.weerg.com/it/stampa-3d-online) | Un pezzo; ABS e ASA FDM. Formato discordante come sopra. PETG ESD a catalogo: non equivale a PETG economico standard | Configuratore su file. Nessuna cifra del progetto verificata |
| [AManTech, Lucca](https://www.amantech.it/) | Un solo pezzo esplicitamente accettato; PETG FDM; spedizione italiana. Formato utile e disponibilità ABS/ASA non verificati | Prezzo automatico caricando STL; nessuna tariffa numerica generale verificata |
| [Eksaria, Zagabria](https://eksaria.com/) | Nessun minimo di quantità, PETG/ASA, macchina fino a 500×500×500 mm, consegne UE | Calcolatore basato su slicing; nessun prezzo senza file. Il corpo attuale rientra nell'inviluppo massimo dichiarato |
| [Imprimakers, Valencia](https://imprimakers.com/en/fdm-3d-printing/) | Nessun minimo di quantità, ABS/ASA/PETG, pezzo singolo fino a 500×500×500 mm, spedizione UE | Calcolatore online; esempio pubblico di custodia ABS da 200 g circa €50. È un esempio diverso dalla bilancia, non un prezzo proporzionale garantito |
| [LayerCrew, Romania](https://layercrew.com/pricing) | Nessun minimo di ordine; preventivatore immediato PETG/PLA, ABS/ASA su offerta | Tariffa pubblica utile come benchmark. Compatibilità macchina con corpo 300×320 mm non verificata, quindi non selezionato per il corpo senza ulteriore controllo |

## Quanto possiamo dire sui costi senza inviare file

I volumi geometrici del [concept-02](../concept-02/verification.json) danno soltanto una scala dimensionale:

| Parte attuale | Volume CAD |
|---|---:|
| Scocca esterna plastica | 404,405 cm³ |
| Copertura piatto plastica | 222,315 cm³ |
| Totale delle due plastiche principali | **626,720 cm³** |
| Base attuale in alluminio | 571,643 cm³ |
| Portapiatto attuale in alluminio | 429,918 cm³ |

Il volume del solido CAD non è la massa di filamento dello slicer: riempimenti, perimetri, supporti e orientamento cambiano il consumo. Mancano inoltre la nuova base nervata e il nuovo piatto strutturale. Sostituire semplicemente il materiale alle due piastre metalliche attuali non è una soluzione verificata.

Imprimakers pubblica inoltre un esempio **PETG da circa 100 cm³ a €19 IVA inclusa**, con riempimento standard del 20%; indica un minimo monetario d'ordine di €4,50. Questa configurazione economica non è una specifica strutturale per la bilancia: moltiplicare automaticamente €0,19/cm³ per i volumi sopra non produce un preventivo attendibile. Il prezzo di ingresso di pochi euro riguarda pezzi piccoli. [Tariffe Imprimakers 2026](https://imprimakers.com/it/prezzi-stampa-3d/).

LayerCrew pubblica un esempio PETG a €42/kg di materiale, €1/ora macchina e €50/ora operatore, esclusa IVA; il prezzo effettivo usa gli SKU correnti. Applicando soltanto quella formula a ipotesi dichiarate si ottiene: [tariffario primario](https://layercrew.com/pricing).

| Scenario ipotetico complessivo | Materiale | Macchina | Operatore | Risultato matematico, IVA esclusa |
|---|---:|---:|---:|---:|
| 1 kg, 25 h stampa, 30 min interventi | €42 | €25 | €25 | €92 |
| 2 kg, 50 h stampa, 60 min interventi | €84 | €50 | €50 | €184 |

**Questi due scenari non sono una stima validata della bilancia, né un preventivo o una fascia promessa.** Masse e tempi sono assunti; non derivano da slicing. Non includono spedizione, minuteria, elettronica, revisione del progetto o rilavorazioni. Servono a capire perché ridurre pezzi separati, supporti e interventi manuali incide sul costo. Non trasferire questo tariffario PETG a Weerg o all'ABS.

## Ridurre la complessità a 3–5 stampati

Architettura da progettare, non geometria già pronta:

1. **Base portante monopezzo:** superficie interna piana; nervature profonde solo sotto, raccordi ai nodi e attacchi piedi integrati. Sedi di dadi commerciali e bulloni passanti distribuiscono il serraggio. La rigidezza viene dall'altezza delle nervature e dal percorso del carico, non dal riempimento generico.
2. **Guscio rimovibile:** integrare fascia comandi, sede display e coperture dei passacavi; evitare tanti piccoli pezzi estetici e clip singole. Preservare accesso al montaggio e allo smontaggio.
3. **Piatto mobile plastico:** riprogettare il fondo nervato e l'attacco alla cella. Una semplice copertura attuale da 4 mm, privata del carrier metallico, non è un piatto portante da 20 kg.
4. **Una traversina/removibile fermaschede e batteria**, solo se necessaria: base interna liscia e componenti esistenti di dimensioni variabili richiedono comunque un ritegno positivo, non solo appoggio.
5. **Un adattatore della cella**, se indispensabile. Preferire distanziali/boccole e minuteria standard realmente quotati; un generico profilo acquistabile non garantisce automaticamente gli appoggi corretti della cella.

Con questa architettura si eliminano le quattro parti CNC su misura soltanto dopo aver ridisegnato il percorso strutturale. Piedi, viti, rondelle larghe, dadi e boccole commerciali restano compatibili con la richiesta di una scocca interamente stampata. I fermi di sovraccarico restano distanti dal piatto durante la misura.

## ABS, PETG o ASA e rischi da verificare

PETG è un riferimento economico per un prototipo da interno: buona adesione fra strati e basso ritiro facilitano le parti grandi. Il maggiore numero di perimetri può essere più utile del solo aumento di infill, ma nessuna percentuale dimostra la portata. [Guida PETG del produttore Prusa](https://help.prusa3d.com/article/petg_2059).

ABS è sensato se il servizio dispone di un processo controllato per scocche grandi e lo quota convenientemente. ASA aggiunge resistenza agli UV; resta soggetto a ritiro e deformazioni durante la stampa, soprattutto su grandi superfici, e richiede condizioni termiche controllate. [Guida ASA Prusa](https://help.prusa3d.com/article/asa_1809), [ABS FDM Weerg](https://www.weerg.com/it/materiali-stampa-3d/abs/abs-alimentare-food).

Per la nuova struttura restano da verificare, con geometria e materiale specifici:

- **Flessione:** carico centrale ed eccentrico da 20 kg sul vero piatto, con deformazione rispetto alla scocca e ai fermi; non basta che il pezzo non si rompa.
- **Serraggio e durata:** schiacciamento locale, separazione degli strati e deformazione lenta sotto precarico o peso sostenuto possono cambiare l'allineamento della cella. Prevedere bulloni passanti e appoggi ampi, senza imporre strutture metalliche su misura.
- **Metrologia:** ritorno a zero, isteresi e sensibilità al punto di applicazione dopo carico e scarico. Una prova di resistenza meccanica non dimostra precisione di 1 g.
- **Stabilità:** mantenere l'impronta dei piedi estesa e verificare il baricentro dell'assieme leggero finale; i margini del concept-02 che includono piastre metalliche non si trasferiscono automaticamente alla variante tutta plastica.
- **Planarità e assemblaggio:** base e piatto grandi, tolleranze del fornitore, accessibilità viti e gioco della parte mobile. I controlli CAD nominali già eseguiti non misurano le distorsioni di stampa.

Il passo utile successivo è scegliere un formato compatibile con ABS Weerg oppure con un servizio FDM da 500 mm, ridurre le parti nel CAD e ottenere un prezzo reale dei soli stampati QTY1. La portata richiesta rimane 20 kg; non è ancora una prestazione fisicamente verificata.
