# Alimentazione tampone per una bilancia con batteria al piombo

Studio di una soluzione SLA **non adottata**. La configurazione montata usa batteria USB-C NASTIMA LiFePO4 e Mini360 MP1482DS: vedere [cablaggio corrente](../docs/WIRING.md). Requisiti, confronti, prezzi e disponibilità sotto riportati appartengono alla ricerca sul piombo e non costituiscono lo stato dell'impianto o una nuova verifica commerciale.

## Esito del confronto per la variante SLA

Esistono schede assemblate che alimentano un apparecchio e contemporaneamente ricaricano una batteria al piombo da 6 V. La proposta economica più concreta è **SECO-LARM ST-2406-2AQ**: il negozio italiano IoTiVedo espone **19,76 € IVA inclusa**, dieci unità disponibili e spedizione rapida. Le spese di trasporto partono da 6,10 €; il costo iniziale è quindi almeno **25,86 € consegnato**, salvo condizioni specifiche dell'ordine.[^1][^2]

Il prodotto comprende alimentazione del carico, ricarica e passaggio automatico alla batteria. Non costituisce però, da solo, un'alimentazione completa già qualificata per questa bilancia: restano conversione a 5 V, protezione dalla scarica e verifica dei consumi. La ricarica a 6 V è limitata a **100 mA**, mentre la corrente disponibile per il carico è molto maggiore. Confondere queste due correnti porterebbe a scegliere il prodotto sbagliato.[^3][^4]

**La scelta consigliata è condizionata all'impiego:** questo modulo merita considerazione se la bilancia rimane spesso collegata e usa la batteria durante gli spostamenti. Per otto ore scollegate ogni giorno, seguite da una sola notte di ricarica, la decisione dipende dall'energia realmente consumata. Una scarica quotidiana di diversi Ah lo esclude; una scarica contenuta può renderlo sufficiente.

Non emerge una scheda pronta, documentata e facilmente acquistabile nella fascia indicativa di 30–50 € che soddisfi contemporaneamente tutti i requisiti: SLA 6 V, ingresso 12 V DC, carica sufficientemente rapida, basso assorbimento a riposo dichiarato, stacco conservativo e avvio garantito dopo lo stacco. Questo è un limite delle offerte verificate, non la dimostrazione che il circuito sia tecnicamente impossibile.

## Requisiti e dati ancora necessari

| Aspetto | Vincolo di progetto |
|---|---|
| Batteria | Piombo sigillato, interna alla bilancia; riferimento attuale 6 V |
| Alimentazione esterna | Alimentatore attuale 12 V, 2,5 A; sostituzione possibile |
| Elettronica alimentata | ESP32 e periferiche, linea di alimentazione a 5 V |
| Autonomia | Circa otto ore disponibili, senza assumere otto ore di scarica quotidiana |
| Uso | Bilancia portatile da banco, impiego quotidiano in gelateria |
| Avvio | Collegando l'alimentazione, funzionamento anche con batteria normalmente scarica |
| Consumo proprio | Ridotto anche quando l'elettronica della bilancia dorme |
| Acquisto | Preferenza Italia/UE, costi contenuti e consegna senza lunga importazione |

La potenza nominale dell'alimentatore indicato è **30 W**: 12 V × 2,5 A. La scritta «300 W» non sarebbe coerente con questi dati. Questa potenza non stabilisce comunque la corrente di ricarica: è il caricatore a limitarla.

Mancano il modello e la capacità in Ah della batteria, le dimensioni libere misurate e il consumo medio reale. «Classica batteria da 6 V» non identifica un formato unico. Per esempio, la Yuasa NP4.5-6 misura circa 70 × 47 × 105,5 mm e immagazzina nominalmente 27 Wh; è un riferimento, non l'identificazione della batteria installata.[^5]

Un'uscita capace di avviarsi con una batteria normalmente scarica non equivale alla capacità di funzionare con una batteria guasta o in cortocircuito. Quest'ultimo è un caso di protezione da guasto, da trattare separatamente.

## Moduli confrontati

| Prodotto o famiglia | Elementi pertinenti | Limite per questa applicazione | Valutazione |
|---|---|---|---|
| **SECO-LARM ST-2406-2AQ** | Modulo pronto; compatibilità SLA 6 V; ingresso DC ammesso; carico e batteria separati | Ricarica lenta; consumo a riposo non pubblicato; assenza di stacco per batteria scarica | **Candidato economico per uso tampone** |
| **SECO-LARM ST-2406-3AQ** | Variante con protezione da batteria scarica | Non migliora la corrente di ricarica; soglia descritta in modo ambiguo nel manuale | Non risolve il principale limite del modello economico |
| **Altronix AL624 / SMP3** | Alimentatori tampone reali, con configurazione a 6 V | Ingresso documentato da trasformatore AC; carica circa 0,3 A; costo complessivo sfavorevole | Scartati come sostituti semplici del 12 V DC |
| **H-Tronic 1191418** | Modulo regolabile fino a 500 mA; 6,9 V previsti per SLA 6 V | Documentazione reperibile, disponibilità attuale non verificata; manca gestione completa del carico | Categoria pertinente, acquisto non concreto |
| **Velleman K8012** | Carica e mantenimento per piombo 6/12 V | Kit da saldare, dismesso, ingresso 18 V AC e dimensioni rilevanti | Scartato per reperibilità e semplicità |
| **Mascot 2544 LA, versione 6 V** | Caricatore DC/DC professionale, ingresso 10–30 V | Versione standard da 2,7 A da confrontare con il limite della piccola SLA; nessun PowerPath completo verificato | Non è la soluzione economica pronta cercata |
| **PowerStream PST-BC1206-15** | Vero caricatore DC 12 V → piombo 6 V | 185 USD; circa 200 × 164 × 67 mm, 1,5 kg | Scartato per costo e ingombro |
| **ADI DC2038A-H / LTC4162-S** | Gestione del carico con priorità e avvio anche con batteria scarica o assente | Scheda di valutazione da configurare; circa 112 € prima delle condizioni fiscali/spedizione | Riferimento tecnico valido, fuori obiettivo economico |
| **DFRobot DFR0580 / CN3767** | Modulo documentato con uscite e protezioni | Progettato per batteria al piombo **12 V**, non 6 V | Non compatibile con la batteria attuale |
| **CN3791 / CN3795 / SD30CRMA** | Schede piccole ed economiche | CN3791 «6 V» si riferisce spesso al pannello; CN3795 regolabile non equivale a un profilo piombo documentato | Non consigliati come caricabatterie SLA già verificati |
| **WinSystems PCM-PS394-500** | Architettura OEM vicina al requisito: batteria e uscita 5 V | Produttore dichiara il prodotto fuori produzione | Scartato per reperibilità |

Le caratteristiche della tabella provengono da manuali o pagine dei produttori; prezzi e disponibilità provengono dai relativi canali di vendita.[^3][^4][^6][^7][^8][^9][^10][^11][^12][^13][^14][^15][^27]

### SECO-LARM: cosa offre realmente

Il manuale ammette **12–24 V AC/DC in modalità 6 V**. Per ST-2406-2AQ indica **1,5 A continui al carico**, **100 mA massimi di carica**, dimensioni **73 × 64 × 35 mm** e nessun cutoff della batteria. Il selettore deve essere configurato a 6 V; l'impostazione di fabbrica indicata è 12 V.[^4]

Sono documentate ricarica e commutazione automatica. Mancano invece un valore di corrente assorbita dalla batteria senza carico, un tempo massimo di commutazione e un test esplicito di avvio dopo l'intervento di un cutoff esterno. Non si può quindi affermare che consumi meno del controller attuale o che l'ESP32 non si resetti durante la commutazione senza una verifica sul sistema completo.

Il manuale descrive un trimmer di regolazione della tensione d'uscita, ma non fornisce un intervallo utile verificato per risolvere ogni combinazione con lo stacco a valle. Non va modificato alla cieca: la tensione effettiva ai morsetti della batteria deve restare conforme alle sue specifiche.[^4]

### Offerte e costo reale

| Articolo | Prezzo osservato | Disponibilità dichiarata | Precisazione |
|---|---:|---|---|
| SECO-LARM ST-2406-2AQ, IoTiVedo | **19,76 € IVA inclusa** | Dieci pezzi; spedizione rapida | Trasporto da 6,10 €; consegna normalmente 24–48 ore lavorative dall'evasione secondo le condizioni del negozio |
| Pololu 2870, Opencircuit | **16,45 € sul sito con IVA NL** | 169 pezzi in magazzino esterno; 5–7 giorni indicati | IVA di destinazione, trasporto e data per l'Italia da confermare prima dell'ordine |
| Altronix SMP3, Raptor Supplies UE | **98,26 € IVA esclusa** | Undici unità; spedizione in due giorni indicata | Richiede anche un ingresso AC appropriato |
| ADI DC2038A-H, Mouser Italia | **111,68 € esposti** | Quattro unità indicate | Trattare IVA e trasporto come aggiuntivi/non confermati nel confronto finale |
| PowerStream PST-BC1206-15 | **185 USD** | Canale statunitense | Esclusi importazione e trasporto; formato non adatto |

Rilevazione delle pagine: **10 settembre 2026**. Sono condizioni dichiarate dai venditori, non prenotazioni o date di consegna garantite.[^1][^2][^10][^12][^16][^17]

SECO-LARM e Pololu sommati costano circa **36 € prima di spedizioni e accessori**, ma non vanno presentati come un kit completo già validato. Un preventivo realistico deve includere portafusibile, connessioni, isolamento, montaggio ed eventuale circuito per il riavvio. Il totale finale non è verificato. Su Amazon.it non è stata verificata un'offerta esatta equivalente che soddisfi tutti i vincoli; una disponibilità italiana controllabile è preferibile a un'inserzione generica.

## Otto ore di autonomia e velocità di ricarica

L'autonomia misura quanto a lungo si può usare la batteria. Il tempo di ricarica dipende da quanta energia si è effettivamente prelevata. Una giornata con frequenti periodi in standby è diversa da otto ore con display, Wi-Fi e audio attivi.

Per valutare il limite di 100 mA, si possono usare questi **esempi di calcolo**, non misure della bilancia. Si assumono 6 V medi della batteria e rendimento complessivo batteria → linea 5 V pari all'85%:

| Corrente media sulla linea 5 V | Energia in otto ore | Carica prelevata stimata dalla SLA | Tempo minimo teorico a 100 mA |
|---:|---:|---:|---:|
| 100 mA | 4 Wh | 0,78 Ah | 7,8 ore |
| 200 mA | 8 Wh | 1,57 Ah | 15,7 ore |
| 400 mA | 16 Wh | 3,14 Ah | 31,4 ore |

Formule: `E = 5 V × I × 8 h`; `Q = E / (6 V × 0,85)`; `t_min = Q / 0,1 A`.

I tempi reali sono maggiori: il calcolo non include le perdite di ricarica, la riduzione della corrente nella fase finale, l'invecchiamento o un assorbimento supplementare del sistema. Sedici ore collegate consentono al massimo teorico di restituire 1,6 Ah. **Con 200 mA medi a 5 V, il bilancio è già privo di margine; con 400 mA medi non torna.** Con consumo inferiore o più ore collegate, il giudizio cambia.

Anche la capacità nominale non è tutta automaticamente utilizzabile mantenendo la batteria in buone condizioni. La scelta finale deve considerare il tasso di scarica, la tensione minima ammessa, la temperatura e la profondità di scarica prevista. Una batteria più grande aumenta l'autonomia, ma non aumenta la velocità con cui un caricatore da 100 mA reintegra l'energia consumata.

## Conversione a 5 V e spegnimento automatico

Il LM2596 è un convertitore che può soltanto abbassare la tensione. Quando una SLA da 6 V scende, soprattutto con diodi e cablaggio in serie, può perdere il margine necessario per mantenere 5 V. La tensione minima di funzionamento del chip non garantisce 5 V d'uscita a qualsiasi carico. Per questa applicazione è preferibile un buck-boost dimensionato sul consumo reale.[^18]

Il **Pololu 2870 S9V11F5S6CMA** abbina uscita a 5 V e soglia di stacco regolabile. Il produttore dichiara ingresso 2–16 V, almeno 3 V per l'avvio, consumo a vuoto inferiore a 0,2 mA su gran parte del campo e un limite di circa 700 mA durante l'avviamento. Quest'ultimo valore richiede attenzione ai carichi accesi al boot.[^19]

La riaccensione richiede circa il 114% della tensione di stacco: impostando, per esempio, 5,7 V, il nodo d'ingresso deve risalire a circa 6,5 V. **Questo è un esempio funzionale, non la soglia prescritta per la batteria installata.** Una linea che torna soltanto a 6,0 V non garantisce la riaccensione dopo lo stacco. La soglia deve includere le cadute di tensione e deve essere compatibile con la tensione disponibile al ritorno dell'alimentazione.[^19]

Il regolatore non elimina l'assorbimento del caricatore che resta collegato alla batteria. Un consumo residuo ipotetico di 10 mA corrisponde a 0,24 Ah al giorno, anche se l'ESP32 è spento. Per risolvere il problema del consumo a riposo bisogna misurare l'intero percorso, non soltanto il nuovo convertitore.

## Alimentazione e ricarica con percorsi separati

Una configurazione con alimentatore a 12 V, caricatore dedicato e selezione automatica della sorgente può rendere l'avvio indipendente dalla tensione della batteria. Il seguente è uno **schema funzionale da completare**, non una distinta pronta o uno schema morsetto per morsetto:

```text
Alimentatore 12 V ──┬────────────────── A ──┐
                   │                      ├─ selezione ── convertitore 5 V ── bilancia
                   │                 B ───┘               con cutoff
                   │                 │
                   └─ caricatore ── nodo BAT
                                     │
Batteria interna (+) ── fusibile ─────┘
```

Il fusibile è vicino alla batteria, prima della derivazione comune a ricarica e scarica. I ritorni negativi non sono rappresentati in questo schema funzionale: il loro collegamento dipende dal caricatore scelto e dalla posizione dello shunt di misura.

La sorgente A alimenta direttamente la conversione a 5 V quando è presente. La sorgente B subentra quando si scollega l'alimentatore. La batteria riceve corrente attraverso il proprio caricatore, mentre l'elettronica non deve attendere che la sua tensione recuperi.

Per la selezione esiste **Pololu 5398**, una coppia di diodi ideali 4–60 V su una scheda di circa 19 × 20 mm. Non carica la batteria e non integra lo stacco per scarica profonda. L'offerta UE verificata presso Kamami riportava un'attesa di quattro settimane: non è coerente con una consegna rapida.[^20][^21]

I due 1N5822 già disponibili possono svolgere la selezione con una maggiore caduta di tensione. Prima di tradurre questo principio in cablaggio devono essere verificati il caricatore esatto, le masse, le correnti inverse e la soglia del cutoff. In particolare, alcune schede CC/CV economiche misurano la corrente sul negativo: collegamenti di massa esterni possono bypassare quella misura. Il fatto che un modulo abbia due trimmer non lo rende automaticamente adatto a questa architettura.

La famiglia **LTC4162-S** dimostra che ricarica del piombo, priorità al carico e avvio con batteria scarica sono disponibili anche in un unico controllore. La relativa demo assemblata resta costosa e richiede configurazione. Il prezzo del solo integrato non è il costo di un modulo pronto da installare.[^11][^12]

## Alternativa con caricatore esterno a 6 V

Un caricatore espressamente previsto per uso tampone può rimanere collegato a batteria e carico, purché la corrente disponibile copra entrambi. Un esempio italiano è **Antei & Paolucci AL-BA80622**, dichiarato per uso tampone, con uscita 6,9 V / 0,8 A e batterie da 6–10 Ah. La pagina espone 33,73 € IVA esclusa, cioè circa 41,15 € IVA inclusa.[^22]

È una possibilità concreta per spostare la ricarica fuori dalla bilancia, ma non soddisfa automaticamente tutti i requisiti. La capacità dichiarata non coincide necessariamente con quella della batteria attuale. Inoltre, con batteria e carico sullo stesso nodo, il caricatore può trovarsi in limitazione di corrente e la tensione può restare sotto quella richiesta per riattivare il cutoff.

La semplice possibilità di «caricare mentre alimenta» non dimostra quindi il **riavvio immediato dopo lo stacco**. Aggiungere un diodo a uno smart charger può alterare la tensione che esso misura o impedirgli di rilevare la batteria. Occorre un caricatore e un circuito specificamente compatibili; un mantenitore automobilistico generico non chiude da solo il progetto.

Un'altra alternativa è **ANSMANN ALCS 2-24A, codice 9164016**, con uso parallelo dichiarato e prezzo diretto di **41,99 €**. Il manuale attuale indica 6,9 V / 800 mA, mentre la pagina prodotto riporta 700 mA: occorre distinguere le revisioni. Sono limiti rilevanti la temperatura ambiente massima di +25 °C e l'indicazione di non lasciare la carica incustodita. Per un locale di lavoro caldo e ricariche notturne non è quindi una scelta automaticamente adatta; il consumo inverso resta non quantificato.[^25]

Sono stati considerati anche Alpha Elettronica KCP0600/U1 e ANSMANN BC. Il primo documenta la carica per piombo, ma questo non basta a dimostrare la gestione del carico continuo. La famiglia BC dichiara il funzionamento parallelo; il manuale BC1A, tuttavia, esclude l'uso commerciale/artigianale/industriale previsto per questa applicazione. Questa esclusione del produttore non va confusa con una valutazione normativa dell'intera bilancia.[^23][^24][^26]

## Misure che decidono l'acquisto e integrazione

La misura decisiva per il caricatore è l'energia consumata durante un intervallo rappresentativo di lavoro. Se l'INA219 esistente misura effettivamente il ramo batteria, corrente e tempo permettono di stimare gli Ah prelevati. Se misura soltanto il ramo del carico, il calcolo va adattato alla posizione del sensore. La sola tensione non fornisce una percentuale precisa mentre si carica o si lavora.

Per il problema del controller attuale, serve anche il consumo dalla batteria con carico scollegato o realmente spento. Questo dato distingue il consumo del gestore di alimentazione da quello del buck e dell'ESP32 in standby. I due problemi possono sommarsi.

La protezione dalla scarica va verificata insieme al recupero: batteria al limite, collegamento dell'alimentatore, scollegamento e successivo avvio devono produrre il comportamento previsto. Va controllata la linea 5 V durante i picchi di Wi-Fi, display e audio, oltre alla temperatura dei moduli nel contenitore chiuso.

Il fusibile deve proteggere il cablaggio alimentato dalla batteria e stare vicino al suo positivo, prima dei tratti che devono essere protetti. Il suo valore dipende da sezione dei cavi, corrente massima, spunto e caratteristiche del fusibile; non si ricava dai soli Ah della batteria. Un fusibile presente sulla scheda non protegge necessariamente il cavo fra batteria e scheda.

Per il montaggio interno servono fissaggio stabile, isolamento dei contatti e protezione da condensa o liquidi. La scelta di un marchio documentato riduce l'incertezza sui componenti, ma non qualifica automaticamente il cablaggio e l'apparecchio finito.

**Decisione operativa:** mantenere il SECO-LARM come candidato economico per uso tampone, senza acquistare una catena di altri moduli sulla sola base dei valori nominali. Se il consumo tra ricariche richiede più di quanto i 100 mA possano reintegrare nel tempo disponibile, occorre un caricatore più potente e una gestione del carico separata, oppure rivedere batteria e alimentatore. Non è motivato pagare oltre cento euro per una scheda di valutazione per risolvere questa bilancia.

## Fonti

Fonti tecniche primarie, salvo pagine dei rivenditori utilizzate per condizioni di acquisto o per ospitare i documenti originali. Accesso il 10 settembre 2026; date di revisione riportate dove identificabili.

[^1]: IoTiVedo, [SECO-LARM ST-2406-2AQ](https://www.iotivedo.it/seco-larm-st-2406-2aq-alimentatore-caricabatterie-uscite-6-12-24-vac-input-6-12-24-vdc.html). Prezzo e stock verificati sulla pagina del negozio.
[^2]: IoTiVedo, [Condizioni di vendita](https://www.iotivedo.it/condizioni-di-vendita). IVA inclusa, spedizioni dall'Italia e costi minimi.
[^3]: SECO-LARM, [ST-2406-2AQ — Power Supply / Charger](https://www.seco-larm.com/product/st-2406-2aq/). Funzioni di alimentazione, ricarica e commutazione.
[^4]: SECO-LARM, [ST-Series Power Supplies/Chargers, manuale](https://www.seco-larm.com/wp-content/uploads/2020/12/MI_ST-2406-xxQ_231002_ML.pdf), revisione 2 ottobre 2023, pp. 2–3 e versione spagnola. Ingresso, correnti, cutoff, dimensioni, selettori e trimmer.
[^5]: GS Yuasa, [NP4.5-6](https://www.yuasa.com/it/np4-5-6). Capacità e ingombri del modello di riferimento.
[^6]: Altronix, [AL624](https://altronix.com/library/pdf/installation_instructions/AL624.pdf) e [SMP3 datasheet](https://www.altronix.com/library/pdf/data_sheets/DSSMP3.pdf). Ingresso AC e caratteristiche del tampone.
[^7]: H-Tronic, [Manuale 1191418](https://www.h-tronic.com/Presse/download/anleitung1191418.pdf). Modulo regolabile per accumulatore al piombo; disponibilità commerciale non verificata.
[^8]: Velleman, [K8012](https://www.velleman.eu/products/view/lead-acid-battery-charger-conditioner-k8012/?id=339194&lang=en) e [manuale di montaggio](https://cdn.velleman.eu/downloads/0/illustrated/illustrated_assembly_manual_k8012.pdf). Kit dismesso e alimentazione richiesta.
[^9]: Mascot, [2544 LA](https://www.mascot.no/products/battery-chargers/2544-la) e [specifiche tecniche](https://www.mascot.no/download/85942bebef852c1910ea5afe6e7d476a04c667d9/tech-spec-2544-la.pdf). Variante DC/DC per piombo 6 V.
[^10]: PowerStream, [DC input 6 volt battery charger PST-BC1206-15](https://www.powerstream.com/DCC-6.htm). Specifiche, ingombro e prezzo.
[^11]: Analog Devices, [LTC4162-S](https://www.analog.com/en/products/ltc4162-s.html). Compatibilità piombo, gestione del carico e avvio con batteria scarica/assente.
[^12]: Mouser Italia, [DC2038A-H](https://www.mouser.it/ProductDetail/Analog-Devices/DC2038A-H?qs=0lSvoLzn4L%252Be8xezkuzp7g%3D%3D). Offerta della scheda di valutazione per piombo.
[^13]: DFRobot, [DFR0580 Solar Power Manager for 12V Lead-Acid Battery](https://www.dfrobot.com/product-1795.html). Batteria prevista, ingresso e protezioni.
[^14]: LaskaKit, [SD30CRMA CN3795](https://www.laskakit.cz/solarni-nabijecka-lithiovych-baterii-sd30crma-cn3795/) e Consonance, [datasheet CN3795](https://www.laskakit.cz/user/related_files/cn3795-consonance.pdf). Chimiche documentate e configurazione.
[^15]: WinSystems, [PCM-PS394-500](https://winsystems.com/product/pcm-ps394-500/). Stato fuori produzione.
[^16]: Opencircuit, [Pololu 2870](https://opencircuit.shop/product/5v-step-up-step-down-voltage-regulator-w). Prezzo con IVA NL, magazzino esterno e tempi indicati.
[^17]: Raptor Supplies UE, [Altronix SMP3](https://eu.raptorsupplies.com/pd/altronix/smp3). Prezzo esposto senza IVA e stock.
[^18]: Texas Instruments, [LM2596 datasheet](https://www.ti.com/lit/ds/symlink/lm2596.pdf), revisione G, marzo 2023. Topologia e limiti della regolazione.
[^19]: Pololu, [2870 — S9V11F5S6CMA](https://www.pololu.com/product/2870). Specifiche, EN, isteresi e corrente di avviamento. La pagina riporta valori tipici di sleep non perfettamente uniformi; qui non viene assunto un unico valore garantito.
[^20]: Pololu, [5398 — Power ORing ideal diode pair](https://www.pololu.com/product/5398). Funzione e intervallo di tensione.
[^21]: Kamami, [Pololu 5398](https://kamami.pl/en/protection-modules/1197880-pololu-power-oring-ideal-diode-pair-4-60v-6a-5902186317079.html). Attesa indicata sulla pagina del prodotto.
[^22]: Antei & Paolucci, [AL-BA80622, caricabatterie 6 V / 800 mA](https://www.anteipaolucci.it/caricabatterie-per-batterie-al-piombo-6v/caricabatterie-per-batterie-al-piombo-da_6v-corrente-di-ricarica_800ma-1408.html). Uso tampone, tensione, capacità previste e prezzo.
[^23]: Alpha Elettronica, [KCP0600/U1](https://www.alphaelettronica.com/kcp0600-u1.html). Caricatore per piombo 2/6/12 V.
[^24]: ANSMANN, [BC 6-12 V / 2 A](https://shop.ansmann.de/en/bc-6-12v-2-a). Funzionamento in parallelo dichiarato dal produttore; non costituisce verifica dell'integrazione nella bilancia.
[^25]: ANSMANN, [ALCS 2-24A](https://shop.ansmann.de/en/alcs-2-24-a) e [manuale 9164016](https://shop.ansmann.de/media/manuals/9164016_ALCS-2-24A_Manual.pdf). Funzionamento parallelo, condizioni ambientali e discrepanza tra revisioni.
[^26]: ANSMANN, [manuale BC 6-12 V / 1 A, 1001-0142](https://shop.ansmann.de/media/manuals/1001-0142_Bleiladegeraet-1A_Manual.pdf), p. 13 inglese. Limiti d'impiego dichiarati.
[^27]: Consonance, [CN3791 datasheet, revisione 1.0](https://w2.electrodragon.com/Chip-cn-dat/CONSONANCE-dat/CN3791-dat/dse-cn3791.pdf), copia del documento originale. Carica per una cella Li-ion, tensione tipica 4,2 V.
