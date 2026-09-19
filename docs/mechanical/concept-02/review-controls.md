# Concept 02 - verifica indipendente dei comandi e degli innesti

Oggetto: lettura del generatore `generate_assembly.py` e confronto con le fonti dei componenti; realizzazione dell'helper separato `controls_cable_details.py` integrato nel generatore. Il revisore non modifica il generatore principale. Non e una prova di stampa, montaggio fisico, resistenza, tenuta o compatibilita elettrica.

## 1. Geometrie controllate

- Due tastierini BerryBase B-SM4T da 70 x 20 x 1 mm, su due sedi 71 x 21: le superfici sono piane e complanari alla fascia inclinata. Le due membrane hanno 11 mm di separazione nominale. Il piatto inizia a Y=-50, mentre il bordo posteriore della membrana superiore rimane circa a Y=-88: circa 38 mm liberi davanti al piatto. Tra bordo destro della cornice OLED e bordo sinistro delle membrane restano 38 mm. Non emerge un'interferenza geometrica del piatto sopra i comandi.
- La cornice OLED e separata e rimovibile. Sono modellate quattro viti M3 svasate e quattro dadi per cornice/scocca, due traversini posteriori con quattro viti M3 e relativi dadi. Le teste posteriori sono raggiungibili dopo l'apertura del fondo. Il fissaggio non inventa interassi del PCB posseduto.
- Il connettore USB-C ha flangia verso l'interno e dado esterno. Il foro nominale di progetto e 12,4 mm nella parete da 3 mm. Il corpo commerciale conserva il filetto M12 x 1 e lo sbalzo verso l'esterno. Dado e tappo sono esplicitamente inviluppi provvisori.
- Le porte della scheda UPS sono state ricavate dal STEP ufficiale, usando le facce anteriori degli involucri e non il centro dell'intero componente. Le coordinate sono in `ups-port-reference.json`: con la trasformazione dell'assieme, entrambe le porte si aprono verso -X. USB-C ingresso ha centro circa (56,588;47,940;50,771); USB-A uscita circa (57,216;64,727;52,752). I cavi devono raggiungerle dal lato sinistro dell'UPS.

## 2. Innesti rappresentati nel CAD corrente

1. **Passacavo tastiera coperto.** Due ponti 24 x 11 x 2,4 mm coprono il tratto dello slot che esce dall'impronta della membrana e proteggono la curva della coda. I sei ritagli adesivi da 0,13 mm stanno all'esterno del rettangolo 70 x 20, senza appoggiare sui tasti. La posizione di uscita e lo spessore esterno del connettore BerryBase restano da campione; le misure Adafruit non provano intercambiabilita.
2. **Code, spine e adattatore tastiera presenti.** Ogni coda ha centrolinea curva di 76 mm e connettore femmina lungo nominalmente 12 mm, totale 88 mm. Le pieghe avvengono solo nel piano YZ; la prima coda scende 5 mm piu della seconda, evitando sovrapposizioni senza forzare curve laterali della membrana. Due pettini maschio 1 x 5, i dieci contatti inclusi, due fasci diametro 4,2 mm e una terminazione 1 x 6 con sei contatti descrivono l'ingombro dell'adattatore su misura. La terminazione e centrata a (-40,5;-48;25), sul vassoio elettronica; rami nominali 152,339 e 139,017 mm, taglio 200 mm. La topologia elettrica e nelle fonti controlli; le giunzioni interne dei singoli fili e la crimpatura non sono modellate a livello di fabbricazione.
3. **Cavi USB e fermacavi presenti.** I due percorsi rappresentano prolunga USB-C e uscita USB-A/5 V. Le spine arrivano alle porte da -X come richiesto dalle facce del STEP ufficiale. Due fermacavi apribili con quattro viti M3 fissano i cavi al fondo; la forma stampata del canale segue il percorso nominale.
4. **Sede O-ring USB semplificata.** Il corpo contiene una gola anulare per evitare l'interpenetrazione dell'anello con la flangia. La gola non e una quota costruttiva pubblicata del componente: rimane provvisoria fino al campione. Il CAD del passapannello rappresenta innesti e ingombri, senza pretendere una replica completa del componente acquistato.
5. **Ritegno UPS sulla lastra corretta.** Il STEP ufficiale colloca la lastra acrilica sotto le batterie: Z globale 21..22,5, con PCB a 47,5..49,1. Z massimo 59,6 corrisponde a un componente superiore e non e una superficie di serraggio. Il generatore corrente trattiene il bordo della lastra inferiore con due staffe rimovibili; la verifica dei contatti contro i singoli solidi ufficiali appartiene all'audit dell'assieme.

## 3. Limiti che richiedono il componente reale

- Il display posseduto non e identificato oltre a controller, risoluzione e diagonale. I traversini si appoggiano all'inviluppo posteriore: bisogna verificare dove il PCB reale e libero da componenti e connettori. Il carrier e una predisposizione sostituibile, non un fissaggio validato per il display di Andrea.
- Il connettore BerryBase a cinque pin non ha inviluppo esterno o pinout pubblicati. Lo slot 16 x 6 e un margine di progetto, non una prova del suo passaggio. Verificare anche la continuita per costruire la matrice 4 x 2.
- Ingresso previsto 5 V / 3 A: la portata continua della prolunga USB-C PCM-0726 resta non pubblicata nelle fonti consultate. Va confermata per l'assieme del cavo e dei contatti; 24 AWG da solo non la dimostra.
- I raggi dei cavi, le impronte dei fermacavi e lo spazio della spina durante inserimento ed estrazione richiedono la forma del cavo acquistato. I centri delle porte nel STEP sono un riferimento meccanico, non una prova di accoppiamento elettrico.

## 4. Esito

La disposizione dei comandi e l'orientamento del passapannello sono coerenti con un assemblaggio accessibile. Coperture, code, connettori, cablaggi e fermacavi sono presenti. `check_keypad_self_fit.py` ha controllato le 33 parti dell'helper definitivo: non risultano sovrapposizioni tra le due code, connettori distinti o fasci. Quattro intersezioni residue rappresentano due ingressi nell'housing e due contatti nel proprio fascio; sono accoppiamenti nominali espliciti, non collisioni fra componenti indipendenti. Il risultato e in `keypad-self-fit-check.json`.

L'audit definitivo `interference-audit.json` registra **0 interferenze inattese su 1004 coppie candidate**, includendo i componenti ufficiali UPS contro le parti di progetto ed escludendo soltanto i contatti interni fra componenti dello stesso modello commerciale. Gli innesti intenzionali dei connettori sono elencati come eccezioni motivate. La verifica di esportazione registra **444 solidi STEP** e **15 STL validi**.

`check_controls_fit.py` resta disponibile per diagnosi mirate; la consegna conserva gli esiti riferiti alla geometria definitiva. Questi risultati verificano il CAD nominale. Restano da provare campioni dei connettori, raggi delle code, adesione, montaggio fisico e compatibilita elettrica.
