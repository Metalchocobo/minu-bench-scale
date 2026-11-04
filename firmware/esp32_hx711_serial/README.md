# esp32_hx711_serial — Firmware HX711 (ESP32)

Le fonti hardware correnti sono la documentazione HX711 confermata da Andrea e i due ritagli di schema forniti: **OLED VCC e INA219 VCC → ESP32 3V3; HX711 VCC e IRF9540 S → linea Mini360 VIN/5V**. La resistenza LED su GPIO15 è **1 kΩ** nello schema. La **100 kΩ pull-down SCK–GND è da prevedere nella replica**, dopo la 220 Ω sul lato HX711 e vicino al modulo: mantiene SCK LOW quando GPIO16 non è pilotato durante avvio/reset. È una scelta di robustezza, non un obbligo del datasheet AVIA né un level shifter; soltanto la presenza sulla bilancia attuale resta probabile, da confermare. I dettagli fisici incerti sono distinti dai collegamenti in [WIRING](../../docs/WIRING.md).

Le fonti canoniche sono mantenute nella root del repository:

- `../../README.md` — comportamento firmware, comandi, MQTT, build e diagnostica.
- `../../docs/WIRING.md` — pinout e cablaggio fisico.
- [Guida illustrata del cablaggio corrente](../../artifacts/wiring-current/README.md) — panoramica, tavole dei collegamenti e componenti identificati.

La tastiera usa le coppie GPIO rilevate sul montaggio e le funzioni corrispondenti all'orientamento del pannello: righe scansione **13 = TARE/DONE, 14 = MODE/UNDO, 19 = TOTAL/WIFI, 21 = ENTER/SKIP**; colonne **17 (TX2) = TARE/MODE/TOTAL/ENTER** e **5 = DONE/UNDO/WIFI/SKIP**. Il frontale funzionale, dall'alto, è **SKIP / OK**, **WIFI / UNDO**, **DONE / TOTAL**, **MODE / TARE**, con **OK = ENTER**. DONE sostituisce CLEAR sul contatto GPIO13/GPIO5; i sei fili non cambiano. Pin e mappa sono definiti rispettivamente in `config/config_pins.h` e `keypad.cpp`; la combinazione WIFI + TOTAL è ricavata dalla mappa. Il wake usa le stesse righe LOW e le colonne 17/5 con pull-up, senza eseguire l'azione del tasto finché non viene rilasciato.

**UNDO breve**, al rilascio, annulla l'ultima pesata nello stack e ripristina la tara precedente: subito per una voce locale, dopo l'ACK di persistenza per una voce registrata nel Manager. Il Manager rimuove il peso registrato e riapre l'ingrediente. **`SD:/MP3/0021.mp3`** viene richiamato soltanto a esito positivo: subito dopo il ripristino locale, oppure all'ACK Manager riuscito anche se il fail-safe ha già ripristinato la tara durante lo sgancio. AUDIO ON/OFF resta valido e 0021 è non interrompibile: si accoda alle altre voci non interrompibili. Se l'annullamento non è possibile, suona soltanto il buzzer d'errore. Gli annullamenti consecutivi procedono una voce per volta, dopo il completamento del precedente; la cronologia è in RAM, viene azzerata da TARE manuale riuscita, `stack clear` o reboot. **UNDO tenuto 2 s** commuta AUDIO ON/OFF senza annullare. Nel wizard UNDO annulla la calibrazione, senza commutare l'audio. Lo standby manuale non è disponibile: restano inattività di 5 minuti e protezione batteria. Dettagli nel [workflow stack](../../README.md#13-weigh-stack--stack-pesate-locale).

**DONE** agisce solo nella pesatura collegata al Manager, anche nel riepilogo senza ingrediente attivo: al rilascio breve imposta **Pesato** e apre il riepilogo completo; tenuto **2 s** imposta **Fatto** e torna al piano di lavoro. I file **`SD:/MP3/0022.mp3`** e **`0023.mp3`** vengono richiamati rispettivamente dopo l'ACK di salvataggio riuscito, una volta; entrambi sono non interrompibili e rispettano AUDIO ON/OFF. DONE non modifica tara o stack e non ha funzioni locali/CLEAR. Durante TARE, acquisizione ENTER, calibrazione o una response pending resta inattivo; un cambio contesto durante la pressione annulla il gesto. Owner `production_id`, response `done` e receipt dello stato sono descritti nel [protocollo](../../README.md#12-mqtt--integrazione-gestionale).

Per identificare i contatti di una replica senza conoscere l'ordine del connettore, usare lo [sketch separato `keypad_wiring_test`](../keypad_wiring_test/README.md): stampa le coppie GPIO sulla seriale a 115200 baud e non scrive in NVS. Le funzioni della bilancia sono sospese durante il test; terminata la rilevazione, si ricarica questo firmware con la mappa verificata. Le sigle R1–R6 del ritaglio non identificano l'ordine fisico dei pin.

Il **buzzer passivo** usa **GPIO22 → 220 Ω / ¼ W → base BC337-40 NPN**, con emettitore a **GND comune**, collettore al **− del buzzer** e **+ del buzzer a ESP32 3V3**. Il **1N5819** è in parallelo ai terminali del buzzer, con **catodo/fascetta al + / 3V3** e **anodo al − / collettore**. Il firmware resta invariato: PWM LEDC attivo HIGH, silenzio LOW. Cablaggio completo in [WIRING, buzzer](../../docs/WIRING.md#6-buzzer--esp32).

## Comandi e log seriali

A **115200 baud**, `help` ristampa l'elenco completo dei comandi disponibili, mostrato anche al boot. Il log periodico batteria è spento a ogni avvio: `batt status` stampa una sola analisi, `battlog on` la stampa subito e poi ogni 3 secondi, `battlog off` la interrompe; la scelta resta solo in RAM. I messaggi di inizializzazione e di errore reale restano visibili.

Ogni ingresso **WORK UNSTABLE → STABLE** stampa sempre `[STABLE] <grammi> g`, con due decimali del latch e indipendentemente da `hxlog`. In sovraccarico aggiunge `[SOVRACCARICO]` e mantiene il valore reale. Questo log non modifica stack o MQTT. `hxlog on/off` abilita il log continuo HX711, spento di default. Dettagli nel [README principale](../../README.md#comandi-e-log-seriali).

## Portata e calibrazione

Cella montata **20 kg nominali**, come riferito da Andrea. In `config/config_scale.h`, `MAX_TOTAL_LOAD_G = 15000` limita il carico complessivo e `CAPACITY_DEAD_LOAD_G = 500` riserva una quota prudenziale per piatto/supporti: questi 500 g sono una stima indicata da Andrea, non una massa misurata. Il totale è `(raw - zero_calibrazione) / CPG + 500 g`, perciò SOVRACCARICO entra a **14,5 kg di contenitore + contenuto**. Il rientro richiede totale sotto **14.950 g** e valore assoluto del netto sotto **15.950 g**; la guardia indipendente sul netto entra oltre **±16 kg**. TARE/ENTER restano bloccati durante SOVRACCARICO.

Il display mantiene **SOVRACCARICO / Rimuovi peso dal piatto / ENTER e TARA bloccati** finché lo stato resta attivo. All'ingresso il buzzer emette automaticamente un **doppio beep una volta per evento**, con riarmo dopo il rientro: 1.500 Hz, 120 ms per tono e 80 ms di pausa, configurabili in `config/config_audio.h`. Lo scheduler è non bloccante e indipendente da AUDIO ON/OFF e DFPlayer; non occorre aggiungere MP3.

Lo zero di calibrazione in **NVS `hxscale/offset_counts`** è separato dalla tara operativa; TARE, ENTER, undo, zero-tracking, Auto-TARE al boot, sleep e recupero WDT non lo modificano. `cal zero` acquisisce zero calibrazione e operativo con **piatto/supporti montati, senza contenitore né ingredienti**; `cal ref <g>` calcola il CPG da quel riferimento e `cal save` salva la coppia di calibrazione anche dopo una tara operativa. `cal status` mostra entrambi gli zero e il totale stimato.

Dopo il cambio cella rifare la calibrazione, preferibilmente con un riferimento noto da **10 kg**. Se la vecchia calibrazione attiva SOVRACCARICO e blocca ENTER/TARE nel wizard, calibrare prima con **2 kg**, poi ripetere con 10 kg. I comandi seriali `cal` restano disponibili per la ricalibrazione deliberata: seguire la procedura con zero a piatto vuoto. Il wizard si apre con **SKIP tenuto 5 s**; procedura completa e controlli nel [README principale](../../README.md#calibrazione).

## Codice bilancia e LED voce

Da seriale a **115200 baud**: `scale code 00-S` salva il codice in **NVS `minu_identity`, chiave `code`**; **al prossimo riavvio** il nome diventa **Ronin 00-S**, visibile dalla prima schermata OLED e nel campo `name` degli status/LWT MQTT. `scale status` mostra nome attivo e codice salvato; `scale reset` salva il default **00**. Sono ammessi **1–12 caratteri ASCII A-Z, a-z, 0-9, `-`, `_`**, con zeri iniziali mantenuti. Il nome attivo resta immutabile fino al riavvio, anche per il worker MQTT. Il `scale_id` tecnico resta il MAC; i record già registrati nel Manager non vengono rinominati automaticamente dalla discovery. Implementazione: `scale_identity.h/.cpp`; flusso completo nel [README principale](../../README.md#codice-e-nome-della-bilancia).

Il LED esistente su **GPIO15 + 1 kΩ** pulsa con sfumature software mentre il DFPlayer conferma la riproduzione via **BUSY GPIO39**. L'effetto simula la voce senza seguirne l'ampiezza o le sillabe; player spento, AUDIO OFF, idle e sola coda non lo attivano. In standby per inattività il PWM viene sganciato e il LED resta fisso; al wake viene ripristinato. Lo sleep per batteria scarica lo lascia spento. `status_led.h/.cpp` gestisce l'effetto non bloccante, con parametri in `config/config_led.h`: **5 kHz, 8 bit, canale LEDC 8** separato dal gruppo del buzzer. Non occorrono modifiche al cablaggio; la resa visiva va verificata sulla bilancia. Riferimento in [WIRING](../../docs/WIRING.md#8-led-audio-e-standby--esp32).

## Ingresso presenza USB: PC817

Il rilevatore USB usa **GPIO36 / VP**, input-only senza pull interni, con **10 kΩ esterna fra GPIO36 e ESP32 3V3**. Il **PC817 DIP-4** si collega così: **USB +5 V → 560 Ω / ¼ W → pin 1 anodo**; **pin 2 catodo → USB−**; **pin 3 emettitore → GND ESP32**; **pin 4 collettore → GPIO36 / VP**. Il lato LED è un ramo **in parallelo all'ingresso USB del caricatore della batteria**; non usare Mini360 OUT+ e non aggiungere un ponte fra USB− e GND della bilancia.

Con ESP32 alimentato, l'ingresso è **LOW con USB presente, HIGH con USB assente**, con **250 ms di livello stabile su entrambi i fronti**. Rileva presenza di tensione USB, non corrente di ricarica o fine carica. Il PC817 fa parte del cablaggio per la replica; la sua installazione fisica sull'esemplare attuale non è confermata. Pinout e connessioni in [WIRING](../../docs/WIRING.md#rilevatore-presenza-usb-pc817-dip-4) e nella [tavola 07](../../artifacts/wiring-current/07-usb-pc817.svg); non sono documentate prove hardware del rilevatore.

## Alimentazione montata e configurazione firmware

La bilancia usa la batteria USB-C NASTIMA LiFePO4 e il Mini360 MP1482DS, con uscita regolata a **5,11 V** e primo funzionamento positivo riferito da Andrea. Le tacche usano fasce indicative LiFePO4 definite in `config/config_battery.h`. L'icona animata e `BatteryStatus.charging` indicano **presenza USB dal PC817**, confermata dopo **250 ms di livello stabile** sia al collegamento sia allo scollegamento (`USB_PRESENCE_DEBOUNCE_MS`); restano attivi anche a batteria piena. L'indicatore parte spento all'avvio finché il primo livello è confermato; una pausa di campionamento superiore a 250 ms richiede una nuova conferma stabile. La lettura USB è indipendente da presenza e validità dell'INA219: errori I2C e campioni invalidi/scaduti non azzerano l'indicatore. La corrente INA219 è quella del carico, perché la ricarica interna USB non attraversa lo shunt esterno: `charging` non misura corrente, carica residua o fine carica. Nel debug seriale `charging=YES/NO` indica presenza USB, mentre `valid=YES/NO` riguarda la lettura INA219. Avvisi sonori, countdown e protezioni conservano soglie e tempi, richiedono campioni INA219 validi e freschi e restano attivi anche con USB presente. Soglie effettive e limiti sono nel [README principale](../../README.md#8-monitoraggio-batteria-ina219); percorso elettrico in [WIRING](../../docs/WIRING.md#9-alimentazione-e-masse).

**Lettura batteria:** tenere premuti **TOTAL + WIFI** per visualizzare tensione e corrente INA aggiornate; rilasciare per chiudere. I tasti singoli WIFI/TOTAL agiscono al rilascio e vengono consumati dalla combinazione. Campioni invalidi/scaduti non vengono mostrati; safety, TARE, ENTER e calibrazione hanno priorità. La schermata è diagnostica e non modifica soglie o rilevamento ricarica. Dettagli nella [sezione INA219](../../README.md#81-collegamenti-ina219).

Il power-gate audio montato usa **IRF9540 + NPN 2N3904**, con marcatura **H331** riferita da Andrea e comando **GPIO2 HIGH = ON**. Il ritaglio audio indica **IRF9540 S → VIN/5V**, **D → VCC DFPlayer**, **100 kΩ G–S**, **GPIO2 → 10 kΩ → base NPN**, **100 kΩ base–GND** ed **emettitore → GND**. H331 non identifica qui produttore o data; il pinout fisico dell'esemplare resta da verificare. Presso il DFPlayer sono confermati un **elettrolitico da 470 µF / 16 V** e un **ceramico 104 = 100 nF**; quantità, collegamenti, polarità e posizione restano da rilevare.

I condensatori restano in gruppi distinti: ingresso Mini360 **220 µF / 16 V + 104**; uscita Mini360 **470 µF / 10 V + 104**; coppia **aggiuntiva sulla PCB custom fra Mini360 ed ESP32, 47 µF / 16 V + 104**, fra 5 V e GND; coppia **presso ESP32**, presente fra VIN/5V e GND con valori fisici ancora ignoti; audio **470 µF / 16 V + 104**. Per la sola coppia presso ESP32 sono consigliati per la replica **47 µF / 16 V + 100 nF (104)**, in parallelo vicino alla scheda, con + dell'elettrolitico a VIN/5V e − a GND. Questi valori consigliati non identificano quelli montati presso ESP32 e non sostituiscono la coppia PCB custom. Pin funzionali, resistenze e riferimenti da confermare sono descritti in [WIRING](../../docs/WIRING.md#7-dfplayer-mini--esp32). L'HX711 resta sulla linea nominale 5 V del Mini360 per il requisito della cella usata. I collegamenti documentati non richiedono una modifica del firmware.

## Console remota MQTT

Il firmware **1.6.0** annuncia `console_version:1` nello status e usa topic dedicati `minu/scale/{id}/console/request` e `/console/response`, non retained. Richiede gateway Manager autorizzato e **ACL broker dedicate prima del rollout**; le credenziali console non vanno al browser. Nessun hardware aggiuntivo. La seriale USB resta necessaria quando rete/firmware non sono operativi.

`probe` rileva la capacità; `open/poll/exec/close` usano boot e sessione esatti, request ID e scadenza futura entro 30 s. Sessione esclusiva con lease di **30 s**, rinnovato soltanto da open della stessa sessione o poll valido. Blocca lo sleep per inattività finché aperta; la protezione batteria conserva priorità. Close/disconnessione/expiry liberano il blocco. Sessioni chiuse mantengono un fence anti-replay bounded di 31 s; una nuova apertura usa un ID nuovo.

I comandi (massimo 127 byte, senza NUL/CR/LF) vengono accodati dal callback ed eseguiti nel loop con `seq` strettamente progressiva; retry esatto non ripete effetti. Diagnostica e configurazione condividono il parser e i guard della USB; `showpass` resta USB-only e le password non entrano nel tee remoto, anche se richieste dalla USB. Mutazioni durante acquisizione/TARE/OTA e cambi rete con receipt pending sono rifiutati busy. Reboot/cambi rete possono perdere l'esito finale.

`remote_console_state.h/.cpp` implementa sessioni, fence e ring RAM 2 KiB; `remote_console.h/.cpp` integra parser JSON, oggetto Print `ConsoleLog` e trasporto. Il ring raccoglie solo con sessione attiva e non pubblica dalle write; poll legge chunk UTF-8 validi con cursori byte, oldest e conteggio perdite. La response è bounded entro il pacchetto runtime da 640 byte. Contratto completo e procedura ACL nel [README principale](../../README.md#console-remota-del-manager).

## Riferimento rapido MQTT firmware 1.6.0

- Un `weigh` può includere `command_id` (8–64 caratteri; lettere ASCII, cifre, `-`, `_`; vuoto solo per legacy). Il firmware lo memorizza con UUID, prodotto, `session_id` e `connection_id`. Se quest'ultima è presente, il comando diventa operativo soltanto con l'owner retained fresco ed esatto e, dopo reconnect, dopo avere processato anche lo snapshot retained `command` corrente.
- Dopo l'attivazione di un comando tokenizzato e solo quando è realmente azionabile pubblica sul topic `response` un `command_ack` transitorio con `command_id`, `session_id`, `connection_id`, UUID, prodotto e `state=active`. Fallback `LOC`, TARE obbligatoria e convergenza owner/command incompleta non producono ACK. Un replay identico dello stesso ID non reinizializza la pesata, ma riaggancia un eventuale fallback locale, aggiorna la connessione corrente e riemette l'ACK. Un payload business conflittuale viene ignorato senza ACK.
- Il Manager può pubblicare sul topic `command`, QoS 1 e non retained, una `confirm_request` fenced da `request_id`, sessione, connessione, comando, UUID e prodotto. Il loop la esegue tramite lo stesso helper e lo stesso lock TARE di ENTER fisico. Lo stesso request ID non ripete l'azione; una cache RAM mantiene gli ultimi due esiti terminali anche se il relativo ACK si perde. `accepted`/`staged` non avanzano la pagina e `failed`/`rejected` lasciano attivo l'ingrediente corrente.
- Un `clear` live cancella un comando moderno solo se coincidono `command_id` e `session_id`: con owner fresco richiede la connessione corrente, mentre senza owner accetta il retained tombstone esatto. Al reconnect il primo retained clear moderno è invece lo snapshot autorevole del broker e pulisce il raw RAM anche se la coppia è già avanzata. La stessa fingerprint comando/connessione chiude inoltre l'attesa owner se il weigh era stato respinto prima di entrare nel raw RAM, senza modificare l'outbox. Il clear speciale `lifecycle=pagehide` sgancia l'owner corrente anche senza comando; cancella il raw solo se coincide anche la fence comando. Un comando v1.2 senza ID richiede la stessa sessione; un comando realmente legacy senza sessione accetta il clear senza ID del Manager nuovo.
- Una response durable `confirm` o `skip` conserva il `command_id` originale. L'`undo` usa `undo_of_response_id` e non inventa un nuovo command ID.
- Se l'owner scade, un comando moderno viene conservato raw in RAM ma reso non operativo e non visibile. Da 1.4.2 la bilancia passa alla modalità locale: TARE e ENTER fisici funzionano senza creare nuove response MQTT; un nuovo `weigh` valido riaggancia il Manager. Da 1.4.4 il loop elabora prima un burst limitato a quattro pacchetti MQTT già disponibili e soltanto dopo valuta la scadenza. Da 1.5.0 la presenza usa 25 secondi dall'arrivo di un owner valido, tollerando un heartbeat da 10 secondi saltato. Solo un timestamp strettamente avanzato sullo stesso lease, con età entro 10 secondi, ottiene fino a 5 secondi di clock/jitter grace; replay uguali e lease scaduti non vengono estesi. Da 1.5.2 il takeover da `LOC` non dipende più dal confronto tra epoch tablet ed ESP32: una nuova connessione/lease diventa un candidato inattivo e soltanto un secondo heartbeat con timestamp strettamente avanzato sulla stessa generazione prova che il tablet è vivo. Gli snapshot retained iniziali e i replay uguali non diventano freschi.
- Il watchdog passivo valida le subscription con i soli owner heartbeat, retained command ed esiti ACK già previsti dal protocollo. Non pubblica probe sui topic business e non richiede nuove ACL. Da 1.5.1 un messaggio `command` ricevuto prova il canale anche quando il suo stato non coincide ancora con `expected_command_id`: la bilancia resta non azionabile fino alla convergenza, ma non risottoscrive né riconnette un socket sano. Un `weigh` moderno valido arrivato prima dell'owner esatto apre una grace owner monotona e deduplicata per comando/connessione; un successivo PUBLISH owner prova il topic anche se il payload è business-divergente. Solo la vera assenza avvia le subscribe bounded ogni 2,5 secondi e al massimo un reconnect per fingerprint; al secondo esaurimento resta `LOC` senza socket churn. L'evidenza owner deve appartenere al trasporto corrente e nuovi ID durante una grace non ne prorogano la scadenza. La grace command parte dal cambio effettivo dell'aspettativa e non viene estesa da heartbeat uguali. Un owner fresco con `expected_command_id=""` pulisce autorevolmente il raw RAM anche senza retained clear, salvo response o `confirm_request` pendenti, e ignora weigh tardivi finché attende clear. Manager legacy senza il campo resta invariato. Per la stessa response l'ACK timeout causa un solo reconnect e resta diagnosticato come `ack_timeout`.
- L'outbox durable resta immutabile e continua il retry ogni secondo fino al `response_ack` esatto o al terminale `response_reject/manager_incident`, ma non può bloccare indefinitamente l'operatore. Il lock fisico dura al massimo 10 secondi, termina prima su `clear` con `lifecycle=pagehide`/perdita owner e viene sganciato immediatamente dalla prima pressione fisica di TARE o ENTER. Un owner/snapshot remoto mai ricevuto usa la stessa grace. DNS, TCP, TLS e MQTT CONNECT vengono eseguiti da un worker FreeRTOS separato: il loop continua sempre a campionare tastiera e bilancia. TARE/ENTER diventano locali; SKIP/UNDO remoto e nuovi outbox restano serializzati. Un `confirm` quarantinato viene rollbackato una sola volta solo con receipt esatta ancora in testa e operatore non sganciato; altrimenti la cronologia viene invalidata, senza fake success, undo o ACK. Un undo rifiutato prima del detach non cambia lo stack; se era già stato applicato localmente, il firmware invalida stack/riferimento, mostra `SYNC ERROR` e attiva lo stesso gate cross-layer. Lo status espone `tare_required=true`, resta `operational_mode=local` e conserva un eventuale comando raw senza renderlo azionabile o ACKarlo. Solo una TARE fisica riuscita rimuove entrambi i gate e consente allo stesso comando valido di riattivarsi con un singolo `command_ack`.
- Lo status retained viene aggiornato ogni 5 secondi con epoch, boot/transport ID, sequenza, modalità, `tare_required`, causa fallback, ultimo errore, mask owner/command/ack e response pending. Il buffer PubSubClient runtime è 640 byte; il Manager limita `user_name` owner a 48 byte UTF-8 e gli ID a 64 caratteri.
- Rollout base 1.5 raccomandato: prima il firmware, poi Manager/backend che pubblicano/interpretano i nuovi campi e possono emettere `response_reject`. Per le patch 1.5.1/1.5.2 su un impianto già 1.5.0, distribuire prima il bundle Manager con recovery owner fenced a doppia prova e poi aggiornare la bilancia. Il firmware precedente tollera la seconda pubblicazione; il firmware 1.5.2 la usa per rimuovere la dipendenza dall'epoch locale durante il takeover da `LOC`. Non serve modificare le ACL MQTT della bilancia.
- Con un owner vivo ENTER richiede un comando attivo. Senza consumer remoto usa il percorso standalone anche se MQTT è ancora connesso; con MQTT disconnesso e un comando remoto non ancora dichiarato stale attende al massimo la stessa finestra di 10 secondi.
- La barra OLED separa il trasporto dalla modalità operativa: le frecce MQTT sono fisse quando il broker è connesso e lampeggiano soltanto durante la disconnessione; il badge compatto `APP` indica un owner Manager fresco, mentre `LOC` indica fallback/standalone. Il badge non lampeggia.
- Se ENTER arriva prima della quiete, il firmware emette un beep basso e mostra **ACQUISIZIONE PESO** per massimo 1,5 s. Registra appena raggiunge STABLE; al timeout accetta il centro RAW robusto solo senza deriva continua e usa lo stesso centro per peso e zero. Movimento persistente o campioni non validi producono doppio beep e un avviso leggibile; successo e fallimento si chiudono automaticamente dopo circa 1,3 s e 2 s.

Verifica locale:

`tests/firmware_remote_console_harness.cpp` compila `remote_console_state.cpp` senza Arduino e verifica lease/rollover, sessione esclusiva, boot/scadenze, replay e conflitti di sequenza, fence delle sessioni chiuse senza eviction, comandi vietati, ring/cursori/perdite e UTF-8 invalido/incompleto. Eseguire anche `firmware_mqtt_protocol_harness.cpp`, che include lo status con capability nel limite pacchetto da 640 byte.

### Verifica contratto clock console Manager/firmware

`tests/firmware_console_clock_bridge.cpp` collega il core C++ reale
`remote_console_state.cpp` a encoder, clock e decoder PHP del repository Manager.
`tests/check_console_clock_contract.php` riproduce il rifiuto con ESP32 avanti
di 12 s, poi verifica scarti ±120 s, snapshot vecchio di 5 s, apertura/poll/comando/
chiusura, replay e rifiuto di retained troppo vecchi. Non usa database, broker o
hardware; non sostituisce una compilazione ESP32 né il parser/trasporto Arduino.

Prerequisiti: dipendenze Composer del Manager installate, PHP, compilatore C++17
e header ArduinoJson 7 della libreria firmware. Su Windows aprire **x64 Native
Tools Command Prompt**, entrare nella root di questo repository ed eseguire ogni
riga separatamente (adattare il percorso ArduinoJson se diverso):

```bat
if not exist "%TEMP%\minu-console-clock" mkdir "%TEMP%\minu-console-clock"
cl /nologo /std:c++17 /EHsc /W4 /WX /I"C:\Users\shado\Documents\Arduino\libraries\ArduinoJson\src" tests\firmware_console_clock_bridge.cpp firmware\esp32_hx711_serial\remote_console_state.cpp /Fe:"%TEMP%\minu-console-clock\console-clock.exe" /Fo:"%TEMP%\minu-console-clock/"
php tests\check_console_clock_contract.php --manager-root="D:\xampp\htdocs\minu\manager" --bridge="%TEMP%\minu-console-clock\console-clock.exe"
```

L'esito atteso è `PASS console clock wire contract`. Per gateway TLS reale,
permessi e sessioni, dalla root Manager eseguire `composer test:domain -- scale`
con `.env.testing` e solo `manager_test`; istruzioni complete in
`docs/agent-knowledge/06c-scale-console.md` del Manager. Queste prove non attestano
deploy Forge o comportamento della bilancia fisica.

### Verifica controlli UNDO

`tests/check_undo_controls.py` compila gli helper di produzione estratti dal file
`.ino` con i moduli reali stack e scala: verifica 21 scenari e 122 assert su
annullamento locale/remoto, ripristino tara, ACK, errori e pressione lunga.
Non usa hardware né database. Dalla root del repository, su Windows:

```powershell
python tests/check_undo_controls.py --vcvars "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
```

Su altre piattaforme omettere `--vcvars`: lo script trova automaticamente `g++`
o `clang++`. Il ripristino della tara mostra subito il peso, senza un overlay
UNDO che lo copra. Il test non sostituisce la compilazione ESP32 o la verifica
della bilancia fisica.

### Verifica controlli DONE

`tests/check_done_controls.py` estrae i veri helper del gesto e del feedback dal `.ino`: verifica breve/lungo, azione singola, annullamento al cambio contesto, blocchi TARE/ENTER/wizard, rollover e audio soltanto all'ACK. `tests/check_done_network.py` verifica staging/outbox e il ramo ACK/reject reali, inclusi riepilogo senza `weigh`, gate owner/prodotto, retry e immutabilità. Usano lo stesso compilatore host della verifica UNDO:

```powershell
python tests/check_done_controls.py --vcvars "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
python tests/check_done_network.py --vcvars "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
```

Le prove host non attestano flash, deploy Manager o comportamento fisico.

### Altri harness firmware

Gli harness `tests/firmware_scale_identity_harness.cpp` e `tests/firmware_status_led_harness.cpp` compilano i moduli reali con stub NVS/PWM: verificano persistenza e applicazione del codice al riavvio, errori di storage, input invalidi, sfumature, stop, standby/wake, rollover e fallback senza PWM. Dalla root, con un compilatore C++17:

```sh
c++ -std=c++17 -Wall -Wextra -Werror -I tests/identity_stubs tests/firmware_scale_identity_harness.cpp firmware/esp32_hx711_serial/scale_identity.cpp -o /tmp/firmware_scale_identity_harness
/tmp/firmware_scale_identity_harness
c++ -std=c++17 -Wall -Wextra -Werror -I tests/status_led_stubs tests/firmware_status_led_harness.cpp firmware/esp32_hx711_serial/status_led.cpp -o /tmp/firmware_status_led_harness
/tmp/firmware_status_led_harness
c++ -std=c++17 -Wall -Wextra -Werror tests/firmware_mqtt_protocol_harness.cpp -o /tmp/firmware_mqtt_protocol_harness
/tmp/firmware_mqtt_protocol_harness
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/esp32_hx711_serial
```

Il test batteria compila il monitor reale con letture INA219 simulate e verifica soglie, tempi, isteresi e interruzioni dei campioni. Dalla root, nel Developer Command Prompt MSVC (output fuori dal repository):

```bat
if not exist "%TEMP%\minu-battery-harness" mkdir "%TEMP%\minu-battery-harness"
cl /nologo /std:c++17 /EHsc /W4 /I tests\battery_stubs tests\firmware_battery_harness.cpp firmware\esp32_hx711_serial\battery_monitor.cpp /Fe:"%TEMP%\minu-battery-harness\firmware_battery_harness.exe" /Fo:"%TEMP%\minu-battery-harness/"
"%TEMP%\minu-battery-harness\firmware_battery_harness.exe"
```
