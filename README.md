# Minù Bench Scale — ESP32 + HX711 + OLED SSD1322 + INA219 (SLA 6 V) · v2026-07-13

Bilancia da banco per uso interno in laboratorio (gelato/pasticceria), pensata per guidare e rendere affidabili le pesate ingredienti (non “pesatura legale”).

Hardware di riferimento:
- **ESP32 DevKit**
- **HX711** (ADC 24 bit) per cella di carico con eccitazione a **5 V**
- **OLED SSD1322** 256×64 (3,12", SPI)
- **Batteria piombo SLA 6 V** con modulo caricatore/protezione **CTK3S**
- **INA219** (CJMCU-219) per tensione/corrente batteria (high-side)
- **Tastierino 4×2** + **buzzer**

---

## 0) Stato e obiettivo

Target:
- Portata: **20 kg**
- Lettura visualizzata: **1 g** nel campo operativo fino a ±16 kg
- Oltre il campo operativo il peso reale non viene trasformato in 16.000 g: compare lo stato bloccante **SOVRACCARICO**, con isteresi di rientro

Funzioni principali (firmware HX711):
- Lettura HX711 con filtri + stati **STABLE / UNSTABLE / LIVE / SOVRACCARICO**
- **Zero-tracking** vicino allo zero + **snap-to-zero** allo scarico
- **TARA** automatica e manuale con centro robusto: tollera il rumore ambientale, distingue la deriva reale e segnala acusticamente gli errori tecnici
- Monitor batteria con **tacche** + stato **charging** stabilizzato; soltanto campioni INA219 validi, plausibili e freschi alimentano filtri e protezioni
- Protezione “batteria scarica” con **avviso + beep + light-sleep**, anti-flapping (debounce 10 s, recovery stabile 30 s) e cooldown avvisi 5 min
- **HX health** runtime: stati **OK / WARN / ERROR / ERROR HARD**, badge **"Errore Cella"** (testo piccolo + triangolino) in alto a destra in WARN, schermata error bloccante (senza mostrare l'ultimo peso), blocco tara/calib, audio 0015.mp3 una sola volta all’ingresso in ERROR

### HX health (runtime, HX711)

Scopo: gestire errori runtime dell’HX711 **senza blocchi in loop** (logica solo a timestamp) e senza impattare feature non correlate.

**Soglie (ms)**
- WARN se nessun campione valido per **≥ 500 ms**
- ERROR se nessun campione valido per **≥ 3.000 ms**
- ERROR HARD se nessun campione valido per **≥ 30.000 ms** oppure **mai** registrato un valore valido
- Ritorno a OK solo dopo campioni validi “OK stabile” per **≥ 800 ms**

**Cosa conta come “campione valido”**
- Valido quando nel loop: `hx711_is_ready()` è true e `hx711_read()` completa.
- Mitigazione “DOUT mascherato” (es. partitore/pull-down): se il raw resta inchiodato (±1 count) per **> 1500 ms**, quei campioni vengono trattati come **non validi** per HX health (non aggiornano `lastRawMs`). Le soglie WARN/ERROR si conteggiano dall’inizio della flatline.
- `hx711_is_ready() == true` e lettura `hx711_read()` completata nel normale loop (niente medie bloccanti).

**Nota importante (cablaggio DOUT su ESP32)**
- Se **DOUT è su GPIO34..39** (es. **GPIO35**), l’ESP32 **non ha pull-up interni**: a modulo scollegato il pin può fluttuare e risultare “ready” a caso.
- In quel caso la rilevazione “modulo non trovato” non è affidabile solo via firmware.
- Una modifica robusta richiede un GPIO libero con pull-up o una rete esterna coerente con i livelli dell'HX711. **GPIO22 non è libero**: nel pinout corrente pilota il buzzer. Qualsiasi cambio di DOUT va quindi coordinato tra cablaggio, `config/config_pins.h` e documentazione.

**Stati e comportamento**

| Stato | Quando | UI | Audio | Azioni |
|---|---|---|---|---|
| OK | campioni regolari | UI normale | nessuno | tara/calib abilitate |
| WARN | assenza campioni ≥ 500 ms | triangolino warning alto a destra | nessuno | tara manuale abilitata; ENTER bloccato finché HX torna OK |
| ERROR | assenza campioni ≥ 3.000 ms | schermata ERROR bloccante, senza mostrare l'ultimo peso | 0015.mp3 **una sola volta** all’ingresso | **tara/calib disabilitate** |
| ERROR HARD | assenza campioni ≥ 30.000 ms o nessun valore valido mai registrato | schermata ERROR bloccante, **non** mostra valore | 0015.mp3 una sola volta all’ingresso | **tara/calib disabilitate** |

---

## 1) Perché HX711 (e perché non NAU7802)

Vincolo emerso sul campo: la cella di carico è stabile/precisa solo se eccitata a **~5 V**.  
Con NAU7802 si è provato a lavorare a 5 V, ma in un setup reale si è verificato **backfeed verso GPIO ESP32** (tensione logica troppo alta) con danni hardware.

Scelta attuale:
- HX711 alimentato a **5 V**
- ESP32 resta a **3,3 V sui GPIO**
- La linea **DOUT va resa 3,3 V safe** (vedi cablaggio)

Firmware disponibili nel repo:
- ✅ `firmware/esp32_hx711_serial/` → corrente
- ⚠️ `archive/esp32_nau7802_serial/` → storico (non consigliato)
- 🧱 eventuali cartelle “legacy” → archivio, non usare per nuove modifiche

---

## 2) Alimentazione (architettura)

Schema logico:
1. Alimentatore → ingresso PV CTK3S  
2. CTK3S BAT → batteria SLA 6 V  
3. CTK3S LOAD → **buck 5 V**  
4. Rail 5 V dal buck alimenta: **ESP32 (VIN/5V)**, **HX711**, **OLED** (se il tuo modulo accetta 5 V)

Motivo: il CTK3S protegge la batteria staccando **LOAD** in undervoltage.

Condensatori consigliati (stabilità rail 5 V):
- Ingresso buck: **220 µF** + **100 nF**
- Uscita buck (rail 5 V): **470 µF** + **100 nF**
- Vicino ESP32 (VIN↔GND): **10–47 µF** + **100 nF**

---

## 3) Pinout (firmware HX711)

| Funzione | Modulo | GPIO ESP32 | Note |
|---|---|---:|---|
| I2C SDA | INA219 | 32 | attenzione a non invertire con SCL |
| I2C SCL | INA219 | 33 | |
| HX711 SCK | HX711 | 16 | spesso serigrafato **RX2** |
| HX711 DOUT | HX711 | 35 | input-only, scelto per evitare strap pin |
| SPI SCK | OLED SSD1322 | 18 | VSPI |
| SPI MOSI | OLED SSD1322 | 23 | |
| SPI CS | OLED SSD1322 | 25 | |
| SPI DC | OLED SSD1322 | 26 | |
| SPI RST | OLED SSD1322 | 27 | |
| Tastiera R1 | Keypad | 17 | |
| Tastiera R2 | Keypad | 5 | |
| Tastiera R3 | Keypad | 13 | |
| Tastiera R4 | Keypad | 14 | |
| Tastiera C1 | Keypad | 19 | |
| Tastiera C2 | Keypad | 21 | |
| Buzzer | buzzer passivo | 22 | LEDC |
| DFPlayer TX | DFPlayer RX | 4 | resistenza 1 kΩ in serie consigliata |
| DFPlayer RX | DFPlayer TX | 34 | opzionale, input-only |
| DFPlayer power-gate | high-side esterno | 2 | HIGH = ON; non alimenta direttamente il modulo |
| DFPlayer BUSY | DFPlayer | 39 | input-only; pull-up esterna consigliata |
| LED standby | LED esterno | 15 | active-high, con resistenza in serie |

---

## 4) Cablaggio HX711 (importante)

### 4.1 Cella di carico → HX711
Collega i 4 fili della cella su **E+, E-, A+, A-** (colori variabili: verifica lo schema della tua cella).

### 4.2 HX711 → ESP32 (logica)

**Alimentazioni**
- HX711 **VCC → 5 V**
- HX711 **GND → GND comune**

**SCK (clock)**
- ESP32 GPIO16 → HX711 SCK **diretto (3,3 V va bene)**  
- Consigli pratici:
  - **220 Ω in serie** sul filo SCK vicino all’ESP32 (anti-ringing/spike)
  - **100 kΩ pulldown** tra SCK e GND vicino al modulo HX711 (tiene SCK LOW durante boot/reset)

> Non usare i classici level shifter bidirezionali “I2C-style” (BSS138) su **SCK**: possono tenere la linea alta e mandare HX711 in **power-down**.

**DOUT (data)**
Se HX711 è a 5 V, **DOUT può andare a 5 V**: va reso “3,3 V safe”.

Soluzione consigliata: **partitore resistivo**
- DOUT (HX711) → **10 kΩ** → GPIO35 (ESP32)
- GPIO35 (ESP32) → **20 kΩ** → GND

(Altri valori equivalenti vanno bene, l’obiettivo è restare ~≤3,3 V sul pin ESP32).

Nota pratica:
- Se colleghi DOUT diretto, prima **misura** con multimetro: se HIGH >3,6 V, è rischioso (nel tempo può rovinare l’ESP32).

---

## 5) HX711: 10 SPS vs 80 SPS

La velocità **non** si imposta da software sull’HX711: dipende dal pin/ponte **RATE** del breakout.

- Tipico default: **10 SPS**
- Per passare a **80 SPS**: modifica la pista/ponticello “RATE” sul retro del modulo (dipende dal modello)

---

## 6) Firmware: comportamento “80 Hz senza rumore a display”

Con HX711 a **80 SPS**:
- il firmware legge tutti i campioni,
- ma **decima/filtra** diversamente per WORK vs LIVE.

Concetto:
- **WORK**: più stabile (decisioni di stato, zero-tracking, ecc.)
- **LIVE**: più reattivo a display, con anti-flicker (e smoothing leggero EMA sul float)

Impostazioni attuali (HX711=80 SPS):
- WORK: decimazione **N=5**
- LIVE: decimazione **N=3** + **EMA veloce** sul valore in grammi prima della quantizzazione (alpha ~0.45)


### Display: anti-flicker
WORK e LIVE usano due visualizzazioni diverse:
1) **LIVE**: il display usa mezzi grammi virtuali (`N.0` / `N.5`) solo sull'OLED. Il valore interno usato da stack/MQTT resta in grammi interi.
2) **LIVE zero clamp**: resta a `0.0` finche' `|g| < 0.50 g`; una volta uscito dallo zero torna a `0.0` solo sotto ~`0.35 g`.
3) **LIVE half window**: su circa **600 ms** analizza media filtrata e candidati interi:
   - frazione bassa / prevalenza bassa (`<= 30%` verso l'intero alto) -> mostra `N.0`
   - zona centrale / oscillazione bilanciata (`31-69%`) -> mostra `N.5`
   - frazione alta / prevalenza alta (`>= 70%`) -> mostra `N+1.0`
   - sticky band da ~`0.08 g` sui confini `.0/.5/1.0` per evitare rimbalzi continui
4) **WORK**: resta quantizzato a grammo intero con banda zero a `1.3 g` / `0.9 g` e isteresi tra interi.

### Anti-spike guard (RAW)
Taglia i glitch singoli (es. un salto momentaneo a valori negativi/assurdi): se un campione fa un salto > **150 g**,
viene scartato e accettato solo se il campione successivo conferma (oppure se anche il successivo resta oltre soglia,
così non rallentiamo i carichi rapidi).

Nota: la guard scarta solo **frame singoli** (o confermati) e quindi non cambia la logica WORK/zt/stati, se non in presenza di glitch reali.

### TARE
- Auto-TARE e tara manuale stimano lo zero con **trimmed mean**: ordinano i campioni e scartano le due code prima della media. Vibrazioni periodiche di frullatori o lavorazioni sul banco allargano il range ma non spostano necessariamente il centro, quindi il range resta diagnostico e non blocca la tara.
- La tara manuale parte dal rilascio del tasto, usa 64 campioni e rifiuta soltanto una deriva sostenuta fra l'inizio e la fine della finestra. Un doppio beep basso più l'audio opzionale `0016.mp3` segnalano un fallimento reale; un beep acuto conferma il successo.
- La tara di lavoro post-**ENTER** è immediata: usa lo stesso RAW WORK filtrato della pesata già validata, senza avviare una seconda finestra fallibile.

### Auto-TARE al boot
- Dopo **300 ms** di assestamento raccoglie fino a **64 campioni** in un massimo di **2 s**. Con almeno 32 campioni validi calcola sempre il centro robusto, scartando un ottavo dei valori per ogni coda, applica lo zero e completa il boot anche su un banco rumoroso.
- La qualità del segnale non è un gate di avvio: il range viene scritto nel log per diagnosi, mentre lo **zero-tracking** resta prudente e rifinisce gli scarti residui entro la propria finestra di ±1,5 g soltanto quando trova una successiva finestra quieta. Non viene allargato, così non può inseguire le vibrazioni o assorbire più facilmente un peso reale.
- Il boot si blocca soltanto se non arrivano almeno 32 campioni validi o se la calibrazione non consente di convertire i raw. In quel caso mostra **Sensore non valido**, emette sempre il doppio beep del buzzer anche senza DFPlayer, richiede **TARE** e riproduce opzionalmente `0016.mp3`.

Questa separazione è intenzionale: il rumore ambientale influenza la precisione istantanea ma non equivale a un guasto; assenza di campioni e calibrazione invalida sono invece errori tecnici che non permettono di costruire uno zero.

Anche la differenza fra TARE ed ENTER è intenzionale. TARE è una richiesta esplicita dell'operatore di assumere il centro corrente come zero e privilegia quindi l'usabilità sul banco reale; ENTER registra invece una quantità e conserva le verifiche strette di quiete, per non salvare una pesata sbagliata.

I parametri (N di media, isteresi, ecc.) sono nel firmware e sono pensati per essere ritoccati in base al tuo rumore reale.

---

## 7) TARA (UI e logica)

Tasto **TARE**:
- arma la tara alla pressione e avvia il campionamento dopo il rilascio debounced, così pressione e rilascio non contaminano la misura; una pressione trattenuta oltre 1,5 s annulla l'operazione
- dopo 60 ms di assestamento valuta gli ultimi 64 campioni, taglia gli 8 estremi per lato e applica il centro robusto normalmente entro circa 0,8–1,0 s dal rilascio
- il range non annulla più l'operazione; il confronto fra la media robusta dei primi e degli ultimi 16 campioni rifiuta soltanto una deriva oltre **10 g/s**, cioè un piatto ancora realmente in movimento. Il timeout tecnico è 1,6 s
- mostra “**- TARA -**” durante l’acquisizione; un movimento reale, campioni insufficienti o un blocco di sicurezza mostrano “**TARA FALLITA** / Premi TARA e riprova” e non modificano l’offset
- durante la tara la pesata è “bloccata” a display (l’utente non deve pesare)
- se la tara manuale riesce, emette un beep positivo, azzera lo stack e salva la tara di riferimento della sessione
- se fallisce, emette il doppio beep di errore e richiede opzionalmente `0016.mp3`; il buzzer resta autorevole anche con AUDIO OFF o DFPlayer assente
- è bloccato nello stato **SOVRACCARICO**, per evitare che un carico oltre il campo operativo venga nascosto impostandolo come zero

Zero di lavoro post-**ENTER**:
- in WORK accetta ENTER solo con HX in stato OK, snapshot fresco, assenza di sovraccarico e pesata realmente STABLE per la finestra configurata; in LIVE richiede almeno 400 ms di segnale quieto
- applica immediatamente come offset lo stesso snapshot RAW filtrato che ha prodotto la pesata accettata
- registra nello stack anche offset/zero-tracking precedenti e, per un comando session-aware, UUID, `product_id`, sessione e receipt del `confirm`; questi dati rendono reversibile il commit con CLEAR breve
- solo dopo il nuovo zero registra il peso nello stack e prepara il `confirm` MQTT; la response resta in retry finché Laravel l'ha persistita e il browser completa clear + ACK
- se una condizione fallisce, oppure MQTT è offline con un comando attivo, offset, stack e comando restano invariati
- non azzera lo stack e non aggiorna la tara di riferimento della sessione

Durante una tara o il breve commit post-ENTER gli altri tasti vengono ignorati, evitando doppi inserimenti e riavvii dell’operazione.
La tara manuale è rifiutata durante un upload OTA, che continua quindi senza interruzioni.

---

## 8) Batteria SLA 6 V (INA219)

### 8.1 Collegamenti INA219
- VCC → **3V3**
- GND → GND
- SDA → GPIO32
- SCL → GPIO33
- VIN+/VIN−: high-side sulla linea batteria (vedi schema del tuo cablaggio)

Se INA non viene trovato:
- prima cosa: controlla **SDA/SCL non invertiti**.
- Il bus I2C usa un timeout `Wire` di **50 ms** per limitare blocchi su transazioni INA219 anomale.

Validità runtime:
- tensione e corrente aggiornano stato, filtro e rilevamento charging soltanto se ogni lettura I2C termina con successo e i valori sono finiti e plausibili
- un errore di lettura non viene convertito in una falsa tensione bassa: resta disponibile l'ultimo campione valido per la sola UI, marcato con la propria età
- i limiti di plausibilità correnti sono 3–9 V sul bus, ±330 mV sullo shunt e ±3.500 mA
- countdown e sleep di protezione richiedono sempre un campione valido più recente di **1,5 s**; un valore stale non può spegnere la bilancia

### 8.2 Soglie tacche (default firmware, tensione filtrata)
Soglie “pratiche” (dipendono da carico/temperatura). Sono tarate per **usabilità UI** (tacche), non per SoC perfetto:
- **4 tacche (FULL)**: ≥ **6.20 V**
- **3 tacche (GOOD)**: ≥ **6.08 V**
- **2 tacche (LOW)**:  ≥ **5.95 V**
- **1 tacca (CRITICAL)**: ≥ **5.85 V**
- **0 tacche (EMPTY)**: < **5.85 V**

Nota: c'è un **cuscinetto** tra 0 tacche e lo stacco per batteria. La sequenza di countdown parte sotto **5.80 V**.

### 8.3 Rilevamento “in carica” (stabilizzato)
L’icona charging è basata sulla **corrente** (negativa = entra in batteria) con:
- isteresi (start/stop),
- debounce temporale,
- **min-on time** (per evitare flicker quando il caricatore/PWM stacca a impulsi).

Default firmware:
- entra in carica se **I < −80 mA**
- esce da carica se **I > −20 mA**
- debounce ingresso **1,5 s**, uscita **10 s**
- min-on **20 s**

---

## 9) Batteria scarica: avviso + light-sleep (protezione ESP)

La protezione qui è pensata per l’ESP32 (evitare latenze/reset quando il buck 5V perde margine).

Comportamento:
- **0 tacche (EMPTY)**: icona batteria **lampeggiante** + avviso sonoro
  - **0011.mp3** (batteria bassa) con **cooldown 5 min**
  - beep buzzer con **cooldown 5 min**
  - Nota: l'avviso 0011 è agganciato al livello **EMPTY (0 tacche)**, non a una finestra fissa di volt.
- Se V scende sotto **5,80 V** per ≥ **5 s**:
  - schermo “batteria scarica, collega alimentatore”
  - **0012.mp3** (batteria critica) **subito all'ingresso countdown** e **di nuovo a metà** (a ~60 s)
  - beep ogni **10 s** per **120 s**
  - poi entra in **LIGHT-SLEEP**
- La soglia hard-low deve restare confermata da campioni INA validi e freschi per circa **3 s** prima dello sleep. Questo lascia anche al debounce charging il tempo di riconoscere l'alimentatore.
- Prima del light-sleep per batteria scarica: DFPlayer viene **spento** via MOSFET (zero consumo audio in sleep).
- Prima della disconnessione pulita MQTT viene pubblicato retained lo stato `sleeping`, così il gestionale non conserva uno status `online` mentre la bilancia dorme.
- Anti-troncamento audio (DFPlayer): applicato un gap ~200 ms tra stop → play e ignorati i primi ~300 ms di BUSY=idle dopo il play (evita tagli tipo 0001→0002).
  - Nota HW: BUSY su GPIO39 non ha pull-up interno; se il BUSY resta instabile, aggiungere pull-up esterno verso 3V3 (10k..47k).
- Wake: premendo un tasto la tastiera risveglia l'ESP32; quel tasto viene consumato fino al rilascio e non può avviare anche TARE, ENTER, CLEAR, SKIP o un nuovo sleep

---


---

## 10) Risparmio energetico per inattività (5 minuti)
Se per **5 minuti** non viene premuto alcun tasto, il peso visualizzato resta fermo entro ±5 g e non esistono comando MQTT o response outbox pending, la bilancia entra in **LIGHT-SLEEP**.

Caratteristiche:
- Wake: **qualsiasi tasto**.
- L'inattività viene azzerata anche da una variazione del peso visualizzato **> 5 g** (in LIVE o WORK).
- Ogni `weigh`/clear MQTT accettato azzera il timer; finché un comando o una response restano pending l'auto-sleep è bloccato.
- Prima della sospensione MQTT viene pubblicato retained `state=sleeping`; il LWT `offline` resta riservato alle disconnessioni impreviste.
- Il tasto che provoca il wake viene ignorato come comando finché non viene rilasciato.
- **Nessun reset** dello stato/pesata: riprende esattamente dove era.
- WiFi/OTA vengono sospesi prima dello sleep e riattivati dopo il wake **solo se l'utente ha lasciato il WiFi ON**.
- **SLEEP breve**: al rilascio avvia la sequenza di standby manuale (schermata **Zzz...** per ~5 s + audio 0003), poi light-sleep anche se è presente un semplice comando `weigh` MQTT. Una response in attesa di receipt/ACK, un upload OTA o il sovraccarico continuano invece a bloccarlo.
- **SLEEP tenuto per 2 secondi**: non entra in standby; commuta il DFPlayer tra **AUDIO OFF** e **AUDIO ON**, con conferma sul display e buzzer.

### Indicatore esterno sleep (LED)
Quando il display è spento, per capire che la bilancia è in sleep, usa un LED su:
- **GPIO15** → **resistenza 330–2.2kΩ** → **anodo LED** → catodo a **GND** (HIGH = LED acceso).

## 11) WiFi + OTA (opzionale)

Il firmware HX711 supporta:
- WiFi **non bloccante** (background)
- OTA via Arduino IDE (porta di rete)

Dove si configura:
- Le credenziali **non** stanno nel codice e **non** vanno versionate.
- Si impostano via **seriale** e vengono salvate in **NVS** (persistono tra aggiornamenti firmware).

Note pratiche:
- di default è attivo **ENABLE_WIFI_OTA = 1**
- l’icona WiFi sul display segue lo stato di connessione (se WiFi è abilitato a compile-time)
- con due reti configurate, il firmware prova gli slot in sequenza; prima di passare allo slot successivo interrompe il tentativo STA corrente e attende una breve pausa non bloccante, cosi il nuovo SSID viene applicato davvero dal driver ESP32

Configurazione credenziali (max 2 reti, via seriale):
- `wifi set 1 "SSID" "PASS"`
- `wifi set 2 "SSID" "PASS"`
- `wifi creds` (lista SSID con password mascherata)
- `wifi creds showpass 1` / `wifi creds showpass 2` (recupero: stampa password in chiaro)
- `wifi clear 1` / `wifi clear 2` / `wifi clear all`
- `wifi apply` (ricarica credenziali e riavvia i tentativi senza reboot)

Password OTA (consigliata):
- `ota status`
- `ota set "PASS" [reboot]`
- `ota clear [reboot]`
- `ota reboot`

Note: la password non viene salvata in chiaro: in NVS resta solo un hash (MD5). Per rendere effettivi set/clear serve un reboot.

Persistenza:
- le credenziali restano in NVS anche dopo OTA/upload standard.
- si perdono solo con un **erase flash** completo o modifiche invasive alle partizioni.

Tasti:
- **WIFI**: abilita/disabilita il WiFi in modo **persistente** (salvato in NVS).
  - Feedback immediato al tasto: **bip** (buzzer). La connessione vera e propria avviene in background.
  - All’avvio il WiFi segue la preferenza utente, non l’ultimo stato momentaneo (es. WiFi spento in sleep).
  - Durante il light-sleep per inattività il WiFi viene comunque spento per consumi, ma al wake viene riattivato **solo** se la preferenza è ON.
  - Lo `scale_id` non dipende dallo stato del modulo WiFi: viene letto dal MAC STA in eFuse anche se la preferenza parte OFF. Se l'identità hardware non è valida, MQTT resta disabilitato invece di usare `000000000000`.
  - Audio:
    - **0005.mp3**: Wi‑Fi modulo ON (toggle manuale)
    - **0006.mp3**: Wi‑Fi modulo OFF (toggle manuale)
    - **0008.mp3**: connessione Wi‑Fi avvenuta (anche in background)
    - **0007.mp3**: errore connessione Wi‑Fi (connect failed / SSID non disponibile, con cooldown)
    - Dopo un wake da light-sleep la riconnessione **non** viene annunciata (comportamento come prima).

- **MODE**: cambio modalità WORK/LIVE con **bip** immediato (buzzer) + audio **0017/0018.mp3**.

Se non ti serve OTA/WiFi:
- imposta `#define ENABLE_WIFI_OTA 0` per ridurre consumi e complessità

---

## 12) MQTT — Integrazione gestionale

Il firmware comunica in tempo reale con il browser del gestionale Laravel/Backpack tramite Mosquitto. Il browser usa WSS, la bilancia MQTTS; Laravel gestisce associazioni, UI e persistenza REST, ma non è nel percorso MQTT real-time.

Abilitazione: `#define ENABLE_MQTT 1` in `net_ota_cloud.h` (richiede `ENABLE_WIFI_OTA 1`).

### Credenziali

Le credenziali MQTT (host, username, password) **non** sono nel codice sorgente. Si configurano via seriale e vengono salvate in NVS (persistono tra aggiornamenti firmware), come per WiFi e OTA.

Comandi seriali:
- `mqtt set "host" "user" "pass"` — salva credenziali MQTT in NVS
- `mqtt creds` — mostra host e user (password mascherata)
- `mqtt creds showpass` — mostra password in chiaro
- `mqtt clear` — cancella credenziali da NVS
- `mqtt apply` — ricarica credenziali e riconnette senza reboot
- `mqtt status` — mostra stato connessione, scale_id, comando attivo

Se le credenziali non sono configurate, MQTT resta inattivo (nessun tentativo di connessione).

Porta bilancia: **8883** (MQTTS/TLS). Porta browser: **8884** (WSS/TLS). Il certificato CA ISRG Root X1 è nel firmware. `MQTT_SCALE_NAME` e `MQTT_FW_VERSION` sono definiti in `net_ota_cloud.h`; la versione corrente è **1.3.0**.

### Topic e QoS effettivo

La bilancia si identifica con il MAC STA letto direttamente dall'eFuse ESP32 (lowercase, senza separatori, 12 hex), usato come `scale_id` nei topic MQTT. L'identità è disponibile anche con WiFi inizialmente OFF; il valore tutto zero è invalido e impedisce l'avvio MQTT.

| Topic | Direzione | QoS | Retain | Uso |
|---|---|---:|---:|---|
| `minu/scale/{scale_id}/command` | Browser → bilancia | 1 | sì | `weigh` e `clear` fenced da sessione e comando |
| `minu/scale/{scale_id}/response` | Bilancia → browser | 0 | no | `command_ack` transitorio; `confirm`, `skip` e `undo` durable con retry applicativo |
| `minu/scale/{scale_id}/ack` | Browser → bilancia | 1 | no | `response_ack` applicativo |
| `minu/scale/{scale_id}/status` | Bilancia → browser | online/sleeping 0; LWT offline 1 | sì | stato operativo, nome e versione firmware |
| `minu/scale/{scale_id}/owner` | Browser → browser | 1 | sì | lease della scheda che controlla la bilancia |

PubSubClient pubblica a QoS 0: per questo una response non viene considerata consegnata dal solo risultato di `publish()`. Il buffer della libreria viene impostato e verificato a runtime con `mqttClient.setBufferSize(512)`; il solo define `MQTT_MAX_PACKET_SIZE` non è sufficiente.

### Payload session-aware e command fence

Comando di pesatura:

```json
{"type":"weigh","uuid":"...","product_id":123,"name":"Zucchero","target_weight":450,"session_id":"...","command_id":"..."}
```

`command_id` identifica una singola attivazione del comando, è lungo 8–64 caratteri e ammette lettere ASCII, cifre, `-` e `_`. Il valore vuoto resta ammesso soltanto per compatibilità con i browser precedenti. Il Manager conserva l'ID sui retry soltanto quando sessione, UUID, prodotto, nome e target sono identici; una variazione semantica genera un ID nuovo. Un replay identico con lo stesso `command_id` non reinizializza il comando e riemette `command_ack`; lo stesso ID riutilizzato con contenuto diverso viene ignorato senza ACK e senza modificare lo stato attivo.

Pulizia comando:

```json
{"type":"clear","session_id":"...","command_id":"..."}
```

Senza response pending la bilancia accetta `clear` soltanto se la fence è coerente: un comando con `command_id` richiede lo stesso ID e la stessa sessione; un comando v1.2 senza ID ma con sessione richiede un clear senza ID della stessa sessione. Solo un comando realmente legacy, privo sia di `command_id` sia di `session_id`, può essere cancellato da un clear senza ID che riporta la sessione del Manager nuovo. Un clear vecchio non può quindi cancellare un'attivazione tokenizzata. Durante un outbox pending ogni `weigh` e `clear` viene ignorato.

Conferma transitoria di attivazione firmware:

```json
{"type":"command_ack","command_id":"...","session_id":"...","uuid":"...","product_id":123,"state":"active"}
```

Il firmware pubblica `command_ack` sul topic `response` dopo avere attivato un `weigh` con `command_id` e lo ripubblica per ogni replay identico dello stesso ID. Questo ACK è QoS 0, non usa l'outbox, non contiene `response_id` e non rappresenta una mutazione Laravel: se viene perso, il browser ripubblica lo stesso `weigh`. I comandi precedenti senza `command_id` non producono il nuovo ACK.

Response firmware:

```json
{"type":"confirm","uuid":"...","product_id":123,"session_id":"...","command_id":"...","response_id":"a1b2c3d40000012300000001","actual_weight":448.0}
{"type":"skip","uuid":"...","product_id":123,"session_id":"...","command_id":"...","response_id":"a1b2c3d40000012300000002"}
{"type":"undo","uuid":"...","product_id":123,"session_id":"...","response_id":"a1b2c3d40000012300000003","undo_of_response_id":"a1b2c3d40000012300000001"}
```

`confirm` e `skip` riportano il `command_id` originale quando il comando lo possiede. `undo` è prodotto soltanto da un CLEAR breve su una voce session-aware reversibile: non inventa un nuovo `command_id`, perché la correlazione durabile è già espressa da `undo_of_response_id`, che punta alla receipt del `confirm` originario. Una voce locale o legacy priva di tale receipt viene annullata soltanto sul firmware.

ACK browser:

```json
{"type":"response_ack","response_id":"a1b2c3d40000012300000001"}
```

Il firmware conserva una sola response session-aware in RAM e la ripubblica ogni **1 secondo** finché riceve l'ACK corrispondente. Il payload staged, inclusi `session_id`, l'eventuale `command_id` e `response_id`, resta byte-per-byte immutabile. Durante questa attesa ENTER, SKIP e altri commit reversibili sono bloccati; i `weigh` e `clear` MQTT ricevuti vengono ignorati senza `command_ack`.

Per firmware/browser v1.3 l'ordine è transazionale:

1. il browser invia a Laravel la response completa, inclusi `scale_id`, `response_id`, `product_id`, UUID e azione;
2. Laravel registra una receipt durabile e applica la mutazione nella stessa transazione; il replay identico è idempotente;
3. soltanto dopo commit o replay già processato il browser pubblica il `clear` retained e attende il PUBACK QoS 1;
4. soltanto dopo il PUBACK del clear pubblica `response_ack` e ne attende il PUBACK QoS 1;
5. infine aggiorna la UI. Su errore REST, conflitto o rete assente non invia clear/ACK: l'outbox firmware continua il retry.

Il PUBACK broker di `response_ack` non prova che il firmware fosse ancora online e l'abbia elaborato. Uno status `offline` o `sleeping` invalida quindi una consegna non conclusa; se il firmware ripubblica una response già persistita, il browser ripete clear e ACK senza ripetere REST o callback UI. Una response dello stesso prodotto della pagina, incluso `undo`, forza la riconciliazione anche se l'ingrediente desiderato è già avanzato o assente; una response di un altro prodotto viene conciliata senza cancellare il comando corrente.

La coppia (`scale_id`, `response_id`) resta l'identità della receipt. La `session_id` e l'eventuale `command_id` del payload staged descrivono l'origine e non vengono ritargettati. Solo un `response_ack` con la `response_id` esatta chiude l'outbox e azzera il comando; ACK estranei, nuovi comandi e clear non possono alterarlo. Se il browser desidera attivare un comando arrivato mentre l'outbox era pending, lo ripubblica dopo avere completato receipt, clear retained e ACK.

I browser precedenti restano supportati. Un comando v1.2 privo di `command_id` ma con sessione può essere cancellato solo da un clear anch'esso privo di ID con la stessa `session_id`; per un comando realmente legacy senza sessione il clear può riportare la sessione del Manager nuovo. Un comando privo di `session_id` usa response one-shot senza `response_id`, senza receipt/ACK applicativo e senza undo remoto.

### Ownership browser

Ogni scheda browser usa un `session_id` stabile durante reload e reconnect, un `connection_id` diverso per ogni documento e un nuovo `command_id` per ogni attivazione logica, conservato nei retry dello stesso comando. Un Web Lock esclusivo impedisce a una scheda duplicata di riusare l'ID copiato da `sessionStorage`; il fallback cross-tab ruota l'ID anche quando la presenza di un'altra istanza resta incerta. Il topic owner retained contiene anche `connection_id` e `lease_id`: il controllo è confermato soltanto dall'eco esatta di utente, sessione, connessione e lease. Il proprietario rinnova il lease ogni **10 secondi**; dopo **30 secondi** senza rinnovo un'altra scheda può reclamarlo. Anche due schede dello stesso utente sono quindi istanze distinte. Una scheda non proprietaria resta connessa, non pubblica comandi e mostra chi detiene il controllo.

**Alla connessione:**
- la bilancia pubblica status `online` retained con scale ID, nome e firmware version;
- registra un LWT `offline` retained;
- si sottoscrive a `command` e `ack` richiedendo QoS 1;
- il browser attende i SUBACK di response/status/owner/command, uno status fresco e la conferma esatta del lease prima di pubblicare o ripubblicare un comando attivo; una coorte SUBACK incompleta o rifiutata resta non-ready e viene ritentata per intero sullo stesso tentativo di connessione;
- il browser non registra un LWT che modifichi `command`: su `pagehide` invalida i callback e chiude localmente il socket senza pubblicare clear o owner release;
- la UI è verde soltanto dopo l'eco retained del comando e il relativo `command_ack` firmware; durante reconnect o riparazione resta in sincronizzazione.

Cambio e disassociazione bilancia sono bloccati finché una response è in sincronizzazione. Se la response inizia mentre il salvataggio del cambio è già in corso, la transizione MQTT attende in modo non bloccante la fine della consegna. Clear e rilascio owner sulla vecchia bilancia vengono pubblicati soltanto dalla connessione che possiede il lease esatto.

**Comando `weigh`:** il browser invia UUID ingrediente, `product_id`, nome e peso target. La bilancia emette un bip distintivo di ricezione, azzera il timer inattività e mostra il target sul display (icona target + grammi).

**Comando `clear`:** per i comandi tokenizzati annulla soltanto la stessa coppia `session_id + command_id`; le eccezioni compatibili per v1.2/legacy sono quelle descritte sopra. Non modifica lo stack locale.

### Tasti (con MQTT attivo)

- **ENTER**: quando MQTT è connesso richiede un comando `weigh` attivo; senza comando emette un warning e non applica tara, push o audio di successo. Con comando valido prepara `confirm` e resta pending fino alla receipt Laravel e all'ACK browser. In modalità standalone il commit locale resta disponibile quando MQTT non è connesso o il WiFi è spento
- **SKIP breve**: scatta al rilascio e prepara `skip` per il comando attivo; senza comando emette un buzzer di avviso
- **SKIP tenuto 5 secondi**: apre il wizard calibrazione senza inviare prima uno `skip`
- **CLEAR breve**: annulla subito una voce locale; per una voce con receipt session-aware prepara `undo` e ripristina offset/zero-tracking soltanto al relativo ACK
- Un secondo ENTER/SKIP/CLEAR reversibile durante il pending viene rifiutato, evitando modifiche e response sovrapposte

### Display MQTT

- **Icona MQTT** (frecce ↑↓) nella barra di stato: visibile quando il WiFi è connesso
  - Fissa: connesso al broker
  - Lampeggiante: disconnesso dal broker (in attesa di riconnessione)
- **Riga info**: valori only, senza label (`WORK | STABLE` / `WORK | UNSTABLE`); in modalità LIVE mostra solo `LIVE`
- **Ingrediente**: quando arriva un comando `weigh`, il nome ingrediente è mostrato sulla riga sotto (max 15 caratteri; oltre: 15 + tre puntini ravvicinati)
- **Peso target**: quando è attivo un comando `weigh`, il peso obiettivo viene mostrato accanto all'icona target

### Disconnessione e riconnessione

- Alla disconnessione dal broker: **doppio beep** buzzer + icona MQTT lampeggiante
- Riconnessione automatica con **backoff esponenziale** (2s → 4s → 8s → 16s → 30s max)
- Durante il TLS handshake MQTT il WDT resta attivo ma viene portato temporaneamente a **20s**; finito l'handshake torna a **8s**
- PubSubClient usa keepalive **15s** e socket timeout **2s**, così una connessione half-open non blocca a lungo il loop
- Prima di una sospensione pulita pubblica retained `sleeping`, poi esegue DISCONNECT; il retained non resta quindi falsamente `online`. Il LWT `offline` copre le cadute impreviste.
- Comando e outbox RAM restano disponibili e, al wake, la connessione e gli eventuali retry vengono ripristinati
- Il log di sospensione (`[MQTT] Sospeso`) viene emesso solo se c'era stato MQTT attivo da chiudere, evitando spam seriale quando il WiFi e' giu

### NTP

Il firmware sincronizza l'orologio via NTP (`pool.ntp.org`) all'avvio, necessario per la validazione del certificato TLS. I tentativi di connessione MQTT sono rinviati finché il clock non è sincronizzato.

---

## 13) Weigh Stack — Stack pesate locale

Stack pesate in RAM (LIFO, max 50 elementi). Ogni voce conserva grammi, offset e zero-tracking precedenti; le pesate session-aware conservano inoltre UUID, `product_id`, sessione e `response_id` del confirm. Lo stack sopravvive al light-sleep ma viene azzerato a ogni reboot, incluso un reset WDT.

### Workflow tipico

1. Metti contenitore, premi **TARA** → avvia tara manuale; se riesce azzera stack e salva la tara di riferimento
2. Aggiungi ingrediente, attendi **STABLE** e premi **ENTER** → zero di lavoro immediato, registrazione nello stack e `confirm` MQTT (display torna a 0)
3. Ripeti per ogni ingrediente
4. **TOTAL** (breve) → mostra overlay di controllo con **Registrato**, **Effettivo** e **Differenza** (10 secondi)
5. **CLEAR** (breve) → annulla realmente l'ultima pesata: subito se locale, oppure dopo receipt/ACK dell'`undo` se associata a Laravel; a quel punto ripristina offset/zero-tracking e rimuove la voce LIFO
6. **CLEAR** (2 secondi) → svuota soltanto lo stack locale, senza produrre una serie di undo remoti

### Tasti

| Tasto | Breve | Lungo (2s) |
|---|---|---|
| **TARE** | Tara manuale; se riesce azzera stack e salva riferimento | — |
| **ENTER** | Valida peso + zero di lavoro + push + MQTT confirm | — |
| **TOTAL** | Mostra overlay confronto: Registrato / Effettivo / Differenza | — |
| **CLEAR** | Avvia undo LIFO; per una voce remota applica il ripristino dello zero dopo l'ACK | Svuota solo lo stack locale |

**Note:**
- ENTER con peso <= 0: ignorato (nessun push, nessuna tara, nessun MQTT)
- ENTER con HX non OK, snapshot vecchio o peso instabile: ignorato senza modificare offset, stack o comando MQTT
- ENTER e TARE in **SOVRACCARICO**: bloccati senza modificare zero, stack o MQTT
- Lo zero di lavoro post-ENTER NON azzera lo stack e NON aggiorna il riferimento
- Se la tara manuale fallisce per deriva reale o campioni insufficienti, offset, riferimento e stack restano invariati
- Le overlay si chiudono dopo 10 secondi; l'overlay TOTAL si chiude anche con TOTAL, TARE o ENTER
- Long press "solido" usato su CLEAR: l'azione breve scatta al rilascio solo se la soglia non è stata raggiunta. Il clear lungo è una manutenzione locale e non annulla in massa le azioni già persistite in Laravel.

### Comandi seriali

- `stack [status]` — mostra contatore, totale, stato riferimento
- `stack list` — lista tutti gli elementi con indice e peso
- `stack clear` — svuota lo stack

### Interazione con MQTT

- ENTER session-aware esegue: validazione → zero di lavoro → push reversibile → staging del `confirm` → associazione della receipt generata alla voce. Se lo staging non riesce, push e zero vengono annullati.
- CLEAR breve legge la voce senza rimuoverla. Se contiene una receipt remota, prepara un nuovo outbox `undo` con `undo_of_response_id`; offset/zero-tracking vengono ripristinati e la voce viene rimossa soltanto dopo l'ACK browser, quindi dopo la persistenza Laravel. Se il primo publish QoS 0 fallisce, lo stato locale resta invariato e l'outbox continua i retry.
- Una voce locale o legacy senza receipt viene annullata subito e solo localmente; non viene inventato un undo Laravel non correlabile.
- Durante una response in attesa di receipt/ACK sono bloccati TARE, ENTER, SKIP, qualsiasi CLEAR incluso quello lungo e le mutazioni del wizard di calibrazione. Da seriale restano bloccati `stack clear` e tutti i comandi `cal ...` mutanti; `cal status` resta consultabile.
- Il payload MQTT `clear` inviato dal browser non equivale al tasto CLEAR: pulisce soltanto UUID, target e sessione del comando retained e non modifica lo stack.

---

## 14) Troubleshooting rapido (i classici)

**DOUT sempre HIGH / letture 0 fisse**
- spesso SCK resta HIGH → HX711 in power-down
- verifica pulldown 100 kΩ su SCK e che non ci sia un level shifter “I2C” su SCK

**INA219 “non trovato”**
- SDA/SCL invertiti (errore più comune)
- indirizzo diverso da 0x40 (raro, ma possibile)

**Rumore alto a 80 SPS**
- normale: si gestisce con decimazione/filtri e con layout/cavi puliti (cella, massa, schermature se serve)

---

## 15) Firmware e cartelle

Firmware corrente (HX711):
- `firmware/esp32_hx711_serial/`

Archivio (legacy, non mantenuto):
- `archive/esp32_nau7802_serial/`

> Nota: la documentazione operativa (cablaggi, comandi seriali, DFPlayer, sleep, debug) è in questo README.

### Architettura modulare

Il firmware è organizzato in moduli con namespace C++ per una migliore manutenibilità:

```
firmware/esp32_hx711_serial/
├── esp32_hx711_serial.ino   # Main loop, comandi seriali e gestione tasti
├── config/
│   ├── config_pins.h        # Pin hardware (HX711, I2C, SPI, keypad, buzzer, DFPlayer)
│   ├── config_audio.h       # AudioConfig:: (volume, timing, tracce MP3)
│   ├── config_scale.h       # ScaleConfig:: (filtri, stati, ZT, display)
│   └── config_battery.h     # BatteryConfig:: (soglie tensione, timing shutdown)
├── audio.h / audio.cpp      # Audio:: (DFPlayer: play, coda, power-gating)
├── scale_filters.h / .cpp   # ScaleFilters:: (mediana, MA, spike guard, storico)
├── scale_state.h / .cpp     # ScaleState:: (offset, tara, stati, ZT, display)
├── dfplayer_driver.h / .cpp # DFPlayer:: (UART, comandi base)
├── buzzer.h / .cpp          # Buzzer:: (beep, toni)
├── hx711_driver.h / .cpp    # HX711 low-level (SCK/DOUT, read)
├── keypad.h / .cpp          # Keypad:: (4x2, debounce, one-shot, wake suppression)
├── battery_monitor.h / .cpp # BatteryMonitor:: (INA219, validità/freshness, charging)
├── hx_health.h / .cpp       # HxHealth:: (OK/WARN/ERROR/ERROR_HARD)
├── ui_display.h / .cpp      # UiDisplay:: (OLED SSD1322, layout, icone)
├── net_ota_cloud.h / .cpp   # Net:: (WiFi/OTA/MQTT, outbox confirm/skip/undo)
├── mqtt_store.h / .cpp      # MqttStore:: (credenziali MQTT in NVS)
├── weigh_stack.h / .cpp     # WeighStack:: (stack LIFO reversibile + receipt metadata)
└── calibration_wizard.h/.cpp # CalWizard:: (wizard calibrazione on-display)
```

**Namespace principali:**
- `Audio::` — gestione DFPlayer (coda FIFO, priorità, power-gating, anti-troncamento)
- `ScaleFilters::` — filtri segnale (mediana 3, media mobile, spike guard, storico range/slope)
- `ScaleState::` — macchina a stati (STABLE/UNSTABLE/LIVE), zero-tracking, tara, quantizzazione display
- `Net::` — WiFi/OTA/MQTT (TLS, identity eFuse, status, comandi e outbox response)
- `MqttStore::` — persistenza credenziali MQTT in NVS
- `WeighStack::` — stack pesate locale reversibile (grammi, offset/ZT precedenti, provenienza e receipt)
- `ScaleConfig::` / `AudioConfig::` / `BatteryConfig::` — parametri configurabili (soglie, timing, tracce)

### Task Watchdog
Il firmware include un **Task Watchdog** (8 secondi) attivo già durante il setup, con reset esplicito nei loop di boot intenzionali. Nel loop principale viene resettato a ogni iterazione; durante il TLS MQTT viene esteso temporaneamente a 20s.

All'avvio il firmware verifica che il `loopTask` sia davvero iscritto al WDT: in seriale stampa `[WDT] OK` se l'aggancio e' attivo, oppure `[WDT] FAIL i=... a=... s=...` se init/add/status falliscono. Il reset periodico del watchdog viene eseguito solo dopo questa verifica.

Dopo un reset WDT la sessione runtime viene azzerata in modo coerente: stack, riferimento, offset di lavoro, zero-tracking e filtri. Nessuna tara runtime parziale viene recuperata da RTC. La UI informa del reset e avvia direttamente una nuova Auto-TARE robusta: il rumore non blocca il recupero, mentre l'assenza di campioni validi mantiene il blocco tecnico con richiesta TARE. La calibrazione persistente in NVS resta valida.

### DFPlayer Mini (audio eventi) + power-gating controllato
Il firmware può suonare file MP3 (es. avviso sleep). Per evitare click e stati strani, **non fa power-cycle a fine brano**.

Comportamento attuale:
- DFPlayer viene portato in stato **pronto** automaticamente **all’avvio** e a ogni **wake** (alimentazione ON + UART init + volume), così un suono può partire subito.
- La riproduzione MP3 è progettata per essere **non bloccante**: mentre l’audio suona, la bilancia continua a leggere HX711 e ad aggiornare UI/log.
- I comandi UART verso DFPlayer vengono inviati senza `flush()` bloccanti: se il modulo audio o UART1 si pianta, il firmware non resta fermo in attesa infinita dello svuotamento TX.
- **Gestione priorità audio**: alcuni MP3 sono **non interrompibili** e, se richiesti mentre un altro non interrompibile sta suonando, vengono messi in **coda FIFO** (solo fra loro). Tutti gli altri MP3 sono **interrompibili**: interrompono l’audio interrompibile in corso e ripartono subito, **senza accodarsi**. Se è in corso un non interrompibile, le richieste interrompibili vengono ignorate (resta il **bip** del buzzer sui tasti).
  - Non interrompibili: **0002, 0007, 0008, 0012, 0013, 0014, 0015, 0016**.
- Durante l’uso resta alimentato, salvo recovery automatico/manuale.
- Se un brano supera il timeout di riproduzione, il firmware fa un **hard reset non bloccante** del DFPlayer: chiude UART, porta GPIO2 LOW per ~800 ms, riaccende il modulo, reinizializza UART/volume e svuota la coda audio.
- **SLEEP tenuto per 2 secondi** commuta manualmente il DFPlayer:
  - da ON a OFF: chiude UART, svuota la coda, taglia VCC via GPIO2 e salva uno snapshot diagnostico;
  - da OFF a ON: esegue un power-cycle non bloccante e mostra prima `RIAVVIO MODULO...`, poi `DFPLAYER PRONTO`;
  - lo stato OFF resta valido durante il light-sleep e il wake; dopo un reboot completo l'audio riparte abilitato.
- Il comando seriale `mp3 reset` forza lo stesso hard reset per debug da banco.
- Viene spento **quando la bilancia entra in standby/light-sleep per inattività** e anche **prima del light-sleep per batteria scarica**.

Diagnostica audio:
- un ring buffer RAM conserva gli ultimi **24 eventi** (`request`, `play`, assenza transizione BUSY, timeout, reset, ready e toggle manuali); sopravvive al light-sleep ma non a un'interruzione di alimentazione;
- al timeout e quando l'operatore passa manualmente ad AUDIO OFF viene salvato in NVS un solo snapshot compatto con stato, traccia, BUSY e comando GPIO2; non vengono eseguite scritture flash per ogni riproduzione;
- `mp3 history` stampa snapshot persistente e cronologia RAM; `mp3 history clear` cancella entrambi;
- `mp3 status` include anche enable manuale, stato alimentazione firmware, UART e livello GPIO2.

Nota: il firmware mantiene comunque il percorso “cold-start” sul primo `mp3` (utile se in futuro vuoi tornare all’accensione on-demand).

### Collegamenti minimi
- **ESP32 GPIO4 (TX1)** → **1kΩ in serie** → **DFPlayer RX**
- **DFPlayer TX** → **ESP32 GPIO34 (RX1)** (opzionale, ma consigliato)
- **DFPlayer BUSY** → **ESP32 GPIO39**
  - BUSY è tipicamente **3.3V** (ok per ESP32)
  - GPIO39 non ha pull interni: se a riposo il segnale risulta instabile/flottante, aggiungi **pull-up 10k..47k a 3V3**
  - evita il pulldown a GND (rischi di leggere "busy" fisso)
- **Altoparlante**: usa **SPK1/SPK2** (8Ω ok)

### Power-gating (high-side) consigliato
- **ESP32 GPIO2** comanda l'alimentazione DFPlayer (HIGH=ON) tramite **NPN + P-MOSFET high-side**.

> Nota: evitiamo di usare il comando "sleep" interno del DFPlayer (0x0A) perché su molti cloni non si risveglia in modo affidabile; per risparmio batteria serio è meglio tagliare VCC.

### Bypass per test (senza MOSFET)
Per provare oggi:
- collega **DFPlayer VCC direttamente a +5V** e **GND a GND**
- lascia GPIO2 non connesso (o connesso ma senza circuito), il firmware funziona lo stesso
- in bypass il comando `mp3 reset` può reinizializzare UART/stato firmware, ma **non può tagliare davvero VCC** al DFPlayer

> Nota: non usiamo più il comando DFPlayer "sleep" (0x0A) durante i test perché su molti cloni non si risveglia in modo affidabile.
> Per il risparmio energetico vero, la strada solida è tagliare VCC con MOSFET high-side.

### Comandi seriale
- `mp3 1` (suona `/MP3/0001.mp3`)
- `mp3 1 5` (cap a 5s, solo come paracadute)
- `stop` (stop della riproduzione; **non** spegne il DFPlayer)
- `vol 20`
- `mp3 status`
- `mp3 reset` (hard reset DFPlayer via power-gate; richiede MOSFET/GPIO2 per tagliare VCC)
- `mp3 history` (snapshot persistente + ultimi 24 eventi RAM)
- `mp3 history clear` (cancella la diagnostica audio)

### Calibrazione

La bilancia può essere calibrata in due modi:
1. **Wizard on-display** — per calibrazione guidata senza PC
2. **Comandi seriali** — per calibrazione remota/debug

#### Wizard calibrazione (on-display)

Il wizard si attiva tenendo premuto **SKIP** per 5 secondi.

**Passi del wizard:**

| Step | Schermata | Azione | Tasti |
|---:|---|---|---|
| 1/4 | ZERO | Piatto vuoto, acquisisci offset | **ENTER** = conferma, **CLEAR** = annulla |
| 2/4 | PLACE | Appoggia peso di riferimento | **ENTER** = conferma, **CLEAR** = annulla |
| 3/4 | VALUE | Seleziona peso in grammi | **SKIP** = +, **TARE** = -, **ENTER** = vai a conferma |
| 4/4 | CONFIRM | Verifica CPG calcolato | **ENTER** = salva in NVS, **CLEAR** = annulla |

**Selezione peso:**
- Da 500g a 20kg
- Step di 500g sotto 2000g
- Step di 50g sopra 2000g

**Validazione:**
- Il CPG deve essere nel range 20-1000 (supporta diverse celle di carico)
- Se fuori range, il wizard rifiuta la calibrazione

**Feedback:**
- Beep singolo: conferma step
- Beep doppio + schermata **SALVATA**: calibrazione scritta e riletta da NVS
- Schermata **ERRORE**: salvataggio o verifica NVS fallita
- Doppio beep basso: errore/annullato

#### Calibrazione via seriale

Per calibrazione remota o debug:

- `cal status` — stampa offset, CPG, ZT counts, grammi attuali
- `cal zero` — imposta il valore raw corrente come nuovo offset (tara "permanente")
- `cal ref <grammi>` — con un peso noto posizionato, calcola il nuovo CPG
- `cal save` — salva offset e CPG in NVS e segnala errore se `Preferences` fallisce
- `cal load` — ricarica offset e CPG da NVS

**Procedura di calibrazione seriale:**
1. Bilancia vuota, esegui `cal zero`
2. Posiziona un peso noto (es. 500g), esegui `cal ref 500`
3. Verifica con `cal status` che i grammi siano corretti
4. Salva con `cal save`

**Note:**
- `cal zero` e `cal ref` sono disabilitati se HX health è in ERROR
- tara/calibrazione sono bloccate durante **SOVRACCARICO**
- `cal ref` rifiuta valori ≤ 0 e CPG fuori range 20-1000
- Dopo `cal zero` o `cal ref` i filtri vengono resettati automaticamente
- dopo WDT viene ricaricata la calibrazione NVS senza applicare una tara runtime RTC

### Legenda MP3 eventi (cartella /MP3)
Metti i file in **SD:/MP3/** con nome a 4 cifre (es. `0001.mp3`).

| File | Evento |
|---:|---|
| 0001.mp3 | Avvio bilancia |
| 0002.mp3 | Boot completato |
| 0003.mp3 | Entrata risparmio energetico (standby manuale o automatico: schermata Zzz... 5s prima del light-sleep) |
| 0004.mp3 | Uscita risparmio energetico (wake) |
| 0005.mp3 | Wi‑Fi **modulo ON** (toggle manuale) |
| 0006.mp3 | Wi‑Fi **modulo OFF** (toggle manuale) |
| 0007.mp3 | Errore connessione Wi‑Fi (connect failed / SSID non disponibile, con cooldown) |
| 0008.mp3 | Connessione Wi‑Fi avvenuta (anche in background) |
| 0011.mp3 | Batteria bassa (una sola volta all'ingresso in 0 tacche, beep resta attivo) |
| 0012.mp3 | Batteria critica (una sola volta all'ingresso fase critica, beep resta attivo) |
| 0013.mp3 | Standby pre‑sleep per batteria scarica (schermata Zzz... 5s prima del light-sleep) |
| 0014.mp3 | Errore lettura batteria (INA) |
| 0015.mp3 | Errore sensore peso (HX) |
| 0016.mp3 | Tara non eseguita, riprovare (errore tecnico al boot o movimento reale nella tara manuale; il buzzer suona comunque) |
| 0017.mp3 | Modalità WORK |
| 0018.mp3 | Modalità LIVE |


### Debug seriale HX711 (opzionale)
Di default il log continuo del sensore è disattivato.

Nota: il parser comandi seriali è **non bloccante** (nessun timeout/attesa). I comandi vengono eseguiti solo quando invii una riga completa (terminata da invio). 
Comandi:
- `hxlog on` / `hxlog off`
- `hxlog ?`
- `hxlog rate <ms>` (50..5000)


### Tastiera: debounce + one-shot + long press
La tastiera ha un debounce software (40 ms) e genera eventi **one-shot**: un tasto premuto produce **un solo evento**, anche se lo tieni premuto.

Se un tasto o una linea resta chiusa per almeno 15 secondi, quel tasto viene soppresso fino al rilascio elettrico: il loop continua a girare e gli altri tasti restano leggibili.

Dopo un wake il tasto che ha risvegliato la bilancia resta soppresso fino al rilascio: il wake non genera una seconda azione applicativa.

**Long press:**
- **SKIP breve**: l'azione scatta al rilascio
- **SKIP tenuto per 5 secondi**: avvia il wizard di calibrazione on-display senza eseguire prima lo SKIP breve/MQTT
- **CLEAR breve**: al rilascio avvia l'undo LIFO reale
- **CLEAR tenuto per 2 secondi**: svuota soltanto lo stack locale, senza undo Laravel multipli
- **SLEEP breve**: entra in standby al rilascio
- **SLEEP tenuto per 2 secondi**: commuta DFPlayer AUDIO OFF/ON, salva il log quando passa a OFF e annulla lo standby
