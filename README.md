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
- La tara di lavoro post-**ENTER** è immediata quando il peso è già quieto. Se non lo è, ENTER apre un'acquisizione visibile di massimo **1,5 s**: appena arriva STABLE registra e tara; al timeout usa un centro RAW robusto soltanto se non rileva una deriva continua.

### Auto-TARE al boot
- Dopo **300 ms** di assestamento raccoglie fino a **64 campioni** in un massimo di **2 s**. Con almeno 32 campioni validi calcola sempre il centro robusto, scartando un ottavo dei valori per ogni coda, applica lo zero e completa il boot anche su un banco rumoroso.
- La qualità del segnale non è un gate di avvio: il range viene scritto nel log per diagnosi, mentre lo **zero-tracking** resta prudente e rifinisce gli scarti residui entro la propria finestra di ±1,5 g soltanto quando trova una successiva finestra quieta. Non viene allargato, così non può inseguire le vibrazioni o assorbire più facilmente un peso reale.
- Il boot si blocca soltanto se non arrivano almeno 32 campioni validi o se la calibrazione non consente di convertire i raw. In quel caso mostra **Sensore non valido**, emette sempre il doppio beep del buzzer anche senza DFPlayer, richiede **TARE** e riproduce opzionalmente `0016.mp3`.

Questa separazione è intenzionale: il rumore ambientale influenza la precisione istantanea ma non equivale a un guasto; assenza di campioni e calibrazione invalida sono invece errori tecnici che non permettono di costruire uno zero.

Anche la differenza fra TARE ed ENTER è intenzionale. TARE è una richiesta esplicita dell'operatore di assumere il centro corrente come zero. ENTER registra invece una quantità: prova prima la quiete normale e, solo dopo il timeout visibile, accetta la media robusta di un liquido che oscilla attorno a un centro fermo; una variazione ancora direzionale viene rifiutata.

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
- con HX OK, snapshot fresco, peso positivo e assenza di sovraccarico, un peso già STABLE in WORK o quieto da almeno 400 ms in LIVE viene registrato subito
- se il gate normale non è pronto, emette un beep basso e mostra **ACQUISIZIONE PESO** con barra di avanzamento per un massimo di **1,5 s**; appena il peso diventa quieto registra e tara senza attendere la fine della barra
- al timeout calcola un centro robusto sui campioni RAW successivi alla pressione. Il range resta diagnostico per tollerare l'oscillazione dei liquidi; una deriva oltre **4 g/s**, campioni insufficienti o non freschi rifiutano l'operazione. Il valore robusto deve inoltre restare nel campo ±16 kg
- nel fallback applica come nuovo zero lo stesso centro RAW da cui ricava i grammi registrati
- registra nello stack anche offset/zero-tracking precedenti e, per un comando session-aware, UUID, `product_id`, sessione e receipt del `confirm`; questi dati rendono reversibile il commit con CLEAR breve
- solo dopo il nuovo zero registra il peso nello stack e prepara il `confirm` MQTT; la response resta in retry finché Laravel l'ha persistita e il browser completa clear + ACK
- durante l'attesa ricontrolla sensore, calibrazione, offset e identità esatta del comando MQTT; se una condizione cambia, oppure MQTT va offline con un comando attivo, offset, stack e comando restano invariati
- non azzera lo stack e non aggiorna la tara di riferimento della sessione

Durante acquisizione e feedback ENTER gli altri tasti vengono ignorati, evitando doppi inserimenti. Il successo **REGISTRATO / Tara applicata** resta visibile per circa **1,3 s** dopo il beep; il rifiuto mostra **PESO IN MOVIMENTO / NON REGISTRATO** oppure **ACQUISIZIONE FALLITA / NON REGISTRATO** per circa **2 s** dopo il doppio beep. Tutte queste schermate si chiudono da sole.
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
Se per **5 minuti** non viene premuto alcun tasto, il peso visualizzato resta fermo entro ±5 g e non esistono comando MQTT operativo o response outbox ancora agganciato al Manager, la bilancia entra in **LIGHT-SLEEP**. Un outbox già passato alla modalità locale continua a vivere in RAM e riprende i retry al wake senza tenere acceso il display.

Caratteristiche:
- Wake: **qualsiasi tasto**.
- L'inattività viene azzerata anche da una variazione del peso visualizzato **> 5 g** (in LIVE o WORK).
- Ogni `weigh`/clear MQTT accettato azzera il timer; un comando operativo o una response ancora agganciata al Manager bloccano l'auto-sleep, mentre dopo il fallback locale non lo bloccano più.
- Prima della sospensione MQTT viene pubblicato retained `state=sleeping`; il LWT `offline` resta riservato alle disconnessioni impreviste.
- Il tasto che provoca il wake viene ignorato come comando finché non viene rilasciato.
- **Nessun reset** dello stato/pesata: riprende esattamente dove era.
- WiFi/OTA vengono sospesi prima dello sleep e riattivati dopo il wake **solo se l'utente ha lasciato il WiFi ON**.
- **SLEEP breve**: al rilascio avvia la sequenza di standby manuale (schermata **Zzz...** per ~5 s + audio 0003), poi light-sleep anche se è presente un semplice comando `weigh` MQTT. Se il worker MQTT sta connettendo, la richiesta resta accodata e lo sleep parte appena il trasporto viene rilasciato, senza richiedere una seconda pressione. Una response ancora nel lock operatore, un upload OTA o il sovraccarico continuano invece a bloccarlo; una response già detached no.
- **SLEEP tenuto per 2 secondi**: non entra in standby; commuta il DFPlayer tra **AUDIO OFF** e **AUDIO ON**, salva la scelta in NVS e la mantiene dopo riavvii e spegnimenti completi, con conferma sul display e buzzer.

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

Porta bilancia: **8883** (MQTTS/TLS). Porta browser: **8884** (WSS/TLS). Il certificato CA ISRG Root X1 è nel firmware. `MQTT_SCALE_NAME` e `MQTT_FW_VERSION` sono definiti in `net_ota_cloud.h`; la versione corrente è **1.5.2**.

### Topic e QoS effettivo

La bilancia si identifica con il MAC STA letto direttamente dall'eFuse ESP32 (lowercase, senza separatori, 12 hex), usato come `scale_id` nei topic MQTT. L'identità è disponibile anche con WiFi inizialmente OFF; il valore tutto zero è invalido e impedisce l'avvio MQTT.

| Topic | Direzione | QoS | Retain | Uso |
|---|---|---:|---:|---|
| `minu/scale/{scale_id}/command` | Browser → bilancia | 1 | `weigh`/`clear` sì; `confirm_request` no | comando retained e richiesta ENTER remota fenced |
| `minu/scale/{scale_id}/response` | Bilancia → browser | 0 | no | `command_ack`/`confirm_request_ack` transitori; `confirm`, `skip` e `undo` durable con retry applicativo |
| `minu/scale/{scale_id}/ack` | Browser → bilancia | 1 | no | `response_ack` o `response_reject` terminale |
| `minu/scale/{scale_id}/status` | Bilancia → browser | online/sleeping 0; LWT offline 1 | sì | heartbeat diagnostico e stato operativo |
| `minu/scale/{scale_id}/owner` | Browser → browser + bilancia | 1 | sì | lease della scheda e `expected_command_id` |

PubSubClient pubblica a QoS 0: per questo una response non viene considerata consegnata dal solo risultato di `publish()`. Il buffer della libreria viene impostato e verificato a runtime con `mqttClient.setBufferSize(640)`; il solo define `MQTT_MAX_PACKET_SIZE` non è sufficiente. L'envelope owner resta bounded perché `user_name` è normalizzato a massimo 48 byte UTF-8 e ciascun ID a 64 caratteri, incluso `expected_command_id`; lo stesso limite copre lo status diagnostico completo.

Firmware 1.5.2 ripubblica lo status retained ogni **5 secondi** e ad ogni variazione diagnostica. Oltre ai campi discovery contiene `boot_id`, `transport_id`, `status_seq`, `published_at` epoch, `uptime_ms`, `operational_mode`, `tare_required`, `fallback_reason`, `last_command_error`, `rx_probe_seq`, `rx_mask`, `rx_health` e `pending_response_id`. `tare_required` è un booleano esplicito: quando un incidente di sincronizzazione non è rollbackabile resta `true`, con `operational_mode=local`, finché una TARE fisica termina con successo. `rx_mask` usa bit owner=1, command=2, ack=4; `rx_probe_seq`, mantenuto per compatibilità dello schema, conta le generazioni SUBSCRIBE/resubscribe del watchdog passivo. `published_at` permette al Manager di non scambiare uno status retained online vecchio per una connessione corrente.

### Payload session-aware e command fence

Comando di pesatura:

```json
{"type":"weigh","uuid":"...","product_id":123,"name":"Zucchero","target_weight":450,"session_id":"...","connection_id":"...","command_id":"..."}
```

`command_id` identifica una singola attivazione del comando, è lungo 8–64 caratteri e ammette lettere ASCII, cifre, `-` e `_`. Il valore vuoto resta ammesso soltanto per compatibilità con i browser precedenti. Il Manager conserva l'ID sui retry soltanto quando sessione, UUID, prodotto, nome e target sono identici; una variazione semantica genera un ID nuovo. Un replay identico con lo stesso `command_id` non reinizializza il comando e riemette `command_ack`; su reload aggiorna soltanto il `connection_id` del documento corrente. Lo stesso ID riutilizzato con contenuto business diverso viene ignorato senza ACK e senza modificare lo stato attivo.

Da firmware 1.4.1 un `weigh` dotato di `connection_id` è attivabile soltanto quando il retained `owner` è fresco e coincide con la sua coppia sessione/connessione. La bilancia calcola la vita residua dai timestamp originali del lease, quindi un owner retained vecchio non viene ringiovanito al reconnect. Il lease di controllo resta di 30 secondi; da 1.5.0 la presenza operativa scade dopo 25 secondi dall'arrivo dell'ultimo owner valido, così tollera un heartbeat da 10 secondi saltato senza sommare lo skew dell'orologio alla finestra di presenza. Un heartbeat con timestamp strettamente avanzato sullo stesso lease può ottenere al massimo 5 secondi di tolleranza per clock/jitter quando l'età è entro 10 secondi; un retained replay uguale o un lease già scaduto non riceve mai questa tolleranza. Da 1.5.2, quando la bilancia è già in fallback con un baseline owner noto, il recupero non dipende più dal confronto tra epoch tablet ed ESP32. Il primo owner di una nuova connessione/lease viene conservato come candidato inattivo; soltanto un secondo heartbeat con timestamp strettamente avanzato sulla stessa generazione prova un publisher vivo e abilita la presenza monotona. Un retained isolato o un replay uguale non può quindi riaprire il controllo. Il percorso non si applica agli snapshot retained iniziali senza baseline, che restano soggetti al TTL wall-clock. Da 1.4.4 il loop elabora fino a quattro pacchetti MQTT già disponibili prima di valutare il fallback locale: il burst bounded copre il normale owner/command accodato senza affamare il campionamento di tasti e bilancia. Dopo ogni reconnect attende inoltre il nuovo snapshot retained `command`: il solo owner non può riattivare per un loop un vecchio comando RAM prima dell'arrivo di un eventuale clear o ingrediente nuovo. Se l'owner sparisce o gli snapshot necessari non arrivano, il comando moderno resta raw in RAM ma non è operativo né visibile. Lo sgancio abilita una modalità locale: TARE e ENTER fisici restano utilizzabili anche con MQTT connesso, senza generare una seconda response remota. Un retained `weigh` della connessione sganciata non può riattivarla: serve un heartbeat più nuovo o una nuova connessione. Solo il relativo `command_ack` riaggancia la modalità gestita. I comandi privi di `connection_id` conservano il comportamento legacy.

Da firmware 1.5.0 la bilancia non si fida più del solo risultato locale di `subscribe()`, perché PubSubClient 2.8 non espone il SUBACK. Il controllo è passivo e non pubblica probe sui topic business: durante un contesto remoto usa owner heartbeat validi, traffico retained `command` e ACK terminali come prova delle singole subscription. Se un ingresso realmente atteso tace, esegue al massimo tre subscribe distanziate di 2,5 secondi e poi ricostruisce una volta il trasporto, conservando byte-per-byte l'outbox. Da 1.5.1 la prova del topic è separata dalla convergenza business: un `command` ricevuto ma diverso da `expected_command_id` prova che la subscription funziona, mantiene il comando non azionabile e attende la riconciliazione senza risottoscrivere o riavviare il socket. La finestra per un comando realmente assente parte dal cambio effettivo dell'aspettativa e non viene prorogata dai successivi heartbeat con lo stesso valore. Un `weigh` moderno strutturalmente valido che precede l'owner esatto apre analogamente un'attesa owner deduplicata per `command_id + connection_id`: un PUBLISH owner successivo prova il trasporto anche se è malformed o appartiene ancora al proprietario precedente, mentre la vera assenza avvia il repair bounded. La stessa fingerprint può ricostruire il socket una sola volta; se anche il nuovo trasporto resta senza evidenza, la bilancia rimane in `LOC` senza loop di reconnect. L'evidenza non attraversa il cambio socket e il timer non viene prorogato dai retry o da nuovi ID arrivati mentre una grace è già aperta. Quando non c'è una response o `confirm_request` in corso, `expected_command_id=""` è una dichiarazione autorevole di clear e rimuove anche un comando raw RAM se il broker non possiede alcun retained da riprodurre; un `weigh` successivo resta ignorato finché l'owner continua ad attendere clear. Durante una response il campo è omesso e outbox/comando restano immutabili; l'assenza mantiene inoltre la compatibilità con Manager precedenti. L'assenza di traffico a bilancia idle senza owner non provoca reconnect. Per uno stesso outbox l'assenza ACK causa un solo rebuild; se il backend continua a non rispondere, il retry prosegue senza loop di reconnect e status espone `ack_timeout`.

Pulizia comando:

```json
{"type":"clear","session_id":"...","connection_id":"...","command_id":"..."}
{"type":"clear","session_id":"...","connection_id":"...","command_id":"...","lifecycle":"pagehide"}
{"type":"clear","session_id":"...","connection_id":"...","lifecycle":"pagehide"}
```

Senza response pending la bilancia accetta un `clear` live soltanto se la fence è coerente: un comando moderno richiede lo stesso `session_id`, lo stesso `command_id` e un `connection_id` non vuoto. Se esiste un owner fresco, la connessione deve essere la sua; in assenza di owner fresco, il retained clear esatto è trattato come tombstone autorevole del broker. Inoltre, il primo snapshot `command` dopo reconnect è autorevole sul raw RAM: un retained clear moderno lo rimuove anche se nel frattempo il broker è avanzato a un'altra coppia sessione/comando. Se il relativo weigh era stato respinto prima di entrare nel raw RAM per owner mancante, il clear con la stessa fingerprint `command_id + connection_id` chiude comunque l'attesa owner diagnostica senza toccare un eventuale outbox. Il nuovo documento proprietario può così pulire il comando del predecessore, mentre un `pagehide` tardivo della vecchia connessione non può cancellare quello nuovo finché il nuovo owner è vivo. Da 1.4.2 il clear `lifecycle=pagehide`, autenticato dall'esatta sessione/connessione owner, può sganciare subito l'operatore anche senza comando o con una response pending; cancella il raw soltanto quando coincide anche la fence comando e non modifica mai l'outbox. I clear normali REST-before-ACK restano ignorati durante il pending. Le regole v1.2/legacy restano invariate.

Conferma transitoria di attivazione firmware:

```json
{"type":"command_ack","command_id":"...","session_id":"...","connection_id":"...","uuid":"...","product_id":123,"state":"active"}
```

Il firmware pubblica `command_ack` sul topic `response` soltanto dopo avere attivato un `weigh` con `command_id` e quando lo stesso comando è realmente azionabile: non emette `state=active` durante fallback `LOC`, TARE obbligatoria o prima della convergenza owner/command. Lo ripubblica per ogni replay identico dello stesso ID una volta ristabilito lo stato azionabile. Da firmware 1.4 riecheggia anche la connessione corrente, così un documento ricaricato non diventa pronto prima che il firmware abbia visto il suo nuovo `connection_id`. Da 1.4.2 ogni nuova eco owner esatta invalida nel Manager la precedente prova di attivazione e forza il replay dello stesso `weigh`: la pagina torna pronta solo sul nuovo ACK, quindi un fallback firmware non può restare nascosto dietro un ACK in cache. Questo ACK è QoS 0, non usa l'outbox, non contiene `response_id` e non rappresenta una mutazione Laravel. I comandi precedenti senza `command_id` non producono il nuovo ACK.

Conferma remota dal Manager:

```json
{"type":"confirm_request","request_id":"...","session_id":"...","connection_id":"...","command_id":"...","uuid":"...","product_id":123}
{"type":"confirm_request_ack","request_id":"...","command_id":"...","connection_id":"...","state":"accepted|staged|failed|rejected","reason":"..."}
```

`confirm_request` è QoS 1 ma **non retained**. È accettata soltanto dalla connessione corrente e per l'esatta identità del `weigh`; il callback MQTT la accoda e il loop esegue lo stesso percorso di ENTER fisico: preflight, lock TARE, eventuale acquisizione da 1,5 s, zero di lavoro, push reversibile e staging del `confirm`. `request_id` rende idempotenti doppio click e retry: lo stesso ID riemette l'ultimo ACK senza ripetere ENTER, un ID diverso durante acquisizione/outbox riceve `rejected/busy`. Una cache RAM conserva gli ultimi due esiti terminali `failed`/`rejected`: finché l'ID resta in uno dei due slot, la perdita del relativo ACK QoS 0 non può trasformarlo più tardi in un nuovo ENTER. `accepted` e `staged` sono solo stati transitori e non fanno avanzare il Manager; `failed`/`rejected` lasciano attivo lo stesso ingrediente. L'unico successo funzionale resta la response durable `confirm` completata da Laravel, clear e `response_ack`.

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
{"type":"response_reject","response_id":"a1b2c3d40000012300000001","reason":"manager_incident"}
```

Il firmware conserva una sola response session-aware in RAM e la ripubblica ogni **1 secondo** finché riceve un esito terminale con la stessa `response_id`. `response_ack` significa mutazione applicata; `response_reject` è accettato esclusivamente con `reason=manager_incident`, elimina l'outbox senza simulare un successo e resta idempotente sui replay. Il payload staged, inclusi `session_id`, l'eventuale `command_id` e `response_id`, resta byte-per-byte immutabile. Il lock fisico di consegna è limitato a **10 secondi**, termina prima su `pagehide` fenced/owner scaduto e viene sganciato immediatamente se l'operatore preme fisicamente TARE o ENTER. L'outbox continua in background, mentre quell'input e i successivi TARE/ENTER diventano esclusivamente locali. SKIP/CLEAR remoto, nuovi `weigh` e nuove response restano serializzati sul singolo outbox. Un `confirm` quarantinato viene ripristinato localmente una sola volta soltanto se la sua receipt è ancora l'ultima voce e il lock operatore non era stato sganciato; altrimenti stack e riferimento vengono invalidati, senza creare successi, undo o ACK per una receipt inesistente. Un `undo` rifiutato prima del detach lascia lo stack invariato. Se il restore detached era già stato applicato, un reject tardivo non tenta una compensazione pericolosa: invalida stack/riferimento locali, mostra `SYNC ERROR — Premi TARA`, blocca ENTER/SKIP/CLEAR e richiede una TARE fisica. Fino alla TARE riuscita lo status espone `tare_required=true`, la modalità resta locale e un eventuale comando raw viene conservato ma non è azionabile né riceve `command_ack`; dopo la TARE lo stesso contesto valido può essere riattivato e confermato una sola volta.

Outbox e cache dei due esiti terminali sono RAM: un reboot le perde, e più di due request terminali diversi possono espellere il più vecchio. Il Manager normale mantiene una sola richiesta irrisolta per scheda; una garanzia attraverso reboot o publisher concorrenti richiederebbe un journal persistente dedicato.

Per firmware/browser v1.4 l'ordine è transazionale:

1. il browser invia a Laravel la response completa, inclusi `scale_id`, `response_id`, `product_id`, UUID e azione;
2. Laravel registra una receipt durabile e applica la mutazione nella stessa transazione; il replay identico è idempotente;
3. soltanto dopo commit o replay già processato il browser pubblica il `clear` retained e attende il PUBACK QoS 1;
4. soltanto dopo il PUBACK del clear pubblica `response_ack` e ne attende il PUBACK QoS 1;
5. infine aggiorna la UI. Se la mutazione non può essere applicata ma Laravel registra durablemente l'incidente, il browser pubblica prima il clear e, dopo il relativo PUBACK, invia `response_reject` con `reason=manager_incident`; senza commit dell'incidente non invia alcun esito terminale.

Il PUBACK broker di `response_ack` non prova che il firmware fosse ancora online e l'abbia elaborato. Uno status `offline` o `sleeping` invalida quindi una consegna non conclusa; se il firmware ripubblica una response già persistita, il browser ripete clear e ACK senza ripetere REST o callback UI. Una response dello stesso prodotto della pagina, incluso `undo`, forza la riconciliazione anche se l'ingrediente desiderato è già avanzato o assente; una response di un altro prodotto viene conciliata senza cancellare il comando corrente.

La coppia (`scale_id`, `response_id`) resta l'identità della receipt. La `session_id` e l'eventuale `command_id` del payload staged descrivono l'origine e non vengono ritargettati. Solo un `response_ack` esatto o un `response_reject/manager_incident` esatto chiude l'outbox e azzera il comando; esiti estranei, nuovi comandi, clear e scadenza owner non possono alterarlo. Se il browser desidera attivare un comando arrivato mentre l'outbox era pending, lo ripubblica dopo avere completato receipt, clear retained ed esito terminale.

I browser precedenti restano supportati. Un comando v1.2 privo di `command_id` ma con sessione può essere cancellato solo da un clear anch'esso privo di ID con la stessa `session_id`; per un comando realmente legacy senza sessione il clear può riportare la sessione del Manager nuovo. Un comando privo di `session_id` usa response one-shot senza `response_id`, senza receipt/ACK applicativo e senza undo remoto.

### Ownership browser

Ogni scheda browser usa un `session_id` stabile durante reload e reconnect, un `connection_id` diverso per ogni documento e un nuovo `command_id` per ogni attivazione logica, conservato nei retry dello stesso comando. Un Web Lock esclusivo impedisce a una scheda duplicata di riusare l'ID copiato da `sessionStorage`; il fallback cross-tab ruota l'ID anche quando la presenza di un'altra istanza resta incerta. Il topic owner retained contiene anche `connection_id`, `lease_id` ed `expected_command_id` quando non c'è una response in corso: il valore è l'ID desiderato oppure `""` per dichiarare che il firmware non deve avere un comando attivo. Il controllo è confermato dall'eco esatta di utente, sessione, connessione e lease, mentre lo stato comando deve coincidere con l'hint. Il proprietario rinnova il lease ogni **10 secondi**; dopo **30 secondi** senza rinnovo un'altra scheda può reclamarlo. Il firmware considera presente il consumer per **25 secondi dall'arrivo** dell'ultimo owner valido, tollerando un heartbeat saltato. Una tolleranza massima di 5 secondi sul TTL è concessa soltanto a timestamp strettamente progressivi dello stesso lease con skew entro 10 secondi; replay uguali e lease scaduti restano esclusi. Il timestamp monotono è verificato soltanto all'interno dello stesso `connection_id + lease_id`. In fallback 1.5.2, una nuova generazione diventa prima un candidato inattivo e viene attivata solo dal secondo heartbeat strettamente progressivo sulla stessa lease; il Manager produce entrambe le prove nella recovery fenced. Il recupero non dipende dall'epoch ESP32 e un retained isolato resta innocuo.

```json
{"user_id":123,"user_name":"Andrea","session_id":"...","connection_id":"...","lease_id":"...","expected_command_id":"...","timestamp":1784720000}
```

**Alla connessione:**
- la bilancia pubblica status `online` retained con scale ID, nome e firmware version;
- registra un LWT `offline` retained;
- si sottoscrive prima a `owner`, poi a `command` e `ack`, richiedendo QoS 1;
- il browser attende i SUBACK di response/status/owner/command, uno status fresco e la conferma esatta del lease prima di pubblicare o ripubblicare un comando attivo; se un SUBACK manca/fallisce o lo status retained fresco non arriva, il connection barrier resta non-ready e ritenta per intero la coorte con callback fenced;
- il browser non registra un LWT che modifichi `command`: con firmware 1.4.2+, su una vera uscita `pagehide`, l'owner esatto pubblica un solo `clear` retained con `lifecycle=pagehide` e poi chiude il socket in modo graceful, anche senza comando o con una response in corso. I passaggi interni tra ingredienti/modalità, i form che tornano alla pesatura e il reload dopo una response marcano invece un handoff: chiudono il vecchio socket senza detach, così il nuovo documento può sostituire owner e comando senza attivare il fallback sticky. Il publish di uscita resta best-effort; se il processo viene terminato prima della consegna, il lock operatore di una response cede al primo TARE/ENTER o entro 10 secondi, mentre la presenza operativa owner decade entro 25 secondi;
- la UI è verde soltanto dopo l'eco retained del comando e il relativo `command_ack` firmware; durante reconnect o riparazione resta in sincronizzazione.
- quando il documento torna visibile, invalida subito la prova di attivazione precedente, sospende l'heartbeat e rilegge l'owner retained; rinnova lo stesso lease soltanto se è ancora esatto, quindi non sovrascrive un takeover avvenuto mentre la scheda era sospesa. La relativa eco fa ripubblicare lo stesso `weigh` e richiede un ACK nuovo.

Ordine di rollout 1.5 raccomandato: aggiornare prima il firmware, poi Manager/backend che pubblicano `expected_command_id`, gestiscono lo status diagnostico e possono usare `response_reject`. Così un terminale reject non incontra mai un firmware precedente che lo ignorerebbe lasciando l'outbox aperto. Non è richiesto alcun cambio ACL: la bilancia continua a pubblicare soltanto `status` e `response`, senza probe su `owner`, `command` o `ack`.

Cambio e disassociazione bilancia sono bloccati finché una `confirm_request` è attiva o una response è in sincronizzazione. Una request staged passata al timeout browser non mantiene il lock e non può essere rieseguita. Se la response inizia mentre il salvataggio del cambio è già in corso, la transizione MQTT attende in modo non bloccante la fine della consegna. Clear e rilascio owner sulla vecchia bilancia vengono pubblicati soltanto dalla connessione che possiede il lease esatto.

**Comando `weigh`:** il browser invia UUID ingrediente, `product_id`, nome e peso target. La bilancia emette un bip distintivo di ricezione, azzera il timer inattività e mostra il target sul display (icona target + grammi).

**Comando `confirm_request`:** il pulsante Conferma del Manager, con bilancia associata, chiede alla bilancia di eseguire ENTER. Non sostituisce il retained `weigh`, non chiama direttamente la conferma Laravel e viene ritentato con lo stesso `request_id` finché il firmware comunica `staged`, `failed` o `rejected`. Dopo reconnect il Manager riattiva prima lo stesso `weigh` e attende un `command_ack` fresco. Se dopo `staged` cade il trasporto o viene revocato l'owner esatto e la response non torna entro 10 secondi, sblocca il percorso manuale senza ripetere ENTER e continua a conciliare un eventuale `confirm` tardivo.

**Conferma manuale:** il pulsante **Pesata già fatta** marca l'ingrediente eseguito senza inventare un peso o una provenienza bilancia. Resta l'unica conferma quando la bilancia non è pronta; nello stato verde `ready` viene mostrato insieme a **Conferma bilancia**. Durante una `confirm_request` attiva o response in corso entrambe le azioni restano bloccate; il timeout browser di una request già staged riabilita soltanto il manuale.

**Comando `clear`:** per i comandi moderni annulla soltanto la stessa coppia `session_id + command_id`; con owner fresco richiede la sua connessione, senza owner accetta il retained tombstone con connessione non vuota. Le eccezioni compatibili per v1.2/legacy sono quelle descritte sopra. Non modifica lo stack locale.

### Tasti (con MQTT attivo)

- **ENTER**: con un owner vivo usa soltanto il comando `weigh` esatto; senza comando emette un warning. Quando non esiste più un consumer remoto, un outbox supera 10 secondi senza ACK oppure l'operatore preme ENTER mentre quell'outbox è ancora agganciato, passa immediatamente alla modalità locale anche con MQTT connesso: applica tara/push locali senza creare una seconda response. Una cattura già iniziata viene annullata se cambia contesto
- **Conferma dal Manager**: con firmware 1.4+ richiama lo stesso flusso di ENTER. La pagina non cambia ingrediente su `accepted`/`staged` né su errore; avanza soltanto dopo la persistenza durable della pesata e l'ACK end-to-end
- **SKIP breve**: scatta al rilascio e prepara `skip` per il comando attivo; senza comando emette un buzzer di avviso
- **SKIP tenuto 5 secondi**: apre il wizard calibrazione senza inviare prima uno `skip`
- **CLEAR breve**: annulla subito una voce locale; per una voce con receipt session-aware prepara `undo` e ripristina offset/zero-tracking soltanto al relativo ACK
- Finché il lock remoto è attaccato SKIP/CLEAR vengono rifiutati. Una pressione fisica di TARE/ENTER forza invece il fallback e prosegue localmente; non crea una seconda response

### Display MQTT

- **Icona MQTT** (frecce ↑↓) nella barra di stato: visibile quando il WiFi è connesso
  - Fissa: connesso al broker
  - Lampeggiante: disconnesso dal broker; il fallback locale non altera più questa icona
- **Badge modalità** compatto nella stessa barra:
  - `APP`: owner Manager fresco e modalità remota ancora agganciata
  - `LOC`: nessun Manager operativo, fallback locale oppure trasporto non disponibile
  - Il badge resta fisso: il lampeggio è riservato alla riconnessione MQTT
- **Riga info**: valori only, senza label (`WORK | STABLE` / `WORK | UNSTABLE`); in modalità LIVE mostra solo `LIVE`
- **Ingrediente**: quando arriva un comando `weigh`, il nome ingrediente è mostrato sulla riga sotto (max 15 caratteri; oltre: 15 + tre puntini ravvicinati)
- **Peso target**: quando è attivo un comando `weigh`, il peso obiettivo viene mostrato accanto all'icona target

### Disconnessione e riconnessione

- Alla disconnessione dal broker: **doppio beep** buzzer + icona MQTT lampeggiante
- Riconnessione automatica con **backoff esponenziale** (2s → 4s → 8s → 16s → 30s max)
- DNS, connessione TCP, handshake TLS, MQTT CONNECT e SUBSCRIBE sono eseguiti da un worker FreeRTOS separato con proprietà esclusiva del trasporto. Il loop principale continua quindi a campionare tastiera, HX711 e UI anche se broker o DNS non rispondono
- Il tentativo usa timeout TCP **2,5s**, handshake TLS **10s**, socket MQTT **1s** e keepalive **15s**; dopo CONNECT il timeout I/O TLS scende a **500ms** e il client passa al loop principale con un handoff atomico
- Il worker non è registrato nel WDT del loop. Sleep normale e reload credenziali ne attendono l'handoff o richiedono un abort cooperativo, senza accessi concorrenti al client
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
2. Aggiungi ingrediente e premi **ENTER** sulla bilancia oppure **Conferma** nel Manager → se è già quieto accetta subito; altrimenti mostra la barra **ACQUISIZIONE PESO** fino a 1,5 s, poi accetta appena STABLE o usa il centro robusto se non c'è deriva. A quel punto applica lo zero di lavoro, registra nello stack e prepara il `confirm` MQTT
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
- ENTER con HX non OK o snapshot vecchio è ignorato. Un peso non quieto avvia invece l'acquisizione da 1,5 s; se continua a muoversi o i campioni non sono validi, il doppio beep e l'avviso auto-dismiss confermano che non è stato registrato
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
- La richiesta remota del Manager non duplica questa logica: viene consumata nel loop, rispetta lo stesso lock TARE e chiama lo stesso helper di ENTER. Un fallimento non genera `confirm`, quindi Laravel e la pagina restano sull'ingrediente corrente.
- CLEAR breve legge la voce senza rimuoverla. Se contiene una receipt remota, prepara un nuovo outbox `undo` con `undo_of_response_id`; offset/zero-tracking vengono ripristinati e la voce viene rimossa soltanto dopo l'ACK browser, quindi dopo la persistenza Laravel. Se il primo publish QoS 0 fallisce, lo stato locale resta invariato e l'outbox continua i retry.
- Una voce locale o legacy senza receipt viene annullata subito e solo localmente; non viene inventato un undo Laravel non correlabile.
- Una response in attesa continua a serializzare SKIP, CLEAR e le mutazioni del wizard. TARE ed ENTER non vengono mai scartati: la prima pressione fisica forza lo sgancio locale immediato, mentre senza input il fail-safe scatta comunque entro 10 secondi. Il payload remoto continua il retry immutabile e le mutazioni seriali/calibrazione restano serializzate sul singolo outbox.
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
Il firmware include un **Task Watchdog** (8 secondi) attivo già durante il setup, con reset esplicito nei loop di boot intenzionali. Nel loop principale viene resettato a ogni iterazione. Il worker di connessione MQTT è separato e non è registrato nel WDT del loop, quindi DNS/TCP/TLS non richiedono di alterarne il timeout.

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
  - la scelta ON/OFF viene salvata nella NVS `minu_audio` e ripristinata dopo light-sleep, reboot e interruzioni di alimentazione; sui dispositivi senza preferenza salvata il valore predefinito resta ON.
- Il comando seriale `mp3 reset` forza lo stesso hard reset per debug da banco.
- Viene spento **quando la bilancia entra in standby/light-sleep per inattività** e anche **prima del light-sleep per batteria scarica**.

Diagnostica audio:
- un ring buffer RAM conserva gli ultimi **24 eventi** (`request`, `play`, assenza transizione BUSY, timeout, reset, ready e toggle manuali); sopravvive al light-sleep ma non a un'interruzione di alimentazione;
- al timeout e quando l'operatore passa manualmente ad AUDIO OFF viene salvato in NVS un solo snapshot compatto con stato, traccia, BUSY e comando GPIO2; non vengono eseguite scritture flash per ogni riproduzione;
- `mp3 history` stampa snapshot persistente e cronologia RAM; `mp3 history clear` cancella entrambi senza modificare la preferenza AUDIO ON/OFF;
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
