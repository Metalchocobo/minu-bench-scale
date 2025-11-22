# Bilancia ESP32 + HX711 — 20 kg / 1 g — Serial Only (v1.0)

Lettura affidabile di una cella di carico con **risoluzione al grammo** su **portata 20 kg** usando un **HX711** economico, con logica di stabilità, filtro di visualizzazione e gestione della deriva nel tempo. Uscita su **Serial Monitor**. L’UI su OLED (SSD1322 3.12″) si aggiungerà in seguito senza cambiare la logica.

---

## NOVITÀ 2025-11-22 — Alimentazione e cablaggio (deciso e realizzato)

**Architettura elettrica attuale**  
- Batteria **piombo 6 V** (SLA/AGM).  
- **CTK3S (PWM)** usato per **carica** e gestione **LOAD**.  
- **Buck step-down** regolato a **5,05–5,10 V** che alimenta l’ESP32.  
- Il buck è alimentato dalla **porta LOAD** del CTK3S così da sfruttare il **Low-Voltage Disconnect (LVD)** interno.  
  - LVD del CTK3S agisce **solo** su LOAD. Se si preleva da **BAT** l’LVD **non** protegge.  
  - Soglie di stacco tipiche per 6 V: **~5,3–5,4 V** con riattacco **~5,6–5,9 V** (verificare sul nostro esemplare).

**Condensatori e disposizione fisica**  
- **Ingresso buck (da LOAD CTK3S):** 220 µF elettrolitico (≥16 V) **+** 100 nF ceramico, **in parallelo**, piazzati **vicinissimi** ai morsetti **IN**.  
- **Uscita buck (rail 5 V):** 470 µF elettrolitico (≥10–16 V) **+** 100 nF ceramico, **in parallelo**, **vicino** ai morsetti **OUT**.  
- **Vicino ai pin dell’ESP32 (VIN/5V ↔ GND):** 10–47 µF **+** 100 nF, a **pochissimi mm** dal pin (decoupling locale per picchi Wi‑Fi).

**Fusibili consigliati**  
- **Tra alimentatore ↔ CTK3S (PV+):** 2 A rapido o medio.  
- **Tra BAT+ ↔ LOAD+ (verso impianto):** 2 A rapido.  
- **Sul 5 V dopo il buck (opzionale):** 1 A rapido per protezione locale.

**Settaggi e note operative**  
- Regola il buck a **5,05–5,10 V** sotto carico leggero.  
- Durante la carica la batteria può stare a **7,2–7,5 V** (tensione di carica), è normale.  
- LVD esterno **non necessario** se il buck resta su **LOAD**; se un domani prelevi da **BAT**, allora serve un LVD in serie al buck.

**Dupont / Millefori vicino a ESP**  
- Mini **millefori “interposer”** a 1–2 cm dal pin VIN/GND dell’ESP con i due cap locali (10–47 µF + 100 nF).  
- Alternativa rapida: ceramico 100 nF **direttamente** tra VIN↔GND sull’ESP e bulk 10–47 µF su pigtail corto.

---

## Concetti chiave

- **Counts (raw)**: il HX711 restituisce un numero “raw” proporzionale alla forza sulla cella.  
- **OFFSET_RAW**: baseline a vuoto (in counts). È la tara “elettronica”.  
- **SCALE**: fattore di conversione dai counts ai grammi. Si ricava con un peso noto:  
  `SCALE = (raw_con_peso - OFFSET_RAW) / peso_grammi`.  
- **Drift naturale**: con il tempo e la temperatura il raw può scivolare.  
- **Zero-tracking (ZT)**: quando siamo **vicini a zero** e **stabili**, **integriamo lentamente** un contatore di deriva (**drift_bucket_counts**) senza far muovere l’indicazione. Questo tiene memoria della deriva accumulata.  
- **Spendi il drift quando il peso cambia**: appena entriamo in **UNSTABLE**, **sommiamo** il bucket all’OFFSET_RAW. Così la **prossima** misura torna allineata allo zero reale.  
- **STABLE/UNSTABLE**: lo stato dipende dalle **finestre temporali**:
  - fast (**0.5 s**): se la variazione supera `THRESH_UNSTABLE` entri in **UNSTABLE**  
  - slow (**2.0 s**): se la variazione resta sotto `THRESH_STABLE` torni **STABLE**  
- **DISPLAY_DEAD_BAND**: non aggiorno il numero mostrato finché la modifica è entro **±0.3 g** (meno tremolii).

---

## Hardware

- **ESP32 DevKit**  
- **HX711** breakout (alimentato a **3.3 V**)  
- **Cella di carico** 4 fili:  
  Rosso→E+, Nero→E−, Verde→A+, Bianco→A− (sul modulo HX711)  
- Collegamenti HX711→ESP32:  
  `DOUT→GPIO25`, `SCK→GPIO5`, `VCC→3V3`, `GND→GND`

> Nota: alimentando a 3.3 V si evitano problemi di livelli logici.  
> Per stabilità: cavo cella **twistato e schermato**, calza a **GND lato scheda**.

---

## Alimentazione — schema logico di riferimento

- **Batteria 6 V** ↔ **CTK3S**  
  - Alimentatore 12 V → CTK3S (ingresso PV) per la carica  
  - **Buck 5 V** collegato alla **porta LOAD** del CTK3S  
- **Condensatori**:  
  - IN buck: **220 µF + 100 nF**  
  - OUT buck: **470 µF + 100 nF**  
  - **ESP32 VIN**: **10–47 µF + 100 nF**  
- **Fusibili**: 2 A su PV+, 2 A su linea BAT→LOAD, 1 A opzionale su 5 V post-buck

---

## Configurazione (tutto all’inizio dello sketch)

- `AVG_N` — numero di letture che mediamo per ogni campione (1..8).  
  Più alto → meno rumore ma risposta più lenta.  
- `SAMPLE_MS` — intervallo di campionamento (ms).  
  70 ms ≈ 14 Hz complessivi. Puoi scendere a 50–60 ms.  
- `THRESH_UNSTABLE` / `THRESH_STABLE` — soglie in grammi per determinare lo stato.  
- `WIN_FAST_MS` / `WIN_SLOW_MS` — finestre per le soglie (0.5 s e 2 s).  
- `DISPLAY_DEAD_BAND` — dead-band del “display” (seriale) in grammi.  
- `ZT_*` — parametri zero-tracking (attivo, banda vicino allo zero, passo minimo).  
- `AUTO_TARE_ON_BOOT` — se vero, una **tara** dopo piccola attesa a boot.

---

## Procedura di avvio e calibrazione

1) Avvia con la cella **scarica**. Se `AUTO_TARE_ON_BOOT=true`, lo sketch fa una tara iniziale.  
2) Apri Serial Monitor a **115200**. Digita `cfg?` per vedere i parametri correnti.  
3) **Calibrazione** con peso noto (es. **2 kg**):  
   - Appoggia il peso da 2 kg  
   - Digita `cal=2000` e **premi Invio**  
   - Attendi il messaggio `[CAL OK]`, poi `cfg?` per verificare `SCALE`  
4) Togli e rimetti il peso: ~0 g a vuoto e ~2000 g con il peso.  
5) Se “trema”, aumenta `AVG_N` a 4–6 o `DISPLAY_DEAD_BAND` a 0.4–0.5 g.

---

## Comandi seriali

- `t` — **Tare** immediata (imposta `OFFSET_RAW` ai counts correnti e azzera il drift bucket).  
- `cal=XXXX` — Calibra la **SCALE** usando un peso noto in grammi (es. `cal=2000`).  
- `cfg?` — Mostra config e valori attuali (OFFSET, SCALE, filtri, ecc.).  
- `zt=on` / `zt=off` — Abilita o disabilita lo **zero-tracking**.  
- `dbg=on` / `dbg=off` — Telemetria estesa ad ogni campione: `rng0.5s`, `drift`, ecc.  
- `help` — Elenco comandi.

---

## Come funziona “drift bucket” (perché è utile)

- A zero e in **STABLE** il raw può scivolare lentamente di decine/centinaia di counts nel tempo.  
- Con lo **zero-tracking** **non** muovo il numero esposto (evito ±1/±2 g fastidiosi), ma **accumulo** quella deriva in `drift_bucket_counts`.  
- Quando il peso cambia davvero (stato → **UNSTABLE**), **spendo** in un colpo solo il bucket aggiornando `OFFSET_RAW`.  
  Risultato: quando scarichi, torni al **vero zero** senza trascinarti l’errore.

---

## Tuning pratico

- **Piccoli ingredienti** (reattività): `SAMPLE_MS` 50–60 ms, `AVG_N=3..4`, `DISPLAY_DEAD_BAND=0.2–0.3`.  
- **Stabilità banco**: `AVG_N=4–6`, soglie `THRESH_*` a 1.0 g, dead-band 0.3–0.5.  
- **Rumore anomalo**: cablaggi, massa a stella, alimentazione pulita, cavo cella schermato.

---

## Collaudo rapido alimentazione

1) **Regola buck** a **5,08 V** con un carico leggero (precarico 330 Ω se serve).  
2) Alimenta **buck da LOAD** e **ESP dal buck**.  
3) **Verifica LVD**: lascia scaricare finché **LOAD** si spegne. Annota la tensione batteria di **stacco** e di **riattacco**. Target ~**5,3–5,4 V** / **≥5,6 V**.

---

## Risoluzione problemi

- **Reset casuali ESP**: verifica i condensatori **vicino a VIN/5V** dell’ESP, cablaggi corti, buck a 5,05–5,10 V.  
- **Rumore/oscillazioni 5 V**: controlla 470 µF + 100 nF su **OUT** buck e 220 µF + 100 nF su **IN** buck.  
- **Scarica profonda**: tieni il buck su **LOAD** (protetto da LVD) oppure usa un **LVD esterno** in serie al buck.

---

## Prossimi passi (quando arriva l’OLED 3.12″ SSD1322)

- Aggiungere **U8g2** (driver SSD1322) e una UI con: peso in grande, ● STABLE/UNSTABLE, NET/TARE, batteria da INA219.  
- Aggiornare solo quando cambiano le cifre (zero flicker).  
- **Anti burn-in**: piccolo pixel-shift ogni 30–60 s o dimmer dopo inattività.

---

## Note tecniche e limiti

- Il HX711 ha limiti intrinseci di rumore/deriva: questa logica li **mitiga** (non li elimina).  
- In futuro, il **NAU7802** può offrire **miglior SNR** e opzioni di filtraggio interno. La logica di stato e drift resta riusabile identica.  
- La **linearità** su tutta la scala con celle economiche non è perfetta: per lavorare “pro” si passa a **calibrazione multi-punto** (minimo 0, 1/3, 2/3, FS) con correzione lineare a tratti.
