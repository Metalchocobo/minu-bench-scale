# Minù Bench Scale – Specifiche di Integrazione con Gestionale

**Versione:** 1.0  
**Data:** 2026-02-10  
**Stato:** Bozza per revisione

---

## 1. Panoramica

La bilancia Minù Bench Scale (ESP32) si integra con il gestionale web Laravel come periferica del browser. La comunicazione avviene tramite un broker MQTT (Mosquitto) installato sul VPS, che funge da relay trasparente tra browser e bilancia.

Laravel non partecipa alla comunicazione real-time. Il suo ruolo è limitato a:

- Servire l'applicazione web
- Fornire dati (ricette, ingredienti, pesi target)
- Persistere i risultati delle pesate via REST API
- Fornire la chiave di autenticazione MQTT alla sessione utente

## 2. Architettura

```
┌──────────────────────── LAN Laboratorio ────────────────────────┐
│                                                                  │
│   ┌─────────┐                                  ┌─────────────┐  │
│   │ Tablet  │                                  │  ESP32       │  │
│   │ Browser │                                  │  Bilancia    │  │
│   └────┬────┘                                  └──────┬──────┘  │
│        │                                              │          │
└────────┼──────────────────────────────────────────────┼──────────┘
         │ wss:// (WebSocket TLS)                       │ mqtts:// (MQTT TLS)
         │                                              │
    ┌────┴──────────────────────────────────────────────┴────┐
    │                    VPS (Digital Ocean)                   │
    │                                                         │
    │   ┌─────────────┐         ┌─────────────────────────┐  │
    │   │  Mosquitto  │         │  Laravel                 │  │
    │   │  MQTT Broker│         │  REST API + Web App      │  │
    │   │  :8883 MQTT │         │                          │  │
    │   │  :8884 WSS  │         │                          │  │
    │   └─────────────┘         └─────────────────────────┘  │
    │                                                         │
    │   Nessuna comunicazione diretta tra Mosquitto e Laravel │
    │   per il flusso di pesatura                             │
    └─────────────────────────────────────────────────────────┘
```

## 3. Identificazione bilancia

Ogni bilancia è identificata dal MAC address WiFi dell'ESP32, disponibile senza configurazione aggiuntiva. Il MAC viene usato come `scale_id` nei topic MQTT.

Formato: minuscolo, senza separatori (es. `a0b1c2d3e4f5`).

L'ESP32 pubblica il proprio MAC nel topic di status al momento della connessione al broker.

## 4. Topic MQTT

Prefisso base: `minu/scale/{scale_id}/`

| Topic | Direzione | Publisher | Subscriber | Retain | QoS |
|---|---|---|---|---|---|
| `minu/scale/{scale_id}/command` | Browser → Bilancia | Browser | ESP32 | Sì | 1 |
| `minu/scale/{scale_id}/response` | Bilancia → Browser | ESP32 | Browser | No | 1 |
| `minu/scale/{scale_id}/status` | Bilancia → Tutti | ESP32 | Browser | Sì | 1 |
| `minu/scale/{scale_id}/owner` | Browser → Browser | Browser | Browser | Sì | 1 |

Note:

- `command` con retain: se la bilancia si disconnette e riconnette, riceve automaticamente l'ultimo comando attivo senza intervento del browser.
- `response` senza retain: le risposte sono eventi puntuali, non devono persistere.
- `status` con retain: qualsiasi browser che si collega vede subito lo stato corrente della bilancia.
- QoS 1 (at least once): garantisce la consegna anche su reti instabili. Il firmware e il browser devono gestire eventuali duplicati tramite UUID.

## 5. Formato messaggi

### 5.1 Command (Browser → Bilancia)

**Invio ingrediente da pesare:**

```json
{
  "type": "weigh",
  "uuid": "f47ac10b-58cc-4372-a567-0e02b2c3d479",
  "name": "Zucchero invertito",
  "target_weight": 450.0
}
```

- `uuid`: identificativo univoco dell'ingrediente nella sessione di pesatura corrente, generato dal gestionale.
- `name`: nome dell'ingrediente. Informativo, la bilancia può ignorarlo se lo spazio display è insufficiente.
- `target_weight`: peso target in grammi.

**Annullamento (nessun ingrediente attivo):**

```json
{
  "type": "clear"
}
```

Il browser invia `clear` quando l'operatore chiude la schermata di pesatura o deseleziona l'ingrediente. La bilancia torna in stato idle e cancella il peso target dal display.

Poiché il topic `command` è retained, inviare `clear` è necessario per evitare che la bilancia, riconnettendosi, riceva un comando obsoleto.

### 5.2 Response (Bilancia → Browser)

**Conferma pesata:**

```json
{
  "type": "confirm",
  "uuid": "f47ac10b-58cc-4372-a567-0e02b2c3d479",
  "actual_weight": 448.5
}
```

**Skip ingrediente:**

```json
{
  "type": "skip",
  "uuid": "f47ac10b-58cc-4372-a567-0e02b2c3d479"
}
```

L'UUID nella response permette al browser di verificare che la risposta si riferisca all'ingrediente corretto. Se il browser ha già inviato un nuovo comando (cambio ingrediente), e arriva una response con UUID diverso da quello attivo, il browser la ignora.

### 5.3 Status (Bilancia → Tutti)

```json
{
  "type": "status",
  "state": "online",
  "scale_id": "a0b1c2d3e4f5",
  "firmware_version": "1.2.0"
}
```

Valori di `state`: `online`, `error`.

Lo stato `offline` non viene pubblicato direttamente dall'ESP32. Viene gestito tramite LWT (sezione 6).

Il campo `firmware_version` è opzionale ma utile per diagnostica.

## 6. Last Will and Testament (LWT)

### 6.1 LWT della bilancia (ESP32)

All'atto della connessione MQTT, l'ESP32 registra il seguente messaggio LWT:

**Topic:** `minu/scale/{scale_id}/status`  
**Retain:** Sì  
**QoS:** 1  
**Payload:**

```json
{
  "type": "status",
  "state": "offline",
  "scale_id": "a0b1c2d3e4f5"
}
```

Se la bilancia perde la connessione, Mosquitto pubblica automaticamente questo messaggio. Il browser vede il cambio di stato senza polling.

Alla riconnessione, l'ESP32 pubblica il messaggio di status `online`, sovrascrivendo l'LWT retained.

### 6.2 LWT del browser

All'atto della connessione MQTT, anche il browser registra un LWT. Questo risolve i casi in cui il browser non riesce a pubblicare `clear` esplicitamente: chiusura tab, crash, kill dell'app, perdita di connessione.

**Topic:** `minu/scale/{scale_id}/command` (della bilancia associata)  
**Retain:** Sì  
**QoS:** 1  
**Payload:**

```json
{
  "type": "clear"
}
```

Se la connessione WebSocket del browser cade per qualsiasi motivo, Mosquitto pubblica `clear` sul topic command. La bilancia riceve il clear e torna in stato idle, cancellando il peso target dal display.

Nota: il LWT del browser va configurato con il `scale_id` della bilancia associata al momento della connessione. Se l'operatore cambia bilancia, il browser deve disconnettersi e riconnettersi con un nuovo LWT che punti alla nuova bilancia.

Nota: il LWT del browser copre solo il topic command. Per la pulizia del topic owner alla disconnessione imprevista, servono due LWT separati, ma MQTT consente un solo LWT per connessione. Soluzione: il topic owner include un `timestamp`. Gli altri browser, se vedono un owner con timestamp vecchio e la bilancia è offline (status), possono considerarlo decaduto e assumere il controllo.

## 7. Associazione bilancia-browser

### 7.1 Discovery

Il browser si sottoscrive a `minu/scale/+/status` (wildcard). Grazie ai messaggi retained, riceve immediatamente lo stato di tutte le bilance che si sono mai connesse al broker.

Il browser presenta la lista delle bilance con il rispettivo stato (online/offline).

### 7.2 Selezione

L'operatore seleziona una bilancia dalla lista. La scelta viene salvata lato server (associata all'utente o alla sessione) tramite chiamata REST a Laravel.

L'associazione persiste tra le sessioni: l'operatore non deve riselezionare la bilancia ogni volta che apre una pesatura.

### 7.3 Gestione proprietà (owner)

La bilancia può essere comandata da un solo browser per volta. La gestione avviene tramite un topic MQTT dedicato.

Quando un browser si associa a una bilancia, pubblica su `minu/scale/{scale_id}/owner`:

```json
{
  "user_id": 5,
  "timestamp": 1707600000
}
```

Retain: Sì, QoS: 1.

Ogni browser sottoscritto a questo topic vede immediatamente chi ha il controllo. Se un browser riceve un messaggio owner con `user_id` diverso dal proprio:

- Smette di inviare comandi alla bilancia
- Mostra un avviso all'operatore (es. "Bilancia in uso da altro operatore")
- Resta connesso al broker (nessuna disconnessione, nessun loop)

La bilancia non è coinvolta: continua a eseguire l'ultimo comando ricevuto, indipendentemente da chi lo ha inviato.

Quando un browser rilascia la bilancia (chiusura schermata, cambio bilancia, disconnessione), il topic owner viene pulito tramite pubblicazione di un messaggio vuoto retained oppure tramite LWT.

### 7.4 Dove si configura

L'associazione bilancia-utente è accessibile da:

- La schermata di pesatura (selezione rapida)
- Una voce nel menù utente (configurazione persistente)

## 8. Flusso operativo completo

1. L'operatore apre la schermata di pesatura nel browser
2. Il browser si connette al broker MQTT via WebSocket (se non già connesso)
3. Il browser si sottoscrive a `minu/scale/{scale_id}/response` e `minu/scale/{scale_id}/status` della bilancia associata
4. L'operatore seleziona un ingrediente da pesare
5. Il browser pubblica il comando `weigh` su `minu/scale/{scale_id}/command`
6. La bilancia riceve, mostra il peso target sul display OLED
7. L'operatore esegue la pesata
8. L'operatore preme conferma o skip sulla bilancia
9. La bilancia pubblica la response su `minu/scale/{scale_id}/response`
10. Il browser riceve la response e agisce come se l'operatore avesse operato direttamente sul browser (aggiorna UI, avanza al prossimo ingrediente, ecc.)
11. Il browser effettua POST REST a Laravel per persistere il risultato
12. Il browser invia il comando per il prossimo ingrediente, oppure `clear` se la pesatura è terminata

### 8.1 Cambio ingrediente senza conferma

Se l'operatore seleziona un altro ingrediente dal browser prima che la bilancia abbia confermato o skippato il precedente:

- Il browser pubblica un nuovo comando `weigh` con il nuovo UUID
- La bilancia sovrascrive il peso target sul display
- Se arriva una response con UUID diverso da quello attivo, il browser la ignora

### 8.2 Disconnessione WiFi della bilancia

- Mosquitto pubblica l'LWT (stato `offline`) sul topic status
- Il browser mostra un indicatore visivo di disconnessione
- L'operatore non è bloccato: può continuare a operare dal browser manualmente
- Alla riconnessione, la bilancia pubblica `online` sul topic status e riceve l'ultimo comando retained (se presente)

### 8.3 Pulizia comando (clear)

Il browser pubblica `clear` in modo esplicito nei seguenti casi:

- L'operatore chiude la schermata di pesatura
- L'operatore termina la ricettazione (tutti gli ingredienti pesati o uscita volontaria)
- L'operatore deseleziona l'ingrediente attivo senza selezionarne un altro

Nei seguenti casi, il `clear` viene pubblicato automaticamente da Mosquitto tramite LWT del browser (sezione 6.2):

- Chiusura del tab o del browser
- Crash del browser o dell'app
- Standby del tablet (il sistema operativo chiude la connessione WebSocket dopo un timeout)
- Perdita di connessione internet del tablet

In tutti i casi, la bilancia riceve `clear` e torna in stato idle.

### 8.4 Standby e risveglio del tablet

Quando il tablet va in standby:

1. Il sistema operativo chiude la connessione WebSocket
2. Mosquitto rileva il timeout e pubblica il LWT del browser (`clear` retained)
3. La bilancia torna in idle

Quando il tablet si risveglia:

1. La pagina di pesatura è ancora aperta nel browser
2. `mqtt.js` rileva la disconnessione e tenta la riconnessione automatica
3. Alla riconnessione riuscita, il browser verifica se c'è un ingrediente attivo nella UI
4. Se sì: ri-pubblica il comando `weigh` con lo stesso UUID e target, la bilancia riprende
5. Se no: non fa nulla, il `clear` del LWT ha già pulito il topic

Nessun polling. Tutto event-driven tramite gli eventi `reconnect` e `offline` di `mqtt.js`.

## 9. Autenticazione MQTT

Approccio semplificato adeguato al contesto (bilancia interna, dati non sensibili):

- Mosquitto configurato con username/password statica
- Le credenziali sono salvate nel `.env` di Laravel
- Laravel le passa al browser nel momento in cui serve la pagina di pesatura (iniettate nel JavaScript o via endpoint dedicato)
- L'ESP32 ha le stesse credenziali nel firmware (o credenziali dedicate per le bilance)
- Connessione TLS obbligatoria su entrambi i canali (WSS per browser, MQTTS per ESP32) per proteggere le credenziali in transito

Evoluzione futura possibile: credenziali per utente o token temporanei, se il contesto di sicurezza lo richiede.

## 10. Configurazione Mosquitto

Listener richiesti:

- Porta **8883**: MQTT over TLS, per le bilance ESP32
- Porta **8884**: WebSocket over TLS, per i browser

Certificati TLS: Let's Encrypt sul dominio `mqtt.tuodominio.it` (o sottodominio dedicato).

Regole firewall (Forge): aprire porte 8883 e 8884 in ingresso.

## 11. Requisiti firmware ESP32

Funzionalità da implementare:

- Client MQTT con TLS (libreria AsyncMqttClient o PubSubClient)
- Sottoscrizione a `minu/scale/{MAC}/command`
- Pubblicazione su `minu/scale/{MAC}/response` e `minu/scale/{MAC}/status`
- Registrazione LWT alla connessione
- Pubblicazione status `online` alla connessione (e riconnessione)
- Parsing JSON dei comandi ricevuti
- Visualizzazione peso target su OLED (al posto del placeholder attuale)
- Mappatura tasti esistenti su azioni `confirm` e `skip`
- Gestione riconnessione automatica al broker
- Il MAC address WiFi viene letto a runtime e usato come `scale_id`

Non richiesto lato firmware: tara (gestita localmente), streaming del peso, logica di ricetta.

## 12. Requisiti lato browser (JavaScript)

- Libreria: `mqtt.js` via CDN o bundle
- Connessione WSS al broker con credenziali iniettate da Laravel
- Client ID unico per ogni browser (es. `browser-{userId}`)
- Registrazione LWT alla connessione: messaggio `clear` retained sul topic command della bilancia associata
- Riconnessione automatica abilitata (built-in in `mqtt.js`)
- Gestione evento `reconnect`: se ingrediente attivo nella UI, ri-pubblica comando `weigh`
- Gestione evento `offline`: mostra indicatore di disconnessione nella UI
- Gestione sottoscrizioni per bilancia selezionata
- Validazione UUID nelle response (ignorare response con UUID non attivo)
- Pubblicazione esplicita di `clear` alla chiusura schermata di pesatura e a fine ricettazione
- Indicatore visivo stato bilancia (online/offline)
- POST REST a Laravel per persistenza risultati
- Cambio bilancia: disconnessione e riconnessione con nuovo LWT che punti alla nuova bilancia
- Gestione owner: alla selezione bilancia, pubblicare su topic owner con user_id e timestamp; sottoscriversi al topic owner e disabilitare invio comandi se un altro user_id prende il controllo

## 13. Requisiti lato Laravel

- Endpoint REST per persistenza pesate (nessuna logica MQTT)
- Salvataggio associazione bilancia-utente (tabella DB o campo utente)
- Credenziali MQTT in `.env`, iniettate nella view/endpoint
- Endpoint per lista bilance note (opzionale: Laravel può non essere coinvolto nella discovery, il browser la fa via MQTT)

## 14. Fuori scope (versione iniziale)

- Streaming peso real-time sul browser
- OTA firmware via gestionale
- Dashboard di monitoraggio bilance lato admin
- Notifiche push su disconnessione prolungata
