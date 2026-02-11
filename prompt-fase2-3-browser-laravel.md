# Prompt per Claude Code: Fase 2 e 3 - Browser MQTT + Integrazione Laravel

## Contesto

Il gestionale Gelateria Minù è un'applicazione Laravel 11 con admin Backpack 6 (tema Tabler, layout orizzontale). Il progetto Laravel si trova in `D:\xampp\htdocs\minu\manager\`. Deve integrarsi con bilance ESP32 tramite un broker MQTT (Mosquitto) sul VPS. La comunicazione real-time avviene direttamente tra browser e bilancia via MQTT. Laravel non partecipa al flusso real-time: serve solo la pagina, fornisce i dati, e persiste i risultati.

Il broker MQTT è già operativo su `mqtt.gelateriaminu.it`:
- Porta 8883: MQTT over TLS (per ESP32)
- Porta 8884: WebSocket over TLS (per browser)
- Autenticazione: username/password

Le credenziali MQTT sono già nel `.env` di Laravel (righe 55-58):
```
MQTT_HOST=mqtt.gelateriaminu.it
MQTT_WSS_PORT=8884
MQTT_USERNAME=minu
MQTT_PASSWORD=Monia080787!
```

Le bilance si identificano con il MAC address WiFi dell'ESP32 (minuscolo, senza separatori, es. `841fe838c774`), usato come `scale_id` nei topic MQTT. Il MAC è fisso per ogni scheda ESP32, non cambia mai.

**IMPORTANTE: prima di scrivere qualsiasi codice, leggi la struttura del progetto Laravel esistente, i modelli, le rotte, i controller e le view già presenti. Adattati all'architettura esistente, non imporne una nuova.**

### Architettura esistente (riferimenti chiave)

- **Modelli:** `app/Models/` — User, Product, Recipe, Ingredient, Category, Vendor, Batch, BulkProduction, Stock, StockMovement, Lead
- **CRUD Controllers:** `app/Http/Controllers/Admin/` — ProductCrudController (ha WeighOperation), RecipeCrudController, IngredientCrudController, ecc.
- **WeighOperation:** `app/Http/Controllers/Admin/Operations/WeighOperation.php` — trait usato da ProductCrudController, gestisce il flusso di pesatura attuale (HTTP-based, form POST per confirm, AJAX per reset)
- **Routes CRUD:** `routes/backpack/custom.php` — tutte le rotte CRUD sotto `/admin`
- **Menu sidebar:** `resources/views/vendor/backpack/ui/inc/menu_items.blade.php`
- **JS assets:** `public/assets/js/admin/forms/` — products.js, weight.js, recipes.js, ecc.
- **Permessi:** Spatie laravel-permission, pattern `{table_name}.see` / `{table_name}.edit`
- **Settings:** usa il package `backpack/settings` (SettingCrudController dal vendor), rotta in `routes/web.php` con middleware `can:settings`
- **Vite:** configurato (`vite.config.js`), jQuery/Moment/daterangepicker via CDN nel config Backpack UI
- **DB:** MySQL, database `manager`

---

## FASE 2: Browser MQTT (JavaScript)

### Obiettivo

Creare un modulo JavaScript riutilizzabile che gestisce la comunicazione MQTT tra il browser e le bilance. Questo modulo verrà usato dalle view di pesatura già esistenti nel gestionale.

### Libreria

`mqtt.js` (https://github.com/mqttjs/MQTT.js). Il progetto carica jQuery e altre librerie via CDN nel config Backpack UI. Per coerenza, includere mqtt.js via CDN in `config/backpack/ui.php` (array `scripts`) oppure direttamente nelle view che lo usano. Il file `ScaleMqttClient` va in `public/assets/js/admin/forms/scale-mqtt-client.js`.

### Modulo da creare: `ScaleMqttClient`

Creare un modulo/classe JavaScript che espone un'interfaccia pulita al resto dell'applicazione. Il modulo deve essere indipendente dal framework UI (funziona con Blade, Livewire, Alpine, qualsiasi cosa il gestionale usi).

#### Configurazione

Il modulo riceve le credenziali MQTT e il `scale_id` della bilancia associata all'utente. Questi dati vengono iniettati da Laravel nella pagina (vedi Fase 3). Lo `scale_id` proviene dalla tabella `scales` del DB (non da discovery MQTT).

```javascript
const scale = new ScaleMqttClient({
    host: 'wss://mqtt.gelateriaminu.it:8884/mqtt',
    username: 'minu',
    password: 'Monia080787!',
    scaleId: '841fe838c774',    // scale_id dalla tabella scales (FK su user)
    userId: 5                   // ID utente Laravel, per gestione owner
});
```

#### Connessione

Alla connessione (`scale.connect()`):

1. Apre connessione WSS verso il broker con client ID unico `browser-{userId}-{random}`
2. Registra LWT: topic `minu/scale/{scaleId}/command`, retain true, QoS 1, payload `{"type": "clear"}`
3. Si sottoscrive a:
   - `minu/scale/{scaleId}/response` (QoS 1) - risposte dalla bilancia
   - `minu/scale/{scaleId}/status` (QoS 1) - stato online/offline
   - `minu/scale/{scaleId}/owner` (QoS 1) - chi controlla la bilancia
4. Pubblica su `minu/scale/{scaleId}/owner` (retain true, QoS 1):
   ```json
   {"user_id": 5, "timestamp": 1707600000}
   ```
5. Abilita riconnessione automatica

#### Metodi pubblici

```javascript
// Invia comando di pesatura alla bilancia
scale.sendWeigh(uuid, name, targetWeight)
// Pubblica su minu/scale/{scaleId}/command:
// {"type": "weigh", "uuid": "...", "name": "...", "target_weight": 450.0}
// Retain: true, QoS: 1
// Salva internamente uuid come activeUuid

// Pulisce il comando attivo
scale.sendClear()
// Pubblica su minu/scale/{scaleId}/command:
// {"type": "clear"}
// Retain: true, QoS: 1
// Resetta activeUuid a null

// Disconnessione pulita
scale.disconnect()
// Pubblica clear, poi disconnette

// Cambia bilancia
scale.changeScale(newScaleId)
// Pubblica clear e owner vuoto sulla vecchia, disconnette, riconnette con nuovo LWT per la nuova bilancia

// Riprende il controllo della bilancia (se un altro operatore l'aveva presa)
scale.claimOwnership()
// Ri-pubblica owner con proprio userId e timestamp corrente
```

#### Callback/eventi

Il modulo emette eventi che il codice chiamante può ascoltare:

```javascript
// La bilancia ha confermato una pesata
scale.onConfirm = function(uuid, actualWeight) { }

// La bilancia ha skippato un ingrediente
scale.onSkip = function(uuid) { }

// Lo stato della bilancia è cambiato
scale.onStatusChange = function(state) { }
// state: 'online', 'offline', 'error'

// Lo stato della connessione MQTT del browser è cambiato
scale.onConnectionChange = function(connected) { }
// connected: true/false

// Un altro operatore ha preso il controllo della bilancia
scale.onOwnerChange = function(isOwner, ownerUserId) { }
// isOwner: true se siamo noi, false se è un altro
// ownerUserId: l'id dell'utente che ha il controllo
```

#### Gestione proprietà (owner)

Quando il modulo riceve un messaggio sul topic `minu/scale/{scaleId}/owner`:

- Se `user_id` corrisponde al proprio `userId`: siamo il proprietario, nessuna azione
- Se `user_id` è diverso dal proprio `userId`: un altro browser ha preso il controllo
  - Impostare flag interno `isOwner = false`
  - Emettere `onOwnerChange(false, ownerUserId)`
  - Tutti i metodi di invio (`sendWeigh`, `sendClear`) diventano no-op finché `isOwner` è false
  - Il modulo continua a ricevere messaggi (status, response) ma non invia comandi

Quando l'utente vuole riprendere il controllo, chiama `scale.claimOwnership()` che ri-pubblica il messaggio owner con il proprio `userId` e timestamp corrente. Gli altri browser riceveranno il cambio e cederanno.

Alla disconnessione pulita (`scale.disconnect()` o `scale.changeScale()`), il browser pubblica un owner vuoto retained per rilasciare il controllo:

```json
{"user_id": null, "timestamp": 1707600000}
```

Alla disconnessione imprevista (crash, standby), il LWT copre solo il topic command (clear). Il topic owner resta con il vecchio valore. Il prossimo browser che si associa sovrascriverà l'owner.

#### Validazione UUID

Quando arriva una response dalla bilancia (`confirm` o `skip`), il modulo confronta l'UUID della response con `activeUuid`. Se non corrispondono, il messaggio viene ignorato silenziosamente (l'operatore ha già cambiato ingrediente dal browser).

#### Gestione riconnessione (standby tablet)

Quando `mqtt.js` si riconnette dopo una disconnessione:

1. Il modulo verifica se `activeUuid` è valorizzato (c'era un ingrediente attivo)
2. Se sì: ri-pubblica il comando `weigh` con lo stesso uuid, name e targetWeight
3. Se no: non fa nulla (il LWT ha già pubblicato `clear`)

Il modulo deve quindi conservare internamente l'ultimo comando `weigh` inviato (uuid, name, targetWeight) per poterlo ri-pubblicare.

### Cosa NON fare lato JavaScript

- Non creare UI/componenti visivi: il modulo è solo logica MQTT, la UI la gestisce il codice chiamante
- Non fare chiamate REST a Laravel: quello lo fa il codice chiamante dopo aver ricevuto `onConfirm`
- Non gestire logica di ricetta o lista ingredienti

---

## FASE 3: Integrazione Laravel

### 3.1 Variabili d'ambiente

Già presenti nel `.env` (righe 55-58). Aggiungere anche in `.env.example` per documentazione.

Creare `config/mqtt.php`:
```php
<?php
return [
    'host' => env('MQTT_HOST', 'mqtt.gelateriaminu.it'),
    'wss_port' => env('MQTT_WSS_PORT', 8884),
    'username' => env('MQTT_USERNAME'),
    'password' => env('MQTT_PASSWORD'),
];
```

### 3.2 Modello e migrazione: tabella `scales`

Le bilance vengono registrate nel gestionale dall'admin. Creare:

**Migrazione** `create_scales_table`:
```
Tabella: scales

- id (auto-increment)
- scale_id (string, 12 chars, unique) — MAC address lowercase senza separatori (es. "841fe838c774")
- name (string) — nome human-readable (es. "Bilancia Laboratorio")
- active (boolean, default true) — se false, non appare nella lista di selezione
- notes (text, nullable) — note libere per l'admin
- created_at
- updated_at
```

**Migrazione** `add_scale_id_to_users_table`:
```
Aggiunge colonna:
- scale_id (unsignedBigInteger, nullable, FK → scales.id) alla tabella users
```

**Modello** `app/Models/Scale.php`:
```php
- Campi fillable: scale_id, name, active, notes
- Relazione: hasMany(User::class, 'scale_id')
- Trait: CrudTrait, HasRoles (come tutti gli altri modelli del progetto)
```

**Aggiornare** `app/Models/User.php`:
```php
- Aggiungere relazione: belongsTo(Scale::class, 'scale_id')
```

### 3.3 Gestione bilance nell'admin (CRUD Backpack)

Le bilance sono un'entità gestita dall'admin nella sezione di configurazione del gestionale, accanto ai Settings già esistenti. L'admin deve poter:

- **Vedere la lista** di tutte le bilance registrate, con nome, identificativo MAC e stato attivo/disattivo
- **Aggiungere una bilancia** manualmente, inserendo l'identificativo MAC (12 caratteri esadecimali minuscoli, senza separatori, es. `841fe838c774`), un nome descrittivo (es. "Bilancia Laboratorio"), lo stato attivo/disattivo e note opzionali
- **Modificare** nome, stato attivo e note di una bilancia esistente
- **Disattivare** una bilancia (non eliminarla) per escluderla dalla lista di selezione degli operatori

Il CRUD delle bilance segue lo stesso pattern degli altri CRUD del progetto: stessi permessi (`scales.see` / `scales.edit`), stessa struttura controller, stessa posizione nelle rotte. La voce di menu "Bilance" va nella sezione "Super Poteri" accanto a "Settings", accessibile solo agli admin.

In più, nella lista delle bilance è presente un bottone **"Cerca bilance sulla rete"** che avvia una discovery MQTT per trovare automaticamente le bilance connesse al broker e proporre la registrazione di quelle non ancora presenti nel database (vedi sezione 3.4).

### 3.4 Discovery MQTT come helper per auto-registrazione

Nella pagina CRUD list delle bilance, aggiungere un bottone "Cerca bilance sulla rete" che:

1. Apre un modal
2. Si connette via WSS al broker MQTT
3. Subscribe a `minu/scale/+/status`
4. Riceve i messaggi retained di tutte le bilance che si sono mai connesse
5. Mostra una lista con:
   - `scale_id` (MAC)
   - `name` (dal firmware)
   - `state` (online/offline)
   - `firmware_version`
   - Bottone "Registra" per le bilance non ancora nel DB
6. Dopo timeout (3 secondi) o click utente, disconnette

Il formato dei messaggi status pubblicati dal firmware (retained):

**Bilancia online:**
```json
{"type":"status","state":"online","scale_id":"841fe838c774","name":"Minu Bench Scale","firmware_version":"1.0.0"}
```

**Bilancia offline (LWT):**
```json
{"type":"status","state":"offline","scale_id":"841fe838c774","name":"Minu Bench Scale"}
```

Quando l'admin clicca "Registra", pre-compila il form di creazione bilancia con `scale_id` e `name` dal messaggio MQTT. L'admin può modificare il nome prima di salvare.

### 3.5 Associazione bilancia-utente

Due approcci complementari:

**1. Nel CRUD utenti (admin)**
Aggiungere un campo select nel form utente per associare una bilancia dalla tabella `scales`. Mostra solo le bilance attive.

**2. Nella schermata di pesatura (operatore)**
Se l'utente non ha una bilancia associata, mostrare un selettore inline con le bilance attive dal DB. La selezione viene salvata sulla colonna `scale_id` della tabella `users`.

Endpoint REST per la selezione dall'operatore:
```
PUT /api/user/scale
Body: { "scale_id": 3 }  // ID dalla tabella scales
Response: 200 { "scale_id": 3, "scale_name": "Bilancia Laboratorio" }
```

```
GET /api/user/scale
Response: 200 { "scale_id": 3, "scale_name": "Bilancia Laboratorio", "scale_mac": "841fe838c774" }
oppure 200 { "scale_id": null }
```

### 3.6 Endpoint REST per registrazione pesata

```
POST /api/weighing/record
Body: {
    "ingredient_uuid": "f47ac10b-58cc-4372-a567-0e02b2c3d479",
    "actual_weight": 448.5,
    "action": "confirm"
}
Response: 200 { "success": true }
```

```
POST /api/weighing/record
Body: {
    "ingredient_uuid": "f47ac10b-58cc-4372-a567-0e02b2c3d479",
    "action": "skip"
}
Response: 200 { "success": true }
```

**Nota:** l'endpoint deve integrarsi con la logica di pesatura esistente in `WeighOperation.php`. Quel trait gestisce già confirm e skip via form POST (rotte `POST /admin/product/{id}/weigh` e `POST /admin/product/{id}/weigh/skip`). Gli ingredienti hanno UUID nel campo JSON `ingredients` dei prodotti. L'endpoint REST deve usare la stessa logica transazionale con `lockForUpdate()`.

### 3.7 Iniezione credenziali MQTT nelle view

Laravel deve rendere disponibili le credenziali MQTT e lo `scale_id` dell'utente al JavaScript.

Approccio consigliato: Blade partial (coerente con il progetto che usa partial e script inline).

```php
// resources/views/partials/mqtt-config.blade.php
<script>
    window.mqttConfig = {
        host: 'wss://{{ config("mqtt.host") }}:{{ config("mqtt.wss_port") }}/mqtt',
        username: '{{ config("mqtt.username") }}',
        password: '{!! config("mqtt.password") !!}',
        scaleId: '{{ auth()->user()->scale?->scale_id ?? "" }}',
        scaleName: '{{ auth()->user()->scale?->name ?? "" }}',
        userId: {{ auth()->user()->id }}
    };
</script>
```

Incluso nelle view che necessitano di MQTT con `@include('partials.mqtt-config')`.

### 3.8 Indicatore stato bilancia

Nella schermata di pesatura, mostrare un indicatore visivo dello stato della bilancia connessa:
- Online: pallino verde (o simile)
- Offline: pallino rosso con avviso
- Nessuna bilancia associata: invito a configurarne una

Aggiornato in tempo reale dal callback `onStatusChange` del modulo `ScaleMqttClient`.

---

## Riepilogo protocollo MQTT (firmware ↔ browser)

### Topic

| Topic | Direzione | QoS | Retain | Descrizione |
|-------|-----------|-----|--------|-------------|
| `minu/scale/{scale_id}/status` | Bilancia → Browser | 1 | **sì** | Stato online/offline con nome bilancia |
| `minu/scale/{scale_id}/command` | Browser → Bilancia | 1 | sì | Comandi pesatura (`weigh`, `clear`) |
| `minu/scale/{scale_id}/response` | Bilancia → Browser | 1 | no | Risposte (`confirm`, `skip`) |
| `minu/scale/{scale_id}/owner` | Browser → Browser | 1 | sì | Chi controlla la bilancia |
| `minu/scale/+/status` | Wildcard discovery | 1 | — | Per scoprire tutte le bilance |

### Messaggi dal firmware (bilancia → broker)

**Status online** (pubblicato alla connessione MQTT, retained):
```json
{"type":"status","state":"online","scale_id":"841fe838c774","name":"Minu Bench Scale","firmware_version":"1.0.0"}
```

**Status offline / LWT** (pubblicato automaticamente dal broker alla disconnessione, retained):
```json
{"type":"status","state":"offline","scale_id":"841fe838c774","name":"Minu Bench Scale"}
```

**Confirm** (quando l'operatore preme ENTER sulla bilancia):
```json
{"type":"confirm","uuid":"f47ac10b-58cc-4372-a567-0e02b2c3d479","actual_weight":448.0}
```

**Skip** (quando l'operatore preme SKIP sulla bilancia):
```json
{"type":"skip","uuid":"f47ac10b-58cc-4372-a567-0e02b2c3d479"}
```

### Messaggi dal browser (browser → bilancia)

**Weigh** (invia comando di pesatura):
```json
{"type":"weigh","uuid":"f47ac10b-58cc-4372-a567-0e02b2c3d479","name":"Zucchero","target_weight":450.0}
```

**Clear** (annulla comando attivo):
```json
{"type":"clear"}
```

### Note sul firmware

- La bilancia gestisce **un solo comando alla volta**: un nuovo `weigh` sovrascrive il precedente
- Dopo `confirm` o `skip`, la bilancia torna automaticamente in idle (cancella target dal display)
- Il campo `name` nel `weigh` è solo per il log seriale, non viene mostrato sul display della bilancia
- Il `target_weight` viene mostrato sul display della bilancia accanto all'icona target (in grammi)
- Il `actual_weight` nel `confirm` è il peso letto dalla bilancia al momento in cui l'operatore preme ENTER (in grammi)

### Bilancia attualmente disponibile per test

- **Nome:** Minu Bench Scale
- **scale_id:** `841fe838c774`
- **MAC:** `84:1f:e8:38:c7:74`
- **Topic:** `minu/scale/841fe838c774/command`, `.../response`, `.../status`

---

## Cosa NON fare

- Non installare package PHP per MQTT (es. php-mqtt/client). Laravel non comunica con Mosquitto.
- Non creare daemon o worker Laravel per MQTT.
- Non creare un sistema di notifiche per disconnessioni prolungate.
- Non sovrascrivere o ristrutturare la logica di pesatura esistente in WeighOperation.php: integrarsi con essa.
- Non usare la discovery MQTT come fonte primaria della lista bilance: la fonte è la tabella `scales` nel DB.

## Risultato atteso

1. L'admin può gestire le bilance dalla sezione Config di Backpack (CRUD con lista, creazione, modifica)
2. L'admin può scoprire bilance sulla rete con il bottone "Cerca bilance" e registrarle con un click
3. L'admin può associare una bilancia a un utente dal CRUD utenti
4. L'operatore può scegliere/cambiare bilancia dalla schermata di pesatura
5. Il modulo JavaScript `ScaleMqttClient` è disponibile nelle pagine del gestionale
6. Nella schermata di pesatura, quando l'utente seleziona un ingrediente, il browser invia il comando alla bilancia via MQTT
7. Quando la bilancia conferma o skippa, il browser riceve la response e aggiorna la UI
8. Il browser salva il risultato via REST API su Laravel
9. Lo stato della bilancia (online/offline) è visibile in tempo reale
10. Chiusura tab, standby tablet e crash sono gestiti correttamente tramite LWT e riconnessione

## Test

- Dall'admin: creare una bilancia con scale_id `841fe838c774`, associarla a un utente
- Dall'admin: usare "Cerca bilance" per trovare la bilancia sulla rete e verificare nome/stato
- Dal browser: selezionare un ingrediente, verificare che il comando arrivi sulla bilancia (usando MQTTX come simulatore)
- Da MQTTX: simulare una response `confirm` e verificare che il browser la riceva e aggiorni la UI
- Testare riconnessione: mettere il tablet in standby, risvegliare, verificare che il comando venga ri-pubblicato
- Testare LWT: chiudere il tab bruscamente, verificare che la bilancia (MQTTX) riceva `clear`
- Testare owner: aprire due browser sulla stessa bilancia, verificare che il primo venga notificato quando il secondo prende il controllo
