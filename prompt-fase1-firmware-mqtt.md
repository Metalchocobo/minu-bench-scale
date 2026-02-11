# Prompt per Claude Code: Fase 1 - Integrazione MQTT nel firmware ESP32 Minù Bench Scale

## Contesto del progetto

Minù Bench Scale è una bilancia di precisione per laboratorio gelato basata su ESP32.

La bilancia deve ora integrarsi con un gestionale web Laravel. La comunicazione avviene tramite un broker MQTT (Mosquitto) installato su un VPS esterno. Il browser del gestionale e la bilancia comunicano tramite MQTT; Laravel non è coinvolto nel flusso real-time.

## Cosa deve fare la bilancia (e solo questo)

La bilancia è una periferica del browser. Riceve comandi e risponde. Non ha logica di ricetta, non sa nulla del gestionale.

### Connessione MQTT

- Connettersi al broker MQTT con TLS sulla porta 8883
- Host: `mqtt.gelateriaminu.it`
- Autenticazione: username/password (da definire come costanti nel firmware, poi si sposteranno in config)
- Riconnessione automatica al broker in caso di disconnessione
- Il MAC address WiFi dell'ESP32 (minuscolo, senza separatori, es. `a0b1c2d3e4f5`) è lo `scale_id` usato nei topic

### UI COnnessione
- Se è conessa al wifi, deve automaticamente connettersi al server.
- Quando connessa al Server, aggiungere un'icona affiancoa quella del WIFI della medesima dimensione del medesimo stile. Se non si connette al servere qualsiasi problema, far lampeggiare l'icona e indicare l'errore via LOG seriale. 
- Se non è connesso al wifi, l'icona non si deve proprio vedere.

### Log
- Loggare via seriale messaggi ed errori relativi alla comunicazione.

### Topic MQTT

| Topic | Direzione | Retain | QoS |
|---|---|---|---|
| `minu/scale/{scale_id}/command` | Riceve (subscribe) | Sì | 1 |
| `minu/scale/{scale_id}/response` | Invia (publish) | No | 1 |
| `minu/scale/{scale_id}/status` | Invia (publish) | Sì | 1 |

### Last Will and Testament (LWT)

Alla connessione, registrare il messaggio LWT:
- Topic: `minu/scale/{scale_id}/status`
- Retain: Sì
- QoS: 1
- Payload:
```json
{"type": "status", "state": "offline", "scale_id": "{scale_id}"}
```

Alla connessione riuscita (e ad ogni riconnessione), pubblicare immediatamente:
- Topic: `minu/scale/{scale_id}/status`
- Retain: Sì
- Payload:
```json
{"type": "status", "state": "online", "scale_id": "{scale_id}", "firmware_version": "x.y.z"}
```

### Messaggi in ricezione (topic command)

**Comando pesatura:**
```json
{
  "type": "weigh",
  "uuid": "f47ac10b-58cc-4372-a567-0e02b2c3d479",
  "name": "Zucchero invertito",
  "target_weight": 450.0
}
```

Alla ricezione:
- Salvare `uuid` e `target_weight` in variabili
- Mostrare il `target_weight` sul display OLED nel placeholder già previsto per il peso target.
- Quando la bilancia non è connessa o legge il comando clear o non legge alcun peso non mostra peso target e relativa icona (quindi di default non si vede nulla)
- Se arriva un nuovo comando `weigh` mentre uno è già attivo, sovrascrivere (aggiornare display con il nuovo target)
- Il campo `name` può essere ignorato lato display se lo spazio è insufficiente

**Comando clear:**
```json
{
  "type": "clear"
}
```

Alla ricezione:
- Cancellare `uuid` e `target_weight`
- Rimuovere il peso target dal display (tornare in stato idle)

### Messaggi in invio (topic response)

**Conferma pesata** (operatore preme il tasto conferma sulla bilancia):
```json
{
  "type": "confirm",
  "uuid": "{uuid corrente}",
  "actual_weight": 448.5
}
```
`actual_weight` è il peso attualmente letto dalla cella di carico al momento della conferma.

**Skip ingrediente** (operatore preme il tasto skip sulla bilancia):
```json
{
  "type": "skip",
  "uuid": "{uuid corrente}"
}
```

Dopo l'invio di `confirm` o `skip`, la bilancia torna in stato idle (cancella uuid e target dal display) e attende il prossimo comando.

Se non c'è nessun comando attivo (uuid vuoto), i tasti confirm e skip non devono pubblicare nulla su MQTT.

### Gestione disconnessione WiFi

- Se cade la connessione con mqtt o wifi seguire le logiche indicate all'inizio del prompt. Se cade solo la connessione mqqt emettere due bip di avviso tramite buzzer. 
- Non bloccare mai il funzionamento base della bilancia (lettura peso, tara, display peso corrente)
- Alla riconnessione, il messaggio retained sul topic command verrà recapitato automaticamente dal broker

## Vincoli tecnici

- Il framework è Arduino IDE con ESP32
- Integra il codice MQTT nel file esistente
- Per il parsing JSON si suggerisce ArduinoJson (se non è già incluso, aggiungilo) 
- Per MQTT valuta PubSubClient o AsyncMqttClient. Scegli in base alla compatibilità con il resto del firmware (se il loop è sincrono, PubSubClient è più semplice)
- TLS: per Let's Encrypt su ESP32, usa il root CA certificate ISRG Root X1. Includilo come stringa nel firmware o usa `WiFiClientSecure` con `setCACert()`
- IMPORTANTE: prima di modificare qualsiasi cosa, leggi il firmware esistente per capire la struttura, i pin usati, le variabili globali, il loop principale e i tasti già mappati. Non inventare API o funzioni che non esistono. Adattati al codice esistente.
- Dopo le modifiche, verifica che il codice compili.

## Cosa NON fare

- Non toccare la logica di lettura peso, tara, calibrazione
- Non toccare la logica audio/DFPlayer
- Non toccare la logica batteria/INA219
- Non toccare la logica OTA
- Non creare un'architettura modulare (il refactoring è un progetto separato)
- Non aggiungere streaming del peso via MQTT
- Non aggiungere logica di ricetta o lista ingredienti sulla bilancia
- Non toccare in generale nulla che non riguardi questa parte IOT. 

## Risultato atteso

La bilancia, una volta connessa al WiFi e al broker MQTT:
1. Pubblica il suo stato `online` con il proprio MAC come scale_id
2. Resta in ascolto di comandi sul topic command
3. Quando riceve `weigh`, mostra il peso target sul display
4. Quando l'operatore preme conferma, pubblica il peso reale su response
5. Quando l'operatore preme skip, pubblica skip su response
6. Quando riceve `clear`, torna in idle
7. Se si disconnette, il broker pubblica automaticamente lo stato `offline` (LWT)
8. Alla riconnessione, riceve l'ultimo comando retained e riprende

## Test

Dopo l'implementazione, si testerà con MQTTX (client MQTT desktop):
- Pubblicare un comando `weigh` a mano e verificare che la bilancia lo riceva e mostri il target
- Premere confirm/skip sulla bilancia e verificare che il messaggio arrivi su MQTTX
- Pubblicare `clear` e verificare che la bilancia torni in idle
- Disconnettere/riconnettere il WiFi e verificare LWT e riconnessione


## Altre note operative
- WiFiClientSecure::connect() non deve essere bloccante. Forse FreeRTOS task separato one-shot
- UUID: 36 char standard → buffer char[37], MQTT_MAX_PACKET_SIZE a 512 è più che sufficiente.
- Icona di MQTT: due frecce ↑↓ (scambio dati bidirezionale) — leggibili anche da piccole dimensioni
-  Due bip per disconnessione MQTT buzzerWarn()
- Strategia riconnessione MQTT: Propongo backoff esponenziale: 2s → 4s → 8s → 16s → 30s (cap). Evita di martellare il broker. Reset del backoff alla connessione riuscita. Ma valuta tu come è meglio.
- Le specifiche dicono: "Se non c'è nessun comando attivo (uuid vuoto), i tasti confirm e skip non devono pubblicare nulla su MQTT." Attualmente ENTER e SKIP riproducono tracce audio (ENTER_PRESSED, SKIP_PRESSED).Enter va lasciato così , anche perchè enter avrà anche un un'altra funzione in futuro. Skip invece bip buzzer di avviso "sordo", come avviso. Va bene anche buzzerWarn()