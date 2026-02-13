# Prompt Firmware: Stack pesate, tara automatica post-confirm, Total e Clear

## Contesto

Minù Bench Scale ha già integrazione MQTT funzionante (Fase 1).
Ora si aggiunge la logica di **stack pesate locale** sulla bilancia.
Questa funzionalità è **indipendente dal gestionale**: funziona anche senza connessione MQTT.

Il workflow tipico in laboratorio è: un unico contenitore (pentola/bowl) dove si aggiungono ingredienti uno dopo l'altro, pesandoli cumulativamente.

## Obiettivo

L'operatore deve poter:
1. Mettere un contenitore, fare tara
2. Aggiungere ingredienti uno alla volta, confermando ogni pesata con Enter
3. Vedere in qualsiasi momento il totale registrato (somma stack)
4. Vedere il peso reale nel contenitore (rispetto alla tara originale) come riscontro
5. Annullare l'ultima pesata se ha commesso un errore
6. Svuotare tutto lo stack per ricominciare

## Struttura dati

### Variabili globali da aggiungere

```cpp
// Stack pesate (LIFO) — RAM, non persistente
#define WEIGH_STACK_MAX 50          // Max ingredienti per sessione
float weighStack[WEIGH_STACK_MAX];  // Pesi registrati
int weighStackCount = 0;            // Numero elementi nello stack

// Tara di riferimento (punto zero con contenitore vuoto)
float referenceOffset = 0.0;        // Offset tara di riferimento
bool hasReferenceOffset = false;    // Se è stata fatta una tara "di riferimento"
```

## Comportamento tasti

### Tasto TARA (comportamento esistente + estensioni)

Mantiene il comportamento attuale (fa tara sulla cella di carico) e aggiunge:
1. **Salva il punto come "tara di riferimento"**: `referenceOffset = <offset corrente della cella>`; `hasReferenceOffset = true`
2. **Azzera lo stack pesate**: `weighStackCount = 0` (nuova sessione di pesatura)

Logica: quando l'operatore preme Tara, sta iniziando una nuova sessione (nuovo contenitore o ricominciare da zero). Quindi lo stack precedente non ha più senso.

### Tasto ENTER/CONFIRM (comportamento esistente + estensioni)

Mantiene il comportamento attuale (se c'è UUID MQTT attivo, pubblica `confirm`) e aggiunge **sempre** (con o senza MQTT):

1. **Leggi il peso corrente** dal display (il valore che l'operatore vede)
2. **Se il peso è > 0**: push nello stack:
   ```cpp
   if (weighStackCount < WEIGH_STACK_MAX && currentWeight > 0) {
       weighStack[weighStackCount] = currentWeight;
       weighStackCount++;
   }
   ```
3. **Esegui tara automatica** (stessa funzione del tasto Tara, ma **senza** azzerare lo stack e **senza** aggiornare la tara di riferimento)
4. Il display torna a `0.00g`, pronto per il prossimo ingrediente

**IMPORTANTE**: la tara automatica post-confirm NON deve azzerare lo stack e NON deve aggiornare `referenceOffset`. È una tara "di lavoro", non "di riferimento".

Ordine operazioni: lettura peso → push stack → pubblica MQTT confirm (se UUID attivo) → tara automatica.

### Tasto TOTAL — Pressione breve

Mostra la **somma dello stack** (totale pesate registrate).

```cpp
float stackTotal = 0.0;
for (int i = 0; i < weighStackCount; i++) {
    stackTotal += weighStack[i];
}
```

Comportamento display:
- Prima pressione: mostra il totale stack sul display, in una modalità visivamente distinguibile (es. icona Σ, o testo "TOT" accanto al valore, o inversione colori — adattati allo stile esistente del display)
- Seconda pressione (o dopo timeout ~3s): torna al peso live
- Se lo stack è vuoto, mostra `0.0g`

### Tasto TOTAL — Pressione lunga (~2 secondi)

Mostra il **peso reale rispetto alla tara di riferimento originale** (= peso fisico nel contenitore).

Questo valore si calcola dalla differenza tra la lettura raw attuale della cella e il `referenceOffset` salvato al momento della tara manuale iniziale.

```cpp
// Pseudocodice — adatta alla lettura raw della cella nel firmware esistente
float realWeight = getCurrentRawReading() - referenceOffset;
```

Comportamento display:
- Mostra il peso reale con indicatore visivo diverso dal total stack (es. icona contenitore, o testo "NET", o altro — distinguibile dal Total breve)
- Seconda pressione (o dopo timeout ~3s): torna al peso live
- Se `hasReferenceOffset == false`, mostra il peso live (non c'è riferimento)

**Utilità**: confrontando Total breve (stack) e Total lungo (peso reale), l'operatore vede se i numeri coincidono. Se divergono, qualcosa è andato storto (ingrediente versato fuori, errore di pesata, ecc.).

### Tasto CLEAR — Pressione breve

**Rimuove l'ultima pesata dallo stack** (LIFO pop). Non fa nient'altro.

```cpp
if (weighStackCount > 0) {
    weighStackCount--;
    // Opzionale: bip conferma breve
}
```

- Non modifica la tara corrente
- Non modifica il display del peso live
- Non modifica la tara di riferimento
- Se lo stack è vuoto, non fa nulla (opzionale: bip di avviso "stack vuoto")

### Tasto CLEAR — Pressione lunga (~2 secondi)

**Svuota completamente lo stack**.

```cpp
weighStackCount = 0;
// Opzionale: bip conferma doppio
```

- Non modifica la tara corrente
- Non modifica il display del peso live
- Non modifica la tara di riferimento

## Riepilogo tasti

| Tasto | Pressione | Effetto tara | Effetto stack | Effetto display | Effetto tara rif. |
|---|---|---|---|---|---|
| **Tara** | — | Nuovo zero | Azzera | → 0 | Salva riferimento |
| **Enter** | — | Tara auto | Push peso | → 0 | Nessuno |
| **Total** | Breve | Nessuno | Nessuno | Mostra somma stack | Nessuno |
| **Total** | Lungo 2s | Nessuno | Nessuno | Mostra peso da rif. | Nessuno |
| **Clear** | Breve | Nessuno | Pop ultimo | Nessuno | Nessuno |
| **Clear** | Lungo 2s | Nessuno | Azzera tutto | Nessuno | Nessuno |

## Interazione con MQTT

- **Nessuna modifica ai messaggi MQTT**. I payload `confirm`, `skip`, `weigh`, `clear`, `status` restano identici.
- Il push nello stack avviene **prima** del publish MQTT confirm, così se il publish fallisce il peso è comunque registrato localmente.
- Il comando MQTT `clear` (dal browser) **non** tocca lo stack pesate (lo stack è locale, il clear MQTT riguarda solo l'UUID/target).
- Il comando MQTT `weigh` **non** tocca lo stack pesate.

## Gestione pressione lunga

Se il firmware non ha già un pattern per distinguere pressione breve/lunga, implementare:

```cpp
// Pseudocodice
unsigned long pressStart = 0;
bool longPressHandled = false;

// Nel loop o nell'handler del tasto:
if (buttonJustPressed) {
    pressStart = millis();
    longPressHandled = false;
}

if (buttonIsHeld && !longPressHandled && (millis() - pressStart >= 2000)) {
    // Azione pressione lunga
    longPressHandled = true;
}

if (buttonJustReleased && !longPressHandled) {
    // Azione pressione breve
}
```

Adatta questo pattern ai tasti TOTAL e CLEAR. I tasti TARA e ENTER restano a pressione singola come oggi.

## Feedback audio (opzionale ma consigliato)

| Azione | Feedback |
|---|---|
| Enter/Confirm con push stack | Bip conferma (esistente) |
| Clear breve (pop) | Bip breve singolo |
| Clear lungo (svuota) | Doppio bip |
| Clear breve su stack vuoto | Bip di avviso (buzzerWarn) |
| Total (mostra) | Nessuno (o bip brevissimo) |

## Vincoli

- La logica stack è in **RAM**, non persistente. Si perde al riavvio — ed è corretto così.
- Il display del Total/peso reale è temporaneo (toggle o timeout). Non deve bloccare il funzionamento normale.
- Non deve mai bloccare il loop principale o la lettura peso.
- WEIGH_STACK_MAX = 50 è più che sufficiente (una ricetta gelato ha 8-15 ingredienti tipicamente).
- Prima di implementare, leggere il firmware per capire come sono gestiti i tasti attualmente (handler nel loop, interrupt, libreria button, ecc.) e adattarsi.

## Cosa NON fare

- Non modificare la logica MQTT esistente
- Non modificare i payload MQTT
- Non trasmettere lo stack via MQTT
- Non rendere lo stack persistente (EEPROM/SPIFFS)
- Non aggiungere UI complesse (liste ingredienti, nomi, ecc.) — solo peso numerico
- Non toccare logica audio/batteria/OTA/calibrazione

## Test

1. Tara con contenitore → display 0
2. Aggiungi 200g, premi Enter → stack [200], display → 0 (tara auto)
3. Aggiungi 300g, premi Enter → stack [200, 300], display → 0
4. Total breve → mostra 500g
5. Total lungo → mostra ~500g (peso reale da tara riferimento)
6. Clear breve → stack [200], nessun cambio display peso
7. Total breve → mostra 200g
8. Clear lungo → stack vuoto
9. Total breve → mostra 0g
10. Tara → nuovo riferimento, stack già vuoto
