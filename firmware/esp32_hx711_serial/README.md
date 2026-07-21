# esp32_hx711_serial — Firmware HX711 (ESP32)

Le fonti canoniche sono mantenute nella root del repository:

- `../../README.md` — comportamento firmware, comandi, MQTT, build e diagnostica.
- `../../docs/WIRING.md` — pinout e cablaggio fisico.

## Riferimento rapido MQTT firmware 1.4.4

- Un `weigh` può includere `command_id` (8–64 caratteri; lettere ASCII, cifre, `-`, `_`; vuoto solo per legacy). Il firmware lo memorizza con UUID, prodotto, `session_id` e `connection_id`. Se quest'ultima è presente, il comando diventa operativo soltanto con l'owner retained fresco ed esatto e, dopo reconnect, dopo avere processato anche lo snapshot retained `command` corrente.
- Dopo l'attivazione di un comando tokenizzato pubblica sul topic `response` un `command_ack` transitorio con `command_id`, `session_id`, `connection_id`, UUID, prodotto e `state=active`. Un replay identico dello stesso ID non reinizializza la pesata, ma riaggancia un eventuale fallback locale, aggiorna la connessione corrente e riemette l'ACK. Un payload business conflittuale viene ignorato senza ACK.
- Il Manager può pubblicare sul topic `command`, QoS 1 e non retained, una `confirm_request` fenced da `request_id`, sessione, connessione, comando, UUID e prodotto. Il loop la esegue tramite lo stesso helper e lo stesso lock TARE di ENTER fisico. Lo stesso request ID non ripete l'azione; una cache RAM mantiene gli ultimi due esiti terminali anche se il relativo ACK si perde. `accepted`/`staged` non avanzano la pagina e `failed`/`rejected` lasciano attivo l'ingrediente corrente.
- Un `clear` live cancella un comando moderno solo se coincidono `command_id` e `session_id`: con owner fresco richiede la connessione corrente, mentre senza owner accetta il retained tombstone esatto. Al reconnect il primo retained clear moderno è invece lo snapshot autorevole del broker e pulisce il raw RAM anche se la coppia è già avanzata. Il clear speciale `lifecycle=pagehide` sgancia l'owner corrente anche senza comando; cancella il raw solo se coincide anche la fence comando. Un comando v1.2 senza ID richiede la stessa sessione; un comando realmente legacy senza sessione accetta il clear senza ID del Manager nuovo.
- Una response durable `confirm` o `skip` conserva il `command_id` originale. L'`undo` usa `undo_of_response_id` e non inventa un nuovo command ID.
- Se l'owner scade, un comando moderno viene conservato raw in RAM ma reso non operativo e non visibile. Da 1.4.2 la bilancia passa alla modalità locale: TARE e ENTER fisici funzionano senza creare nuove response MQTT; un nuovo `weigh` valido riaggancia il Manager. Da 1.4.4 il loop elabora prima un burst limitato a quattro pacchetti MQTT già disponibili e soltanto dopo valuta la scadenza, così un normale heartbeat accodato sul limite dei 15 secondi resta valido senza affamare tasti e bilancia.
- L'outbox durable resta immutabile e continua il retry ogni secondo fino al `response_ack` esatto, ma non può bloccare indefinitamente l'operatore. Il lock fisico dura al massimo 10 secondi, termina prima su `clear` con `lifecycle=pagehide`/perdita owner e viene sganciato immediatamente dalla prima pressione fisica di TARE o ENTER. Un owner/snapshot remoto mai ricevuto usa la stessa grace. DNS, TCP, TLS e MQTT CONNECT vengono eseguiti da un worker FreeRTOS separato: il loop continua sempre a campionare tastiera e bilancia. TARE/ENTER diventano locali; SKIP/CLEAR remoto e nuovi outbox restano serializzati. Un undo detached viene applicato localmente una volta sola e non ripetuto all'ACK tardivo.
- Con un owner vivo ENTER richiede un comando attivo. Senza consumer remoto usa il percorso standalone anche se MQTT è ancora connesso; con MQTT disconnesso e un comando remoto non ancora dichiarato stale attende al massimo la stessa finestra di 10 secondi.
- La barra OLED separa il trasporto dalla modalità operativa: le frecce MQTT sono fisse quando il broker è connesso e lampeggiano soltanto durante la disconnessione; il badge compatto `APP` indica un owner Manager fresco, mentre `LOC` indica fallback/standalone. Il badge non lampeggia.
- Se ENTER arriva prima della quiete, il firmware emette un beep basso e mostra **ACQUISIZIONE PESO** per massimo 1,5 s. Registra appena raggiunge STABLE; al timeout accetta il centro RAW robusto solo senza deriva continua e usa lo stesso centro per peso e zero. Movimento persistente o campioni non validi producono doppio beep e un avviso leggibile; successo e fallimento si chiudono automaticamente dopo circa 1,3 s e 2 s.

Verifica locale:

```sh
c++ -std=c++17 -Wall -Wextra -Werror tests/firmware_mqtt_protocol_harness.cpp -o /tmp/firmware_mqtt_protocol_harness
/tmp/firmware_mqtt_protocol_harness
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/esp32_hx711_serial
```
