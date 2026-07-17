# esp32_hx711_serial — Firmware HX711 (ESP32)

Le fonti canoniche sono mantenute nella root del repository:

- `../../README.md` — comportamento firmware, comandi, MQTT, build e diagnostica.
- `../../docs/WIRING.md` — pinout e cablaggio fisico.

## Riferimento rapido MQTT firmware 1.4.0

- Un `weigh` può includere `command_id` (8–64 caratteri; lettere ASCII, cifre, `-`, `_`; vuoto solo per legacy). Il firmware lo memorizza con UUID, prodotto e `session_id`.
- Dopo l'attivazione di un comando tokenizzato pubblica sul topic `response` un `command_ack` transitorio con `command_id`, `session_id`, `connection_id`, UUID, prodotto e `state=active`. Un replay identico dello stesso ID non riattiva il comando; su reload aggiorna solo la connessione corrente e riemette l'ACK. Un payload business conflittuale viene ignorato senza ACK.
- Il Manager può pubblicare sul topic `command`, QoS 1 e non retained, una `confirm_request` fenced da `request_id`, sessione, connessione, comando, UUID e prodotto. Il loop la esegue tramite lo stesso helper e lo stesso lock TARE di ENTER fisico. Lo stesso request ID non ripete l'azione; una cache RAM mantiene gli ultimi due esiti terminali anche se il relativo ACK si perde. `accepted`/`staged` non avanzano la pagina e `failed`/`rejected` lasciano attivo l'ingrediente corrente.
- Un `clear` cancella un comando tokenizzato solo se coincidono sia `command_id` sia `session_id`. Un comando v1.2 senza ID richiede la stessa sessione; un comando realmente legacy senza sessione accetta il clear senza ID del Manager nuovo.
- Una response durable `confirm` o `skip` conserva il `command_id` originale. L'`undo` usa `undo_of_response_id` e non inventa un nuovo command ID.
- L'outbox durable è immutabile: mentre è pending, `weigh` e `clear` sono ignorati. Solo il `response_ack` con la `response_id` esatta chiude l'outbox.
- Con MQTT connesso ENTER richiede un comando attivo e, se manca, produce solo un warning. Il commit standalone resta disponibile con MQTT disconnesso o WiFi spento.
- Se ENTER arriva prima della quiete, il firmware emette un beep basso e mostra **ACQUISIZIONE PESO** per massimo 1,5 s. Registra appena raggiunge STABLE; al timeout accetta il centro RAW robusto solo senza deriva continua e usa lo stesso centro per peso e zero. Movimento persistente o campioni non validi producono doppio beep e un avviso leggibile; successo e fallimento si chiudono automaticamente dopo circa 1,3 s e 2 s.

Verifica locale:

```sh
c++ -std=c++17 -Wall -Wextra -Werror tests/firmware_mqtt_protocol_harness.cpp -o /tmp/firmware_mqtt_protocol_harness
/tmp/firmware_mqtt_protocol_harness
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/esp32_hx711_serial
```
