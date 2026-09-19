# Alimentazione USB-C della proposta meccanica

Variante meccanica Waveshare del 7 settembre 2026, non installata: permette di definire un ingombro concreto e rimovibile nel CAD. L'alimentazione corrente usa invece la batteria USB NASTIMA BK06-LF60-NATC LiFePO4 6,4 V / 6 Ah e il Mini360 MP1482DS, con uscita regolata da Andrea a 5,11 V e primo funzionamento riferito positivo. Il vecchio impianto SLA / CTK3S è sostituito; cablaggio corrente e limiti del monitoraggio firmware sono descritti in [WIRING](../../WIRING.md) e nel [README principale](../../../README.md).

## Componenti selezionati

| Quantità | Componente | Acquisto e documentazione | Interfaccia meccanica |
|---:|---|---|---|
| 1 kit | **Waveshare UPS HAT (D), SKU 25507**, versione cinese con 2 batterie 21700, protezione acrilica e viteria | [Waveshare Cina](https://www.waveshare.net/shop/UPS-HAT-D.htm). Contenuto del kit verificato nella pagina; prezzo e spedizione in Italia da confermare. | PCB 85 × 56 mm; ingombro complessivo del modello ufficiale **85 × 62,464 × 38,6 mm**. Batterie e protezione inferiori comprese. |
| alternativa | **Waveshare UPS HAT (D) EN, SKU 25567** | [Waveshare internazionale](https://www.waveshare.com/product/ups-hat-d.htm), USD 24,99 rilevati. **Batterie escluse**: non sommare i due kit nella distinta. | Stesso riferimento meccanico; procurare le due batterie 21700 compatibili dal fornitore. |
| 1 | **Premier PCM-0726 / Adafruit 6069**, prolunga USB-C da pannello, 300 mm | [Adafruit](https://www.adafruit.com/product/6069), oppure [Premier Cable Cina](https://www.premier-cable.store/products/panel-mount-usb-3-1-type-c-waterproof-cable). | Flangia interna, dado e tappo esterni; foro e fissaggi nel CAD. Dettagli in research-controls.md. |
| 1 | **Adafruit 2697**, cavo USB-A → jack, 22 AWG, 1 m, da trasformare in cablaggio interno | [Adafruit](https://www.adafruit.com/product/2697), USD 2,75 rilevati. | Conservare il maschio USB-A, accorciare a circa 200 mm, eliminare il jack e terminare sul distributore 5 V esistente. Lunghezza definitiva secondo percorso, con riserva di manutenzione. Sagoma del connettore e percorso sono inviluppi di progetto. |
| 1 esterno | **Raspberry Pi alimentatore USB-C 15 W, spina EU**, 5,1 V / 3 A | [Produttore e rivenditori](https://www.raspberrypi.com/products/power-supply/), [scheda ufficiale](https://datasheets.raspberrypi.org/power-supply/usb-c-power-supply-product-brief.pdf). | Esterno alla bilancia; il cavo entra nella presa sul guscio. Non inserito tra i pezzi interni da stampare. |

Prezzi osservati, in valute e regimi fiscali diversi: non rappresentano il costo totale consegnato. Nessun acquisto effettuato per questa variante Waveshare. Il kit cinese con batterie evita di dichiarare compatibile un elemento 21700 più lungo del vano nominale: il produttore indica **21 × 70 mm**. Ad esempio, una Samsung 50E da 70,6 mm non è stata approvata come sostituzione.

## Funzione e disposizione

Percorso previsto: alimentatore USB-C esterno → prolunga da pannello → ingresso USB-C UPS → batterie / convertitore → uscita USB-A 5 V → cablaggio verso l'elettronica esistente. Il modulo integra caricatore con gestione simultanea di carica e carico, protezioni, convertitore e misura INA219. Fonte: [manuale Waveshare](https://www.waveshare.com/wiki/UPS_HAT_%28D%29).

Il vano di alimentazione è sul fondo piano, su un vassoio isolante rimovibile. L'ingombro comprende la protezione acrilica e i contatti superiori: non è stato usato il solo spessore del PCB. I cablaggi si fissano al basamento; nessun cavo deve tendersi contro cella o piatto mobile. L'accesso ai connettori e ai comandi del modulo richiede l'apertura del guscio.

Il modello 3D ufficiale è conservato in [sources/waveshare-ups-hat-d/UPS-HAT_D.stp](sources/waveshare-ups-hat-d/UPS-HAT_D.stp), scaricato dal [file ZIP del produttore](https://files.waveshare.com/wiki/UPS-HAT-D/UPS-HAT-D%203D%20Drawing.zip). Il riferimento contiene 297 solidi; limiti originali X −85…0, Y 0…62,464, Z −28,1…10,5 mm. Lo schema è in [sources/waveshare-ups-hat-d-schematic.pdf](sources/waveshare-ups-hat-d-schematic.pdf).

## Verifiche elettriche che il CAD non sostituisce

- Il produttore descrive l'UPS come uscita fino a 2,5 A; la corrente effettiva dipende dallo stato batteria e dal carico. Il progetto deve misurare consumo e picchi di ESP32, OLED e audio prima di fissare autonomia e margine.
- La prolunga PCM-0726 riporta conduttori 24 AWG ma non una corrente continua certificata di 3 A. Verificare il limite del fornitore e la caduta di tensione del percorso completo; la dimensione dei fili da sola non certifica i connettori.
- Il firmware attuale usa tacche indicative per **LiFePO4 2S** e protezioni a **5,80/5,70 V**, incompatibili con il pacco 1S della variante Waveshare. Prima dell'eventuale adozione di questa variante occorre una modifica separata di monitoraggio, soglie batteria, calibrazione corrente e gestione spegnimento. INA219 UPS usa indirizzo 0x43 e il gestore MCU 0x2D secondo manuale; non sono una sostituzione automatica del sensore attuale. Per i limiti delle tacche e dello stato charging con la NASTIMA installata, vedere il [README principale](../../../README.md).
- Terminare il cablaggio 5 V verificando polarità e isolamento. Non alimentare simultaneamente l'ESP32 dalla USB di programmazione e dalla nuova linea VIN senza avere verificato il circuito della specifica DevKit.

Questa è una predisposizione meccanica per la variante Waveshare; la sua catena di ricarica non è stata provata sul banco.
