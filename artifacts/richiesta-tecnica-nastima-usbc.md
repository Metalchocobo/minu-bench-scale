# Richiesta tecnica NASTIMA — pronta, non inviata

Bozza facoltativa per chiarimenti al produttore; non è stata inviata. La batteria scelta e il Mini360 sono montati e Andrea riferisce un primo funzionamento positivo: vedere [stato corrente](mini360/README.md). Le domande sotto riguardano condizioni specifiche non coperte da quella prova generale e non tengono aperto il task di cablaggio.

Destinatario: amazon.eu@nastima.com

Oggetto: BK06-LF60-NATC — USB-C charging with connected load and restart after low-battery cutoff

Hello NASTIMA technical support,

I have installed your 6 V / 6 Ah LiFePO4 battery with integrated USB-C charging, SKU BK06-LF60-NATC, sold on Amazon Italy under ASIN B0FDB3VWDF:
https://www.amazon.it/dp/B0FDB3VWDF

The application is a portable electronic scale. The battery terminals supply a Mini360 DC/DC converter set to 5.11 V and the electronics. Initial operation appears normal. The charging connection is a 5 V USB power supply connected to the battery's USB-C charging port; the ESP32 programming USB connection is separate.

Could your technical team please confirm the following for this exact model?

1. May the load remain connected and operating while the battery charges through USB-C, including during long periods with USB power connected? Do charge termination, automatic recharge and all BMS protections operate correctly in this configuration?
2. If the battery reaches its low-voltage protection cutoff and the BMS disconnects the output, will connecting USB power automatically restore terminal voltage with the load still connected? Is output restored immediately or only after the cells reach a recovery threshold? Please specify the threshold, any restart delay, and whether the load must first be disconnected.
3. What continuous load power or current can the system support while charging through USB-C, and are there different limits during recovery from low-voltage cutoff?

The manual specifies the USB input rating but does not describe these operating conditions. A model-specific answer or a circuit/block diagram would help document these limits for the installed application.

Thank you.

---

## Fonti dei riferimenti

- [Prodotto esatto](https://www.amazon.it/dp/B0FDB3VWDF).
- [Manuale del modello BK06-LF60-NATC](https://m.media-amazon.com/images/I/91H3v%2BEZy2L.pdf): riporta amazon.us@nastima.com.
- [Manuale ufficiale NASTIMA europeo di un altro prodotto](https://m.media-amazon.com/images/I/A1FmSmEUd9L.pdf), pagina 2: usato esclusivamente per verificare il contatto amazon.eu@nastima.com, non le caratteristiche della batteria in esame.
- [Contatto alternativo del produttore Melasta](https://melasta.net/pages/contact-us): service01@melasta.net.

Stato della verifica: nessuna specifica pubblica recuperata conferma l'avvio con il carico collegato dopo intervento della protezione da scarica eccessiva. La richiesta non è stata inviata.
