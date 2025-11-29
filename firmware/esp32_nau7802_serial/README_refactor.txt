Refactoring bilancia ESP32 + NAU7802 + OLED SSD1322
===================================================

File principali:

- esp32_nau7802_serial_refactored.ino
    Sketch principale: logica NAU7802, filtri, state machine STABLE/UNSTABLE,
    comandi seriali, NVS, integrazione con UI e battery monitor.

- ui_display.h / ui_display.cpp
    Tutta la gestione dell'OLED SSD1322:
      * inizializzazione SPI + U8G2 (ui_init)
      * schermata di boot con logo Minù (ui_showBoot)
      * layout principale del peso (ui_renderWeight)
      * icone batteria / rete / target
      * formattazione numeri (es. 12.250 g)
      * lettura BatteryStatus per livelli e stato di carica.

- battery_monitor.h / battery_monitor.cpp
    Modulo dedicato al CJMCU-219 (INA219):
      * battery_init()
      * battery_update(nowMs)
      * battery_get_status()
      * battery_debug_print()

- keypad.h / keypad.cpp
    Gestione del tastierino frontale 4x2:
      * scansione matrice 4x2 con debounce
      * generazione di eventi (KEY_TARE, KEY_MODE, ecc.)
      * integrazione nel loop principale tramite handleKeyEvent()

Mappatura pin ESP32 DevKit
--------------------------

I2C (NAU7802 + INA219 CJMCU-219)
  - SDA = GPIO32  (etichetta scheda: D27)
  - SCL = GPIO33  (etichetta scheda: D26)

OLED SSD1322 256x64 (SPI)
  - CLK  = GPIO18 (D18, SPI HW)
  - MOSI = GPIO23 (D23, SPI HW)
  - CS   = GPIO25
  - DC   = GPIO26
  - RST  = GPIO27

Tastierino frontale 4x2 (8 tasti)
  Fisicamente: TARE, ENTER, ZERO, UP, UNIT, SET, CALI, MODE

  Collegamento fili -> GPIO:

  - Filo 1 (R1) = GPIO4
  - Filo 2 (R2) = GPIO5
  - Filo 3 (R3) = GPIO13
  - Filo 4 (R4) = GPIO14
  - Filo 5 (C1) = GPIO19
  - Filo 6 (C2) = GPIO21

  Mappa tasti (R = riga, C = colonna):

    R1-C1 (filo1+filo5) -> TARE
    R1-C2 (filo1+filo6) -> ENTER
    R2-C1 (filo2+filo5) -> ZERO
    R2-C2 (filo2+filo6) -> UP
    R3-C1 (filo3+filo5) -> UNIT
    R3-C2 (filo3+filo6) -> SET
    R4-C1 (filo4+filo5) -> CALI
    R4-C2 (filo4+filo6) -> MODE

Comportamento attuale dei tasti
-------------------------------

- TARE:
    Esegue la tara, usando la stessa logica del comando seriale "t".
- MODE:
    Alterna tra:
      * modalità WORK / normal (setMode("work"))
      * modalità FINE / live (setMode("live"))
    mantenendo allineati MA, deadband e stato ST (ST on in WORK, ST off in LIVE).

- ENTER, ZERO, UP, UNIT, SET, CALI:
    Già mappati e letti dal firmware, ma al momento non eseguono azioni:
    sono lasciati liberi per menu, calibrazioni o altre funzioni future.
