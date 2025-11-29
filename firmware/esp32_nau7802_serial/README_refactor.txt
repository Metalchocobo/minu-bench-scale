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
