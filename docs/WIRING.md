# Wiring (serial only, in attesa OLED)

**Moduli**: ESP32 DevKit, HX711, cella 4 fili.

- Cella → HX711
  - Rosso → E+
  - Nero  → E-
  - Verde → A+
  - Bianco→ A-

- HX711 → ESP32
  - VCC → 3V3
  - GND → GND
  - DT  → GPIO25
  - SCK → GPIO5

> Alimenta HX711 a 3.3 V. GND comune.
