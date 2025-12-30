# Wiring (HX711 + OLED + INA219)

**Moduli**: ESP32 DevKit, HX711 (alimentato a 5 V), cella 4 fili.

- Cella → HX711
  - Rosso → E+
  - Nero  → E-
  - Verde → A+
  - Bianco→ A-

- HX711 → ESP32 (logica)
  - VCC → 5V (rail buck)
  - GND → GND comune
  - SCK → GPIO16 (diretto)
  - DT/DOUT → GPIO35 **solo dopo level shift/partitore** (mai 5 V diretto su GPIO)

Level shifting consigliato:

- Modulo bidirezionale 3.3V↔5V: HV=5V, LV=3V3, GND comune
- DOUT lato HV ↔ GPIO35 lato LV
- (opzionale) SCK GPIO16 lato LV ↔ SCK lato HV
