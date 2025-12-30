#include "scale_core.h"

// Qui non riscriviamo la logica della bilancia:
// ci appoggiamo alle funzioni e variabili già definite nello sketch principale.

// Dichiarazioni "extern" verso esp32_hx711_serial.ino
bool initHX711();
void autoTareOnBoot();
void loadFromNVS();
void setMA(int n);
void resetFiltersAndState();

extern int   maN;
extern int   MA_DEFAULT;
extern float deadbandUnstable;
extern float DB_UNSTABLE_N;
extern bool  stEnable;
extern bool  ztEnable;

// Inizializzazione completa del core HX711 + preset di lavoro default
void scale_init() {
  Serial.println(F("[SCALE] Init core bilancia..."));

  if (!initHX711()) {
    Serial.println(F("[SCALE] ERRORE init HX711, controlla cablaggio/alimentazione."));
  }

  // Parametri da NVS (offset/scala/ref)
  loadFromNVS();

  // Preset di default: work/normal
  maN = MA_DEFAULT;
  setMA(maN);
  deadbandUnstable = DB_UNSTABLE_N;
  stEnable = true;
  ztEnable = true;
  resetFiltersAndState();

  // Auto-tare semplice all'avvio (se abilitata)
  autoTareOnBoot();

  Serial.println(F("[SCALE] Init core completato."));
}
