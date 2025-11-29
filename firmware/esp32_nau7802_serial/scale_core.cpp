#include "scale_core.h"

// Qui non riscriviamo la logica della bilancia:
// ci appoggiamo alle funzioni e variabili già definite nello sketch principale.

// Dichiarazioni "extern" verso esp32_nau7802_serial.ino
bool initNAU7802();
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

// Inizializzazione completa del core NAU + preset di lavoro default
void scale_init() {
  Serial.println(F("[SCALE] Init core bilancia..."));

  if (!initNAU7802()) {
    Serial.println(F("[SCALE] ERRORE init NAU7802, controlla cablaggio/alimentazione."));
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
