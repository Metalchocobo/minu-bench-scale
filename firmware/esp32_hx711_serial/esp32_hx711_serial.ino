/*
  ===============================================================
  Bilancia ESP32 + HX711 — 20 kg / 1 g — Serial Only
  Stati STABLE/UNSTABLE + Dead-band display + Drift bucket
  Versione: 1.0 (solo seriale)
  ===============================================================

  OBIETTIVO
  - Realizzare una lettura stabile a 1 g su cella economica, gestendo:
    1) rumore di misura (medie + finestre temporali)
    2) stabilità del peso (STABLE/UNSTABLE con regole semplici)
    3) deriva lenta nel tempo (zero-tracking vicino allo zero)
    4) ri-ancoraggio “intelligente” quando il peso cambia (spendo la deriva)

  CONCETTI CHIAVE
  - OFFSET_RAW: riferimento a vuoto in “counts” del HX711 (baseline).
  - SCALE: fattore counts-per-gram (lo ricavo con un peso noto).
  - g = (raw - OFFSET_EFF) / SCALE, dove OFFSET_EFF tiene conto del drift bucket.
  - STATO: STABLE quando le variazioni in una finestra lunga (2 s) sono sotto soglia;
           UNSTABLE quando in una finestra breve (0,5 s) la variazione supera soglia.
  - DISPLAY_DEAD_BAND: non aggiorno il numero mostrato se la variazione è entro ±X g.
  - DRIFT BUCKET: accumulo la deriva (vicino a 0 e in STABLE) senza modificare subito il display;
                  quando il peso cambia davvero (UNSTABLE) “spendo” il drift allineando l’offset.

  NOTE
  - Questo sketch parla SOLO su seriale. L’OLED SSD1322 lo aggiungeremo in seguito,
    ma tutta la logica (stati, dead-band, drift) resta identica.

  PIN USATI (ESP32 DevKit):
    HX711.DOUT -> GPIO 25
    HX711.SCK  -> GPIO 5
    Serial     -> 115200
*/

#include <Arduino.h>
#include "HX711.h"
#include <math.h>      // fabsf, isnan
#include <limits.h>    // LONG_MIN

// ===============================================================
// CONFIGURAZIONE UTENTE (tutto qui in alto per facilità di tuning)
// ===============================================================

// Pin HX711 su ESP32
static const int PIN_DOUT = 25;   // HX711 DT (DOUT)
static const int PIN_SCK  = 5;    // HX711 SCK (CLK)

// Target della bilancia
const float G_MAX = 20000.0f;     // 20 kg in grammi (limite sicurezza)

// Campionamento
int   AVG_N             = 4;      // quante letture medie per ogni campione (1..8 tipico)
unsigned long SAMPLE_MS = 70;     // intervallo tra campioni (ms). ~70 ms ≈ ~14 Hz

// Finestra di stabilità
// Regola: se in 0.5 s la variazione supera THRESH_UNSTABLE → UNSTABLE
//         se per 2.0 s la variazione resta sotto THRESH_STABLE → STABLE
float THRESH_UNSTABLE   = 1.0f;   // g su 0.5 s per entrare in UNSTABLE
float THRESH_STABLE     = 1.0f;   // g su 2.0 s per tornare STABLE
unsigned long WIN_FAST_MS = 500;  // 0.5 s
unsigned long WIN_SLOW_MS = 2000; // 2.0 s

// Filtro visualizzazione (dead-band)
// Non aggiorno gDisp se la differenza dal precedente è entro ±dead-band
float DISPLAY_DEAD_BAND = 0.3f;   // g (tipicamente 0.2–0.5)

// Zero-tracking (deriva vicino allo zero)
// Vicino allo zero e in STABLE accumulo drift (in counts) senza toccare il numero mostrato.
// Quando il peso cambia (UNSTABLE), spendo il drift riallineando OFFSET_RAW.
bool  ZT_ENABLE     = true;       // abilita/disabilita zero-tracking
float ZT_NEAR_G     = 0.5f;       // considerato “vicino a zero” entro ±0.5 g
int   ZT_STEP_MIN   = 1;          // minimo passo in counts per integrare drift (evita zapping)

// Auto-tare su boot
bool AUTO_TARE_ON_BOOT = true;    // tara automatica all’avvio
unsigned long BOOT_STAB_WAIT_MS = 400; // attesa breve per stabilizzarsi prima di tarare

// Debug esteso
bool DEBUG_VERBOSE = false;       // se true stampa telemetria ogni campione

// ===============================================================
// VARIABILI DI STATO
// ===============================================================
HX711 scale;

// Calibrazione: y[g] = (raw - OFFSET_RAW - drift_bucket_counts) / SCALE
long  OFFSET_RAW = 0;             // baseline a vuoto (counts)
float SCALE      = 1.0f;          // counts per gram (si ricava con "cal=XXXX")

// Stato STABLE/UNSTABLE
enum State { STABLE, UNSTABLE };
State state = UNSTABLE;

// Ring buffer per finestre (salvo ultimi valori in grammi)
const int RB_FAST_SZ = 32;        // copre ~0.5 s a ~15–20 Hz (dipende da SAMPLE_MS)
const int RB_SLOW_SZ = 64;        // copre ~2 s
float rbFast[RB_FAST_SZ]; int iFast=0, nFast=0;
float rbSlow[RB_SLOW_SZ]; int iSlow=0, nSlow=0;

// Drift bucket: deriva accumulata in counts (ancora “non spesa”)
long drift_bucket_counts = 0;

// Display su seriale
long  gDispPrev   = LONG_MIN;     // ultimo valore intero “mostrato” (retro-compat)
float gPrintPrev  = NAN;          // ultimo valore FLOAT che ha fatto scattare la stampa (per dead-band reale)

// Timing
unsigned long lastSampleMs = 0;

// ===============================================================
// UTILITY DI MISURA
// ===============================================================

// Lettura media “robusta” dal HX711 (blocca finché pronto)
long readRawAvg(int n) {
  long s = 0;
  for (int i = 0; i < n; i++) {
    while (!scale.is_ready()) { delayMicroseconds(200); } // evita busy-wait “puro”
    s += scale.read();
  }
  return s / n;
}

// Inserisce un campione g nelle due finestre
void pushRB(float g) {
  rbFast[iFast] = g; iFast = (iFast + 1) % RB_FAST_SZ; if (nFast < RB_FAST_SZ) nFast++;
  rbSlow[iSlow] = g; iSlow = (iSlow + 1) % RB_SLOW_SZ; if (nSlow < RB_SLOW_SZ) nSlow++;
}

// Ritorna l’escursione (max-min) di una finestra
float rangeRB(const float *buf, int n) {
  if (n <= 0) return 0;
  float mn = buf[0], mx = buf[0];
  for (int i = 1; i < n; i++) {
    float v = buf[i];
    if (v < mn) mn = v;
    if (v > mx) mx = v;
  }
  return mx - mn;
}

// Decide STABLE/UNSTABLE in base alle due finestre
void updateStability() {
  float rngFast = rangeRB(rbFast, nFast); // ~0.5 s
  float rngSlow = rangeRB(rbSlow, nSlow); // ~2 s

  if (state == STABLE) {
    if (rngFast >= THRESH_UNSTABLE) state = UNSTABLE;
  } else {
    // Torno STABLE solo se la finestra lunga è sotto soglia e... la finestra è "piena"
    if (nSlow >= RB_SLOW_SZ && rngSlow < THRESH_STABLE) state = STABLE;
  }
}

// Converte raw → grammi usando OFFSET effettivo (offset + drift accumulato)
float rawToG(long raw) {
  long eff_offset = OFFSET_RAW + drift_bucket_counts;
  return (raw - eff_offset) / SCALE;
}

// Zero-tracking: se vicino a zero e stabile, integro lentamente il drift (in counts)
void zeroTrackIfNearZero(float g, long raw) {
  if (!ZT_ENABLE) return;
  if (state != STABLE) return;
  if (fabs(g) > ZT_NEAR_G) return;

  long target = OFFSET_RAW + drift_bucket_counts; // dove “vorremmo” stare
  long diff = raw - target;
  if (labs(diff) > ZT_STEP_MIN) {
    drift_bucket_counts += (diff > 0 ? 1 : -1); // passo 1 count per ciclo (lento, controllato)
  }
}

// Quando il peso cambia (UNSTABLE), spendo subito il drift accumulato, riallineando l’offset
void spendDriftOnChange() {
  if (state == UNSTABLE && drift_bucket_counts != 0) {
    OFFSET_RAW += drift_bucket_counts;
    drift_bucket_counts = 0;
  }
}

// ===============================================================
// CALIBRAZIONE, TARE, COMANDI
// ===============================================================
void tareNow() {
  OFFSET_RAW = readRawAvg(AVG_N);
  drift_bucket_counts = 0;
}

void calibrateWithKnown(float g_ref) {
  // 1) Misuro a vuoto per fissare OFFSET_RAW
  OFFSET_RAW = readRawAvg(AVG_N);
  delay(50);

  // 2) Chiedo conferma da seriale quando il peso noto è appoggiato
  if (g_ref <= 0.0f) {
    Serial.println(F("[ERR] cal=XXXX: valore non valido (deve essere > 0)"));
    return;
  }

  drift_bucket_counts = 0; // evito di “sporcare” la CAL con drift accumulato
  Serial.println(F("Posa il peso di riferimento e premi INVIO (inizio calibrazione)..."));
  while (Serial.available()) Serial.read();
  while (!Serial.available()) { delay(10); }
  while (Serial.available()) Serial.read(); // svuoto il buffer
  delay(400); // piccolo tempo di assestamento

  // 3) Misuro con il peso e calcolo SCALE (counts per gram)
  long raw1 = readRawAvg(AVG_N);
  long d = raw1 - OFFSET_RAW;
  if (d == 0) d = 1;
  SCALE = (float)d / g_ref;  // counts per grammo

  Serial.print(F("[CAL OK] SCALE=")); Serial.println(SCALE, 6);
}

void printHelp() {
  Serial.println();
  Serial.println(F("Comandi seriale:"));
  Serial.println(F("  t               -> Tare (azzera OFFSET_RAW)"));
  Serial.println(F("  cal=XXXX        -> Calibrazione con peso noto in grammi (es. cal=2000)"));
  Serial.println(F("  cfg?            -> Mostra configurazione attuale"));
  Serial.println(F("  zt=on|off       -> Abilita/Disabilita zero-tracking"));
  Serial.println(F("  dbg=on|off      -> Debug esteso (telemetria ogni campione)"));
  Serial.println(F("  help            -> Questo elenco"));
}

void printCfg() {
  Serial.print(F("OFFSET_RAW=")); Serial.print(OFFSET_RAW);
  Serial.print(F("  SCALE="));     Serial.print(SCALE, 6);
  Serial.print(F("  AVG_N="));     Serial.print(AVG_N);
  Serial.print(F("  SAMPLE_MS=")); Serial.print(SAMPLE_MS);
  Serial.print(F("  DB="));        Serial.print(DISPLAY_DEAD_BAND);
  Serial.print(F("  THR_UN="));    Serial.print(THRESH_UNSTABLE);
  Serial.print(F("  THR_ST="));    Serial.print(THRESH_STABLE);
  Serial.print(F("  ZT="));        Serial.print(ZT_ENABLE ? "ON" : "OFF");
  Serial.println();
}

void handleSerialCmd() {
  static String s;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (s.length()) {
        if (s == "t") {
          tareNow(); Serial.println(F("[OK] Tare eseguita"));
        } else if (s.startsWith("cal=")) {
          float gref = s.substring(4).toFloat();
          calibrateWithKnown(gref); printCfg();
        } else if (s == "cfg?") {
          printCfg();
        } else if (s == "zt=on") {
          ZT_ENABLE = true; Serial.println(F("[OK] Zero-track ON"));
        } else if (s == "zt=off") {
          ZT_ENABLE = false; Serial.println(F("[OK] Zero-track OFF"));
        } else if (s == "dbg=on") {
          DEBUG_VERBOSE = true; Serial.println(F("[OK] Debug verbose ON"));
        } else if (s == "dbg=off") {
          DEBUG_VERBOSE = false; Serial.println(F("[OK] Debug verbose OFF"));
        } else if (s == "help") {
          printHelp();
        } else {
          Serial.println(F("[?] Comando sconosciuto. Digita 'help'"));
        }
      }
      s = "";
    } else {
      s += c;
    }
  }
}

// ===============================================================
// SETUP / LOOP
// ===============================================================
void setup() {
  Serial.begin(115200);
  Serial.println(F("HX711 Serial Scale — 20kg/1g"));
  printHelp();

  // Avvio HX711
  scale.begin(PIN_DOUT, PIN_SCK);
  delay(100);

  // Tare automatica su boot (dopo breve stabilizzazione)
  OFFSET_RAW = readRawAvg(AVG_N);
  if (AUTO_TARE_ON_BOOT) {
    delay(BOOT_STAB_WAIT_MS);
    OFFSET_RAW = readRawAvg(AVG_N);
  }

  printCfg();
}

void loop() {
  handleSerialCmd();

  // cadenza di campionamento
  unsigned long now = millis();
  if (now - lastSampleMs < SAMPLE_MS) return;
  lastSampleMs = now;

  // Lettura (media) e conversione
  long raw = readRawAvg(AVG_N);
  float g   = rawToG(raw);

  // Limiti di sicurezza
  if (g >  G_MAX) g =  G_MAX;
  if (g < -G_MAX) g = -G_MAX;

  // Aggiorno finestre e stato
  pushRB(g);
  updateStability();

  // Zero-track vicino a zero (accumula deriva in bucket, non cambia subito il numero)
  zeroTrackIfNearZero(g, raw);

  // Se il peso si muove (UNSTABLE), spendo il drift subito (riallineo offset)
  spendDriftOnChange();

  // Visualizzazione su seriale con dead-band REALE (su float, non sull'intero arrotondato)
  bool mustPrint = false;
  if (isnan(gPrintPrev)) {
    mustPrint = true;              // prima stampa
  } else if (fabsf(g - gPrintPrev) > DISPLAY_DEAD_BAND) {
    mustPrint = true;              // stampa solo se la differenza reale supera la banda
  }
  if (DEBUG_VERBOSE) mustPrint = true; // debug forza sempre la stampa

  if (mustPrint) {
  long gDisp = lroundf(g);       // arrotondo al grammo solo quando stampo
  gDispPrev  = gDisp;            // mantengo anche la versione “storica” intera
  gPrintPrev = g;                // e salvo il valore FLOAT che ha causato la stampa
    if (DEBUG_VERBOSE) {
      // Telemetria estesa (utile per capire range finestre e deriva)
      float rngFast = rangeRB(rbFast, nFast);
      float rngSlow = rangeRB(rbSlow, nSlow);
      Serial.print(F("raw="));   Serial.print(raw);
      Serial.print(F("  g="));   Serial.print(g, 2);
      Serial.print(F("  gDisp=")); Serial.print(gDisp);
      Serial.print(F("  state=")); Serial.print(state == STABLE ? "STABLE" : "UNSTABLE");
      Serial.print(F("  rng0.5s=")); Serial.print(rngFast, 2);
      Serial.print(F("  rng2s="));   Serial.print(rngSlow, 2);
      Serial.print(F("  off="));     Serial.print(OFFSET_RAW);
      Serial.print(F("  sc="));      Serial.print(SCALE, 6);
      Serial.print(F("  drift="));   Serial.println(drift_bucket_counts);
    } else {
      // Stampa compatta quando non serve telemetria
      Serial.print(F("g="));    Serial.print(gDisp);
      Serial.print(F("  "));    Serial.println(state == STABLE ? "[STABLE]" : "[UNSTABLE]");
    }
  }
}
