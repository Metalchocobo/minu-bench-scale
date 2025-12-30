
// =======================================================
// ESP32 DevKit + HX711 (5 V) + OLED SSD1322 256x64 + INA219
// Versione "light" con stati STABLE/UNSTABLE + ZT + OLED
//  • HX711 alimentato a 5 V (eccitazione cella a 5 V)
//  • DOUT (5 V) verso ESP32 SOLO tramite level shifter / partitore
//  • INA219 su I2C (bus separato dal HX)
//  • OLED SSD1322 su SPI hardware (CLK=18, MOSI=23)
// =======================================================

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>

#include "hx711_driver.h"
#include "battery_monitor.h"
#include "ui_display.h"
#include "keypad.h"
#include "scale_core.h"
#include "buzzer.h"
#include "net_ota_cloud.h"

Preferences prefs;


// ========================= CONFIG =========================

// [0] Calibrazione seed (valori indicativi, sovrascritti da NVS + CAL)
const long  DEFAULT_REF_RAW  = 265290;   // raw medio con 2000 g
const long  DEFAULT_ZERO_RAW = 51471;    // raw medio a vuoto
const long  DEFAULT_REF_G    = 2000;     // grammi di riferimento
const float DEFAULT_CPG      = float(DEFAULT_REF_RAW - DEFAULT_ZERO_RAW) / float(DEFAULT_REF_G);
// ≈ 106.91 counts/grammo

// [1] Auto-TARE all’avvio (semplice, niente logiche complicate)
const bool     AUTO_TARE_ON_BOOT   = true;
const uint16_t AUTO_TARE_SAMPLES   = 25;
const uint16_t AUTO_TARE_SETTLE_MS = 800;

// [2] Limite operativo visuale (±16 kg)
const float MAX_DISPLAY_G = 16000.0f;
const bool  FLAG_OVERLOAD = true; // logga over=1 se clamp

// [3] Filtri display
int   MA_DEFAULT       = 6;      // normal / work
int   MA_FINE          = 4;      // fine / live
float DB_UNSTABLE_N    = 0.10f;  // deadband in UNSTABLE (normal)
float DB_UNSTABLE_F    = 0.05f;  // deadband in UNSTABLE (fine)
float       deadbandUnstable = DB_UNSTABLE_N;

// [4] Stati STABLE/UNSTABLE (semplificati)
const bool  ST_ENABLE_DEFAULT   = true;
float       ST_LEAVE_DELTA_G    = 0.80f;  // se |gLive - gLatch| ≥ 0.80 g → UNSTABLE
float       ST_ENTER_RANGE_G    = 0.60f;  // range < 0.60 g → STABLE
uint32_t    ST_TO_STABLE_MS     = 1200;   // finestra per valutare il rientro (1.2 s)

// [5] Zero-Tracking (solo near-zero + snap allo scarico)
const bool  ZT_ENABLE_DEFAULT   = true;

// 5a) Micro-correzione lenta vicino a 0
float       ZT_WINDOW_G         = 1.5f;     // attivo se |g| ≤ 1.5 g
uint32_t    ZT_QUIET_MS         = 1200;     // serve “quiete” per almeno 1.2 s
float       ZT_QUIET_RANGE_G    = 0.4f;     // range ≤ 0.4 g
float       ZT_QUIET_SLOPE_GPS  = 0.3f;     // |slope| ≤ 0.3 g/s
uint32_t    ZT_PERIOD_MS        = 1000;     // passo ogni 1.0 s
float       ZT_STEP_G           = 0.02f;    // ~0.02 g per passo
float       ZT_MAX_G            = 3.0f;     // cap totale ±3 g

// 5b) Snap-to-zero su scarico
const bool  UNLOAD_SNAP_ENABLE   = true;
float       UNLOAD_CROSS_WIN_G   = 6.0f;    // esegui snap se |g| ≤ 6 g
float       UNLOAD_SLOPE_GPS_NEG = -3.0f;   // pendenza ≤ −3 g/s
uint32_t    UNLOAD_SLOPE_WIN_MS  = 400;     // finestra per slope scarico
float       UNLOAD_STABLE_RANGE_G= 0.8f;    // range “quiete” per snap
uint32_t    UNLOAD_STABLE_MS     = 300;     // ms di quiete
uint32_t    UNLOAD_COOLDOWN_MS   = 2000;    // minimo tra due snap
float       UNLOAD_SNAP_MAX_G    = 5.0f;    // correzione max snap (g)

// [6] Campionamento logica (separato dal rate interno del HX711)
// Nota: HX711 tipicamente 10 SPS o 80 SPS (dipende dal pin RATE sul breakout).
const uint32_t SAMPLE_MS = 60; // ~16 Hz logica + UI

// [OLED] Cadenzamento refresh
const uint32_t OLED_UPDATE_MS = 60;

// ========================= PIN & OGGETTI =========================

// I2C (solo INA219). Manteniamo gli stessi pin usati finora.
const int I2C_SDA = 32;
const int I2C_SCL = 33;

// HX711 (5 V): scegliamo pin "sicuri" per il boot.
// DOUT su GPIO 34/35/36/39 (input-only) evita i pin di strap.
static const int HX_SCK  = 16;
static const int HX_DOUT = 35;



// ========================= CALIBRAZIONE RUNTIME =========================
// g = (rawAvg - (OFFSET_RAW + zero_track_counts)) / SCALE_CPG
long  OFFSET_RAW = 0;
float SCALE_CPG  = DEFAULT_CPG;
long  REF_G      = DEFAULT_REF_G;

// Zero-Tracking accumulatore (solo software)
long  zero_track_counts = 0;
bool  ztEnable = ZT_ENABLE_DEFAULT;


// ========================= FILTRI =========================
long medBuf[3] = {0,0,0};
int  medIdx=0, medCount=0;

long median3(long a,long b,long c){
  long M = max(a, max(b,c));
  long m = min(a, min(b,c));
  return a + b + c - M - m;
}

long pushMedian3(long x){
  medBuf[medIdx] = x;
  medIdx = (medIdx + 1) % 3;
  if (medCount < 3) medCount++;
  if (medCount < 3){
    long s=0;
    for(int i=0;i<medCount;i++) s += medBuf[i];
    return s / (medCount ? medCount : 1);
  }
  return median3(medBuf[0], medBuf[1], medBuf[2]);
}

void resetMedian(){
  medIdx=0; medCount=0;
  medBuf[0]=medBuf[1]=medBuf[2]=0;
}

long maBuf[8];
int  maIdx=0, maCount=0, maN=MA_DEFAULT;
long maSum=0;

void setMA(int n){
  maN = constrain(n,1,8);
  maIdx=0; maCount=0; maSum=0;
  for(int i=0;i<8;i++) maBuf[i]=0;
}

long pushMA(long x){
  if (maCount == maN) {
    maSum -= maBuf[maIdx];
  } else {
    maCount++;
  }
  maBuf[maIdx] = x;
  maSum += x;
  maIdx++;
  if (maIdx >= maN) maIdx = 0;
  return maSum / (maCount ? maCount : 1);
}

// ========================= STORICO (range/slope) =========================
const int MAX_HIST = 64;
float gHist[MAX_HIST];
int   hIdx=0, hCount=0;

void pushHist(float g){
  gHist[hIdx] = g;
  hIdx = (hIdx + 1) % MAX_HIST;
  if (hCount < MAX_HIST) hCount++;
}

float histAt(int k){
  if (hCount == 0) return 0.0f;
  k = constrain(k, 0, min(hCount, MAX_HIST) - 1);
  int idx = (hIdx - 1 - k + MAX_HIST) % MAX_HIST;
  return gHist[idx];
}

float rangeLastNSamples(int n){
  n = min(n, hCount);
  if (n <= 0) return 0.0f;
  float mn = histAt(0), mx = mn;
  for(int i=1;i<n;i++){
    float v = histAt(i);
    if (v < mn) mn = v;
    if (v > mx) mx = v;
  }
  return mx - mn;
}

float slopeLastNSamples(int n){
  n = min(n, hCount);
  if (n < 2) return 0.0f;
  float last  = histAt(0);
  float first = histAt(n-1);
  float dt    = (float)(n-1) * (SAMPLE_MS / 1000.0f);
  if (dt <= 0.0f) return 0.0f;
  return (last - first) / dt; // g/s
}

// ========================= STATI & DISPLAY =========================
enum State { STABLE, UNSTABLE };
bool  stEnable = ST_ENABLE_DEFAULT;
State state    = UNSTABLE;

enum ScaleMode { SCALE_MODE_WORK = 0, SCALE_MODE_LIVE };
ScaleMode currentMode = SCALE_MODE_WORK;


float gLive = 0.0f;
float gLatch = 0.0f;
float dispUnstable = 0.0f;
bool  dispInit = false;

unsigned long lastSampleMs  = 0;
unsigned long lastZTtickMs  = 0;
unsigned long lastSnapMs    = 0;
unsigned long lastOledMs    = 0;

// ========================= UTILITY =========================
inline long effectiveOffsetCounts(){
  return OFFSET_RAW + zero_track_counts;
}

void resetFiltersAndState(){
  resetMedian();
  setMA(maN);
  hIdx = 0;
  hCount = 0;
  state = UNSTABLE;
  dispInit = false;
}

// ========================= NVS / HELP =========================
void printHelp(){
  Serial.println(F("\nComandi:"));
  Serial.println(F("  t              -> TARE"));
  Serial.println(F("  c <g>          -> CAL con peso noto (es: c 2000)"));
  Serial.println(F("  p              -> stampa parametri/stato"));
  Serial.println(F("  s              -> salva OFFSET/SCALE/REF in NVS"));
  Serial.println(F("  m work         -> preset lavoro (normal + stati)"));
  Serial.println(F("  m normal       -> MA=6, deadband 0.10 g, ST on"));
  Serial.println(F("  m fine         -> micro / live: MA=4, deadband 0.05 g, ST off"));
  Serial.println(F("  m live         -> alias di m fine"));
  Serial.println(F("  st on/off/?    -> abilita/disabilita stati STABLE/UNSTABLE"));
  Serial.println(F("  zt on/off/reset/? -> zero-tracking (near-zero + snap)"));
}

void loadFromNVS(){
  prefs.begin("minu_scale", true);
  long  off = prefs.getLong ("offset", 0);
  float sc  = prefs.getFloat("scale",  NAN);
  long  rg  = prefs.getLong ("ref_g",  DEFAULT_REF_G);
  prefs.end();

  SCALE_CPG = (!isnan(sc) && sc>0.01f) ? sc : DEFAULT_CPG;
  REF_G     = rg;
  OFFSET_RAW= (off != 0) ? off : DEFAULT_ZERO_RAW;
}

void saveToNVS(){
  prefs.begin("minu_scale", false);
  prefs.putLong ("offset", OFFSET_RAW);
  prefs.putFloat("scale",  SCALE_CPG);
  prefs.putLong ("ref_g",  REF_G);
  prefs.end();
  Serial.println(F("[SAVE] Parametri salvati in NVS."));
}

// ========================= LETTURA HX711 (blocking per TARE/CAL) =========================
long readRawHXOnceBlocking(){
  return hx711_read_blocking(200);
}

long readRawHXAvg(int n){
  if (n <= 1) return readRawHXOnceBlocking();
  long s=0;
  for(int i=0;i<n;i++){
    s += readRawHXOnceBlocking();
  }
  return s / n;
}

// ========================= ZERO-TRACKING CORE =========================
bool isQuietForZT(){
  int nQuiet = constrain((int)ceil((float)ZT_QUIET_MS / (float)SAMPLE_MS), 2, MAX_HIST);
  float rngQ = rangeLastNSamples(nQuiet);
  float slpQ = fabsf(slopeLastNSamples(nQuiet));
  return (rngQ <= ZT_QUIET_RANGE_G) && (slpQ <= ZT_QUIET_SLOPE_GPS);
}

void applyZTstepTowardZero(float gNow){
  if (SCALE_CPG <= 0.01f) return;

  long stepCounts = lroundf(ZT_STEP_G * SCALE_CPG);
  if (stepCounts < 1) stepCounts = 1;
  long cap = lroundf(ZT_MAX_G * SCALE_CPG);

  if      (gNow >  ZT_STEP_G) zero_track_counts += stepCounts;
  else if (gNow < -ZT_STEP_G) zero_track_counts -= stepCounts;

  if (zero_track_counts >  cap) zero_track_counts =  cap;
  if (zero_track_counts < -cap) zero_track_counts = -cap;
}

// Snap-to-zero su scarico: ritorna true se eseguito
bool trySnapOnUnload(float gNow, float slope_now, bool quietNow){
  if (!UNLOAD_SNAP_ENABLE) return false;

  unsigned long now = millis();
  if (now - lastSnapMs < UNLOAD_COOLDOWN_MS) return false;

  if (slope_now > UNLOAD_SLOPE_GPS_NEG) return false;           // deve essere sufficientemente negativa
  if (fabsf(gNow) > UNLOAD_CROSS_WIN_G) return false;           // vicino allo zero

  if (!quietNow){
    int nSt = constrain((int)ceil((float)UNLOAD_STABLE_MS / (float)SAMPLE_MS), 2, MAX_HIST);
    float rngSt = rangeLastNSamples(nSt);
    if (rngSt > UNLOAD_STABLE_RANGE_G) return false;
  }

  float snap_g = gNow;
  if (snap_g >  UNLOAD_SNAP_MAX_G) snap_g =  UNLOAD_SNAP_MAX_G;
  if (snap_g < -UNLOAD_SNAP_MAX_G) snap_g = -UNLOAD_SNAP_MAX_G;

  long deltaCounts = lroundf(snap_g * SCALE_CPG);
  zero_track_counts += deltaCounts;

  long cap = lroundf(ZT_MAX_G * SCALE_CPG);
  if (zero_track_counts >  cap) zero_track_counts =  cap;
  if (zero_track_counts < -cap) zero_track_counts = -cap;

  lastSnapMs = now;
  Serial.println(F("[ZT] Snap-to-zero (scarico)."));
  return true;
}

// ========================= TARE / CAL / MODALITÀ =========================
void doTare(){
  long rawZero = readRawHXAvg(15);
  OFFSET_RAW = rawZero;
  zero_track_counts = 0;
  resetFiltersAndState();
  Serial.print(F("[TARE] OFFSET_RAW="));
  Serial.println(OFFSET_RAW);
}

void doCal(long ref_g){
  long rawRef = readRawHXAvg(15);
  long delta  = rawRef - OFFSET_RAW;
  if (delta == 0) delta = 1;
  SCALE_CPG = (float)delta / (float)ref_g;
  zero_track_counts = 0;
  resetFiltersAndState();
  Serial.print(F("[CAL]  SCALE_CPG="));
  Serial.println(SCALE_CPG,6);
}

void setMode(const String& mode){
  if (mode.equalsIgnoreCase("work") || mode.equalsIgnoreCase("normal")){
    maN = MA_DEFAULT;
    setMA(maN);
    deadbandUnstable = DB_UNSTABLE_N;
    stEnable = true;
    ztEnable = true;
    resetFiltersAndState();
    currentMode = SCALE_MODE_WORK;
    Serial.println(F("[MODE] work/normal: MA=6, DB=0.10 g, ST=on, ZT=on"));
  }
  else if (mode.equalsIgnoreCase("fine") || mode.equalsIgnoreCase("live")){
    maN = MA_FINE;
    setMA(maN);
    deadbandUnstable = DB_UNSTABLE_F;
    stEnable = false;   // live
    ztEnable = true;
    resetFiltersAndState();
    currentMode = SCALE_MODE_LIVE;
    Serial.println(F("[MODE] fine/live: MA=4, DB=0.05 g, ST=off, ZT=on"));
  }
  else {
    Serial.println(F("[MODE] sconosciuta. Usa: m work | m normal | m fine | m live"));
  }
}


void toggleModeFromKeypad(){
  if (currentMode == SCALE_MODE_WORK){
    // Passa a modalità LIVE (fine)
    setMode("live");
  } else {
    // Torna a modalità WORK (normal)
    setMode("work");
  }
}

// Gestione degli eventi tastiera frontale
void handleKeyEvent(KeyCode key){
  switch (key){
    case KEY_TARE:
      Serial.println(F("[KEYPAD] TARE pressed"));
      doTare();
      buzzerKeyClick();
      break;

    case KEY_MODE:
      Serial.println(F("[KEYPAD] MODE pressed"));
      toggleModeFromKeypad();
      buzzerOk();
      break;

    // Tasti ancora non utilizzati, lasciati intenzionalmente liberi:
    case KEY_ENTER:
      Serial.println(F("[KEYPAD] ENTER pressed"));
      break;

    case KEY_ZERO:
      Serial.println(F("[KEYPAD] ZERO pressed"));
      break;

    case KEY_UP:
      Serial.println(F("[KEYPAD] UP pressed"));
      break;

    case KEY_UNIT:
      Serial.println(F("[KEYPAD] UNIT pressed"));
      break;

    case KEY_SET:
      Serial.println(F("[KEYPAD] SET pressed"));
      break;

    case KEY_CALI:
      Serial.println(F("[KEYPAD] CALI pressed"));
      break;

    case KEY_NONE:
    default:
      break;
  }
}


// ========================= AUTO-TARE ALL’AVVIO =========================
void autoTareOnBoot(){
  if (!AUTO_TARE_ON_BOOT){
    Serial.println(F("[BOOT] Auto-TARE disattivata."));
    return;
  }

  Serial.println(F("[BOOT] Auto-TARE semplice..."));
  delay(AUTO_TARE_SETTLE_MS);

  long rawZero = readRawHXAvg(AUTO_TARE_SAMPLES);
  OFFSET_RAW = rawZero;
  zero_track_counts = 0;
  resetFiltersAndState();

  Serial.print(F("[BOOT] Auto-TARE OK. OFFSET_RAW="));
  Serial.println(OFFSET_RAW);
}

// ========================= INIT HX711 =========================
bool initHX711(){
  Serial.println(F("[HX711] Init HX711..."));

  hx711_begin(HX_DOUT, HX_SCK);
  hx711_set_gain(HX711_GAIN_128_A);

  // Attendo che DOUT vada LOW (dato pronto) per verificare cablaggio.
  if (!hx711_wait_ready(800)){
    Serial.println(F("[HX711] FAIL: nessun dato (DOUT non va LOW). Controlla cablaggio/level-shifter."));
    return false;
  }

  long first = hx711_read();
  Serial.print(F("[HX711] OK. first_raw="));
  Serial.println(first);
  return true;
}


// ========================= SETUP =========================
void setup(){
  Serial.begin(115200);
  delay(200);
  buzzerInit();
  
  Serial.println(F("\n=== ESP32 + HX711 — versione light (5 V) + OLED SSD1322 ==="));

  printHelp();

#if ENABLE_WIFI_OTA
  Net::wifiSetup();
  Net::otaSetup("minu-bench-scale");
#endif

#if ENABLE_ARDUINO_CLOUD
  Net::cloudSetup();
#endif

  // I2C per INA219
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  // Inizializza il core della bilancia (HX711 + preset + auto-tare)
  scale_init();

  // UI / OLED
  ui_init();
  
  ui_showBoot();
  buzzerBootMelody();
  //delay(3000);

  lastOledMs = millis();
  
  battery_init();
  keypad_init();

}

//debug batteria
static uint32_t lastBattDebug = 0;
// ========================= LOOP =========================
void loop(){
#if ENABLE_WIFI_OTA || ENABLE_ARDUINO_CLOUD
  Net::update();
#endif

  // --- Comandi seriale ---
  if (Serial.available()){
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.equalsIgnoreCase("t")) {
      doTare();
    }
    else if (cmd.startsWith("c") || cmd.startsWith("C")){
      long ref = REF_G;
      int sp = cmd.indexOf(' ');
      if (sp > 0){
        long v = cmd.substring(sp+1).toInt();
        if (v > 0) ref = v;
      }
      REF_G = ref;
      Serial.print(F("[CAL] Riferimento: "));
      Serial.print(ref);
      Serial.println(F(" g (fai TARE, metti il peso noto, poi c)"));
      doCal(ref);
    }
    else if (cmd.equalsIgnoreCase("p")){
      float zt_g = (SCALE_CPG>0.01f) ? (float)zero_track_counts / SCALE_CPG : 0.0f;
      Serial.print(F("[INFO] OFFSET_RAW=")); Serial.print(OFFSET_RAW);
      Serial.print(F("  SCALE_CPG="));       Serial.print(SCALE_CPG,6);
      Serial.print(F("  REF_G="));           Serial.print(REF_G);
      Serial.print(F("  MA="));              Serial.print(maN);
      Serial.print(F("  DB_UNSTABLE="));     Serial.print(deadbandUnstable,2);
      Serial.print(F("  ST="));              Serial.print(stEnable?"on ":"off");
      Serial.print(F("  ZT="));              Serial.print(ztEnable?"on ":"off");
      Serial.print(F("  zt_cnt="));          Serial.print(zero_track_counts);
      Serial.print(F(" (≈"));                Serial.print(zt_g,2); Serial.println(F(" g)"));
    }
    else if (cmd.equalsIgnoreCase("s")){
      saveToNVS();
    }
    else if (cmd.startsWith("m ")){
      String m = cmd.substring(2);
      m.trim();
      setMode(m);
    }
    else if (cmd.equalsIgnoreCase("st on")){
      stEnable = true;
      resetFiltersAndState();
      Serial.println(F("[ST] on"));
    }
    else if (cmd.equalsIgnoreCase("st off")){
      stEnable = false;
      resetFiltersAndState();
      Serial.println(F("[ST] off (LIVE)"));
    }
    else if (cmd.equalsIgnoreCase("st ?")){
      Serial.print(F("[ST] ")); Serial.println(stEnable?"on":"off");
    }
    else if (cmd.equalsIgnoreCase("zt on")){
      ztEnable = true;
      Serial.println(F("[ZT] on"));
    }
    else if (cmd.equalsIgnoreCase("zt off")){
      ztEnable = false;
      Serial.println(F("[ZT] off"));
    }
    else if (cmd.equalsIgnoreCase("zt reset")){
      zero_track_counts = 0;
      Serial.println(F("[ZT] accumulatore azzerato."));
    }
    else if (cmd.equalsIgnoreCase("zt ?")){
      float zt_g = (SCALE_CPG>0.01f) ? (float)zero_track_counts / SCALE_CPG : 0.0f;
      Serial.print(F("[ZT] ")); Serial.print(ztEnable?"on ":"off");
      Serial.print(F("  accum=")); Serial.print(zero_track_counts);
      Serial.print(F(" counts (≈")); Serial.print(zt_g,2); Serial.println(F(" g)"));
    }
    else {
      printHelp();
    }
  }

  // --- Cadenzamento letture + tastiera ---
  unsigned long now = millis();

  // Tastiera frontale
  keypad_update(now);
  KeyCode key = keypad_get_event();
  if (key != KEY_NONE){
    handleKeyEvent(key);
  }

  if (now - lastSampleMs < SAMPLE_MS) {
    // comunque aggiorno OLED se è ora
    if (now - lastOledMs >= OLED_UPDATE_MS) {
      // niente nuovo dato, ma puoi tenerlo aggiornato con l'ultimo gDisp
      // (non faccio nulla qui: gDisp verrà aggiornato nel ramo principale)
    }
    return;
  }

  // Attendo che l'HX711 abbia un nuovo dato pronto. Importante: non avanzare il "timebase"
  // se il dato non è pronto, altrimenti le finestre temporali (range/slope/stability) si sballano.
  if (!hx711_is_ready()){
    return;
  }

  lastSampleMs = now;

  
  //aggiorno stato batteria
  battery_update(now);

   // ogni 3 secondi stampa lo stato batteria
    if (now - lastBattDebug > 3000) {
        lastBattDebug = now;
        BatteryStatus st = battery_get_status();
        battery_debug_print(st);
    }

  // 1) Lettura grezza HX711
  long raw = hx711_read();

  // 2) Filtri (mediana + MA)
  long rawMed = pushMedian3(raw);
  long rawAvg = pushMA(rawMed);

  // 3) Conversione in grammi
  long  offEff = effectiveOffsetCounts();
  gLive = (SCALE_CPG > 0.01f) ? (float)(rawAvg - offEff) / SCALE_CPG : 0.0f;

  // 4) Storico (per range/slope)
  pushHist(gLive);

  int n05    = constrain((int)ceil(500.0f  / (float)SAMPLE_MS), 1, MAX_HIST);
  int n2s    = constrain((int)ceil(2000.0f / (float)SAMPLE_MS), 1, MAX_HIST);
  int nSlope = constrain((int)ceil(800.0f  / (float)SAMPLE_MS), 2, MAX_HIST);

  float rng05      = rangeLastNSamples(n05);
  float rng2s      = rangeLastNSamples(n2s);
  float slope_gps  = slopeLastNSamples(nSlope);

  // 5) State machine STABLE/UNSTABLE + display numerico
  long  gDisp = 0;
  bool  overload = false;
  const char* modeLabel = stEnable ? (state == STABLE ? "STABLE" : "UNSTABLE") : "LIVE";

  if (!stEnable){
    // LIVE: segue gLive con deadband leggera
    if (!dispInit){
      dispUnstable = gLive;
      dispInit = true;
    }
    if (fabsf(gLive - dispUnstable) > deadbandUnstable){
      dispUnstable = gLive;
    }
    gDisp = lroundf(dispUnstable);
  } else {
    if (state == STABLE){
      // lascia STABLE se ti discosti dal latch
      if (fabsf(gLive - gLatch) >= ST_LEAVE_DELTA_G){
        state = UNSTABLE;
        dispInit = false;
      }
      gDisp = lroundf(gLatch);
    } else {
      // UNSTABLE: approccia il live con deadband
      if (!dispInit){
        dispUnstable = gLive;
        dispInit = true;
      }
      if (fabsf(gLive - dispUnstable) > deadbandUnstable){
        dispUnstable = gLive;
      }
      gDisp = lroundf(dispUnstable);

      // rientro STABLE se il range in finestra è piccolo
      int   nStab   = constrain((int)ceil((float)ST_TO_STABLE_MS / (float)SAMPLE_MS), 2, MAX_HIST);
      float rngStab = rangeLastNSamples(nStab);
      if (rngStab < ST_ENTER_RANGE_G){
        state  = STABLE;
        gLatch = gLive;
        gDisp  = lroundf(gLatch);
      }
    }
  }

  // 6) Zero-Tracking (snap + micro) se attivo
  if (ztEnable){
    // a) Snap-to-zero su scarico
    if (UNLOAD_SNAP_ENABLE){
      int   nSlopeUnload = constrain((int)ceil((float)UNLOAD_SLOPE_WIN_MS / (float)SAMPLE_MS), 2, MAX_HIST);
      float slopeUnload  = slopeLastNSamples(nSlopeUnload);
      int   nSt          = constrain((int)ceil((float)UNLOAD_STABLE_MS / (float)SAMPLE_MS), 2, MAX_HIST);
      float rngSt        = rangeLastNSamples(nSt);
      bool  quietNow     = (rngSt <= UNLOAD_STABLE_RANGE_G);
      (void)trySnapOnUnload(gLive, slopeUnload, quietNow);
    }

    // b) Micro-ZT lento vicino a 0
    if (fabsf(gLive) <= ZT_WINDOW_G && isQuietForZT()){
      if (now - lastZTtickMs >= ZT_PERIOD_MS){
        applyZTstepTowardZero(gLive);
        lastZTtickMs = now;
      }
    }
  }

  // 7) Clamp ±16 kg
  float base = (!stEnable ? dispUnstable : (state==STABLE ? gLatch : dispUnstable));
  if (fabsf(base) > MAX_DISPLAY_G){
    overload = FLAG_OVERLOAD;
    if (gDisp >  (long)MAX_DISPLAY_G) gDisp =  (long)MAX_DISPLAY_G;
    if (gDisp < -(long)MAX_DISPLAY_G) gDisp = -(long)MAX_DISPLAY_G;
  }

  // 8) Log
  Serial.print(F("rawAvg="));   Serial.print(rawAvg);
  Serial.print(F("  g="));      Serial.print(gLive,2);
  Serial.print(F("  gDisp="));  Serial.print(gDisp);
  Serial.print(F("  state="));  Serial.print(modeLabel);
  Serial.print(F("  rng0.5s="));Serial.print(rng05,2);
  Serial.print(F("  slope="));  Serial.print(slope_gps,2); Serial.print(F(" g/s"));
  Serial.print(F("  rng2s="));  Serial.print(rng2s,2);
  Serial.print(F("  over="));   Serial.print(overload ? 1 : 0);
  Serial.print(F("  zt_cnt=")); Serial.print(zero_track_counts);
  Serial.print(F("  off="));    Serial.print(OFFSET_RAW);
  Serial.print(F("  sc="));     Serial.println(SCALE_CPG,4);

  // 9) OLED update cadenzato
  if (now - lastOledMs >= OLED_UPDATE_MS){
    lastOledMs = now;
    ui_renderWeight(gDisp, modeLabel);
  }

}
