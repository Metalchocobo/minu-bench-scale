// =======================================================
// ESP32 + NAU7802 — Stati + ZT + Snap + DriftBucket + Auto-TARE after-stabilization (±16 kg)
//  • Lettura: NAU7802 → Median(3) → MovingAverage(non-bloccante)
//  • Stati STABLE/UNSTABLE (preset rapidi: m fine / m work)
//  • Zero-Tracking (AZT) prudente + Snap-to-zero allo scarico
//  • Drift Bucket: stima creep in STABLE + APPLICAZIONE alla transizione ST→UNST (uscita da sosta)
//  • Auto-TARE al boot SOLO dopo stabilizzazione (range + slope bassi)
//  • Comandi: t, c <g>, p, s, m normal|fine|fine-only|work, st on/off/?,
//             zt on/off/reset/?, dr on/off/reset/?
//
// Nota: DRIFT_APPLY_ON_LEAVE=true = bucket applicato quando lasci STABLE
// =======================================================

#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"
#include <Preferences.h>
#include <math.h>

// ========================= CONFIG (TUTTO QUI) =========================
// [0] Calibrazione seed (dai tuoi log HX). Dopo CAL+SAVE userai NVS.
const long  DEFAULT_REF_RAW  = 268427;   // raw medio con 2000 g
const long  DEFAULT_ZERO_RAW = 54608;    // raw medio a vuoto
const long  DEFAULT_REF_G    = 2000;     // grammi di riferimento
const float DEFAULT_CPG      = float(DEFAULT_REF_RAW - DEFAULT_ZERO_RAW) / float(DEFAULT_REF_G);
// ≈ 107.66 counts/grammo

// [1] Auto-TARE all’avvio
const bool     AUTO_TARE_ON_BOOT   = true;   // auto-tara al boot
const uint16_t AUTO_TARE_SAMPLES   = 25;     // campioni per media zero
const uint16_t AUTO_TARE_SETTLE_MS = 800;    // warm-up prima di valutare la quiete
const bool     AUTO_TARE_GUARD     = false;  // se true, fai auto-tare solo se vicino a 0
const float    AUTO_TARE_MAX_ABS_G = 200.0f; // soglia guard (se attiva)

// [1b] Auto-TARE “dopo stabilizzazione”
const bool     AUTO_TARE_WAIT_STABLE = true; // abilita logica di attesa quiete
const uint32_t AT_WAIT_MAX_MS        = 5000; // timeout massimo
const uint32_t AT_QUIET_MS           = 1200; // deve restare “quiete” per almeno così
const float    AT_RANGE_G            = 0.6f; // range max (g) nella finestra
const float    AT_SLOPE_GPS          = 0.5f; // pendenza max (g/s)

// [2] Limite operativo visuale (±16 kg anche se la cella è 30 kg)
const float MAX_DISPLAY_G = 16000.0f;
const bool  FLAG_OVERLOAD = true;      // stampa flag over=1 se oltre clamp

// [3] Filtri display (modalità)
const int   MA_DEFAULT    = 6;      // normal
const int   MA_FINE       = 4;      // fine
const float DB_UNSTABLE_N = 0.20f;  // deadband in UNSTABLE (normal)
const float DB_UNSTABLE_F = 0.10f;  // deadband in UNSTABLE (fine)

// [4] Stati STABLE/UNSTABLE (soglie robuste)
const bool  ST_ENABLE_DEFAULT = true; // true = stati ON, false = LIVE
float    ST_DELTA_LEAVE_G   = 1.80f;  // range ≥ 1.8 g (win breve) → lascia STABLE
uint32_t ST_TO_UNSTABLE_MS  = 500;    // finestra breve (ms)
float    ST_SLOPE_GPS       = 4.00f;  // |slope| ≥ 4.0 g/s e |g−latch| ≥ ST_FROM_LATCH_G
uint32_t ST_SLOPE_WIN_MS    = 800;    // finestra slope (ms)
float    ST_FROM_LATCH_G    = 2.0f;   // distanza minima dal latch per lasciare STABLE
float    ST_DELTA_ENTER_G   = 0.9f;   // range < 0.9 g (win lunga) → rientra STABLE
uint32_t ST_TO_STABLE_MS    = 2000;   // finestra lunga (ms)

// [5] Zero-Tracking (separato da OFFSET_RAW)
const bool  ZT_ENABLE_DEFAULT   = true;    // abilita AZT/snap di default
// 5a) Near-zero tracking prudente
float    ZT_WINDOW_G         = 1.0f;       // attivo se |g| ≤ 1.0 g
uint32_t ZT_QUIET_MS         = 800;        // serve “quiete” ≥0.8 s
float    ZT_QUIET_RANGE_G    = 0.6f;       // range ≤ 0.6 g
float    ZT_QUIET_SLOPE_GPS  = 0.5f;       // |slope| ≤ 0.5 g/s
uint32_t ZT_PERIOD_MS        = 500;        // passo ogni 0.5 s
float    ZT_STEP_G           = 0.10f;      // 0.10 g per passo
float    ZT_MAX_G            = 4.0f;       // cap ±4 g tra due TARE
// 5b) Snap-to-zero su scarico
const bool  UNLOAD_SNAP_ENABLE   = true;   // ON
float    UNLOAD_CROSS_WIN_G   = 6.0f;      // snap se |g| ≤ 6 g
float    UNLOAD_SLOPE_GPS_NEG= -3.0f;      // pendenza ≤ −3 g/s
uint32_t UNLOAD_SLOPE_WIN_MS = 500;        // finestra slope (ms)
float    UNLOAD_STABLE_RANGE_G= 1.2f;      // range “quiete” per snap
uint32_t UNLOAD_STABLE_MS    = 300;        // ms di quiete
uint32_t UNLOAD_COOLDOWN_MS  = 2000;       // minimo tra due snap
float    UNLOAD_SNAP_MAX_G   = 6.0f;       // correzione massima snap (g)

// [6] Fast rientro STABLE dopo snap (scorciatoia)
const bool     SNAP_FORCE_STABLE      = true;
const uint32_t SNAP_STABLE_QUIET_MS   = 300;
const float    SNAP_STABLE_RANGE_G    = 0.8f;
const uint32_t SNAP_STABLE_TIMEOUT_MS = 1500;

// [7] Drift Bucket (compensazione creep/return-to-zero)
const bool     DRIFT_ENABLE_DEFAULT   = true;   // ON di default
float    DRIFT_MIN_LOAD_G     = 100.0f;         // stima drift solo con latch ≥100 g
uint32_t DRIFT_TAU_MS         = 6000;           // costante di tempo EMA (~6 s)
uint32_t DRIFT_GATE_WIN_MS    = 1200;           // finestra per gating “quiete”
float    DRIFT_RANGE_G        = 0.4f;           // range massimo in gating
float    DRIFT_SLOPE_GPS      = 0.3f;           // |slope| massimo in gating
float    DRIFT_LOCK_BAND_G    = 0.5f;           // |gLive−gLatch| ≤ 0.5 g
float    DRIFT_CAP_G          = 10.0f;          // limite assoluto bucket (±10 g)

// [7b] DOVE applicare il drift bucket
const bool DRIFT_APPLY_ON_LEAVE = true;  // true: applica il bucket alla transizione ST→UNST (uscita da sosta)

// [8] Cadenzamento logica (non il sample rate del NAU)
const uint32_t SAMPLE_MS = 110;  // ~9 Hz logica, NAU a 40 SPS → diversi campioni mediati

// ========================= PIN & OGGETTI =========================
const int I2C_SDA = 21;
const int I2C_SCL = 22;

NAU7802     myScale;
Preferences prefs;

// ========================= CALIBRAZIONE RUNTIME =========================
// g = (rawAvg - (OFFSET_RAW + zero_track_counts)) / SCALE_CPG
long  OFFSET_RAW = 0;                 // offset di base (TARE/CAL)
float SCALE_CPG  = DEFAULT_CPG;       // counts per grammo
long  REF_G      = DEFAULT_REF_G;     // peso noto per CAL

// Zero-Tracking (accumulatore separato in counts)
long  zero_track_counts = 0;
bool  ztEnable = ZT_ENABLE_DEFAULT;

// ========================= FILTRI =========================
long medBuf[3]={0,0,0}; int medIdx=0, medCount=0;
long median3(long a,long b,long c){
  long M=max(a,max(b,c)), m=min(a,min(b,c));
  return a+b+c-M-m;
}
long pushMedian3(long x){
  medBuf[medIdx]=x;
  medIdx=(medIdx+1)%3;
  if(medCount<3) medCount++;
  if(medCount<3){
    long s=0; for(int i=0;i<medCount;i++) s+=medBuf[i];
    return s/(medCount?medCount:1);
  }
  return median3(medBuf[0],medBuf[1],medBuf[2]);
}
void resetMedian(){ medIdx=0; medCount=0; medBuf[0]=medBuf[1]=medBuf[2]=0; }

long maBuf[8]; int maIdx=0, maCount=0, maN=MA_DEFAULT; long maSum=0;
void setMA(int n){
  maN=constrain(n,1,8);
  maIdx=0; maCount=0; maSum=0;
  for(int i=0;i<8;i++) maBuf[i]=0;
}
long pushMA(long x){
  if(maCount==maN) maSum-=maBuf[maIdx];
  else maCount++;
  maBuf[maIdx]=x;
  maSum+=x;
  maIdx++;
  if(maIdx>=maN) maIdx=0;
  return maSum/(maCount?maCount:1);
}

// ========================= STORICO (range/slope) =========================
const int MAX_HIST=64; float gHist[MAX_HIST]; int hIdx=0,hCount=0;
void  pushHist(float g){
  gHist[hIdx]=g;
  hIdx=(hIdx+1)%MAX_HIST;
  if(hCount<MAX_HIST) hCount++;
}
float histAt(int k){
  if(hCount==0) return 0.0f;
  k=constrain(k,0,min(hCount,MAX_HIST)-1);
  int idx=(hIdx-1-k+MAX_HIST)%MAX_HIST;
  return gHist[idx];
}
float rangeLastNSamples(int n){
  n=min(n,hCount); if(n<=0) return 0.0f;
  float mn=histAt(0), mx=mn;
  for(int i=1;i<n;i++){
    float v=histAt(i);
    if(v<mn) mn=v;
    if(v>mx) mx=v;
  }
  return mx-mn;
}
float slopeLastNSamples(int n){
  n=min(n,hCount);
  if(n<2) return 0.0f;
  float last=histAt(0), first=histAt(n-1);
  float dt=((n-1)*(SAMPLE_MS/1000.0f));
  if(dt<=0) return 0.0f;
  return (last-first)/dt;
}

// ========================= STATI & DISPLAY =========================
enum State { STABLE, UNSTABLE };
bool  stEnable = ST_ENABLE_DEFAULT;
State state = UNSTABLE;

float gLive=0.0f, gLatch=0.0f, dispUnstable=0.0f; bool dispInit=false;
float deadbandUnstable = DB_UNSTABLE_N;

unsigned long lastSample=0;
unsigned long lastZTtickMs   = 0;
unsigned long lastSnapMs     = 0;

// Fast rientro STABLE post-snap
bool          postSnapActive = false;
unsigned long postSnapStart  = 0;

// ========================= DRIFT BUCKET (runtime) =========================
bool  driftEnable = DRIFT_ENABLE_DEFAULT;
float driftEstCounts = 0.0f;   // EMA lenta dell'errore raw vs atteso
long  driftBucketCounts = 0;   // bucket clampato (counts) applicato quando esci da STABLE
float driftAlpha = 0.0f;       // coefficiente EMA per DRIFT_TAU_MS

// ========================= FUNZIONI NAU: LETTURA MEDIA =========================

// Lettura NAU bloccante di un singolo sample
long readRawNAUOnceBlocking() {
  unsigned long t0 = millis();
  while (!myScale.available()) {
    if (millis() - t0 > 200) {
      return 0; // timeout di sicurezza
    }
    delay(2);
  }
  return myScale.getReading();
}

// Media di N letture (usata per TARE/CAL/autoTare)
long readRawNAUAvg(int n) {
  if (n <= 1) return readRawNAUOnceBlocking();
  long s = 0;
  for (int i = 0; i < n; i++) {
    s += readRawNAUOnceBlocking();
  }
  return s / n;
}

// ========================= NVS / UTILITY =========================
void printHelp(){
  Serial.println(F("\nComandi:"));
  Serial.println(F("  t              -> TARE"));
  Serial.println(F("  c <g>          -> CAL con peso noto (es: c 2000)"));
  Serial.println(F("  p              -> stampa parametri/stato"));
  Serial.println(F("  s              -> salva OFFSET/SCALE/REF in NVS"));
  Serial.println(F("  m normal|fine|fine-only|work  -> preset/filtri"));
  Serial.println(F("  st on/off/?    -> abilita/disabilita stati (STABLE/UNSTABLE)"));
  Serial.println(F("  zt on/off/reset/? -> zero-tracking (AZT/snap) toggle/reset"));
  Serial.println(F("  dr on/off/reset/? -> drift bucket toggle/reset/info"));
}

void loadFromNVS(){
  prefs.begin("minu_scale", true);
  long  off = prefs.getLong ("offset", 0);
  float sc  = prefs.getFloat("scale",  NAN);
  long  rg  = prefs.getLong ("ref_g",  DEFAULT_REF_G);
  prefs.end();
  SCALE_CPG = (!isnan(sc) && sc>0.01f) ? sc : DEFAULT_CPG;
  REF_G     = rg;
  OFFSET_RAW= off; // placeholder, poi auto-tare se attiva
}

void saveToNVS(){
  prefs.begin("minu_scale", false);
  prefs.putLong ("offset", OFFSET_RAW);
  prefs.putFloat("scale",  SCALE_CPG);
  prefs.putLong ("ref_g",  REF_G);
  prefs.end();
  Serial.println(F("[SAVE] Parametri salvati."));
}

// ========================= ZERO-TRACKING CORE =========================
inline long effectiveOffsetCounts(){ return OFFSET_RAW + zero_track_counts; }

bool isQuietForZT(){
  int nQuiet = constrain((int)ceil((float)ZT_QUIET_MS / (float)SAMPLE_MS), 2, MAX_HIST);
  float rngQ = rangeLastNSamples(nQuiet);
  float slpQ = fabsf(slopeLastNSamples(nQuiet));
  return (rngQ <= ZT_QUIET_RANGE_G) && (slpQ <= ZT_QUIET_SLOPE_GPS);
}

// Passo di correzione verso 0, entro i limiti ±ZT_MAX_G
void applyZTstepTowardZero(float gNow){
  if (SCALE_CPG <= 0.01f) return;
  long cap = lroundf(ZT_MAX_G * SCALE_CPG);
  long stepCounts = lroundf(ZT_STEP_G * SCALE_CPG);
  if (stepCounts < 1) stepCounts = 1;

  if      (gNow >  ZT_STEP_G) zero_track_counts += stepCounts;  // aumenta offset effettivo → g scende
  else if (gNow < -ZT_STEP_G) zero_track_counts -= stepCounts;  // diminuisci offset effettivo → g sale

  if (zero_track_counts >  cap) zero_track_counts =  cap;
  if (zero_track_counts < -cap) zero_track_counts = -cap;
}

// Snap-to-zero su scarico: ritorna true se eseguito
bool trySnapOnUnload(float gNow, float slope_now, bool quietNow){
  if (!UNLOAD_SNAP_ENABLE) return false;
  unsigned long now = millis();
  if (now - lastSnapMs < UNLOAD_COOLDOWN_MS) return false;

  if (slope_now > UNLOAD_SLOPE_GPS_NEG) return false;                 // deve essere ≤ soglia (negativa)
  if (fabsf(gNow) > UNLOAD_CROSS_WIN_G) return false;                 // vicino a 0
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
  Serial.println(F("[ZT] Snap-to-zero eseguito (scarico)."));
  return true;
}

// ========================= TARE / CAL =========================
void doTare(){
  long rawZero = readRawNAUAvg(15);
  OFFSET_RAW = rawZero;
  zero_track_counts=0;
  driftEstCounts=0.0f; driftBucketCounts=0;
  resetMedian(); setMA(maN); hIdx=0; hCount=0; state=UNSTABLE; dispInit=false;
  Serial.print(F("[TARE] OFFSET_RAW=")); Serial.println(OFFSET_RAW);
}

void doCal(long ref_g){
  long rawRef = readRawNAUAvg(15);
  long delta  = rawRef - OFFSET_RAW;
  if (delta == 0) delta = 1;
  SCALE_CPG = (float)delta / (float)ref_g;
  zero_track_counts=0;
  driftEstCounts=0.0f; driftBucketCounts=0;
  resetMedian(); setMA(maN); hIdx=0; hCount=0; state=UNSTABLE; dispInit=false;
  Serial.print(F("[CAL]  SCALE_CPG=")); Serial.println(SCALE_CPG,6);
}

void setMode(const String& mode){
  if(mode.equalsIgnoreCase("normal")){
    setMA(MA_DEFAULT);
    deadbandUnstable=DB_UNSTABLE_N;
    // non tocchiamo ST/ZT/DR qui: restano come sono
    Serial.println(F("[MODE] normal  (MA=6, DB=0.20 g)"));
  }
  else if(mode.equalsIgnoreCase("fine")){
    setMA(MA_FINE);
    deadbandUnstable=DB_UNSTABLE_F;
    // Micro-pesate: niente stati, niente drift, niente zero-tracking automatico
    stEnable    = false;
    driftEnable = false;
    driftEstCounts   = 0.0f;
    driftBucketCounts= 0;
    ztEnable   = false;
    Serial.println(F("[PRESET] micro: m=fine, MA=4, DB=0.10 g, ST=off, ZT=off, DR=off"));
  }
  else {
    Serial.println(F("[MODE] sconosciuta. Usa: m normal | m fine | m fine-only | m work"));
    return;
  }
  dispInit=false;
}

// ========================= AUTO-TARE AFTER-STABILIZATION =========================
void autoTareOnBoot(){
  if(!AUTO_TARE_ON_BOOT){
    Serial.println(F("[BOOT] Auto-TARE disattivata."));
    return;
  }

  Serial.println(F("[BOOT] Auto-TARE after-stabilization..."));
  delay(AUTO_TARE_SETTLE_MS); // piccolo warm-up

  if (!AUTO_TARE_WAIT_STABLE) {
    long rawZero = readRawNAUAvg(AUTO_TARE_SAMPLES);
    OFFSET_RAW = rawZero; zero_track_counts = 0;
    driftEstCounts=0.0f; driftBucketCounts=0;
    resetMedian(); setMA(maN); hIdx=0; hCount=0; state=UNSTABLE; dispInit=false;
    Serial.print(F("[BOOT] Auto-TARE OK (semplice). OFFSET_RAW=")); Serial.println(OFFSET_RAW);
    return;
  }

  // Attesa stabilità (range+slope bassi) entro timeout
  float cpg = (SCALE_CPG > 0.01f) ? SCALE_CPG : DEFAULT_CPG;
  long  rangeCountsMax = lroundf(AT_RANGE_G * cpg);

  int N = (int)ceil((float)AT_QUIET_MS / (float)SAMPLE_MS);
  if (N < 5) N = 5;
  const int MAXN = 80;
  if (N > MAXN) N = MAXN;

  long buf[MAXN]; int n=0, head=0;
  unsigned long t0 = millis();
  bool stabilized = false;

  while (millis() - t0 < AT_WAIT_MAX_MS){
    unsigned long tStart = millis();
    while (!myScale.available() && (millis() - tStart < 30)) {
      delay(1);
    }
    if (!myScale.available()) continue;

    long r = myScale.getReading();
    if (n < N){ buf[n++] = r; } else { buf[head] = r; head = (head+1)%N; }

    if (n >= N){
      long mn = buf[0], mx = buf[0];
      for (int i=1;i<N;i++){ if (buf[i]<mn) mn=buf[i]; if (buf[i]>mx) mx=buf[i]; }
      long rangeCounts = mx - mn;

      long first = buf[(head)%N];
      long last  = buf[(head+N-1)%N];
      float dt   = (float)(N-1) * (SAMPLE_MS/1000.0f);
      float slope_gps = (dt>0.0f && cpg>0.01f) ? ((last - first)/cpg)/dt : 0.0f;

      bool quietRange = (rangeCounts <= rangeCountsMax);
      bool quietSlope = (fabsf(slope_gps) <= AT_SLOPE_GPS);

      if (quietRange && quietSlope){
        stabilized = true;
        break;
      }
    }
  }

  long rawZero = readRawNAUAvg(AUTO_TARE_SAMPLES);
  OFFSET_RAW = rawZero;
  zero_track_counts = 0;
  driftEstCounts = 0.0f;
  driftBucketCounts = 0;

  resetMedian(); setMA(maN); hIdx=0; hCount=0; state=UNSTABLE; dispInit=false;

  if (stabilized){
    Serial.print(F("[BOOT] Auto-TARE OK (stabile). OFFSET_RAW="));
    Serial.println(OFFSET_RAW);
  } else {
    Serial.print(F("[BOOT] Auto-TARE OK (timeout, media robusta). OFFSET_RAW="));
    Serial.println(OFFSET_RAW);
  }
}

// ========================= INIT NAU7802 =========================
bool initNAU7802() {
  Serial.println(F("[NAU] Init NAU7802..."));

  if (!myScale.begin()) {
    Serial.println(F("[NAU] begin() FAIL – modulo non trovato"));
    return false;
  }

  bool ok = true;
  Serial.println(F("[NAU] CONFIG:"));

  // LDO interno 3.3V
  if (myScale.setLDO(NAU7802_LDO_3V3) == false) {
    Serial.println(F("  LDO       = 3.3V (NAU7802_LDO_3V3)  esito: FAIL"));
    ok = false;
  } else {
    Serial.println(F("  LDO       = 3.3V (NAU7802_LDO_3V3)  esito: OK"));
  }

  // Gain 128
  if (myScale.setGain(NAU7802_GAIN_128) == false) {
    Serial.println(F("  Gain      = 128  (NAU7802_GAIN_128) esito: FAIL"));
    ok = false;
  } else {
    Serial.println(F("  Gain      = 128  (NAU7802_GAIN_128) esito: OK"));
  }

  // Sample rate 40 SPS (meno rumore di 80 SPS)
  if (myScale.setSampleRate(NAU7802_SPS_40) == false) {
    Serial.println(F("  SampleRate= 40SPS (NAU7802_SPS_40)  esito: FAIL"));
    ok = false;
  } else {
    Serial.println(F("  SampleRate= 40SPS (NAU7802_SPS_40)  esito: OK"));
  }

  myScale.setChannel(NAU7802_CHANNEL_1);

  if (myScale.calibrateAFE() == false) {
    Serial.println(F("  calibrazione AFE                     esito: FAIL"));
    ok = false;
  } else {
    Serial.println(F("  calibrazione AFE                     esito: OK"));
  }

  uint8_t rev = myScale.getRevisionCode();
  Serial.print(F("[NAU] Revision code = 0x"));
  Serial.println(rev, HEX);

  return ok;
}

// ========================= SETUP =========================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n=== ESP32 + NAU7802 — Stati + ZT + Snap + DriftBucket + AT-after-stabilization (±16 kg) ==="));

  printHelp();

  // I2C su ESP32: SDA=21, SCL=22
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!initNAU7802()) {
    Serial.println(F("[NAU] ERRORE inizializzazione – controlla I2C/alimentazione."));
    // se vuoi bloccare: while(1){ delay(500); }
  }

  // coefficiente EMA per il drift (alpha = 1 - exp(-dt/tau))
  driftAlpha = 1.0f - expf(-(float)SAMPLE_MS / (float)DRIFT_TAU_MS);

  loadFromNVS();
  setMA(MA_DEFAULT);
  Serial.print(F("[LOAD] DEFAULT_CPG=")); Serial.print(DEFAULT_CPG,6);
  Serial.print(F("  SCALE_CPG="));        Serial.print(SCALE_CPG,6);
  Serial.print(F("  REF_G="));            Serial.println(REF_G);

  autoTareOnBoot();
}

// ========================= LOOP =========================
void loop(){
  // --- Comandi ---
  if (Serial.available()){
    String cmd=Serial.readStringUntil('\n'); cmd.trim();

    if (cmd.equalsIgnoreCase("t")) doTare();
    else if (cmd.startsWith("c")||cmd.startsWith("C")){
      long ref=REF_G; int sp=cmd.indexOf(' ');
      if(sp>0){ long v=cmd.substring(sp+1).toInt(); if(v>0) ref=v; }
      REF_G=ref; Serial.print(F("[CAL] Riferimento: ")); Serial.print(ref); Serial.println(F(" g"));
      Serial.println(F("      (Fai TARE, metti il peso, poi questo comando)"));
      doCal(ref);
    }
    else if (cmd.equalsIgnoreCase("p")){
      float zt_g    = (SCALE_CPG>0.01f) ? (float)zero_track_counts / SCALE_CPG : 0.0f;
      float drift_g = (SCALE_CPG>0.01f) ? (float)driftBucketCounts / SCALE_CPG : 0.0f;
      Serial.print(F("[INFO] OFFSET_RAW=")); Serial.print(OFFSET_RAW);
      Serial.print(F("  SCALE_CPG="));       Serial.print(SCALE_CPG,6);
      Serial.print(F("  REF_G="));           Serial.print(REF_G);
      Serial.print(F("  MA="));              Serial.print(maN);
      Serial.print(F("  DB_UNSTABLE="));     Serial.print(deadbandUnstable,2);
      Serial.print(F("  ST="));              Serial.print(stEnable?"on ":"off");
      Serial.print(F("  ZT="));              Serial.print(ztEnable?"on ":"off");
      Serial.print(F("  DR="));              Serial.print(driftEnable?"on ":"off");
      Serial.print(F("  zt_cnt="));          Serial.print(zero_track_counts);
      Serial.print(F("  drift_bucket="));    Serial.print(driftBucketCounts);
      Serial.print(F(" (≈"));                Serial.print(drift_g,2); Serial.println(F(" g)"));
    }
    else if (cmd.equalsIgnoreCase("s")) { saveToNVS(); }

    // ====== PRESET "UN TASTO" ======
    else if (cmd.equalsIgnoreCase("m fine")) {
      setMode("fine");
    }
    else if (cmd.equalsIgnoreCase("m fine-only")) {
      setMA(MA_FINE);
      deadbandUnstable=DB_UNSTABLE_F;
      Serial.println(F("[MODE] fine-only (solo MA/deadband)"));
    }
    else if (cmd.equalsIgnoreCase("m normal")) {
      setMode("normal");
    }
    else if (cmd.equalsIgnoreCase("m work")) {
      setMA(MA_DEFAULT);    // filtro “normal”
      deadbandUnstable=DB_UNSTABLE_N;
      ztEnable    = true;   // ZT on
      stEnable    = true;   // stati ON
      driftEnable = true;   // drift on
      Serial.println(F("[PRESET] work: m=normal, zt=on, st=on, dr=on"));
    }
    // ====== /PRESET ======

    else if (cmd.equalsIgnoreCase("st on"))  { stEnable=true;  Serial.println(F("[ST] on"));  }
    else if (cmd.equalsIgnoreCase("st off")) {
      stEnable=false; Serial.println(F("[ST] off (LIVE)"));
      driftEstCounts=0.0f; driftBucketCounts=0;
    }
    else if (cmd.equalsIgnoreCase("st ?"))   { Serial.print(F("[ST] ")); Serial.println(stEnable?"on":"off"); }

    else if (cmd.equalsIgnoreCase("zt on"))  { ztEnable=true;  Serial.println(F("[ZT] on"));  }
    else if (cmd.equalsIgnoreCase("zt off")) { ztEnable=false; Serial.println(F("[ZT] off")); }
    else if (cmd.equalsIgnoreCase("zt reset")) { zero_track_counts=0; Serial.println(F("[ZT] accumulatore azzerato.")); }
    else if (cmd.equalsIgnoreCase("zt ?")){
      float zt_g = (SCALE_CPG>0.01f) ? (float)zero_track_counts / SCALE_CPG : 0.0f;
      Serial.print(F("[ZT] ")); Serial.print(ztEnable?"on ":"off");
      Serial.print(F("  accum=")); Serial.print(zero_track_counts); Serial.print(F(" counts (≈"));
      Serial.print(zt_g,2); Serial.println(F(" g)"));
    }

    else if (cmd.equalsIgnoreCase("dr on"))  { driftEnable=true;  Serial.println(F("[DRIFT] on"));  }
    else if (cmd.equalsIgnoreCase("dr off")) { driftEnable=false; Serial.println(F("[DRIFT] off")); }
    else if (cmd.equalsIgnoreCase("dr reset")) { driftEstCounts=0.0f; driftBucketCounts=0; Serial.println(F("[DRIFT] bucket azzerato.")); }
    else if (cmd.equalsIgnoreCase("dr ?")) {
      float drift_g = (SCALE_CPG>0.01f) ? (float)driftBucketCounts / SCALE_CPG : 0.0f;
      Serial.print(F("[DRIFT] ")); Serial.print(driftEnable?"on ":"off");
      Serial.print(F("  bucket=")); Serial.print(driftBucketCounts);
      Serial.print(F(" counts (≈")); Serial.print(drift_g,2); Serial.println(F(" g)"));
    }
    else { printHelp(); }
  }

  // --- Cadenzamento letture ---
  unsigned long now=millis();
  if (now - lastSample < SAMPLE_MS) return;
  lastSample += SAMPLE_MS;

  if (!myScale.isConnected()) {
    Serial.println(F("[ERR] NAU non connesso (isConnected=false)."));
    return;
  }

  if (!myScale.available()) {
    // nessun dato pronto, salta questo giro
    return;
  }

  // 1) Lettura grezza NAU
  long raw = myScale.getReading();

  // 2) Filtri
  long rawMed = pushMedian3(raw);
  long rawAvg = pushMA(rawMed);

  // 3) Conversione in grammi (offset effettivo = OFFSET_RAW + zero_track_counts)
  long  offEff = effectiveOffsetCounts();
  gLive = (SCALE_CPG>0.01f) ? (rawAvg - offEff)/SCALE_CPG : 0.0f;

  // 4) Storico per range/slope
  pushHist(gLive);
  int nUnst  = constrain((int)ceil((float)ST_TO_UNSTABLE_MS / (float)SAMPLE_MS), 1, MAX_HIST);
  int nSlope = constrain((int)ceil((float)ST_SLOPE_WIN_MS   / (float)SAMPLE_MS), 2, MAX_HIST);
  int nStab  = constrain((int)ceil((float)ST_TO_STABLE_MS   / (float)SAMPLE_MS), 1, MAX_HIST);
  float rngUnst   = rangeLastNSamples(nUnst);
  float slope_gps = slopeLastNSamples(nSlope);
  float rngStab   = rangeLastNSamples(nStab);

  // 4b) Fast rientro STABLE post-snap (scorciatoia)
  if (postSnapActive && SNAP_FORCE_STABLE) {
    int nQ = constrain((int)ceil((float)SNAP_STABLE_QUIET_MS / (float)SAMPLE_MS), 2, MAX_HIST);
    float rngQ = rangeLastNSamples(nQ);
    if (rngQ <= SNAP_STABLE_RANGE_G) {
      state  = STABLE;
      gLatch = gLive;
      postSnapActive = false;
    } else if (now - postSnapStart > SNAP_STABLE_TIMEOUT_MS) {
      postSnapActive = false; // abbandona fast-path, regole normali
    }
  }

  // 5) State machine + display (qui applichiamo il DRIFT all'uscita da STABLE)
  long gDisp = 0; bool overload=false;
  const char* modeLabel = stEnable ? (state==STABLE?"STABLE":"UNSTABLE") : "LIVE";

  if (!stEnable){
    // LIVE: segue gLive con deadband leggera
    if (!dispInit){ dispUnstable = gLive; dispInit = true; }
    if (fabsf(gLive - dispUnstable) > deadbandUnstable) dispUnstable = gLive;
    gDisp = lroundf(dispUnstable);
  } else {
    // STABLE/UNSTABLE robusti
    if (state==STABLE){
      bool leaveByRange = (rngUnst >= ST_DELTA_LEAVE_G);
      bool leaveBySlope = (fabsf(slope_gps) >= ST_SLOPE_GPS) && (fabsf(gLive - gLatch) >= ST_FROM_LATCH_G);
      bool leaving = leaveByRange || leaveBySlope;

      if (leaving){
        // —— Applicazione DRIFT alla transizione ST→UNST (uscita da sosta) ——
        if (DRIFT_APPLY_ON_LEAVE && driftEnable && (fabsf(gLatch) >= DRIFT_MIN_LOAD_G) && (driftBucketCounts != 0)) {
          zero_track_counts += driftBucketCounts;

          long cap = lroundf(ZT_MAX_G * SCALE_CPG);
          if (zero_track_counts >  cap) zero_track_counts =  cap;
          if (zero_track_counts < -cap) zero_track_counts = -cap;

          Serial.print(F("[DRIFT] applied on ST→UNST "));
          Serial.print(driftBucketCounts); Serial.print(F(" counts (≈"));
          Serial.print( (SCALE_CPG>0.01f)? ((float)driftBucketCounts/SCALE_CPG) : 0.0f , 2);
          Serial.println(F(" g)"));

          driftEstCounts = 0.0f;
          driftBucketCounts = 0;   // evita ri-applicazioni
        }

        state=UNSTABLE;
        dispInit=false;
      }
      gDisp = lroundf(gLatch);
    } else { // UNSTABLE
      if (!dispInit){ dispUnstable = gLive; dispInit=true; }
      if (fabsf(gLive - dispUnstable) > deadbandUnstable) dispUnstable = gLive;
      gDisp = lroundf(dispUnstable);
      if (rngStab < ST_DELTA_ENTER_G){ state=STABLE; gLatch=gLive; gDisp=lroundf(gLatch); }
    }
  }

  // 6) DRIFT BUCKET — stima creep mentre STABLE e “fermo”
  if (driftEnable && stEnable && state==STABLE && fabsf(gLatch) >= DRIFT_MIN_LOAD_G){
    int nGate = constrain((int)ceil((float)DRIFT_GATE_WIN_MS / (float)SAMPLE_MS), 2, MAX_HIST);
    float rngGate  = rangeLastNSamples(nGate);
    float slpGate  = fabsf(slopeLastNSamples(nGate));
    bool lockBand  = (fabsf(gLive - gLatch) <= DRIFT_LOCK_BAND_G);
    if (rngGate <= DRIFT_RANGE_G && slpGate <= DRIFT_SLOPE_GPS && lockBand){
      long rawExpected = offEff + lroundf(SCALE_CPG * gLatch);
      long errCounts   = rawAvg - rawExpected;
      // EMA lenta
      driftEstCounts += driftAlpha * ( (float)errCounts - driftEstCounts );
      // clamp a ±DRIFT_CAP
      long cap = lroundf(DRIFT_CAP_G * SCALE_CPG);
      long est = lroundf(driftEstCounts);
      if (est >  cap) est =  cap;
      if (est < -cap) est = -cap;
      driftBucketCounts = est;
    }
  }

  // 7) Zero-Tracking (AZT + snap) — attivi solo se ztEnable
  if (ztEnable){
    // a) Snap-to-zero su scarico (se abil.)
    if (UNLOAD_SNAP_ENABLE){
      int nSlopeUnload = constrain((int)ceil((float)UNLOAD_SLOPE_WIN_MS / (float)SAMPLE_MS), 2, MAX_HIST);
      float slopeUnload = slopeLastNSamples(nSlopeUnload);
      int nSt = constrain((int)ceil((float)UNLOAD_STABLE_MS / (float)SAMPLE_MS), 2, MAX_HIST);
      float rngSt = rangeLastNSamples(nSt);
      bool quietNow = (rngSt <= UNLOAD_STABLE_RANGE_G);
      bool didSnap = trySnapOnUnload(gLive, slopeUnload, quietNow);

      // >>> Applica DRIFT allo snap SOLO se NON lo hai già applicato in uscita
      if (didSnap && driftEnable && (driftBucketCounts != 0) && !DRIFT_APPLY_ON_LEAVE){
        zero_track_counts += driftBucketCounts;
        long cap = lroundf(ZT_MAX_G * SCALE_CPG);
        if (zero_track_counts >  cap) zero_track_counts =  cap;
        if (zero_track_counts < -cap) zero_track_counts = -cap;

        Serial.print(F("[DRIFT] applied on unload "));
        Serial.print(driftBucketCounts); Serial.print(F(" counts (≈"));
        Serial.print( (SCALE_CPG>0.01f)? ((float)driftBucketCounts/SCALE_CPG) : 0.0f , 2);
        Serial.println(F(" g)"));
        driftEstCounts = 0.0f;
        driftBucketCounts = 0;
      }

      if (didSnap && SNAP_FORCE_STABLE) {
        // azzera lo storico e prepara il rientro rapido
        hIdx = 0; hCount = 0;
        dispInit = false;
        postSnapActive = true;
        postSnapStart  = now;
      }
    }

    // b) AZT lento near-zero (solo se in zona, e in "quiete")
    if (fabsf(gLive) <= ZT_WINDOW_G){
      if (isQuietForZT()){
        if (now - lastZTtickMs >= ZT_PERIOD_MS){
          applyZTstepTowardZero(gLive);
          lastZTtickMs = now;
        }
      }
    }
  }

  // 8) Clamp ±16 kg
  float base = (!stEnable ? dispUnstable : (state==STABLE? gLatch : dispUnstable));
  if (fabsf(base) > MAX_DISPLAY_G){
    overload = FLAG_OVERLOAD;
    if (gDisp >  (long)MAX_DISPLAY_G) gDisp =  (long)MAX_DISPLAY_G;
    if (gDisp < -(long)MAX_DISPLAY_G) gDisp = -(long)MAX_DISPLAY_G;
  }

  // 9) Log
  Serial.print(F("rawAvg="));   Serial.print(rawAvg);
  Serial.print(F("  g="));      Serial.print(gLive,2);
  Serial.print(F("  gDisp="));  Serial.print(gDisp);
  Serial.print(F("  state="));  Serial.print(modeLabel);
  Serial.print(F("  rng0.5s="));Serial.print(rngUnst,2);
  Serial.print(F("  slope="));  Serial.print(slope_gps,2); Serial.print(F(" g/s"));
  Serial.print(F("  rng2s="));  Serial.print(rngStab,2);
  if (FLAG_OVERLOAD){ Serial.print(F("  over=")); Serial.print(overload?1:0); }
  Serial.print(F("  zt_cnt=")); Serial.print(zero_track_counts);
  Serial.print(F("  drift="));  Serial.print(driftBucketCounts);
  Serial.print(F("  off="));    Serial.print(OFFSET_RAW);
  Serial.print(F("  sc="));     Serial.println(SCALE_CPG,4);
}
