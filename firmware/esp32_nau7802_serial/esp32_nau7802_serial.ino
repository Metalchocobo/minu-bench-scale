
// =======================================================
// ESP32 DevKit + NAU7802 (5 V) + OLED SSD1322 256x64
// Versione "light" con stati STABLE/UNSTABLE + ZT + OLED
//  • NAU7802 su I2C (D27 = SDA, D26 = SCL)
//  • OLED SSD1322 su SPI hardware (CLK=18, MOSI=23)
//    CS=D21, DC=D22, RST=D19  (tutti lato basso destro)
// =======================================================

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <U8g2lib.h>
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"
#include <Preferences.h>
#include <math.h>
#include "battery_monitor.h"

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
const int   MA_DEFAULT       = 6;      // normal / work
const int   MA_FINE          = 4;      // fine / live
const float DB_UNSTABLE_N    = 0.10f;  // deadband in UNSTABLE (normal)
const float DB_UNSTABLE_F    = 0.05f;  // deadband in UNSTABLE (fine)
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

// [6] Campionamento logica (non il sample rate del NAU)
const uint32_t SAMPLE_MS = 60; // ~16 Hz logica, NAU a 40 SPS

// [OLED] Cadenzamento refresh
const uint32_t OLED_UPDATE_MS = 60;

// ========================= PIN & OGGETTI =========================

// NAU7802: I2C spostato su D27 (SDA) e D26 (SCL) per averli vicini
const int I2C_SDA = 32;  // etichetta scheda: D27 (GPIO27)
const int I2C_SCL = 33;  // etichetta scheda: D26 (GPIO26)

// OLED SSD1322 su SPI hardware (lato basso destro)
//   CLK  = GPIO18  (pin D18, fisso SPI HW)
//   MOSI = GPIO23  (pin D23, fisso SPI HW)
static const int OLED_CS  = 25;  // pin D21 (GPIO21)
static const int OLED_DC  = 26;  // pin D22 (GPIO22)
static const int OLED_RST = 27;  


NAU7802     myScale;
Preferences prefs;

// U8g2: SSD1322 256x64, 4-wire SPI hardware, full buffer
// Assicurati che in u8g2.h sia attivo U8G2_16BIT per 256x64.
U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI oled(U8G2_R0,
                                         /* cs=*/ OLED_CS,
                                         /* dc=*/ OLED_DC,
                                         /* reset=*/ OLED_RST);

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

// ========================= LETTURA NAU (blocking per TARE/CAL) =========================
long readRawNAUOnceBlocking(){
  unsigned long t0 = millis();
  while (!myScale.available()){
    if (millis() - t0 > 200) {
      return 0; // timeout
    }
    delay(2);
  }
  return myScale.getReading();
}

long readRawNAUAvg(int n){
  if (n <= 1) return readRawNAUOnceBlocking();
  long s=0;
  for(int i=0;i<n;i++){
    s += readRawNAUOnceBlocking();
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
  long rawZero = readRawNAUAvg(15);
  OFFSET_RAW = rawZero;
  zero_track_counts = 0;
  resetFiltersAndState();
  Serial.print(F("[TARE] OFFSET_RAW="));
  Serial.println(OFFSET_RAW);
}

void doCal(long ref_g){
  long rawRef = readRawNAUAvg(15);
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
    Serial.println(F("[MODE] work/normal: MA=6, DB=0.10 g, ST=on, ZT=on"));
  }
  else if (mode.equalsIgnoreCase("fine") || mode.equalsIgnoreCase("live")){
    maN = MA_FINE;
    setMA(maN);
    deadbandUnstable = DB_UNSTABLE_F;
    stEnable = false;   // live
    ztEnable = true;
    resetFiltersAndState();
    Serial.println(F("[MODE] fine/live: MA=4, DB=0.05 g, ST=off, ZT=on"));
  }
  else {
    Serial.println(F("[MODE] sconosciuta. Usa: m work | m normal | m fine | m live"));
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

  long rawZero = readRawNAUAvg(AUTO_TARE_SAMPLES);
  OFFSET_RAW = rawZero;
  zero_track_counts = 0;
  resetFiltersAndState();

  Serial.print(F("[BOOT] Auto-TARE OK. OFFSET_RAW="));
  Serial.println(OFFSET_RAW);
}

// ========================= INIT NAU7802 =========================
bool initNAU7802(){
  Serial.println(F("[NAU] Init NAU7802..."));

  if (!myScale.begin(Wire)){
    Serial.println(F("[NAU] begin() FAIL – modulo non trovato"));
    return false;
  }

  bool ok = true;

  if (!myScale.setLDO(NAU7802_LDO_3V0)){
    Serial.println(F("  LDO       = 3.0V (NAU7802_LDO_3V0)  esito: FAIL"));
    ok = false;
  } else {
    Serial.println(F("  LDO       = 3.0V (NAU7802_LDO_3V0)  esito: OK"));
  }

  if (!myScale.setGain(NAU7802_GAIN_128)){
    Serial.println(F("  Gain      = 128  (NAU7802_GAIN_128) esito: FAIL"));
    ok = false;
  } else {
    Serial.println(F("  Gain      = 128  (NAU7802_GAIN_128) esito: OK"));
  }

  if (!myScale.setSampleRate(NAU7802_SPS_40)){
    Serial.println(F("  SampleRate= 40SPS (NAU7802_SPS_40)  esito: FAIL"));
    ok = false;
  } else {
    Serial.println(F("  SampleRate= 40SPS (NAU7802_SPS_40)  esito: OK"));
  }

  myScale.setChannel(NAU7802_CHANNEL_1);

  if (!myScale.calibrateAFE()){
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
// ------------------------------------------------------
// Helpers per testo centrato
// ------------------------------------------------------
void drawCenteredText(const char *str, int16_t y) {
  int16_t w = oled.getStrWidth(str);
  int16_t x = (256 - w) / 2;
  oled.drawStr(x, y, str);
}
void drawCenteredTextUTF8(const char *str, int16_t y) {
  int16_t w = oled.getUTF8Width(str);   // larghezza testo UTF-8
  int16_t x = (256 - w) / 2;           // 256 = larghezza display
  oled.drawUTF8(x, y, str);
}

#define MINU_LOGO_WIDTH  56
#define MINU_LOGO_HEIGHT 56

static const unsigned char MINU_LOGO_bits[] U8X8_PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0xE0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xF3,
  0x03, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x03, 0x0E, 0x00, 0x00, 0x00, 0x00,
  0xFE, 0x01, 0x30, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x01, 0xC0, 0x00, 0x00,
  0x00, 0x00, 0xF8, 0x01, 0x80, 0x01, 0x00, 0x00, 0xC0, 0x80, 0x01, 0x00,
  0x02, 0x00, 0x00, 0x20, 0x00, 0x01, 0x00, 0x04, 0x00, 0x00, 0x30, 0x00,
  0x02, 0x00, 0x08, 0x00, 0x00, 0x18, 0x00, 0x04, 0x00, 0x18, 0x00, 0x00,
  0x08, 0x00, 0x08, 0x02, 0x10, 0x00, 0x00, 0x04, 0x00, 0x88, 0x07, 0x20,
  0x00, 0x00, 0x04, 0x00, 0xD0, 0x04, 0x20, 0x00, 0x00, 0x02, 0xC6, 0x61,
  0x04, 0x40, 0x00, 0x00, 0x02, 0x67, 0x21, 0x06, 0x40, 0x00, 0x00, 0xC0,
  0x24, 0x11, 0x02, 0x00, 0x00, 0x00, 0x38, 0x16, 0x09, 0x03, 0x00, 0x00,
  0xFC, 0x07, 0x92, 0x05, 0x01, 0x00, 0x00, 0x00, 0x00, 0x8A, 0x84, 0x05,
  0x18, 0x00, 0x00, 0x00, 0xC7, 0x82, 0x04, 0x20, 0x00, 0x00, 0x00, 0xC5,
  0xC1, 0x72, 0x02, 0x00, 0x00, 0x01, 0xC3, 0x41, 0x72, 0x0A, 0x1E, 0x00,
  0x81, 0xE3, 0x40, 0x59, 0x8A, 0x21, 0x00, 0x81, 0x61, 0x20, 0x5D, 0x6F,
  0x20, 0x00, 0x81, 0x61, 0x20, 0xCB, 0x1E, 0x00, 0x00, 0x83, 0x00, 0x30,
  0x60, 0x08, 0x00, 0x00, 0x02, 0x00, 0x10, 0x08, 0x00, 0x00, 0x00, 0x02,
  0x00, 0x10, 0x08, 0x00, 0x00, 0x00, 0x06, 0x00, 0x10, 0x10, 0x00, 0x00,
  0x00, 0x04, 0x00, 0x10, 0x10, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x30, 0x30,
  0x01, 0x00, 0x00, 0x08, 0x00, 0x20, 0x18, 0x02, 0x00, 0x00, 0x18, 0x00,
  0x00, 0x38, 0x02, 0x00, 0x00, 0x30, 0x00, 0x00, 0xF8, 0x04, 0x00, 0x00,
  0x60, 0x00, 0x00, 0xF8, 0x05, 0x00, 0x00, 0xC0, 0x00, 0x00, 0xF0, 0x07,
  0x00, 0x00, 0x80, 0x01, 0x00, 0xE0, 0x07, 0x00, 0x00, 0x00, 0x07, 0x00,
  0xE0, 0x07, 0x00, 0x00, 0x00, 0x1C, 0x00, 0xC0, 0x07, 0x00, 0x00, 0x00,
  0xF0, 0x00, 0x02, 0x07, 0x00, 0x00, 0x00, 0x80, 0xFF, 0x01, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// ========================= OLED: MESSAGGI DI BOOT =========================
void drawLayoutBoot() {
  oled.begin();
  oled.clearBuffer();

  // ----- Gruppo logo + scritta "Ronin 00" -----
  const char* roninText = "Ronin 00";

  // Font più piccolo e centrato rispetto al logo
  oled.setFont(u8g2_font_logisoso24_tf);
  int16_t textW = oled.getStrWidth(roninText);

  const int16_t GAP      = 8;   // spazio tra logo e testo
  const int16_t centerY  = 26;  // centro verticale del gruppo (alzato rispetto a prima)

  int16_t groupW = MINU_LOGO_WIDTH + GAP + textW;
  int16_t groupX = (256 - groupW) / 2;  // centro il gruppo orizzontalmente

  // Logo Minù 56x56, centrato su centerY
  int16_t logoX = groupX;
  int16_t logoY = centerY - (MINU_LOGO_HEIGHT / 2);  // con 56px viene abbastanza alto
  oled.drawXBMP(logoX, logoY, MINU_LOGO_WIDTH, MINU_LOGO_HEIGHT, MINU_LOGO_bits);

  // Scritta "Ronin 00" centrata verticalmente rispetto al logo
  const int16_t TEXT_H = 24;  // altezza nominale del font logisoso24
  int16_t textX = logoX + MINU_LOGO_WIDTH + GAP;
  int16_t textBaselineY = centerY + (TEXT_H / 2) - 1;  // -1 per tenerla ben centrata visivamente
  oled.drawStr(textX, textBaselineY, roninText);

  // ----- Riga di stato in basso -----
  oled.setFont(u8g2_font_6x12_tr);
  drawCenteredText("Boot inizializzato...", 62);

  oled.sendBuffer();
  delay(3000);
}


void drawBatteryIcon(int x, int y, uint8_t level, bool charging) {
  const int w = 19;   // <– prima era 18, allungata di 1 px
  const int h = 8;

  // cornice batteria
  oled.drawFrame(x, y, w, h);
  // terminale a destra
  oled.drawBox(x + w, y + 2, 2, h - 4);

  // livello base
  uint8_t bars = (level > 4) ? 4 : level;

  // animazione carica: 0→4 tacche a ciclo
  if (charging) {
    uint32_t t = millis();
    uint8_t phase = (t / 250) % 5; // 0,1,2,3,4
    bars = phase;
  }

  // barre interne (4 step)
  int barWidth   = 3;
  int barHeight  = h - 4; // dentro il frame
  int spacing    = 1;

  for (int i = 0; i < 4; i++) {
    if (i < bars) {
      int bx = x + 2 + i * (barWidth + spacing);
      int by = y + 2;
      oled.drawBox(bx, by, barWidth, barHeight);
    }
  }
}

void drawNetIcon(int x, int y, bool connected) {
  int baseY = y + 6;  // "suolo" delle tacche

  // 3 barrette crescenti
  oled.drawBox(x,     baseY - 2, 2, 2); // alta 2
  oled.drawBox(x + 3, baseY - 4, 2, 4); // alta 4
  oled.drawBox(x + 6, baseY - 6, 2, 6); // alta 6

  if (!connected) {
    // slash di disconnessione
    oled.drawLine(x,     y,
                  x + 7, y + 6);
  }
}

void drawTargetIcon(int x, int y) {
  int cx = x + 5;
  int cy = y + 5;

  // cerchio principale
  oled.drawCircle(cx, cy, 4);
  // puntino centrale (opzionale ma carino)
  oled.drawDisc(cx, cy, 1);

  // croce che sporge oltre il cerchio
  oled.drawVLine(cx, y,     11);  // verticale: da top a bottom del box
  oled.drawHLine(x,  cy,    11);  // orizzontale: da sinistra a destra
}

// ========================= UI HELPER FUNCTIONS =========================

// Mode: preset di lavoro (work / normal vs fine / live)
const char* uiGetModeValue() {
  if (stEnable) {
    // Preset \"work/normal\": MA_DEFAULT, ST on
    return "WORK";
  } else {
    // Preset \"fine/live\": MA_FINE, ST off
    return "LIVE";
  }
}

// State: STABLE / UNSTABLE / LIVE
inline const char* uiGetStateValue(const char* modeLabel) {
  // modeLabel arriva già dalla state machine come \"STABLE\", \"UNSTABLE\" o \"LIVE\"
  return modeLabel;
}

// Batteria: per ora wrapper placeholder in attesa di aggancio completo a BatteryStatus
uint8_t uiGetBatteryLevel4Step() {
  BatteryStatus st = battery_get_status();
  switch (st.level) {
    case BATT_LEVEL_FULL:     return 4; // 4 tacche piene
    case BATT_LEVEL_GOOD:     return 3; // 3 tacche
    case BATT_LEVEL_LOW:      return 2; // 2 tacche
    case BATT_LEVEL_CRITICAL:
    default:                  return 1; // 1 tacca (quasi scarica)
  }
}

bool uiIsBatteryCharging() {
  BatteryStatus st = battery_get_status();
  return st.charging;
}

// Formattazione grammi in stile \"12.250\" (puntino come separatore delle migliaia)
void formatGramsWithDot(long gDisp, char* buf, size_t len) {
  long value = gDisp;
  bool neg = false;
  if (value < 0) {
    neg = true;
    value = -value;
  }

  char tmp[16];
  int pos = 0;
  int digits = 0;

  if (value == 0) {
    tmp[pos++] = '0';
  } else {
    while (value > 0 and pos < (int)sizeof(tmp) - 1) {
      if (digits > 0 and (digits % 3) == 0) {
        tmp[pos++] = '.';
      }
      int digit = value % 10;
      tmp[pos++] = '0' + digit;
      value /= 10;
      digits++;
    }
  }

  if (neg and pos < (int)sizeof(tmp) - 1) {
    tmp[pos++] = '-';
  }
  tmp[pos] = '\0';

  int outLen = (pos < (int)len - 1) ? pos : (int)len - 1;
  for (int i = 0; i < outLen; ++i) {
    buf[i] = tmp[pos - 1 - i];
  }
  buf[outLen] = '\0';
}



void drawLayout1(long gDisp, const char* modeLabel) {
  const int16_t BASELINE_Y = 56;   // baseline comune per le "g"

  // ------- Valori dinamici per la UI -------
  const char* modeValue  = uiGetModeValue();   // es: WORK / LIVE
  const char* stateValue = modeLabel;          // STABLE / UNSTABLE / LIVE

  // ------- Riga alta: batteria + wifi -------
  uint8_t batteryLevel = uiGetBatteryLevel4Step();  // 0..4 (placeholder finché non agganciamo BatteryStatus)
  bool    isCharging   = uiIsBatteryCharging();     // placeholder
  bool    netConnected = false;                     // placeholder in attesa gestione reale NET

  // Usa le tue funzioni già definite
  drawBatteryIcon(2, 2, batteryLevel, isCharging);
  drawNetIcon(30, 3, netConnected);

  // ------- Info Mode / State (allineate) -------
  oled.setFont(u8g2_font_6x12_tr);

  const int16_t LABEL_X = 2;

  // Calcolo X comune per i valori, in base alla label più lunga
  int16_t wModeLabel  = oled.getStrWidth("Mode:");
  int16_t wStateLabel = oled.getStrWidth("State:");
  int16_t labelMaxW   = (wModeLabel > wStateLabel) ? wModeLabel : wStateLabel;
  const int16_t VALUE_X = LABEL_X + labelMaxW + 8; // 4 px di spazio dopo i due punti

  // Riga 1: Mode
  oled.drawStr(LABEL_X, 24, "Mode:");
  oled.drawStr(VALUE_X, 24, modeValue);

  // Riga 2: State
  oled.drawStr(LABEL_X, 36, "State:");
  oled.drawStr(VALUE_X, 36, stateValue);

  // ------- Icona target (mirino approvato) -------
  // 11x11 px, cerchio + croce che sporge
  drawTargetIcon(2, BASELINE_Y - 11);

  // ------- Target: placeholder fisso 10.500g (da collegare in futuro) -------
  const char *targetStr = "10.500";

  // numero target, piccolo
  oled.setFont(u8g2_font_logisoso16_tn);
  int16_t targetX     = 2 + 11 + 3;              // mirino (11) + 3 px
  int16_t targetWidth = oled.getStrWidth(targetStr);
  oled.drawStr(targetX, BASELINE_Y, targetStr);

  // "g" del target RELATIVA al numero
  oled.setFont(u8g2_font_6x12_tr);
  const int16_t TARGET_G_MARGIN = 0;             // se è ancora vicino, alza questo valore
  int16_t gTargetX             = targetX + targetWidth + TARGET_G_MARGIN;
  oled.drawStr(gTargetX, BASELINE_Y, "g");

  // ------- Peso principale: gDisp formattato come 12.250 -------

  char weightStr[16];
  formatGramsWithDot(gDisp, weightStr, sizeof(weightStr));

  // 1) Calcolo larghezze con i font giusti
  oled.setFont(u8g2_font_logisoso46_tn);
  int16_t weightWidth = oled.getStrWidth(weightStr);

  oled.setFont(u8g2_font_6x12_tr);
  int16_t gWidth = oled.getStrWidth("g");

  const int16_t RIGHT_MARGIN = 2;   // distanza dal bordo destro
  const int16_t MAIN_OVERLAP = 0;   // quanto il numero "entra" sotto la g (versione che ti piaceva)

  // Posizione della g principale fissata sul bordo destro
  int16_t gMainX = 256 - gWidth - RIGHT_MARGIN;
  oled.drawStr(gMainX, BASELINE_Y, "g");

  // Numero grande spinto parecchio verso destra (overlap sulla g)
  oled.setFont(u8g2_font_logisoso46_tn);
  int16_t weightX = gMainX - weightWidth + MAIN_OVERLAP;
  oled.drawStr(weightX, BASELINE_Y, weightStr);
}



// ========================= OLED: FUNZIONE DI RENDER =========================
void oledRender(long gDisp, const char* modeLabel){
  oled.clearBuffer();
  drawLayout1(gDisp, modeLabel);
  oled.sendBuffer();
}


// ========================= SETUP =========================
void setup(){
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n=== ESP32 + NAU7802 — versione light (5 V) + OLED SSD1322 ==="));

  printHelp();

  // I2C per NAU
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!initNAU7802()){
    Serial.println(F("[NAU] ERRORE inizializzazione, controlla cablaggio/alimentazione."));
  }

  loadFromNVS();
  maN = MA_DEFAULT;
  setMA(maN);
  deadbandUnstable = DB_UNSTABLE_N;

  Serial.print(F("[LOAD] DEFAULT_CPG=")); Serial.print(DEFAULT_CPG,6);
  Serial.print(F("  SCALE_CPG="));        Serial.print(SCALE_CPG,6);
  Serial.print(F("  REF_G="));            Serial.println(REF_G);

  autoTareOnBoot();

  // SPI + OLED
  SPI.begin(18, -1, 23, OLED_CS); // SCK=18, MISO unused, MOSI=23, SS=OLED_CS
  drawLayoutBoot();

  lastOledMs = millis();
  
  battery_init();
}
//debug batteria
static uint32_t lastBattDebug = 0;
// ========================= LOOP =========================
void loop(){
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

  // --- Cadenzamento letture ---
  unsigned long now = millis();
  if (now - lastSampleMs < SAMPLE_MS) {
    // comunque aggiorno OLED se è ora
    if (now - lastOledMs >= OLED_UPDATE_MS) {
      // niente nuovo dato, ma puoi tenerlo aggiornato con l'ultimo gDisp
      // (non faccio nulla qui: gDisp verrà aggiornato nel ramo principale)
    }
    return;
  }
  lastSampleMs += SAMPLE_MS;

  if (!myScale.available()){
    return; // nessun nuovo dato dal NAU
  }

  
  //aggiorno stato batteria
  battery_update(now);

   // ogni 3 secondi stampa lo stato batteria
    if (now - lastBattDebug > 3000) {
        lastBattDebug = now;
        BatteryStatus st = battery_get_status();
        battery_debug_print(st);
    }

  // 1) Lettura grezza NAU
  long raw = myScale.getReading();

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
    oledRender(gDisp, modeLabel);
  }

}
