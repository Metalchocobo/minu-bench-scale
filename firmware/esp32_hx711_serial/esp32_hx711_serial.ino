
// =======================================================
// ESP32 DevKit + HX711 + OLED SSD1322 256x64
// Versione "light" con stati STABLE/UNSTABLE + ZT + OLED
//  • HX711: DOUT=GPIO35, SCK=GPIO16 (RX2)
//    NOTE: il rate 10 Hz / 80 Hz è HARDWARE (pin RATE del modulo)
//  • INA219 su I2C (monitor batteria)
//  • OLED SSD1322 su SPI hardware (CLK=18, MOSI=23)
//    CS=D21, DC=D22, RST=D19
// =======================================================

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <U8g2lib.h>
#include "battery_monitor.h"
#include "hx711_driver.h"
#include "ui_display.h"
#include "keypad.h"
#include "scale_core.h"
#include <Preferences.h>
#include <math.h>
#include "buzzer.h"
#include "net_ota_cloud.h"
#include <esp_sleep.h>
#include <driver/gpio.h>
Preferences prefs; 
// Stato boot (per gestire errori "hard" vs "ack")
static bool g_hxBeginOk = false;

// ========================= HX711 (driver separato) =========================
// Pins richiesti dall'utente: SCK su RX2 (GPIO16), DOUT su GPIO35
static const int HX711_SCK_PIN  = 16;
static const int HX711_DOUT_PIN = 35;

// Gain: A=128 (1 impulso extra dopo i 24 bit)
static const HX711Gain HX711_GAIN = HX711_GAIN_128_A;

// Timing: micro-delay per stabilizzare lettura (valori piccoli, ma non a 0)
static const uint8_t HX711_PULSE_US = 1;

// Se DOUT non va mai LOW, consideriamo "sensore non trovato"
static const uint32_t HX711_INIT_TIMEOUT_MS = 1200;

static long readRawHXOnceBlocking(uint32_t timeoutMs){
  return hx711_read_blocking(timeoutMs);
}

static long readRawHXAvg(int n){
  if (n <= 1) return readRawHXOnceBlocking(200);
  long s = 0;
  for(int i=0;i<n;i++){
    long r = readRawHXOnceBlocking(200);
    if (r == 0) return 0;
    s += r;
  }
  return s / n;
}

// ========================= CONFIG =========================

// [0] Calibrazione seed (valori indicativi, sovrascritti da NVS + CAL)
const long  DEFAULT_REF_RAW  = 265290;   // raw medio con 2000 g
const long  DEFAULT_ZERO_RAW = 51471;    // raw medio a vuoto
const long  DEFAULT_REF_G    = 2000;     // grammi di riferimento
const float DEFAULT_CPG      = float(DEFAULT_REF_RAW - DEFAULT_ZERO_RAW) / float(DEFAULT_REF_G);
// seed: 500 counts/grammo (HX711)
// NOTE: se arrivi da firmware NAU, la scala salvata in NVS NON è valida: fai CAL.

// [1] Auto-TARE all’avvio (semplice, niente logiche complicate)
const bool     AUTO_TARE_ON_BOOT   = true;
const uint16_t AUTO_TARE_SAMPLES   = 25;
const uint16_t AUTO_TARE_SETTLE_MS = 800;
const uint16_t AUTO_TARE_MIN_SAMPLES = 10;    // minimo campioni richiesti
const float    AUTO_TARE_RANGE_G    = 0.40f;  // finestra di quiete (range) per fermarsi prima
const uint32_t AUTO_TARE_QUIET_MS    = 400;    // deve restare in range per questo tempo

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

// [6] Campionamento logica (non il sample rate del sensore)
const uint32_t SAMPLE_MS = 60; // ~16 Hz logica (HX711 10 Hz o 80 Hz, hardware)

// [OLED] Cadenzamento refresh
const uint32_t OLED_UPDATE_MS = 60;

// ========================= PIN & OGGETTI =========================

// HX711: I2C spostato su D27 (SDA) e D26 (SCL) per averli vicini
const int I2C_SDA = 32;  // etichetta scheda: D27 (GPIO27)
const int I2C_SCL = 33;  // etichetta scheda: D26 (GPIO26)



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


// --- Decimazione HX711 quando lavori a 80 SPS ---
// Obiettivo: ridurre rumore senza perdere reattività.
// WORK: usato per logiche STABLE/UNSTABLE, ZT, ecc.  (N più alto)
// LIVE: usato SOLO per visualizzazione in modalità LIVE (stEnable=false) (N più basso)
static const uint8_t DECIM_WORK_N = 5;       // ~16 Hz se HX=80 SPS
static const uint8_t DECIM_LIVE_N = 2;       // ~40 Hz se HX=80 SPS
static const uint8_t LIVE_DEBOUNCE_HITS = 3; // 3 aggiornamenti consecutivi uguali

// Ultimi valori "pubblici" per il display
static long gDispWorkLast = 0;
static const char* gStateLabelWorkLast = "----";
static long gDispLiveLast = 0;

// Accumulatori per decimazione
static long gLiveSum = 0;
static uint8_t gLiveCnt = 0;
static long gWorkSum = 0;
static uint8_t gWorkCnt = 0;

// Debounce display LIVE
static long gLiveCand = 0;
static uint8_t gLiveHits = 0;


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
  // Comportamento "come prima" (firmware NAU): nessuna firma sensore.
  // Se in NVS ci sono valori vecchi/non compatibili, vedrai pesi sballati -> fai CAL e poi salva.
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
  prefs.putLong("offset", OFFSET_RAW);
  prefs.putFloat("scale", SCALE_CPG);
  prefs.putLong("ref_g",  REF_G);
  prefs.end();
  Serial.println(F("[SAVE] Parametri salvati in NVS."));
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
bool autoTareOnBoot(uint16_t maxSamples){
  if (!AUTO_TARE_ON_BOOT){
    Serial.println(F("[BOOT] Auto-TARE disattivata."));
    return true;
  }

  // Auto-tare "smart":
  // - minimo AUTO_TARE_MIN_SAMPLES
  // - continua finché la finestra (range) rientra sotto soglia per AUTO_TARE_QUIET_MS
  // - massimo maxSamples (di default AUTO_TARE_SAMPLES = 25)
  Serial.println(F("[BOOT] Auto-TARE smart..."));
  delay(AUTO_TARE_SETTLE_MS);

  const uint16_t minSamples = AUTO_TARE_MIN_SAMPLES;
  if (maxSamples < minSamples) maxSamples = minSamples;
  if (maxSamples > 25) maxSamples = 25; // safety: buffer fisso

  long buf[25];
  uint16_t n = 0;
  uint32_t quietStart = 0;

  while (n < maxSamples){
    long r = readRawHXOnceBlocking(200);
    if (r == 0) {
      Serial.println(F("[BOOT] Auto-TARE FAIL (timeout lettura HX711)"));
      return false;
    }

    buf[n++] = r;

    if (n >= minSamples){
      // Range sull'ultima finestra minSamples
      long mn = buf[n - minSamples];
      long mx = mn;
      for (uint16_t i = n - minSamples; i < n; i++){
        long v = buf[i];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
      }

      float rangeG = 999999.0f;
      if (SCALE_CPG > 0.01f){
        rangeG = float(mx - mn) / SCALE_CPG;
      }

      if (rangeG <= AUTO_TARE_RANGE_G){
        if (quietStart == 0) quietStart = millis();
        if (millis() - quietStart >= AUTO_TARE_QUIET_MS){
          break; // stabile abbastanza
        }
      } else {
        quietStart = 0;
      }
    }
  }

  // Media sugli ultimi minSamples (se disponibili), così ignoriamo eventuali transienti iniziali
  uint16_t useN = (n >= minSamples) ? minSamples : n;
  long s = 0;
  for (uint16_t i = n - useN; i < n; i++){
    s += buf[i];
  }
  long rawZero = s / (long)useN;

  OFFSET_RAW = rawZero;
  zero_track_counts = 0;
  resetFiltersAndState();

  Serial.print(F("[BOOT] Auto-TARE OK. OFFSET_RAW="));
  Serial.print(OFFSET_RAW);
  Serial.print(F("  n=")); Serial.print(n);
  Serial.print(F("  useN=")); Serial.println(useN);

  if (n >= minSamples && quietStart == 0){
    Serial.println(F("[BOOT] Auto-TARE: non ha trovato una finestra 'quiet' entro il max, ma ha impostato comunque l'offset. Se serve: premi TARA."));
  }

  return true;
}

// ========================= INIT HX711 =========================
bool initHX711(){
  Serial.println(F("[HX711] Init HX711..."));

  g_hxBeginOk = false;

  hx711_begin(HX711_DOUT_PIN, HX711_SCK_PIN, HX711_GAIN, HX711_PULSE_US);

  uint32_t t0 = millis();
  while (millis() - t0 < HX711_INIT_TIMEOUT_MS) {
    if (hx711_is_ready()) {
      // prima lettura di stabilizzazione (scarta)
      (void)hx711_read();
      g_hxBeginOk = true;
      Serial.println(F("[HX711] OK (DOUT ready)"));
      return true;
    }
    delay(10);
  }

  Serial.println(F("[HX711] FAIL - DOUT non pronto (cablaggio/alimentazione/livello logico?)"));
  return false;
}

// ------------------------- BOOT UX helpers -------------------------
static void bootPump(uint32_t ms) {
  uint32_t start = millis();
  while (millis() - start < ms) {
#if ENABLE_WIFI_OTA || ENABLE_ARDUINO_CLOUD
    Net::update();
#endif
    // Manteniamo viva la lettura tastiera anche durante boot (utile se qualcosa si blocca)
    uint32_t now = millis();
    keypad_update(now);
    (void)keypad_get_event();
    delay(10);
  }
}

static void bootShow(const char* line1, const char* line2, uint16_t holdMs = 350) {
  ui_showBoot(line1, line2);
  bootPump(holdMs);
}

static void bootWaitTare() {
  while (true) {
    uint32_t now = millis();
    keypad_update(now);
    KeyCode k = keypad_get_event();
    if (k == KEY_TARE) {
      buzzerKeyClick();
      return;
    }
#if ENABLE_WIFI_OTA || ENABLE_ARDUINO_CLOUD
    Net::update();
#endif
    delay(10);
  }
}


// ========================= SETUP =========================
void setup(){
  Serial.begin(115200);
  delay(50);

  // DISPLAY SUBITO (per mostrare che la bilancia sta avviando)
  ui_init();
  ui_showBoot("Accensione...", "");

  // Suono subito dopo che il display è acceso
  buzzerInit();
  buzzerBootMelody();
  
  Serial.println(F("\n=== ESP32 + HX711 — versione light + OLED SSD1322 ==="));

  printHelp();

  // Tastierino (non mostriamo la fase, ma ci serve già da boot per gestire ACK errori)
  keypad_init();

#if ENABLE_WIFI_OTA
  Net::wifiSetup();
  Net::otaSetup("minu-bench-scale");
#endif

#if ENABLE_ARDUINO_CLOUD
  Net::cloudSetup();
#endif


  // ------------------------ Sequenza boot "leggibile" ------------------------
  bootShow("I2C", "Avvio bus...");
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  bootShow("I2C", "OK");

  // HX711
  bootShow("Sensore peso", "Init HX711...");
  bool hxOk = initHX711();

  // Se il modulo non è trovato, è un errore duro: ritenta su TARE
  while (!g_hxBeginOk) {
    ui_showError("HX711", "Modulo non trovato", "Premi TARA per ritentare");
    bootWaitTare();
    bootShow("Sensore peso", "Ritentativo...");
    hxOk = initHX711();
  }

  // Config/AFE fallite: mostriamo errore, ma dopo ACK si procede (può comunque funzionare)
  if (!hxOk) {
    ui_showError("HX711", "Init: FAIL", "Premi TARA per continuare");
    bootWaitTare();
  }
  bootShow("Sensore peso", "OK");

  // NVS
  bootShow("Parametri", "Carico NVS...");
  loadFromNVS();
  bootShow("Parametri", "OK");

  // Preset iniziale
  bootShow("Preset", "WORK");
  // Preset WORK = modalità "work/normal" (MA default, ST on, ZT on)
  setMode("work");

  // Auto-TARE
  if (AUTO_TARE_ON_BOOT) {
    bootShow("Auto-TARE", "In corso...");
    bool tareOk = autoTareOnBoot(AUTO_TARE_SAMPLES);
    if (!tareOk) {
      ui_showError("Auto-TARE", "Timeout/FAIL", "Premi TARA per continuare");
      bootWaitTare();
    }
  }

  // Batteria
  bootShow("Batteria", "Init INA219...");
  battery_init();
  if (!battery_is_available()) {
    ui_showError("Batteria", "INA219 non trovato", "Premi TARA per continuare");
    bootWaitTare();
  }

  bootShow("Avvio", "Completato");

  lastOledMs = millis();

}

//debug batteria
static uint32_t lastBattDebug = 0;

// ========================= BATTERIA SLA 6V: sicurezza ESP =========================
// Sotto certe tensioni (filtrate) il buck a 5V può perdere margine -> latenze/reset.
// Strategia:
//  - 0 tacche (EMPTY): beep di avviso ogni 60s (ma UI normale)
//  - "fase di stacco" (<= V_SAFE_SHUTDOWN_MIN_V): mostra solo avviso + beep ogni 10s per 60s, poi LIGHT-SLEEP
//  - Wake SOLO da tastiera (qualsiasi tasto). Niente wake automatico.

static const float    V_SAFE_SHUTDOWN_MIN_V     = 5.80f;   // inizia countdown per sleep (V)
static const float    V_SAFE_SHUTDOWN_CLEAR_V   = 5.90f;   // isteresi: annulla countdown se risali sopra
static const float    V_HARD_SLEEP_MIN_V        = 5.70f;   // troppo bassa: vai a sleep subito
static const uint32_t SAFE_SHUTDOWN_DEBOUNCE_MS = 5000;    // deve restare sotto soglia per 5s

static const uint32_t PRE_SLEEP_COUNTDOWN_MS    = 60000;   // 60s avviso prima dello sleep
static const uint32_t BEEP_EMPTY_MS             = 60000;   // beep ogni 60s (0 tacche)
static const uint32_t BEEP_COUNTDOWN_MS         = 10000;   // beep ogni 10s durante countdown

static uint32_t g_lowBattSinceMs      = 0;
static uint32_t g_preSleepStartMs     = 0;
static uint32_t g_lastEmptyBeepMs     = 0;
static uint32_t g_lastCountdownBeepMs = 0;

static void enterLowBatteryLightSleep() {
  // Ultima schermata: avviso e richiesta alimentatore, poi sleep.
  if (battery_is_available()) {
    BatteryStatus st = battery_get_status();
    ui_showBatteryShutdown(st.voltage_V);
  } else {
    ui_showError("BATTERIA", "INA219 non trovato", "");
  }

  // Un piccolo margine per far vedere la schermata
  delay(80);

  // Riduci consumi
#if ENABLE_WIFI_OTA || ENABLE_ARDUINO_CLOUD
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
#endif

  // Prepara wake da tastiera:
  // - Righe a LOW fisso
  // - Colonne INPUT_PULLUP
  // Premendo un tasto, una colonna viene tirata LOW -> wake.
  const int* rows = nullptr; int nRows = 0;
  const int* cols = nullptr; int nCols = 0;
  keypad_get_pins(&rows, &nRows, &cols, &nCols);

  for (int i = 0; i < nRows; i++) {
    pinMode(rows[i], OUTPUT);
    digitalWrite(rows[i], LOW);
  }
  for (int i = 0; i < nCols; i++) {
    pinMode(cols[i], INPUT_PULLUP);
    // abilita wake su livello LOW (solo in light-sleep)
    gpio_wakeup_enable((gpio_num_t)cols[i], GPIO_INTR_LOW_LEVEL);
  }

  // Spegni display (riduce consumi). Rimane spento finché non riparti.
  ui_powerSave(true);

  // Abilita wake GPIO (light-sleep) e vai in sleep.
  // Niente timer wake: resta lì finché non premi un tasto.
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
  esp_sleep_enable_gpio_wakeup();

  delay(20);
  esp_light_sleep_start();

  // Dopo wake: riparti pulito (re-init periferiche).
  ESP.restart();
}

static bool maybeEnterSafeShutdown(uint32_t nowMs) {
  if (!battery_is_available()) return false;
  BatteryStatus st = battery_get_status();

  // Se è in carica o risalito sopra clear: azzera tutto
  if (st.charging || st.voltage_V >= V_SAFE_SHUTDOWN_CLEAR_V) {
    g_lowBattSinceMs = 0;
    g_preSleepStartMs = 0;
    g_lastEmptyBeepMs = 0;
    g_lastCountdownBeepMs = 0;
    return false;
  }

  // Stato "0 tacche": beep ogni minuto, UI normale
  if (st.level == BATT_LEVEL_EMPTY && st.voltage_V > V_SAFE_SHUTDOWN_MIN_V) {
    if (g_lastEmptyBeepMs == 0 || (nowMs - g_lastEmptyBeepMs) >= BEEP_EMPTY_MS) {
      buzzerWarn();
      g_lastEmptyBeepMs = nowMs;
    }
    return false; // non oscurare la pesata
  }

  // Sotto hard-min: sleep immediato (dopo una schermata)
  if (st.voltage_V <= V_HARD_SLEEP_MIN_V) {
    buzzerWarn();
    enterLowBatteryLightSleep(); // non ritorna
    return true;
  }

  // Sotto soglia: debounce + countdown prima di andare a sleep
  if (st.voltage_V <= V_SAFE_SHUTDOWN_MIN_V) {
    if (g_lowBattSinceMs == 0) g_lowBattSinceMs = nowMs;

    // Debounce prima di entrare nella fase "stacco"
    if ((nowMs - g_lowBattSinceMs) < SAFE_SHUTDOWN_DEBOUNCE_MS) {
      return false; // ancora nessun avviso bloccante
    }

    // Avvio countdown
    if (g_preSleepStartMs == 0) {
      g_preSleepStartMs = nowMs;
      g_lastCountdownBeepMs = 0;
    }

    // Durante countdown: mostra SOLO avviso
    ui_showBatteryShutdown(st.voltage_V);

    if (g_lastCountdownBeepMs == 0 || (nowMs - g_lastCountdownBeepMs) >= BEEP_COUNTDOWN_MS) {
      buzzerWarn();
      g_lastCountdownBeepMs = nowMs;
    }

    if ((nowMs - g_preSleepStartMs) >= PRE_SLEEP_COUNTDOWN_MS) {
      enterLowBatteryLightSleep(); // non ritorna
    }

    return true; // oscuriamo la pesata mentre siamo in countdown
  }

  // Qualsiasi altra condizione: reset dei timer "low"
  g_lowBattSinceMs = 0;
  g_preSleepStartMs = 0;
  g_lastCountdownBeepMs = 0;
  return false;
}

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

  // Stato batteria + safety shutdown (indipendente dal sensore peso)
  battery_update(now);
  if (maybeEnterSafeShutdown(now)) { return; }

  // Ogni 3 secondi stampa lo stato batteria
  if (now - lastBattDebug > 3000) {
    lastBattDebug = now;
    BatteryStatus st = battery_get_status();
    battery_debug_print(st);
  }

  
// ====================== HX711 @ 80 SPS (o 10 SPS) + decimazione ======================
// Leggiamo ogni volta che HX711 ha un dato pronto.
// - WORK (N=5): alimenta stabilità/zero-tracking e quindi "qualità" della pesata.
// - LIVE (N=2): SOLO per display in modalità LIVE (stEnable=false), con debounce.
bool haveNewWork = false;
long rawAvg = 0; // per log/debug (WORK)
long raw = 0;

if (hx711_is_ready()) {
  raw = hx711_read();

  // Mediana veloce su stream raw (aiuta contro spike)
  long rawMed = pushMedian3(raw);

  // Accumulo per LIVE e WORK
  gLiveSum += rawMed;
  gLiveCnt++;
  gWorkSum += rawMed;
  gWorkCnt++;

  // ---- LIVE: N=2 (solo display quando stEnable=false) ----
  if (gLiveCnt >= DECIM_LIVE_N) {
    long rawLive = gLiveSum / (long)DECIM_LIVE_N;
    gLiveSum = 0;
    gLiveCnt = 0;

    long offEff = effectiveOffsetCounts();
    float gFast = (SCALE_CPG > 0.01f) ? ((float)(rawLive - offEff) / SCALE_CPG) : 0.0f;
    long cand = lroundf(gFast);

    // debounce: accetto solo se uguale per N hit consecutivi
    if (cand == gLiveCand) {
      if (gLiveHits < 255) gLiveHits++;
    } else {
      gLiveCand = cand;
      gLiveHits = 1;
    }
    if (gLiveHits >= LIVE_DEBOUNCE_HITS) {
      gDispLiveLast = gLiveCand;
      gLiveHits = LIVE_DEBOUNCE_HITS;
    }
  }

  // ---- WORK: N=5 (logiche stabilità) ----
  if (gWorkCnt >= DECIM_WORK_N) {
    long rawWork = gWorkSum / (long)DECIM_WORK_N;
    gWorkSum = 0;
    gWorkCnt = 0;

    lastSampleMs = now;
    rawAvg = pushMA(rawWork); // MA applicata sul flusso WORK
    haveNewWork = true;
  }
}

if (haveNewWork) {
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

    // Pubblico i valori WORK per OLED (modalità WORK)
    gDispWorkLast = gDisp;
    gStateLabelWorkLast = modeLabel;
}

// ====================== OLED update cadenzato (sempre) ======================
if (now - lastOledMs >= OLED_UPDATE_MS) {
  lastOledMs = now;

  if (!stEnable) {
    // LIVE: usa flusso veloce (N=2) + debounce
    ui_renderWeight(gDispLiveLast, "LIVE");
  } else {
    ui_renderWeight(gDispWorkLast, gStateLabelWorkLast);
  }
}


}
