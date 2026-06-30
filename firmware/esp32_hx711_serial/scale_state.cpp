#include "scale_state.h"
#include "scale_filters.h"
#include <Preferences.h>
#include <math.h>

namespace ScaleState {

// ========================= STATO INTERNO =========================
static long  g_offsetRaw    = ScaleConfig::DEFAULT_ZERO_RAW;
static float g_scaleCpg     = ScaleConfig::defaultCpg();
static long  g_ztCounts     = 0;

static WeighState g_weighState = STATE_UNSTABLE;
static bool  g_stEnable     = ScaleConfig::ST_ENABLE_DEFAULT;
static bool  g_ztEnable     = ScaleConfig::ZT_ENABLE_DEFAULT;

static float g_gramsLatch   = 0.0f;
static float g_dispUnstable = 0.0f;
static bool  g_dispInit     = false;

static long  g_dispWorkLast = 0;
static long  g_dispLiveLast = 0;
static long  g_dispLiveHalfLast = 0;  // fixed point: grams * 2
static const char* g_stateLabelWorkLast = "UNSTABLE";

static float g_deadbandUnstable = ScaleConfig::DB_UNSTABLE_N;

// Ultimo rawAvg (per calibrazione wizard)
static long g_lastRawAvg = 0;

// ========================= DISPLAY LIVE STATE =========================
static long     g_lastDispLive     = 0;
static uint32_t g_lastDispLiveMs   = 0;
static bool     g_wasZeroLive      = true;
static bool     g_reversalLockLive = false;
static long     g_reversalLockVal  = 0;
static uint32_t g_reversalLockMs   = 0;
static float    g_liveHalfSamples[ScaleConfig::DISP_LIVE_HALF_WIN_N];
static long     g_liveHalfVotes[ScaleConfig::DISP_LIVE_HALF_WIN_N];
static uint8_t  g_liveHalfIdx      = 0;
static uint8_t  g_liveHalfCount    = 0;
static bool     g_liveHalfHasLast  = false;
static float    g_liveHalfLastIn   = 0.0f;

// ========================= DISPLAY WORK STATE =========================
static long g_lastDispWork = 0;
static bool g_wasZeroWork  = true;

// ========================= ZERO-TRACKING STATE =========================
static bool     g_ztLoadDetected = false;
static uint32_t g_ztLoadClearMs  = 0;
static uint32_t g_ztQuietStartMs = 0;
static uint32_t g_lastSnapMs     = 0;
static uint32_t g_lastZtTickMs   = 0;

// ========================= TARA STATE =========================
static bool     g_tareActive      = false;
static bool     g_tareUiActive    = false;
static uint32_t g_tareStartMs     = 0;
static uint32_t g_tareUiStartMs   = 0;
static uint32_t g_tareUiEndMs     = 0;
static long     g_tareSamples[64];
static uint8_t  g_tareSampleCount = 0;

// ========================= GETTERS/SETTERS =========================

void setOffsetRaw(long offset)   { g_offsetRaw = offset; }
long getOffsetRaw()              { return g_offsetRaw; }
void setScaleCpg(float cpg)      { g_scaleCpg = cpg; }
float getScaleCpg()              { return g_scaleCpg; }
void setZtCounts(long counts)    { g_ztCounts = counts; }
long getZtCounts()               { return g_ztCounts; }

long effectiveOffsetCounts() {
  return g_offsetRaw + g_ztCounts;
}

void setWeighState(WeighState st) { g_weighState = st; }
WeighState getWeighState()        { return g_weighState; }

void setStEnable(bool en) { g_stEnable = en; }
bool isStEnabled()        { return g_stEnable; }
void setZtEnable(bool en) { g_ztEnable = en; }
bool isZtEnabled()        { return g_ztEnable; }

void setGramsLatch(float g) { g_gramsLatch = g; }
float getGramsLatch()       { return g_gramsLatch; }

void setDispUnstable(float g) { g_dispUnstable = g; }
float getDispUnstable()       { return g_dispUnstable; }
void setDispInit(bool v)      { g_dispInit = v; }
bool isDispInit()             { return g_dispInit; }

void setDispWorkLast(long g)  { g_dispWorkLast = g; }
long getDispWorkLast()        { return g_dispWorkLast; }
void setDispLiveLast(long g)  { g_dispLiveLast = g; }
long getDispLiveLast()        { return g_dispLiveLast; }
void setDispLiveHalfLast(long gX2) { g_dispLiveHalfLast = gX2; }
long getDispLiveHalfLast()         { return g_dispLiveHalfLast; }

long halfToRoundedGram(long gX2) {
  return (gX2 >= 0) ? ((gX2 + 1) / 2) : ((gX2 - 1) / 2);
}

const char* getStateLabelWorkLast() { return g_stateLabelWorkLast; }
void setStateLabelWorkLast(const char* label) { g_stateLabelWorkLast = label; }

void setLastRawAvg(long raw) { g_lastRawAvg = raw; }
long getLastRawAvg()         { return g_lastRawAvg; }

// ========================= DISPLAY QUANTIZZAZIONE =========================

static void resetLiveHalfVote() {
  g_liveHalfIdx = 0;
  g_liveHalfCount = 0;
  g_liveHalfHasLast = false;
  g_liveHalfLastIn = 0.0f;
}

static long liveRoundGram(float g) {
  return (g >= 0.0f) ? (long)(g + 0.5f) : (long)(g - 0.5f);
}

static void pushLiveHalfSample(float g, long vote) {
  if (g_liveHalfHasLast && fabsf(g - g_liveHalfLastIn) >= 1.5f) {
    resetLiveHalfVote();
  }

  g_liveHalfSamples[g_liveHalfIdx] = g;
  g_liveHalfVotes[g_liveHalfIdx] = vote;
  g_liveHalfIdx++;
  if (g_liveHalfIdx >= ScaleConfig::DISP_LIVE_HALF_WIN_N) g_liveHalfIdx = 0;
  if (g_liveHalfCount < ScaleConfig::DISP_LIVE_HALF_WIN_N) g_liveHalfCount++;

  g_liveHalfLastIn = g;
  g_liveHalfHasLast = true;
}

static float liveHalfAverage() {
  if (g_liveHalfCount == 0) return 0.0f;

  float sum = 0.0f;
  for (uint8_t i = 0; i < g_liveHalfCount; i++) {
    sum += g_liveHalfSamples[i];
  }
  return sum / (float)g_liveHalfCount;
}

static long liveHalfFromAverage(float avg) {
  float mag = fabsf(avg);
  long whole = (long)floorf(mag);
  float frac = mag - (float)whole;

  long magX2 = 0;
  if (frac <= ScaleConfig::DISP_LIVE_HALF_LOW_RATIO) {
    magX2 = whole * 2;
  } else if (frac >= ScaleConfig::DISP_LIVE_HALF_HIGH_RATIO) {
    magX2 = (whole + 1) * 2;
  } else {
    magX2 = whole * 2 + 1;
  }

  return (avg < 0.0f) ? -magX2 : magX2;
}

static long liveHalfVoteResult(float avg, long latestVote) {
  if (g_liveHalfCount == 0) return latestVote * 2;

  long low = g_liveHalfVotes[0];
  long high = low;
  uint8_t highCount = 0;

  for (uint8_t i = 0; i < g_liveHalfCount; i++) {
    long v = g_liveHalfVotes[i];
    if (v < low) low = v;
    if (v > high) high = v;
  }

  if ((high - low) > 1) {
    return liveHalfFromAverage(avg);
  }

  if (high == low) {
    return liveHalfFromAverage(avg);
  }

  for (uint8_t i = 0; i < g_liveHalfCount; i++) {
    if (g_liveHalfVotes[i] == high) highCount++;
  }

  float highRatio = (float)highCount / (float)g_liveHalfCount;
  if (highRatio <= ScaleConfig::DISP_LIVE_HALF_LOW_RATIO) return low * 2;
  if (highRatio >= ScaleConfig::DISP_LIVE_HALF_HIGH_RATIO) return high * 2;
  return low * 2 + 1;
}

static long applyLiveHalfSticky(long cand, long prevHalf, float avg) {
  long diff = cand - prevHalf;
  if (diff == 0) return cand;
  if (diff > 1 || diff < -1) return cand;

  float boundary = (float)(cand + prevHalf) * 0.25f;
  if (diff > 0 && avg < (boundary + ScaleConfig::DISP_LIVE_HALF_STICKY_G)) {
    return prevHalf;
  }
  if (diff < 0 && avg > (boundary - ScaleConfig::DISP_LIVE_HALF_STICKY_G)) {
    return prevHalf;
  }
  return cand;
}

long displayLiveStable(float gIn, long prevDisp, uint32_t nowMs) {
  // Banda zero con isteresi
  float absG = fabsf(gIn);
  bool wantZero = g_wasZeroLive
    ? (absG < ScaleConfig::DISP_ZERO_ENTER_LIVE_G)
    : (absG < ScaleConfig::DISP_ZERO_EXIT_LIVE_G);

  if (wantZero) {
    g_wasZeroLive = true;
    g_lastDispLive = 0;
    g_lastDispLiveMs = nowMs;
    g_reversalLockLive = false;
    return 0;
  }

  g_wasZeroLive = false;

  // Quantizza a 1 g
  long cand = (gIn >= 0.0f) ? (long)(gIn + 0.5f) : (long)(gIn - 0.5f);

  // Reversal lock
  if (g_reversalLockLive && cand == g_reversalLockVal) {
    if ((nowMs - g_reversalLockMs) < ScaleConfig::DISP_LIVE_REVERSAL_LOCK_MS) {
      return g_lastDispLive;
    }
    g_reversalLockLive = false;
  }

  // Isteresi
  long prev = g_lastDispLive;
  float dG = fabsf(gIn - (float)prev);
  if (dG < ScaleConfig::DISP_STEP_HYS_LIVE_G) {
    cand = prev;
  }

  // Eccezione 0→±1
  if (prev == 0 && cand != 0) {
    // Nessun lock
  } else if (cand != prev && cand == g_lastDispLive - (cand - prev)) {
    g_reversalLockLive = true;
    g_reversalLockVal  = cand;
    g_reversalLockMs   = nowMs;
    cand = prev;
  }

  g_lastDispLive   = cand;
  g_lastDispLiveMs = nowMs;
  return cand;
}

long displayLiveHalfStable(float gIn, long prevHalf, uint32_t nowMs) {
  float absG = fabsf(gIn);
  bool wantZero = g_wasZeroLive
    ? (absG < ScaleConfig::DISP_ZERO_ENTER_LIVE_G)
    : (absG < ScaleConfig::DISP_ZERO_EXIT_LIVE_G);

  if (wantZero) {
    g_wasZeroLive = true;
    g_lastDispLive = 0;
    g_lastDispLiveMs = nowMs;
    g_reversalLockLive = false;
    resetLiveHalfVote();
    return 0;
  }

  g_wasZeroLive = false;
  long latestVote = liveRoundGram(gIn);
  pushLiveHalfSample(gIn, latestVote);

  float avg = liveHalfAverage();
  long cand = liveHalfVoteResult(avg, latestVote);
  cand = applyLiveHalfSticky(cand, prevHalf, avg);

  g_lastDispLive = halfToRoundedGram(cand);
  g_lastDispLiveMs = nowMs;
  return cand;
}

long displayLiveQuant(float gIn, long prevDisp) {
  // Semplice arrotondamento per WORK
  float absG = fabsf(gIn);
  bool wantZero = g_wasZeroWork
    ? (absG < ScaleConfig::DISP_ZERO_ENTER_WORK_G)
    : (absG < ScaleConfig::DISP_ZERO_EXIT_WORK_G);

  if (wantZero) {
    g_wasZeroWork = true;
    return 0;
  }
  g_wasZeroWork = false;

  long cand = (gIn >= 0.0f) ? (long)(gIn + 0.5f) : (long)(gIn - 0.5f);

  float dG = fabsf(gIn - (float)prevDisp);
  if (dG < ScaleConfig::DISP_STEP_HYS_WORK_G && prevDisp != 0) {
    return prevDisp;
  }
  return cand;
}

long displayWorkQuant(float gIn, long prevDisp) {
  float absG = fabsf(gIn);
  bool wantZero = g_wasZeroWork
    ? (absG < ScaleConfig::DISP_ZERO_ENTER_WORK_G)
    : (absG < ScaleConfig::DISP_ZERO_EXIT_WORK_G);

  if (wantZero) {
    g_wasZeroWork = true;
    g_lastDispWork = 0;
    return 0;
  }
  g_wasZeroWork = false;

  long cand = (gIn >= 0.0f) ? (long)(gIn + 0.5f) : (long)(gIn - 0.5f);

  float dG = fabsf(gIn - (float)prevDisp);
  if (dG < ScaleConfig::DISP_STEP_HYS_WORK_G && prevDisp != 0) {
    cand = prevDisp;
  }

  g_lastDispWork = cand;
  return cand;
}

// ========================= ZERO-TRACKING =========================

void applyZtStepTowardZero(float gLive) {
  if (fabsf(gLive) > ScaleConfig::ZT_WINDOW_G) return;
  if (fabsf((float)g_ztCounts / g_scaleCpg) >= ScaleConfig::ZT_MAX_G) return;

  float stepG = ScaleConfig::ZT_STEP_G;
  long  stepCnt = (long)(stepG * g_scaleCpg);
  if (stepCnt < 1) stepCnt = 1;

  if (gLive > 0.01f) {
    g_ztCounts += stepCnt;
  } else if (gLive < -0.01f) {
    g_ztCounts -= stepCnt;
  }

  // Cap
  long maxCnt = (long)(ScaleConfig::ZT_MAX_G * g_scaleCpg);
  if (g_ztCounts > maxCnt)  g_ztCounts = maxCnt;
  if (g_ztCounts < -maxCnt) g_ztCounts = -maxCnt;
}

bool isQuietForZt() {
  float rng   = ScaleFilters::rangeLastNSamples(
    constrain((int)(ScaleConfig::ZT_QUIET_MS / ScaleConfig::SAMPLE_MS), 2, 64)
  );
  float slope = fabsf(ScaleFilters::slopeLastNSamples(
    constrain((int)(800.0f / ScaleConfig::SAMPLE_MS), 2, 64),
    ScaleConfig::SAMPLE_MS
  ));
  return (rng <= ScaleConfig::ZT_QUIET_RANGE_G && slope <= ScaleConfig::ZT_QUIET_SLOPE_GPS);
}

bool trySnapOnUnload(float gLive, float slopeGps, bool quietNow) {
  if (fabsf(gLive) > ScaleConfig::UNLOAD_CROSS_WIN_G) {
    if (fabsf(gLive) > 2.0f) g_ztLoadDetected = true;
    return false;
  }

  if (!g_ztLoadDetected) return false;

  if (slopeGps > ScaleConfig::UNLOAD_SLOPE_GPS_NEG) {
    return false;
  }

  if (!quietNow) {
    g_ztQuietStartMs = 0;
    return false;
  }

  uint32_t now = millis();
  if (g_ztQuietStartMs == 0) g_ztQuietStartMs = now;
  if ((now - g_ztQuietStartMs) < ScaleConfig::UNLOAD_STABLE_MS) return false;

  // Cooldown
  if (g_lastSnapMs != 0 && (now - g_lastSnapMs) < ScaleConfig::UNLOAD_COOLDOWN_MS) return false;

  // Snap
  float correction = gLive;
  if (correction > ScaleConfig::UNLOAD_SNAP_MAX_G)  correction = ScaleConfig::UNLOAD_SNAP_MAX_G;
  if (correction < -ScaleConfig::UNLOAD_SNAP_MAX_G) correction = -ScaleConfig::UNLOAD_SNAP_MAX_G;

  long deltaCnt = (long)(correction * g_scaleCpg);
  long maxCnt   = (long)(ScaleConfig::ZT_MAX_G * g_scaleCpg);

  g_ztCounts += deltaCnt;
  if (g_ztCounts > maxCnt)  g_ztCounts = maxCnt;
  if (g_ztCounts < -maxCnt) g_ztCounts = -maxCnt;

  g_ztLoadDetected = false;
  g_ztQuietStartMs = 0;
  g_lastSnapMs     = now;

  Serial.print(F("[ZT] snap: "));
  Serial.print(correction, 2);
  Serial.println(F(" g"));

  return true;
}

// ========================= TARA =========================

void tareAccumSample(long rawUse) {
  if (!g_tareActive) return;
  if (g_tareSampleCount >= 64) return;

  uint32_t now = millis();
  if ((now - g_tareStartMs) < ScaleConfig::TARE_SETTLE_MS) return;

  g_tareSamples[g_tareSampleCount++] = rawUse;
}

void tareStart(uint32_t nowMs) {
  g_tareActive      = true;
  g_tareStartMs     = nowMs;
  g_tareSampleCount = 0;
  g_tareUiActive    = true;
  g_tareUiStartMs   = nowMs;
  g_tareUiEndMs     = nowMs + ScaleConfig::TARE_UI_CLAMP_MS;
}

bool tareMaybeApply(uint32_t nowMs) {
  if (!g_tareActive) return false;
  if ((nowMs - g_tareStartMs) < ScaleConfig::TARE_UI_CLAMP_MS) return false;

  g_tareActive = false;

  if (g_tareSampleCount < ScaleConfig::TARE_MIN_SAMPLES) {
    Serial.print(F("[TARE] pochi campioni: "));
    Serial.println(g_tareSampleCount);

    if (g_tareSampleCount > 0) {
      int64_t sum = 0;
      for (uint8_t i = 0; i < g_tareSampleCount; i++) sum += g_tareSamples[i];
      g_offsetRaw = (long)(sum / g_tareSampleCount);
      g_ztCounts  = 0;
    }
    ScaleFilters::resetAll();
    return true;
  }

  // Insertion sort
  for (uint8_t i = 1; i < g_tareSampleCount; i++) {
    long key = g_tareSamples[i];
    int j = (int)i - 1;
    while (j >= 0 && g_tareSamples[j] > key) {
      g_tareSamples[j + 1] = g_tareSamples[j];
      j--;
    }
    g_tareSamples[j + 1] = key;
  }

  uint8_t trim = (g_tareSampleCount >= 40) ? (g_tareSampleCount / 10) : 0;
  uint8_t i0   = trim;
  uint8_t i1   = g_tareSampleCount - trim;

  int64_t s = 0;
  uint8_t useN = 0;
  for (uint8_t i = i0; i < i1; i++) {
    s += g_tareSamples[i];
    useN++;
  }

  if (useN == 0) {
    Serial.println(F("[TARE] errore useN=0"));
    return false;
  }

  g_offsetRaw = (long)(s / useN);
  g_ztCounts  = 0;

  Serial.print(F("[TARE] OK. OFFSET="));
  Serial.print(g_offsetRaw);
  Serial.print(F(" n="));
  Serial.println(useN);

  ScaleFilters::resetAll();
  resetFiltersAndState();
  return true;
}

bool isTareUiActive()        { return g_tareUiActive; }
void setTareUiActive(bool v) { g_tareUiActive = v; }
uint32_t getTareUiStartMs()  { return g_tareUiStartMs; }
void setTareUiStartMs(uint32_t ms) { g_tareUiStartMs = ms; }
uint32_t getTareUiEndMs()    { return g_tareUiEndMs; }
void setTareUiEndMs(uint32_t ms)   { g_tareUiEndMs = ms; }

// ========================= INATTIVITÀ =========================
static long g_inactLastWeightG = 0;
static bool g_weightActivityFlag = false;

void inactivityNoteWeightActivity(uint32_t nowMs, long gDispNow) {
  (void)nowMs;
  long delta = gDispNow - g_inactLastWeightG;
  if (delta < 0) delta = -delta;

  if (delta > ScaleConfig::INACTIVITY_WEIGHT_DELTA_G) {
    g_weightActivityFlag = true;
  }
  g_inactLastWeightG = gDispNow;
}

// Ritorna true una sola volta se c'è stata variazione peso (pattern one-shot)
bool popWeightActivity() {
  if (g_weightActivityFlag) {
    g_weightActivityFlag = false;
    return true;
  }
  return false;
}

// ========================= RESET =========================

void resetFiltersAndState() {
  ScaleFilters::resetAll();

  g_weighState   = STATE_UNSTABLE;
  g_gramsLatch   = 0.0f;
  g_dispUnstable = 0.0f;
  g_dispInit     = false;
  g_dispWorkLast = 0;
  g_dispLiveLast = 0;
  g_dispLiveHalfLast = 0;

  g_lastDispLive     = 0;
  g_wasZeroLive      = true;
  g_reversalLockLive = false;
  resetLiveHalfVote();

  g_lastDispWork = 0;
  g_wasZeroWork  = true;

  g_ztLoadDetected = false;
  g_ztQuietStartMs = 0;
  g_lastZtTickMs   = 0;
}

// ========================= NVS =========================
static Preferences g_prefs;

void loadFromNVS() {
  g_prefs.begin("hxscale", true);
  g_offsetRaw = g_prefs.getLong("offset_counts", ScaleConfig::DEFAULT_ZERO_RAW);
  g_scaleCpg  = g_prefs.getFloat("scale_cpg", ScaleConfig::defaultCpg());
  g_prefs.end();

  Serial.print(F("[NVS] OFFSET="));
  Serial.print(g_offsetRaw);
  Serial.print(F(" CPG="));
  Serial.println(g_scaleCpg, 4);
}

bool saveToNVS() {
  g_prefs.begin("hxscale", false);
  size_t wroteOffset = g_prefs.putLong("offset_counts", g_offsetRaw);
  size_t wroteScale  = g_prefs.putFloat("scale_cpg", g_scaleCpg);
  g_prefs.end();

  bool ok = (wroteOffset > 0 && wroteScale > 0);
  Serial.println(ok ? F("[NVS] Salvato") : F("[NVS] ERRORE save"));
  return ok;
}

// ========================= MODALITÀ PRESET =========================

void setMode(const char* mode) {
  if (!mode) return;

  if (strcmp(mode, "live") == 0 || strcmp(mode, "fine") == 0) {
    g_stEnable = false;
    g_ztEnable = false;
    ScaleFilters::setMA(ScaleConfig::MA_FINE);
    g_deadbandUnstable = ScaleConfig::DB_UNSTABLE_F;
    Serial.println(F("[MODE] LIVE"));
  } else {
    // Default: work
    g_stEnable = true;
    g_ztEnable = true;
    ScaleFilters::setMA(ScaleConfig::MA_DEFAULT);
    g_deadbandUnstable = ScaleConfig::DB_UNSTABLE_N;
    Serial.println(F("[MODE] WORK"));
  }

  resetFiltersAndState();
}

} // namespace ScaleState
