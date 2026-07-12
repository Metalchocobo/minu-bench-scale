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
static TareMode g_tareMode        = TARE_MODE_MANUAL;
static TareUiState g_tareUiState  = TARE_UI_IDLE;
static long     g_tareSamples[ScaleConfig::TARE_SAMPLE_BUF_N];
static uint32_t g_tareSampleMs[ScaleConfig::TARE_SAMPLE_BUF_N];
static uint8_t  g_tareSampleCount = 0;
static uint8_t  g_tareStableSampleCount = 0;
static uint32_t g_tareLastEvalSampleMs = 0;

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

struct TareParams {
  uint32_t settleMs;
  uint32_t minMs;
  uint32_t maxMs;
  uint8_t minSamples;
  float maxRangeG;
  float maxSlopeGps;
};

static TareParams tareParamsForMode(TareMode mode) {
  (void)mode;

  return {
    ScaleConfig::TARE_MANUAL_SETTLE_MS,
    ScaleConfig::TARE_MANUAL_MIN_MS,
    ScaleConfig::TARE_MANUAL_MAX_MS,
    ScaleConfig::TARE_MANUAL_WINDOW_SAMPLES,
    ScaleConfig::TARE_MANUAL_RANGE_G,
    ScaleConfig::TARE_MANUAL_SLOPE_GPS
  };
}

static long tareWinsorize(long value, long low, long high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

static bool tareBuildStableOffset(const TareParams& p, long* offsetOut, float* rangeGOut, float* slopeGpsOut, uint8_t* useNOut) {
  if (offsetOut) *offsetOut = 0;
  if (rangeGOut) *rangeGOut = 999999.0f;
  if (slopeGpsOut) *slopeGpsOut = 999999.0f;
  if (useNOut) *useNOut = 0;

  uint8_t n = p.minSamples;
  if (g_tareSampleCount < n || n > ScaleConfig::TARE_SAMPLE_BUF_N) return false;
  if (g_scaleCpg <= 0.01f) return false;

  uint8_t start = g_tareSampleCount - n;
  for (uint8_t i = 1; i < n; i++) {
    uint32_t sampleGapMs = g_tareSampleMs[start + i] - g_tareSampleMs[start + i - 1];
    if (sampleGapMs > ScaleConfig::TARE_MANUAL_SAMPLE_MAX_AGE_MS) return false;
  }
  uint32_t windowSpanMs = g_tareSampleMs[start + n - 1] - g_tareSampleMs[start];
  if (windowSpanMs > ScaleConfig::TARE_MANUAL_WINDOW_MAX_MS) return false;

  long sorted[ScaleConfig::TARE_SAMPLE_BUF_N];
  for (uint8_t i = 0; i < n; i++) sorted[i] = g_tareSamples[start + i];

  for (uint8_t i = 1; i < n; i++) {
    long key = sorted[i];
    int j = (int)i - 1;
    while (j >= 0 && sorted[j] > key) {
      sorted[j + 1] = sorted[j];
      j--;
    }
    sorted[j + 1] = key;
  }

  uint8_t trim = ScaleConfig::TARE_MANUAL_TRIM_SAMPLES;
  if ((uint8_t)(trim * 2) >= n) return false;
  uint8_t i0 = trim;
  uint8_t i1 = n - trim;
  if (i1 <= i0) return false;

  long centralMin = sorted[i0];
  long centralMax = sorted[i1 - 1];
  float rangeG = fabsf((float)(centralMax - centralMin) / g_scaleCpg);

  int64_t sum = 0;
  uint8_t useN = 0;
  for (uint8_t i = i0; i < i1; i++) {
    sum += sorted[i];
    useN++;
  }
  if (useN == 0) return false;

  uint8_t edgeN = ScaleConfig::TARE_MANUAL_EDGE_SAMPLES;
  if ((uint8_t)(edgeN * 2) > n) edgeN = n / 2;
  if (edgeN == 0) return false;

  int64_t firstSum = 0;
  int64_t lastSum = 0;
  uint32_t firstTimeSum = 0;
  uint32_t lastTimeSum = 0;
  uint32_t baseMs = g_tareSampleMs[start];
  for (uint8_t i = 0; i < edgeN; i++) {
    uint8_t firstIdx = start + i;
    uint8_t lastIdx = start + n - edgeN + i;
    firstSum += tareWinsorize(g_tareSamples[firstIdx], centralMin, centralMax);
    lastSum += tareWinsorize(g_tareSamples[lastIdx], centralMin, centralMax);
    firstTimeSum += g_tareSampleMs[firstIdx] - baseMs;
    lastTimeSum += g_tareSampleMs[lastIdx] - baseMs;
  }

  float firstAvg = (float)firstSum / (float)edgeN;
  float lastAvg = (float)lastSum / (float)edgeN;
  uint32_t firstCenterMs = firstTimeSum / edgeN;
  uint32_t lastCenterMs = lastTimeSum / edgeN;
  uint32_t dtMs = lastCenterMs - firstCenterMs;
  float slopeGps = 999999.0f;
  if (dtMs > 0) {
    float deltaG = (lastAvg - firstAvg) / g_scaleCpg;
    slopeGps = fabsf(deltaG) / ((float)dtMs / 1000.0f);
  }

  if (rangeGOut) *rangeGOut = rangeG;
  if (slopeGpsOut) *slopeGpsOut = slopeGps;
  if (useNOut) *useNOut = useN;

  if (rangeG > p.maxRangeG) return false;
  if (slopeGps > p.maxSlopeGps) return false;

  if (offsetOut) *offsetOut = (long)(sum / useN);
  return true;
}

void tareAccumSample(long rawUse) {
  if (!g_tareActive) return;

  uint32_t now = millis();
  TareParams p = tareParamsForMode(g_tareMode);
  if ((now - g_tareStartMs) < p.settleMs) return;

  uint8_t idx = g_tareSampleCount;
  if (g_tareSampleCount >= ScaleConfig::TARE_SAMPLE_BUF_N) {
    for (uint8_t i = 1; i < ScaleConfig::TARE_SAMPLE_BUF_N; i++) {
      g_tareSamples[i - 1] = g_tareSamples[i];
      g_tareSampleMs[i - 1] = g_tareSampleMs[i];
    }
    idx = ScaleConfig::TARE_SAMPLE_BUF_N - 1;
  } else {
    g_tareSampleCount++;
  }

  g_tareSamples[idx] = rawUse;
  g_tareSampleMs[idx] = now;
}

void tareStart(uint32_t nowMs, TareMode mode) {
  TareParams p = tareParamsForMode(mode);

  g_tareActive      = true;
  g_tareMode        = mode;
  g_tareStartMs     = nowMs;
  g_tareSampleCount = 0;
  g_tareStableSampleCount = 0;
  g_tareLastEvalSampleMs = 0;
  g_tareUiActive    = true;
  g_tareUiState     = TARE_UI_SETTLING;
  g_tareUiStartMs   = nowMs;
  g_tareUiEndMs     = nowMs + p.maxMs + ScaleConfig::TARE_FAIL_HOLD_MS;
}

bool tareApplyWorking(long rawOffset, uint32_t nowMs) {
  if (g_tareActive || rawOffset == 0 || g_scaleCpg <= 0.01f) return false;

  g_tareActive      = false;
  g_tareMode        = TARE_MODE_AUTO;
  g_tareStartMs     = nowMs;
  g_tareSampleCount = 0;
  g_tareStableSampleCount = 0;
  g_tareLastEvalSampleMs = 0;
  g_tareUiActive    = true;
  g_tareUiState     = TARE_UI_OK;
  g_tareUiStartMs   = nowMs;
  g_tareUiEndMs     = nowMs + ScaleConfig::TARE_OK_HOLD_MS;

  g_offsetRaw = rawOffset;
  g_ztCounts = 0;

  Serial.print(F("[TARE] WORK off="));
  Serial.println(g_offsetRaw);

  resetFiltersAndState();
  return true;
}

TareResult tareUpdate(uint32_t nowMs) {
  if (!g_tareActive) return TARE_RESULT_IDLE;

  TareParams p = tareParamsForMode(g_tareMode);
  uint32_t elapsedMs = nowMs - g_tareStartMs;

  long newOffset = 0;
  float rangeG = 0.0f;
  float slopeGps = 0.0f;
  uint8_t useN = 0;

  // Keep maxMs a hard deadline even if a delayed loop brings a valid sample.
  if (elapsedMs >= p.maxMs) {
    g_tareActive = false;
    g_tareUiState = TARE_UI_FAILED;
    g_tareUiEndMs = nowMs + ScaleConfig::TARE_FAIL_HOLD_MS;

    (void)tareBuildStableOffset(p, &newOffset, &rangeG, &slopeGps, &useN);
    Serial.print(F("[TARE] FAIL t="));
    Serial.print(elapsedMs);
    Serial.print(F(" n="));
    Serial.print(g_tareSampleCount);
    Serial.print(F(" r="));
    Serial.print(rangeG, 2);
    Serial.print(F(" s="));
    Serial.println(slopeGps, 2);
    return TARE_RESULT_FAILED;
  }

  bool stableNow = false;
  bool hasNewSample = false;
  if (g_tareSampleCount > 0) {
    uint32_t latestSampleMs = g_tareSampleMs[g_tareSampleCount - 1];
    hasNewSample = (latestSampleMs != g_tareLastEvalSampleMs);
    if (hasNewSample) {
      bool gapFresh = g_tareLastEvalSampleMs == 0 ||
        (uint32_t)(latestSampleMs - g_tareLastEvalSampleMs) <= ScaleConfig::TARE_MANUAL_SAMPLE_MAX_AGE_MS;
      g_tareLastEvalSampleMs = latestSampleMs;
      bool sampleFresh = (uint32_t)(nowMs - latestSampleMs) <= ScaleConfig::TARE_MANUAL_SAMPLE_MAX_AGE_MS;
      if (gapFresh && sampleFresh && elapsedMs >= p.minMs) {
        stableNow = tareBuildStableOffset(p, &newOffset, &rangeG, &slopeGps, &useN);
      }
      g_tareStableSampleCount = stableNow ? (uint8_t)(g_tareStableSampleCount + 1) : 0;
    }
  }

  if (hasNewSample && stableNow &&
      g_tareStableSampleCount >= ScaleConfig::TARE_MANUAL_STABLE_SAMPLES) {
    g_tareActive = false;
    g_tareUiState = TARE_UI_OK;
    g_tareUiEndMs = nowMs + ScaleConfig::TARE_OK_HOLD_MS;

    g_offsetRaw = newOffset;
    g_ztCounts = 0;

    Serial.print(F("[TARE] OK t="));
    Serial.print(elapsedMs);
    Serial.print(F(" off="));
    Serial.print(g_offsetRaw);
    Serial.print(F(" n="));
    Serial.print(useN);
    Serial.print(F(" r="));
    Serial.print(rangeG, 2);
    Serial.print(F(" s="));
    Serial.println(slopeGps, 2);

    resetFiltersAndState();
    return TARE_RESULT_APPLIED;
  }

  return TARE_RESULT_PENDING;
}

bool tareMaybeApply(uint32_t nowMs) {
  return tareUpdate(nowMs) == TARE_RESULT_APPLIED;
}

bool isTareActive()          { return g_tareActive; }
bool isTareUiActive()        { return g_tareUiActive; }
void setTareUiActive(bool v) {
  g_tareUiActive = v;
  if (!v && !g_tareActive) g_tareUiState = TARE_UI_IDLE;
}
uint32_t getTareUiStartMs()  { return g_tareUiStartMs; }
void setTareUiStartMs(uint32_t ms) { g_tareUiStartMs = ms; }
uint32_t getTareUiEndMs()    { return g_tareUiEndMs; }
void setTareUiEndMs(uint32_t ms)   { g_tareUiEndMs = ms; }
TareMode getTareMode()       { return g_tareMode; }
TareUiState getTareUiState() { return g_tareUiState; }

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
