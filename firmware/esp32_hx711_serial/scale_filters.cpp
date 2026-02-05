#include "scale_filters.h"
#include <math.h>

namespace ScaleFilters {

// ========================= MEDIANA 3 =========================
static long g_medBuf[3] = {0, 0, 0};
static int  g_medIdx    = 0;
static int  g_medCount  = 0;

static long median3(long a, long b, long c) {
  long M = max(a, max(b, c));
  long m = min(a, min(b, c));
  return a + b + c - M - m;
}

void resetMedian() {
  g_medIdx   = 0;
  g_medCount = 0;
  g_medBuf[0] = g_medBuf[1] = g_medBuf[2] = 0;
}

long pushMedian3(long x) {
  g_medBuf[g_medIdx] = x;
  g_medIdx = (g_medIdx + 1) % 3;
  if (g_medCount < 3) g_medCount++;

  if (g_medCount < 3) {
    long s = 0;
    for (int i = 0; i < g_medCount; i++) s += g_medBuf[i];
    return s / (g_medCount ? g_medCount : 1);
  }
  return median3(g_medBuf[0], g_medBuf[1], g_medBuf[2]);
}

// ========================= MEDIA MOBILE (MA) =========================
static long g_maBuf[8] = {0};
static int  g_maIdx    = 0;
static int  g_maCount  = 0;
static int  g_maN      = ScaleConfig::MA_DEFAULT;
static long g_maSum    = 0;

void resetMA() {
  g_maIdx   = 0;
  g_maCount = 0;
  g_maSum   = 0;
  for (int i = 0; i < 8; i++) g_maBuf[i] = 0;
}

void setMA(int n) {
  g_maN = constrain(n, 1, 8);
  resetMA();
}

int getMA() {
  return g_maN;
}

long pushMA(long x) {
  if (g_maCount == g_maN) {
    g_maSum -= g_maBuf[g_maIdx];
  } else {
    g_maCount++;
  }
  g_maBuf[g_maIdx] = x;
  g_maSum += x;
  g_maIdx++;
  if (g_maIdx >= g_maN) g_maIdx = 0;
  return g_maSum / (g_maCount ? g_maCount : 1);
}

// ========================= STORICO =========================
static const int MAX_HIST = 64;
static float g_gHist[MAX_HIST];
static int   g_hIdx   = 0;
static int   g_hCount = 0;

void resetHist() {
  g_hIdx   = 0;
  g_hCount = 0;
}

void pushHist(float g) {
  g_gHist[g_hIdx] = g;
  g_hIdx = (g_hIdx + 1) % MAX_HIST;
  if (g_hCount < MAX_HIST) g_hCount++;
}

float histAt(int k) {
  if (g_hCount == 0) return 0.0f;
  k = constrain(k, 0, min(g_hCount, MAX_HIST) - 1);
  int idx = (g_hIdx - 1 - k + MAX_HIST) % MAX_HIST;
  return g_gHist[idx];
}

float rangeLastNSamples(int n) {
  n = min(n, g_hCount);
  if (n <= 0) return 0.0f;
  float mn = histAt(0), mx = mn;
  for (int i = 1; i < n; i++) {
    float v = histAt(i);
    if (v < mn) mn = v;
    if (v > mx) mx = v;
  }
  return mx - mn;
}

float slopeLastNSamples(int n, float sampleMs) {
  n = min(n, g_hCount);
  if (n < 2) return 0.0f;
  float last  = histAt(0);
  float first = histAt(n - 1);
  float dt    = (float)(n - 1) * (sampleMs / 1000.0f);
  if (dt <= 0.0f) return 0.0f;
  return (last - first) / dt; // g/s
}

int histCount() {
  return g_hCount;
}

// ========================= ANTI-SPIKE GUARD =========================
static bool     g_hasLastGood = false;
static long     g_lastGood    = 0;
static bool     g_pending     = false;
static long     g_pendingRaw  = 0;
static uint32_t g_pendingMs   = 0;

void resetSpikeGuard() {
  g_hasLastGood = false;
  g_lastGood    = 0;
  g_pending     = false;
  g_pendingRaw  = 0;
  g_pendingMs   = 0;
}

bool rawSpikeGuard(long rawMed, uint32_t nowMs, float scaleCpg, long* outRaw) {
  if (!outRaw) return true;

  // Se non calibrati, non bloccare
  if (scaleCpg <= 0.01f) {
    *outRaw = rawMed;
    return true;
  }

  if (!g_hasLastGood) {
    g_hasLastGood = true;
    g_lastGood    = rawMed;
    g_pending     = false;
    *outRaw       = rawMed;
    return true;
  }

  // Timeout pending
  if (g_pending && (uint32_t)(nowMs - g_pendingMs) > ScaleConfig::RAW_SPIKE_TIMEOUT_MS) {
    g_pending = false;
  }

  float dG = fabsf((float)(rawMed - g_lastGood) / scaleCpg);

  if (!g_pending) {
    if (dG >= ScaleConfig::RAW_SPIKE_JUMP_G) {
      // Suspect: drop e aspetta conferma
      g_pending    = true;
      g_pendingRaw = rawMed;
      g_pendingMs  = nowMs;
      return false;
    }
    g_lastGood = rawMed;
    *outRaw    = rawMed;
    return true;
  }

  // Pending: accetta se conferma o se anche questo è un salto
  float dPendingG = fabsf((float)(rawMed - g_pendingRaw) / scaleCpg);
  if (dPendingG <= ScaleConfig::RAW_SPIKE_CONFIRM_G || dG >= ScaleConfig::RAW_SPIKE_JUMP_G) {
    g_pending  = false;
    g_lastGood = rawMed;
    *outRaw    = rawMed;
    return true;
  }

  // Era un glitch singolo
  g_pending  = false;
  g_lastGood = rawMed;
  *outRaw    = rawMed;
  return true;
}

// ========================= RESET COMPLETO =========================
void resetAll() {
  resetMedian();
  resetMA();
  resetHist();
  resetSpikeGuard();
}

} // namespace ScaleFilters
