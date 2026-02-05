#pragma once

#include <Arduino.h>
#include "config/config_scale.h"

// =============================================================================
// SCALE_FILTERS.H - Filtri per bilancia (mediana, MA, spike guard, storico)
// =============================================================================

namespace ScaleFilters {

// ========================= MEDIANA 3 =========================
void   resetMedian();
long   pushMedian3(long x);

// ========================= MEDIA MOBILE (MA) =========================
void   resetMA();
void   setMA(int n);
int    getMA();
long   pushMA(long x);

// ========================= STORICO (range/slope) =========================
void   resetHist();
void   pushHist(float g);
float  histAt(int k);
float  rangeLastNSamples(int n);
float  slopeLastNSamples(int n, float sampleMs = ScaleConfig::SAMPLE_MS);
int    histCount();

// ========================= ANTI-SPIKE GUARD =========================
// Ritorna true se il campione è valido, false se scartato
// outRaw contiene il valore da usare (o invariato se scartato)
bool   rawSpikeGuard(long rawMed, uint32_t nowMs, float scaleCpg, long* outRaw);
void   resetSpikeGuard();

// ========================= RESET COMPLETO =========================
void   resetAll();

} // namespace ScaleFilters
