#include "weigh_stack.h"

namespace WeighStack {

// ========================= INTERNAL STATE =========================
static float g_stack[MAX_ITEMS];
static int   g_count = 0;

// Reference offset (raw counts at manual TARE)
static long g_refOffset    = 0;
static bool g_hasReference = false;

// ========================= STACK OPERATIONS =========================

bool push(float grams) {
  if (g_count >= MAX_ITEMS) return false;
  g_stack[g_count++] = grams;
  return true;
}

bool pop() {
  if (g_count <= 0) return false;
  g_count--;
  return true;
}

void clear() {
  g_count = 0;
}

float total() {
  float sum = 0.0f;
  for (int i = 0; i < g_count; i++) {
    sum += g_stack[i];
  }
  return sum;
}

int count() {
  return g_count;
}

float get(int index) {
  if (index < 0 || index >= g_count) return 0.0f;
  return g_stack[index];
}

// ========================= REFERENCE OFFSET =========================

void setReferenceOffset(long rawOffset) {
  g_refOffset = rawOffset;
  g_hasReference = true;
}

long getReferenceOffset() {
  return g_refOffset;
}

bool hasReference() {
  return g_hasReference;
}

void clearReference() {
  g_refOffset = 0;
  g_hasReference = false;
}

} // namespace WeighStack
