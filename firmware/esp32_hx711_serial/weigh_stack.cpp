#include "weigh_stack.h"

namespace WeighStack {

// ========================= INTERNAL STATE =========================
static Entry g_stack[MAX_ITEMS];
static int   g_count = 0;

// Reference offset (raw counts at manual TARE)
static long g_refOffset    = 0;
static bool g_hasReference = false;

// ========================= STACK OPERATIONS =========================

bool push(float grams) {
  return pushCommit(grams, 0, 0, nullptr, 0, nullptr);
}

bool pushCommit(float grams, long oldOffset, long oldZt,
                const char* uuid, uint32_t productId, const char* sessionId) {
  if (g_count >= MAX_ITEMS) return false;
  Entry& entry = g_stack[g_count++];
  memset(&entry, 0, sizeof(entry));
  entry.grams = grams;
  entry.oldOffset = oldOffset;
  entry.oldZt = oldZt;
  entry.productId = productId;
  if (uuid) strlcpy(entry.uuid, uuid, sizeof(entry.uuid));
  if (sessionId) strlcpy(entry.sessionId, sessionId, sizeof(entry.sessionId));
  return true;
}

bool setLastConfirmResponseId(const char* responseId) {
  if (g_count <= 0 || !responseId || responseId[0] == '\0') return false;
  strlcpy(g_stack[g_count - 1].confirmResponseId, responseId,
          sizeof(g_stack[g_count - 1].confirmResponseId));
  return true;
}

const Entry* peek() {
  return (g_count > 0) ? &g_stack[g_count - 1] : nullptr;
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
    sum += g_stack[i].grams;
  }
  return sum;
}

int count() {
  return g_count;
}

float get(int index) {
  if (index < 0 || index >= g_count) return 0.0f;
  return g_stack[index].grams;
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
