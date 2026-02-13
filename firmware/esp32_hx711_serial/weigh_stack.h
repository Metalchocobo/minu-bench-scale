#pragma once

#include <Arduino.h>

// =============================================================================
// WEIGH_STACK.H - Local weigh stack (LIFO), RAM only, not persistent
// =============================================================================
// Stores individual ingredient weights pushed via ENTER key.
// Stack resets on TARE (new session) or CLEAR long press.
// Lost on reboot — this is correct behavior.

namespace WeighStack {

static const int MAX_ITEMS = 50;

// Push a weight onto the stack. Returns true if pushed, false if full.
bool push(float grams);

// Pop the last weight from the stack. Returns true if popped, false if empty.
bool pop();

// Clear the entire stack.
void clear();

// Get the sum of all weights in the stack.
float total();

// Get the number of items in the stack.
int count();

// Get a specific item by index (0 = oldest, count-1 = newest).
// Returns 0.0 if index is out of range.
float get(int index);

// Reference offset (raw counts at the time of manual TARE)
void   setReferenceOffset(long rawOffset);
long   getReferenceOffset();
bool   hasReference();
void   clearReference();

} // namespace WeighStack
