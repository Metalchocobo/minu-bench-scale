#pragma once

template <typename PollStep, typename HasPendingInput>
inline unsigned mqttPollInboundBounded(
    PollStep pollStep, HasPendingInput hasPendingInput, unsigned maxPasses) {
  if (maxPasses == 0) return 0;
  unsigned passes = 0;
  do {
    pollStep();
    passes += 1;
  } while (passes < maxPasses && hasPendingInput());
  return passes;
}

template <typename TransportUpdate, typename FallbackUpdate>
inline void mqttRunUpdateCycle(
    TransportUpdate transportUpdate, FallbackUpdate fallbackUpdate) {
  transportUpdate();
  fallbackUpdate();
}
