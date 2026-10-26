// Real EpdBus::waitBusy() on the UcIdleHigh (X4 Pro UC8179/UC8279) path, against
// a simulated clock: BUSY_N is a pure function of time, and time advances only
// through the waits the bus itself performs, so every return instant is exact.
#include <cassert>
#include <cstdio>
#include <vector>

#include "bus/EpdBus.h"

using freeink::BusyPolarity;
using freeink::EpdBus;

namespace {
struct Window {
  uint64_t from, to;  // [from, to) in microseconds
};
// BUSY_N: HIGH (idle) outside [assertUs, releaseUs), LOW inside except blips.
struct BusyScript {
  uint64_t assertUs = 0, releaseUs = 0;
  std::vector<Window> highBlips;
} script;
}  // namespace

int hostBusyAt(uint64_t us) {
  if (us < script.assertUs || us >= script.releaseUs) return HIGH;
  for (const auto& b : script.highBlips)
    if (us >= b.from && us < b.to) return HIGH;
  return LOW;
}

static EpdBus& bus() {
  static EpdBus b;
  static bool begun = false;
  if (!begun) {
    hostBusyPin = 6;
    b.begin({12, 11, 13, 18, 14, 6}, 10000000, BusyPolarity::UcIdleHigh);
    begun = true;
  }
  return b;
}

// Issue a bare activation opcode at t=0 under `s`, then run the wait.
static bool activate(const BusyScript& s, bool refreshWait = false) {
  script = s;
  hostMicros = 0;
  bus().cmd(0x12);
  if (refreshWait) {
    bus().waitRefreshComplete(" refresh");
    return true;
  }
  return bus().waitBusy(" activation");
}

int main() {
  // Every scenario asserts at 0.5 ms and releases at 300 ms; the wait samples
  // BUSY_N once per millisecond, so its first idle read lands at 300.000 ms.
  BusyScript clean{500, 300000, {}};

  // A genuine release ends the wait after exactly two confirmation samples:
  // 200 us of added latency, nothing else.
  assert(activate(clean));
  assert(hostMicros == 300000 + 200);
  assert(freeink::takeBusyGlitchCount() == 0);

  // A one-sample HIGH blip mid-waveform no longer ends the wait (it used to
  // return at 100 ms, mid-waveform, and the next DTM1 sync was discarded).
  BusyScript blip = clean;
  blip.highBlips = {{100000, 100001}};
  assert(activate(blip));
  assert(hostMicros >= 300000 && hostMicros <= 301200);

  // Two consecutive HIGH samples (a 150 us blip) are still rejected.
  blip.highBlips = {{100000, 100150}};
  assert(activate(blip));
  assert(hostMicros >= 300000 && hostMicros <= 301200);

  // Repeated blips, each rejected, still converge on the real release. (Each
  // rejection shifts the 1 ms sample grid by the confirmation samples it took,
  // hence the offsets: every blip lands on a sample.)
  blip.highBlips = {{50000, 50001}, {120100, 120220}, {250300, 250301}};
  assert(activate(blip));
  assert(hostMicros >= 300000 && hostMicros <= 301200);

  // The refresh-completion entry point shares the same loop.
  blip.highBlips = {{100000, 100001}};
  activate(blip, true);
  assert(hostMicros >= 300000 && hostMicros <= 301200);
  // Every rejection above is counted for the host's field log (1+1+3+1), even
  // though no Serial line need ever print; take() drains.
  assert(freeink::takeBusyGlitchCount() == 6 && freeink::takeBusyGlitchCount() == 0);
  freeink::noteLostActivation();
  freeink::noteLostActivation();
  assert(freeink::takeLostActivationCount() == 2 && freeink::takeLostActivationCount() == 0);

  // Lost activation: BUSY_N never asserts after the bare opcode. Unchanged —
  // the 50 ms assertion grace runs out and the wait reports no BUSY seen.
  assert(!activate(BusyScript{0, 0, {}}));
  assert(hostMicros >= 50000 && hostMicros < 52000);

  // Stuck-low line: the 30 s runaway backstop still fires (no hang).
  assert(activate(BusyScript{500, ~0ull, {}}));
  assert(hostMicros > 30000000 && hostMicros < 30002000);

  // A readiness wait after DATA (no bare opcode) with the controller idle is
  // still one tick and one sample: no grace, no confirmation samples.
  script = clean;
  script.assertUs = script.releaseUs = 0;
  hostMicros = 0;
  bus().cmd(0x10);
  bus().data(0x00);
  hostBusyReads = 0;
  assert(!bus().waitBusy(" ready"));
  assert(hostMicros == 1000 && hostBusyReads == 1);

  std::puts("EpdBus UcIdleHigh release debounce, assertion grace and backstop passed");
}
