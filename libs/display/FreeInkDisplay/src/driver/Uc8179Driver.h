#pragma once

// UC8179 panel driver — Xteink X4 / X4 Pro, newer production run (800x480 B/W).
// UltraChip UC8179 driven in KW mode (PSR KW/R=1): 1-bpp, DTM1 = OLD plane,
// DTM2 = NEW plane, differential refresh — the same KW paradigm and command set
// as the UC8279d X3 driver. It is a *separate* driver because the UC8179 needs
// an explicit PLL / booster / VCOM bring-up that the UC8279d (pure-OTP) omits:
// on OTP defaults alone the UC8179 never develops an image.
//
// Recovered from the X4 Pro OEM firmware (UC8179_800x480 init FUN_4214dff8 /
// full-update FUN_4214e584, via Ghidra). It runs the factory OTP waveforms
// (PSR REG=0) — the MTP holds temperature-compensated LUT sets — so no custom
// LUT upload is needed; only the power rails are programmed here. PENDING
// HARDWARE VALIDATION on a UC8179 (screenType=1 / hw_calib=2) X4 / X4 Pro unit.
//
// BUSY_N: low while busy (PON/DRF/POF all flag). Production waits one RTOS tick
// and then polls until BUSY_N is HIGH; it does not require observing a LOW edge.

#include "PanelDriver.h"

namespace freeink {

struct Uc8179Config {
  // PSR (0x00) byte 0, as written at INIT. The OEM writes 0x3B here (REG bit5=1)
  // and re-asserts (psr0 & 0xDF)=0x1B (REG=0 -> OTP LUTs) just before every PON.
  uint8_t psr0;
  // PSR (0x00) byte 1.
  uint8_t psr1;
  // PFS power-off sequence (cmd 0x03). (PLL is 0x30 and stays panel-programmed.)
  uint8_t pfs;
  // Booster soft-start (cmd 0x06), 4 bytes.
  uint8_t btst[4];
  // Gate-scan selection (cmd 0xE1).
  uint8_t gateScan;
  // CCSET cascade/output enable (cmd 0xE0).
  uint8_t ccset;
  // TSSET forced temperature (cmd 0xE5) for a full refresh — selects the OTP
  // waveform's frame count/rate.
  uint8_t tsset;
  // TSSET (cmd 0xE5) for a fast/partial refresh (the OEM uses a different value).
  uint8_t tssetFast;
  // CDI (0x50) byte0 asserted during a refresh (before DRF); byte1 is 0x07.
  uint8_t cdiActive;
  // CDI (0x50) byte0 restored after the refresh completes; byte1 is 0x07.
  uint8_t cdiIdle;
  // TRES (0x61) gate count. The X4 Pro panel is addressed as 800x600 even though
  // only 480 rows are visible — the OTP waveform is tuned for the full 600-gate
  // scan, so the DTM transfer is padded to this height. (Visible height comes
  // from the board profile.)
  uint16_t tresHeight;
  // Power Save (cmd 0xE3). 0 disables the write; 0x22 is the UC8179 value used
  // by GxEPD2 to prevent artifacts with dithered images. Appended to preserve
  // the field order of existing aggregate board configurations.
  uint8_t powerSave;
};

const Uc8179Config& uc8179DefaultConfig();

class Uc8179Driver : public PanelDriver {
 public:
  explicit Uc8179Driver(const Uc8179Config& cfg = uc8179DefaultConfig());

  uint32_t spiHz() const override;
  BusyPolarity busyPolarity() const override { return BusyPolarity::UcIdleHigh; }
  PanelGeometry geometry() const override;

  void begin(EpdBus& bus) override;
  void deepSleep(EpdBus& bus) override;

  void display(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode, bool turnOff) override;
  bool displayStart(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode, bool turnOff) override;
  void displayFinish(EpdBus& bus, const uint8_t* fb) override;
  bool supportsAsyncDisplay() const override { return true; }

  void requestResync(uint8_t settlePasses) override;
  void skipInitialResync() override;

  // --- 4-level grayscale (anti-aliasing) ---
  // CrossPoint supplies two full 1bpp overlay masks. The driver combines them
  // with the displayed B/W base to recover Factory.bin's absolute 2-bit planes,
  // then sends plane0 -> DTM 0x10 and plane1 -> DTM 0x13. Full-buffer path only
  // (supportsStripGrayscale stays false; conversion needs the complete base).
  // display() routes the Fast B/W paint that follows an AA page through
  // transitionGrayscaleBase() (stock XTF_PRE_BW_MID). That waveform re-drives
  // every pixel from the retained previous base: changed pixels get the full
  // 25-frame single-polarity drive, and unchanged ones get the OEM's corrective
  // kick (4 frames toward black on KK, 1 toward white on WW) which cancels the
  // 2-frame gray drive the AA pass applied. The correction is per-pixel, so a
  // photo's large gray areas cost exactly what AA text edges cost. Hosts
  // therefore do not need to force a clean refresh after a grayscale page.
  bool fastAfterGrayscaleSafe() const override { return true; }
  void displayGrayscaleBase(EpdBus& bus, const uint8_t* fb, RefreshMode fallback, bool turnOff) override;
  void preconditionGrayscale(EpdBus& bus, uint16_t x, uint16_t y, uint16_t w, uint16_t h) override;
  void copyGrayscaleLsb(EpdBus& bus, const uint8_t* lsb) override;
  void copyGrayscaleMsb(EpdBus& bus, const uint8_t* msb) override;
  void displayGray(EpdBus& bus, const uint8_t* fb, bool turnOff, const unsigned char* lut, bool factoryMode) override;
  void cleanupGrayscaleBuffers(EpdBus& bus, const uint8_t* bw) override;

#ifdef FREEINK_UC8179_RAIL_POWEROFF
  // --- analog rail parking (booster / VGH / VGL / VSH / VSL / VCOM) ---
  // Nothing on this controller ever asks for turnOff (displayGray() discards it
  // for stock parity), so without these two hooks the rails latch ON at the
  // first refresh and stay on for the whole reading session. Registers, the
  // LUT-mode selection and both RAM planes survive a POF — only DSLP loses them,
  // which is why deepSleep() can already power off and back on freely.
  void controllerIdle(EpdBus& bus) override;
  void beginDisplayWork() override;
#endif

#ifdef FREEINK_UC8179_DOUBLE_GRAY_PRE
  void requestDeepGrayEqualize() override;
#endif

 private:
  void initController(EpdBus& bus);
  // Stream a framebuffer into a RAM plane (ramCmd): reverse row order, use PSR
  // SHL for horizontal panel direction, then pad to the addressed gate count.
  // Used for both NEW plane (0x13) and OLD-plane sync (0x10).
  void streamPlane(EpdBus& bus, uint8_t ramCmd, const uint8_t* fb, bool invert = false);
  // Stream lhs XOR rhs with the same orientation and white gate padding. Used
  // to translate CrossPoint's MSB transition mask into stock absolute plane1.
  void streamPlaneXor(EpdBus& bus, uint8_t ramCmd, const uint8_t* lhs, const uint8_t* rhs);
  // Run the vendor XTF_PRE_BW_MID transition with the previous B/W base in
  // DTM1 and the new base in DTM2. It replaces the ordinary B/W activation and
  // leaves analog power on for the AA pass that follows. `drfTag` names the
  // activation in the serial busy-wait timing print, so a repeated pass can be
  // measured separately.
  void runGrayscalePrecondition(EpdBus& bus, const char* drfTag = " 8179_gray_pre_DRF");
  // Blocking, non-flashing B/W transition used by a Fast page immediately
  // after AA. The generic reader path does not call displayGrayscaleBase(), so
  // display() routes its post-AA Fast base here as well.
  void transitionGrayscaleBase(EpdBus& bus, const uint8_t* fb, bool turnOff);
#ifdef FREEINK_UC8179_RAIL_POWEROFF
  // Ride out a prewarm PON that beginDisplayWork() fired without waiting. Every
  // path that touches the controller after a prewarm calls this first: the
  // UC8179 discards RAM/LUT/DRF writes while BUSY_N is low. That is an
  // invariant, not a convention — every public entry point that reaches the bus
  // (display*/copyGrayscale*/cleanupGrayscaleBuffers/deepSleep/controllerIdle)
  // opens with this call, so _ponPending can never survive into a bus write.
  // Free when nothing is pending.
  void settlePrewarm(EpdBus& bus);
#endif
#ifdef FREEINK_UC8179_LEAN_STREAMS
  // Stream the OLD-plane restore that transitionGrayscaleBase() deferred (see
  // _dtm1Stale). No-op when nothing is owed.
  void flushPendingOldPlane(EpdBus& bus);
#endif

  const Uc8179Config& _cfg;

  uint16_t _w;        // visible width (800)
  uint16_t _h;        // visible height (480)
  uint16_t _wb;       // width in bytes (100)
  uint16_t _tresH;    // addressed gate count (600) — DTM padded to this
  uint32_t _bufferSize;

  // Stock Factory.bin uses absolute AA planes and derives its B/W base as
  // plane0 & plane1. CrossPoint supplies overlay masks after displaying the B/W
  // base separately, so preserve that base in X4 Pro PSRAM and fold it into the
  // masks. The resulting (plane0,plane1) selectors are black=(0,0), dark=(1,0),
  // light=(0,1), white=(1,1). The allocation temporarily holds absolute plane0,
  // then copyGrayscaleMsb() recovers the clean B/W base for stock's RAM restore.
  uint8_t* _grayBase = nullptr;
  bool _grayBaseValid = false;
  bool _absoluteGrayPlanes = false;

  bool _isScreenOn = false;
  // Force the first refresh after begin() to a full flash, so a partial update
  // never runs against an unknown on-screen state (e.g. a retained boot image).
  bool _needFullClear = true;
  // True once the OLD plane (0x10) holds a valid previous displayed frame, so a
  // differential partial or stock AA base transition has a real baseline.
  bool _oldPlaneValid = false;
  // True when both controller planes have been restored to the displayed B/W
  // base. False while an ordinary refresh or AA selector upload is in flight.
  bool _bwPlanesSynced = false;
  // Set after every grayscale refresh. The next ordinary Fast B/W paint uses
  // stock's non-flashing XTF_PRE_BW_MID transition instead of DU. Explicit Half
  // remains the complement-driven GC scrub for periodic and sleep cleanup.
  bool _redriveAfterGray = false;
  // Tracks whether the first AA page has completed; Factory.bin skips the
  // XTF_PRE_BW_MID pre-pass only for that first page. AA activation itself uses
  // CDI 0x29 every time; 0xA9 is restored only after B/W/preconditioning passes.
  bool _grayRefreshedOnce = false;

  // Async split state (see Uc8279Driver for the contract).
  bool _pendingRefresh = false;
  bool _pendingTurnOff = false;
  bool _pendingPartial = false;  // this refresh used the PTIN/PTOUT partial path

#ifdef FREEINK_UC8179_RAIL_POWEROFF
  // beginDisplayWork() has no bus parameter (PanelDriver's signature), so the
  // bus handed to begin() is kept for the prewarm PON. The facade owns exactly
  // one bus for the lifetime of the driver.
  EpdBus* _bus = nullptr;
  // A PON was commanded but not waited on, so the ~127 ms rail ramp overlaps the
  // host's CPU-side page composition. Cleared by settlePrewarm().
  bool _ponPending = false;
#endif
#ifdef FREEINK_UC8179_LEAN_STREAMS
  // The controller's OLD plane (DTM1) does NOT hold the most recently displayed
  // B/W base: transitionGrayscaleBase() skipped its restore because an AA pass
  // normally overwrites DTM1 within microseconds anyway. Strictly orthogonal to
  // _oldPlaneValid, which says a legal baseline exists at all: with _dtm1Stale
  // set, DTM1 still holds the page BEFORE the one just painted, and the owed
  // frame lives in _grayBase. Any consumer that reads DTM1 as the OLD plane
  // flushes first; any write to DTM1 discharges the debt.
  bool _dtm1Stale = false;
#endif
#ifdef FREEINK_UC8179_DOUBLE_GRAY_PRE
  // One-shot: the next base transition runs the XTF_PRE_BW_MID corrective a
  // second time with DTM1 == DTM2, equalizing black depth so the previous page's
  // text cannot ghost through this page's grays (see transitionGrayscaleBase()).
  // Set by requestDeepGrayEqualize(), which the host calls just before the base
  // display of an image page.
  //
  // Staleness is impossible by construction rather than by convention: the only
  // two functions in this driver that activate a B/W base are
  // transitionGrayscaleBase() and displayStart(), and BOTH clear the flag —
  // the first by consuming it, the second by discarding it. display() picks
  // exactly one of the two; displayGrayscaleBase() and displayWindow() route
  // through display(); and initController() clears it across begin()/wake. So
  // the flag is always answered by the first base activation after it is set,
  // and no later page can observe it. Nothing between the hint and that
  // activation can activate a base either: displayGray(), copyGrayscale*() and
  // cleanupGrayscaleBuffers() all follow the base, never precede it.
  bool _deepEqualizeNext = false;
#endif
};

PanelDriver& uc8179Driver();

}  // namespace freeink
