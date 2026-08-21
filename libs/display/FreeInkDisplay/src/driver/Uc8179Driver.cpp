#include "Uc8179Driver.h"

#include <Arduino.h>

#include <string.h>

#include <BoardConfig.h>
#if defined(BOARD_HAS_PSRAM)
#include <esp_heap_caps.h>
#endif

namespace freeink {
namespace {
// UC8179 command set (UC8179 datasheet + OEM UC8179_800x480 stream, via Ghidra).
constexpr uint8_t CMD_PANEL_SETTING = 0x00;       // PSR
constexpr uint8_t CMD_POWER_OFF = 0x02;           // POF
constexpr uint8_t CMD_PFS = 0x03;                 // PFS (power-off sequence; PLL is 0x30)
constexpr uint8_t CMD_POWER_ON = 0x04;            // PON
constexpr uint8_t CMD_BOOSTER_SOFT_START = 0x06;  // BTST
constexpr uint8_t CMD_DEEP_SLEEP = 0x07;          // DSLP (check code 0xA5)
constexpr uint8_t CMD_DTM1 = 0x10;                // OLD plane in KW mode
constexpr uint8_t CMD_DTM2 = 0x13;                // NEW plane in KW mode
constexpr uint8_t CMD_DISPLAY_REFRESH = 0x12;     // DRF
constexpr uint8_t CMD_PARTIAL_WINDOW = 0x90;      // PTL
constexpr uint8_t CMD_PARTIAL_IN = 0x91;          // PTIN (partial refresh in)
constexpr uint8_t CMD_PARTIAL_OUT = 0x92;         // PTOUT (partial refresh out)
constexpr uint8_t CMD_VCOM_DATA_INTERVAL = 0x50;  // CDI
constexpr uint8_t CMD_RESOLUTION = 0x61;          // TRES
constexpr uint8_t CMD_GATE_SOURCE_START = 0x65;   // GSST (4 data bytes)
constexpr uint8_t CMD_CCSET = 0xE0;               // CCSET (cascade/output enable)
constexpr uint8_t CMD_GATE_SCAN = 0xE1;           // gate-scan selection
constexpr uint8_t CMD_POWER_SAVE = 0xE3;          // PWS (VCOM/source line periods)
constexpr uint8_t CMD_TSSET = 0xE5;               // TSSET (forced temperature; frame-rate lever)

constexpr uint8_t CDI_INTERVAL = 0x07;  // CDI byte1, constant

// 4-level grayscale (AA) waveform LUTs — stock's REAL grayscale set (the short
// 2-frame LUTs FUN_4214ebd0 actually uploads @app1 DROM 0x3c5d8994..), uploaded
// in custom-LUT mode (PSR REG=1). NOTE: unlike the (dead, grainy) gray_full set,
// here the register command is sent SEPARATELY — blob byte0 is DATA, not the cmd.
// Each LUT is 42 (0x2A) data bytes; only the first ~12 are non-zero. Level select
// by (old=0x10/LSB, new=0x13/MSB): (0,0)=LUTKK black, (0,1)=LUTKW, (1,0)=LUTWK,
// (1,1)=LUTWW white. This is the byte-exact stock set; CrossPoint's overlay-mask
// representation is converted to these absolute selectors before upload rather
// than modifying the waveform.
constexpr uint8_t GRAY_LUT_LEN = 42;  // 0x2A data bytes, command sent separately
struct GrayLut {
  uint8_t cmd;
  uint8_t data[GRAY_LUT_LEN];
};
const GrayLut kGrayLuts[5] = {
    {0x20, {0x00, 0x02, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}},  // LUTC / VCOM
    {0x21, {0x08, 0x02, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}},  // LUTWW (white)
    {0x22, {0x20, 0x02, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}},  // LUTKW
    {0x23, {0x20, 0x02, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}},  // LUTWK
    {0x24, {0x00, 0x02, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}},  // LUTKK (black)
};

// OEM XTF_PRE_BW_MID conditioning waveform. Each row is command-prefixed:
// byte 0 selects LUT register 0x20..0x24 and the remaining 42 bytes are data.
// It runs over equal B/W planes immediately before the short AA waveform so
// gray and white particle states do not relax after the page stops updating.
const uint8_t kGrayPreBwMid[5][43] = {
    {0x20, 0x00, 0x06, 0x01, 0x06, 0x06, 0x01, 0x00, 0x02, 0x04, 0x00, 0x00, 0x01},
    {0x21, 0x20, 0x06, 0x01, 0x06, 0x06, 0x01, 0x00, 0x02, 0x04, 0x00, 0x00, 0x01},
    {0x22, 0xAA, 0x06, 0x01, 0x06, 0x06, 0x01, 0xA0, 0x02, 0x04, 0x00, 0x00, 0x01},
    {0x23, 0x55, 0x06, 0x01, 0x06, 0x06, 0x01, 0x50, 0x02, 0x04, 0x00, 0x00, 0x01},
    {0x24, 0x00, 0x06, 0x01, 0x06, 0x06, 0x01, 0x10, 0x02, 0x04, 0x00, 0x00, 0x01},
};
}  // namespace

const Uc8179Config& uc8179DefaultConfig() {
  static const Uc8179Config cfg = {
      0x3F,                      // psr0: 0x3B + SHL for FreeInk's framebuffer orientation;
                                 // refresh re-asserts psr0 & 0xDF = 0x1F (OTP + SHL)
      0x0A,                      // psr1
      0x20,                      // pfs (0x03 power-off sequence)
      {0x25, 0x25, 0x3C, 0x25},  // btst (0x06 booster soft-start)
      0x02,                      // gateScan (0xE1)
      0x02,                      // ccset (0xE0)
      0x1E,                      // tsset (0xE5) full refresh (forced-temperature value)
      0x5A,                      // tssetFast (0xE5) fast refresh — REQUIRED: this is the
                                 // frame-rate lever that makes the partial shorter (per RE)
      0x29,                      // cdiActive (0x50, during refresh)
      0xA9,                      // cdiIdle (0x50, restored after)
      600,                       // tresHeight — panel addressed 800x600 (480 visible)
      0x22,                      // powerSave (0xE3): VCOM 2 lines, source 2 * 660 ns
  };
  return cfg;
}

// Visible geometry comes from the active BoardProfile (X4 / X4 Pro, 800x480).
Uc8179Driver::Uc8179Driver(const Uc8179Config& cfg)
    : _cfg(cfg),
      _w(BoardConfig::ACTIVE.displayWidth),
      _h(BoardConfig::ACTIVE.displayHeight),
      _wb(BoardConfig::ACTIVE.displayWidth / 8),
      _tresH(cfg.tresHeight),
      _bufferSize(static_cast<uint32_t>(BoardConfig::ACTIVE.displayWidth / 8) * BoardConfig::ACTIVE.displayHeight) {}

uint32_t Uc8179Driver::spiHz() const {
  // UC8179 serial write timing is rated to 20 MHz, same as the rest of the family.
  return BoardConfig::ACTIVE.displaySpiHz != 0 ? BoardConfig::ACTIVE.displaySpiHz : 16000000;
}

PanelGeometry Uc8179Driver::geometry() const { return {_w, _h, _wb, _bufferSize}; }

// The OEM init (FUN_4214dff8): PSR, TRES (800x600), GSST, PFS, BTST, E1. No plane
// fill, no CDI/VCOM here — those are (re)asserted per refresh. OTP waveforms
// (PSR REG bit cleared at refresh), so no LUT upload.
void Uc8179Driver::initController(EpdBus& bus) {
  bus.cmd(CMD_PANEL_SETTING);
  bus.data(_cfg.psr0);
  bus.data(_cfg.psr1);

  // TRES: HRES (16-bit BE) then VRES (16-bit BE). Width from the visible geometry
  // (800 -> 0x03,0x20), height is the addressed gate count (600 -> 0x02,0x58).
  bus.cmd(CMD_RESOLUTION);
  bus.data(static_cast<uint8_t>((_w >> 8) & 0xFF));
  bus.data(static_cast<uint8_t>(_w & 0xFF));
  bus.data(static_cast<uint8_t>((_tresH >> 8) & 0xFF));
  bus.data(static_cast<uint8_t>(_tresH & 0xFF));

  // GSST is a 4-byte register (S_START, banks, G_START x2); the vendor reference
  // writes all four zero bytes.
  bus.cmd(CMD_GATE_SOURCE_START);
  bus.data(0x00);
  bus.data(0x00);
  bus.data(0x00);
  bus.data(0x00);

  bus.cmd(CMD_PFS);
  bus.data(_cfg.pfs);

  bus.cmd(CMD_BOOSTER_SOFT_START);
  bus.data(_cfg.btst[0]);
  bus.data(_cfg.btst[1]);
  bus.data(_cfg.btst[2]);
  bus.data(_cfg.btst[3]);

  bus.cmd(CMD_GATE_SCAN);
  bus.data(_cfg.gateScan);

  // GxEPD2 added this UC8179 setting specifically for dithered-bitmap
  // stability. Keep it configurable because the X4 Pro uses different glass
  // and a 600-gate scan; zero lets a board preserve its OTP/default behavior.
  if (_cfg.powerSave != 0) {
    bus.cmd(CMD_POWER_SAVE);
    bus.data(_cfg.powerSave);
  }

  _isScreenOn = false;
  _grayRefreshedOnce = false;
  _bwPlanesSynced = false;
  _grayBaseValid = false;
  _absoluteGrayPlanes = false;
#ifdef FREEINK_UC8179_RAIL_POWEROFF
  _ponPending = false;
#endif
#ifdef FREEINK_UC8179_LEAN_STREAMS
  _dtm1Stale = false;
#endif
#ifdef FREEINK_UC8179_DOUBLE_GRAY_PRE
  _deepEqualizeNext = false;
#endif
}

void Uc8179Driver::begin(EpdBus& bus) {
#if defined(BOARD_HAS_PSRAM)
  if (_grayBase == nullptr) {
    _grayBase = static_cast<uint8_t*>(heap_caps_malloc(_bufferSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  }
#endif
#ifdef FREEINK_UC8179_RAIL_POWEROFF
  _bus = &bus;  // beginDisplayWork() takes no bus parameter; see the header
#endif
  bus.reset(50);
  initController(bus);
}

void Uc8179Driver::display(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode, bool turnOff) {
  // CrossPoint's whole-plane text-AA path calls ordinary displayBuffer(FAST)
  // for its B/W base. After an AA page, route that base through stock's
  // non-flashing previous->current transition; promoting it to GC fixed the
  // charge but caused a full flash on every page.
  if (mode == RefreshMode::Fast && _redriveAfterGray && _grayRefreshedOnce && _oldPlaneValid && !_needFullClear) {
    transitionGrayscaleBase(bus, fb, turnOff);
    return;
  }
  displayStart(bus, fb, prev, mode, turnOff);
  displayFinish(bus, fb);
}

void Uc8179Driver::transitionGrayscaleBase(EpdBus& bus, const uint8_t* fb, bool turnOff) {
  if (!fb) return;
#ifdef FREEINK_UC8179_RAIL_POWEROFF
  settlePrewarm(bus);
#endif
#ifdef FREEINK_UC8179_LEAN_STREAMS
  // XTF_PRE_BW_MID drives DTM1 (previous base) against DTM2 (new base), so any
  // deferred OLD-plane restore has to land before _grayBase is overwritten with
  // this page's snapshot. Normally already discharged by the AA upload of the
  // page that deferred it, so this costs nothing on the reading path.
  flushPendingOldPlane(bus);
#endif
  _grayBaseValid = false;
  _absoluteGrayPlanes = false;
  if (_grayBase != nullptr) {
    memcpy(_grayBase, fb, _bufferSize);
    _grayBaseValid = true;
  }

  bus.waitBusy(" 8179_gray_base_ready");
  // DTM1 retains the preceding page's clean B/W base; DTM2 receives the new
  // base. XTF_PRE_BW_MID drives that real transition without the OTP GC flash.
  streamPlane(bus, CMD_DTM2, fb);
  _bwPlanesSynced = false;
  runGrayscalePrecondition(bus);

  // Keep the generic B/W baseline coherent in case no AA pass follows (Home or
  // a menu). An AA upload may immediately overwrite these planes; its cached
  // B/W snapshot above remains intact.
#ifdef FREEINK_UC8179_LEAN_STREAMS
  // ...and on the reading path an AA upload always does, which made this 60 KB
  // stream (~26-30 ms at 20 MHz) dead work on every anti-aliased page. Owe it
  // instead: _grayBase holds the frame, and the handful of consumers that read
  // DTM1 as the OLD plane flush the debt before they need it.
  if (_grayBaseValid) {
    _dtm1Stale = true;
  } else {
    streamPlane(bus, CMD_DTM1, fb);  // no snapshot to restore from later
  }
#else
  streamPlane(bus, CMD_DTM1, fb);
#endif
  _oldPlaneValid = true;
  _bwPlanesSynced = true;
  _redriveAfterGray = false;
  _needFullClear = false;

#ifdef FREEINK_UC8179_DOUBLE_GRAY_PRE
  // One XTF_PRE_BW_MID pass leaves blacks at two different depths. In the
  // old->new transition a was-white pixel that becomes black takes WK (25 frames
  // VDH, saturating) while a was-black pixel that stays black takes only the KK
  // corrective (4 frames VDH), and the AA waveform's gray drive is a mere ~2
  // frames off black — so the previous page's text pattern survives as density
  // variation inside the new page's gray areas. Visible as text ghosting through
  // an image page's grays.
  //
  // Running the precondition a SECOND time with DTM1 == DTM2 == the new base
  // fires only the KK/WW correctives, uniformly across the panel, equalizing
  // black depth before the AA planes land. No RAM plane is touched by the
  // activation, so DTM1/DTM2 and every flag above stay exactly as the
  // single-pass path left them; the pass also re-arms the registers it needs
  // itself (PTIN + full PTL window, PSR REG=1, PFS/gate-scan, CDI active, CCSET,
  // TSSET 0x5A and the kGrayPreBwMid LUT set — none of which the first pass tore
  // down) and exits through the same PTOUT + idle-CDI restore, leaving power on.
  //
  // It costs a second full activation (~666 ms measured), which is why it is
  // opt-in per transition rather than automatic: only a page whose grays are
  // large enough to show the ghost — an image page — is worth it, and the driver
  // cannot tell an image page from a text page. The host arms it through
  // requestDeepGrayEqualize() just before the base display call.
  //
  // One-shot: consumed here whether or not the pass ends up running, so a hint
  // can never carry over into a later page. See the header for the full
  // staleness argument.
  if (_deepEqualizeNext) {
    _deepEqualizeNext = false;
#ifdef FREEINK_UC8179_LEAN_STREAMS
    // The equalizing pass is defined by DTM1 == DTM2, and lean streams has just
    // DEFERRED the DTM1 restore (_dtm1Stale) instead of performing it — DTM1
    // still holds the page before this one. Settle that debt here, or the second
    // activation would re-run the old->new transition rather than the corrective
    // one and drive the changed pixels twice. flushPendingOldPlane() streams
    // _grayBase, which is this page's base (memcpy'd from fb at the top), so
    // after it DTM1 and DTM2 agree exactly as they do on the non-lean path.
    // The ~26-30 ms this gives back is the price of the equalize, paid only on
    // the pages that asked for it.
    flushPendingOldPlane(bus);
#endif
    runGrayscalePrecondition(bus, " 8179_gray_pre2_DRF");
  }
#endif

  if (turnOff && _isScreenOn) {
    bus.cmd(CMD_POWER_OFF);
    bus.waitBusy(" 8179_gray_base_POF");
    _isScreenOn = false;
  }
}

void Uc8179Driver::displayGrayscaleBase(EpdBus& bus, const uint8_t* fb, RefreshMode fallback, bool turnOff) {
  if (!fb) return;

  // Factory.bin's first gray_aa call paints its B/W base normally. Later calls
  // do NOT paint the new base and then condition two equal planes: they load the
  // retained previous base into DTM1, the new base into DTM2, and use
  // XTF_PRE_BW_MID as the page transition itself. Our former extra equal-plane
  // pass caused the visible gray muddling seen in hardware testing and did not
  // discharge the AA residue left by the preceding page.
  // Half is the explicit charge-scrub request. Keep it as a real B/W activation
  // instead of replacing it with the differential stock AA transition.
  if (fallback == RefreshMode::Half || !_grayRefreshedOnce || !_oldPlaneValid || _needFullClear) {
    display(bus, fb, nullptr, fallback, turnOff);
    return;
  }

  transitionGrayscaleBase(bus, fb, turnOff);
}

// Stream a framebuffer into RAM plane `ramCmd`, mirrored vertically via row
// reversal. SHL in PSR handles the horizontal panel direction for FreeInk's
// framebuffer convention. White padding fills the non-visible gates.
void Uc8179Driver::streamPlane(EpdBus& bus, uint8_t ramCmd, const uint8_t* fb, bool invert) {
#ifdef FREEINK_UC8179_LEAN_STREAMS
  if (ramCmd == CMD_DTM1) _dtm1Stale = false;  // a write discharges any deferred restore
#endif
  if (invert) {
    bus.sendPlaneFlippedInverted(ramCmd, fb, _h, _wb);
  } else {
    bus.sendPlaneFlipped(ramCmd, fb, _h, _wb);
  }
  uint8_t whiteRow[128];
  const uint16_t wb = _wb <= sizeof(whiteRow) ? _wb : sizeof(whiteRow);
  memset(whiteRow, 0xFF, wb);
#ifdef FREEINK_UC8179_LEAN_STREAMS
  // The 120 non-visible gate rows used to be 120 EpdBus::data() calls, each a
  // full SPI begin/endTransaction with its own CS pulse. Same bytes on the
  // wire, one burst — matching how sendPlaneFlipped() streams the visible rows.
  if (_h < _tresH) {
    bus.beginTxn();
    for (uint16_t y = _h; y < _tresH; y++) bus.rawWriteBytes(whiteRow, wb);
    bus.endTxn();
  }
#else
  for (uint16_t y = _h; y < _tresH; y++) bus.data(whiteRow, wb);
#endif
}

void Uc8179Driver::streamPlaneXor(EpdBus& bus, uint8_t ramCmd, const uint8_t* lhs, const uint8_t* rhs) {
#ifdef FREEINK_UC8179_LEAN_STREAMS
  if (ramCmd == CMD_DTM1) _dtm1Stale = false;  // as in streamPlane(): a write discharges the debt
#endif
  uint8_t row[128];
  const uint16_t wb = _wb <= sizeof(row) ? _wb : sizeof(row);
  bus.cmd(ramCmd);
  bus.beginTxn();
  for (int y = static_cast<int>(_h) - 1; y >= 0; y--) {
    const uint32_t offset = static_cast<uint32_t>(y) * _wb;
    for (uint16_t x = 0; x < wb; x++) row[x] = static_cast<uint8_t>(lhs[offset + x] ^ rhs[offset + x]);
    bus.rawWriteBytes(row, wb);
  }
  bus.endTxn();
  memset(row, 0xFF, wb);
#ifdef FREEINK_UC8179_LEAN_STREAMS
  if (_h < _tresH) {  // one burst for the gate padding, as in streamPlane()
    bus.beginTxn();
    for (uint16_t y = _h; y < _tresH; y++) bus.rawWriteBytes(row, wb);
    bus.endTxn();
  }
#else
  for (uint16_t y = _h; y < _tresH; y++) bus.data(row, wb);
#endif
}

bool Uc8179Driver::displayStart(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode, bool turnOff) {
  (void)prev;
#ifdef FREEINK_UC8179_RAIL_POWEROFF
  // Unlike every other entry point here, this one writes controller RAM before
  // its first waitBusy — ride out a prewarm ramp before the plane stream below.
  settlePrewarm(bus);
#endif
#ifdef FREEINK_UC8179_LEAN_STREAMS
  // Only the differential DU branch reads DTM1 as the OLD plane; the Full/Half
  // branches rewrite it a few lines down, as does a dark-background Fast. Settle
  // the debt here, before _grayBase is repurposed as this frame's snapshot.
  if (_dtm1Stale) {
    if (mode == RefreshMode::Fast && !_needFullClear && _oldPlaneValid && !_darkBackground) {
      flushPendingOldPlane(bus);
    } else {
      _dtm1Stale = false;
    }
  }
#endif
#ifdef FREEINK_UC8179_DOUBLE_GRAY_PRE
  // The other half of the deep-equalize hint's one-shot contract. This is the
  // only base activation that does NOT go through transitionGrayscaleBase(), so
  // consuming it here too means the hint is always answered — used there,
  // discarded here — by the first base activation that follows it, and can never
  // be seen by a later page. Discarding is correct rather than a loss: reaching
  // this function instead means either a Full/Half clearing waveform, which
  // equalizes black depth by construction, or a differential update with no
  // grayscale pass behind it (display() only diverts a post-AA Fast) — neither
  // leaves the uneven black the corrective pass exists to remove.
  _deepEqualizeNext = false;
#endif
  _bwPlanesSynced = false;
  _absoluteGrayPlanes = false;
  _grayBaseValid = false;
  // Stock derives its B/W base from absolute gray planes (plane0 & plane1).
  // CrossPoint instead displays that B/W base first and then reuses its single
  // framebuffer for transition masks. Preserve the base before returning from
  // this potentially asynchronous entry point so the masks can be converted to
  // stock's absolute selector encoding later.
  if (_grayBase != nullptr && fb != nullptr) {
    memcpy(_grayBase, fb, _bufferSize);
    _grayBaseValid = true;
  }
  // Full and Half use the clearing OTP GC waveform; only an explicit Fast
  // request may use the differential DU partial (PTIN/PTOUT). Half additionally
  // forces every pixel into a transition cell by loading DTM1 with the target's
  // complement. This matters for AA cleanup: a white target paired with a white
  // OLD plane selects WW and can look clean while leaving old text charge parked
  // underneath; gray exposes that latent charge later.
  //
  // GHOSTING FIX: the OLD plane (0x10) MUST hold the PREVIOUS displayed frame for
  // a partial, not a flat 0xFF. In KW mode the (old,new) pair selects the per-
  // pixel LUT; with old=0xFF only WW/WK fire (white-stays and white->black), so
  // KW (black->white) NEVER runs and last page's text is never erased = heavy
  // ghosting. Feeding the previous frame lets KW clear it. (0x10 is synced to the
  // just-displayed frame in displayFinish; a full refresh reseeds it to white.)
  // Half is the explicit strong scrub. Post-AA Fast paints are intercepted by
  // display() and use stock's non-flashing XTF_PRE_BW_MID transition instead.
  const bool scrub = (mode == RefreshMode::Half);
  const bool fast = (mode == RefreshMode::Fast) && !scrub && !_needFullClear && _oldPlaneValid;

  // NEW plane (0x13) = new frame.
  streamPlane(bus, CMD_DTM2, fb);
  if (!fast) {
    if (scrub) {
      // Charge scrub: target white is driven through BW and target black through
      // WB. No WW/BB pixel is allowed to idle with charge from an older AA page.
      streamPlane(bus, CMD_DTM1, fb, /*invert=*/true);
    } else {
      // Full/forced-first flash retains the known absolute-from-white behavior.
      // This is the one DTM1 write in the driver that does not go through
      // streamPlane(), so it does not clear _dtm1Stale itself — it does not have
      // to: reaching this branch means `fast` was false, which is exactly the
      // case the _dtm1Stale block at the top of this function already discharged.
      uint8_t whiteRow[128];
      const uint16_t wb = _wb <= sizeof(whiteRow) ? _wb : sizeof(whiteRow);
      memset(whiteRow, 0xFF, wb);
      bus.cmd(CMD_DTM1);
      for (uint16_t y = 0; y < _tresH; y++) bus.data(whiteRow, wb);
    }
  } else if (_darkBackground) {
    // Inverted content uses the target complement even with DU so unchanged dark
    // background pixels do not park residue.
    streamPlane(bus, CMD_DTM1, fb, /*invert=*/true);
  }
  // (Ordinary Fast: OLD still holds the previous frame from displayFinish.)
  // A completed ordinary refresh supersedes any pending post-AA transition.
  _redriveAfterGray = false;

  // --- Refresh setup (exact OEM order) -----------------------------------------
  bus.cmd(CMD_VCOM_DATA_INTERVAL);
  bus.data(_cfg.cdiActive);  // 0x29
  bus.data(CDI_INTERVAL);
  bus.cmd(CMD_CCSET);
  bus.data(_cfg.ccset);  // 0x02
  bus.cmd(CMD_TSSET);
  bus.data(fast ? _cfg.tssetFast : _cfg.tsset);  // fast 0x5A (frame lever) / full 0x1E
  bus.cmd(CMD_PANEL_SETTING);
  bus.data(static_cast<uint8_t>(_cfg.psr0 & 0xDF));  // 0x1F: REG cleared -> OTP + SHL
  bus.data(_cfg.psr1);
  if (fast) {
    // Fast-only: PFS/gate-scan re-assert. Full omits these; without them the OTP
    // waveform runs at the full frame count (same duration + garbled).
    bus.cmd(CMD_PFS);
    bus.data(_cfg.pfs);  // 0x03 <- 0x20
    bus.cmd(CMD_GATE_SCAN);
    bus.data(_cfg.gateScan);  // 0xE1 <- 0x02
  }

  if (!_isScreenOn) {
    bus.cmd(CMD_POWER_ON);
    bus.waitBusy(" 8179_PON");
    _isScreenOn = true;
  }

  if (fast) bus.cmd(CMD_PARTIAL_IN);  // PTIN — whole-panel partial (no 0x90 window)
  bus.cmd(CMD_DISPLAY_REFRESH);
  // Confirm the waveform started (BUSY dropped) before returning, so
  // displayFinish() only rides out the completion edge.
  {
    const int8_t busyPin = bus.pins().busy;
    const unsigned long t0 = millis();
    while (digitalRead(busyPin) == HIGH && millis() - t0 < 50) delay(1);
  }
  _pendingPartial = fast;
  _pendingTurnOff = turnOff;
  _pendingRefresh = true;
  return true;
}

void Uc8179Driver::displayFinish(EpdBus& bus, const uint8_t* fb) {
#ifdef FREEINK_UC8179_RAIL_POWEROFF
  settlePrewarm(bus);
#endif
  if (!_pendingRefresh) return;
  _pendingRefresh = false;

  bus.waitRefreshComplete(" 8179_DRF");
  if (_pendingPartial) bus.cmd(CMD_PARTIAL_OUT);  // PTOUT closes the partial window
  // Restore the idle CDI (border) after the refresh, as the OEM does.
  bus.cmd(CMD_VCOM_DATA_INTERVAL);
  bus.data(_cfg.cdiIdle);  // 0xA9
  bus.data(CDI_INTERVAL);

  // Sync the OLD plane (0x10) with the just-displayed frame so the NEXT partial
  // diffs against it (KW clears erased pixels -> no ghosting). This is the piece
  // that makes fast page turns clean.
  streamPlane(bus, CMD_DTM1, fb);
  _oldPlaneValid = true;
  _bwPlanesSynced = true;
  _needFullClear = false;

  if (_pendingTurnOff) {
    bus.cmd(CMD_POWER_OFF);
    bus.waitBusy(" 8179_POF");
    _isScreenOn = false;
  }
}

void Uc8179Driver::requestResync(uint8_t settlePasses) {
  (void)settlePasses;
  _needFullClear = true;  // next refresh does a full flash to clear ghosting
}

void Uc8179Driver::skipInitialResync() { _needFullClear = false; }

void Uc8179Driver::deepSleep(EpdBus& bus) {
#ifdef FREEINK_UC8179_RAIL_POWEROFF
  settlePrewarm(bus);  // never POF into a ramp still in progress
#endif
  _grayBaseValid = false;
  _absoluteGrayPlanes = false;
#ifdef FREEINK_UC8179_LEAN_STREAMS
  _dtm1Stale = false;  // DSLP clears controller RAM; nothing left to restore into
#endif
  if (_isScreenOn) {
    bus.cmd(CMD_POWER_OFF);
    bus.waitBusy(" 8179 power-down");
    _isScreenOn = false;
  }
  bus.cmd(CMD_DEEP_SLEEP);
  bus.data(0xA5);
}

// --- 4-level grayscale (anti-aliasing) --------------------------------------
// Load the two bitplanes (oriented + padded like the B/W path) into controller
// RAM; displayGray() then runs the custom-LUT grayscale waveform. CrossPoint's
// masks are converted below to Factory.bin's absolute plane0/plane1 encoding;
// the resulting (DTM1,DTM2) pair selects the WW/BW/WB/BB LUT per pixel.
void Uc8179Driver::runGrayscalePrecondition(EpdBus& bus, const char* drfTag) {
  // Factory.bin skips XTF_PRE_BW_MID for its first AA page. Callers must have
  // retained the previous B/W base in DTM1 and loaded the new base into DTM2.
  if (!_oldPlaneValid || !_grayRefreshedOnce) return;

  bus.waitBusy(" 8179_gray_pre_ready");
  bus.cmd(CMD_PARTIAL_IN);
  const uint16_t xEnd = static_cast<uint16_t>(_w - 1);
  const uint16_t yEnd = static_cast<uint16_t>(_h - 1);
  const uint8_t fullWindow[9] = {0x00,
                                 0x00,
                                 static_cast<uint8_t>(xEnd >> 8),
                                 static_cast<uint8_t>(xEnd | 0x07),
                                 0x00,
                                 0x00,
                                 static_cast<uint8_t>(yEnd >> 8),
                                 static_cast<uint8_t>(yEnd),
                                 0x01};
  bus.cmdData(CMD_PARTIAL_WINDOW, fullWindow, sizeof(fullWindow));
  bus.cmd(CMD_PANEL_SETTING);
  bus.data(_cfg.psr0);  // REG=1: run the external XTF_PRE_BW_MID tables
  bus.data(_cfg.psr1);
  bus.cmd(CMD_PFS);
  bus.data(_cfg.pfs);
  bus.cmd(CMD_GATE_SCAN);
  bus.data(_cfg.gateScan);
  bus.cmd(CMD_VCOM_DATA_INTERVAL);
  bus.data(_cfg.cdiActive);  // Factory.bin FUN_4214eab4 uses 0x29 for every pre-pass
  bus.data(CDI_INTERVAL);
  bus.cmd(CMD_CCSET);
  bus.data(_cfg.ccset);
  bus.cmd(CMD_TSSET);
  bus.data(_cfg.tssetFast);
  for (const auto& l : kGrayPreBwMid) {
    bus.cmd(l[0]);
    bus.data(&l[1], GRAY_LUT_LEN);
  }

  if (!_isScreenOn) {
    bus.cmd(CMD_POWER_ON);
    bus.waitBusy(" 8179_gray_pre_PON");
    _isScreenOn = true;
  }
  bus.cmd(CMD_DISPLAY_REFRESH);
  bus.waitBusy(drfTag);
  bus.cmd(CMD_PARTIAL_OUT);
  bus.cmd(CMD_VCOM_DATA_INTERVAL);
  bus.data(_cfg.cdiIdle);
  bus.data(CDI_INTERVAL);
}

void Uc8179Driver::preconditionGrayscale(EpdBus& bus, uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  // Intentionally no extra pass. The correct stock transition has to run while
  // DTM1 still contains the previous page and is therefore performed by
  // displayGrayscaleBase(). After a normal B/W activation both planes are the
  // current page; conditioning that equal pair only adds visible gray muddling.
  (void)bus;
  (void)x;
  (void)y;
  (void)w;
  (void)h;
}

void Uc8179Driver::copyGrayscaleLsb(EpdBus& bus, const uint8_t* lsb) {
#ifdef FREEINK_UC8179_RAIL_POWEROFF
  settlePrewarm(bus);
#endif
  if (!lsb) return;
  bus.waitBusy(" 8179_gray_lsb");  // prior base refresh must finish before RAM writes
  _absoluteGrayPlanes = false;
  if (_grayBaseValid) {
    // Factory firmware feeds the AA LUT absolute selectors, written here as
    // (plane0/DTM1, plane1/DTM2):
    //   black=(0,0), dark=(1,0), light=(0,1), white=(1,1).
    // CrossPoint instead supplies (maskLsb, maskMsb):
    //   black/white=(0,0), dark=(1,1), light=(0,1),
    // while its B/W base is 0 for every non-white pixel and 1 for white.
    // Folding the base into its LSB mask produces stock plane0:
    //   plane0 = base | maskLsb.
    for (uint32_t i = 0; i < _bufferSize; i++) {
      _grayBase[i] = static_cast<uint8_t>(_grayBase[i] | lsb[i]);
    }
    streamPlane(bus, CMD_DTM1, _grayBase);
    _absoluteGrayPlanes = true;
  } else {
    streamPlane(bus, CMD_DTM1, lsb);  // compatibility fallback without a base snapshot
  }
  _grayBaseValid = false;  // _grayBase now holds absolute plane0, not the B/W base
  _bwPlanesSynced = false;
}

void Uc8179Driver::copyGrayscaleMsb(EpdBus& bus, const uint8_t* msb) {
#ifdef FREEINK_UC8179_RAIL_POWEROFF
  settlePrewarm(bus);
#endif
  if (!msb) return;
  bus.waitBusy(" 8179_gray_msb");
  if (_absoluteGrayPlanes) {
    // With plane0=(base|maskLsb), stock plane1 is plane0 XOR maskMsb:
    // black 0^0=0, dark 1^1=0, light 0^1=1, white 1^0=1.
    streamPlaneXor(bus, CMD_DTM2, _grayBase, msb);
    // The stock gray_aa routine restores BOTH controller planes to its B/W base
    // after the gray activation. Recover that base now while plane0 and the MSB
    // mask are still available: base = plane0 & plane1.
    for (uint32_t i = 0; i < _bufferSize; i++) {
      _grayBase[i] = static_cast<uint8_t>(_grayBase[i] & (_grayBase[i] ^ msb[i]));
    }
    _grayBaseValid = true;
  } else {
    streamPlane(bus, CMD_DTM2, msb);  // compatibility fallback
  }
}

void Uc8179Driver::displayGray(EpdBus& bus, const uint8_t* fb, bool turnOff, const unsigned char* lut,
                               bool factoryMode) {
  // fb = the reader's current frame; used to re-seed the B/W baseline below.
  (void)lut;          // waveform comes from the built-in gray LUT set (kGrayLuts)
  (void)factoryMode;  // 4-level is absolute (defined by the planes)
  (void)turnOff;      // Factory.bin gray_aa leaves analog power enabled
#ifdef FREEINK_UC8179_RAIL_POWEROFF
  settlePrewarm(bus);
#endif

  // The base refresh must be fully complete before we upload LUTs / stream — the
  // controller drops LUT/DTM/DRF writes while BUSY.
  bus.waitBusy(" 8179_gray_ready");
  _bwPlanesSynced = false;

  // Custom-LUT 4-level grayscale — the EXACT stock gray_aa stream (FUN_4214ec2c),
  // byte-for-byte apart from FreeInk's SHL orientation bit: PSR 0x3F (REG bit5=1
  // custom LUT; the B/W path masks to 0x1F/OTP) -> upload the 5 short LUTs
  // separately, 42 data bytes each) -> CDI 0x29/07 -> PON -> DRF. Unlike the
  // gray_full path, Factory.bin's gray_aa function sends no POF afterward. It
  // also sends no E0/E5/booster here; those belong to prebw/gray_full.
  bus.cmd(CMD_PANEL_SETTING);
  bus.data(_cfg.psr0);  // 0x3F: REG=1 (custom LUT) + KW + SHL
  bus.data(_cfg.psr1);
  for (const auto& l : kGrayLuts) {
    bus.cmd(l.cmd);
    bus.data(l.data, GRAY_LUT_LEN);
  }
  bus.cmd(CMD_VCOM_DATA_INTERVAL);
  // Factory.bin FUN_4214ec2c calls vtable +0x118 unconditionally; the UC8179
  // getter at 0x422988b0 returns 0x29. Unlike UC8279, it does not switch the AA
  // activation to the idle/hold CDI after the first page.
  bus.data(_cfg.cdiActive);
  bus.data(CDI_INTERVAL);
  _grayRefreshedOnce = true;

  if (!_isScreenOn) {
    bus.cmd(CMD_POWER_ON);
    bus.waitBusy(" 8179_gray_PON");
    _isScreenOn = true;
  }
  bus.cmd(CMD_DISPLAY_REFRESH);
  bus.waitBusy(" 8179_gray_DRF");
  // Deliberately remain powered. FUN_4214ec2c returns after DRF and RAM/base
  // bookkeeping without issuing command 0x02; deepSleep() still powers down.
  // Its bookkeeping writes the clean B/W base to BOTH DTM1 and DTM2. Besides
  // preserving the next transition's old frame, this prevents a stale gray
  // selector plane from being reused by a later refresh (especially sleep).
  if (_grayBaseValid) {
    streamPlane(bus, CMD_DTM1, _grayBase);
#ifndef FREEINK_UC8179_LEAN_STREAMS
    streamPlane(bus, CMD_DTM2, _grayBase);
#else
    // The DTM2 half of that restore is provably dead on this driver, so the AA
    // page does not pay its 60 KB (~26-30 ms). Every path that can follow
    // displayGray() writes the NEW plane before it is ever activated:
    // displayStart() streams DTM2 as its first act on both branches;
    // transitionGrayscaleBase() streams DTM2 before runGrayscalePrecondition()
    // (its only caller), which is also the whole sleep/screensaver route;
    // copyGrayscaleMsb() streams DTM2 for the next AA page; deepSleep() reads no
    // plane at all; and cleanupGrayscaleBuffers()'s fallback rewrites both.
    // displayGrayCalibration() was audited and excluded rather than covered: it
    // is PanelDriver's base implementation (this driver does not override it)
    // and has no caller anywhere in the SDK or in CrossPoint, so it cannot
    // observe the missing restore. Wiring it up later would mean feeding it
    // planes through copyGrayscaleLsb/Msb first, which write DTM1 and DTM2
    // themselves — so it would still never read a stale NEW plane. The
    // "stale gray selector" the comment above guards against therefore never
    // reaches a DRF. _bwPlanesSynced accordingly means the baseline is coherent
    // for whatever refresh comes next, which is what its one consumer
    // (cleanupGrayscaleBuffers) actually asks.
#endif
    _oldPlaneValid = true;
    _bwPlanesSynced = true;
    _needFullClear = false;
  }
  _grayBaseValid = false;
  _absoluteGrayPlanes = false;

  // `fb` is the MSB mask here, not the B/W frame; the recovered base above was
  // used for the RAM restore. Physically, AA still leaves intermediate charge
  // that a plain DU diff does not neutralize. Route the next Fast B/W base through
  // stock's non-flashing transition; an explicit Half remains the strong purge.
  (void)fb;
  _redriveAfterGray = true;
}

void Uc8179Driver::cleanupGrayscaleBuffers(EpdBus& bus, const uint8_t* bw) {
#ifdef FREEINK_UC8179_RAIL_POWEROFF
  settlePrewarm(bus);
#endif
  bus.waitBusy(" 8179_gray_cleanup");
#ifdef FREEINK_UC8179_LEAN_STREAMS
  // Gray exit is a baseline consumer, and it drops _grayBase below — settle any
  // deferred OLD-plane restore while the frame it owes is still available. A
  // normal AA page discharged it at copyGrayscaleLsb(), so this is only reached
  // when a gray sequence was abandoned between base and planes.
  flushPendingOldPlane(bus);
#endif
  _grayBaseValid = false;
  _absoluteGrayPlanes = false;
  if (!bw) {
    // No baseline provided — fall back to a full flash on the next B/W refresh.
    _needFullClear = true;
    _oldPlaneValid = false;
    return;
  }
  if (_bwPlanesSynced && _oldPlaneValid) return;
  // Compatibility fallback when no PSRAM base snapshot was available. Stock
  // restores both planes, not only DTM1.
  streamPlane(bus, CMD_DTM1, bw);
  streamPlane(bus, CMD_DTM2, bw);
  _oldPlaneValid = true;
  _bwPlanesSynced = true;
  _needFullClear = false;
}

#ifdef FREEINK_UC8179_LEAN_STREAMS
void Uc8179Driver::flushPendingOldPlane(EpdBus& bus) {
  if (!_dtm1Stale) return;
  _dtm1Stale = false;
  if (_grayBase != nullptr && _grayBaseValid) {
    // Own wait rather than relying on the caller's: this runs ahead of the
    // point where each caller would otherwise settle the controller, and the
    // UC8179 drops RAM writes issued while BUSY_N is low.
    bus.waitBusy(" 8179_old_plane_flush");
    streamPlane(bus, CMD_DTM1, _grayBase);
    _bwPlanesSynced = true;
    return;
  }
  // Unreachable by construction: the debt is only taken on with a valid
  // snapshot, and every site that drops _grayBaseValid either writes DTM1 or
  // flushes first. Kept as a fence — DTM1 would otherwise still hold the page
  // before last, which a differential would ghost straight through.
  _oldPlaneValid = false;
  _bwPlanesSynced = false;
  _needFullClear = true;
}
#endif

#ifdef FREEINK_UC8179_RAIL_POWEROFF
void Uc8179Driver::settlePrewarm(EpdBus& bus) {
  if (!_ponPending) return;
  _ponPending = false;
  bus.waitBusy(" 8179_prewarm_PON");
}

// Park the analog rails once the host's controller-work queue has drained. POF
// is the whole sequence: PFS (0x20 = 3 frames) runs the documented power-off
// ramp, and the controller keeps its registers, its PSR LUT-mode selection, any
// uploaded LUT tables and both RAM planes across it — deepSleep() has always
// relied on that, and displayStart()/runGrayscalePrecondition()/displayGray()
// each re-assert CDI, CCSET, TSSET and PSR in their own setup block before the
// next PON regardless. So nothing has to be re-armed here.
void Uc8179Driver::controllerIdle(EpdBus& bus) {
  settlePrewarm(bus);
  if (!_isScreenOn) return;  // idempotent: no SPI at all once the rails are down
  bus.cmd(CMD_POWER_OFF);
  bus.waitBusy(" 8179_idle_POF");
  _isScreenOn = false;
}

// Rail prewarm from the host's input dispatch, before the page is composed.
// PON costs a measured 127 ms on this panel (3-6x the SSD1677 ramp, and BTST is
// already at the datasheet minimum), so it is commanded WITHOUT waiting: the
// ramp then runs concurrently with the host's CPU-side composition instead of
// landing on the critical path inside displayStart(). _isScreenOn is claimed
// immediately so the display paths skip their own PON, and _ponPending makes the
// first of them ride the remainder of the ramp out through bus.waitBusy() —
// which keeps the wait inside the host's light-sleep busy-slice hook, same as
// every other wait here.
void Uc8179Driver::beginDisplayWork() {
  if (_isScreenOn || _bus == nullptr) return;
  _bus->cmd(CMD_POWER_ON);
  _isScreenOn = true;
  _ponPending = true;
}
#endif

#ifdef FREEINK_UC8179_DOUBLE_GRAY_PRE
// Arm the deep black-depth equalize for the NEXT base transition only. The host
// calls this immediately before the base display of a page whose grays are large
// enough for the uneven-black ghost to show (an image page); text pages leave it
// unset and keep the single-pass cost.
void Uc8179Driver::requestDeepGrayEqualize() { _deepEqualizeNext = true; }
#endif

// Per-board config injection, same idiom as the other drivers: define
// `const Uc8179Config& yourConfig();` in namespace freeink and build with
// -DFREEINK_UC8179_CONFIG=yourConfig.
#ifdef FREEINK_UC8179_CONFIG
const Uc8179Config& FREEINK_UC8179_CONFIG();
static const Uc8179Config& uc8179ActiveConfig() { return FREEINK_UC8179_CONFIG(); }
#else
static const Uc8179Config& uc8179ActiveConfig() { return uc8179DefaultConfig(); }
#endif

PanelDriver& uc8179Driver() {
  static Uc8179Driver instance(uc8179ActiveConfig());
  return instance;
}

}  // namespace freeink
