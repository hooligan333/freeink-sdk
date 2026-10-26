#include <cassert>
#include <algorithm>
#include <vector>
#include <cstdio>
#include <cstring>
#include <type_traits>
// Inspect private state without adding test-only methods to the SDK API.
#define private public
#include "FreeInkDisplay.h"
#include "src/driver/Ssd1677Driver.h"
#include "src/lut/Ssd1677Luts.h"
#include "src/driver/Uc8179Driver.h"
#include "src/driver/Uc8279X4Driver.h"
#undef private

using namespace freeink;
using Bytes = std::vector<uint8_t>;

static uint8_t lastRegister(const EpdBus& bus, uint8_t command) {
  for (auto i = bus.writes.rbegin(); i != bus.writes.rend(); ++i)
    if (i->command == command && !i->bytes.empty()) return i->bytes[0];
  assert(false);
  return 0;
}

static const Bytes& lastPlane(const EpdBus& bus, uint8_t command) {
  for (auto i = bus.writes.rbegin(); i != bus.writes.rend(); ++i)
    if (i->command == command) return i->bytes;
  assert(false);
  static const Bytes empty;
  return empty;
}

static Bytes frame(unsigned seed) {
  Bytes b(48000);
  for (size_t i=0; i<b.size(); ++i) b[i]=uint8_t((i*37 + i/100*11 + seed) ^ (i>>8));
  return b;
}

template<class Driver>
static void testStream(bool reverse, unsigned gateOffset) {
  Driver d;
  EpdBus bus;
  const auto a=frame(3), b=frame(89);
  for (bool invert : {false,true}) {
    bus.clear();
    d.streamPlane(bus,0x13,a.data(),invert);
    assert(bus.writes.size()==1);
    const auto& w=bus.writes.front();
    assert(w.transactions==1 && w.bytes.size()==60000);
    for (unsigned y=0; y<600; ++y) for (unsigned x=0; x<100; ++x) {
      uint8_t expected=0xff;
      if (y>=gateOffset && y<gateOffset+480) {
        unsigned row=y-gateOffset;
        if (reverse) row=479-row;
        expected=a[row*100+x];
        if (invert) expected=uint8_t(~expected);
      }
      assert(w.bytes[y*100+x]==expected);
    }
  }
  bus.clear();
  d.streamPlaneXor(bus,0x13,a.data(),b.data());
  assert(bus.writes.size()==1 && bus.writes[0].transactions==1);
  assert(bus.writes[0].bytes.size()==60000);
  for (unsigned y=0; y<600; ++y) for (unsigned x=0; x<100; ++x) {
    uint8_t expected=0xff;
    if (y>=gateOffset && y<gateOffset+480) {
      unsigned row=y-gateOffset;
      if (reverse) row=479-row;
      expected=a[row*100+x]^b[row*100+x];
    }
    assert(bus.writes[0].bytes[y*100+x]==expected);
  }
  d._grayRefreshedOnce=true;
  d._oldPlaneValid=true;
  d._needFullClear=false;
  bus.clear();
  d.displayGrayscaleBase(bus,a.data(),RefreshMode::Full,false);
  assert(lastRegister(bus,0xe5)==0x1e); // honor Full even after an AA page
  for (const auto& w : bus.writes)
    if (w.command==0x10 || w.command==0x13) assert(w.transactions==1);
}

static void testSsd() {
  const auto b=frame(33);
  for (auto mode : {RefreshMode::Full,RefreshMode::Half,RefreshMode::Fast}) {
    EpdBus bus;
    Ssd1677Driver d;
    d.begin(bus);
    bus.clear();
    d.display(bus,b.data(),nullptr,mode,false);
    assert(lastRegister(bus,0x22)==(mode==RefreshMode::Full ? 0xf7 : 0xd7));
    bus.clear();
    d.display(bus,b.data(),nullptr,RefreshMode::Fast,false);
    assert(lastRegister(bus,0x22)==0xfc);
  }
  // The first-paint-after-boot/wake promotion must survive turnOff. A caller that powers
  // the panel down on every refresh (CrossPoint's sunlight fading fix) otherwise never
  // consumes the one-shot, and its differential FAST cannot clear the sleep screen.
  {
    EpdBus bus;
    Ssd1677Driver d;
    d.begin(bus);
    // The activation value is not the LAST 0x22 write here: 0xfc does not self-power-off,
    // so the driver follows it with the separate 0x22=0x03 power-down sequence.
    const auto activated = [&bus](uint8_t seq) {
      for (const auto& w : bus.writes)
        if (w.command == 0x22 && !w.bytes.empty() && w.bytes[0] == seq) return true;
      return false;
    };
    bus.clear();
    d.display(bus,b.data(),nullptr,RefreshMode::Fast,true);
    assert(activated(0xd7) && !activated(0xfc));
    bus.clear();
    d.display(bus,b.data(),nullptr,RefreshMode::Fast,true);
    assert(activated(0xfc) && !activated(0xd7));
  }
  EpdBus bus;
  Ssd1677Driver d;
  d.begin(bus);
  bus.clear();
  d.displayGray(bus,b.data(),false,nullptr,false);
  assert(lastRegister(bus,0x22)==0xcc && d._isScreenOn);
  d.deepSleep(bus);
  assert(lastRegister(bus,0x22)==3 && !d._isScreenOn);
  bus.clear();
  d.displayGray(bus,b.data(),true,nullptr,false);
  assert(lastRegister(bus,0x22)==0xcf && !d._isScreenOn);
}

template<class Driver>
static void testAsyncFrame() {
  Driver driver;
  FreeInkDisplay display(12,11,13,18,14,6);
  display._driver=&driver;
  display.begin();
  const auto submitted=frame(27), redrawn=frame(94);
  std::memcpy(display.getFrameBuffer(),submitted.data(),submitted.size());
  display.displayBufferAsync(FreeInkDisplay::FAST_REFRESH);
  assert(display.isRefreshPending());
  std::memcpy(display.getFrameBuffer(),redrawn.data(),redrawn.size());
  display.completeDisplay();
  // Finish must sync the frame actually submitted, even after the caller draws.
  const auto& old=display._bus.writes.back();
  assert(old.command==0x10 && old.bytes.size()==60000);
  for (unsigned y=0; y<480; ++y) {
    unsigned dst;
    if constexpr (std::is_same<Driver,Uc8179Driver>::value) dst=479-y;
    else dst=120+y;
    assert(std::equal(submitted.begin()+y*100,submitted.begin()+(y+1)*100,old.bytes.begin()+dst*100));
  }
  assert(!display.isRefreshPending());
  // Shadow-free entry points require the live frame to survive until finish.
  std::memcpy(display.getFrameBuffer(),redrawn.data(),redrawn.size());
  display.triggerDisplay(FreeInkDisplay::FAST_REFRESH,false);
  display.completeDisplay();
  const auto& next=display._bus.writes.back();
  unsigned firstRow=std::is_same<Driver,Uc8179Driver>::value ? 479 : 0;
  unsigned offset=std::is_same<Driver,Uc8179Driver>::value ? 0 : 12000;
  assert(std::equal(redrawn.begin()+firstRow*100,redrawn.begin()+(firstRow+1)*100,next.bytes.begin()+offset));
#ifdef EINK_DISPLAY_SINGLE_BUFFER_MODE
  std::memcpy(display.getFrameBuffer(),submitted.data(),submitted.size());
  display.displayAsyncImpl(FreeInkDisplay::FAST_REFRESH,false,true);
  display.completeDisplay();
  const auto& noShadow=display._bus.writes.back();
  assert(std::equal(submitted.begin()+firstRow*100,submitted.begin()+(firstRow+1)*100,
                    noShadow.bytes.begin()+offset));
#endif
  display.releaseBuffers();
  free(driver._grayBase);
}

// A driver with no grayscale implementation must never advertise support.
class BwOnlyDriver : public PanelDriver {
 public:
  uint32_t spiHz() const override { return 10000000; }
  BusyPolarity busyPolarity() const override { return BusyPolarity::ActiveHigh; }
  PanelGeometry geometry() const override { return {800, 480, 100, 48000}; }
  void begin(EpdBus&) override {}
  void deepSleep(EpdBus&) override {}
  void display(EpdBus&, const uint8_t*, const uint8_t*, RefreshMode, bool) override {}
};

class CombinedGrayDriver : public BwOnlyDriver {
 public:
  GrayscaleCapabilities grayscaleCapabilities(GrayscaleMode mode = GrayscaleMode::Overlay) const override {
    if (mode != GrayscaleMode::Overlay) return {};
    return {GrayscaleEncoding::OverlayMasks, GrayscaleBase::Combined, true, false, true};
  }
};

static void testCapabilities() {
  FreeInkDisplay display(1, 2, 3, 4, 5, 6);
  assert(!display.grayscaleCapabilities().supported());
  BwOnlyDriver bw;
  display._driver = &bw;
  assert(!display.grayscaleCapabilities().supported());
  assert(!display.supportsStripGrayscale());

  Ssd1677Driver ssd;
  display._driver = &ssd;
  auto caps = display.grayscaleCapabilities();
  assert(caps.supported() && caps.encoding == GrayscaleEncoding::OverlayMasks);
  assert(caps.stripUploads && caps.asyncBase && !caps.stagingWhileBusy);
  assert(caps.base == GrayscaleBase::Separate);
  assert(display.supportsStripGrayscale() == caps.stripUploads);
  assert(display.supportsAsyncGrayscaleBase() == caps.asyncBase);
  assert(display.grayscaleCapabilities(GrayscaleMode::Absolute).supported());
  assert(!display.grayscaleCapabilities(static_cast<GrayscaleMode>(255)).supported());
  display._inversionDirty = true;
  assert(!display.grayscaleCapabilities().asyncBase);
  assert(display.grayscaleCapabilities().stripUploads);
  assert(!display.supportsAsyncGrayscaleBase());
  display._inversionDirty = false;

  CombinedGrayDriver combined;
  display._driver = &combined;
  assert(display.combinesGrayscaleBase());
  assert(display.supportsBusyGrayscaleStaging());
  assert(!display.supportsAsyncGrayscaleBase());
  display._inverted = true;
  caps = display.grayscaleCapabilities();
  assert(!caps.supported() && !caps.stripUploads && !caps.asyncBase && !caps.stagingWhileBusy);
  assert(!display.combinesGrayscaleBase() && !display.supportsBusyGrayscaleStaging());
  assert(!display.supportsStripGrayscale());
  display._inverted = false;

  Uc8179Driver uc8179;
  Uc8279X4Driver uc8279;
  for (PanelDriver* driver : {static_cast<PanelDriver*>(&uc8179), static_cast<PanelDriver*>(&uc8279)}) {
    display._driver = driver;
    caps = display.grayscaleCapabilities();
    assert(caps.supported() && !caps.stripUploads && !caps.asyncBase && !caps.stagingWhileBusy);
    assert(display.grayscaleCapabilities(GrayscaleMode::Absolute).supported());
    assert(!display.grayscaleCapabilities(GrayscaleMode::Absolute).stripUploads);
  }
  // Queries must neither start a refresh nor write the panel bus.
  assert(display._bus.writes.empty() && !display.isRefreshPending());
}

static void testAbsolutePipeline() {
  Ssd1677Driver driver;
  FreeInkDisplay display(1, 2, 3, 4, 5, 6);
  display._driver = &driver;
  display.begin();
  const auto bw = frame(33), lsb = frame(14), msb = frame(29);
  std::memcpy(display.getFrameBuffer(), bw.data(), bw.size());
  const auto absolute = GrayscaleMode::Absolute;
  assert(display.grayscaleCapabilities(absolute).base == GrayscaleBase::Combined);
  for (bool strips : {false, true}) {
    display._bus.clear();
    assert(display.displayGrayscaleBase(absolute));
    assert(display._bus.writes.empty());
    if (strips) {
      for (unsigned y = 0; y < 480; y += 80) {
        display.writeGrayscalePlaneStrip(FreeInkDisplay::GRAY_PLANE_LSB, lsb.data() + y * 100, y, 80);
        display.writeGrayscalePlaneStrip(FreeInkDisplay::GRAY_PLANE_MSB, msb.data() + y * 100, y, 80);
      }
    } else display.copyGrayscaleBuffers(lsb.data(), msb.data());
    Bytes plane0, plane1;
    for (const auto& w : display._bus.writes) {
      if (w.command == 0x24) plane0.insert(plane0.end(), w.bytes.begin(), w.bytes.end());
      if (w.command == 0x26) plane1.insert(plane1.end(), w.bytes.begin(), w.bytes.end());
    }
    assert(plane0.size() == lsb.size() && plane1.size() == msb.size());
    for (size_t i = 0; i < lsb.size(); ++i) {
      assert(plane0[i] == uint8_t(~lsb[i]) && plane1[i] == uint8_t(~msb[i]));
    }
    for (const auto& w : display._bus.writes) assert(w.command != 0x20);
    display.displayGrayBuffer(false);
    assert(std::count_if(display._bus.writes.begin(), display._bus.writes.end(),
                         [](const auto& w) { return w.command == 0x20; }) == 1);
    assert(lastRegister(display._bus, 0x22) == 0xCC);
    assert(driver._isScreenOn && driver._needsGrayClear);
    display.cleanupGrayscaleBuffers(bw.data());
    assert(driver._needsGrayClear);
    display._bus.clear();
    display.displayBufferAsync(FreeInkDisplay::FAST_REFRESH);
    assert(lastRegister(display._bus, 0x22) == 0xD7);
    display.waitRefreshComplete();
    assert(!driver._needsGrayClear);
  }
  // Explicit shutdown must use its own activation after the gray refresh wait.
  assert(display.displayGrayscaleBase(absolute));
  display.copyGrayscaleBuffers(lsb.data(), msb.data());
  display._bus.clear();
  display.displayGrayBuffer(true);
  std::vector<uint8_t> activations;
  for (const auto& w : display._bus.writes) {
    if (w.command == 0x22) activations.push_back(w.bytes.at(0));
  }
  assert((activations == std::vector<uint8_t>{0xCC, 0x03}));
  assert(!driver._isScreenOn && driver._needsGrayClear);
  for (unsigned failure = 0; failure < 6; ++failure) {
    assert(display.displayGrayscaleBase(absolute));
    display._bus.clear();
    if (failure == 0) display.copyGrayscaleLsbBuffers(lsb.data()); // missing MSB
    if (failure == 1) display.writeGrayscalePlaneStrip(FreeInkDisplay::GRAY_PLANE_LSB, lsb.data(), 80, 80);
    if (failure == 2) display.writeGrayscalePlaneStrip(FreeInkDisplay::GRAY_PLANE_LSB, lsb.data(), 0, 481);
    if (failure == 3) display.copyGrayscaleLsbBuffers(nullptr);
    if (failure == 4) {
      display.copyGrayscaleBuffers(lsb.data(), msb.data());
      display.copyGrayscaleLsbBuffers(lsb.data()); // duplicate plane
    }
    if (failure == 5) {
      display.copyGrayscaleMsbBuffers(msb.data());
      display.copyGrayscaleLsbBuffers(lsb.data());
    }
    display.displayGrayBuffer();
    for (const auto& w : display._bus.writes) assert(w.command != 0x20);
    display.cleanupGrayscaleBuffers(nullptr);
    display._bus.clear();
    display.displayBuffer(FreeInkDisplay::FAST_REFRESH, true);
    assert(lastRegister(display._bus, 0x22) == 0xD7);
  }
  assert(display.displayGrayscaleBase(absolute));
  display.copyGrayscaleLsbBuffers(lsb.data());
  display.setInverted(true);
  assert(display._grayscaleMode == GrayscaleMode::Overlay);
  assert(!display.displayGrayscaleBase(absolute));
  display.setInverted(false);
  display._bus.clear();
  assert(display.displayGrayscaleBase(absolute));
  assert(display._bus.writes.empty() && display._inversionDirty);
  display.copyGrayscaleBuffers(lsb.data(), msb.data());
  display.displayGrayBuffer();
  assert(!display._inversionDirty);
  assert(display.displayGrayscaleBase(absolute));
  display.deepSleep();
  assert(display._grayscaleMode == GrayscaleMode::Overlay);
  display.releaseBuffers();

  Ssd1677Config unsupported = ssd1677DefaultConfig();
  unsupported.absoluteGrayscale = false;
  Ssd1677Driver other(unsupported);
  FreeInkDisplay noAbsolute(1, 2, 3, 4, 5, 6);
  noAbsolute._driver = &other;
  assert(!noAbsolute.displayGrayscaleBase(absolute));
  assert(noAbsolute._bus.writes.empty());
}

template<class Driver>
static void testUltraChipAbsolute(bool reverse, unsigned gateOffset, bool inverted) {
  for (bool snapshot : {false, true}) for (bool turnOff : {false, true}) {
    Driver driver;
    FreeInkDisplay display(1, 2, 3, 4, 5, 6);
    display._driver = &driver;
    display.begin();
    if (!snapshot) { free(driver._grayBase); driver._grayBase = nullptr; }
    const auto bw = frame(91), lsb = frame(13), msb = frame(29);
    memcpy(display.getFrameBuffer(), bw.data(), bw.size());
    assert(display.displayGrayscaleBase(GrayscaleMode::Absolute));
    display._bus.clear();
    display.copyGrayscaleBuffers(lsb.data(), msb.data());
    for (unsigned p = 0; p < 2; ++p) {
      const auto& input = p ? msb : lsb;
      const auto command = p ? 0x13 : 0x10;
      const auto it = std::find_if(display._bus.writes.begin(), display._bus.writes.end(),
                                  [command](const auto& w) { return w.command == command; });
      assert(it != display._bus.writes.end() && it->bytes.size() == 60000);
      for (unsigned y = 0; y < 600; ++y) for (unsigned x = 0; x < 100; ++x) {
        uint8_t expected = 0xff;
        if (y >= gateOffset && y < gateOffset + 480) {
          unsigned row = y - gateOffset;
          if (reverse) row = 479 - row;
          expected = input[row * 100 + x];
          if (inverted) expected = uint8_t(~expected);
        }
        assert(it->bytes[y * 100 + x] == expected);
      }
    }
    if (snapshot) assert(memcmp(driver._grayBase, bw.data(), bw.size()) == 0);
    display.displayGrayBuffer(turnOff);
    assert(driver._isScreenOn == !turnOff);
    assert(driver._needFullClear && !driver._absoluteInput);
    display.cleanupGrayscaleBuffers(bw.data());
    assert(driver._needFullClear);
    display.displayBuffer(FreeInkDisplay::FAST_REFRESH);
    assert(!driver._needFullClear);
    // Incomplete uploads are canceled without a gray activation and still force a clean.
    assert(display.displayGrayscaleBase(GrayscaleMode::Absolute));
    display.copyGrayscaleLsbBuffers(lsb.data());
    display._bus.clear();
    display.displayGrayBuffer();
    for (const auto& w : display._bus.writes) assert(w.command != 0x12);
    display.cleanupGrayscaleBuffers(bw.data());
    assert(driver._needFullClear && !driver._absoluteInput);
    display.displayGrayscaleBase(FreeInkDisplay::HALF_REFRESH);
    assert(!driver._absoluteInput);
    display.releaseBuffers();
    free(driver._grayBase);
    driver._grayBase = nullptr;
  }
}

static void testUc8179GrayShadeSplit() {
  Uc8179Driver driver;
  freeink::EpdBus bus;
  driver.begin(bus);
  const auto bw = frame(91), lsb = frame(13), msb = frame(29);
  for (auto mode : {GrayscaleMode::Absolute, GrayscaleMode::Overlay}) {
    driver.beginGrayscale(bus, bw.data(), mode, RefreshMode::Half, false);
    driver.copyGrayscaleLsb(bus, lsb.data());
    driver.copyGrayscaleMsb(bus, msb.data());
    bus.clear();
    driver.displayGray(bus, bw.data(), false, nullptr, mode == GrayscaleMode::Absolute);
    const auto light = std::find_if(bus.writes.begin(), bus.writes.end(),
                                    [](const auto& w) { return w.command == 0x22; });
    const auto dark = std::find_if(bus.writes.begin(), bus.writes.end(),
                                   [](const auto& w) { return w.command == 0x23; });
    assert(light != bus.writes.end() && dark != bus.writes.end());
    assert(light->bytes.size() == 42 && dark->bytes.size() == 42);
    assert(light->bytes[0] == 0x20 && light->bytes[1] == 2 && light->bytes[2] == 2);
    auto expected = light->bytes;
    expected[2] = 1;
    expected[3] = 2;
    assert(dark->bytes == expected);
    std::vector<size_t> pllWrites;
    size_t refresh = bus.writes.size();
    for (size_t i = 0; i < bus.writes.size(); ++i) {
      if (bus.writes[i].command == 0x30 && bus.writes[i].bytes.size() == 1) pllWrites.push_back(i);
      if (bus.writes[i].command == 0x12) refresh = i;
    }
    if (mode == GrayscaleMode::Absolute) {
      assert(pllWrites.size() == 2);
      assert(bus.writes[pllWrites[0]].bytes[0] == 0x05 && bus.writes[pllWrites[1]].bytes[0] == 0x06);
      assert(pllWrites[0] < refresh && refresh < pllWrites[1]);
    } else {
      assert(pllWrites.empty());
    }
  }
  free(driver._grayBase);
}

template<class Driver>
static void testDirectSleep() {
  Driver driver;
  FreeInkDisplay display(1, 2, 3, 4, 5, 6);
  display._driver = &driver;
  display.begin();
  const auto bw = frame(11), lsb = frame(17), msb = frame(29);
  std::memcpy(display.getFrameBuffer(), bw.data(), bw.size());
  const auto activations = [&]() {
    return std::count_if(display._bus.writes.begin(), display._bus.writes.end(),
                         [](const auto& w) { return w.command == 0x12; });
  };
  assert(display.grayscaleCapabilities(GrayscaleMode::Direct).base == GrayscaleBase::Combined);
  for (auto fallback : {FreeInkDisplay::HALF_REFRESH, FreeInkDisplay::FAST_REFRESH}) {
    display._bus.clear();
    assert(display.displayGrayscaleBase(GrayscaleMode::Direct, fallback));
    assert(activations() == 0);
    display.copyGrayscaleBuffers(lsb.data(), msb.data());
    assert(activations() == 0);
    display.displayGrayBuffer(false);
    assert(activations() == 1);
    display.cleanupGrayscaleBuffers(bw.data());
    display.displayBuffer(FreeInkDisplay::FAST_REFRESH, false);
    assert(activations() > 1);
  }
  display._bus.clear();
  assert(display.displayGrayscaleBase(GrayscaleMode::Direct));
  display.copyGrayscaleLsbBuffers(lsb.data());
  display.displayGrayBuffer(false);
  assert(activations() == 0);
  display.deepSleep();
  free(driver._grayBase);
}

static void testUc8279X4WaveformSelection() {
  Uc8279X4Driver driver;
  EpdBus bus;
  driver.begin(bus);
  const auto bw = frame(11);
  Bytes lsb(48000, 0), msb(48000, 0);
  const auto bank = [&]() {
    std::vector<Bytes> rows(5);
    for (const auto& w : bus.writes)
      if (w.command >= 0x20 && w.command <= 0x24 && w.bytes.size() == 49) rows[w.command - 0x20] = w.bytes;
    for (const auto& row : rows) assert(row.size() == 49);
    return rows;
  };
  const auto render = [&](GrayscaleMode mode, uint8_t mask, bool factory) {
    std::fill(msb.begin(), msb.end(), mask);
    driver.beginGrayscale(bus, bw.data(), mode, RefreshMode::Half, false);
    driver.copyGrayscaleLsb(bus, lsb.data());
    driver.copyGrayscaleMsb(bus, msb.data());
    bus.clear();
    driver.displayGray(bus, bw.data(), false, nullptr, factory);
    return bank();
  };
  const auto quality = render(GrayscaleMode::Direct, 0, false);
  assert(quality[2] != quality[3]);
  assert(render(GrayscaleMode::Absolute, 0, false) == quality);
  for (uint8_t variant : {0x02, 0x68, 0x69}) {
    BoardConfig::ACTIVE.displayControllerVariant = variant;
    for (uint8_t mask : {0x00, 0x01, 0x03}) {
      const auto text = render(GrayscaleMode::Overlay, mask, false);
      assert(text != quality);
      assert(text[2] == text[3]);
      const uint8_t timing = variant == 0x02 ? 2 : 3;
      for (unsigned t = 0; t < 5; ++t) {
        Bytes expected = {1, 2, timing, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1};
        expected.resize(49, 0);
        if (t == 1) expected[3] |= 0x40;
        if (t == 2 || t == 3) expected[2] |= 0x80;
        if (t == 4) expected[3] |= 0x80;
        assert(text[t] == expected);
      }
    }
    assert(render(GrayscaleMode::Overlay, 0x07, false) == quality);
    assert(render(GrayscaleMode::Overlay, 0, true) == quality);
    // An image pass must not leave the next sparse page on the quality bank.
    assert(render(GrayscaleMode::Overlay, 0x01, false) != quality);
  }
  BoardConfig::ACTIVE.displayControllerVariant = 0x68;
  free(driver._grayBase);
}

static void testStickyAbsolute() {
  BoardConfig::ACTIVE.board = BoardConfig::Board::Sticky;
  auto& driver = static_cast<Ssd1677Driver&>(ssd1677Driver());
  assert(driver._cfg.grayPowerUpFirst);
  assert(driver._cfg.fullSeqOverride == 0xF7);
  assert(driver.grayscaleCapabilities(GrayscaleMode::Absolute).supported());
  FreeInkDisplay display(1, 2, 3, 4, 5, 6);
  display._driver = &driver;
  display.begin();
  const auto bw = frame(33), lsb = frame(14), msb = frame(29);
  std::memcpy(display.getFrameBuffer(), bw.data(), bw.size());
  display._bus.clear();
  assert(display.displayGrayscaleBase(GrayscaleMode::Absolute));
  assert(display._bus.writes.empty());
  display.copyGrayscaleBuffers(lsb.data(), msb.data());
  display.displayGrayBuffer(false);
  assert(lastRegister(display._bus, 0x22) == 0xCC);
  assert(driver._isScreenOn);
  display.cleanupGrayscaleBuffers(bw.data());
  display._bus.clear();
  display.displayBuffer(FreeInkDisplay::FAST_REFRESH);
  assert(lastRegister(display._bus, 0x22) == 0xF7);
  display.releaseBuffers();
}

static void testMetalio() {
  BoardConfig::ACTIVE.board = BoardConfig::Board::MetalioEInk4;
  auto& driver = static_cast<Ssd1677Driver&>(ssd1677Driver());
  EpdBus bus;
  driver.begin(bus);
  assert(lastRegister(bus, 0x3C) == 0x01);
  assert(driver.grayscaleCapabilities(GrayscaleMode::Absolute).supported());
  assert(driver.grayscaleCapabilities().supported());
  const auto bw = frame(42);
  bus.clear();
  // Cold FAST uses the demo's two-partial black pulse, never OTP D7. Both
  // phases run the vendor B/W fast LUT (0xCC activation, no OTP 0xFC).
  driver.display(bus, bw.data(), nullptr, RefreshMode::Fast, false);
  assert(lastRegister(bus, 0x22) == 0xCC);
  unsigned activations = 0;
  std::vector<Bytes> oldPlanes;
  for (const auto& w : bus.writes) {
    if (w.command == 0x20) ++activations;
    if (w.command == 0x26) oldPlanes.push_back(w.bytes);
    if (w.command == 0x22) assert(w.bytes == Bytes({0xCC}));
    if (w.command == 0x32) assert(w.bytes == Bytes(lut_fast_metalio, lut_fast_metalio + 105));
    assert(w.command != 0x1A);
  }
  assert(activations == 2 && bus.waits == 2);
  assert(oldPlanes.size() == 3);
  assert(oldPlanes[0] == Bytes(bw.size(), 0xFF));
  assert(oldPlanes[1] == Bytes(bw.size(), 0x00));
  assert(oldPlanes[2] == bw);
  assert(!driver._customLutActive);
  bus.clear();
  driver.display(bus, bw.data(), nullptr, RefreshMode::Fast, false);
  assert(lastRegister(bus, 0x22) == 0xCC);
  assert(lastRegister(bus, 0x18) == 0x80);
  assert(lastRegister(bus, 0x3C) == 0x80);
  assert(lastRegister(bus, 0x2C) == 0x10);
  for (const auto& w : bus.writes) if (w.command == 0x21) assert(w.bytes == Bytes({0, 0}));
  assert(driver._isScreenOn);
  bus.clear();
  driver.displayGray(bus, bw.data(), false, nullptr, false);
  // X4 external-LUT AA path: LUT + voltage tail written, border parked at 0xC0,
  // absolute 0xCC activation.
  assert(lastRegister(bus, 0x22) == 0xCC);
  assert(lastRegister(bus, 0x3C) == 0xC0);
  bool lutWritten = false;
  for (const auto& w : bus.writes) if (w.command == 0x32) lutWritten = true;
  assert(lutWritten);
  // Exit grayscale the way the facade does so the next FAST isn't promoted.
  driver.cleanupGrayscaleBuffers(bus, bw.data());
  bus.clear();
  driver.displayStart(bus, bw.data(), bw.data(), RefreshMode::Fast, true);
  assert(lastRegister(bus, 0x22) == 0xCF);  // LUT paint, then analog/clock off
  driver.displayFinish(bus, bw.data());
  assert(lastRegister(bus, 0x22) == 0xCF);
  assert(!driver._isScreenOn);
  // Deferred FAST must resync both planes from the frame retained by the facade.
  assert(lastPlane(bus, 0x24) == bw && lastPlane(bus, 0x26) == bw);
  bus.clear();
  driver.displayStart(bus, bw.data(), nullptr, RefreshMode::Half, true);
  assert(bus.waits == 1);  // black pulse done; target waveform still pending
  unsigned pulses = 0;
  for (const auto& w : bus.writes) if (w.command == 0x20) ++pulses;
  assert(pulses == 2);
  assert(lastPlane(bus, 0x26) == Bytes(bw.size(), 0));
  driver.displayFinish(bus, bw.data());
  assert(lastPlane(bus, 0x26) == bw && lastPlane(bus, 0x24) == bw);
  assert(lastRegister(bus, 0x22) == 0xCF);
  driver.deepSleep(bus);
  assert(lastRegister(bus, 0x10) == 0x03);
}

static size_t lastIndex(const EpdBus& bus, uint8_t command) {
  for (size_t i = bus.writes.size(); i-- > 0;)
    if (bus.writes[i].command == command) return i;
  assert(false);
  return 0;
}

static bool writtenAfter(const EpdBus& bus, uint8_t command, size_t index) {
  for (size_t i = index + 1; i < bus.writes.size(); ++i)
    if (bus.writes[i].command == command) return true;
  return false;
}

static unsigned commandCount(const EpdBus& bus, uint8_t command) {
  return unsigned(
      std::count_if(bus.writes.begin(), bus.writes.end(), [command](const auto& w) { return w.command == command; }));
}

// A clearing (non-differential) B/W refresh: no PTIN, full-waveform TSSET, DTM1
// seeded white before the DRF, then the frame committed after it.
static void assertFullClear(const EpdBus& bus, const Uc8179Driver& d) {
  const size_t drf = lastIndex(bus, 0x12);
  assert(commandCount(bus, 0x91) == 0 && lastRegister(bus, 0xe5) == 0x1e);
  const auto first =
      std::find_if(bus.writes.begin(), bus.writes.end(), [](const auto& w) { return w.command == 0x10; });
  assert(first != bus.writes.end() && size_t(first - bus.writes.begin()) < drf);
  assert(std::all_of(first->bytes.begin(), first->bytes.end(), [](uint8_t v) { return v == 0xff; }));
  assert(writtenAfter(bus, 0x10, drf));
  assert(d._oldPlaneValid && !d._needFullClear);
}

// A PON or DRF whose BUSY_N never asserted must not be committed as the DTM1
// differential baseline, and the next refresh must be a full clear.
static void testUc8179LostActivation() {
  const auto a = frame(5), b = frame(71), c = frame(123), lsb = frame(13), msb = frame(29);
  Uc8179Driver d;
  EpdBus bus;
  d.begin(bus);
  d.display(bus, a.data(), nullptr, RefreshMode::Full, false);
  assert(d._oldPlaneValid && !d._needFullClear && d._isScreenOn);
  hostLostActivations = 0;

  // Lost DRF on a differential page: BUSY_N never drops inside the start poll.
  bus.clear();
  hostBusyLevel = HIGH;
  d.display(bus, b.data(), nullptr, RefreshMode::Fast, false);
  hostBusyLevel = 0;
  assert(commandCount(bus, 0x91) == 1);                    // it was the DU partial
  assert(!writtenAfter(bus, 0x10, lastIndex(bus, 0x12)));  // ...and was NOT committed
  assert(!d._oldPlaneValid && d._needFullClear && !d._bwPlanesSynced);
  assert(d._isScreenOn && commandCount(bus, 0x02) == 0);  // PON was fine: rails stay up
  assert(hostLostActivations == 1);
  // The next Fast request self-heals as a full clear, then DU resumes.
  bus.clear();
  d.display(bus, c.data(), nullptr, RefreshMode::Fast, false);
  assertFullClear(bus, d);
  bus.clear();
  d.display(bus, a.data(), nullptr, RefreshMode::Fast, false);
  assert(commandCount(bus, 0x91) == 1 && writtenAfter(bus, 0x10, lastIndex(bus, 0x12)));
  assert(hostLostActivations == 1);

  // PON lost, and its re-issue too: the frame still fires (never skipped), but
  // is not committed, the rails are not claimed, and they are parked so the
  // next PON starts from a known-off state.
  d.display(bus, b.data(), nullptr, RefreshMode::Fast, true);  // POF: rails down
  assert(!d._isScreenOn);
  bus.clear();
  bus.failWaits = {1, 2};  // 8179_PON, 8179_PON_retry
  d.display(bus, c.data(), nullptr, RefreshMode::Fast, false);
  const size_t drf = lastIndex(bus, 0x12);
  assert(commandCount(bus, 0x04) == 2 && writtenAfter(bus, 0x02, drf));
  assert(!writtenAfter(bus, 0x10, drf));
  assert(!d._isScreenOn && !d._oldPlaneValid && d._needFullClear);
  assert(hostLostActivations == 2);
  bus.clear();
  d.display(bus, a.data(), nullptr, RefreshMode::Fast, false);
  assert(commandCount(bus, 0x04) == 1 && d._isScreenOn);
  assertFullClear(bus, d);

  // A PON whose re-issue is observed is a recovered frame, not a lost one.
  d.display(bus, b.data(), nullptr, RefreshMode::Fast, true);
  bus.clear();
  bus.failWaits = {1};
  d.display(bus, c.data(), nullptr, RefreshMode::Fast, false);
  assert(commandCount(bus, 0x04) == 2 && commandCount(bus, 0x02) == 0);
  assert(commandCount(bus, 0x91) == 1 && writtenAfter(bus, 0x10, lastIndex(bus, 0x12)));
  assert(d._isScreenOn && d._oldPlaneValid && !d._needFullClear);
  assert(hostLostActivations == 2);

  // AA page, so the next base goes through the XTF_PRE_BW_MID transition.
  const auto aaPage = [&](const Bytes& base) {
    d.beginGrayscale(bus, base.data(), GrayscaleMode::Overlay, RefreshMode::Fast, false);
    d.copyGrayscaleLsb(bus, lsb.data());
    d.copyGrayscaleMsb(bus, msb.data());
  };
  aaPage(a);
  d.displayGray(bus, msb.data(), false, nullptr, false);
  assert(d._grayRefreshedOnce && d._redriveAfterGray && d._oldPlaneValid && !d._needFullClear);

  // Lost gray pre-pass DRF (waits: gray_base_ready, gray_pre_ready, DRF).
  bus.clear();
  bus.failWaits = {3};
#ifdef FREEINK_UC8179_DOUBLE_GRAY_PRE
  d.requestDeepGrayEqualize();
#endif
  d.beginGrayscale(bus, b.data(), GrayscaleMode::Overlay, RefreshMode::Fast, false);
  assert(commandCount(bus, 0x12) == 1);  // also: no equalize pass after a lost transition
  assert(!writtenAfter(bus, 0x10, lastIndex(bus, 0x12)));
  assert(!d._oldPlaneValid && d._needFullClear && hostLostActivations == 3);
#ifdef FREEINK_UC8179_LEAN_STREAMS
  assert(!d._dtm1Stale);
#endif
#ifdef FREEINK_UC8179_DOUBLE_GRAY_PRE
  assert(!d._deepEqualizeNext);
#endif
  // The AA pass on top still renders, but must not launder the lost base back
  // into the baseline through its own DTM1 restore.
  d.copyGrayscaleLsb(bus, lsb.data());
  d.copyGrayscaleMsb(bus, msb.data());
  bus.clear();
  d.displayGray(bus, msb.data(), false, nullptr, false);
  assert(commandCount(bus, 0x12) == 1 && !writtenAfter(bus, 0x10, lastIndex(bus, 0x12)));
  assert(!d._oldPlaneValid && d._needFullClear && hostLostActivations == 3);
  d.cleanupGrayscaleBuffers(bus, msb.data());
  assert(d._needFullClear);  // the plane restore does not cancel the owed clear
  bus.clear();
  aaPage(c);
  assertFullClear(bus, d);
  d.displayGray(bus, msb.data(), false, nullptr, false);

  // Lost AA activation itself (waits: gray_ready, DRF).
  aaPage(a);
  bus.clear();
  bus.failWaits = {2};
  d.displayGray(bus, msb.data(), false, nullptr, false);
  assert(!writtenAfter(bus, 0x10, lastIndex(bus, 0x12)));
  assert(!d._oldPlaneValid && d._needFullClear && hostLostActivations == 4);
  bus.clear();
  d.display(bus, b.data(), nullptr, RefreshMode::Fast, false);  // no post-AA transition
  assertFullClear(bus, d);

  // Lost direct-gray DRF (waits: direct_ready, PON — rails went down with the
  // direct setup's reset — then DRF). Nothing to commit there; the next B/W page
  // still repaints through the BW_TARGET pre-pass and a full clear.
  d.beginGrayscale(bus, a.data(), GrayscaleMode::Direct, RefreshMode::Fast, false);
  d.copyGrayscaleLsb(bus, lsb.data());
  d.copyGrayscaleMsb(bus, msb.data());
  bus.clear();
  bus.failWaits = {3};
  d.displayGray(bus, msb.data(), false, nullptr, true);
  assert(commandCount(bus, 0x12) == 1 && hostLostActivations == 5);
  assert(d._directGrayOnPanel && d._needFullClear && !d._oldPlaneValid);
  bus.clear();
  d.display(bus, b.data(), nullptr, RefreshMode::Fast, false);
  assert(commandCount(bus, 0x12) == 2 && commandCount(bus, 0x91) == 1);  // BW_TARGET is the only PTIN
  assert(lastRegister(bus, 0xe5) == 0x1e && writtenAfter(bus, 0x10, lastIndex(bus, 0x12)));
  assert(d._oldPlaneValid && !d._needFullClear && hostLostActivations == 5);

  // Lost PON on a turnOff page: activationLanded() parks the rails, and the
  // turnOff tail must not send a second POF.
  d.display(bus, a.data(), nullptr, RefreshMode::Fast, true);
  bus.clear();
  bus.failWaits = {1, 2};
  d.display(bus, b.data(), nullptr, RefreshMode::Fast, true);
  assert(commandCount(bus, 0x02) == 1 && !d._isScreenOn && hostLostActivations == 6);
  assert(hostNotedLostActivations == hostLostActivations);  // counter tracks the log line
  free(d._grayBase);
}

static void aaPage(Uc8179Driver& d, EpdBus& bus, const Bytes& base) {
  const auto lsb = frame(13), msb = frame(29);
  d.beginGrayscale(bus, base.data(), GrayscaleMode::Overlay, RefreshMode::Fast, false);
  d.copyGrayscaleLsb(bus, lsb.data());
  d.copyGrayscaleMsb(bus, msb.data());
  d.displayGray(bus, msb.data(), false, nullptr, false);
}

#ifdef FREEINK_UC8179_RAIL_POWEROFF
// Healthy prewarm: the PON is fired from beginDisplayWork() and ridden out by
// settlePrewarm(), so the display path issues none and the detector stays quiet —
// including when composition outlasted the ramp and neither prewarm wait saw it.
static void testUc8179HealthyPrewarm() {
  const auto a = frame(5), b = frame(71);
  const unsigned lost = hostLostActivations;
  Uc8179Driver d;
  EpdBus bus;
  d.begin(bus);
  d.display(bus, a.data(), nullptr, RefreshMode::Full, false);
  for (bool rampSeen : {true, false}) {
    d.controllerIdle(bus);
    assert(!d._isScreenOn);
    bus.clear();
    d.beginDisplayWork();
    assert(d._ponPending && !d._isScreenOn);
    if (!rampSeen) bus.failWaits = {1, 2};  // 8179_prewarm_PON, 8179_prewarm_PON_retry
    d.display(bus, b.data(), nullptr, RefreshMode::Fast, false);
    assert(commandCount(bus, 0x04) == (rampSeen ? 1u : 2u) && commandCount(bus, 0x02) == 0);
    assert(d._isScreenOn && commandCount(bus, 0x91) == 1 && writtenAfter(bus, 0x10, lastIndex(bus, 0x12)));
    assert(d._oldPlaneValid && !d._needFullClear);
  }
  assert(hostLostActivations == lost);
  free(d._grayBase);
}
#endif

#ifdef FREEINK_UC8179_DOUBLE_GRAY_PRE
static void testUc8179DeepEqualize() {
  const auto a = frame(5), b = frame(71), c = frame(123), lsb = frame(13), msb = frame(29);
  const unsigned lost = hostLostActivations;
  Uc8179Driver d;
  EpdBus bus;
  d.begin(bus);
  d.display(bus, a.data(), nullptr, RefreshMode::Full, false);
  aaPage(d, bus, a);

  // Healthy equalize: two pre-pass activations, baseline committed, quiet.
  bus.clear();
  d.requestDeepGrayEqualize();
  d.beginGrayscale(bus, b.data(), GrayscaleMode::Overlay, RefreshMode::Fast, false);
  assert(commandCount(bus, 0x12) == 2 && !d._deepEqualizeNext);
  assert(d._oldPlaneValid && !d._needFullClear && hostLostActivations == lost);
  d.copyGrayscaleLsb(bus, lsb.data());
  d.copyGrayscaleMsb(bus, msb.data());
  bus.clear();
  d.displayGray(bus, msb.data(), false, nullptr, false);
  assert(writtenAfter(bus, 0x10, lastIndex(bus, 0x12)) && !d._needFullClear);

  // Lost SECOND pass. Waits: gray_base_ready, gray_pre_ready, DRF,
  // [lean: old_plane_flush], gray_pre_ready, DRF.
  bus.clear();
#ifdef FREEINK_UC8179_LEAN_STREAMS
  bus.failWaits = {6};
#else
  bus.failWaits = {5};
#endif
  d.requestDeepGrayEqualize();
  d.beginGrayscale(bus, c.data(), GrayscaleMode::Overlay, RefreshMode::Fast, false);
  assert(commandCount(bus, 0x12) == 2 && hostLostActivations == lost + 1);
  assert(!d._oldPlaneValid && d._needFullClear);
  d.copyGrayscaleLsb(bus, lsb.data());
  d.copyGrayscaleMsb(bus, msb.data());
  bus.clear();
  d.displayGray(bus, msb.data(), false, nullptr, false);
  assert(!writtenAfter(bus, 0x10, lastIndex(bus, 0x12)) && d._needFullClear);
  bus.clear();
  d.beginGrayscale(bus, a.data(), GrayscaleMode::Overlay, RefreshMode::Fast, false);
  assertFullClear(bus, d);
  free(d._grayBase);
}
#endif

int main(int argc, char** argv) {
  if (argc > 1 && std::strcmp(argv[1], "metalio") == 0) {
    testMetalio();
    std::puts("Metalio refresh, temperature, grayscale fallback and async power-down passed");
    return 0;
  }
  if (argc > 1) {
    testStickyAbsolute();
    std::puts("Sticky absolute capability, activation, power-down and B/W recovery passed");
    return 0;
  }
  testUc8179GrayShadeSplit();
  testUc8279X4WaveformSelection();
  testDirectSleep<Uc8179Driver>();
  testDirectSleep<Uc8279X4Driver>();
  testUltraChipAbsolute<Uc8179Driver>(true, 0, false);
  for (uint8_t variant : {0x02, 0x68, 0x69}) {
    BoardConfig::ACTIVE.displayControllerVariant = variant;
    testUltraChipAbsolute<Uc8279X4Driver>(false, 120, true);
  }
  testAbsolutePipeline();
  testCapabilities();
  testStream<Uc8179Driver>(true,0);
  testStream<Uc8279X4Driver>(false,120);
  testSsd();
  testAsyncFrame<Uc8179Driver>();
  testAsyncFrame<Uc8279X4Driver>();
  assert(hostLostActivations == 0);  // no healthy-path flow above trips the detector
  testUc8179LostActivation();
#ifdef FREEINK_UC8179_RAIL_POWEROFF
  testUc8179HealthyPrewarm();
#endif
#ifdef FREEINK_UC8179_DOUBLE_GRAY_PRE
  testUc8179DeepEqualize();
#endif
  std::puts("Pro plane bytes, transaction counts, clean refreshes, power state and async frame ownership passed");
}
