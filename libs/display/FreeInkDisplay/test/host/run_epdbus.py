#!/usr/bin/env python3
"""Compile the real EpdBus against a simulated clock and a scripted BUSY pin."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parents[1] / "src"
with tempfile.TemporaryDirectory(prefix="freeink-epdbus-test-") as directory:
    root = Path(directory)
    for folder in ("bus", "driver", "freertos"):
        (root / folder).mkdir()
    for name in ("EpdBus.cpp", "EpdBus.h"):
        shutil.copy2(SOURCE / "bus" / name, root / "bus" / name)
    # Time only moves when the code under test waits, so every wait is exact.
    # BUSY is read through hostBusyAt(), which the test scripts per scenario.
    (root / "Arduino.h").write_text('''#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstring>
#define HIGH 1
#define LOW 0
#define INPUT 1
#define OUTPUT 3
#define INPUT_PULLUP 5
#define CHANGE 3
#define MSBFIRST 1
#define IRAM_ATTR
#define DRAM_ATTR
inline uint64_t hostMicros = 0;
inline int8_t hostBusyPin = -1;
inline unsigned hostBusyReads = 0;
int hostBusyAt(uint64_t us);
inline unsigned long millis() { return static_cast<unsigned long>(hostMicros / 1000); }
inline void delay(unsigned long ms) { hostMicros += ms * 1000; }
inline void delayMicroseconds(unsigned us) { hostMicros += us; }
inline int digitalRead(int pin) {
  if (pin != hostBusyPin) return LOW;
  ++hostBusyReads;
  return hostBusyAt(hostMicros);
}
inline void digitalWrite(int, int) {}
inline void pinMode(int, int) {}
inline int digitalPinToInterrupt(int pin) { return pin; }
inline void attachInterrupt(int, void (*)(), int) {}
inline void detachInterrupt(int) {}
struct HostSerial {
  explicit operator bool() const { return true; }
  template <class... A> void printf(const char* f, A... a) { std::printf(f, a...); }
};
inline HostSerial Serial;
''')
    (root / "SPI.h").write_text('''#pragma once
#include <cstdint>
#define SPI_MODE0 0
struct SPISettings {
  SPISettings() = default;
  SPISettings(uint32_t, int, int) {}
};
struct SPIClass {
  void begin(int8_t, int8_t, int8_t, int8_t) {}
  void beginTransaction(const SPISettings&) {}
  void endTransaction() {}
  uint8_t transfer(uint8_t) { return 0; }
  void writeBytes(const uint8_t*, uint32_t) {}
};
inline SPIClass SPI;
''')
    (root / "BoardConfig.h").write_text('''#pragma once
namespace BoardConfig {
inline bool isOnePage() { return false; }
}
''')
    (root / "driver/gpio.h").write_text('''#pragma once
typedef int gpio_num_t;
inline void gpio_hold_dis(gpio_num_t) {}
''')
    (root / "freertos/FreeRTOS.h").write_text('''#pragma once
typedef int BaseType_t;
typedef void* SemaphoreHandle_t;
#define pdFALSE 0
#define pdTRUE 1
#define pdMS_TO_TICKS(ms) (ms)
#define portYIELD_FROM_ISR() do {} while (0)
''')
    (root / "freertos/semphr.h").write_text('''#pragma once
inline SemaphoreHandle_t xSemaphoreCreateBinary() { static int s; return &s; }
inline BaseType_t xSemaphoreTake(SemaphoreHandle_t, int) { return pdFALSE; }
inline BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t, BaseType_t*) { return pdTRUE; }
''')
    exe = root / "epdbus"
    subprocess.run([os.environ.get("CXX", "c++"), "-std=c++17", "-Wall", "-Wextra", "-Wno-unused-parameter",
                    "-DARDUINO=1", "-I" + str(root), str(HERE / "test_epdbus.cpp"), str(root / "bus/EpdBus.cpp"),
                    "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
