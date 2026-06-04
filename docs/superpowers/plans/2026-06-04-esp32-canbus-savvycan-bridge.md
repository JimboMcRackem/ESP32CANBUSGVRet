# ESP32 CAN-to-SavvyCAN Bridge (GVRET) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build ESP-IDF firmware for an ESP32-WROOM-32 that reads CAN frames via the built-in TWAI controller and streams them over USB serial to SavvyCAN using the GVRET protocol, bidirectionally, with robust error recovery and LED/button indicators.

**Architecture:** Portable, hardware-independent protocol/settings logic lives in PlatformIO `lib/` libraries (unit-tested natively); ESP-IDF hardware glue lives in `src/`. Two FreeRTOS tasks (CAN receive + host comms) are decoupled by a queue so bus timing never blocks on USB. The GVRET protocol layer translates between `CanFrame` structs and SavvyCAN's binary wire format.

**Tech Stack:** PlatformIO, ESP-IDF framework, C++17, ESP-IDF TWAI driver (`driver/twai.h`), NVS, FreeRTOS, esp_task_wdt; Unity test framework on the PlatformIO `native` platform for host-side unit tests.

---

## Reference

The GVRET serial protocol byte layouts below mirror the canonical open-source ESP32RET / GVRET-Serial firmware (the firmware SavvyCAN was designed against):
- ESP32RET: https://github.com/collin80/ESP32RET
- GVRET: https://github.com/collin80/GVRET

When verifying handshake byte layouts during Task 3 and Task 9, cross-check against `SerialConsole`/`processIncomingByte` in that source.

## File Structure

Portable (compiled for both `native` tests and the ESP32):
- `lib/gvret/can_frame.h` — `CanFrame` POD struct shared everywhere. No IDF deps.
- `lib/gvret/gvret.h` / `lib/gvret/gvret.cpp` — GVRET constants, `encodeFrame()`, `Parser` + `Handler` interface. No IDF deps.
- `lib/gvret/CMakeLists.txt` — registers the lib as an IDF component.
- `lib/settings/settings.h` / `lib/settings/settings.cpp` — pure settings struct + `sanitizeBitrate()`. No IDF deps.
- `lib/settings/CMakeLists.txt` — registers the lib as an IDF component.

ESP32-only (compiled only for the `esp32dev` env, never in `native` tests):
- `src/CMakeLists.txt` — IDF component registration for the app sources.
- `src/config_store.h` / `src/config_store.cpp` — NVS persistence of bitrate.
- `src/can_driver.h` / `src/can_driver.cpp` — TWAI wrapper (init, rx/tx, alerts/bus-off recovery, reconfigure).
- `src/usb_serial.h` / `src/usb_serial.cpp` — UART0 transport at 1 Mbaud.
- `src/indicators.h` / `src/indicators.cpp` — LED states + GPIO25 button (debounce, short/long press).
- `src/main.cpp` — `app_main()`, FreeRTOS tasks, `Handler` implementation, watchdog.

Project root:
- `platformio.ini` — `esp32dev` (espidf) + `native` (unity) envs.
- `.gitignore` — append `.pio/`.

**Why `lib/` for portable code:** PlatformIO's test runner does **not** compile `src/` during `pio test` (the `test_build_src` option defaults to false), so IDF-only code in `src/` never reaches the native toolchain. Code in `lib/` is compiled into tests on demand by the Library Dependency Finder when a test includes its header, and is compiled as an IDF component for the firmware build. This keeps the protocol logic testable on the host with zero hardware.

---

## Task 0: Project scaffold and verify both environments build

**Files:**
- Create: `platformio.ini`
- Create: `src/CMakeLists.txt`
- Create: `src/main.cpp`
- Modify: `.gitignore`

- [ ] **Step 1: Create `platformio.ini`**

```ini
[platformio]
default_envs = esp32dev

[env:esp32dev]
platform = espressif32
board = esp32dev
framework = espidf
monitor_speed = 115200
build_flags =
    -std=gnu++17

[env:native]
platform = native
test_framework = unity
build_flags =
    -std=gnu++17
    -Wall
```

- [ ] **Step 2: Append `.pio/` to `.gitignore`**

Add these lines to the end of `.gitignore`:

```gitignore
# PlatformIO
.pio/
.vscode/.browse.c_cpp.db*
.vscode/c_cpp_properties.json
.vscode/launch.json
.vscode/ipch
```

- [ ] **Step 3: Create `src/CMakeLists.txt`**

```cmake
idf_component_register(
    SRCS
        "main.cpp"
        "config_store.cpp"
        "can_driver.cpp"
        "usb_serial.cpp"
        "indicators.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver nvs_flash esp_timer freertos esp_system esp_common
    PRIV_REQUIRES gvret settings
)
```

> Note: `config_store.cpp`, `can_driver.cpp`, `usb_serial.cpp`, and `indicators.cpp` do not exist yet. To keep this task's build green, create empty stub files for them now; later tasks fill them in.

- [ ] **Step 4: Create stub source files so the IDF build links**

Create `src/config_store.cpp`, `src/can_driver.cpp`, `src/usb_serial.cpp`, `src/indicators.cpp` each containing only:

```cpp
// stub — implemented in a later task
```

- [ ] **Step 5: Create minimal `src/main.cpp`**

```cpp
#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" void app_main(void) {
    printf("esp32 canbus gvret bridge: boot\n");
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

- [ ] **Step 6: Verify the firmware build succeeds**

Run: `pio run -e esp32dev`
Expected: build completes, ends with `SUCCESS`. (First run downloads the ESP-IDF toolchain; this can take several minutes.)

- [ ] **Step 7: Verify the native environment is usable**

Run: `pio test -e native`
Expected: PlatformIO reports `No tests found` (no test files yet) but the env initializes without error. This confirms the `native` toolchain works before we depend on it.

- [ ] **Step 8: Commit**

```bash
git add platformio.ini .gitignore src/
git commit -m "chore: scaffold PlatformIO ESP-IDF project with native test env"
```

---

## Task 1: Settings + bitrate sanitization (portable, TDD)

**Files:**
- Create: `lib/settings/settings.h`
- Create: `lib/settings/settings.cpp`
- Create: `lib/settings/CMakeLists.txt`
- Test: `test/test_settings/test_settings.cpp`

- [ ] **Step 1: Write the failing test**

Create `test/test_settings/test_settings.cpp`:

```cpp
#include <unity.h>
#include "settings.h"

void setUp(void) {}
void tearDown(void) {}

void test_default_bitrate_is_500k(void) {
    TEST_ASSERT_EQUAL_UINT32(500000u, settings::kDefaultBitrate);
}

void test_valid_bitrate_passes_through(void) {
    TEST_ASSERT_EQUAL_UINT32(250000u, settings::sanitizeBitrate(250000u));
    TEST_ASSERT_EQUAL_UINT32(1000000u, settings::sanitizeBitrate(1000000u));
}

void test_zero_bitrate_falls_back_to_default(void) {
    TEST_ASSERT_EQUAL_UINT32(settings::kDefaultBitrate, settings::sanitizeBitrate(0u));
}

void test_out_of_range_bitrate_falls_back_to_default(void) {
    TEST_ASSERT_EQUAL_UINT32(settings::kDefaultBitrate, settings::sanitizeBitrate(5u));
    TEST_ASSERT_EQUAL_UINT32(settings::kDefaultBitrate, settings::sanitizeBitrate(2000000u));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_default_bitrate_is_500k);
    RUN_TEST(test_valid_bitrate_passes_through);
    RUN_TEST(test_zero_bitrate_falls_back_to_default);
    RUN_TEST(test_out_of_range_bitrate_falls_back_to_default);
    return UNITY_END();
}
```

- [ ] **Step 2: Create the header**

Create `lib/settings/settings.h`:

```cpp
#pragma once
#include <cstdint>

namespace settings {

constexpr uint32_t kDefaultBitrate = 500000;
constexpr uint32_t kMinBitrate = 10000;
constexpr uint32_t kMaxBitrate = 1000000;

// Returns `requested` if it is within [kMinBitrate, kMaxBitrate];
// otherwise returns kDefaultBitrate. Treats 0 / out-of-range as invalid.
uint32_t sanitizeBitrate(uint32_t requested);

}  // namespace settings
```

- [ ] **Step 3: Run test to verify it fails**

Run: `pio test -e native -f test_settings`
Expected: FAIL — undefined reference to `settings::sanitizeBitrate`.

- [ ] **Step 4: Implement**

Create `lib/settings/settings.cpp`:

```cpp
#include "settings.h"

namespace settings {

uint32_t sanitizeBitrate(uint32_t requested) {
    if (requested < kMinBitrate || requested > kMaxBitrate) {
        return kDefaultBitrate;
    }
    return requested;
}

}  // namespace settings
```

- [ ] **Step 5: Create the IDF component manifest**

Create `lib/settings/CMakeLists.txt`:

```cmake
idf_component_register(SRCS "settings.cpp" INCLUDE_DIRS ".")
```

- [ ] **Step 6: Run test to verify it passes**

Run: `pio test -e native -f test_settings`
Expected: PASS — 4 tests, 0 failures.

- [ ] **Step 7: Commit**

```bash
git add lib/settings test/test_settings
git commit -m "feat(settings): add bitrate sanitization with unit tests"
```

---

## Task 2: CanFrame struct + GVRET frame encoder (portable, TDD)

**Files:**
- Create: `lib/gvret/can_frame.h`
- Create: `lib/gvret/gvret.h`
- Create: `lib/gvret/gvret.cpp`
- Create: `lib/gvret/CMakeLists.txt`
- Test: `test/test_gvret/test_gvret_encode.cpp`

- [ ] **Step 1: Create the shared CanFrame struct**

Create `lib/gvret/can_frame.h`:

```cpp
#pragma once
#include <cstdint>

// Hardware-independent representation of a single CAN frame, shared between the
// TWAI driver layer and the GVRET protocol layer.
struct CanFrame {
    uint32_t id = 0;            // 11-bit or 29-bit identifier
    bool extended = false;      // true => 29-bit extended ID
    bool rtr = false;           // remote transmission request
    uint8_t dlc = 0;            // data length code, 0..8
    uint8_t data[8] = {0};
    uint32_t timestamp_us = 0;  // capture time, microseconds
    uint8_t bus = 0;            // originating/target bus index (0)
};
```

- [ ] **Step 2: Write the failing test**

Create `test/test_gvret/test_gvret_encode.cpp`:

```cpp
#include <unity.h>
#include <cstring>
#include "gvret.h"

void setUp(void) {}
void tearDown(void) {}

void test_encode_standard_frame_layout(void) {
    CanFrame f;
    f.id = 0x123;
    f.extended = false;
    f.dlc = 2;
    f.data[0] = 0xAA;
    f.data[1] = 0xBB;
    f.timestamp_us = 0x04030201;
    f.bus = 0;

    uint8_t out[gvret::kMaxEncodedFrame];
    size_t n = gvret::encodeFrame(f, out, sizeof(out));

    TEST_ASSERT_EQUAL_size_t(12u + 2u, n);
    TEST_ASSERT_EQUAL_HEX8(0xF1, out[0]);          // prefix
    TEST_ASSERT_EQUAL_HEX8(0x00, out[1]);          // build-can-frame marker
    TEST_ASSERT_EQUAL_HEX8(0x01, out[2]);          // timestamp LE
    TEST_ASSERT_EQUAL_HEX8(0x02, out[3]);
    TEST_ASSERT_EQUAL_HEX8(0x03, out[4]);
    TEST_ASSERT_EQUAL_HEX8(0x04, out[5]);
    TEST_ASSERT_EQUAL_HEX8(0x23, out[6]);          // id LE
    TEST_ASSERT_EQUAL_HEX8(0x01, out[7]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[8]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[9]);          // bit 31 clear => standard
    TEST_ASSERT_EQUAL_HEX8(0x02, out[10]);         // dlc | (bus<<4)
    TEST_ASSERT_EQUAL_HEX8(0xAA, out[11]);
    TEST_ASSERT_EQUAL_HEX8(0xBB, out[12]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[13]);         // checksum byte
}

void test_encode_extended_frame_sets_high_bit(void) {
    CanFrame f;
    f.id = 0x18DAF110;
    f.extended = true;
    f.dlc = 0;
    f.timestamp_us = 0;

    uint8_t out[gvret::kMaxEncodedFrame];
    size_t n = gvret::encodeFrame(f, out, sizeof(out));

    TEST_ASSERT_EQUAL_size_t(12u, n);
    uint32_t id = (uint32_t)out[6] | ((uint32_t)out[7] << 8) |
                  ((uint32_t)out[8] << 16) | ((uint32_t)out[9] << 24);
    TEST_ASSERT_TRUE((id & 0x80000000u) != 0u);    // extended flag
    TEST_ASSERT_EQUAL_HEX32(0x18DAF110u, id & 0x7FFFFFFFu);
}

void test_encode_clamps_dlc_and_respects_capacity(void) {
    CanFrame f;
    f.dlc = 8;
    uint8_t out[gvret::kMaxEncodedFrame];
    size_t n = gvret::encodeFrame(f, out, sizeof(out));
    TEST_ASSERT_EQUAL_size_t(20u, n);

    // Buffer too small => returns 0, writes nothing.
    uint8_t tiny[4];
    TEST_ASSERT_EQUAL_size_t(0u, gvret::encodeFrame(f, tiny, sizeof(tiny)));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_encode_standard_frame_layout);
    RUN_TEST(test_encode_extended_frame_sets_high_bit);
    RUN_TEST(test_encode_clamps_dlc_and_respects_capacity);
    return UNITY_END();
}
```

- [ ] **Step 3: Create the gvret header**

Create `lib/gvret/gvret.h`:

```cpp
#pragma once
#include <cstddef>
#include <cstdint>
#include "can_frame.h"

namespace gvret {

// Protocol bytes (mirror ESP32RET / GVRET).
constexpr uint8_t kCmdPrefix       = 0xF1;
constexpr uint8_t kBuildCanFrame   = 0;
constexpr uint8_t kTimeSync        = 1;
constexpr uint8_t kGetDigInputs    = 2;
constexpr uint8_t kGetAnaInputs    = 3;
constexpr uint8_t kSetDigOutputs   = 4;
constexpr uint8_t kSetupCanbus     = 5;
constexpr uint8_t kGetCanbusParams = 6;
constexpr uint8_t kGetDeviceInfo   = 7;
constexpr uint8_t kSetSingleWire   = 8;
constexpr uint8_t kKeepAlive       = 9;
constexpr uint8_t kSetSystemType   = 10;
constexpr uint8_t kEchoCanFrame    = 11;
constexpr uint8_t kGetNumBuses     = 12;
constexpr uint8_t kGetExtBuses     = 13;
constexpr uint8_t kSetExtBuses     = 14;

// Reported to SavvyCAN in GET_DEVICE_INFO.
constexpr uint16_t kBuildNumber   = 0x0001;
constexpr uint8_t  kEepromVersion = 0x14;
constexpr uint8_t  kNumBuses      = 1;

// 2 header + 4 timestamp + 4 id + 1 dlc/bus + 8 data + 1 checksum.
constexpr size_t kMaxEncodedFrame = 20;

// Encode a received CAN frame into GVRET binary for SavvyCAN.
// Returns bytes written, or 0 if outCap is too small. DLC is clamped to 8.
size_t encodeFrame(const CanFrame& f, uint8_t* out, size_t outCap);

// Actions the parser performs in response to host commands.
struct Handler {
    virtual ~Handler() = default;
    virtual void onTransmitFrame(const CanFrame& f) = 0;
    virtual void onSetBitrate(uint8_t bus, uint32_t bitrate,
                              bool enabled, bool listenOnly) = 0;
    virtual uint32_t currentBitrate(uint8_t bus) = 0;
    virtual bool busEnabled(uint8_t bus) = 0;
    virtual uint32_t nowMicros() = 0;
    virtual void writeBytes(const uint8_t* data, size_t len) = 0;
    // Called whenever any valid command is received (for connection tracking).
    virtual void onHostCommand() {}
};

// Byte-at-a-time parser for the host -> device GVRET command stream.
class Parser {
public:
    explicit Parser(Handler& handler) : handler_(handler) {}
    void feed(uint8_t b);
    void reset();

private:
    enum class State : uint8_t { Prefix, Command, Payload };
    Handler& handler_;
    State state_ = State::Prefix;
    uint8_t cmd_ = 0;
    uint8_t buf_[16] = {0};
    uint8_t got_ = 0;
    uint8_t need_ = 0;   // fixed payload length; 0xFF => variable (build frame)

    void beginCommand(uint8_t cmd);
    void completeFixedCommand();
    void feedBuildFrameByte(uint8_t b);
    // Build-can-frame assembly state:
    CanFrame txFrame_{};
    uint8_t bfStep_ = 0;
    uint8_t bfDataIdx_ = 0;

    void replyDeviceInfo();
    void replyCanbusParams();
    void replyTimeSync();
    void replyKeepAlive();
    void replyNumBuses();
    void replyExtBuses();
};

}  // namespace gvret
```

- [ ] **Step 4: Implement `encodeFrame` (Parser bodies come in Task 3)**

Create `lib/gvret/gvret.cpp` with the encoder and empty-but-present Parser members so the lib links:

```cpp
#include "gvret.h"

namespace gvret {

size_t encodeFrame(const CanFrame& f, uint8_t* out, size_t outCap) {
    uint8_t dlc = f.dlc > 8 ? 8 : f.dlc;
    size_t needed = 12u + dlc;
    if (outCap < needed) return 0;

    uint32_t id = f.id;
    if (f.extended) id |= 0x80000000u;

    out[0] = kCmdPrefix;
    out[1] = kBuildCanFrame;
    out[2] = (uint8_t)(f.timestamp_us & 0xFF);
    out[3] = (uint8_t)((f.timestamp_us >> 8) & 0xFF);
    out[4] = (uint8_t)((f.timestamp_us >> 16) & 0xFF);
    out[5] = (uint8_t)((f.timestamp_us >> 24) & 0xFF);
    out[6] = (uint8_t)(id & 0xFF);
    out[7] = (uint8_t)((id >> 8) & 0xFF);
    out[8] = (uint8_t)((id >> 16) & 0xFF);
    out[9] = (uint8_t)((id >> 24) & 0xFF);
    out[10] = (uint8_t)(dlc + (uint8_t)(f.bus << 4));
    for (uint8_t i = 0; i < dlc; ++i) out[11 + i] = f.data[i];
    out[11 + dlc] = 0;  // checksum (always 0 in GVRET)
    return needed;
}

// --- Parser members are implemented in Task 3 ---
void Parser::feed(uint8_t) {}
void Parser::reset() {}
void Parser::beginCommand(uint8_t) {}
void Parser::completeFixedCommand() {}
void Parser::feedBuildFrameByte(uint8_t) {}
void Parser::replyDeviceInfo() {}
void Parser::replyCanbusParams() {}
void Parser::replyTimeSync() {}
void Parser::replyKeepAlive() {}
void Parser::replyNumBuses() {}
void Parser::replyExtBuses() {}

}  // namespace gvret
```

- [ ] **Step 5: Create the IDF component manifest**

Create `lib/gvret/CMakeLists.txt`:

```cmake
idf_component_register(SRCS "gvret.cpp" INCLUDE_DIRS ".")
```

- [ ] **Step 6: Run test to verify it passes**

Run: `pio test -e native -f test_gvret`
Expected: PASS — 3 tests, 0 failures.

- [ ] **Step 7: Commit**

```bash
git add lib/gvret test/test_gvret
git commit -m "feat(gvret): add CanFrame and GVRET frame encoder with tests"
```

---

## Task 3: GVRET command parser (portable, TDD)

This task replaces the stub Parser members from Task 2 with the real state machine and tests the host→device commands SavvyCAN uses for live streaming.

**Files:**
- Modify: `lib/gvret/gvret.cpp` (replace the stub Parser members)
- Test: `test/test_gvret/test_gvret_parser.cpp`

- [ ] **Step 1: Write the failing tests with a mock Handler**

Create `test/test_gvret/test_gvret_parser.cpp`:

```cpp
#include <unity.h>
#include <vector>
#include <cstring>
#include "gvret.h"

namespace {

struct MockHandler : gvret::Handler {
    std::vector<CanFrame> txFrames;
    std::vector<uint8_t> out;
    uint8_t lastBus = 0xFF;
    uint32_t lastBitrate = 0;
    bool lastEnabled = false;
    bool lastListenOnly = false;
    int commandCount = 0;

    void onTransmitFrame(const CanFrame& f) override { txFrames.push_back(f); }
    void onSetBitrate(uint8_t bus, uint32_t br, bool en, bool lo) override {
        lastBus = bus; lastBitrate = br; lastEnabled = en; lastListenOnly = lo;
    }
    uint32_t currentBitrate(uint8_t) override { return 500000; }
    bool busEnabled(uint8_t) override { return true; }
    uint32_t nowMicros() override { return 0x11223344; }
    void writeBytes(const uint8_t* d, size_t n) override {
        out.insert(out.end(), d, d + n);
    }
    void onHostCommand() override { commandCount++; }
};

void feedAll(gvret::Parser& p, const std::vector<uint8_t>& bytes) {
    for (uint8_t b : bytes) p.feed(b);
}

}  // namespace

MockHandler* h = nullptr;
gvret::Parser* parser = nullptr;

void setUp(void) { h = new MockHandler(); parser = new gvret::Parser(*h); }
void tearDown(void) { delete parser; delete h; }

void test_build_can_frame_standard_is_transmitted(void) {
    // 0xF1,0x00, id(4 LE)=0x123, bus=0, len=2, data 0xDE 0xAD, checksum 0x00
    feedAll(*parser, {0xF1, 0x00,
                      0x23, 0x01, 0x00, 0x00,
                      0x00,
                      0x02,
                      0xDE, 0xAD,
                      0x00});
    TEST_ASSERT_EQUAL_size_t(1u, h->txFrames.size());
    TEST_ASSERT_EQUAL_HEX32(0x123u, h->txFrames[0].id);
    TEST_ASSERT_FALSE(h->txFrames[0].extended);
    TEST_ASSERT_EQUAL_UINT8(2, h->txFrames[0].dlc);
    TEST_ASSERT_EQUAL_HEX8(0xDE, h->txFrames[0].data[0]);
    TEST_ASSERT_EQUAL_HEX8(0xAD, h->txFrames[0].data[1]);
}

void test_build_can_frame_extended_flag(void) {
    // id with bit 31 set => extended; stored id masks it off
    feedAll(*parser, {0xF1, 0x00,
                      0x10, 0xF1, 0xDA, 0x98,  // 0x98DAF110 (bit31 set)
                      0x00,
                      0x00,
                      0x00});
    TEST_ASSERT_EQUAL_size_t(1u, h->txFrames.size());
    TEST_ASSERT_TRUE(h->txFrames[0].extended);
    TEST_ASSERT_EQUAL_HEX32(0x18DAF110u, h->txFrames[0].id);
}

void test_time_sync_reply(void) {
    feedAll(*parser, {0xF1, 0x01});
    // reply: 0xF1, 0x01, micros LE (0x11223344)
    TEST_ASSERT_EQUAL_size_t(6u, h->out.size());
    TEST_ASSERT_EQUAL_HEX8(0xF1, h->out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x01, h->out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x44, h->out[2]);
    TEST_ASSERT_EQUAL_HEX8(0x33, h->out[3]);
    TEST_ASSERT_EQUAL_HEX8(0x22, h->out[4]);
    TEST_ASSERT_EQUAL_HEX8(0x11, h->out[5]);
}

void test_keepalive_reply(void) {
    feedAll(*parser, {0xF1, 0x09});
    TEST_ASSERT_EQUAL_size_t(4u, h->out.size());
    TEST_ASSERT_EQUAL_HEX8(0xF1, h->out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x09, h->out[1]);
    TEST_ASSERT_EQUAL_HEX8(0xDE, h->out[2]);
    TEST_ASSERT_EQUAL_HEX8(0xAD, h->out[3]);
}

void test_get_device_info_reply(void) {
    feedAll(*parser, {0xF1, 0x07});
    // 0xF1,0x07, build LO, build HI, eepromVer, fileOut(0), autoLog(0), swMode(0)
    TEST_ASSERT_EQUAL_size_t(8u, h->out.size());
    TEST_ASSERT_EQUAL_HEX8(0xF1, h->out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x07, h->out[1]);
    TEST_ASSERT_EQUAL_HEX8((uint8_t)(gvret::kBuildNumber & 0xFF), h->out[2]);
    TEST_ASSERT_EQUAL_HEX8((uint8_t)(gvret::kBuildNumber >> 8), h->out[3]);
    TEST_ASSERT_EQUAL_HEX8(gvret::kEepromVersion, h->out[4]);
}

void test_get_num_buses_reply(void) {
    feedAll(*parser, {0xF1, 0x0C});
    TEST_ASSERT_EQUAL_size_t(3u, h->out.size());
    TEST_ASSERT_EQUAL_HEX8(0xF1, h->out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x0C, h->out[1]);
    TEST_ASSERT_EQUAL_HEX8(gvret::kNumBuses, h->out[2]);
}

void test_setup_canbus_sets_bitrate(void) {
    // can0 = 250000 (0x0003D090) enabled, not listen-only; can1 = 0 (disabled)
    feedAll(*parser, {0xF1, 0x05,
                      0x90, 0xD0, 0x03, 0x00,   // can0 = 250000 LE
                      0x00, 0x00, 0x00, 0x00}); // can1 = 0
    TEST_ASSERT_EQUAL_UINT8(0, h->lastBus);
    TEST_ASSERT_EQUAL_UINT32(250000u, h->lastBitrate);
    TEST_ASSERT_TRUE(h->lastEnabled);
    TEST_ASSERT_FALSE(h->lastListenOnly);
}

void test_setup_canbus_listen_only_bit(void) {
    // can0 speed with bit31 set => listen-only, masked speed = 500000
    // 500000 = 0x0007A120; with bit31 -> 0x8007A120
    feedAll(*parser, {0xF1, 0x05,
                      0x20, 0xA1, 0x07, 0x80,
                      0x00, 0x00, 0x00, 0x00});
    TEST_ASSERT_EQUAL_UINT32(500000u, h->lastBitrate);
    TEST_ASSERT_TRUE(h->lastListenOnly);
    TEST_ASSERT_TRUE(h->lastEnabled);
}

void test_unknown_command_resyncs_on_next_prefix(void) {
    // Unknown command 0x7F should not consume the following valid keepalive.
    feedAll(*parser, {0xF1, 0x7F});
    feedAll(*parser, {0xF1, 0x09});
    TEST_ASSERT_EQUAL_size_t(4u, h->out.size());  // only keepalive replied
}

void test_command_count_increments(void) {
    feedAll(*parser, {0xF1, 0x09});
    feedAll(*parser, {0xF1, 0x01});
    TEST_ASSERT_EQUAL_INT(2, h->commandCount);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_build_can_frame_standard_is_transmitted);
    RUN_TEST(test_build_can_frame_extended_flag);
    RUN_TEST(test_time_sync_reply);
    RUN_TEST(test_keepalive_reply);
    RUN_TEST(test_get_device_info_reply);
    RUN_TEST(test_get_num_buses_reply);
    RUN_TEST(test_setup_canbus_sets_bitrate);
    RUN_TEST(test_setup_canbus_listen_only_bit);
    RUN_TEST(test_unknown_command_resyncs_on_next_prefix);
    RUN_TEST(test_command_count_increments);
    return UNITY_END();
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pio test -e native -f test_gvret`
Expected: FAIL — the stub Parser members do nothing, so assertions fail.

- [ ] **Step 3: Replace the stub Parser members in `lib/gvret/gvret.cpp`**

Delete the `// --- Parser members are implemented in Task 3 ---` block and its stub bodies, and add this implementation in its place (keep `encodeFrame` as-is above it):

```cpp
namespace {
uint32_t le32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
}  // namespace

void Parser::reset() {
    state_ = State::Prefix;
    cmd_ = 0;
    got_ = 0;
    need_ = 0;
    bfStep_ = 0;
    bfDataIdx_ = 0;
    txFrame_ = CanFrame{};
}

void Parser::feed(uint8_t b) {
    switch (state_) {
        case State::Prefix:
            if (b == kCmdPrefix) state_ = State::Command;
            return;
        case State::Command:
            beginCommand(b);
            return;
        case State::Payload:
            if (cmd_ == kBuildCanFrame) {
                feedBuildFrameByte(b);
            } else {
                if (got_ < sizeof(buf_)) buf_[got_] = b;
                got_++;
                if (got_ >= need_) completeFixedCommand();
            }
            return;
    }
}

void Parser::beginCommand(uint8_t cmd) {
    cmd_ = cmd;
    got_ = 0;
    bfStep_ = 0;
    bfDataIdx_ = 0;
    txFrame_ = CanFrame{};

    switch (cmd) {
        case kBuildCanFrame:
            state_ = State::Payload;  // variable length, custom handling
            return;
        case kSetupCanbus:
            need_ = 8;  // can0 (4) + can1 (4)
            state_ = State::Payload;
            return;
        case kSetSingleWire:
        case kSetSystemType:
            need_ = 1;
            state_ = State::Payload;
            return;
        case kTimeSync:
            handler_.onHostCommand();
            replyTimeSync();
            state_ = State::Prefix;
            return;
        case kKeepAlive:
            handler_.onHostCommand();
            replyKeepAlive();
            state_ = State::Prefix;
            return;
        case kGetDeviceInfo:
            handler_.onHostCommand();
            replyDeviceInfo();
            state_ = State::Prefix;
            return;
        case kGetCanbusParams:
            handler_.onHostCommand();
            replyCanbusParams();
            state_ = State::Prefix;
            return;
        case kGetNumBuses:
            handler_.onHostCommand();
            replyNumBuses();
            state_ = State::Prefix;
            return;
        case kGetExtBuses:
            handler_.onHostCommand();
            replyExtBuses();
            state_ = State::Prefix;
            return;
        default:
            // Unknown / unsupported command: resync at the next prefix byte.
            state_ = State::Prefix;
            return;
    }
}

void Parser::feedBuildFrameByte(uint8_t b) {
    // Layout: id(4 LE) | bus(1) | len(1) | data[len] | checksum(1)
    switch (bfStep_) {
        case 0: case 1: case 2: case 3:
            buf_[bfStep_] = b;
            if (bfStep_ == 3) {
                uint32_t id = le32(buf_);
                txFrame_.extended = (id & 0x80000000u) != 0u;
                txFrame_.id = id & 0x7FFFFFFFu;
            }
            bfStep_++;
            return;
        case 4:
            txFrame_.bus = b & 0x03;
            bfStep_++;
            return;
        case 5:
            txFrame_.dlc = b & 0x0F;
            if (txFrame_.dlc > 8) txFrame_.dlc = 8;
            bfDataIdx_ = 0;
            bfStep_ = (txFrame_.dlc > 0) ? 6 : 7;
            return;
        case 6:
            if (bfDataIdx_ < 8) txFrame_.data[bfDataIdx_] = b;
            bfDataIdx_++;
            if (bfDataIdx_ >= txFrame_.dlc) bfStep_ = 7;
            return;
        case 7:  // checksum byte (ignored), frame complete
            handler_.onHostCommand();
            handler_.onTransmitFrame(txFrame_);
            state_ = State::Prefix;
            return;
        default:
            state_ = State::Prefix;
            return;
    }
}

void Parser::completeFixedCommand() {
    handler_.onHostCommand();
    if (cmd_ == kSetupCanbus) {
        uint32_t can0 = le32(&buf_[0]);
        bool listenOnly = (can0 & 0x80000000u) != 0u;
        uint32_t speed = can0 & 0x7FFFFFFFu;
        bool enabled = speed != 0u;
        handler_.onSetBitrate(0, speed, enabled, listenOnly);
    }
    // kSetSingleWire / kSetSystemType: accepted, no action needed.
    state_ = State::Prefix;
}

void Parser::replyTimeSync() {
    uint32_t now = handler_.nowMicros();
    uint8_t r[6] = {kCmdPrefix, kTimeSync,
                    (uint8_t)(now & 0xFF), (uint8_t)((now >> 8) & 0xFF),
                    (uint8_t)((now >> 16) & 0xFF), (uint8_t)((now >> 24) & 0xFF)};
    handler_.writeBytes(r, sizeof(r));
}

void Parser::replyKeepAlive() {
    uint8_t r[4] = {kCmdPrefix, kKeepAlive, 0xDE, 0xAD};
    handler_.writeBytes(r, sizeof(r));
}

void Parser::replyDeviceInfo() {
    uint8_t r[8] = {kCmdPrefix, kGetDeviceInfo,
                    (uint8_t)(kBuildNumber & 0xFF), (uint8_t)(kBuildNumber >> 8),
                    kEepromVersion,
                    0,   // fileOutputType
                    0,   // autoStartLogging
                    0};  // single-wire mode
    handler_.writeBytes(r, sizeof(r));
}

void Parser::replyCanbusParams() {
    uint32_t s0 = handler_.currentBitrate(0);
    uint8_t en0 = handler_.busEnabled(0) ? 1 : 0;
    uint8_t r[12] = {
        kCmdPrefix, kGetCanbusParams,
        en0,
        (uint8_t)(s0 & 0xFF), (uint8_t)((s0 >> 8) & 0xFF),
        (uint8_t)((s0 >> 16) & 0xFF), (uint8_t)((s0 >> 24) & 0xFF),
        0,            // can1 disabled
        0, 0, 0, 0};  // can1 speed = 0
    handler_.writeBytes(r, sizeof(r));
}

void Parser::replyNumBuses() {
    uint8_t r[3] = {kCmdPrefix, kGetNumBuses, kNumBuses};
    handler_.writeBytes(r, sizeof(r));
}

void Parser::replyExtBuses() {
    // No extended buses (SWCAN/LIN). Report all disabled (speed 0).
    uint8_t r[14] = {kCmdPrefix, kGetExtBuses,
                     0, 0, 0, 0,   // SWCAN speed
                     0, 0, 0, 0,   // LIN1 speed
                     0, 0, 0, 0};  // LIN2 speed
    handler_.writeBytes(r, sizeof(r));
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pio test -e native -f test_gvret`
Expected: PASS — all encode + parser tests (13 total), 0 failures.

- [ ] **Step 5: Commit**

```bash
git add lib/gvret/gvret.cpp test/test_gvret/test_gvret_parser.cpp
git commit -m "feat(gvret): implement GVRET command parser with tests"
```

---

## Task 4: NVS-backed config store (ESP32)

**Files:**
- Create: `src/config_store.h`
- Modify: `src/config_store.cpp` (replace stub)

- [ ] **Step 1: Create the header**

Create `src/config_store.h`:

```cpp
#pragma once
#include <cstdint>

namespace config_store {

// Initialize NVS. Safe to call once at boot. Returns true on success.
bool begin();

// Load the persisted CAN bitrate, or settings::kDefaultBitrate if none stored.
uint32_t loadBitrate();

// Persist the CAN bitrate. Returns true on success.
bool saveBitrate(uint32_t bitrate);

}  // namespace config_store
```

- [ ] **Step 2: Implement using NVS**

Replace the contents of `src/config_store.cpp`:

```cpp
#include "config_store.h"
#include "settings.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

namespace {
const char* kTag = "config_store";
const char* kNamespace = "canbridge";
const char* kKeyBitrate = "bitrate";
}  // namespace

namespace config_store {

bool begin() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "nvs_flash_init failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

uint32_t loadBitrate() {
    nvs_handle_t handle;
    if (nvs_open(kNamespace, NVS_READONLY, &handle) != ESP_OK) {
        return settings::kDefaultBitrate;
    }
    uint32_t value = settings::kDefaultBitrate;
    esp_err_t err = nvs_get_u32(handle, kKeyBitrate, &value);
    nvs_close(handle);
    if (err != ESP_OK) return settings::kDefaultBitrate;
    return settings::sanitizeBitrate(value);
}

bool saveBitrate(uint32_t bitrate) {
    bitrate = settings::sanitizeBitrate(bitrate);
    nvs_handle_t handle;
    if (nvs_open(kNamespace, NVS_READWRITE, &handle) != ESP_OK) return false;
    bool ok = (nvs_set_u32(handle, kKeyBitrate, bitrate) == ESP_OK) &&
              (nvs_commit(handle) == ESP_OK);
    nvs_close(handle);
    return ok;
}

}  // namespace config_store
```

- [ ] **Step 3: Verify it compiles**

Run: `pio run -e esp32dev`
Expected: build `SUCCESS`.

- [ ] **Step 4: Commit**

```bash
git add src/config_store.h src/config_store.cpp
git commit -m "feat(config): add NVS-backed bitrate persistence"
```

---

## Task 5: TWAI CAN driver wrapper (ESP32)

**Files:**
- Create: `src/can_driver.h`
- Modify: `src/can_driver.cpp` (replace stub)

- [ ] **Step 1: Create the header**

Create `src/can_driver.h`:

```cpp
#pragma once
#include <cstdint>
#include "can_frame.h"

namespace can_driver {

// Install + start the TWAI driver on the given pins at the given bitrate.
// Returns true on success.
bool start(uint32_t bitrate, int rxPin, int txPin, bool listenOnly);

// Stop and uninstall the driver.
void stop();

// Stop + reinstall at a new bitrate/mode. Returns true on success.
bool restart(uint32_t bitrate, bool listenOnly);

// Receive one frame. timeoutMs may be 0 for non-blocking.
// Returns true if a frame was received.
bool receive(CanFrame& out, uint32_t timeoutMs);

// Transmit one frame. Returns true if queued/sent.
bool transmit(const CanFrame& f, uint32_t timeoutMs);

// Poll TWAI alerts and auto-recover from bus-off.
// Sets `errorActive` true while the controller is in an error/bus-off state.
void serviceAlerts(bool& errorActive);

uint32_t bitrate();
bool enabled();
bool listenOnly();

}  // namespace can_driver
```

- [ ] **Step 2: Implement the TWAI wrapper**

Replace the contents of `src/can_driver.cpp`:

```cpp
#include "can_driver.h"
#include "driver/twai.h"
#include "esp_timer.h"
#include "esp_log.h"

namespace {
const char* kTag = "can_driver";
uint32_t g_bitrate = 500000;
bool g_enabled = false;
bool g_listenOnly = false;
bool g_recovering = false;

// Map a bitrate to a TWAI timing config. Falls back to 500k for unknown rates.
bool timingFor(uint32_t bitrate, twai_timing_config_t& out) {
    switch (bitrate) {
        case 1000000: out = TWAI_TIMING_CONFIG_1MBITS();   return true;
        case 800000:  out = TWAI_TIMING_CONFIG_800KBITS(); return true;
        case 500000:  out = TWAI_TIMING_CONFIG_500KBITS(); return true;
        case 250000:  out = TWAI_TIMING_CONFIG_250KBITS(); return true;
        case 125000:  out = TWAI_TIMING_CONFIG_125KBITS(); return true;
        case 100000:  out = TWAI_TIMING_CONFIG_100KBITS(); return true;
        case 50000:   out = TWAI_TIMING_CONFIG_50KBITS();  return true;
        case 25000:   out = TWAI_TIMING_CONFIG_25KBITS();  return true;
        default:      out = TWAI_TIMING_CONFIG_500KBITS(); return false;
    }
}
}  // namespace

namespace can_driver {

bool start(uint32_t bitrate, int rxPin, int txPin, bool listenOnly) {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)txPin, (gpio_num_t)rxPin,
        listenOnly ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL);
    g.tx_queue_len = 32;
    g.rx_queue_len = 64;
    g.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED |
                       TWAI_ALERT_ERR_PASS | TWAI_ALERT_RX_QUEUE_FULL;

    twai_timing_config_t t;
    timingFor(bitrate, t);
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g, &t, &f) != ESP_OK) {
        ESP_LOGE(kTag, "driver install failed");
        return false;
    }
    if (twai_start() != ESP_OK) {
        ESP_LOGE(kTag, "driver start failed");
        twai_driver_uninstall();
        return false;
    }
    g_bitrate = bitrate;
    g_enabled = true;
    g_listenOnly = listenOnly;
    g_recovering = false;
    ESP_LOGI(kTag, "TWAI started @ %u bps listenOnly=%d", bitrate, listenOnly);
    return true;
}

void stop() {
    twai_stop();
    twai_driver_uninstall();
    g_enabled = false;
}

bool restart(uint32_t bitrate, bool listenOnly) {
    if (g_enabled) stop();
    // rx/tx pins are fixed by main; restart reuses them via globals set there.
    extern int g_rxPin;
    extern int g_txPin;
    return start(bitrate, g_rxPin, g_txPin, listenOnly);
}

bool receive(CanFrame& out, uint32_t timeoutMs) {
    twai_message_t msg;
    if (twai_receive(&msg, pdMS_TO_TICKS(timeoutMs)) != ESP_OK) return false;
    out = CanFrame{};
    out.id = msg.identifier;
    out.extended = msg.extd != 0;
    out.rtr = msg.rtr != 0;
    out.dlc = msg.data_length_code > 8 ? 8 : msg.data_length_code;
    for (int i = 0; i < out.dlc; ++i) out.data[i] = msg.data[i];
    out.timestamp_us = (uint32_t)esp_timer_get_time();
    out.bus = 0;
    return true;
}

bool transmit(const CanFrame& f, uint32_t timeoutMs) {
    if (!g_enabled || g_listenOnly) return false;
    twai_message_t msg = {};
    msg.identifier = f.id;
    msg.extd = f.extended ? 1 : 0;
    msg.rtr = f.rtr ? 1 : 0;
    msg.data_length_code = f.dlc > 8 ? 8 : f.dlc;
    for (int i = 0; i < msg.data_length_code; ++i) msg.data[i] = f.data[i];
    return twai_transmit(&msg, pdMS_TO_TICKS(timeoutMs)) == ESP_OK;
}

void serviceAlerts(bool& errorActive) {
    uint32_t alerts = 0;
    // Non-blocking poll.
    if (twai_read_alerts(&alerts, 0) != ESP_OK) {
        errorActive = g_recovering;
        return;
    }
    if (alerts & TWAI_ALERT_BUS_OFF) {
        ESP_LOGW(kTag, "bus-off detected, initiating recovery");
        twai_initiate_recovery();
        g_recovering = true;
    }
    if (alerts & TWAI_ALERT_BUS_RECOVERED) {
        ESP_LOGI(kTag, "bus recovered, restarting");
        twai_start();
        g_recovering = false;
    }
    errorActive = g_recovering || (alerts & TWAI_ALERT_ERR_PASS);
}

uint32_t bitrate() { return g_bitrate; }
bool enabled() { return g_enabled; }
bool listenOnly() { return g_listenOnly; }

}  // namespace can_driver
```

- [ ] **Step 3: Verify it compiles**

Run: `pio run -e esp32dev`
Expected: build `SUCCESS`. (The `g_rxPin`/`g_txPin` externs are defined in `main.cpp` in Task 8; if building this task standalone, temporarily define them at the top of `can_driver.cpp` as `int g_rxPin = 4; int g_txPin = 5;` and remove when Task 8 adds them. Prefer to build-verify after Task 8.)

- [ ] **Step 4: Commit**

```bash
git add src/can_driver.h src/can_driver.cpp
git commit -m "feat(can): add TWAI driver wrapper with bus-off recovery"
```

---

## Task 6: USB serial transport (ESP32)

**Files:**
- Create: `src/usb_serial.h`
- Modify: `src/usb_serial.cpp` (replace stub)

- [ ] **Step 1: Create the header**

Create `src/usb_serial.h`:

```cpp
#pragma once
#include <cstddef>
#include <cstdint>

namespace usb_serial {

// Install the UART0 driver at the given baud. SavvyCAN/GVRET uses 1000000.
void begin(uint32_t baud);

// Read up to maxLen bytes without blocking. Returns bytes read (0 if none).
size_t read(uint8_t* buf, size_t maxLen);

// Write len bytes (blocks until queued).
void write(const uint8_t* buf, size_t len);

}  // namespace usb_serial
```

- [ ] **Step 2: Implement on UART0**

Replace the contents of `src/usb_serial.cpp`:

```cpp
#include "usb_serial.h"
#include "driver/uart.h"

namespace {
constexpr uart_port_t kPort = UART_NUM_0;
constexpr int kRxBuf = 2048;
constexpr int kTxBuf = 4096;
}  // namespace

namespace usb_serial {

void begin(uint32_t baud) {
    uart_config_t cfg = {};
    cfg.baud_rate = (int)baud;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    uart_driver_install(kPort, kRxBuf, kTxBuf, 0, nullptr, 0);
    uart_param_config(kPort, &cfg);
    // UART0 default pins (TX0/RX0) route to the USB-UART bridge; no pin remap.
}

size_t read(uint8_t* buf, size_t maxLen) {
    int n = uart_read_bytes(kPort, buf, maxLen, 0);
    return n < 0 ? 0 : (size_t)n;
}

void write(const uint8_t* buf, size_t len) {
    uart_write_bytes(kPort, (const char*)buf, len);
}

}  // namespace usb_serial
```

> Note: GVRET binary frames share UART0 with ESP-IDF's log output. In Task 8 we lower the runtime log level so logs don't corrupt the data stream. For production you may also set the bootloader log level to "No output" via `menuconfig`.

- [ ] **Step 3: Verify it compiles**

Run: `pio run -e esp32dev`
Expected: build `SUCCESS` (after Task 8 provides `main.cpp` externs; otherwise build-verify at Task 8).

- [ ] **Step 4: Commit**

```bash
git add src/usb_serial.h src/usb_serial.cpp
git commit -m "feat(usb): add UART0 transport at 1Mbaud"
```

---

## Task 7: Indicators — LEDs + reset button (ESP32)

**Files:**
- Create: `src/indicators.h`
- Modify: `src/indicators.cpp` (replace stub)

- [ ] **Step 1: Create the header**

Create `src/indicators.h`:

```cpp
#pragma once
#include <cstdint>

namespace indicators {

enum class Button { None, ShortPress, LongPress };

// Configure GPIO26 (CAN activity), GPIO27 (connected), GPIO25 (button input).
void begin(int canLedPin, int connLedPin, int buttonPin);

// Record CAN traffic; triggers a brief activity flicker.
void noteCanActivity();

// SavvyCAN client connection state (drives GPIO27).
void setConnected(bool connected);

// CAN error/bus-off state (fast-blinks the activity LED).
void setError(bool error);

// Drive LED outputs based on current state. Call frequently with a ms clock.
void update(uint32_t nowMs);

// Debounced button poll; returns a press event once per gesture.
Button pollButton(uint32_t nowMs);

}  // namespace indicators
```

- [ ] **Step 2: Implement LED + button logic**

Replace the contents of `src/indicators.cpp`:

```cpp
#include "indicators.h"
#include "driver/gpio.h"

namespace {
int g_canLed = 26, g_connLed = 27, g_button = 25;

// Activity LED
bool g_activityPending = false;
uint32_t g_activityUntilMs = 0;
bool g_error = false;
bool g_connected = false;

// Button debounce / press timing
constexpr uint32_t kDebounceMs = 30;
constexpr uint32_t kLongPressMs = 3000;
bool g_btnStableDown = false;
bool g_btnRaw = false;
uint32_t g_btnChangeMs = 0;
uint32_t g_btnDownMs = 0;
bool g_longFired = false;

void writeLed(int pin, bool on) { gpio_set_level((gpio_num_t)pin, on ? 1 : 0); }
}  // namespace

namespace indicators {

void begin(int canLedPin, int connLedPin, int buttonPin) {
    g_canLed = canLedPin; g_connLed = connLedPin; g_button = buttonPin;

    gpio_config_t out = {};
    out.mode = GPIO_MODE_OUTPUT;
    out.pin_bit_mask = (1ULL << g_canLed) | (1ULL << g_connLed);
    gpio_config(&out);

    gpio_config_t in = {};
    in.mode = GPIO_MODE_INPUT;
    in.pull_up_en = GPIO_PULLUP_ENABLE;   // button to GND, active-low
    in.pin_bit_mask = (1ULL << g_button);
    gpio_config(&in);

    writeLed(g_canLed, false);
    writeLed(g_connLed, false);
}

void noteCanActivity() { g_activityPending = true; }
void setConnected(bool connected) { g_connected = connected; }
void setError(bool error) { g_error = error; }

void update(uint32_t nowMs) {
    // Connection LED: solid when connected.
    writeLed(g_connLed, g_connected);

    // Activity LED: error => fast blink; else flicker ~40ms per frame burst.
    if (g_error) {
        writeLed(g_canLed, (nowMs / 100) % 2 == 0);
        return;
    }
    if (g_activityPending) {
        g_activityPending = false;
        g_activityUntilMs = nowMs + 40;
    }
    writeLed(g_canLed, nowMs < g_activityUntilMs);
}

Button pollButton(uint32_t nowMs) {
    bool raw = gpio_get_level((gpio_num_t)g_button) == 0;  // active-low
    if (raw != g_btnRaw) {
        g_btnRaw = raw;
        g_btnChangeMs = nowMs;
        return Button::None;
    }
    if ((nowMs - g_btnChangeMs) < kDebounceMs) return Button::None;

    // raw is now the debounced stable level
    if (raw && !g_btnStableDown) {       // press edge
        g_btnStableDown = true;
        g_btnDownMs = nowMs;
        g_longFired = false;
        return Button::None;
    }
    if (raw && g_btnStableDown && !g_longFired &&
        (nowMs - g_btnDownMs) >= kLongPressMs) {
        g_longFired = true;
        return Button::LongPress;        // fire while still held
    }
    if (!raw && g_btnStableDown) {       // release edge
        g_btnStableDown = false;
        if (!g_longFired) return Button::ShortPress;
    }
    return Button::None;
}

}  // namespace indicators
```

- [ ] **Step 3: Verify it compiles**

Run: `pio run -e esp32dev`
Expected: build `SUCCESS` (build-verify together with Task 8).

- [ ] **Step 4: Commit**

```bash
git add src/indicators.h src/indicators.cpp
git commit -m "feat(indicators): add activity/connection LEDs and reset button"
```

---

## Task 8: Wire everything in main + FreeRTOS tasks + watchdog (ESP32)

**Files:**
- Modify: `src/main.cpp` (replace the Task 0 stub)

- [ ] **Step 1: Replace `src/main.cpp` with the full application**

```cpp
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_log.h"

#include "can_frame.h"
#include "gvret.h"
#include "settings.h"
#include "config_store.h"
#include "can_driver.h"
#include "usb_serial.h"
#include "indicators.h"

// CAN RX/TX pins (to the transceiver). "D4"/"D5" on the DevKit.
int g_rxPin = 4;
int g_txPin = 5;

namespace {
const char* kTag = "main";

// Indicator pins.
constexpr int kCanLedPin = 26;
constexpr int kConnLedPin = 27;
constexpr int kButtonPin = 25;

constexpr uint32_t kUsbBaud = 1000000;
constexpr uint32_t kConnTimeoutMs = 4000;  // no host commands => disconnected
constexpr int kRxQueueDepth = 256;

QueueHandle_t g_rxQueue = nullptr;        // CanFrame from CAN task -> comms task
volatile uint32_t g_lastCmdMs = 0;        // last host command time
volatile uint32_t g_dropped = 0;          // overflow counter

uint32_t millis32() { return (uint32_t)(esp_timer_get_time() / 1000); }

// Handler the GVRET parser calls. Runs in the comms task context.
class BridgeHandler : public gvret::Handler {
public:
    void onTransmitFrame(const CanFrame& f) override {
        can_driver::transmit(f, 10);
        indicators::noteCanActivity();
    }
    void onSetBitrate(uint8_t bus, uint32_t bitrate, bool enabled,
                      bool listenOnly) override {
        if (bus != 0) return;
        uint32_t br = settings::sanitizeBitrate(bitrate);
        if (!enabled) { can_driver::stop(); return; }
        can_driver::restart(br, listenOnly);
        config_store::saveBitrate(br);
    }
    uint32_t currentBitrate(uint8_t) override { return can_driver::bitrate(); }
    bool busEnabled(uint8_t) override { return can_driver::enabled(); }
    uint32_t nowMicros() override { return (uint32_t)esp_timer_get_time(); }
    void writeBytes(const uint8_t* data, size_t len) override {
        usb_serial::write(data, len);
    }
    void onHostCommand() override { g_lastCmdMs = millis32(); }
};

// CAN task: block on receive, push frames to the queue (drop-oldest on full).
void canTask(void*) {
    CanFrame f;
    for (;;) {
        if (can_driver::receive(f, 50)) {
            if (xQueueSend(g_rxQueue, &f, 0) != pdTRUE) {
                CanFrame discard;
                xQueueReceive(g_rxQueue, &discard, 0);  // drop oldest
                xQueueSend(g_rxQueue, &f, 0);
                g_dropped++;
            }
        }
    }
}

// Comms task: drain queue -> encode -> USB; read USB -> parse; manage state.
void commsTask(void*) {
    BridgeHandler handler;
    gvret::Parser parser(handler);

    esp_task_wdt_add(nullptr);

    uint8_t rxBytes[512];
    uint8_t encoded[gvret::kMaxEncodedFrame];
    CanFrame frame;
    bool wasConnected = false;

    for (;;) {
        esp_task_wdt_reset();

        // 1) Outbound: drain CAN frames to the host.
        int drained = 0;
        while (drained < 64 && xQueueReceive(g_rxQueue, &frame, 0) == pdTRUE) {
            size_t n = gvret::encodeFrame(frame, encoded, sizeof(encoded));
            if (n) usb_serial::write(encoded, n);
            indicators::noteCanActivity();
            drained++;
        }

        // 2) Inbound: feed host bytes to the parser.
        size_t got = usb_serial::read(rxBytes, sizeof(rxBytes));
        for (size_t i = 0; i < got; ++i) parser.feed(rxBytes[i]);

        // 3) Connection state.
        bool connected =
            (millis32() - g_lastCmdMs) < kConnTimeoutMs && g_lastCmdMs != 0;
        if (connected != wasConnected) {
            indicators::setConnected(connected);
            wasConnected = connected;
        }

        // 4) CAN error/recovery state.
        bool errorActive = false;
        can_driver::serviceAlerts(errorActive);
        indicators::setError(errorActive);

        if (drained == 0 && got == 0) vTaskDelay(pdMS_TO_TICKS(2));
    }
}

}  // namespace

extern "C" void app_main(void) {
    esp_log_level_set("*", ESP_LOG_NONE);  // keep UART0 clean for GVRET data

    config_store::begin();
    uint32_t bitrate = config_store::loadBitrate();

    usb_serial::begin(kUsbBaud);
    indicators::begin(kCanLedPin, kConnLedPin, kButtonPin);
    can_driver::start(bitrate, g_rxPin, g_txPin, /*listenOnly=*/false);

    g_rxQueue = xQueueCreate(kRxQueueDepth, sizeof(CanFrame));

    xTaskCreatePinnedToCore(canTask, "can", 4096, nullptr, 10, nullptr, 1);
    xTaskCreatePinnedToCore(commsTask, "comms", 8192, nullptr, 9, nullptr, 0);

    // Main loop: indicator updates + button handling.
    for (;;) {
        uint32_t now = millis32();
        indicators::update(now);
        switch (indicators::pollButton(now)) {
            case indicators::Button::ShortPress:
                ESP_LOGI(kTag, "short press: reset connectivity");
                can_driver::restart(can_driver::bitrate(),
                                    can_driver::listenOnly());
                g_lastCmdMs = 0;  // force disconnected until host re-handshakes
                break;
            case indicators::Button::LongPress:
                ESP_LOGI(kTag, "long press: reboot");
                esp_restart();
                break;
            default:
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

- [ ] **Step 2: Configure the task watchdog timeout in `platformio.ini`**

Add to `[env:esp32dev]` `build_flags` (so the 5s WDT comfortably exceeds the comms loop period):

```ini
    -DCONFIG_ESP_TASK_WDT_TIMEOUT_S=5
```

So the section reads:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = espidf
monitor_speed = 115200
build_flags =
    -std=gnu++17
    -DCONFIG_ESP_TASK_WDT_TIMEOUT_S=5
```

- [ ] **Step 3: Build the complete firmware**

Run: `pio run -e esp32dev`
Expected: build `SUCCESS`, linking all modules.

- [ ] **Step 4: Confirm the native tests still pass (no regressions)**

Run: `pio test -e native`
Expected: all settings + gvret tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/main.cpp platformio.ini
git commit -m "feat: wire CAN<->GVRET bridge with tasks, recovery, and watchdog"
```

---

## Task 9: On-target verification + SavvyCAN smoke test + docs

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Flash to hardware**

Run: `pio run -e esp32dev -t upload`
Expected: upload completes; device resets.

> Requires an ESP32-WROOM-32 DevKit connected via USB and a CAN transceiver (e.g. SN65HVD230) wired: ESP32 GPIO4→transceiver RXD, GPIO5→transceiver TXD, transceiver CANH/CANL to the bus, common ground. Activity LED on GPIO26, connection LED on GPIO27 (each via a ~330Ω resistor to GND), button on GPIO25 to GND.

- [ ] **Step 2: TWAI loopback self-test (no live bus needed)**

Temporarily change the `can_driver::start` mode in `main.cpp` to `TWAI_MODE_NO_ACK` (self-reception) or add a one-off test that transmits a frame and confirms `canTask` receives it. Confirm via SavvyCAN (next step) that a transmitted frame echoes back. Revert to `TWAI_MODE_NORMAL` after.

> Document the result of this check (pass/fail + what you observed) in the commit message or PR. Do not claim it passed without observing the echoed frame.

- [ ] **Step 3: Connect SavvyCAN**

1. Open SavvyCAN → Connection → New Device Connection → Serial → select the ESP32's COM port → speed 1000000.
2. Confirm SavvyCAN recognizes the device (device info populates) and the connection LED (GPIO27) goes solid.
3. With a live bus or a second CAN node, confirm frames stream into SavvyCAN and the activity LED (GPIO26) flickers.
4. In SavvyCAN, change the bus speed (e.g. to 250 kbps) and confirm frames still flow at the new rate; reboot the device and confirm it comes back at 250 kbps (NVS persistence).
5. Send a frame from SavvyCAN and confirm it appears on the bus (or echoes in loopback).
6. Short-press GPIO25 → confirm connection resets (LED behavior) and re-handshakes. Long-press (≥3s) → confirm device reboots.

> Record the observed result of each numbered check. These are manual, hardware-dependent verifications — report exactly what happened, including any check that failed.

- [ ] **Step 4: Update the README**

Replace `README.md` with usage docs:

```markdown
# ESP32CANBUSGVRet

ESP-IDF firmware that bridges a CAN bus to SavvyCAN over USB using the GVRET
protocol. Reads frames via the ESP32's built-in TWAI controller and streams them
to SavvyCAN; SavvyCAN can also transmit frames onto the bus.

## Hardware

- ESP32-WROOM-32 DevKit.
- CAN transceiver (e.g. SN65HVD230 / TJA1050).

| Function | GPIO | Notes |
|---|---|---|
| CAN RX | 4 (D4) | -> transceiver RXD |
| CAN TX | 5 (D5) | -> transceiver TXD |
| CAN activity LED | 26 | flickers on traffic; fast-blinks on bus error |
| Connection LED | 27 | solid when SavvyCAN is connected |
| Reset button | 25 | short press = reset connectivity; long press (>=3s) = reboot |

## Build & flash

```
pio run -e esp32dev -t upload
```

## Run unit tests (host)

```
pio test -e native
```

## Connect SavvyCAN

Connection -> New Device Connection -> Serial -> select the ESP32 COM port ->
speed 1000000. Default CAN bitrate is 500 kbps (configurable from SavvyCAN;
the last-used rate persists across reboots).

## Build system

PlatformIO + ESP-IDF. Portable protocol/settings code lives in `lib/` and is
unit-tested on the host via the `native` environment; ESP32 hardware glue lives
in `src/`.
```

- [ ] **Step 5: Commit and push**

```bash
git add README.md src/main.cpp
git commit -m "docs: usage + wiring; finalize verification"
git push -u origin main
```

---

## Self-Review

**Spec coverage check (against `2026-06-04-esp32-canbus-savvycan-bridge-design.md`):**
- PlatformIO + ESP-IDF + C++ → Task 0. ✅
- Module breakdown (can_driver, gvret, usb_serial, indicators, config_store, main) → Tasks 4–8. ✅
- Pin map (RX=4, TX=5, LED=26/27, button=25) → Task 7 + Task 8 constants. ✅
- GVRET protocol + handshake + bidirectional frames → Tasks 2, 3. ✅
- Default 500 kbps, configurable via SET_CANBUS_PARAMS, persisted to NVS → Tasks 1, 3, 4, 8. ✅
- Two FreeRTOS tasks decoupled by queue, 1 Mbaud USB → Task 8, Task 6. ✅
- Robustness: bus-off recovery (Task 5 `serviceAlerts`), backpressure drop-oldest + counter (Task 8 `canTask`), reconnect/connection timeout (Task 8 `commsTask`), atomic bitrate change (Task 5 `restart` + Task 8 handler), watchdog (Task 8). ✅
- Reset button short/long behavior → Task 7 + Task 8 main loop. ✅
- Testing: native unit tests for gvret + settings (Tasks 1–3), on-target loopback + SavvyCAN manual checklist (Task 9). ✅
- Out-of-scope items (Wi-Fi/BT, SD logging, multi-bus) → not implemented. ✅

**Placeholder scan:** No "TBD"/"implement later". The only deferred-implementation pattern is the intentional Parser stubs in Task 2 that Task 3 replaces (each shown as real code, with the replacement fully specified). ✅

**Type consistency:** `CanFrame` fields used identically across gvret, can_driver, main. `gvret::Handler` virtuals (`onTransmitFrame`, `onSetBitrate`, `currentBitrate`, `busEnabled`, `nowMicros`, `writeBytes`, `onHostCommand`) match `BridgeHandler` overrides in Task 8. `can_driver` API (`start`, `stop`, `restart`, `receive`, `transmit`, `serviceAlerts`, `bitrate`, `enabled`, `listenOnly`) used consistently in main. `g_rxPin`/`g_txPin` externs declared in `can_driver.cpp` and defined in `main.cpp`. ✅

**Known verification risks to confirm in Task 9:** exact GVRET byte layouts for GET_DEVICE_INFO / GET_CANBUS_PARAMS / SETUP_CANBUS / GET_EXT_BUSES are mirrored from ESP32RET and must be confirmed against a live SavvyCAN handshake; the `SETUP_CANBUS` listen-only/enable bit interpretation is the most likely thing to need adjustment.
