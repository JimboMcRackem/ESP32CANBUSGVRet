# ESP32 CAN-to-SavvyCAN Bridge (GVRET) — Design

**Date:** 2026-06-04
**Status:** Approved (pending implementation plan)

## Goal

Firmware for an ESP32-WROOM-32 that reads frames off a CAN bus using the
ESP32's built-in TWAI ("Two-Wire Automotive Interface") controller and streams
them over USB serial to [SavvyCAN](https://www.savvycan.com/) using the **GVRET**
protocol, so the device is recognized natively by SavvyCAN's serial connection.
The bridge is bidirectional (SavvyCAN can also transmit frames onto the bus).

Data connections should be robust: the firmware recovers automatically from CAN
bus-off / error states and from USB/SavvyCAN disconnects, and never lets bus
timing block on slow USB.

## Toolchain & Architecture

- **IDE:** VS Code with the **PlatformIO** extension.
- **Framework:** **ESP-IDF** (native Espressif SDK), target `esp32`
  (ESP32-WROOM-32 DevKit).
- **Language:** C++.
- CAN access via the native ESP-IDF TWAI driver (`driver/twai.h`).

### Modules

Each module has one clear purpose, a well-defined interface, and can be
understood/tested independently.

| Module | Responsibility | Depends on |
|---|---|---|
| `can_driver` | Wrap TWAI: init, RX/TX queues, error/bus-off monitoring, reconfigure bitrate | ESP-IDF `driver/twai.h` |
| `gvret` | GVRET serial protocol state machine: parse SavvyCAN commands, encode/decode CAN frame packets | (pure logic — host-testable) |
| `usb_serial` | USB CDC/UART transport: buffered read/write to host | ESP-IDF UART/USB |
| `indicators` | LED control (activity flicker, connection state) + GPIO25 button (debounce, short/long press) | ESP-IDF GPIO |
| `config_store` | NVS-backed persistence of bitrate / settings | ESP-IDF NVS |
| `main` | Wire modules together; FreeRTOS tasks + main event loop | all of the above |

## Pin Map

| Function | GPIO | Notes |
|---|---|---|
| CAN RX | **GPIO 4** ("D4") | → CAN transceiver RXD |
| CAN TX | **GPIO 5** ("D5") | → CAN transceiver TXD |
| CAN activity LED | **GPIO 26** | output; flicker on frame RX/TX; fast blink on error state |
| USB / SavvyCAN connected LED | **GPIO 27** | output; solid when SavvyCAN client connected, off/slow-blink when enumerated but idle |
| Reset button | **GPIO 25** | input with pull-up, button to GND; short press = reset connectivity, long press (≥3 s) = full reboot |

An external CAN transceiver (e.g. SN65HVD230 / TJA1050) sits between GPIO4/GPIO5
and the physical CAN bus. The ESP32 provides only the CAN controller.

## Data Flow

```
CAN bus ──▶ transceiver ──▶ TWAI RX ──▶ can_driver RX queue ──▶ gvret encode ──▶ usb_serial ──▶ SavvyCAN
SavvyCAN ──▶ usb_serial ──▶ gvret decode ──▶ can_driver TX ──▶ transceiver ──▶ CAN bus
```

- **CAN task:** blocks on `twai_receive`, pushes frames to a thread-safe queue.
- **Comms task:** drains the queue → GVRET-encode → USB; also reads USB →
  GVRET-decode → TWAI TX.
- Queues decouple CAN bus timing from USB timing.
- USB serial baud: **1,000,000** (GVRET / ESP32RET convention).

## GVRET Protocol Scope

Implement the core GVRET command set SavvyCAN uses on connect:

- Device / firmware ID handshake.
- Time-sync.
- Build-time params query.
- **GET / SET_CANBUS_PARAMS** — enable bus + set bitrate (lets SavvyCAN change
  bitrate live).
- **Binary CAN frame** packets in both directions, standard (11-bit) and
  extended (29-bit) IDs.

This is the minimum that makes SavvyCAN auto-recognize the device and stream
live frames bidirectionally.

## Bitrate Handling

- Default **500 kbps** at boot.
- Configurable: honor `SET_CANBUS_PARAMS` from SavvyCAN at runtime.
- Persist last-used bitrate to **NVS** so it survives reboots.

## Robustness

- **TWAI bus-off / error recovery:** monitor alerts (`TWAI_ALERT_BUS_OFF`,
  error-passive, RX-queue-full); on bus-off, auto-initiate recovery and re-enter
  the running state. Activity LED indicates error state (fast blink).
- **Backpressure:** bounded RX queue; if USB can't keep up, drop oldest frame
  with a counter rather than blocking the CAN task. Optionally report overflow.
- **Reconnect:** detect SavvyCAN disconnect (handshake timeout / no client),
  drop to "enumerated but idle" (GPIO27 reflects this), re-handshake cleanly when
  SavvyCAN returns.
- **Atomic bitrate change:** stop → reconfigure → restart TWAI safely, then
  persist to NVS.
- **Watchdog:** task watchdog on the main loop so a wedged state self-recovers
  via reboot.

### Reset button behavior (GPIO 25)

- **Short press:** reset *connectivity* — re-init TWAI driver, recover from
  bus-off, reset the serial/GVRET session.
- **Long press (≥3 s):** full device reboot (`esp_restart()`).

## Testing Strategy

- **Host-side unit tests** (PlatformIO `native` env) for pure logic: GVRET
  encode/decode and the config store. No hardware required; fast feedback.
- **On-target smoke tests:** TWAI loopback / self-test mode to validate frame
  round-trip without a live bus.
- **Manual verification:** connect SavvyCAN over USB, confirm device recognition,
  live frame streaming, bitrate change, bidirectional TX, and LED/button
  behavior against a real bus or transceiver loopback.

## Out of Scope (YAGNI)

- Wi-Fi / Bluetooth streaming (USB serial only for now).
- SD-card logging.
- Multiple CAN channels (single TWAI controller).
- GVRET commands beyond what SavvyCAN needs for live serial streaming.
