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

The ESP32 provides only the CAN controller; the transceiver converts the
controller's TX/RX logic levels to the differential CANH/CANL bus. Wire a common
ground, and put each LED in series with a ~330R resistor to GND. The button
connects GPIO25 to GND (an internal pull-up is enabled, so it is active-low).

## Build & flash

```
platformio run -e esp32dev -t upload
```

(Use `platformio`, not `pio`, if the `pio` shim is blocked on your system.)

## Run unit tests (host)

The portable protocol/settings logic is unit-tested on the host with a native
C++ compiler (no hardware needed):

```
platformio test -e native
```

## Connect SavvyCAN

Connection -> New Device Connection -> Serial -> select the ESP32 COM port ->
speed 1000000. Default CAN bitrate is 500 kbps. SavvyCAN can change the bitrate
at runtime, and the last-used rate is persisted to NVS so it survives reboots.

## Architecture

PlatformIO + ESP-IDF, C++17.

- `lib/gvret/` — GVRET protocol: `CanFrame`, frame encoder, command parser
  (hardware-independent; unit-tested in the `native` env).
- `lib/settings/` — bitrate validation (hardware-independent; unit-tested).
- `src/can_driver.*` — TWAI driver wrapper with bus-off recovery.
- `src/usb_serial.*` — UART0 transport at 1 Mbaud.
- `src/config_store.*` — NVS persistence of the bitrate.
- `src/indicators.*` — activity/connection LEDs and the reset button.
- `src/main.cpp` — two FreeRTOS tasks (CAN receive + host comms) decoupled by a
  queue, the GVRET handler, watchdog, and button handling.

Design and implementation notes live in `docs/superpowers/`.
```
