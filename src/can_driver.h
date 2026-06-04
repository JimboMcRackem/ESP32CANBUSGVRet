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
