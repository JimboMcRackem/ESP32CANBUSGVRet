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
