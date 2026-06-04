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
