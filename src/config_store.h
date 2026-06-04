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
