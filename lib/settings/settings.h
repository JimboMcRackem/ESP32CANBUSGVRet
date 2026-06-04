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
