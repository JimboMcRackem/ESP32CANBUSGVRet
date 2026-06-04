#include "settings.h"

namespace settings {

uint32_t sanitizeBitrate(uint32_t requested) {
    if (requested < kMinBitrate || requested > kMaxBitrate) {
        return kDefaultBitrate;
    }
    return requested;
}

}  // namespace settings
