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
