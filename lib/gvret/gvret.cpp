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

}  // namespace gvret
