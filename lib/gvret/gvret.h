#pragma once
#include <cstddef>
#include <cstdint>
#include "can_frame.h"

namespace gvret {

// Protocol bytes (mirror ESP32RET / GVRET).
constexpr uint8_t kCmdPrefix       = 0xF1;
constexpr uint8_t kBuildCanFrame   = 0;
constexpr uint8_t kTimeSync        = 1;
constexpr uint8_t kGetDigInputs    = 2;
constexpr uint8_t kGetAnaInputs    = 3;
constexpr uint8_t kSetDigOutputs   = 4;
constexpr uint8_t kSetupCanbus     = 5;
constexpr uint8_t kGetCanbusParams = 6;
constexpr uint8_t kGetDeviceInfo   = 7;
constexpr uint8_t kSetSingleWire   = 8;
constexpr uint8_t kKeepAlive       = 9;
constexpr uint8_t kSetSystemType   = 10;
constexpr uint8_t kEchoCanFrame    = 11;
constexpr uint8_t kGetNumBuses     = 12;
constexpr uint8_t kGetExtBuses     = 13;
constexpr uint8_t kSetExtBuses     = 14;

// Reported to SavvyCAN in GET_DEVICE_INFO.
constexpr uint16_t kBuildNumber   = 0x0001;
constexpr uint8_t  kEepromVersion = 0x14;
constexpr uint8_t  kNumBuses      = 1;

// 2 header + 4 timestamp + 4 id + 1 dlc/bus + 8 data + 1 checksum.
constexpr size_t kMaxEncodedFrame = 20;

// Encode a received CAN frame into GVRET binary for SavvyCAN.
// Returns bytes written, or 0 if outCap is too small. DLC is clamped to 8.
size_t encodeFrame(const CanFrame& f, uint8_t* out, size_t outCap);

// Actions the parser performs in response to host commands.
struct Handler {
    virtual ~Handler() = default;
    virtual void onTransmitFrame(const CanFrame& f) = 0;
    virtual void onSetBitrate(uint8_t bus, uint32_t bitrate,
                              bool enabled, bool listenOnly) = 0;
    virtual uint32_t currentBitrate(uint8_t bus) = 0;
    virtual bool busEnabled(uint8_t bus) = 0;
    virtual uint32_t nowMicros() = 0;
    virtual void writeBytes(const uint8_t* data, size_t len) = 0;
    // Called whenever any valid command is received (for connection tracking).
    virtual void onHostCommand() {}
};

// Byte-at-a-time parser for the host -> device GVRET command stream.
class Parser {
public:
    explicit Parser(Handler& handler) : handler_(handler) {}
    void feed(uint8_t b);
    void reset();

private:
    enum class State : uint8_t { Prefix, Command, Payload };
    Handler& handler_;
    State state_ = State::Prefix;
    uint8_t cmd_ = 0;
    uint8_t buf_[16] = {0};
    uint8_t got_ = 0;
    uint8_t need_ = 0;   // fixed payload length; 0xFF => variable (build frame)

    void beginCommand(uint8_t cmd);
    void completeFixedCommand();
    void feedBuildFrameByte(uint8_t b);
    // Build-can-frame assembly state:
    CanFrame txFrame_{};
    uint8_t bfStep_ = 0;
    uint8_t bfDataIdx_ = 0;

    void replyDeviceInfo();
    void replyCanbusParams();
    void replyTimeSync();
    void replyKeepAlive();
    void replyNumBuses();
    void replyExtBuses();
};

}  // namespace gvret
