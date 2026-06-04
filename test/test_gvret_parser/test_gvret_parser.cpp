#include <unity.h>
#include <vector>
#include <cstring>
#include "gvret.h"

namespace {

struct MockHandler : gvret::Handler {
    std::vector<CanFrame> txFrames;
    std::vector<uint8_t> out;
    uint8_t lastBus = 0xFF;
    uint32_t lastBitrate = 0;
    bool lastEnabled = false;
    bool lastListenOnly = false;
    int commandCount = 0;

    void onTransmitFrame(const CanFrame& f) override { txFrames.push_back(f); }
    void onSetBitrate(uint8_t bus, uint32_t br, bool en, bool lo) override {
        lastBus = bus; lastBitrate = br; lastEnabled = en; lastListenOnly = lo;
    }
    uint32_t currentBitrate(uint8_t) override { return 500000; }
    bool busEnabled(uint8_t) override { return true; }
    uint32_t nowMicros() override { return 0x11223344; }
    void writeBytes(const uint8_t* d, size_t n) override {
        out.insert(out.end(), d, d + n);
    }
    void onHostCommand() override { commandCount++; }
};

void feedAll(gvret::Parser& p, const std::vector<uint8_t>& bytes) {
    for (uint8_t b : bytes) p.feed(b);
}

}  // namespace

MockHandler* h = nullptr;
gvret::Parser* parser = nullptr;

void setUp(void) { h = new MockHandler(); parser = new gvret::Parser(*h); }
void tearDown(void) { delete parser; delete h; }

void test_build_can_frame_standard_is_transmitted(void) {
    // 0xF1,0x00, id(4 LE)=0x123, bus=0, len=2, data 0xDE 0xAD, checksum 0x00
    feedAll(*parser, {0xF1, 0x00,
                      0x23, 0x01, 0x00, 0x00,
                      0x00,
                      0x02,
                      0xDE, 0xAD,
                      0x00});
    TEST_ASSERT_EQUAL_size_t(1u, h->txFrames.size());
    TEST_ASSERT_EQUAL_HEX32(0x123u, h->txFrames[0].id);
    TEST_ASSERT_FALSE(h->txFrames[0].extended);
    TEST_ASSERT_EQUAL_UINT8(2, h->txFrames[0].dlc);
    TEST_ASSERT_EQUAL_HEX8(0xDE, h->txFrames[0].data[0]);
    TEST_ASSERT_EQUAL_HEX8(0xAD, h->txFrames[0].data[1]);
}

void test_build_can_frame_extended_flag(void) {
    // id with bit 31 set => extended; stored id masks it off
    feedAll(*parser, {0xF1, 0x00,
                      0x10, 0xF1, 0xDA, 0x98,  // 0x98DAF110 (bit31 set)
                      0x00,
                      0x00,
                      0x00});
    TEST_ASSERT_EQUAL_size_t(1u, h->txFrames.size());
    TEST_ASSERT_TRUE(h->txFrames[0].extended);
    TEST_ASSERT_EQUAL_HEX32(0x18DAF110u, h->txFrames[0].id);
}

void test_time_sync_reply(void) {
    feedAll(*parser, {0xF1, 0x01});
    // reply: 0xF1, 0x01, micros LE (0x11223344)
    TEST_ASSERT_EQUAL_size_t(6u, h->out.size());
    TEST_ASSERT_EQUAL_HEX8(0xF1, h->out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x01, h->out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x44, h->out[2]);
    TEST_ASSERT_EQUAL_HEX8(0x33, h->out[3]);
    TEST_ASSERT_EQUAL_HEX8(0x22, h->out[4]);
    TEST_ASSERT_EQUAL_HEX8(0x11, h->out[5]);
}

void test_keepalive_reply(void) {
    feedAll(*parser, {0xF1, 0x09});
    TEST_ASSERT_EQUAL_size_t(4u, h->out.size());
    TEST_ASSERT_EQUAL_HEX8(0xF1, h->out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x09, h->out[1]);
    TEST_ASSERT_EQUAL_HEX8(0xDE, h->out[2]);
    TEST_ASSERT_EQUAL_HEX8(0xAD, h->out[3]);
}

void test_get_device_info_reply(void) {
    feedAll(*parser, {0xF1, 0x07});
    // 0xF1,0x07, build LO, build HI, eepromVer, fileOut(0), autoLog(0), swMode(0)
    TEST_ASSERT_EQUAL_size_t(8u, h->out.size());
    TEST_ASSERT_EQUAL_HEX8(0xF1, h->out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x07, h->out[1]);
    TEST_ASSERT_EQUAL_HEX8((uint8_t)(gvret::kBuildNumber & 0xFF), h->out[2]);
    TEST_ASSERT_EQUAL_HEX8((uint8_t)(gvret::kBuildNumber >> 8), h->out[3]);
    TEST_ASSERT_EQUAL_HEX8(gvret::kEepromVersion, h->out[4]);
}

void test_get_num_buses_reply(void) {
    feedAll(*parser, {0xF1, 0x0C});
    TEST_ASSERT_EQUAL_size_t(3u, h->out.size());
    TEST_ASSERT_EQUAL_HEX8(0xF1, h->out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x0C, h->out[1]);
    TEST_ASSERT_EQUAL_HEX8(gvret::kNumBuses, h->out[2]);
}

void test_setup_canbus_sets_bitrate(void) {
    // can0 = 250000 (0x0003D090) enabled, not listen-only; can1 = 0 (disabled)
    feedAll(*parser, {0xF1, 0x05,
                      0x90, 0xD0, 0x03, 0x00,   // can0 = 250000 LE
                      0x00, 0x00, 0x00, 0x00}); // can1 = 0
    TEST_ASSERT_EQUAL_UINT8(0, h->lastBus);
    TEST_ASSERT_EQUAL_UINT32(250000u, h->lastBitrate);
    TEST_ASSERT_TRUE(h->lastEnabled);
    TEST_ASSERT_FALSE(h->lastListenOnly);
}

void test_setup_canbus_listen_only_bit(void) {
    // can0 speed with bit31 set => listen-only, masked speed = 500000
    // 500000 = 0x0007A120; with bit31 -> 0x8007A120
    feedAll(*parser, {0xF1, 0x05,
                      0x20, 0xA1, 0x07, 0x80,
                      0x00, 0x00, 0x00, 0x00});
    TEST_ASSERT_EQUAL_UINT32(500000u, h->lastBitrate);
    TEST_ASSERT_TRUE(h->lastListenOnly);
    TEST_ASSERT_TRUE(h->lastEnabled);
}

void test_unknown_command_resyncs_on_next_prefix(void) {
    // Unknown command 0x7F should not consume the following valid keepalive.
    feedAll(*parser, {0xF1, 0x7F});
    feedAll(*parser, {0xF1, 0x09});
    TEST_ASSERT_EQUAL_size_t(4u, h->out.size());  // only keepalive replied
}

void test_command_count_increments(void) {
    feedAll(*parser, {0xF1, 0x09});
    feedAll(*parser, {0xF1, 0x01});
    TEST_ASSERT_EQUAL_INT(2, h->commandCount);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_build_can_frame_standard_is_transmitted);
    RUN_TEST(test_build_can_frame_extended_flag);
    RUN_TEST(test_time_sync_reply);
    RUN_TEST(test_keepalive_reply);
    RUN_TEST(test_get_device_info_reply);
    RUN_TEST(test_get_num_buses_reply);
    RUN_TEST(test_setup_canbus_sets_bitrate);
    RUN_TEST(test_setup_canbus_listen_only_bit);
    RUN_TEST(test_unknown_command_resyncs_on_next_prefix);
    RUN_TEST(test_command_count_increments);
    return UNITY_END();
}
