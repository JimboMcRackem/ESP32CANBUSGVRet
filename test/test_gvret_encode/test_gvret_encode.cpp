#include <unity.h>
#include <cstring>
#include "gvret.h"

void setUp(void) {}
void tearDown(void) {}

void test_encode_standard_frame_layout(void) {
    CanFrame f;
    f.id = 0x123;
    f.extended = false;
    f.dlc = 2;
    f.data[0] = 0xAA;
    f.data[1] = 0xBB;
    f.timestamp_us = 0x04030201;
    f.bus = 0;

    uint8_t out[gvret::kMaxEncodedFrame];
    size_t n = gvret::encodeFrame(f, out, sizeof(out));

    TEST_ASSERT_EQUAL_size_t(12u + 2u, n);
    TEST_ASSERT_EQUAL_HEX8(0xF1, out[0]);          // prefix
    TEST_ASSERT_EQUAL_HEX8(0x00, out[1]);          // build-can-frame marker
    TEST_ASSERT_EQUAL_HEX8(0x01, out[2]);          // timestamp LE
    TEST_ASSERT_EQUAL_HEX8(0x02, out[3]);
    TEST_ASSERT_EQUAL_HEX8(0x03, out[4]);
    TEST_ASSERT_EQUAL_HEX8(0x04, out[5]);
    TEST_ASSERT_EQUAL_HEX8(0x23, out[6]);          // id LE
    TEST_ASSERT_EQUAL_HEX8(0x01, out[7]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[8]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[9]);          // bit 31 clear => standard
    TEST_ASSERT_EQUAL_HEX8(0x02, out[10]);         // dlc | (bus<<4)
    TEST_ASSERT_EQUAL_HEX8(0xAA, out[11]);
    TEST_ASSERT_EQUAL_HEX8(0xBB, out[12]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[13]);         // checksum byte
}

void test_encode_extended_frame_sets_high_bit(void) {
    CanFrame f;
    f.id = 0x18DAF110;
    f.extended = true;
    f.dlc = 0;
    f.timestamp_us = 0;

    uint8_t out[gvret::kMaxEncodedFrame];
    size_t n = gvret::encodeFrame(f, out, sizeof(out));

    TEST_ASSERT_EQUAL_size_t(12u, n);
    uint32_t id = (uint32_t)out[6] | ((uint32_t)out[7] << 8) |
                  ((uint32_t)out[8] << 16) | ((uint32_t)out[9] << 24);
    TEST_ASSERT_TRUE((id & 0x80000000u) != 0u);    // extended flag
    TEST_ASSERT_EQUAL_HEX32(0x18DAF110u, id & 0x7FFFFFFFu);
}

void test_encode_clamps_dlc_and_respects_capacity(void) {
    CanFrame f;
    f.dlc = 8;
    uint8_t out[gvret::kMaxEncodedFrame];
    size_t n = gvret::encodeFrame(f, out, sizeof(out));
    TEST_ASSERT_EQUAL_size_t(20u, n);

    // Buffer too small => returns 0, writes nothing.
    uint8_t tiny[4];
    TEST_ASSERT_EQUAL_size_t(0u, gvret::encodeFrame(f, tiny, sizeof(tiny)));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_encode_standard_frame_layout);
    RUN_TEST(test_encode_extended_frame_sets_high_bit);
    RUN_TEST(test_encode_clamps_dlc_and_respects_capacity);
    return UNITY_END();
}
