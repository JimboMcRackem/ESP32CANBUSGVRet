#include <unity.h>
#include "settings.h"

void setUp(void) {}
void tearDown(void) {}

void test_default_bitrate_is_500k(void) {
    TEST_ASSERT_EQUAL_UINT32(500000u, settings::kDefaultBitrate);
}

void test_valid_bitrate_passes_through(void) {
    TEST_ASSERT_EQUAL_UINT32(250000u, settings::sanitizeBitrate(250000u));
    TEST_ASSERT_EQUAL_UINT32(1000000u, settings::sanitizeBitrate(1000000u));
}

void test_zero_bitrate_falls_back_to_default(void) {
    TEST_ASSERT_EQUAL_UINT32(settings::kDefaultBitrate, settings::sanitizeBitrate(0u));
}

void test_out_of_range_bitrate_falls_back_to_default(void) {
    TEST_ASSERT_EQUAL_UINT32(settings::kDefaultBitrate, settings::sanitizeBitrate(5u));
    TEST_ASSERT_EQUAL_UINT32(settings::kDefaultBitrate, settings::sanitizeBitrate(2000000u));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_default_bitrate_is_500k);
    RUN_TEST(test_valid_bitrate_passes_through);
    RUN_TEST(test_zero_bitrate_falls_back_to_default);
    RUN_TEST(test_out_of_range_bitrate_falls_back_to_default);
    return UNITY_END();
}
