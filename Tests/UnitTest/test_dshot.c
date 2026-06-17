#include "drivers/dshot/dshot.h"
#include "unity.h"

#include <stdint.h>
#include <stdbool.h>

#ifndef UNIT_TEST
#error "UNIT_TEST must be defined for this file"
#endif

void setUp(void) {}
void tearDown(void) {}

/* --- Packet encoding tests --- */

void test_dshot_packet_zero_throttle(void) {
    /* value=0: packet=0, checksum=0, full=0x0000 */
    uint16_t result = DShotPreparePacket(0);
    TEST_ASSERT_EQUAL_HEX16(0x0000, result);
}

void test_dshot_packet_min_throttle(void) {
    /* value=48 (DSHOT_MIN_THROTTLE):
       packet = 96 = 0x0060
       csum   = (0x0060 ^ 0x0006 ^ 0x0000) & 0xF = 0x66 & 0xF = 6
       full   = (96 << 4) | 6 = 0x0606 */
    uint16_t result = DShotPreparePacket(DSHOT_MIN_THROTTLE);
    TEST_ASSERT_EQUAL_HEX16(0x0606, result);
}

void test_dshot_packet_max_throttle(void) {
    /* value=2047 (DSHOT_MAX_THROTTLE):
       packet = 4094 = 0x0FFE
       csum   = (0x0FFE ^ 0x00FF ^ 0x000F) & 0xF = 0x0F0E & 0xF = 14 = 0xE
       full   = (4094 << 4) | 14 = 0xFFEE */
    uint16_t result = DShotPreparePacket(DSHOT_MAX_THROTTLE);
    TEST_ASSERT_EQUAL_HEX16(0xFFEE, result);
}

void test_dshot_packet_known_value_8(void) {
    /* value=8: packet=16=0x0010
       csum = (0x0010 ^ 0x0001 ^ 0x0000) & 0xF = 0x0011 & 0xF = 1
       full = (16 << 4) | 1 = 0x0101
       Note: last bit (bit 0 of full) = 1 */
    uint16_t result = DShotPreparePacket(8);
    TEST_ASSERT_EQUAL_HEX16(0x0101, result);
}

void test_dshot_packet_checksum_nibble_xor(void) {
    /* Verify the 4-bit checksum equals XOR of the three nibbles of the
       11-bit throttle + telemetry field.
       value=100: packet=200=0x00C8
       nibbles: 0x8, 0xC, 0x0 → csum = 8^12^0 = 4
       full = (200 << 4) | 4 = 0x0C84 */
    uint16_t result = DShotPreparePacket(100);
    TEST_ASSERT_EQUAL_HEX16(0x0C84, result);
}

void test_dshot_packet_lsb_is_checksum_lsb(void) {
    /* The LSB of the 16-bit packet is the LSB of the checksum.
       For value=8, checksum=1 (odd) → LSB=1. */
    uint16_t result8 = DShotPreparePacket(8);
    TEST_ASSERT_EQUAL_UINT16(1, result8 & 0x1);

    /* For value=0, checksum=0 → LSB=0. */
    uint16_t result0 = DShotPreparePacket(0);
    TEST_ASSERT_EQUAL_UINT16(0, result0 & 0x1);
}

/* --- Timing constant tests --- */

void test_dshot_ticks_for_1_is_75_percent_of_period(void) {
    /* 320 / 427 ≈ 0.75. Verify with integer arithmetic: 320 * 4 == 427 * 3 ±1 */
    uint32_t period  = DSHOT_ARR + 1U;  /* 427 */
    uint32_t target  = (period * 3U) / 4U;  /* 320 */
    TEST_ASSERT_UINT32_WITHIN(2, target, DSHOT_TICKS_FOR_1);
}

void test_dshot_ticks_for_0_is_375_percent_of_period(void) {
    /* 160 / 427 ≈ 0.375. Verify: 160 * 8 == 427 * 3 ±2 */
    uint32_t period  = DSHOT_ARR + 1U;  /* 427 */
    uint32_t target  = (period * 3U) / 8U;  /* 160 */
    TEST_ASSERT_UINT32_WITHIN(2, target, DSHOT_TICKS_FOR_0);
}

void test_dshot_ticks_for_1_greater_than_ticks_for_0(void) {
    TEST_ASSERT_GREATER_THAN_UINT32(DSHOT_TICKS_FOR_0, DSHOT_TICKS_FOR_1);
}

void test_dshot_ticks_within_period(void) {
    TEST_ASSERT_LESS_THAN_UINT32(DSHOT_ARR + 1U, DSHOT_TICKS_FOR_1);
    TEST_ASSERT_LESS_THAN_UINT32(DSHOT_ARR + 1U, DSHOT_TICKS_FOR_0);
}

void test_dshot_buffer_size_is_frame_plus_reset(void) {
    TEST_ASSERT_EQUAL_UINT32(DSHOT_FRAME_SIZE + 2U, DSHOT_DMA_BUFFER_SIZE);
}

/* --- Run all tests --- */

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_dshot_packet_zero_throttle);
    RUN_TEST(test_dshot_packet_min_throttle);
    RUN_TEST(test_dshot_packet_max_throttle);
    RUN_TEST(test_dshot_packet_known_value_8);
    RUN_TEST(test_dshot_packet_checksum_nibble_xor);
    RUN_TEST(test_dshot_packet_lsb_is_checksum_lsb);
    RUN_TEST(test_dshot_ticks_for_1_is_75_percent_of_period);
    RUN_TEST(test_dshot_ticks_for_0_is_375_percent_of_period);
    RUN_TEST(test_dshot_ticks_for_1_greater_than_ticks_for_0);
    RUN_TEST(test_dshot_ticks_within_period);
    RUN_TEST(test_dshot_buffer_size_is_frame_plus_reset);
    return UNITY_END();
}
