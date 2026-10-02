// Tier 3: exchange IDs kept in RTC memory. Runs on the board and in QEMU.

#include "unity.h"
#include "detection_link/ExchangeId.hpp"
#include "detection_link/Protocol.hpp"

using namespace detection_link;

TEST_CASE("target: retained exchange IDs are non-zero and count up", "[detection_link][exchange_id]")
{
    const uint32_t first = retained_exchange_id();
    const uint32_t second = retained_exchange_id();
    TEST_ASSERT_NOT_EQUAL(NO_EXCHANGE, first);
    TEST_ASSERT_NOT_EQUAL(NO_EXCHANGE, second);
    if (first != 0xFFFFFFFF) {
        TEST_ASSERT_EQUAL_UINT32(first + 1, second);
    }
}
