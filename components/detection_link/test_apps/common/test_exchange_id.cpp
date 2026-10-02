// Tier 1: exchange ID generation, shared by the host and target test apps

#include "unity.h"
#include "detection_link/ExchangeId.hpp"
#include "detection_link/Protocol.hpp"

using namespace detection_link;

static uint32_t s_random_value;
static int s_random_calls;

static uint32_t fake_random()
{
    ++s_random_calls;
    return s_random_value;
}

TEST_CASE("exchange id: fresh state is seeded from the random source", "[detection_link][exchange_id]")
{
    ExchangeIdState state = {0, 0};
    s_random_value = 1000;
    s_random_calls = 0;
    TEST_ASSERT_EQUAL_UINT32(1001, next_exchange_id(state, fake_random));
    TEST_ASSERT_EQUAL(1, s_random_calls);
    TEST_ASSERT_EQUAL_UINT32(EXCHANGE_ID_MARKER, state.marker);
}

TEST_CASE("exchange id: retained state counts up without reseeding", "[detection_link][exchange_id]")
{
    // As after a deep-sleep wakeup: the marker and counter survived
    ExchangeIdState state = {EXCHANGE_ID_MARKER, 500};
    s_random_calls = 0;
    TEST_ASSERT_EQUAL_UINT32(501, next_exchange_id(state, fake_random));
    TEST_ASSERT_EQUAL_UINT32(502, next_exchange_id(state, fake_random));
    TEST_ASSERT_EQUAL(0, s_random_calls);
}

TEST_CASE("exchange id: garbage state after power-on is reseeded", "[detection_link][exchange_id]")
{
    ExchangeIdState state = {0xDEADBEEF, 7};
    s_random_value = 0x40000000;
    s_random_calls = 0;
    TEST_ASSERT_EQUAL_UINT32(0x40000001, next_exchange_id(state, fake_random));
    TEST_ASSERT_EQUAL(1, s_random_calls);
}

TEST_CASE("exchange id: zero is never produced", "[detection_link][exchange_id]")
{
    ExchangeIdState state = {EXCHANGE_ID_MARKER, 0xFFFFFFFF};
    TEST_ASSERT_EQUAL_UINT32(1, next_exchange_id(state, fake_random));

    // A seed that would produce zero is skipped too
    ExchangeIdState seeded = {0, 0};
    s_random_value = 0xFFFFFFFF;
    TEST_ASSERT_NOT_EQUAL(NO_EXCHANGE, next_exchange_id(seeded, fake_random));
}
