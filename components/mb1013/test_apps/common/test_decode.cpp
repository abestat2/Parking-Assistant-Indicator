// Tier 1: pure decoding tests, shared by the host and target test apps

#include "unity.h"
#include "MB1013_decode.hpp"

using namespace mb1013;

static rmt_symbol_word_t symbol(uint32_t level0, uint32_t duration0, uint32_t level1, uint32_t duration1) {
    rmt_symbol_word_t s = {};
    s.level0 = level0;
    s.duration0 = duration0;
    s.level1 = level1;
    s.duration1 = duration1;
    return s;
}

TEST_CASE("decode: complete pulse returns its width in mm", "[mb1013][decode]")
{
    // Rising edge, 1500 us high, then low until the idle threshold ends the capture
    rmt_symbol_word_t symbols[] = { symbol(1, 1500, 0, 0) };
    TEST_ASSERT_EQUAL_INT32(1500, decode_pulse_mm(symbols, 1));
}

TEST_CASE("decode: pulse longer than the receive window is unresolved", "[mb1013][decode]")
{
    // High level outlasted the idle threshold, so the driver marked it with a zero duration
    rmt_symbol_word_t symbols[] = { symbol(1, 0, 0, 0) };
    TEST_ASSERT_EQUAL_INT32(RANGE_UNRESOLVED, decode_pulse_mm(symbols, 1));
}

TEST_CASE("decode: capture starting low means a pulse was in progress", "[mb1013][decode]")
{
    // First edge seen was the falling edge of a pulse that began before the receiver was armed
    rmt_symbol_word_t symbols[] = { symbol(0, 0, 0, 0) };
    TEST_ASSERT_EQUAL_INT32(RANGE_IN_PROGRESS, decode_pulse_mm(symbols, 1));
}

TEST_CASE("decode: capture starting low with a later pulse is still in progress", "[mb1013][decode]")
{
    // A later pulse in the same capture is not trusted; the caller should read again
    rmt_symbol_word_t symbols[] = { symbol(0, 800, 1, 1200), symbol(0, 0, 0, 0) };
    TEST_ASSERT_EQUAL_INT32(RANGE_IN_PROGRESS, decode_pulse_mm(symbols, 2));
}

TEST_CASE("decode: empty capture is unresolved", "[mb1013][decode]")
{
    rmt_symbol_word_t symbols[] = { symbol(1, 1500, 0, 0) };
    TEST_ASSERT_EQUAL_INT32(RANGE_UNRESOLVED, decode_pulse_mm(symbols, 0));
    TEST_ASSERT_EQUAL_INT32(RANGE_UNRESOLVED, decode_pulse_mm(nullptr, 0));
    TEST_ASSERT_EQUAL_INT32(RANGE_UNRESOLVED, decode_pulse_mm(nullptr, 1));
}

TEST_CASE("decode: only the first pulse in a capture is used", "[mb1013][decode]")
{
    rmt_symbol_word_t symbols[] = { symbol(1, 700, 0, 200), symbol(1, 900, 0, 0) };
    TEST_ASSERT_EQUAL_INT32(700, decode_pulse_mm(symbols, 2));
}

TEST_CASE("decode: boundary widths are returned unchanged", "[mb1013][decode]")
{
    // 300 mm sensor minimum, 5000 mm sensor maximum, 32767 = largest 15-bit duration
    const uint32_t widths[] = { 1, 300, 5000, 32767 };
    for (uint32_t width : widths) {
        rmt_symbol_word_t symbols[] = { symbol(1, width, 0, 0) };
        TEST_ASSERT_EQUAL_INT32(static_cast<int32_t>(width), decode_pulse_mm(symbols, 1));
    }
}

TEST_CASE("decode: result codes are distinct and negative", "[mb1013][decode]")
{
    TEST_ASSERT_LESS_THAN_INT32(0, RANGE_UNRESOLVED);
    TEST_ASSERT_LESS_THAN_INT32(0, RANGE_IN_PROGRESS);
    TEST_ASSERT_LESS_THAN_INT32(0, RANGE_ERROR);
    TEST_ASSERT_NOT_EQUAL(RANGE_UNRESOLVED, RANGE_IN_PROGRESS);
    TEST_ASSERT_NOT_EQUAL(RANGE_UNRESOLVED, RANGE_ERROR);
    TEST_ASSERT_NOT_EQUAL(RANGE_IN_PROGRESS, RANGE_ERROR);
}
