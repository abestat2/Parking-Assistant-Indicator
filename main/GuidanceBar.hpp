#pragma once

#include <cstdint>

// What the LED bar shows for one distance reading while guiding a car in
struct GuidanceBar {
    bool too_close; // Whole bar red: the car has passed the stop distance
    uint8_t lit;    // LEDs lit from the start of the bar
};

/**
 * @brief Map a distance to the bar: all LEDs green at max_mm or farther, shrinking linearly to one
 *        LED at min_mm, and the whole bar red closer than min_mm
 * @param distance_mm Measured distance, >= 0
 * @param min_mm Stop distance
 * @param max_mm Distance where guidance starts; must be greater than min_mm
 * @param led_count LEDs in the bar, at least 2
 */
constexpr GuidanceBar guidance_bar(int32_t distance_mm, int32_t min_mm, int32_t max_mm, uint8_t led_count)
{
    if (distance_mm < min_mm) {
        return {true, led_count};
    }
    if (distance_mm >= max_mm) {
        return {false, led_count};
    }
    // min_mm lights one LED; each further (max - min) / (count - 1) lights one more
    const int64_t extra = static_cast<int64_t>(distance_mm - min_mm) * (led_count - 1) / (max_mm - min_mm);
    return {false, static_cast<uint8_t>(1 + extra)};
}

static_assert(guidance_bar(299, 300, 2000, 8).too_close);
static_assert(guidance_bar(300, 300, 2000, 8).lit == 1);
static_assert(guidance_bar(1999, 300, 2000, 8).lit == 7);
static_assert(guidance_bar(2000, 300, 2000, 8).lit == 8);
static_assert(guidance_bar(5000, 300, 2000, 8).lit == 8 && !guidance_bar(5000, 300, 2000, 8).too_close);
