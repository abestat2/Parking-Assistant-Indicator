#pragma once

#include "hal/rmt_types.h"
#include <cstdint>
#include <cstddef>

namespace mb1013 {

// Results returned in place of a distance. All are negative, so any value >= 0 is a resolved distance in mm.

// No complete pulse was measured: the sensor sent nothing, or the pulse outlasted the receive window
constexpr int32_t RANGE_UNRESOLVED = -1;
// The capture started partway through a pulse; call again to measure the next one
constexpr int32_t RANGE_IN_PROGRESS = -2;
// The driver is not initialized or the RMT peripheral reported an error
constexpr int32_t RANGE_ERROR = -3;

/**
 * @brief Convert one RMT capture of the MB1013 PW output into a distance
 *
 * The RMT receiver starts recording on the first edge after it is armed and stops once a level
 * lasts longer than the idle threshold, which it marks with a zero duration. With a 1 MHz
 * channel, one tick is 1 us, which is 1 mm on the MB1013 PW interface.
 *
 * @param symbols Captured symbols
 * @param num_symbols Number of captured symbols
 * @return int32_t Distance in mm, or one of the RANGE_* codes above
 */
int32_t decode_pulse_mm(const rmt_symbol_word_t* symbols, size_t num_symbols);

} // namespace mb1013
