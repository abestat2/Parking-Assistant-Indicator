#include "MB1013_decode.hpp"

namespace mb1013 {

int32_t decode_pulse_mm(const rmt_symbol_word_t* symbols, size_t num_symbols) {
    if (!symbols || num_symbols == 0) {
        return RANGE_UNRESOLVED;
    }

    const rmt_symbol_word_t& first = symbols[0];

    // The PW line idles low, so a capture armed between pulses starts on a rising edge.
    // Starting low means the first edge seen was the falling edge of a pulse already in progress.
    if (first.level0 == 0) {
        return RANGE_IN_PROGRESS;
    }

    // A zero duration marks a level that outlasted the idle threshold: the pulse never ended
    if (first.duration0 == 0) {
        return RANGE_UNRESOLVED;
    }

    return static_cast<int32_t>(first.duration0);
}

} // namespace mb1013
