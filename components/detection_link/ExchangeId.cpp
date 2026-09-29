#include "detection_link/ExchangeId.hpp"
#include "detection_link/Protocol.hpp"

namespace detection_link {

uint32_t next_exchange_id(ExchangeIdState& state, uint32_t (*random)()) {
    if (state.marker != EXCHANGE_ID_MARKER) {
        state.counter = random();
        state.marker = EXCHANGE_ID_MARKER;
    }
    ++state.counter;
    if (state.counter == NO_EXCHANGE) {
        ++state.counter;
    }
    return state.counter;
}

} // namespace detection_link
