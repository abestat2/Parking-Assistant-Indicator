#include "detection_link/ExchangeId.hpp"
#include "esp_attr.h"
#include "esp_random.h"

namespace detection_link {

// RTC slow memory keeps its contents through deep sleep. It is reloaded with these zeros on a
// power-on or reset, which clears the marker and makes next_exchange_id() reseed.
RTC_DATA_ATTR static ExchangeIdState s_exchange_ids = {0, 0};

uint32_t retained_exchange_id() {
    return next_exchange_id(s_exchange_ids, esp_random);
}

} // namespace detection_link
