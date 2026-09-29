#pragma once

#include <cstdint>

namespace detection_link {

// Counter state that must survive deep sleep (keep it in RTC memory). The marker tells a
// retained counter apart from uninitialized memory after a power-on or reset.
struct ExchangeIdState {
    uint32_t marker;
    uint32_t counter;
};

constexpr uint32_t EXCHANGE_ID_MARKER = 0x45584944; // "EXID"

/**
 * @brief Produce the next exchange ID from retained state
 *
 * On first use, and whenever the retained state was lost (power-on), the counter is seeded with
 * a random value, so a restart does not replay the IDs of the previous power cycle. After that it
 * counts up, so IDs are not reused across deep-sleep wakeups. NO_EXCHANGE (0) is skipped.
 *
 * @param state Retained counter state
 * @param random Random source used to seed the counter
 * @return uint32_t A non-zero exchange ID
 */
uint32_t next_exchange_id(ExchangeIdState& state, uint32_t (*random)());

/**
 * @brief next_exchange_id() over a counter kept in RTC memory, seeded from esp_random()
 *
 * Suitable as LinkConfig::exchange_id_source on the receiver. Call it after Wi-Fi has started so
 * esp_random() draws on the RF noise entropy source. Only available on chip targets.
 */
uint32_t retained_exchange_id();

} // namespace detection_link
