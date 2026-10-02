#pragma once

#include "esp_err.h"
#include <array>
#include <cstdint>
#include <cstddef>

namespace detection_link {

using MacAddress = std::array<uint8_t, 6>;

// Monotonic milliseconds. The default reads std::chrono::steady_clock; tests substitute their own.
using Clock = int64_t (*)();
int64_t steady_clock_ms();

// Largest packet the transport accepts in either direction. Larger packets are rejected.
constexpr size_t MAX_PACKET_SIZE = 32;

struct TransportConfig {
    // Wi-Fi channel both devices use. Wi-Fi must already be started on it.
    uint8_t channel = 1;
    // The only peer this device exchanges packets with
    MacAddress peer = {};
    // A send whose completion has not arrived after this long is reported as failed
    uint32_t send_completion_timeout_ms = 200;
};

// Immediate outcome of submitting a packet. Delivery is reported later by on_send_complete().
enum class SubmitResult {
    Ok,
    Busy,            // A transmission is in flight, or its completion has not been dispatched yet
    NotStarted,
    InvalidArgument, // Empty or oversized packet
    Failed,          // The radio rejected the submission
};

// Diagnostic counters. They only increase until the transport is started again.
struct TransportStats {
    uint32_t packets_queued = 0;
    uint32_t packets_dropped = 0;        // Receive queue was full
    uint32_t packets_rejected = 0;       // Empty or oversized
    uint32_t submit_failures = 0;
    uint32_t completion_timeouts = 0;
    uint32_t unexpected_completions = 0; // Late completions after a timeout, or with nothing in flight
};

class TransportHandler {
public:
    virtual ~TransportHandler() = default;
    virtual void on_packet(const MacAddress& from, const uint8_t* data, size_t length) = 0;
    // delivered is the transmission result reported by the radio for the send tagged with token
    virtual void on_send_complete(uint32_t token, bool delivered) = 0;
};

/**
 * @brief A unicast packet link to a single peer with one transmission in flight at a time
 *
 * Events are queued from the radio's context and dispatched only from update(), on the caller's task.
 */
class Transport {
public:
    virtual ~Transport() = default;
    virtual esp_err_t begin(const TransportConfig& config) = 0;
    virtual SubmitResult send(const MacAddress& peer, const uint8_t* data, size_t length, uint32_t token) = 0;
    // True while a send is in flight or its completion is waiting for update()
    virtual bool send_pending() const = 0;
    // Dispatch queued receive and completion events to handler. The handler may call send().
    virtual void update(TransportHandler& handler) = 0;
    virtual void end() = 0;
    virtual TransportStats stats() const = 0;
};

} // namespace detection_link
