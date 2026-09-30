#pragma once

#include "detection_link/Transport.hpp"
#include "esp_now.h"
#include <mutex>

namespace detection_link {

/**
 * @brief Transport over ESP-NOW addressed unicast
 *
 * Owns ESP-NOW initialization, the peer entry and the ESP-NOW callbacks. Wi-Fi must already be
 * started on the configured channel; the application owns Wi-Fi startup and shutdown.
 *
 * The ESP-NOW callbacks run on the Wi-Fi task. They only copy the packet or send result into
 * bounded storage guarded by a mutex; update() dispatches it on the application's task.
 * Send completions have their own reserved slot, and a new send is refused until the previous
 * completion has been dispatched, so a full receive queue can never hide a completion.
 *
 * ESP-NOW callbacks carry no context pointer, so only one instance can be started at a time.
 */
class EspNowTransport : public Transport {
public:
    static constexpr size_t RX_QUEUE_DEPTH = 8;

    explicit EspNowTransport(Clock clock = steady_clock_ms);
    ~EspNowTransport() override;

    EspNowTransport(const EspNowTransport&) = delete;
    EspNowTransport& operator=(const EspNowTransport&) = delete;

    /**
     * @return esp_err_t ESP_OK, ESP_ERR_INVALID_STATE if this or another instance is already
     *         started, or the error from ESP-NOW initialization or adding the peer
     */
    esp_err_t begin(const TransportConfig& config) override;
    SubmitResult send(const MacAddress& peer, const uint8_t* data, size_t length, uint32_t token) override;
    bool send_pending() const override;
    void update(TransportHandler& handler) override;
    /**
     * @brief Stop ESP-NOW. After this returns, no callback touches this object. Queued events are
     *        discarded, and a send still in flight is abandoned without a completion.
     */
    void end() override;
    TransportStats stats() const override;

private:
    struct Packet {
        MacAddress from;
        uint8_t length;
        uint8_t data[MAX_PACKET_SIZE];
    };

    static void on_recv(const esp_now_recv_info_t* info, const uint8_t* data, int length);
    static void on_send(const esp_now_send_info_t* info, esp_now_send_status_t status);
    void clear_locked();

    // Guards s_instance and every member below it
    static std::mutex s_mutex;
    static EspNowTransport* s_instance;

    Clock m_clock;
    TransportConfig m_config;
    bool m_started = false;

    Packet m_rx[RX_QUEUE_DEPTH];
    size_t m_rx_head = 0;
    size_t m_rx_count = 0;

    bool m_in_flight = false;
    uint32_t m_in_flight_token = 0;
    int64_t m_in_flight_since_ms = 0;
    // Sends that timed out; their late completions are discarded as they arrive
    uint32_t m_abandoned_sends = 0;

    bool m_completion_ready = false;
    uint32_t m_completion_token = 0;
    bool m_completion_delivered = false;

    TransportStats m_stats;
};

} // namespace detection_link
