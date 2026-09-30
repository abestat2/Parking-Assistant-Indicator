#include "detection_link/EspNowTransport.hpp"
#include <cstring>

namespace detection_link {

std::mutex EspNowTransport::s_mutex;
EspNowTransport* EspNowTransport::s_instance = nullptr;

EspNowTransport::EspNowTransport(Clock clock) : m_clock(clock) {
}

EspNowTransport::~EspNowTransport() {
    end();
}

esp_err_t EspNowTransport::begin(const TransportConfig& config) {
    {
        std::lock_guard<std::mutex> lock(s_mutex);
        if (m_started || s_instance) {
            return ESP_ERR_INVALID_STATE;
        }
        m_config = config;
        m_stats = {};
        clear_locked();
        // Claim the callbacks before registering them
        s_instance = this;
    }

    esp_err_t err = esp_now_init();
    if (err != ESP_OK) {
        std::lock_guard<std::mutex> lock(s_mutex);
        s_instance = nullptr;
        return err;
    }
    err = esp_now_register_recv_cb(on_recv);
    if (err == ESP_OK) {
        err = esp_now_register_send_cb(on_send);
    }
    if (err == ESP_OK) {
        esp_now_peer_info_t peer = {};
        std::memcpy(peer.peer_addr, config.peer.data(), config.peer.size());
        peer.channel = config.channel;
        peer.ifidx = WIFI_IF_STA;
        peer.encrypt = false;
        err = esp_now_add_peer(&peer);
    }
    if (err != ESP_OK) {
        // esp_now_deinit() also removes any registered callbacks and peers
        esp_now_deinit();
        std::lock_guard<std::mutex> lock(s_mutex);
        s_instance = nullptr;
        return err;
    }

    std::lock_guard<std::mutex> lock(s_mutex);
    m_started = true;
    return ESP_OK;
}

SubmitResult EspNowTransport::send(const MacAddress& peer, const uint8_t* data, size_t length, uint32_t token) {
    {
        std::lock_guard<std::mutex> lock(s_mutex);
        if (!m_started) {
            return SubmitResult::NotStarted;
        }
        if (!data || length == 0 || length > MAX_PACKET_SIZE) {
            return SubmitResult::InvalidArgument;
        }
        if (m_in_flight || m_completion_ready) {
            return SubmitResult::Busy;
        }
        // Mark the send in flight first: its completion can arrive before esp_now_send() returns
        m_in_flight = true;
        m_in_flight_token = token;
        m_in_flight_since_ms = m_clock();
    }

    if (esp_now_send(peer.data(), data, length) == ESP_OK) {
        return SubmitResult::Ok;
    }

    std::lock_guard<std::mutex> lock(s_mutex);
    m_in_flight = false;
    ++m_stats.submit_failures;
    return SubmitResult::Failed;
}

bool EspNowTransport::send_pending() const {
    std::lock_guard<std::mutex> lock(s_mutex);
    return m_in_flight || m_completion_ready;
}

void EspNowTransport::update(TransportHandler& handler) {
    // Take everything queued so far, then dispatch without the lock so the handler can send
    Packet packets[RX_QUEUE_DEPTH];
    size_t packet_count = 0;
    bool completion_ready = false;
    uint32_t completion_token = 0;
    bool completion_delivered = false;
    {
        std::lock_guard<std::mutex> lock(s_mutex);
        if (!m_started) {
            return;
        }
        if (m_in_flight && m_clock() - m_in_flight_since_ms >= m_config.send_completion_timeout_ms) {
            // Report the send as failed; if its completion turns up later it is discarded
            m_in_flight = false;
            ++m_abandoned_sends;
            ++m_stats.completion_timeouts;
            m_completion_ready = true;
            m_completion_token = m_in_flight_token;
            m_completion_delivered = false;
        }
        while (m_rx_count > 0) {
            packets[packet_count++] = m_rx[m_rx_head];
            m_rx_head = (m_rx_head + 1) % RX_QUEUE_DEPTH;
            --m_rx_count;
        }
        if (m_completion_ready) {
            completion_ready = true;
            completion_token = m_completion_token;
            completion_delivered = m_completion_delivered;
            m_completion_ready = false;
        }
    }

    for (size_t i = 0; i < packet_count; ++i) {
        handler.on_packet(packets[i].from, packets[i].data, packets[i].length);
    }
    if (completion_ready) {
        handler.on_send_complete(completion_token, completion_delivered);
    }
}

void EspNowTransport::end() {
    {
        std::lock_guard<std::mutex> lock(s_mutex);
        if (!m_started) {
            return;
        }
    }
    esp_now_unregister_recv_cb();
    esp_now_unregister_send_cb();
    esp_now_deinit();

    // A callback already running waits here, then finds no instance
    std::lock_guard<std::mutex> lock(s_mutex);
    s_instance = nullptr;
    m_started = false;
    clear_locked();
}

TransportStats EspNowTransport::stats() const {
    std::lock_guard<std::mutex> lock(s_mutex);
    return m_stats;
}

void EspNowTransport::clear_locked() {
    m_rx_head = 0;
    m_rx_count = 0;
    m_in_flight = false;
    m_abandoned_sends = 0;
    m_completion_ready = false;
}

void EspNowTransport::on_recv(const esp_now_recv_info_t* info, const uint8_t* data, int length) {
    std::lock_guard<std::mutex> lock(s_mutex);
    EspNowTransport* self = s_instance;
    if (!self || !self->m_started || !info || !info->src_addr) {
        return;
    }
    if (!data || length <= 0 || static_cast<size_t>(length) > MAX_PACKET_SIZE) {
        ++self->m_stats.packets_rejected;
        return;
    }
    if (self->m_rx_count == RX_QUEUE_DEPTH) {
        ++self->m_stats.packets_dropped;
        return;
    }
    Packet& packet = self->m_rx[(self->m_rx_head + self->m_rx_count) % RX_QUEUE_DEPTH];
    std::memcpy(packet.from.data(), info->src_addr, packet.from.size());
    packet.length = static_cast<uint8_t>(length);
    std::memcpy(packet.data, data, length);
    ++self->m_rx_count;
    ++self->m_stats.packets_queued;
}

void EspNowTransport::on_send(const esp_now_send_info_t* info, esp_now_send_status_t status) {
    std::lock_guard<std::mutex> lock(s_mutex);
    EspNowTransport* self = s_instance;
    if (!self || !self->m_started) {
        return;
    }
    // Completions arrive in send order, so the oldest abandoned send owns this one
    if (self->m_abandoned_sends > 0) {
        --self->m_abandoned_sends;
        ++self->m_stats.unexpected_completions;
        return;
    }
    if (!self->m_in_flight) {
        ++self->m_stats.unexpected_completions;
        return;
    }
    self->m_in_flight = false;
    self->m_completion_ready = true;
    self->m_completion_token = self->m_in_flight_token;
    self->m_completion_delivered = status == ESP_NOW_SEND_SUCCESS;
}

} // namespace detection_link
