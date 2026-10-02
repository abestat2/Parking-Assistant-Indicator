#include "detection_link/DetectionLink.hpp"

namespace detection_link {

DetectionLink::DetectionLink(Transport& transport, Clock clock) : m_transport(transport), m_clock(clock) {
}

DetectionLink::~DetectionLink() {
    end();
}

esp_err_t DetectionLink::begin(Role role, const LinkConfig& config) {
    if (m_started) {
        return ESP_ERR_INVALID_STATE;
    }
    if (role == Role::Receiver &&
        (!config.exchange_id_source || config.request_attempt_limit == 0 || config.ack_attempt_limit == 0)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (role == Role::Detector && config.response_attempt_limit == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = m_transport.begin(config.transport);
    if (err != ESP_OK) {
        return err;
    }

    m_config = config;
    m_role = role;
    m_started = true;
    m_now = m_clock();
    m_health = Health::Unknown;
    m_diagnostics = {};
    m_pending_kind = PendingSend::None;
    m_rx_state = ReceiverState::Idle;
    m_exchange_id = NO_EXCHANGE;
    m_active = {};
    m_recent = {};
    m_contact_deadline = m_now + config.initial_contact_grace_ms;
    m_event_head = 0;
    m_event_count = 0;
    return ESP_OK;
}

void DetectionLink::set_local_item_state(ItemState state) {
    m_local_state = state;
}

esp_err_t DetectionLink::request_status() {
    if (!m_started || m_role != Role::Receiver || exchange_in_progress()) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint32_t id = m_config.exchange_id_source();
    if (id == NO_EXCHANGE) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    m_now = m_clock();
    m_exchange_id = id;
    m_rx_state = ReceiverState::Requesting;
    m_exchange_deadline = m_now + m_config.exchange_deadline_ms;
    m_next_attempt_at = m_now;
    m_request_attempts = 0;
    m_ack_attempts = 0;
    m_last_request_submit_failed = false;
    // Submit the first request now rather than on the next update()
    receiver_service();
    return ESP_OK;
}

void DetectionLink::update() {
    if (!m_started) {
        return;
    }
    m_now = m_clock();
    m_transport.update(*this);
    if (m_role == Role::Receiver) {
        receiver_service();
    } else {
        detector_service();
    }
}

bool DetectionLink::poll_event(LinkEvent& event) {
    if (m_event_count == 0) {
        return false;
    }
    event = m_events[m_event_head];
    m_event_head = (m_event_head + 1) % EVENT_QUEUE_DEPTH;
    --m_event_count;
    return true;
}

void DetectionLink::end() {
    if (!m_started) {
        return;
    }
    m_transport.end();
    m_started = false;
    m_pending_kind = PendingSend::None;
    if (exchange_in_progress()) {
        m_rx_state = ReceiverState::Idle;
    }
    m_active.active = false;
}

bool DetectionLink::exchange_in_progress() const {
    return m_rx_state == ReceiverState::Requesting || m_rx_state == ReceiverState::AwaitingResponse ||
           m_rx_state == ReceiverState::Acknowledging;
}

// ---- Transport events ----

void DetectionLink::on_packet(const MacAddress& from, const uint8_t* data, size_t length) {
    if (from != m_config.transport.peer) {
        ++m_diagnostics.wrong_peer;
        return;
    }
    Message message;
    if (decode(data, length, message) != DecodeResult::Ok) {
        ++m_diagnostics.malformed;
        return;
    }
    if (m_role == Role::Receiver) {
        receiver_on_message(message);
    } else {
        detector_on_message(message);
    }
}

void DetectionLink::on_send_complete(uint32_t token, bool delivered) {
    // Only the send this link is waiting on; anything else belongs to a cancelled exchange
    if (m_pending_kind == PendingSend::None || token != m_pending_token) {
        return;
    }
    const PendingSend kind = m_pending_kind;
    m_pending_kind = PendingSend::None;
    if (!delivered) {
        ++m_diagnostics.transmit_failures;
    }
    if (m_role == Role::Receiver) {
        receiver_on_send_complete(kind, delivered);
    } else {
        detector_on_send_complete(kind, m_pending_exchange, delivered);
    }
}

SubmitResult DetectionLink::submit(MessageType type, uint32_t exchange_id, ItemState state, PendingSend kind) {
    uint8_t packet[MAX_MESSAGE_SIZE];
    const size_t length = encode(Message{type, exchange_id, state}, packet, sizeof(packet));

    uint32_t token = ++m_next_token;
    if (token == 0) {
        token = ++m_next_token;
    }
    const SubmitResult result = m_transport.send(m_config.transport.peer, packet, length, token);
    if (result == SubmitResult::Ok) {
        m_pending_token = token;
        m_pending_kind = kind;
        m_pending_exchange = exchange_id;
    } else if (result != SubmitResult::Busy) {
        ++m_diagnostics.submit_failures;
    }
    return result;
}

// ---- Receiver ----

void DetectionLink::receiver_on_message(const Message& message) {
    if (message.type != MessageType::StatusResponse || message.exchange_id != m_exchange_id ||
        m_exchange_id == NO_EXCHANGE) {
        ++m_diagnostics.stale_messages;
        return;
    }
    switch (m_rx_state) {
    case ReceiverState::Requesting:
    case ReceiverState::AwaitingResponse:
        // Accept even if the request's own completion has not been processed yet
        m_received_state = message.item_state;
        m_rx_state = ReceiverState::Acknowledging;
        m_next_attempt_at = m_now;
        break;
    case ReceiverState::Acknowledging:
    case ReceiverState::Complete:
        // Already accepted; the acknowledgment in progress (or sent) covers it
        ++m_diagnostics.duplicate_responses;
        break;
    default:
        ++m_diagnostics.stale_messages;
        break;
    }
}

void DetectionLink::receiver_on_send_complete(PendingSend kind, bool delivered) {
    if (m_rx_state == ReceiverState::Requesting && kind == PendingSend::Request) {
        if (delivered) {
            m_rx_state = ReceiverState::AwaitingResponse;
            m_response_deadline = m_now + m_config.response_timeout_ms;
        } else {
            m_next_attempt_at = m_now + m_config.retry_delay_ms;
        }
    } else if (m_rx_state == ReceiverState::Acknowledging && kind == PendingSend::Ack) {
        if (delivered) {
            m_rx_state = ReceiverState::Complete;
            emit({LinkEventType::ExchangeComplete, m_exchange_id, m_received_state, FailureReason::None});
            set_health_ok();
        } else {
            m_next_attempt_at = m_now + m_config.retry_delay_ms;
        }
    }
    // Anything else is superseded, e.g. a request completing after its response was accepted
}

void DetectionLink::receiver_service() {
    if (!exchange_in_progress()) {
        return;
    }
    if (m_now >= m_exchange_deadline) {
        receiver_fail(m_rx_state == ReceiverState::Acknowledging ? FailureReason::AckTransmissionFailure
                                                                 : FailureReason::ResponseTimeout);
        return;
    }
    if (m_rx_state == ReceiverState::AwaitingResponse) {
        if (m_now < m_response_deadline) {
            return;
        }
        m_rx_state = ReceiverState::Requesting;
        m_next_attempt_at = m_now;
    }
    // One transmission at a time: wait for the previous one's completion to be processed
    if (m_transport.send_pending()) {
        return;
    }

    if (m_rx_state == ReceiverState::Requesting) {
        if (m_request_attempts >= m_config.request_attempt_limit) {
            receiver_fail(m_last_request_submit_failed ? FailureReason::TransportFailure
                                                       : FailureReason::ResponseTimeout);
            return;
        }
        if (m_now < m_next_attempt_at) {
            return;
        }
        ++m_request_attempts;
        const SubmitResult result = submit(MessageType::StatusRequest, m_exchange_id, ItemState::Unknown,
                                           PendingSend::Request);
        if (result == SubmitResult::Busy) {
            --m_request_attempts;
        } else {
            m_last_request_submit_failed = result != SubmitResult::Ok;
            if (result != SubmitResult::Ok) {
                m_next_attempt_at = m_now + m_config.retry_delay_ms;
            }
        }
    } else if (m_rx_state == ReceiverState::Acknowledging) {
        if (m_ack_attempts >= m_config.ack_attempt_limit) {
            receiver_fail(FailureReason::AckTransmissionFailure);
            return;
        }
        if (m_now < m_next_attempt_at) {
            return;
        }
        ++m_ack_attempts;
        const SubmitResult result = submit(MessageType::StatusAck, m_exchange_id, ItemState::Unknown,
                                           PendingSend::Ack);
        if (result == SubmitResult::Busy) {
            --m_ack_attempts;
        } else if (result != SubmitResult::Ok) {
            m_next_attempt_at = m_now + m_config.retry_delay_ms;
        }
    }
}

void DetectionLink::receiver_fail(FailureReason reason) {
    m_rx_state = ReceiverState::Failed;
    emit({LinkEventType::ExchangeFailed, m_exchange_id, ItemState::Unknown, reason});
    set_health_lost(reason);
}

// ---- Detector ----

void DetectionLink::detector_on_message(const Message& message) {
    if (message.type == MessageType::StatusRequest) {
        if (m_active.active && message.exchange_id == m_active.id) {
            // Resend the same snapshot; repeats coalesce into one pending response. The request
            // shows the peer is reachable, so answer it without waiting out a retry delay.
            ++m_diagnostics.duplicate_requests;
            m_active.response_wanted = true;
            m_active.response_attempts = 0;
            m_active.next_response_at = m_now;
            return;
        }
        if (m_recent.valid && message.exchange_id == m_recent.id) {
            // That exchange already finished; a late repeat gets no new snapshot
            ++m_diagnostics.duplicate_requests;
            return;
        }
        if (m_active.active) {
            // The receiver has moved on to a new exchange without acknowledging this one
            detector_abandon_active();
        }
        ++m_diagnostics.requests_received;
        m_active.active = true;
        m_active.id = message.exchange_id;
        m_active.snapshot = m_local_state;
        m_active.ack_deadline = m_now + m_config.detector_ack_deadline_ms;
        m_active.response_wanted = true;
        m_active.response_attempts = 0;
        m_active.next_response_at = m_now;
    } else if (message.type == MessageType::StatusAck) {
        if (m_active.active && message.exchange_id == m_active.id) {
            // Accept even if the response's own completion has not been processed yet
            detector_complete_active();
        } else if (m_recent.valid && message.exchange_id == m_recent.id) {
            ++m_diagnostics.duplicate_acks;
        } else {
            // No known exchange, e.g. the detector restarted; only a fresh request can complete one
            ++m_diagnostics.unknown_acks;
        }
    } else {
        ++m_diagnostics.stale_messages;
    }
}

void DetectionLink::detector_on_send_complete(PendingSend kind, uint32_t exchange_id, bool delivered) {
    // A completion arriving after the exchange finished cannot undo it
    if (kind != PendingSend::Response || !m_active.active || exchange_id != m_active.id) {
        return;
    }
    if (!delivered) {
        m_active.response_wanted = true;
        m_active.next_response_at = m_now + m_config.retry_delay_ms;
    }
}

void DetectionLink::detector_service() {
    if (m_active.active && m_now >= m_active.ack_deadline) {
        detector_abandon_active();
    }
    if (m_now >= m_contact_deadline) {
        set_health_lost(FailureReason::ReceiverOverdue);
    }

    if (!m_active.active || !m_active.response_wanted || m_now < m_active.next_response_at ||
        m_transport.send_pending()) {
        return;
    }
    if (m_active.response_attempts >= m_config.response_attempt_limit) {
        // Stop here; a duplicate request or the acknowledgment deadline decides what happens next
        m_active.response_wanted = false;
        return;
    }
    ++m_active.response_attempts;
    const SubmitResult result = submit(MessageType::StatusResponse, m_active.id, m_active.snapshot,
                                       PendingSend::Response);
    if (result == SubmitResult::Ok) {
        m_active.response_wanted = false;
    } else if (result == SubmitResult::Busy) {
        --m_active.response_attempts;
    } else {
        m_active.next_response_at = m_now + m_config.retry_delay_ms;
    }
}

void DetectionLink::detector_complete_active() {
    m_active.active = false;
    m_recent = {true, m_active.id, m_active.snapshot};
    m_contact_deadline = m_now + m_config.expected_contact_interval_ms + m_config.contact_margin_ms;
    emit({LinkEventType::ExchangeComplete, m_active.id, m_active.snapshot, FailureReason::None});
    set_health_ok();
}

void DetectionLink::detector_abandon_active() {
    m_active.active = false;
    m_recent = {true, m_active.id, m_active.snapshot};
    emit({LinkEventType::ExchangeFailed, m_active.id, ItemState::Unknown, FailureReason::AcknowledgmentMissing});
}

// ---- Events and health ----

void DetectionLink::emit(const LinkEvent& event) {
    if (m_event_count == EVENT_QUEUE_DEPTH) {
        ++m_diagnostics.events_dropped;
        return;
    }
    m_events[(m_event_head + m_event_count) % EVENT_QUEUE_DEPTH] = event;
    ++m_event_count;
}

void DetectionLink::set_health_lost(FailureReason reason) {
    if (m_health == Health::Lost) {
        return;
    }
    m_health = Health::Lost;
    emit({LinkEventType::CommunicationLost, NO_EXCHANGE, ItemState::Unknown, reason});
}

void DetectionLink::set_health_ok() {
    const bool was_lost = m_health == Health::Lost;
    m_health = Health::Ok;
    if (was_lost) {
        emit({LinkEventType::CommunicationRestored, NO_EXCHANGE, ItemState::Unknown, FailureReason::None});
    }
}

const char* to_string(FailureReason reason) {
    switch (reason) {
    case FailureReason::None: return "None";
    case FailureReason::InitializationFailure: return "InitializationFailure";
    case FailureReason::ResponseTimeout: return "ResponseTimeout";
    case FailureReason::AckTransmissionFailure: return "AckTransmissionFailure";
    case FailureReason::AcknowledgmentMissing: return "AcknowledgmentMissing";
    case FailureReason::ReceiverOverdue: return "ReceiverOverdue";
    case FailureReason::TransportFailure: return "TransportFailure";
    }
    return "Invalid";
}

} // namespace detection_link
