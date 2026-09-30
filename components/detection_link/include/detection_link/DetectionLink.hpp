#pragma once

#include "detection_link/Protocol.hpp"
#include "detection_link/Transport.hpp"
#include <cstdint>
#include <cstddef>

namespace detection_link {

enum class Role {
    Detector, // Always powered: keeps the current item state and answers requests
    Receiver, // Wakes, requests the state, acknowledges it, then lets the application sleep
};

enum class FailureReason {
    None,                   // Events that are not failures
    InitializationFailure,  // Wi-Fi or ESP-NOW could not start (begin() returns the error)
    ResponseTimeout,        // Receiver did not obtain a valid response
    AckTransmissionFailure, // Receiver has a response but could not transmit its acknowledgment
    AcknowledgmentMissing,  // Detector responded but no acknowledgment arrived in time
    ReceiverOverdue,        // Detector has had no completed exchange within the contact allowance
    TransportFailure,       // Packets could not be submitted to the radio
};

enum class Health {
    Unknown, // No exchange has completed or failed yet (detector: still within the initial grace)
    Ok,
    Lost,
};

// All durations are milliseconds on the device's own monotonic clock
struct LinkConfig {
    TransportConfig transport;

    // Receiver
    uint32_t response_timeout_ms = 50;    // Wait for a response after a request transmits successfully
    uint8_t request_attempt_limit = 3;    // Request transmissions, including the first
    uint8_t ack_attempt_limit = 3;        // Acknowledgment transmissions, including the first
    uint32_t exchange_deadline_ms = 400;  // Upper bound on one exchange, from request_status()
    // Returns a fresh, non-zero exchange ID for each exchange. Required for the receiver; use
    // retained_exchange_id() so IDs are not reused across deep-sleep wakeups.
    uint32_t (*exchange_id_source)() = nullptr;

    // Both roles: pause before resubmitting after a submission error or failed transmission
    uint32_t retry_delay_ms = 10;

    // Detector
    uint8_t response_attempt_limit = 3;       // Response transmissions per received request
    uint32_t detector_ack_deadline_ms = 500;   // Wait for the acknowledgment; cover the receiver's retries
    uint32_t expected_contact_interval_ms = 60000;
    uint32_t contact_margin_ms = 15000;
    uint32_t initial_contact_grace_ms = 90000; // Overdue deadline after begin(), before any contact
};

enum class LinkEventType {
    ExchangeComplete,      // exchange_id, item_state
    ExchangeFailed,        // exchange_id, reason
    CommunicationLost,     // reason
    CommunicationRestored,
};

struct LinkEvent {
    LinkEventType type;
    uint32_t exchange_id;
    ItemState item_state;
    FailureReason reason;
};

// Diagnostic counters since begin()
struct LinkDiagnostics {
    uint32_t requests_received = 0;     // Detector: requests that started a new exchange
    uint32_t duplicate_requests = 0;    // Detector: repeats of the active or last exchange
    uint32_t duplicate_acks = 0;        // Detector: repeats for an exchange already finished
    uint32_t unknown_acks = 0;          // Detector: for no known exchange, e.g. after a restart
    uint32_t duplicate_responses = 0;   // Receiver: repeats of an accepted response
    uint32_t stale_messages = 0;        // Valid messages of the wrong type or for another exchange
    uint32_t wrong_peer = 0;
    uint32_t malformed = 0;
    uint32_t submit_failures = 0;
    uint32_t transmit_failures = 0;     // Sends the radio reported as not delivered
    uint32_t events_dropped = 0;        // Event queue was full because poll_event() was not called
};

/**
 * @brief Request/response/acknowledge protocol between a detector and a receiver
 *
 * One class serves both roles; begin() selects which. All protocol work happens in update(), which
 * never blocks: retries are paced by deadlines checked on later calls. Events are queued and read
 * with poll_event() on the application's task.
 *
 * Receiver: request_status() starts an exchange. It ends with exactly one ExchangeComplete (after the
 * acknowledgment has been transmitted successfully) or ExchangeFailed. Then the application may
 * call end(), stop Wi-Fi and sleep.
 *
 * Detector: set_local_item_state() keeps the latest sensor state. Each new request is answered from a
 * snapshot taken when it arrives. ExchangeComplete is emitted when the acknowledgment arrives.
 *
 * The link does not own Wi-Fi, sleep, or the transport object; it starts and stops the transport.
 */
class DetectionLink : private TransportHandler {
public:
    static constexpr size_t EVENT_QUEUE_DEPTH = 8;

    explicit DetectionLink(Transport& transport, Clock clock = steady_clock_ms);
    ~DetectionLink() override;

    DetectionLink(const DetectionLink&) = delete;
    DetectionLink& operator=(const DetectionLink&) = delete;

    /**
     * @return esp_err_t ESP_OK; ESP_ERR_INVALID_STATE if already started; ESP_ERR_INVALID_ARG for a
     *         receiver without exchange_id_source or with zero attempt limits; or the transport's error
     */
    esp_err_t begin(Role role, const LinkConfig& config);

    // Detector: the latest sensor state. May be called at any time, including before begin().
    void set_local_item_state(ItemState state);

    /**
     * @brief Receiver: start a new exchange with a fresh exchange ID
     * @return esp_err_t ESP_OK; ESP_ERR_INVALID_STATE if not started, not a receiver, or an
     *         exchange is still in progress
     */
    esp_err_t request_status();

    // Pump the transport, advance the protocol and check deadlines
    void update();

    // Take the next queued event. Returns false when there are none.
    bool poll_event(LinkEvent& event);

    /**
     * @brief Cancel any exchange without an event and stop the transport. Wi-Fi is left running.
     *        Queued events stay readable with poll_event().
     */
    void end();

    bool started() const { return m_started; }
    // Receiver: true between request_status() and its ExchangeComplete or ExchangeFailed
    bool exchange_in_progress() const;
    Health health() const { return m_health; }
    LinkDiagnostics diagnostics() const { return m_diagnostics; }

private:
    enum class ReceiverState { Idle, Requesting, AwaitingResponse, Acknowledging, Complete, Failed };
    enum class PendingSend { None, Request, Ack, Response };

    void on_packet(const MacAddress& from, const uint8_t* data, size_t length) override;
    void on_send_complete(uint32_t token, bool delivered) override;

    void receiver_on_message(const Message& message);
    void receiver_on_send_complete(PendingSend kind, bool delivered);
    void receiver_service();
    void receiver_fail(FailureReason reason);

    void detector_on_message(const Message& message);
    void detector_on_send_complete(PendingSend kind, uint32_t exchange_id, bool delivered);
    void detector_service();
    void detector_complete_active();
    void detector_abandon_active();

    SubmitResult submit(MessageType type, uint32_t exchange_id, ItemState state, PendingSend kind);
    void emit(const LinkEvent& event);
    void set_health_lost(FailureReason reason);
    void set_health_ok();

    Transport& m_transport;
    Clock m_clock;
    LinkConfig m_config = {};
    Role m_role = Role::Receiver;
    bool m_started = false;
    int64_t m_now = 0;
    Health m_health = Health::Unknown;
    LinkDiagnostics m_diagnostics;

    // The send this link is waiting on. Completions with any other token are ignored.
    uint32_t m_next_token = 0;
    uint32_t m_pending_token = 0;
    PendingSend m_pending_kind = PendingSend::None;
    uint32_t m_pending_exchange = NO_EXCHANGE;

    // Receiver
    ReceiverState m_rx_state = ReceiverState::Idle;
    uint32_t m_exchange_id = NO_EXCHANGE;
    int64_t m_exchange_deadline = 0;
    int64_t m_response_deadline = 0;
    int64_t m_next_attempt_at = 0;
    uint8_t m_request_attempts = 0;
    uint8_t m_ack_attempts = 0;
    bool m_last_request_submit_failed = false;
    ItemState m_received_state = ItemState::Unknown;

    // Detector
    ItemState m_local_state = ItemState::Unknown;
    struct DetectorExchange {
        bool active = false;
        uint32_t id = NO_EXCHANGE;
        ItemState snapshot = ItemState::Unknown;
        int64_t ack_deadline = 0;
        bool response_wanted = false;  // Duplicate requests coalesce into this one flag
        uint8_t response_attempts = 0;
        int64_t next_response_at = 0;
    } m_active;
    // The last exchange that completed or was abandoned, so its repeats are recognized
    struct {
        bool valid = false;
        uint32_t id = NO_EXCHANGE;
        ItemState snapshot = ItemState::Unknown;
    } m_recent;
    int64_t m_contact_deadline = 0;

    LinkEvent m_events[EVENT_QUEUE_DEPTH];
    size_t m_event_head = 0;
    size_t m_event_count = 0;
};

const char* to_string(FailureReason reason);

} // namespace detection_link
