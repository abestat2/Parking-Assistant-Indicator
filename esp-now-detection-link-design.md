# ESP-NOW Detection Link — Implementation Specification

## Purpose and agreed behavior

Two ESP32-C3 devices exchange the current state of a sensor:

- **Detector:** continuously powered, maintains the current sensor state, and waits for requests.
- **Receiver:** normally in deep sleep; wakes on an application-defined schedule, requests the state, acknowledges the response, and returns control to the application for sleep.

The detector does not broadcast continuously. Communication timing is configurable and will be tuned on hardware. The receiver ends a successful exchange immediately rather than waiting out a fixed listening window.

The application decides when to enter deep sleep. For the intended application, it sleeps after the status acknowledgment has successfully finished transmitting and any necessary application work is complete. Item state does not automatically keep the communication class awake.

## Responsibilities

### `EspNowTransport`

Owns ESP-NOW initialization/deinitialization, peer configuration, packet submission, and delivery of receive/transmit events. Wi-Fi must already be started and on the configured channel; application startup owns Wi-Fi initialization and shutdown.

- Use one transport instance per device.
- Both devices use a fixed, agreed channel and known peer MAC addresses.
- Use addressed unicast for requests, responses, and acknowledgments.
- Allow one ESP-NOW transmission in flight at a time, simplifying send-result correlation.
- Report immediate submission errors separately from asynchronous transmission results.
- Copy incoming packets and required metadata into bounded, thread-safe storage.
- Keep Wi-Fi callbacks short; dispatch events from the application's normal task through `update()`.
- Reject oversized packets and surface queue overflow as a diagnostic.
- Ensure transmit completion cannot be silently lost behind a full receive queue, for example by reserving a completion slot.
- Shutdown must prevent callbacks from accessing destroyed objects or freed buffers.

The transport does not interpret item states, schedule retries, operate indicators, or enter sleep.

### `DetectionLink`

Uses `EspNowTransport` by composition. A configured role selects `Detector` or `Receiver` behavior in the same class.

Owns message encoding/validation, exchange IDs, protocol states, deadlines, retries, and communication-health events. It exposes `update()` and never performs blocking retry delays.

The detector application supplies the current sensor state continuously. A request is answered from that cached state, avoiding a sensor conversion in the response path.

### Application

Owns the sensor, physical outputs, error indications, Wi-Fi startup, wake schedule, and actual sleep entry. It supplies configuration and handles link events.

After `ExchangeComplete`, the application may process the received state, stop the link and radio, configure the next timer wakeup, and enter deep sleep. After `ExchangeFailed`, it chooses an error indication and recovery/sleep policy. Communication code must not force an indefinite awake period.

## Normal exchange

```text
Receiver                                     Detector
Wake and initialize Wi-Fi/ESP-NOW
STATUS_REQUEST(exchange ID) ----------------->
                         <------------------- STATUS_RESPONSE(exchange ID, state)
Validate and store state
STATUS_ACK(exchange ID) ---------------------> Record completed exchange
ACK send-completion callback succeeds
Emit ExchangeComplete in application task
Application shuts down radio and sleeps
```

An exchange succeeds locally on the receiver when a valid response has been accepted and the corresponding acknowledgment has successfully completed transmission. A successful return from the send function alone is insufficient.

The detector records success when its application receives and validates `STATUS_ACK`. There is no fourth message confirming that processing back to the receiver. Consequently, the two devices may occasionally disagree about completion; each reports what it actually observed.

## Protocol

All messages carry an application protocol identifier, protocol version, message type, and exchange ID. Encode fields explicitly with fixed-width integers and a documented byte order; do not transmit the memory layout of C++ objects.

| Message | Payload | Meaning |
| --- | --- | --- |
| `STATUS_REQUEST` | None | Request a current-state snapshot. |
| `STATUS_RESPONSE` | Item state: `Present`, `Absent`, or `Unknown` | Return the snapshot associated with this exchange. |
| `STATUS_ACK` | None | Confirm acceptance of that exchange's response. |

`Unknown` allows the detector to report that no valid sensor state is available. Communication can succeed while the sensor state is unknown. Missing communication is a separate error and must not be interpreted as `Absent`.

Generate a fresh exchange ID for each new receiver exchange; retries reuse it. IDs must avoid reuse across deep-sleep restarts, using retained identity/counter state or a suitably generated boot nonce and counter. The exact representation will be selected with the framework.

Each exchange has one immutable response snapshot. Duplicate requests receive the same cached response, even if the live sensor changes during the exchange. Cache lifetime covers the configured retry horizon. A new exchange takes a fresh snapshot.

Accept messages only from the configured peer, with the expected identifier, version, type, length, and active exchange ID. Unexpected or stale packets must not extend deadlines or clear errors. Duplicate acknowledgments are harmless and must not repeatedly extend the detector's health timer.

No next-contact delay is included in the acknowledgment. The expected contact interval is local detector configuration.

## Receiver state machine

```text
Idle -> Requesting -> AwaitingResponse -> Acknowledging -> Complete
             |              |                 |
             +--------------+-----------------+-------> Failed
                         bounded retry paths
```

1. After radio initialization, start a fresh exchange and its overall deadline.
2. Submit `STATUS_REQUEST` and track its transmission.
3. A submission error or reported transmission failure permits an early retry, without waiting through the full response timeout. Use a small configurable retry delay where appropriate.
4. After successful request transmission, allow a short response deadline. If no valid response arrives, retry within the attempt and overall limits.
5. Accept a matching response as soon as it arrives, including before the request's send-completion event is processed. A received response supersedes a later failure result for that request. Still wait until the transport is free before submitting the acknowledgment.
6. Store the response and submit `STATUS_ACK` as soon as possible.
7. Retry failed acknowledgment transmissions within their limits. A duplicate matching response may be acknowledged again, but must not reapply application actions or create uncontrolled additional transmissions.
8. After successful ACK transmission completion, stop protocol retries and emit exactly one `ExchangeComplete` event containing the accepted state and exchange ID.
9. On exhausted retries or a deadline, emit exactly one `ExchangeFailed` event with its reason. Never emit success for a later completion belonging to a terminated exchange.

There is no deliberate post-success listening delay. If an overall deadline expires while a transmission is still pending, terminate further protocol work and perform controlled transport shutdown before the application powers down Wi-Fi. Success and failure must have distinct cleanup paths.

Local transmission results help detect some lost requests quickly. A short response timeout remains necessary when a request appears to transmit successfully but no response is received.

## Detector behavior

- Maintain the latest sensor state independently of radio requests.
- On a valid new request, capture a snapshot, send its response promptly, and wait for the matching acknowledgment.
- On duplicate requests, resend the cached response when the transport is available. Coalesce duplicates rather than building an unbounded response queue.
- Allow bounded retries for immediate response-transmission failures. If a response transmission succeeds but the acknowledgment is missing, wait for a duplicate request or the acknowledgment deadline; do not broadcast unsolicited responses indefinitely.
- Accept a matching acknowledgment even if the response's transmission-result event has not yet been processed. Do not let that later result undo completion.
- On a valid acknowledgment, record completion and update communication health.
- Continue accepting later exchanges after a failed one so that communication can recover.
- Handle detector restarts: an acknowledgment with no matching known exchange cannot establish completion; a fresh request is needed.

For the initial two-device implementation, keep at most one active exchange plus a bounded record of recent completion for duplicate handling.

## Timing and error reporting

| Configuration | Purpose |
| --- | --- |
| Response timeout | Maximum wait for a response after a successful request transmission. |
| Request attempt limit | Bound request transmissions, including the initial attempt. |
| ACK attempt limit | Bound acknowledgment transmissions, including the initial attempt. |
| Retry delay | Avoid immediate repeated submissions during temporary failures. |
| Exchange deadline | Bound the receiver's protocol time after initialization. |
| Detector ACK deadline | Bound waiting for the final acknowledgment; cover the receiver's retry horizon. |
| Expected contact interval | Expected time between completed receiver exchanges during normal operation. |
| Contact margin | Allow startup variation, application work, retries, and timer drift. |
| Initial contact grace | Avoid declaring the receiver overdue immediately after detector startup. |

The expected contact interval must cover the actual cycle, including sleep, startup, communication, and application work. Both applications must agree on the schedule through configuration. Changing that schedule requires updating the detector's expectation.

Distinguish these errors:

- **Initialization failure:** Wi-Fi or ESP-NOW could not start.
- **Response timeout:** receiver did not obtain a valid response.
- **ACK transmission failure:** receiver obtained a response but could not successfully transmit its acknowledgment within policy.
- **Acknowledgment missing:** detector responded but did not receive an acknowledgment within policy.
- **Receiver overdue:** detector has no completed exchange within its configured contact allowance.
- **Transport failure:** submission, queue, or completion handling failed.

The detector measures contact deadlines with its own clock; clock synchronization is unnecessary. New requests may be recorded diagnostically but do not count as acknowledged delivery. Emit health-change events on transitions, not continuously on every `update()`.

Keep current sensor state, last received state, freshness, and communication health distinct. Successful communication with a sensor reporting `Unknown` clears communication errors without claiming a valid item reading.

## Proposed application-facing API shape

These names describe the intended interface, not framework-specific declarations:

```text
EspNowTransport
  begin(config)                -> result
  send(peer, bytes, token)      -> submission result
  update()                     -> dispatch receive/transmit events
  end()                        -> controlled shutdown

DetectionLink
  begin(role, config)           -> result
  setLocalItemState(state)     // detector role
  requestStatus()              // receiver role
  update()                     // pump transport and protocol
  end()                        // cancel work and stop transport

Events delivered in the application task
  ExchangeComplete { exchangeId, itemState }
  ExchangeFailed { exchangeId, reason }
  CommunicationLost { reason }
  CommunicationRestored
```

For the detector, `ExchangeComplete` occurs on accepted ACK reception. For the receiver, it occurs on successful ACK transmission completion. Link shutdown leaves Wi-Fi shutdown and sleep entry to the application. Event handlers should schedule shutdown after the current event dispatch returns.

## Power and validation plan

Deep sleep powers down the radio and restarts normal application execution on wake. Do not depend on ordinary class members surviving it. Recreate radio/link state on each wake and retain only the small amount of identity or application data actually needed.

Measure total wake energy, not only response latency. Instrument wake/startup, radio-ready, request submission, response reception, ACK completion, and sleep entry. Avoid unnecessary initialization, blocking logging, and fixed post-exchange delays in the production wake path.

Validate on two ESP32-C3 boards:

1. Successful `Present`, `Absent`, and `Unknown` exchanges, followed by immediate application-controlled sleep.
2. Lost request, response, and acknowledgment; bounded retries and correct errors.
3. Detector absent and receiver absent; receiver awake time stays bounded and detector eventually reports overdue contact.
4. Duplicate, stale, malformed, and unexpected-peer messages; no repeated application actions or false health resets.
5. Response arriving before request completion, and ACK arriving before response completion.
6. Either device restarting during an exchange; later requests recover correctly.
7. Repeated deep-sleep cycles, queue pressure, shutdown during failure, and timing drift over the intended interval.

Choose timeouts from measured latency under expected interference, with margin. The optimization target is minimum energy per successful check with an acceptable failure rate, rather than the smallest possible timeout.

## Decisions still needed before implementation

- Arduino-ESP32 or ESP-IDF, including version.
- Sleep/contact interval and desired failure-retry schedule.
- Error indication and whether the application sleeps immediately or briefly indicates an error first.
- Current-state reporting only, or latching events that occur entirely between wakeups. The initial protocol above reports current state only.
- Peer/channel provisioning and whether encrypted unicast is required.

## References

- [Espressif ESP32-C3 ESP-NOW documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c3/api-reference/network/esp_now.html): initialization, peers, callback context, and transmission-result semantics.
- [Espressif ESP32-C3 sleep documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c3/api-reference/system/sleep_modes.html): deep sleep, radio shutdown, timer wakeup, and retained memory.

Protocol choices, API shape, timing policies, and validation scenarios in this document are the proposed application design. They are not ESP-NOW delivery guarantees.
