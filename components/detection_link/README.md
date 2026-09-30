# detection_link

ESP-NOW request/response/acknowledge link between an always-powered **detector** and a
deep-sleeping **receiver**, implementing `esp-now-detection-link-design.md` at the repo root.

| File | Contents |
| --- | --- |
| `Protocol.hpp` | Wire format (little-endian, 8-byte header, 9-byte response), `encode()` / `decode()` |
| `Transport.hpp` | `Transport` interface, `TransportConfig`, `SubmitResult`, `TransportStats` |
| `EspNowTransport.hpp` | ESP-NOW implementation of `Transport` |
| `DetectionLink.hpp` | Protocol state machine for both roles, config, events, health, diagnostics |
| `ExchangeId.hpp` | Exchange IDs that are not reused across deep sleep (`retained_exchange_id()`) |

## Using it

Wi-Fi is the application's job: start it in STA mode on the agreed channel before `begin()`, and
stop it after `end()`.

Receiver (one exchange per wake):

```cpp
EspNowTransport transport;
DetectionLink link(transport);
LinkConfig config;
config.transport.channel = CHANNEL;
config.transport.peer = DETECTOR_MAC;
config.exchange_id_source = retained_exchange_id;
if (link.begin(Role::Receiver, config) == ESP_OK && link.request_status() == ESP_OK) {
    LinkEvent event;
    bool done = false;
    while (!done) {
        link.update();
        while (link.poll_event(event)) {
            if (event.type == LinkEventType::ExchangeComplete) { /* use event.item_state */ done = true; }
            if (event.type == LinkEventType::ExchangeFailed)   { /* event.reason */ done = true; }
        }
        vTaskDelay(1);
    }
}
link.end();
// stop Wi-Fi, set the timer wakeup, esp_deep_sleep_start()
```

Detector (runs forever): call `set_local_item_state()` whenever the sensor reading changes, and
`update()` / `poll_event()` from the main loop. `CommunicationLost { ReceiverOverdue }` and
`CommunicationRestored` report contact health.

## Choices the design left open

- **Transport injection.** `DetectionLink` holds a `Transport&` rather than an `EspNowTransport`
  member, so the protocol can be tested against a scripted transport and a manual clock.
- **Events are polled.** `poll_event()` drains a bounded queue, which keeps handlers out of the
  protocol code, so ending the link from event handling is always safe.
- **Exchange IDs** are a 32-bit counter in RTC memory, seeded from `esp_random()` after power-on.
- **Duplicate requests** for an exchange that already completed or was abandoned are ignored
  instead of answered.
- **Completion watchdog.** If ESP-NOW never reports a send result, the transport reports failure
  after `send_completion_timeout_ms` and discards the late result if it turns up.
- **Detector overdue timer** starts at `initial_contact_grace_ms` after `begin()`, then runs
  `expected_contact_interval_ms + contact_margin_ms` from each completed exchange.
- **Receiver health** only lasts for one link session, because deep sleep resets it.
- Unencrypted unicast, 32-byte packet limit, 8-slot receive and event queues.

## Tests

- `test_apps/common/` — protocol, exchange IDs, both link roles, and a receiver and detector
  running against each other over a simulated radio with scripted losses. Runs on the host and
  on the ESP32-C3 (and in QEMU).
- `test_apps/host/` — `run_host_tests.sh`: the common tests, plus `EspNowTransport` against the
  CMock ESP-NOW mock.
- `test_apps/target/` — `run_qemu_tests.sh` / `.ps1` run everything except `[hw]` in QEMU.
  `run_hw_tests.ps1` builds, flashes and runs every test, including the `[hw]` radio tests, on an
  ESP32-C3 plugged into the PC (one board, no peer needed). It finds the board by USB vendor ID
  and picks the console to match. Boards using the chip's built-in USB port need the
  USB-Serial/JTAG console (`sdkconfig.usb_serial_jtag`), or the Unity menu never sees its input.
  CI runs it in the `hardware-tests` job on the runner PC.

The two-board validation plan in the design document still needs real hardware.
