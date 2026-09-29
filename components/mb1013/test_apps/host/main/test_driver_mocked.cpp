// Tier 2: driver logic against CMock mocks of the RMT driver and FreeRTOS queues

#include <cstring>
#include <type_traits>
#include "unity.h"
#include "MB1013.hpp"

extern "C" {
#include "Mockrmt_rx.h"
#include "Mockrmt_common.h"
#include "Mockqueue.h"
}

using namespace mb1013;

static_assert(!std::is_copy_constructible_v<MB1013>, "MB1013 must not be copyable");
static_assert(!std::is_copy_assignable_v<MB1013>, "MB1013 must not be copy-assignable");

static constexpr gpio_num_t TEST_GPIO = GPIO_NUM_4;
static rmt_channel_handle_t const FAKE_CHANNEL = reinterpret_cast<rmt_channel_handle_t>(0x1000);
static QueueHandle_t const FAKE_QUEUE = reinterpret_cast<QueueHandle_t>(0x2000);

// State captured by the stubs below
static rmt_rx_channel_config_t s_channel_config;
static rmt_rx_done_callback_t s_on_recv_done;
static void* s_callback_ctx;
static rmt_receive_config_t s_receive_config;
static void* s_receive_buffer;
static size_t s_receive_buffer_size;
static rmt_rx_done_event_data_t s_event_to_deliver;
static rmt_rx_done_event_data_t s_event_sent_from_isr;
static BaseType_t s_isr_woken;

extern "C" void setUp(void)
{
    Mockrmt_rx_Init();
    Mockrmt_common_Init();
    Mockqueue_Init();
    s_on_recv_done = nullptr;
    s_callback_ctx = nullptr;
}

extern "C" void tearDown(void)
{
    Mockrmt_rx_Verify();
    Mockrmt_common_Verify();
    Mockqueue_Verify();
    Mockrmt_rx_Destroy();
    Mockrmt_common_Destroy();
    Mockqueue_Destroy();
}

static esp_err_t stub_new_rx_channel(const rmt_rx_channel_config_t* config, rmt_channel_handle_t* ret_chan, int calls)
{
    s_channel_config = *config;
    *ret_chan = FAKE_CHANNEL;
    return ESP_OK;
}

static esp_err_t stub_register_callbacks(rmt_channel_handle_t channel, const rmt_rx_event_callbacks_t* cbs, void* user_data, int calls)
{
    TEST_ASSERT_EQUAL_PTR(FAKE_CHANNEL, channel);
    s_on_recv_done = cbs->on_recv_done;
    s_callback_ctx = user_data;
    return ESP_OK;
}

static esp_err_t stub_receive(rmt_channel_handle_t channel, void* buffer, size_t buffer_size, const rmt_receive_config_t* config, int calls)
{
    TEST_ASSERT_EQUAL_PTR(FAKE_CHANNEL, channel);
    s_receive_buffer = buffer;
    s_receive_buffer_size = buffer_size;
    s_receive_config = *config;
    return ESP_OK;
}

static BaseType_t stub_queue_receive_event(QueueHandle_t queue, void* buffer, TickType_t ticks, int calls)
{
    TEST_ASSERT_EQUAL_PTR(FAKE_QUEUE, queue);
    TEST_ASSERT_EQUAL_UINT32(pdMS_TO_TICKS(150), ticks);
    std::memcpy(buffer, &s_event_to_deliver, sizeof(s_event_to_deliver));
    return pdTRUE;
}

static BaseType_t stub_queue_send_from_isr(QueueHandle_t queue, const void* item, BaseType_t* woken, BaseType_t position, int calls)
{
    TEST_ASSERT_EQUAL_PTR(FAKE_QUEUE, queue);
    std::memcpy(&s_event_sent_from_isr, item, sizeof(s_event_sent_from_isr));
    *woken = s_isr_woken;
    return pdTRUE;
}

static void expect_init_ok()
{
    xQueueGenericCreate_ExpectAndReturn(1, sizeof(rmt_rx_done_event_data_t), queueQUEUE_TYPE_BASE, FAKE_QUEUE);
    rmt_new_rx_channel_Stub(stub_new_rx_channel);
    rmt_rx_register_event_callbacks_Stub(stub_register_callbacks);
    rmt_enable_ExpectAndReturn(FAKE_CHANNEL, ESP_OK);
}

static void expect_deinit()
{
    rmt_disable_ExpectAndReturn(FAKE_CHANNEL, ESP_OK);
    rmt_del_channel_ExpectAndReturn(FAKE_CHANNEL, ESP_OK);
    vQueueDelete_Expect(FAKE_QUEUE);
}

// Arm a read that delivers `event` through the queue
static void expect_read_delivering(const rmt_rx_done_event_data_t& event)
{
    s_event_to_deliver = event;
    xQueueGenericReset_ExpectAndReturn(FAKE_QUEUE, pdFALSE, pdPASS);
    rmt_receive_Stub(stub_receive);
    xQueueReceive_Stub(stub_queue_receive_event);
}

// ---- init() ----

TEST_CASE("mocked: init configures and enables the RX channel", "[mb1013][host]")
{
    MB1013 sensor(TEST_GPIO, 48);
    expect_init_ok();
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());

    TEST_ASSERT_EQUAL(TEST_GPIO, s_channel_config.gpio_num);
    TEST_ASSERT_EQUAL_UINT32(1000000, s_channel_config.resolution_hz);
    TEST_ASSERT_EQUAL_UINT32(48, s_channel_config.mem_block_symbols);
    TEST_ASSERT_EQUAL_UINT32(0, s_channel_config.flags.invert_in);
    TEST_ASSERT_NOT_NULL(s_on_recv_done);
    TEST_ASSERT_EQUAL_PTR(FAKE_QUEUE, s_callback_ctx);

    expect_deinit();
    sensor.deinit();
}

TEST_CASE("mocked: init fails with no memory when the queue cannot be created", "[mb1013][host]")
{
    MB1013 sensor(TEST_GPIO);
    xQueueGenericCreate_ExpectAndReturn(1, sizeof(rmt_rx_done_event_data_t), queueQUEUE_TYPE_BASE, nullptr);
    TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM, sensor.init());
    // No RMT calls are expected, and deinit has nothing to release
    sensor.deinit();
}

TEST_CASE("mocked: init passes through a channel creation error and frees the queue", "[mb1013][host]")
{
    MB1013 sensor(TEST_GPIO);
    xQueueGenericCreate_ExpectAndReturn(1, sizeof(rmt_rx_done_event_data_t), queueQUEUE_TYPE_BASE, FAKE_QUEUE);
    rmt_new_rx_channel_ExpectAnyArgsAndReturn(ESP_ERR_NOT_FOUND);
    vQueueDelete_Expect(FAKE_QUEUE);
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, sensor.init());
    sensor.deinit();
}

TEST_CASE("mocked: init cleans up when callback registration fails", "[mb1013][host]")
{
    MB1013 sensor(TEST_GPIO);
    xQueueGenericCreate_ExpectAndReturn(1, sizeof(rmt_rx_done_event_data_t), queueQUEUE_TYPE_BASE, FAKE_QUEUE);
    rmt_new_rx_channel_Stub(stub_new_rx_channel);
    rmt_rx_register_event_callbacks_ExpectAnyArgsAndReturn(ESP_ERR_INVALID_STATE);
    rmt_del_channel_ExpectAndReturn(FAKE_CHANNEL, ESP_OK);
    vQueueDelete_Expect(FAKE_QUEUE);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, sensor.init());
    sensor.deinit();
}

TEST_CASE("mocked: init cleans up when enabling the channel fails", "[mb1013][host]")
{
    MB1013 sensor(TEST_GPIO);
    xQueueGenericCreate_ExpectAndReturn(1, sizeof(rmt_rx_done_event_data_t), queueQUEUE_TYPE_BASE, FAKE_QUEUE);
    rmt_new_rx_channel_Stub(stub_new_rx_channel);
    rmt_rx_register_event_callbacks_Stub(stub_register_callbacks);
    rmt_enable_ExpectAndReturn(FAKE_CHANNEL, ESP_FAIL);
    rmt_del_channel_ExpectAndReturn(FAKE_CHANNEL, ESP_OK);
    vQueueDelete_Expect(FAKE_QUEUE);
    TEST_ASSERT_EQUAL(ESP_FAIL, sensor.init());
    sensor.deinit();
}

TEST_CASE("mocked: init after a failed init can succeed", "[mb1013][host]")
{
    MB1013 sensor(TEST_GPIO);
    xQueueGenericCreate_ExpectAndReturn(1, sizeof(rmt_rx_done_event_data_t), queueQUEUE_TYPE_BASE, FAKE_QUEUE);
    rmt_new_rx_channel_ExpectAnyArgsAndReturn(ESP_ERR_NOT_FOUND);
    vQueueDelete_Expect(FAKE_QUEUE);
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, sensor.init());

    expect_init_ok();
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    expect_deinit();
    sensor.deinit();
}

TEST_CASE("mocked: second init returns invalid state and leaves the channel alone", "[mb1013][host]")
{
    MB1013 sensor(TEST_GPIO);
    expect_init_ok();
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());

    // No further queue or RMT calls may happen
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, sensor.init());

    // The original channel still works
    rmt_rx_done_event_data_t event = {};
    rmt_symbol_word_t symbols[1] = {};
    symbols[0].level0 = 1;
    symbols[0].duration0 = 1234;
    event.received_symbols = symbols;
    event.num_symbols = 1;
    expect_read_delivering(event);
    TEST_ASSERT_EQUAL_INT32(1234, sensor.read_range_mm());

    expect_deinit();
    sensor.deinit();
}

// ---- read_range_mm() ----

TEST_CASE("mocked: read before init reports an error without touching the driver", "[mb1013][host]")
{
    MB1013 sensor(TEST_GPIO);
    TEST_ASSERT_EQUAL_INT32(RANGE_ERROR, sensor.read_range_mm());
}

TEST_CASE("mocked: read arms the receiver and decodes the delivered pulse", "[mb1013][host]")
{
    MB1013 sensor(TEST_GPIO);
    expect_init_ok();
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());

    rmt_symbol_word_t symbols[1] = {};
    symbols[0].level0 = 1;
    symbols[0].duration0 = 2750;
    rmt_rx_done_event_data_t event = {};
    event.received_symbols = symbols;
    event.num_symbols = 1;
    expect_read_delivering(event);

    TEST_ASSERT_EQUAL_INT32(2750, sensor.read_range_mm());
    TEST_ASSERT_EQUAL_UINT32(1000, s_receive_config.signal_range_min_ns);
    TEST_ASSERT_EQUAL_UINT32(10 * 1000 * 1000, s_receive_config.signal_range_max_ns);
    TEST_ASSERT_NOT_NULL(s_receive_buffer);
    TEST_ASSERT_EQUAL_UINT32(64 * sizeof(rmt_symbol_word_t), s_receive_buffer_size);

    expect_deinit();
    sensor.deinit();
}

TEST_CASE("mocked: read returns each decode result unchanged", "[mb1013][host]")
{
    MB1013 sensor(TEST_GPIO);
    expect_init_ok();
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());

    rmt_symbol_word_t symbols[1] = {};
    rmt_rx_done_event_data_t event = {};
    event.received_symbols = symbols;
    event.num_symbols = 1;

    // Over-range: high level with no end
    symbols[0].level0 = 1;
    symbols[0].duration0 = 0;
    expect_read_delivering(event);
    TEST_ASSERT_EQUAL_INT32(RANGE_UNRESOLVED, sensor.read_range_mm());

    // Armed mid-pulse: capture starts low
    symbols[0].level0 = 0;
    expect_read_delivering(event);
    TEST_ASSERT_EQUAL_INT32(RANGE_IN_PROGRESS, sensor.read_range_mm());

    expect_deinit();
    sensor.deinit();
}

TEST_CASE("mocked: read reports an error when the receiver cannot be armed", "[mb1013][host]")
{
    MB1013 sensor(TEST_GPIO);
    expect_init_ok();
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());

    xQueueGenericReset_ExpectAndReturn(FAKE_QUEUE, pdFALSE, pdPASS);
    rmt_receive_ExpectAnyArgsAndReturn(ESP_ERR_INVALID_STATE);
    // xQueueReceive must not be called
    TEST_ASSERT_EQUAL_INT32(RANGE_ERROR, sensor.read_range_mm());

    expect_deinit();
    sensor.deinit();
}

TEST_CASE("mocked: timeout cancels the receive and the next read works", "[mb1013][host]")
{
    MB1013 sensor(TEST_GPIO);
    expect_init_ok();
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());

    xQueueGenericReset_ExpectAndReturn(FAKE_QUEUE, pdFALSE, pdPASS);
    rmt_receive_ExpectAnyArgsAndReturn(ESP_OK);
    xQueueReceive_ExpectAnyArgsAndReturn(pdFALSE);
    rmt_disable_ExpectAndReturn(FAKE_CHANNEL, ESP_OK);
    rmt_enable_ExpectAndReturn(FAKE_CHANNEL, ESP_OK);
    TEST_ASSERT_EQUAL_INT32(RANGE_UNRESOLVED, sensor.read_range_mm());

    rmt_symbol_word_t symbols[1] = {};
    symbols[0].level0 = 1;
    symbols[0].duration0 = 900;
    rmt_rx_done_event_data_t event = {};
    event.received_symbols = symbols;
    event.num_symbols = 1;
    expect_read_delivering(event);
    TEST_ASSERT_EQUAL_INT32(900, sensor.read_range_mm());

    expect_deinit();
    sensor.deinit();
}

// ---- on_recv_done callback ----

TEST_CASE("mocked: callback forwards the event and reports whether a task woke", "[mb1013][host]")
{
    MB1013 sensor(TEST_GPIO);
    expect_init_ok();
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    TEST_ASSERT_NOT_NULL(s_on_recv_done);

    rmt_symbol_word_t symbols[1] = {};
    rmt_rx_done_event_data_t event = {};
    event.received_symbols = symbols;
    event.num_symbols = 1;

    xQueueGenericSendFromISR_Stub(stub_queue_send_from_isr);

    s_isr_woken = pdFALSE;
    TEST_ASSERT_FALSE(s_on_recv_done(FAKE_CHANNEL, &event, s_callback_ctx));
    TEST_ASSERT_EQUAL_PTR(symbols, s_event_sent_from_isr.received_symbols);
    TEST_ASSERT_EQUAL_UINT32(1, s_event_sent_from_isr.num_symbols);

    s_isr_woken = pdTRUE;
    TEST_ASSERT_TRUE(s_on_recv_done(FAKE_CHANNEL, &event, s_callback_ctx));

    expect_deinit();
    sensor.deinit();
}

// ---- lifecycle ----

TEST_CASE("mocked: deinit twice or without init does nothing", "[mb1013][host]")
{
    MB1013 never_initialized(TEST_GPIO);
    never_initialized.deinit();

    MB1013 sensor(TEST_GPIO);
    expect_init_ok();
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    expect_deinit();
    sensor.deinit();
    sensor.deinit();
}

TEST_CASE("mocked: destructor releases the channel and queue", "[mb1013][host]")
{
    expect_init_ok();
    expect_deinit();
    {
        MB1013 sensor(TEST_GPIO);
        TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    }
}

TEST_CASE("mocked: read after deinit reports an error", "[mb1013][host]")
{
    MB1013 sensor(TEST_GPIO);
    expect_init_ok();
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    expect_deinit();
    sensor.deinit();
    TEST_ASSERT_EQUAL_INT32(RANGE_ERROR, sensor.read_range_mm());
}
