#include "MB1013.hpp"
#include "esp_log.h"
#include "esp_check.h"
#include "soc/soc_caps.h"

#if SOC_HAS(RMT)
static constexpr rmt_clock_source_t RMT_CLK_SRC = RMT_CLK_SRC_DEFAULT;
#else
// Host (linux) builds have no RMT clock tree; the driver there is a test mock
static constexpr rmt_clock_source_t RMT_CLK_SRC = 0;
#endif

MB1013::MB1013(gpio_num_t gpio_num, size_t rmt_mem_symbols)
    : m_gpio_num(gpio_num), m_rmt_mem_symbols(rmt_mem_symbols), m_rx_channel(nullptr), m_rx_queue(nullptr), m_rx_symbols{} {}

MB1013::~MB1013() {
    deinit();
}

bool MB1013::on_recv_done(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t* edata, void* user_ctx) {
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t queue = static_cast<QueueHandle_t>(user_ctx);
    xQueueSendFromISR(queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

esp_err_t MB1013::init() {
    // Refuse to re-initialize so an existing channel is never overwritten or leaked
    ESP_RETURN_ON_FALSE(!m_rx_channel && !m_rx_queue, ESP_ERR_INVALID_STATE, TAG, "Already initialized");

    ESP_LOGI(TAG, "Initializing MB1013 on GPIO %d using RMT", m_gpio_num);

    m_rx_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    ESP_RETURN_ON_FALSE(m_rx_queue, ESP_ERR_NO_MEM, TAG, "Failed to create RX queue");

    rmt_rx_channel_config_t rx_chan_config = {};
    rx_chan_config.gpio_num = m_gpio_num;
    rx_chan_config.clk_src = RMT_CLK_SRC;
    rx_chan_config.resolution_hz = RMT_RESOLUTION_HZ;
    rx_chan_config.mem_block_symbols = m_rmt_mem_symbols;
    rx_chan_config.flags.invert_in = 0; // Look for an active-high pulse

    rmt_rx_event_callbacks_t cbs = {};
    cbs.on_recv_done = on_recv_done;

    esp_err_t ret = rmt_new_rx_channel(&rx_chan_config, &m_rx_channel);
    if (ret == ESP_OK) {
        ret = rmt_rx_register_event_callbacks(m_rx_channel, &cbs, m_rx_queue);
    }
    if (ret == ESP_OK) {
        ret = rmt_enable(m_rx_channel);
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize RMT RX channel: %s", esp_err_to_name(ret));
        // Undo the partial setup so init() can be retried; the channel was never enabled
        if (m_rx_channel) {
            rmt_del_channel(m_rx_channel);
            m_rx_channel = nullptr;
        }
        vQueueDelete(m_rx_queue);
        m_rx_queue = nullptr;
    }

    return ret;
}

int32_t MB1013::read_range_mm() {
    if (!m_rx_channel) {
        ESP_LOGE(TAG, "Driver not initialized");
        return mb1013::RANGE_ERROR;
    }

    // Pulses shorter than 1 us are treated as glitches. Reception ends once the line
    // holds one level for 10 ms, which is longer than the MB1013's maximum ~5 ms pulse
    // but shorter than the ~95 ms low gap between measurements.
    rmt_receive_config_t receive_config = {};
    receive_config.signal_range_min_ns = 1000;
    receive_config.signal_range_max_ns = 10 * 1000 * 1000;

    xQueueReset(m_rx_queue);

    // Arm the hardware capture unit
    if (rmt_receive(m_rx_channel, m_rx_symbols, sizeof(m_rx_symbols), &receive_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to arm RMT receiver");
        return mb1013::RANGE_ERROR;
    }

    // The sensor fires a measurement every ~100 ms, so one should complete within 150 ms
    rmt_rx_done_event_data_t rx_data;
    if (xQueueReceive(m_rx_queue, &rx_data, pdMS_TO_TICKS(150)) != pdTRUE) {
        ESP_LOGW(TAG, "Timed out waiting for a pulse");
        // Cancel the pending receive so the next call can arm the channel again
        rmt_disable(m_rx_channel);
        rmt_enable(m_rx_channel);
        return mb1013::RANGE_UNRESOLVED;
    }

    int32_t range_mm = mb1013::decode_pulse_mm(rx_data.received_symbols, rx_data.num_symbols);
    if (range_mm == mb1013::RANGE_UNRESOLVED) {
        ESP_LOGW(TAG, "Pulse longer than the receive window");
    }
    return range_mm;
}

void MB1013::deinit() {
    if (m_rx_channel) {
        rmt_disable(m_rx_channel);
        rmt_del_channel(m_rx_channel);
        m_rx_channel = nullptr;
        ESP_LOGI(TAG, "MB1013 Driver uninstalled cleanly");
    }
    if (m_rx_queue) {
        vQueueDelete(m_rx_queue);
        m_rx_queue = nullptr;
    }
}
