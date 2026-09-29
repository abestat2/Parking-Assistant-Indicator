#pragma once

#include "driver/rmt_rx.h"
#include "hal/gpio_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "MB1013_decode.hpp"
#include <cstdint>
#include <cstddef>

class MB1013 {
public:
    /**
     * @brief Construct a new MB1013 object using the RMT peripheral
     * @param gpio_num The GPIO pin connected to the MB1013 PW output
     * @param rmt_mem_symbols Memory size for RMT ring buffer (default 64)
     */
    MB1013(gpio_num_t gpio_num, size_t rmt_mem_symbols = 64);

    /**
     * @brief Destroy the MB1013 object and clean up RMT resources
     */
    ~MB1013();

    // Prevent copies to avoid accidental resource or channel duplication
    MB1013(const MB1013&) = delete;
    MB1013& operator=(const MB1013&) = delete;

    /**
     * @brief Initialize the RMT RX peripheral channel
     * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if already initialized
     *         (the existing channel is left untouched), other error code on failure
     */
    esp_err_t init();

    /**
     * @brief Read the current distance from the sensor
     * @return int32_t Target range in millimeters (>= 0), or one of the negative mb1013::RANGE_* codes:
     *         RANGE_UNRESOLVED if no complete pulse was measured, RANGE_IN_PROGRESS if the read began
     *         partway through a pulse (call again), RANGE_ERROR if the driver is not initialized or failed
     */
    int32_t read_range_mm();

    /**
     * @brief Deinitialize and release the RMT channel resources
     */
    void deinit();

private:
    static bool on_recv_done(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t* edata, void* user_ctx);

    static constexpr const char* TAG = "MB1013";
    // 1 MHz gives 1 tick = 1 us, and the MB1013 PW output is 1 us per mm
    static constexpr uint32_t RMT_RESOLUTION_HZ = 1000000;
    static constexpr size_t RX_BUFFER_SYMBOLS = 64;

    gpio_num_t m_gpio_num;
    size_t m_rmt_mem_symbols;
    rmt_channel_handle_t m_rx_channel;
    QueueHandle_t m_rx_queue;
    // The driver writes into this buffer asynchronously, so it must outlive each read
    rmt_symbol_word_t m_rx_symbols[RX_BUFFER_SYMBOLS];
};
