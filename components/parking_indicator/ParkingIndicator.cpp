#include "ParkingIndicator.hpp"
#include "esp_log.h"

ParkingIndicator::ParkingIndicator(gpio_num_t led_d_pin) {
    led_strip_config_ = {};
    led_strip_config_.strip_gpio_num = led_d_pin,
    led_strip_config_.max_leds = 8,
    led_strip_config_.led_model = LED_MODEL_SK6812,
    led_strip_config_.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRBW;

    rmt_config_ = {};
    rmt_config_.clk_src = RMT_CLK_SRC_DEFAULT;
    rmt_config_.resolution_hz = 10 * 1000 * 1000;

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&led_strip_config_, &rmt_config_, &led_strip_));
    indicator_enabled_ = false;

    current_color_ = {0, 0, 0, 0};
 
}

esp_err_t ParkingIndicator::enable_indicator() {
    if (led_strip_) {
        indicator_enabled_ = true;
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t ParkingIndicator::disable_indicator() {
    if (led_strip_) {
        indicator_enabled_ = false;
        ESP_ERROR_CHECK(led_strip_clear(led_strip_));
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t ParkingIndicator::clear_indicator() {
    if (led_strip_) {
        ESP_ERROR_CHECK(led_strip_clear(led_strip_));
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t ParkingIndicator::set_indicator_color(LEDColor *color) {
    if (led_strip_ && color) {
        current_color_ = *color;
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t ParkingIndicator::write_indicator_level(float level) {
    if (!led_strip_ || level < 0.0f || level > 100.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    // Each LED is one 100/max_leds step (12.5% for 8 LEDs). LED i lights once
    // level reaches (i + 1) steps; below one step the whole bar lights.
    const uint32_t num_leds = led_strip_config_.max_leds;
    const float step = 100.0f / (float)num_leds;
    const bool below_first_step = level < step;

    for (uint32_t i = 0; i < num_leds; i++) {
        const bool on = below_first_step || level >= (float)(i + 1) * step;
        esp_err_t err = on
            ? led_strip_set_pixel_rgbw(led_strip_, i, current_color_.red, current_color_.green,
                                       current_color_.blue, current_color_.white)
            : led_strip_set_pixel_rgbw(led_strip_, i, 0, 0, 0, 0);
        if (err != ESP_OK) {
            return err;
        }
    }
    return led_strip_refresh(led_strip_);
}

ParkingIndicator::~ParkingIndicator() {
    if (led_strip_) {
        // Add code to delete the LED strip device members
    }
}