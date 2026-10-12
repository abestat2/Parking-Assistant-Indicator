#pragma once

#include "hal/gpio_types.h"
#include "led_strip.h"

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint8_t white;
} LEDColor;

class ParkingIndicator {
private:
    led_strip_config_t led_strip_config_;
    led_strip_rmt_config_t rmt_config_;
    led_strip_handle_t led_strip_;
    LEDColor current_color_;

    bool indicator_enabled_;
    static constexpr const char* TAG = "LED_STRIP";

protected:
    
public:
    ParkingIndicator(gpio_num_t led_d_pin);
    // Owns the LED strip device, which the destructor deletes
    ParkingIndicator(const ParkingIndicator&) = delete;
    ParkingIndicator& operator=(const ParkingIndicator&) = delete;
    esp_err_t enable_indicator();
    esp_err_t disable_indicator();
    esp_err_t clear_indicator();
    esp_err_t set_indicator_color(LEDColor *color);
    esp_err_t write_indicator_level(float level);
    ~ParkingIndicator();
};
