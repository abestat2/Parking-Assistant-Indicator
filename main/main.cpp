#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_log.h"
#include "MB1013.hpp"


extern "C" void app_main(void)
{
    MB1013 sonar(GPIO_NUM_18);

    ESP_ERROR_CHECK(sonar.init());

    uint8_t baseMac[6];
    if(esp_read_mac(baseMac, ESP_MAC_WIFI_STA) == ESP_OK) {
        ESP_LOGI("MAC", "MAC address: %02x:%02x:%02x:%02x:%02x:%02x",
                 baseMac[0], baseMac[1], baseMac[2],
                 baseMac[3], baseMac[4], baseMac[5]);
    }
    else {
        ESP_LOGE("MAC", "Failed to read MAC address");
    }

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}