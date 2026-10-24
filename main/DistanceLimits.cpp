#include "DistanceLimits.hpp"
#include "nvs.h"
#include "sdkconfig.h"

namespace distance_limits {

static const char* const NAMESPACE = "indicator";
static const char* const MIN_KEY = "min_mm";
static const char* const MAX_KEY = "max_mm";

detection_link::RangeLimits load()
{
    const detection_link::RangeLimits configured = {CONFIG_INDICATOR_STOP_DISTANCE_MM,
                                                    CONFIG_INDICATOR_GUIDE_DISTANCE_MM};
    nvs_handle_t handle;
    if (nvs_open(NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return configured;
    }
    detection_link::RangeLimits saved;
    const bool found = nvs_get_u16(handle, MIN_KEY, &saved.min_mm) == ESP_OK &&
                       nvs_get_u16(handle, MAX_KEY, &saved.max_mm) == ESP_OK;
    nvs_close(handle);
    return found && valid(saved) ? saved : configured;
}

esp_err_t save(detection_link::RangeLimits limits)
{
    if (!valid(limits)) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_u16(handle, MIN_KEY, limits.min_mm);
    if (err == ESP_OK) {
        err = nvs_set_u16(handle, MAX_KEY, limits.max_mm);
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

} // namespace distance_limits
