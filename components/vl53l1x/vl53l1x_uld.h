#pragma once

// Access to the ST ULD's default configuration table, defined in vl53l1x_uld.c

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// The table covers registers 0x2D through 0x87; vl53l1x_uld.c checks this at compile time
#define VL53L1X_DEFAULT_CONFIGURATION_START 0x2D
#define VL53L1X_DEFAULT_CONFIGURATION_SIZE (0x87 - 0x2D + 1)

const uint8_t* vl53l1x_uld_default_configuration(void);

#ifdef __cplusplus
}
#endif
