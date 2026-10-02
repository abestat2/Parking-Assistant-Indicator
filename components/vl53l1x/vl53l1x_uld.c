// Builds the unmodified ST ULD core in this translation unit.
//
// VL53L1X_SensorInit() in the ULD waits for its first measurement with an unbounded loop, which
// spins forever if I2C fails. The VL53L1X class reimplements that sequence with a timeout, and
// needs the ULD's default configuration table to do so. The table is file-static in
// VL53L1X_api.c, so including the source here gives access to it without editing ST's files.

#include "VL53L1X_api.c"
#include "vl53l1x_uld.h"

_Static_assert(sizeof(VL51L1X_DEFAULT_CONFIGURATION) == VL53L1X_DEFAULT_CONFIGURATION_SIZE,
               "ULD default configuration no longer covers registers 0x2D to 0x87");

const uint8_t* vl53l1x_uld_default_configuration(void)
{
    return VL51L1X_DEFAULT_CONFIGURATION;
}
