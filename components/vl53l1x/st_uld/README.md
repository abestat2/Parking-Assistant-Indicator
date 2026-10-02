# ST VL53L1X Ultra Lite Driver (ULD)

Unmodified files from ST's VL53L1X ULD API, STSW-IMG009 version 3.5.4
(https://www.st.com/en/embedded-software/stsw-img009.html), under ST's license in `LICENSE.txt`
(BSD 3-clause, SLA0103). User manual: UM2510.

Copied from https://github.com/gschorcht/riot_st_vl53l1x_uld_api, which repackages the ST
release for RIOT-OS. Only the ST files were taken; that repository's RIOT platform layer was not.

- `core/` — the driver. `VL53L1X_api.c` is compiled through `../vl53l1x_uld.c`.
  `VL53L1X_calibration.c` is kept for reference but not built: like `VL53L1X_SensorInit()`, it
  waits for measurements with unbounded loops.
- `platform/` — ST's declarations of the platform functions. They are implemented for
  `I2cDevice` in `../vl53l1x_platform.cpp`.

To update, replace these files with a newer release and run the host tests, which exercise the
ULD against a simulated sensor.
