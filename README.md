# Parking-Assistant-Indicator
[![Tests](https://github.com/abestat2/Parking-Assistant-Indicator/actions/workflows/tests.yml/badge.svg?branch=main)](https://github.com/abestat2/Parking-Assistant-Indicator/actions/workflows/tests.yml) [![Coverage](https://github.com/abestat2/Parking-Assistant-Indicator/raw/badges/coverage.svg)](https://github.com/abestat2/Parking-Assistant-Indicator/actions/workflows/tests.yml)

Indicator half of device that helps park your car in a garage at the correct distance

## Getting the code

The ESP-NOW detection link (`detection_link`) lives in the [ESP-NOW-Transport](https://github.com/abestat2/ESP-NOW-Transport)
repo, included as a submodule at `external/ESP-NOW-Transport`. Clone with submodules:

```sh
git clone --recursive https://github.com/abestat2/Parking-Assistant-Indicator.git
# or, in an existing clone:
git submodule update --init
```
