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

## Distance limits

The LED bar works between a stop distance (one LED lit, closer is all red) and a full-bar distance
(the whole bar is green). The menuconfig values (**Parking guidance**) are the defaults. The
detector can replace them: its reply to a request carries the limits set for this stall on the Pi's
web page, and the indicator saves them in NVS (namespace `indicator`, keys `min_mm` and `max_mm`)
and uses them from then on, through deep sleep and power loss. Limits that are out of range (30 to
4000 mm, stop below full-bar) are ignored and logged. The values in use are logged at power-up and
sent to the detector with every request.
