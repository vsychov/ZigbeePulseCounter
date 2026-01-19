# Honeywell BK-G4Mt build (reed pulse)

Hardware notes for using this firmware with a Honeywell BK-G4Mt gas meter that exposes a reed-switch pulse output.

## Photos

- Before LED removal vs after: `images/bk-g4mt/bk-g4mt-board-ws2812-before.jpg`, `images/bk-g4mt/bk-g4mt-board-ws2812-after.jpg`.
- Node assembly: `images/bk-g4mt/bk-g4mt-assembled.jpg`, `images/bk-g4mt/bk-g4mt-internals.jpg`.
- Installed on the meter: `images/bk-g4mt/bk-g4mt-installed-1.jpg`, `images/bk-g4mt/bk-g4mt-installed-2.jpg`.

## Parts used

- ESP32-H2 board (Waveshare ESP32-H2 Mini tested): https://www.amazon.de/dp/B0FPCS299V.
- Reed switch (Meder MK06-5-B): https://www.tme.eu/de/details/mk65b/reed-relais/meder/mk06-5-b/
- 2x14500 LiFePO4 cells and holder (https://www.amazon.de/dp/B08JV9S4XY) (wired in parallel for ~3.2 V), or USB/5 V if running mains-powered.
- Voltage divider for battery sense (if enabled): 300 k / 100 k resistors, 100 nF capacitor on ADC input.
- Light-gauge wires, small screw terminals or solder lugs, heat-shrink.

## Board prep

- Remove the onboard WS2812 LED and its transistor on the Waveshare board to cut idle current (see photos above).
- If you do not need battery sensing, you can leave the ADC divider unpopulated and disable `CONFIG_BATTERY_ADC_ENABLE`.

## Wiring

Default pins in the repo builds use `PULSE_GPIO=10` with pull-up enabled. The reed contact closes to GND.

```
PULSE_GPIO ----+----o/ o---- GND
               |
               +--[internal pull-up]
```

Battery sense (optional):

```
              +3.3V
                |
               [300k]
                |
                +-------- GPIO_1 --------+
                |                        |
              [100nF]                  [100k]
                |                        |
               GND                      GND
                |
                +-------- GPIO_10 -------+
                |
              [ REED SWITCH ]
                |
               GND
```

Battery holder tweak for parallel cells (to keep ~3.2 V instead of ~6.4 V):
- Cut the bottom contact plate so the cells are isolated.
- Add a small spacer so the cells cannot touch.
- Solder wires to each cell and route them to the top contacts to achieve parallel wiring.

## Firmware configuration tips

- Set `CONFIG_ZB_VARIANT_GAS` for this meter type.
- Choose `CONFIG_PULSE_GPIO` to match your wiring (defaults to 10 in the sample configs).
- Set `CONFIG_PULSE_DEBOUNCE_MS` and `CONFIG_PULSE_MIN_WIDTH_MS` for your reed (50 ms and 10 ms are conservative).
- Set `CONFIG_PULSE_PER_UNIT_NUMERATOR` to your meter’s pulse rate (BK-G4Mt reed output is typically 0.01 m3/pulse ⇒ 100 pulses per m3).
- If powered from mains/USB, disable `CONFIG_SLEEPY_END_DEVICE` and `CONFIG_BATTERY_ADC_ENABLE`. For battery builds, leave them enabled.

After adjusting Kconfig (via `idf.py menuconfig` or an override file), build and flash:

```bash
idf.py -DSDKCONFIG_DEFAULTS="sdkconfig.defaults" set-target esp32h2 build flash monitor
```
