# Zigbee Pulse Meter (ESP32H2)

Firmware for ESP32H2: pulse counter (dry contact) that exposes Zigbee Simple Metering (0x0702) and Power Configuration (0x0001). Pulse settings are controlled through manufacturer-specific cluster 0xFD10. Meter type and measurement units are chosen at build time.

**Full code is generated with ChatGPT 5.2 Pro + Codex gpt-5.1-codex-max xhigh.**

## Background and low-power notes

- Off-the-shelf pulse counters exist (for example https://shop.smarthome-europe.com/en/domotique/5499-lixee-zigbee-30-pulse-meter-water-gas-jeedom-and-home-assistant-compatible-3770014375155.html and https://www.amazon.de/dp/B0CGVB6LGC with a swappable pulse sensor), but this firmware targets a DIY build.
- ESP32-C6 was tested first, but even in light sleep the board draws several mA, which is not acceptable for multi-month battery use (target was at least 6 months).
- ESP32-H2 gives a much lower standby current. A Waveshare ESP32-H2 Mini (ESP32-H2FH4S) consumes about 700 uA in light sleep out of the box; removing the WS2812 LED and its driver drops that to roughly 234 uA. With the battery ADC circuitry attached the measured idle draw is about 300 uA, which works out to roughly 5 months from two 14500 LiFePO4 cells at 3.2 V.
- Power measurements: [with WS2812 present](images/h2_ws2812_installed.jpg) vs [after removing it](images/h2_ws2812_removed.jpg).
- If starting today, a Seeed XIAO MG24 (EFR32MG24) would be preferable: it can sleep with RAM retention at around 5 uA while keeping Zigbee state.
- Reference implementation is published at https://github.com/vsychov/ZigbeePulseCounter (AI-assisted; expect rough edges).
- Photos: [assembled device](images/device_assembled.jpg), [internals](images/device_internals.jpg).

## Quick start

```bash
idf.py set-target esp32c6
idf.py build flash monitor
```

## Hardware

### Dry contact (pulses)

- Reed switch for Gas: https://www.tme.eu/de/details/mk65b/reed-relais/meder/mk06-5-b/
- `PULSE_GPIO` (default GPIO9), input with pull-up.
- Contact closes to GND.
- Falling-edge interrupt, default debounce 50 ms.

Example wiring:

```
PULSE_GPIO ----+----o/ o---- GND
               |
               +--[internal pull-up]
```

### Zigbee factory reset button

- `FACTORY_RESET_BUTTON_GPIO`, active low.
- Long press (8 s) performs Zigbee factory reset and reboots the device.
- Can be set to a separate GPIO or disabled (`-1`).

### Battery (ADC)

- Measured via `adc_oneshot`.
- Voltage divider: `Rtop` and `Rbot` are set in Kconfig.
- Hardware used: 2x14500 battery case (https://www.amazon.de/dp/B08JV9S4XY), resistors 100k and 300k (0.25 W), and a 104 (100 nF) ceramic capacitor on the ADC input.

Formula:

```
Vbat = Vadc * (Rtop + Rbot) / Rbot
```

Wiring reference (also shown in the internals photo):

```
              +3.3V
                |
                |        ┌───────────────┐
                |        │   BATTERY     │
                |        └───────────────┘
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

Notes:

- GPIO_1 is pulled up to 3.3V through a 300k resistor.
- GPIO_1 is shunted to GND by a 100k resistor and a 100nF capacitor.
- GPIO_10 is connected to GND through the reed switch.
- The battery supplies the 3.3V and GND rails.

## Zigbee

- Role: End Device (sleepy, RxOffWhenIdle=true), built on the ZED library.
- Primary cluster: Simple Metering (0x0702).
- Battery: Power Configuration (0x0001).
- OTA: Zigbee OTA Upgrade (0x0019).
- Custom cluster: 0xFD10 (manufacturer code 0x1234) - only the counter reset command (write-only attr 0x0008).
- Zigbee Model ID comes from `CONFIG_ZB_MODEL_IDENTIFIER` (Kconfig). Convenience strings:
  - `ESP32C6-PulseMeter-Gas` (m3, m3/h)
  - `ESP32C6-PulseMeter-Water` (m3, m3/h)
  - `ESP32C6-PulseMeter-Electric` (kWh, kW)

## Configuration (Kconfig)

All parameters are set at build time through `menuconfig`:

- `Device variant` (Gas / Water / Electric) - single choice that automatically sets modelId, UnitOfMeasure, and MeteringDeviceType.
- `PULSE_PER_UNIT_NUMERATOR` - number of pulses that equals one accounting unit (kWh or m3).
- `PULSE_DEBOUNCE_MS` - pulse debounce time.
- modelId/UnitOfMeasure/MeteringDeviceType are selected automatically from `Device variant` (no manual edits).
- No custom Zigbee settings besides the reset command (cluster 0xFD10, attribute 0x0008 boolean).
- `FACTORY_RESET_BUTTON_GPIO` - GPIO for the hardware Zigbee factory reset button (8 s long press). `-1` disables it.

UnitOfMeasure and MeteringDeviceType are set only at build time (Kconfig):
- `ZB_UNIT_OF_MEASURE`
- `ZB_METERING_DEVICE_TYPE`
- `ZB_MODEL_IDENTIFIER` (modelId string consumed by the Z2M converter)

Example: 10000 pulses per 1 kWh -> `PULSE_PER_UNIT_NUMERATOR = 10000`.

## Zigbee2MQTT

The repository includes an external converter `z2m/zigbee_pulse_meter.js` that:
- binds `seMetering`/`genPowerCfg`;
- configures reporting for `instantaneousDemand` and 48-bit `currentSummationDelivered`;
- reads `multiplier/divisor` and formatting so the UI immediately shows correct units;
- adds a `reset_counter` button in the UI (cluster 0xFD10, attr 0x0008).

### Installing the external converter

1. Copy `z2m/zigbee_pulse_meter.js` into the Zigbee2MQTT directory (usually `/config`).
2. Add to `configuration.yaml`:

```
external_converters:
  - zigbee_pulse_meter.js
```

3. Restart Zigbee2MQTT and re-interview the device.

The converter supports three modelId strings (see above). Pick the one you need via `CONFIG_ZB_MODEL_IDENTIFIER` at build time so the UI shows correct units (m3/m3h or kWh/kW).

### Example attribute write (Z2M)

```
zigbee2mqtt/DEVICE_ID/set
{
  "state": "ON"
}
```

## Reporting

- Report intervals and thresholds are configured via standard Configure Reporting from the coordinator/Z2M.
- Standard seMetering attributes are reported: `currentSummationDelivered` (48-bit, Z2M provides delta as an array, for example `[0, 1]`) and `instantaneousDemand`.
- The external converter binds `seMetering`, calls `reporting.instantaneousDemand`/`reporting.currentSummDelivered`, and reads `multiplier/divisor`, `summationFormatting/demandFormatting`, `unitOfMeasure`, and the current value so the UI reflects the firmware scale.

## Project files

- `main/main.c` - Zigbee init, event handling, reporting.
- `main/pulse.c` - pulse handling, debounce, min width.
- `main/metering.c` - converts pulses to the 0x0702 summation.
- `main/power.c` - battery measurement and USB detect.
- `main/config_cluster.c` - counter/NVS storage and custom cluster 0xFD10 (reset).
- `main/ota.c` - OTA client init.

## Kconfig settings

Core:
- `PULSE_PER_UNIT_NUMERATOR` - pulses per accounting unit (kWh or m3).
- `PULSE_DEBOUNCE_MS` - debounce.
- `ZB_UNIT_OF_MEASURE`, `ZB_METERING_DEVICE_TYPE`, `ZB_MODEL_IDENTIFIER` - units, device type, modelId.
- Reset command - custom cluster 0xFD10, attribute 0x0008 (from Z2M/HA UI "reset_counter").

Channels and scan:
- `ZB_CHANNEL_MASK` (primary) and `ZB_SECONDARY_CHANNEL_MASK` (secondary, 0 to disable). Default 11-26 (0x07FFF800). If you know the network channel, set it for faster join and better odds with weak signal.
- `ZB_BDB_SCAN_DURATION` - scan duration per channel (exponent, default 6). Higher means it listens longer and has a better chance to catch the network.

All parameters are available via `idf.py menuconfig`.

### DEMAND_* recommendations (instantaneous power decay)

Values below were chosen so that graphs do not drop to zero instantly on rare pulses but also do not linger for too long.

| Variant                       | Pulses per unit | Typical flow              | `DEMAND_RISE_TAU_S` | `DEMAND_DECAY_TAU_S` | `DEMAND_IDLE_TIMEOUT_S` | Notes                                                                                 |
|-------------------------------|-----------------|---------------------------|---------------------|----------------------|-------------------------|---------------------------------------------------------------------------------------|
| Gas (0.01 m3/pulse)           | 100 pulses/m3   | 1-2 m3/h (peak ~6 m3/h)   | 8-12                | 90-120               | 120-180                 | At 1 m3/h a pulse every ~36 s: keep timeout >60-90 s to avoid zeroing between pulses. |
| Water (10 l/pulse)            | 100 pulses/m3   | 0.1-1 m3/h                | 10-15               | 120-180              | 240-360                 | At 0.1 m3/h a pulse every ~6 min: need a long timeout so it does not drop to zero.    |
| Electric (10k pulses/kWh)     | 10000 pulses/kWh| 0.5-5 kW (peaks 6-10 kW)  | 3-6                 | 20-40                | 10-20                   | Pulses arrive more often (hundredths of a second at high load), so fast response.     |

Tune values for your meter/sensor and real flow range. If you see the graph fall too quickly with sparse pulses, raise `DEMAND_DECAY_TAU_S` and `DEMAND_IDLE_TIMEOUT_S`; if it lingers too long, decrease them.

### OTA updates

- Zigbee OTA client (cluster 0x0019) is enabled, manufacturer `0x1234`, image type `CONFIG_ZB_OTA_IMAGE_TYPE` (default `0x0001`), version `CONFIG_ZB_OTA_FILE_VERSION`.
- Before a release, bump `CONFIG_ZB_OTA_FILE_VERSION`, build the firmware (`idf.py build` -> `build/zigbee_pulse_meter.bin`).
- Package into an OTA image:
  ```
  python $HOME/esp/esp-zigbee-sdk/tools/image_builder_tool/image_builder_tool.py \
    -c zigbee_pulse_meter-ota.bin \
    -v 0x00000004 \
    -m 0x1234 \
    -i 0x0001 \
    -f build/zigbee_pulse_meter.bin
  ```
  The `-v/-m/-i` values must match Kconfig (version/image type/manufacturer).
- Copy the resulting file to the Zigbee2MQTT OTA folder (usually `data/ota/`), restart Z2M, re-interview the device, and start the update from the UI.
- If `image_builder_tool.py` complains about missing `zigpy`, first activate a suitable Python (for example, `pyenv shell 3.12.9` or `python -m venv .venv && source .venv/bin/activate`), then install the dependency:
  ```
  python -m pip install zigpy
  ```
  After installing, run `image_builder_tool.py` again.

## Connecting to Zigbee2MQTT

- In Zigbee2MQTT the permit-join window is opened for 254 s.
- A factory-new device repeatedly tries to join with attempt intervals <= ~40-90 s so it does not miss the window.
- After a successful join, on reboot steering is not started again (normal rejoin/operation only).
- For faster and more reliable join, set the exact channel (primary/secondary) and increase `ZB_BDB_SCAN_DURATION`.

### Join scenarios

1) **Initial pairing**: open permit join in Z2M for 254 s. A factory-new device enters pairing mode automatically (BDB network steering). If you need to start from scratch manually, perform a factory reset (see above).
2) **Normal rejoin after power/reboot**: the device starts as an End Device and attempts rejoin without permit join. Steering is not restarted again unless an error occurs.
3) **Reflashing without erase**: `zb_storage` is preserved, so permit join is not needed after a successful rejoin. If the stack cannot rejoin (incompatible data), the device falls back to pairing mode with retry intervals <= 90 s. If it still does not connect, perform a factory reset and pair again.
