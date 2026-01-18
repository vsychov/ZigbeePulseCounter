# Repository Guidelines

## Project Structure & Modules
- Firmware source lives in `main/` (C): `main.c`, `pulse.c`, `metering.c`, `power.c`, `ota.c`, `config_cluster.c`, headers alongside.
- Zigbee2MQTT external converter is in `z2m/zigbee_pulse_meter.js`.
- Zigbee/partition settings: `sdkconfig`, `sdkconfig.defaults`, `partitions.csv`.
- Docs: `README.md`, Zigbee reference `zcl.txt`.

## Build & Development
- Activate toolchain and build:  
  `source $HOME/esp/esp-idf/export.sh && idf.py set-target esp32h2 && idf.py build`
- Flash & monitor: `idf.py -p <PORT> flash monitor`
- Clean (rarely needed): `idf.py fullclean`
- Keep `sdkconfig.defaults` in sync with `sdkconfig` when changing Kconfig defaults.

## Coding Style & Naming
- Language: C17; prefer static functions and minimal globals.
- Indent with 4 spaces; keep lines < 120 chars.
- Use ESP-IDF logging (`ESP_LOGI/W/E`) with clear tags.
- Use `APP_`/`CONFIG_` prefixes for app-level constants; Zigbee IDs in hex.
- ASCII only unless protocol requires otherwise.

## Testing Guidelines
- No automated tests in tree; validate by building and running on device with `idf.py flash monitor`.
- For Zigbee behavior, re-interview in Z2M after flashing and check attribute reports (metering, power, FD10).

## Commit & PR Guidance
- Commit messages: imperative, short summary (e.g., `fix metering reset`, `add fd10 reporting`).
- In PRs: describe feature/fix, affected modules, and how it was verified (build/flash/monitor logs).
- Attach relevant logs for Zigbee commissioning/reporting changes.

## Agent-Specific Tips
- Use `apply_patch` for edits; avoid reverting user changes. Do not run destructive git commands.
- Prefer `rg` for searches. Keep changes in ASCII. Avoid adding dependencies; rely on ESP-IDF + managed components.
- When editing Zigbee behavior, update both firmware (`main/`) and converter (`z2m/`) if the model ID or attributes change.
- After altering Kconfig options, rebuild `sdkconfig` via `idf.py menuconfig` or by editing `sdkconfig.defaults` and regenerating `sdkconfig`.
- ALWAYS After changing firmware, test it for success build: `. $HOME/esp/esp-idf/export.sh && idf.py build`
- ESP-IDF sdk stored at `$HOME/esp/esp-idf`, you can check it for usage examples
- Some examples also at `$HOME/esp/esp-zigbee-sdk`
- Don't touch git

## Security & Config Notes
- Manufacturer code: `0x1234`; custom cluster: `0xFD10`; device ID default `0x7777`.
- Model identifier set via `CONFIG_ZB_MODEL_IDENTIFIER` (e.g., `ESP32C6-PulseMeter-Gas/Water/Electric`) to match Z2M converter.
