# Repository Guidelines

## Project Structure & Module Organization
The firmware lives in `src/`, with `main.cpp` orchestrating boot, MQTT, and CN105 bridging, while companion modules such as `CascadeNetwork`, `Ecodan*`, and `MELCloudDecoder` isolate protocol parsing and discovery publishing. Configuration headers (`MQTTConfig.h`, `Flags.h`, `Debug.h`) act as feature toggles; keep defaults minimal and document non-obvious settings in `documentation/`. Hardware notes and protocol references reside in `documentation/`; update them whenever topics, pins, or telemetry fields change. Generated binaries and board-specific artifacts belong under `build/` or `.pio/` and should stay out of commits.

## Build, Test, and Development Commands
- `pio run` builds every environment declared in `platformio.ini`.
- `pio run -e m5stack-atoms3 -t upload` (swap the environment name as needed) flashes a connected board.
- `pio device monitor --environment m5stack-atoms3 --baud 115200` tails runtime logs; capture traces when reporting bugs.
- `pio run -t clean` resets the workspace before switching branches or PlatformIO cores.

## Coding Style & Naming Conventions
Stay with Arduino-flavoured C++: 4-space indentation, opening braces on the same line, minimal inline comments, and `//` for single-line notes. Name types with PascalCase (`CascadeNetwork`), functions with camelCase (`publishStatus`), and constants/macros in UPPER_SNAKE_CASE. Prefer `constexpr`, enums, and helper functions over duplicated literals. Header guards are used today; `#pragma once` is acceptable so long as the file stays consistent. Regenerate `compile_commands.json` via `pio run -t compiledb` whenever you add or rename modules so clangd remains accurate.

## Testing Guidelines
Automated unit tests are not yet present; new ones should live in `test/` using the PlatformIO harness and run via `pio test -e <env>`. Document any hardware-in-the-loop steps (sensor mocks, MQTT brokers, Home Assistant version) inside the PR. Before requesting review, flash at least one target board and share monitor output that shows a full MQTT publish cycle. Add sample payloads or captures to `documentation/` whenever you adjust the CN105 protocol mapping.

## Commit & Pull Request Guidelines
Recent history favours short, scoped subjects such as `debug: add ECODAN serial...`; follow the `module: concise change` pattern to signal intent. Keep commits logically grouped, separating configuration churn from behavioural edits. Pull requests should describe the feature, list the environments tested, and link any tracking issues or forum threads. Include screenshots or MQTT payload excerpts when altering discovery topics or dashboards, and flag breaking changes prominently for downstream integrators.

## Hardware & Configuration Tips
Maintain reproducible defaults by mirroring new options into `MQTTConfig.h` and `Flags.h`, and document board-specific wiring in `documentation/Hardware and Recovery.md`. When experimenting, keep local secrets in an untracked `data/` folder and reference them via `LittleFS`. Verify CN105 serial polarity before powering hardware; incorrect wiring can lock the controller.
