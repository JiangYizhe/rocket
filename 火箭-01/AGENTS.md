# Repository Guidelines

## Project Structure & Module Organization
This is a Keil uVision STM32F103RC firmware project. Application entry points and interrupt hooks live in `User/`, including `main.c`, `stm32f10x_it.c`, and `stm32f10x_conf.h`. Board, sensor, and flight-control modules live in `Hardware/`, including `Servo`, `IMU`, `LSM6DSV16X`, `Ignition`, `FlightControl`, `RocketState`, and `Rocket`. Shared timing utilities are in `System/`, CMSIS/startup files are in `Start/`, and the STM32F10x standard peripheral library is in `Library/`.

Generated outputs are stored in `Objects/` and `Listings/`; avoid editing these by hand. Keil configuration is held in `project.uvprojx`, `project.uvoptx`, and user-specific `.uvguix.*` files.

## Build, Test, and Development Commands
Open `project.uvprojx` in Keil uVision and build target `Target 1` for `STM32F103RC`.

- Keil: `Project > Build target` builds `Objects/project.axf`.
- Keil: `Project > Rebuild all target files` performs a clean full rebuild.
- `keilkill.bat` can be used to clean common Keil intermediate files when needed.

If adding command-line automation, document the exact Keil/ARMCC path and keep generated artifacts out of source changes unless they are intentionally versioned.

## Coding Style & Naming Conventions
Use C consistent with the existing STM32 standard peripheral style. Indent blocks with tabs, keep braces on their own lines, and include local headers with quotes, for example `#include "IMU.h"`.

Name driver modules by peripheral or device, using paired `.c`/`.h` files such as `Hardware/IMU.c` and `Hardware/IMU.h`. Use function prefixes matching the module, for example `IMU_Init` and `IMU_Read`. Keep hardware constants and register definitions in headers with clear uppercase names.

## Testing Guidelines
There is no host-side unit test framework. Validate changes by rebuilding in Keil, flashing the STM32F103RC target, and exercising the affected hardware path. For interrupt, watchdog, timing, or sensor changes, record the board setup, expected behavior, and observed result.

## Commit & Pull Request Guidelines
No Git history is present in this checkout, so no project-specific commit convention can be inferred. Use short imperative commit subjects such as `Add WWDG refresh handling` or `Fix IMU initialization timeout`.

Pull requests should describe the firmware behavior changed, list affected modules, mention the target board/configuration tested, and include logs or serial output when useful. Do not mix generated Keil output with source edits unless required.

## Agent-Specific Instructions
Before editing, inspect the relevant `.c`, `.h`, and `project.uvprojx` entries. Preserve user-specific Keil settings unless the task explicitly requires changing them. Prefer small, hardware-focused changes and avoid broad rewrites of vendor library files in `Library/` or CMSIS files in `Start/`.
