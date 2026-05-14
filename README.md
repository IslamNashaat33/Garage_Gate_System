# Smart Parking Garage Gate Embedded System

**CSE323: Advanced Embedded Systems Design**  
**Spring 2026**

## Project Summary

This repository contains an embedded control system for a smart parking garage gate implemented on the Tiva-C TM4C123GH6PM microcontroller. The system uses FreeRTOS to coordinate tasks for button input processing, gate state control, safety monitoring, and hardware actuation.

Key features:

- Driver and security panel control with priority handling
- Manual and one-touch automatic gate movement modes
- Obstacle detection with automatic reverse behavior
- Limit-switch protection for fully open / fully closed positions
- Modular HAL and RTOS-based task design

## Repository Layout

- `Project_team55/` — main project folder containing source files and project configuration
  - `hardware.c`, `hardware.h` — low-level MCU register setup and hardware abstraction definitions
  - `main.c` — application entry point, FreeRTOS initialization, and scheduler start
  - `tasks.c`, `tasks.h` — RTOS task implementations and shared application interfaces


## Build & Run Instructions

1. Open `Project_team55/project.uvprojx` in Keil uVision.
2. Verify that the target device is set to `TM4C123GH6PM`.
3. Confirm that `Project_team55/Required_Files/` and `Project_team55/RTE/` are included in the project.
4. Build the project using **Build > Rebuild all target files**.
5. Flash to the TM4C123GXL development board and run.

> Note: This project is organized for Keil uVision. If you need to port to another toolchain, use `hardware.h` and `hardware.c` to adapt MCU register mappings.

## Hardware Interface

### Input Signals

- `PB0` — Driver OPEN button
- `PB1` — Driver CLOSE button
- `PB2` — Security OPEN button
- `PB3` — Security CLOSE button
- `PE1` — Gate fully open limit sensor
- `PE2` — Gate fully closed limit sensor
- `PE3` — Obstacle sensor

### Output Signals

- `PA2` — Motor open control
- `PA3` — Motor close control
- `PF1` — Red LED
- `PF2` — Blue LED
- `PF3` — Green LED

## System Behavior

### Button Handling Modes

- **Manual mode**: Hold OPEN or CLOSE for longer than 600ms; gate moves only while the button remains pressed.
- **Auto mode**: A short press (< 600ms) triggers a full open or close operation until the corresponding limit is reached.

### Safety Rules

- If an obstacle is detected while closing, the gate stops and reverses open for 0.5 seconds.
- The gate stops automatically when the open or closed limit sensor is reached.
- Security panel commands override driver panel commands when both are active.

## Software Design

### Main Modules

- `main.c` — initializes the MCU, creates RTOS primitives, starts tasks, and begins the scheduler.
- `hardware.c` / `hardware.h` — configures GPIO, clocks, and maps physical pins to logical signals.
- `tasks.c` / `tasks.h` — implements FreeRTOS tasks, debouncing, FSM logic, and system coordination.

### RTOS Components

- Tasks: Input processing, gate control, safety monitoring, and hardware actuation
- Queues: Button event transfer and command dispatch
- Semaphores: ISR signaling for immediate hardware events
- Mutexes: Shared state protection for gate control variables

## Notes

- The project uses active-low buttons and sensors. Input logic is inverted accordingly in software.
- The system is intended as a lab exercise for embedded systems design and real-time control.

## Contact

For questions about the implementation or build environment, refer to the source files in `Project_team55/` or contact the project author/instructor.
