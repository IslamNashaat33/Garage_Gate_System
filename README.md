# Smart Parking Garage Gate Embedded System

**CSE323: Advanced Embedded Systems Design**  
**Spring 2026**

## 1. Project Overview

This project implements a Smart Parking Garage Gate System using the Tiva-C TM4C123GH6PM microcontroller running FreeRTOS. The system simulates an automated parking gate that can be controlled from both a driver's panel and a security panel. It includes safety features such as obstacle detection, motion limits, and preemptive security priority loops, ensuring responsive and safe real-time operations.

## 2. Project Architecture & File Hierarchy

To ensure modularity, maintainability, and clean decoupling of the hardware logic from the system's business logic, the application is structured across 5 primary files:

- **`main.c`**  
  **Purpose:** The entry point of the system.  
  **Why we need it:** It acts as the primary orchestrator. It triggers hardware initialization, creates the FreeRTOS synchronization primitives (Queues, Semaphores, Mutexes), spawns the RTOS tasks (Input, Gate Control, Safety), and finally starts the FreeRTOS scheduler to begin execution.

- **`tasks.h`**  
  **Purpose:** Public API and declarations for RTOS tasks.  
  **Why we need it:** It acts as the bridge connecting the application logic, sharing task prototypes, external queue handles, and thread-safe data structures across the application cleanly without exposing internal task implementations.

- **`tasks.c`**  
  **Purpose:** Real-time multitasking logic and Finite State Machine (FSM).  
  **Why we need it:** This file contains the core brains of the operation. It implements the FreeRTOS tasks that evaluate input logic, execute the gate's state machine, and handle inter-task communication. This abstracts the behavioral logic away from the actual hardware.

- **`hardware.h`**  
  **Purpose:** Hardware Abstraction Layer (HAL) definitions and hardware map.  
  **Why we need it:** It maps high-level abstract variables (e.g., `MOTOR_OPEN`, `RED_LED`) to specific microcontroller pins. It also contains system-wide definitions for events, states, and timing thresholds. Centralizing these definitions makes it easy to port the project to another MCU or alter pin wirings without rewriting any task logic.

- **`hardware.c`**  
  **Purpose:** Direct register-level hardware manipulation.  
  **Why we need it:** It executes the low-level CMSIS instructions. This file handles clock initialization, configures GPIO port directions/pull-ups, actuates motors and LEDs, and processes hardware Interrupt Service Routines (ISRs) for immediate reaction to limits and obstacles.

## 3. Hardware Mapping

### Outputs

- **LED Indicators:** `PF1` = Red LED, `PF2` = Blue LED, `PF3` = Green LED
- **Motor Status(Port A):** `PA2` = Opening Motor, `PA3` = Closing Motor

### Inputs (Active-Low Buttons/Sensors)

- **Port B (Driver & Security Panels):**
  - `PB0` = Driver OPEN
  - `PB1` = Driver CLOSE
  - `PB2` = Security OPEN
  - `PB3` = Security CLOSE
- **Port E (Sensors):**
  - `PE1` = Limit OPEN (fully-open sensor)
  - `PE2` = Limit CLOSED (fully-closed sensor)
  - `PE3` = Obstacle sensor

### Gate State & LED Indication Table

| State                   | LED Color | Pins Active          |
| ----------------------- | --------- | -------------------- |
| **GATE_IDLE_CLOSED**    | Blue      | BLUE_LED             |
| **GATE_IDLE_OPEN**      | Cyan      | GREEN_LED + BLUE_LED |
| **GATE_OPENING**        | Green     | GREEN_LED            |
| **GATE_CLOSING**        | Red       | RED_LED              |
| **GATE_STOPPED_MIDWAY** | Yellow    | RED_LED + GREEN_LED  |
| **GATE_REVERSING**      | Magenta   | RED_LED + BLUE_LED   |

## 4. Project Objectives

- Apply real-time operating system concepts using FreeRTOS
- Design a multitasking embedded system
- Implement safe gate control using limit buttons
- Use RTOS mechanisms: tasks, queues, semaphores, and mutexes
- Develop a finite state machine for system behavior

## 5. System Functional Requirements

### Manual Mode

When the OPEN or CLOSE button is held (>= 600ms), the gate moves only while the button remains pressed. Releasing the button stops the gate.

### One-Touch Auto Mode

A brief press of the button (< 600ms) causes the gate to move fully in that direction until the corresponding limit button is pressed.

### Obstacle Protection

If an obstacle is detected while the gate is auto-closing:

- Gate stops immediately
- Gate reverses (opens) for 0.5 seconds
- Gate stops completely

### Priority Control

Security panel commands take priority over driver panel commands when both are active simultaneously.

---

## 6. FreeRTOS Task Design

| Task                  | Priority | Description                                                             |
| --------------------- | -------- | ----------------------------------------------------------------------- |
| **Safety Task**       | Highest  | Monitors obstacle signals; enforces safety behavior and system override |
| **Input Task**        | High     | Reads and debounces all buttons every 20ms; sends events to a queue     |
| **Gate Control Task** | Medium   | Implements the FSM; decides gate actions handling priority loops        |
| **LED/Motor Control** | Medium   | Controls motor relays and RGB indicators based on state                 |

---

## 7. Inter-Task Communication

| Mechanism     | Purpose                                                      |
| ------------- | ------------------------------------------------------------ |
| **Queue**     | Send button events, timestamps, and commands between tasks   |
| **Semaphore** | Signal immediate limit or obstacle activation from ISRs      |
| **Mutex**     | Protect shared gate state variables avoiding race conditions |
