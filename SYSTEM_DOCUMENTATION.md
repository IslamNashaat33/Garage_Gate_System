# Garage Gate System - Complete Documentation

## Table of Contents
1. [System Overview](#1-system-overview)
2. [Hardware Architecture](#2-hardware-architecture)
3. [GPIO Pin Configuration](#3-gpio-pin-configuration)
4. [Software Components](#4-software-components)
5. [State Machine](#5-state-machine)
6. [Task Architecture](#6-task-architecture)
7. [Flow Diagrams](#7-flow-diagrams)
8. [Inter-Task Communication](#8-inter-task-communication)
9. [Safety Features](#9-safety-features)
10. [Quick Reference](#10-quick-reference)

---

## 1. System Overview

### Purpose
Automated garage gate controller with:
- Driver controls (open/close buttons)
- Security system controls (separate open/close buttons)
- Limit switches (open/closed positions)
- Obstacle detection safety
- LED status indication
- FSM-based deterministic control

### Platform
- **MCU:** Tiva TM4C123 (ARM Cortex-M4)
- **OS:** FreeRTOS
- **Language:** C

---

## 2. Hardware Architecture

### Input Devices

| Device | Purpose | GPIO | Port/Pin |
|--------|---------|------|----------|
| Driver Open Button | Driver manual open | Input | PB0 |
| Driver Close Button | Driver manual close | Input | PB1 |
| Security Open Button | Security system open | Input | PB2 |
| Security Close Button | Security system close | Input | PB3 |
| Open Limit Switch | Gate fully open position | Input | PE1 |
| Closed Limit Switch | Gate fully closed position | Input | PE2 |
| Obstacle Sensor | Safety beam interruption | Input | PE3 |

### Output Devices

| Device | Purpose | GPIO | Port/Pin |
|--------|---------|------|----------|
| Green LED | Opening state indicator | Output | PF3 |
| Red LED | Closing state indicator | Output | PF1 |
| Motor Open | Gate motor direction | Output | PF3* |
| Motor Close | Gate motor direction | Output | PF1* |

*Note: Motor driver connections share LED pins (need H-bridge interface in hardware)

---

## 3. GPIO Pin Configuration

```
+====================+================+==========+============+============+
| Signal             | Port           | Pin      | Direction  | Pull-up    |
+====================+================+==========+============+============+
| LED_RED             | PF             | PF1      | Output     | -          |
+--------------------+----------------+----------+------------+------------+
| LED_GREEN          | PF             | PF3      | Output     | -          |
+--------------------+----------------+----------+------------+------------+
| DRIVER_OPEN        | PB             | PB0      | Input      | Weak PU    |
+--------------------+----------------+----------+------------+------------+
| DRIVER_CLOSE       | PB             | PB1      | Input      | Weak PU    |
+--------------------+----------------+----------+------------+------------+
| SECURITY_OPEN      | PB             | PB2      | Input      | Weak PU    |
+--------------------+----------------+----------+------------+------------+
| SECURITY_CLOSE     | PB             | PB3      | Input      | Weak PU    |
+--------------------+----------------+----------+------------+------------+
| LIMIT_OPEN         | PE             | PE1      | Input      | Weak PU    |
+--------------------+----------------+----------+------------+------------+
| LIMIT_CLOSED       | PE             | PE2      | Input      | Weak PU    |
+--------------------+----------------+----------+------------+------------+
| OBSTACLE           | PE             | PE3      | Input      | Weak PU    |
+====================+================+==========+============+============+
```

**Active Level:** All inputs are **active-low** (button pressed = 0V due to internal pull-up)
**Interrupt:** GPIO_BOTH_EDGES (triggers on any edge change)

---

## 4. Software Components

### 4.1 File Structure

```
Garage_Gate_System/
├── main.c          # Entry point, MCU init, FreeRTOS start
├── gpio.c/h        # Hardware abstraction layer
├── fsm.c/h        # Finite state machine
├── tasks.c/h      # FreeRTOS tasks
└── README.md      # Basic project info
```

### 4.2 Component Responsibilities

#### GPIO Module (gpio.c/h)
| Function | Responsibility |
|----------|----------------|
| `GPIO_HardwareInit()` | Initialize all GPIO pins, enable interrupts |
| `GPIO_RegisterInputQueue()` | Register queue for ISR-to-task communication |
| `GPIO_SetGreenLed(bool)` | Control PF3 output |
| `GPIO_SetRedLed(bool)` | Control PF1 output |
| `GPIO_GateMotorSetDirection()` | Set motor direction (OPEN/CLOSE/STOP) |
| `GPIO_IsOpenLimitActive()` | Read PE1 state |
| `GPIO_IsClosedLimitActive()` | Read PE2 state |
| `GPIOB_Handler()` | ISR: driver/security buttons (PB0-3) |
| `GPIOE_Handler()` | ISR: limits/obstacle (PE1-3) |

#### FSM Module (fsm.c/h)
| Function | Responsibility |
|----------|----------------|
| `FSM_Init()` | Initialize FSM context |
| `FSM_ProcessEvent()` | Process event, update state, control motor |
| `SourceHasAnyCommand()` | Check if source has active command |
| `GetEffectiveSource()` | Get priority source (security > driver) |
| `StopSafely()` | Stop motor to safe state |

#### Tasks Module (tasks.c/h)
| Task | Responsibility |
|------|----------------|
| `InputTask` | Debounce buttons, convert to FSM events |
| `GateControlTask` | Execute FSM, control motor |
| `LedControlTask` | LED status indication |
| `SafetyTask` | Highest priority obstacle handling |
| `StatusTask` | Telemetry placeholder |

---

## 5. State Machine

### 5.1 States

```
+------------------+-------+----------------------------------------+
| State            | Value | Description                          |
+------------------+-------+----------------------------------------+
| GATE_STATE_IDLE_OPEN    | 0     | Gate open, stationary           |
+------------------+-------+----------------------------------------+
| GATE_STATE_IDLE_CLOSED | 1     | Gate closed, stationary        |
+------------------+-------+----------------------------------------+
| GATE_STATE_OPENING     | 2     | Motor running OPEN           |
+------------------+-------+----------------------------------------+
| GATE_STATE_CLOSING    | 3     | Motor running CLOSE         |
+------------------+-------+----------------------------------------+
| GATE_STATE_STOPPED_MIDWAY | 4  | Stopped between positions    |
+------------------+-------+----------------------------------------+
| GATE_STATE_REVERSING    | 5     | Safety reverse (500ms)       |
+------------------+-------+----------------------------------------+
```

### 5.2 Motor Directions

```
+------------------+-------+
| Direction        | Value |
+------------------+-------+
| GATE_MOTOR_STOP  | 0     |
+------------------+-------+
| GATE_MOTOR_OPEN  | 1     |
+------------------+-------+
| GATE_MOTOR_CLOSE | 2     |
+------------------+-------+
```

### 5.3 Command Sources (Priority)

```
+------------------+-------+------------------+
| Source           | Value | Priority         |
+------------------+-------+------------------+
| CMD_SOURCE_DRIVER   | 0     | Lower           |
+------------------+-------+------------------+
| CMD_SOURCE_SECURITY| 1     | Higher          |
+------------------+-------+------------------+
| CMD_SOURCE_NONE   | 2     | N/A             |
+------------------+-------+------------------+
```

### 5.4 FSM Events

```
+----------------------------+----------------------------------------+
| Event                      | Description                          |
+----------------------------+----------------------------------------+
| FSM_EVENT_CMD_OPEN_PRESS    | Open button pressed                  |
+----------------------------+----------------------------------------+
| FSM_EVENT_CMD_OPEN_RELEASE  | Open button released                |
+----------------------------+----------------------------------------+
| FSM_EVENT_CMD_CLOSE_PRESS    | Close button pressed               |
+----------------------------+----------------------------------------+
| FSM_EVENT_CMD_CLOSE_RELEASE| Close button released             |
+----------------------------+----------------------------------------+
| FSM_EVENT_CMD_OPEN_AUTO   | One-touch auto open              |
+----------------------------+----------------------------------------+
| FSM_EVENT_CMD_CLOSE_AUTO  | One-touch auto close             |
+----------------------------+----------------------------------------+
| FSM_EVENT_LIMIT_OPEN      | Open limit switch reached         |
+----------------------------+----------------------------------------+
| FSM_EVENT_LIMIT_CLOSED   | Closed limit switch reached        |
+----------------------------+----------------------------------------+
| FSM_EVENT_SAFETY_OBSTACLE| Obstacle detected while closing    |
+----------------------------+----------------------------------------+
| FSM_EVENT_CONFLICT        | Both open+close pressed          |
+----------------------------+----------------------------------------+
| FSM_EVENT_REVERSE_TIMEOUT| Reverse duration complete        |
+----------------------------+----------------------------------------+
```

### 5.5 State Transition Diagram

```
                          +--------------------+
                          |                    |
    +-------------------  | GATE_STATE_IDLE_OPEN  |--  (FSM_EVENT_LIMIT_OPEN)
    |                    |                    |      (motor stops)
    |                    +--------------------+
    |                          ^
    |                          |
    |                   FSM_EVENT_CMD_
    |                   OPEN_PRESS/
    |                   OPEN_AUTO
    |                          |
    +--------------------+      |
    |                    |      |
    | GATE_STATE_OPENING |<-----
    | (motor = OPEN)     |
    |                    |
    +--------------------+
    |                          |
    |                          | FSM_EVENT_CMD_
    |                          | CLOSE_PRESS/
    |                   FSM_EVENT_CMD_       CLOSE_AUTO
    |                   OPEN_PRESS/
    |                   OPEN_AUTO            |
    |                          |             v
    |                          |   +--------------------+
    |                          |   |                    |
    +--------------------+   |   | GATE_STATE_CLOSING |
    |                    |   |   | (motor = CLOSE)    |
    | GATE_STATE_        |<--+---                    |
    | STOPPED_MIDWAY     |   |   +--------------------+
    | (motor = STOP)    |   |           ^
    |                    |   |           |
    +--------------------+   |           | FSM_EVENT_LIMIT_CLOSED
        ^                  |           | (motor stops)
        |                  |           |
        |                  |    FSM_EVENT_
        |                  |    SAFETY_OBSTACLE
        |                  |    (while closing)
        |                  |
        |           +------+--------+
        |           |                 |
        |           | GATE_STATE_      |
        +---------- | REVERSING       |
                    | (motor = OPEN,  |
                    |  500ms)       |
                    +----------------+
                             |
                             | REVERSE_TIMEOUT
                             v
```

---

## 6. Task Architecture

### 6.1 Task Priority Structure

```
+--------------------+-------------+------------------------+
| Task               | Priority    | Stack Size             |
+--------------------+-------------+------------------------+
| SafetyTask         | +4 (Highest)| configMINIMAL_STACK_   |
|                    |             | SIZE + 96             |
+--------------------+-------------+------------------------+
| InputTask         | +3          | configMINIMAL_STACK_  |
|                    |             | SIZE + 128           |
+--------------------+-------------+------------------------+
| GateControlTask   | +2          | configMINIMAL_STACK_  |
|                    |             | SIZE + 128           |
+--------------------+-------------+------------------------+
| LedControlTask    | +2          | configMINIMAL_STACK_|
|                    |             | SIZE + 96            |
+--------------------+-------------+------------------------+
| StatusTask        | +1 (Lowest) | configMINIMAL_STACK_|
|                    |             | SIZE + 96            |
+--------------------+-------------+------------------------+
```

### 6.2 Task Details

#### InputTask (Priority +3)
```
Purpose:      Process raw GPIO input events, apply debouncing
Input:        g_inputQueue (from ISRs)
Output:       g_gateEventQueue (to FSM)
Debounce:     30ms filter
Auto Mode:    Press > 500ms = auto-toggle
```

#### GateControlTask (Priority +2)
```
Purpose:      Main FSM processing, motor control
Input:       g_gateEventQueue, limit semaphores, reverse timeout
Output:       Motor direction via GPIO
Initialization: Reads initial limit switches for start state
```

#### LedControlTask (Priority +2)
```
Purpose:      LED status indication
Green ON:    Gate opening
Red ON:      Gate closing
Both OFF:    Gate stopped
```

#### SafetyTask (Priority +4 - Highest)
```
Purpose:     Highest priority safety handling
Trigger:     g_obstacleSem (given by InputTask)
Action:      Sends high-priority event to g_gateEventQueue
```

#### StatusTask (Priority +1 - Lowest)
```
Purpose:     Telemetry placeholder (UART hook)
Status:     Currently just delays
```

---

## 7. Flow Diagrams

### 7.1 Complete System Flow

```
+======================================================================+
+======================================================================+
+                     HARDWARE LAYER                                  +
+======================================================================+
|   Driver Buttons      Security Buttons    Limit Switches  Obstacle  |
|   (PB0-PB1)           (PB2-PB3)            (PE1-PE2)      (PE3)     |
+---------------------------+--------------------+------------------+-----------+
                            |                    |                |
                            v                    v                v
+======================================================================+
+                        GPIO ISR LAYER                               +
+======================================================================+
|            GPIOB_Handler()          |        GPIOE_Handler()         |
|   (driver open/close)              |   (limits + obstacle)        |
|   (security open/close)            |                             |
+-----------------------------------+-------------------------------+
|            PushIsrEvent()          |        PushIsrEvent()        |
+-----------------------------------+-------------------------------+
                            |                    |
                            v                    v
+======================================================================+
+                      INPUT QUEUE (g_inputQueue)                     =
+======================================================================+
+                              |                                      |
+                              v                                      |
+======================================================================+
+                       INPUT TASK (Priority +3)                    =
+======================================================================+
|   1. Receive GpioInputEvent                                           |
|   2. Debounce check (30ms)                                            |
|   3. Convert to FSM event type                                        |
|   4. Check for conflicts (open+close pressed)                        |
|   5. Give limit/obstacle semaphores if triggered                     |
|   6. Queue event to gate queue                                        |
+-----------------------------------+-------------------------------+
                            |           |
                            v           v (obstacle)
+--------------------+       |    +-------------+
|   g_gateEventQueue |<------+-   | g_obstacle  |
+--------------------+       |    | Semaphore   |
                            |    +-------------+
                            |         |
                            |         v
+---------------------------v---------v--------------------------------+
+                    SAFETY TASK (Priority +4 - Highest)                =
+-------------------------------------------------------------------+
|   1. Take g_obstacleSem                                            |
|   2. Queue high-priority SAFETY_OBSTACLE to gate queue              |
+-------------------------------------------------------------------+
                            |
+----------------------------v----------------------------------------+
+                    GATE CONTROL TASK (Priority +2)                   =
+-------------------------------------------------------------------+
|   1. Receive FSM event                                             |
|   2. Process in FSM_ProcessEvent()                                 |
|   3. Update state machine                                         |
|   4. Determine motor direction                                     |
|   5. Call GPIO_GateMotorSetDirection()                            |
+-------------------------------------------------------------------+
                            |
+----------------------------v----------------------------------------+
+                    MOTOR CONTROL (GPIO)                             =
+-------------------------------------------------------------------+
|   Motor Direction: OPEN / CLOSE / STOP                            |
+-------------------------------------------------------------------+

+======================================================================+
+                    LED CONTROL TASK (Priority +2)                   =
+======================================================================+
|   State: GATE_STATE_OPENING    -> Green LED ON                      |
|   State: GATE_STATE_CLOSING   -> Red LED ON                         |
|   State: Other                -> Both LEDs OFF                        |
+======================================================================+
```

### 7.2 Event Processing Flow

```
+=========================================================================+
+                     BUTTON EVENT FLOW                                =
+=========================================================================+
|                                                                         |
|   [Physical Button Press/Release]                                        |
|              |                                                       |
|              v                                                       |
|   +------------------+                                                |
|   | GPIO ISR        |                                                 |
|   | GPIOB_Handler() |                                                 |
|   +--------+-------+                                                |
|            |                                                       |
|            v                                                       |
|   +------------------+                                                |
|   | PushIsrEvent()  |  Creates GpioInputEvent                       |
|   | g_inputQueue   |  with button ID + timestamp                   |
|   +--------+-------+                                                |
|            |                                                       |
|            v (portENTER_CRITICAL)                                   |
|   +------------------+                                                |
|   | InputTask       |                                                |
|   | Debounce 30ms   |  Check g_lastDebounceTick[]                  |
|   +--------+-------+                                                |
|            |                                                       |
|            v                                                       |
|   +------------------+                                                |
|   | Convert to      |  Maps button -> FSM_EVENT_*                   |
|   | FSM Event       |                                                |
|   +--------+-------+                                                |
|            |                                                       |
|            v                                                       |
|   +------------------+                                                |
|   | Check Conflict  |  Both open+close = FSM_EVENT_CONFLICT         |
|   | Check Auto Mode |  Press > 500ms = AUTO event                    |
|   +--------+-------+                                                |
|            |                                                       |
|            v                                                       |
|   +------------------+                                                |
|   | Give Semaphore  |  If LIMIT or OBSTACLE                         |
|   | g_inputQueue  |  -> g_gateEventQueue                            |
|   +--------+-------+                                                |
|            |                                                       |
|            v                                                       |
|   +------------------+                                                |
|   | GateControlTask |                                                |
|   | FSM_ProcessEvent|                                                |
|   +--------+-------+                                                |
|            |                                                       |
|            v                                                       |
|   +------------------+                                                |
|   | Update State   |  See State Transition Diagram                |
|   | Control Motor  |  GPIO_GateMotorSetDirection()              |
|   +------------------+                                                |
+=========================================================================+
```

### 7.3 Safety/Obstacle Flow

```
+=========================================================================+
+                    SAFETY EVENT FLOW                               =
+=========================================================================+
|                                                                         |
|   [Obstacle Sensor Triggered]                                        |
|              |                                                       |
|              v                                                       |
|   +------------------+                                                |
|   | GPIOE_Handler() |  OBSTACLE pin goes LOW                        |
|   +--------+-------+                                                |
|            |                                                       |
|            v                                                       |
|   +------------------+                                                |
|   | PushIsrEvent()  |  BUTTON_OBSTACLE                              |
|   | g_inputQueue   |                                                |
|   +--------+-------+                                                |
|            |                                                       |
|            v                                                       |
|   +------------------+                                                |
|   | InputTask       |                                                |
|   | xQueueReceive()  |                                                |
|   +--------+-------+                                                |
|            |                                                       |
|            v                                                       |
|   +------------------+                                                |
|   | Give Semaphore   |  xSemaphoreGive(g_obstacleSem)            |
|   +--------+-------+                                                |
|            |                                                       |
|            v                                                       |
|   +------------------+                                                |
|   | SafetyTask       |  Priority +4 (HIGHEST)                      |
|   | xSemaphoreTake()|                                                |
|   +--------+-------+                                                |
|            |                                                       |
|            v                                                       |
|   +------------------+                                                |
|   | Queue High-Prio |  xQueueSendToFront()                        |
|   | Event           |  FSM_EVENT_SAFETY_OBSTACLE                  |
|   +--------+-------+                                                |
|            |                                                       |
|            v                                                       |
|   +------------------+                                                |
|   | GateControlTask |                                                |
|   | FSM Processing  |                                                |
|   +--------+-------+                                                |
|            |                                                       |
|            v                                                       |
|   +------------------+                                                |
|   | If CLOSING       |  Transition to GATE_STATE_REVERSING         |
|   | -> REVERSING     |  Motor direction = OPEN                   |
|   +--------+-------+  Duration = 500ms                             |
|            |                                                       |
|            v (500ms)                                                |
|   +------------------+                                                |
|   | Reverse Timeout |  FSM_EVENT_REVERSE_TIMEOUT                |
|   | -> IDLE_OPEN    |  Stop motor                               |
|   +------------------+                                                |
+=========================================================================+
```

### 7.4 Limit Switch Flow

```
+=========================================================================+
+                     LIMIT SWITCH FLOW                              =
+=========================================================================+
|                                                                         |
|   [Limit Switch Triggered (OPEN or CLOSED)]                            |
|              |                                                       |
|              v                                                       |
|   +------------------+                                                |
|   | GPIOE_Handler() |  LIMIT_OPEN or LIMIT_CLOSED pin goes LOW      |
|   +--------+-------+                                                |
|            |                                                       |
|            v                                                       |
|   +------------------+                                                |
|   | PushIsrEvent()  |                                                |
|   | g_inputQueue   |                                                |
|   +--------+-------+                                                |
|            |                                                       |
|            v                                                       |
|   +------------------+                                                |
|   | InputTask       |                                                |
|   | Detect button   |                                                |
|   +--------+-------+                                                |
|            |                                                       |
|            v                                                       |
|   +------------------+                                                |
|   | Give Semaphore  |  g_limitOpenSem or g_limitClosedSem          |
|   +--------+-------+                                                |
|            |                                                       |
|            v                                                       |
|   +------------------+                                                |
|   | GateControlTask |  Takes semaphore (blocks)                     |
|   | FSM Processing  |                                                |
|   +--------+-------+                                                |
|            |                                                       |
|            v                                                       |
|   +------------------+                                                |
|   | FSM Update      |  LIMIT_OPEN   -> IDLE_OPEN                   |
|   |                |  LIMIT_CLOSED -> IDLE_CLOSED                  |
|   +--------+-------+                                                |
|            |                                                       |
|            v                                                       |
|   +------------------+                                                |
|   | Stop Motor      |  GATE_MOTOR_STOP                             |
|   +------------------+                                                |
+=========================================================================+
```

---

## 8. Inter-Task Communication

### 8.1 Queues

```
+-------------------+------+------------+--------------------------------+
| Queue Name        | Size | Type       | Flow                           |
+-------------------+------+------------+--------------------------------+
| g_inputQueue     | 24   | GpioInputEvent | ISR -> InputTask            |
+-------------------+------+------------+--------------------------------+
| g_gateEventQueue  | 24   | FsmEvent   | InputTask -> GateControlTask  |
+-------------------+------+------------+--------------------------------+
```

**Queue Flow:**
```
ISR -> g_inputQueue -> InputTask -> debounce -> convert -> g_gateEventQueue -> GateControlTask -> FSM
```

### 8.2 Semaphores

```
+---------------------+-------+------------------+--------------------+
| Semaphore           | Type  | Given By         | Taken By           |
+---------------------+-------+------------------+--------------------+
| g_limitOpenSem     | Binary| InputTask       | GateControlTask    |
+---------------------+-------+------------------+--------------------+
| g_limitClosedSem   | Binary| InputTask       | GateControlTask    |
+---------------------+-------+------------------+--------------------+
| g_obstacleSem     | Binary| InputTask       | SafetyTask         |
+---------------------+-------+------------------+--------------------+
```

### 8.3 Mutex

```
+---------------+-------+------------------------------------------+
| Mutex         | Type  | Protected Resource                       |
+---------------+-------+------------------------------------------+
| g_stateMutex  | Recursive | g_status (GateSystemStatus)         |
+---------------+-------+------------------------------------------+
```

---

## 9. Safety Features

### 9.1 Safety Mechanisms

| Feature | Trigger | Response |
|---------|---------|----------|
| Obstacle Detection | PE3 goes LOW while closing | Reverse 500ms |
| Conflict Handling | Both open+close pressed | Safe stop |
| Limit Protection | Open/closed limit reached | Stop motor |
| Priority Inversion | Security > Driver commands | Security wins |

### 9.2 Priority Rules

1. **Security Source > Driver Source**
   - If security and driver both press, security command executes
2. **Conflict = Stop**
   - If conflicting commands detected (open+close), motor stops
3. **Safety Highest Priority**
   - SafetyTask runs at highest priority (+4)

---

## 10. Quick Reference

### 10.1 Key Functions to Modify

| To Change | Edit Function | File |
|-----------|--------------|------|
| Button behavior | `InputTask` | tasks.c |
| State transitions | `FSM_ProcessEvent()` | fsm.c |
| Motor control | `GPIO_GateMotorSetDirection()` | gpio.c |
| LED behavior | `LedControlTask` | tasks.c |
| Safety response | `SafetyTask` | tasks.c |

### 10.2 Configuration Points

| Parameter | Location | Default |
|-----------|----------|---------|
| Debounce time | `g_debounceTicks` in tasks.c | 30ms |
| Auto mode threshold | `g_autoThresholdTicks` | 500ms |
| Reverse duration | `g_reverseTimeout` | 500ms |
| Queue sizes | `CREATE_QUEUE()` calls | 24 |

### 10.3 Adding New Features

#### Add new button:
1. Add button ID in `fsm.h` `ButtonId` enum
2. Add pin in `gpio.c` `GPIO_HardwareInit()`
3. Add handler in appropriate ISR
4. Update `InputTask` mapping

#### Add new state:
1. Add state in `fsm.h` `GateState` enum
2. Add transition in `FSM_ProcessEvent()`
3. Add LED behavior in `LedControlTask`

#### Add new command:
1. Add event in `fsm.h` `FsmEvent` enum
2. Handle in `FSM_ProcessEvent()`
3. Add debounce logic in `InputTask`

---

## Quick Edit Guide

### Common Changes:

**Change debounce time:**
```c
// In tasks.c
#define DEBOUNCE_TICKS_MS  30  // Change this value
```

**Change reverse duration:**
```c
// In tasks.c - change g_reverseTimeout ticks
const TickType_t g_reverseTimeout = pdMS_TO_TICKS(500);
```

**Add new GPIO pin:**
```c
// In gpio.c:
// 1. Add define
#define MY_NEW_PIN    PORTX, PIN_Y

// 2. In GPIO_HardwareInit()
GPIOPinTypeGPIOInput(PORTX, PIN_Y, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

// 3. In ISR (if needed)
// Add to appropriate GPIOs_Handler()

// 4. Read function
GPIOPinTypeGPIOInput(MY_NEW_PIN);
```

**Add new FSM state:**
```c
// In fsm.h:
typedef enum {
    GATE_STATE_IDLE_OPEN = 0,
    GATE_STATE_IDLE_CLOSED,
    GATE_STATE_OPENING,
    GATE_STATE_CLOSING,
    GATE_STATE_STOPPED_MIDWAY,
    GATE_STATE_REVERSING,
    GATE_STATE_NEW_STATE,  // Add here
    GATE_STATE_COUNT
} GateState;
```

**Add new event:**
```c
// In fsm.h:
typedef enum {
    FSM_EVENT_CMD_OPEN_PRESS = 0,
    FSM_EVENT_CMD_OPEN_RELEASE,
    FSM_EVENT_CMD_CLOSE_PRESS,
    FSM_EVENT_CMD_CLOSE_RELEASE,
    FSM_EVENT_CMD_OPEN_AUTO,
    FSM_EVENT_CMD_CLOSE_AUTO,
    FSM_EVENT_LIMIT_OPEN,
    FSM_EVENT_LIMIT_CLOSED,
    FSM_EVENT_SAFETY_OBSTACLE,
    FSM_EVENT_CONFLICT,
    FSM_EVENT_REVERSE_TIMEOUT,
    FSM_EVENT_NEW_EVENT,  // Add here
    FSM_EVENT_COUNT
} FsmEvent;
```

---

*Last Updated: April 2026*