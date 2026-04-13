# Smart Parking Garage Gate System (TM4C123GH6PM + FreeRTOS)

This template implements an event-driven, multitasking architecture for a smart parking gate controller.

## Files

- `main.c`: MCU/clock initialization, GPIO init, FreeRTOS startup
- `gpio.h` / `gpio.c`: GPIO HAL, interrupt handlers, ISR-to-queue event push
- `fsm.h` / `fsm.c`: deterministic finite-state machine and transition logic
- `tasks.h` / `tasks.c`: FreeRTOS tasks, queues, semaphores, mutex, debounce, and command handling

## RTOS Design

- Highest priority: Safety task
- High priority: Input task
- Medium priority: Gate Control + LED Control tasks
- Low priority: Status task

## Inter-task Communication

- Queue (`g_inputQueue`): raw GPIO events from ISR to Input task
- Queue (`g_gateEventQueue`): command/safety events to Gate task
- Binary semaphores: open/closed limit and obstacle signals
- Mutex (`g_stateMutex`): protects shared gate status

## FSM States

- `GATE_STATE_IDLE_OPEN`
- `GATE_STATE_IDLE_CLOSED`
- `GATE_STATE_OPENING`
- `GATE_STATE_CLOSING`
- `GATE_STATE_STOPPED_MIDWAY`
- `GATE_STATE_REVERSING`

## Behavior Implemented

- Manual mode (hold-to-run)
- One-touch auto mode (short press)
- Obstacle safety override (stop, reverse 0.5 s, stop)
- Security command priority over driver
- Conflict handling (simultaneous open/close -> safe stop)

## Notes for Hardware Bring-up

- Pin mapping in `gpio.c` is a clean default and should be adjusted to your board wiring.
- Inputs are configured as active-low with pull-ups.
- Keep ISR routines minimal and route all logic through queues/tasks.
