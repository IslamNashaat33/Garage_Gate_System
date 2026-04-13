#ifndef FSM_H
#define FSM_H

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

typedef enum
{
    GATE_STATE_IDLE_OPEN = 0,
    GATE_STATE_IDLE_CLOSED,
    GATE_STATE_OPENING,
    GATE_STATE_CLOSING,
    GATE_STATE_STOPPED_MIDWAY,
    GATE_STATE_REVERSING
} GateState;

typedef enum
{
    GATE_MOTOR_STOP = 0,
    GATE_MOTOR_OPEN,
    GATE_MOTOR_CLOSE
} GateMotorDirection;

typedef enum
{
    CMD_SOURCE_DRIVER = 0,
    CMD_SOURCE_SECURITY,
    CMD_SOURCE_NONE
} CommandSource;

typedef enum
{
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
    FSM_EVENT_REVERSE_TIMEOUT
} FsmEventType;

typedef struct
{
    FsmEventType type;
    CommandSource source;
    TickType_t tick;
} FsmEvent;

typedef struct
{
    GateState state;
    GateMotorDirection motorDirection;

    bool openHeld[2];
    bool closeHeld[2];

    bool autoActive[2];
    GateMotorDirection autoDirection[2];

    TickType_t reverseDeadlineTick;
} FsmContext;

void FSM_Init(FsmContext *ctx, GateState initialState);
void FSM_ProcessEvent(FsmContext *ctx, const FsmEvent *event, TickType_t nowTick);

#endif
