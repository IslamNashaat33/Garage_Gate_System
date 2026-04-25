#include "fsm.h"

#define SOURCE_COUNT             (2U)
#define REVERSE_DURATION_TICKS   pdMS_TO_TICKS(500U)

static bool SourceHasAnyCommand(const FsmContext *ctx, CommandSource source)
{
    uint32_t idx = (uint32_t)source;

    return ctx->openHeld[idx] ||
           ctx->closeHeld[idx] ||
           ctx->autoActive[idx];
}

static CommandSource GetEffectiveSource(const FsmContext *ctx)
{
    if (SourceHasAnyCommand(ctx, CMD_SOURCE_SECURITY))
    {
        return CMD_SOURCE_SECURITY;
    }

    if (SourceHasAnyCommand(ctx, CMD_SOURCE_DRIVER))
    {
        return CMD_SOURCE_DRIVER;
    }

    return CMD_SOURCE_NONE;
}

static void ClearAutoAll(FsmContext *ctx)
{
    uint32_t i;

    for (i = 0U; i < SOURCE_COUNT; i++)
    {
        ctx->autoActive[i] = false;
        ctx->autoDirection[i] = GATE_MOTOR_STOP;
    }
}

static void ClearDriverCommands(FsmContext *ctx)
{
    uint32_t idx = (uint32_t)CMD_SOURCE_DRIVER;

    ctx->openHeld[idx] = false;
    ctx->closeHeld[idx] = false;
    ctx->autoActive[idx] = false;
    ctx->autoDirection[idx] = GATE_MOTOR_STOP;
}

static void StopSafely(FsmContext *ctx, GateState targetState)
{
    ctx->state = targetState;
    ctx->motorDirection = GATE_MOTOR_STOP;
}

static void ResolveMotionFromCommands(FsmContext *ctx)
{
    CommandSource src = GetEffectiveSource(ctx);

    if (src == CMD_SOURCE_NONE)
    {
        if (ctx->state == GATE_STATE_OPENING ||
            ctx->state == GATE_STATE_CLOSING ||
            ctx->state == GATE_STATE_STOPPED_MIDWAY)
        {
            StopSafely(ctx, GATE_STATE_STOPPED_MIDWAY);
        }
        return;
    }

    uint32_t idx = (uint32_t)src;
    bool openHeld = ctx->openHeld[idx];
    bool closeHeld = ctx->closeHeld[idx];

    if (openHeld && closeHeld)
    {
        ClearAutoAll(ctx);
        StopSafely(ctx, GATE_STATE_STOPPED_MIDWAY);
        return;
    }

    if (openHeld)
    {
        ctx->state = GATE_STATE_OPENING;
        ctx->motorDirection = GATE_MOTOR_OPEN;
        return;
    }

    if (closeHeld)
    {
        ctx->state = GATE_STATE_CLOSING;
        ctx->motorDirection = GATE_MOTOR_CLOSE;
        return;
    }

    if (ctx->autoActive[idx])
    {
        if (ctx->autoDirection[idx] == GATE_MOTOR_OPEN)
        {
            ctx->state = GATE_STATE_OPENING;
            ctx->motorDirection = GATE_MOTOR_OPEN;
            return;
        }

        if (ctx->autoDirection[idx] == GATE_MOTOR_CLOSE)
        {
            ctx->state = GATE_STATE_CLOSING;
            ctx->motorDirection = GATE_MOTOR_CLOSE;
            return;
        }
    }

    StopSafely(ctx, GATE_STATE_STOPPED_MIDWAY);
}

void FSM_Init(FsmContext *ctx, GateState initialState)
{
    uint32_t i;

    ctx->state = initialState;
    ctx->motorDirection = GATE_MOTOR_STOP;
    ctx->reverseDeadlineTick = 0U;

    for (i = 0U; i < SOURCE_COUNT; i++)
    {
        ctx->openHeld[i] = false;
        ctx->closeHeld[i] = false;
        ctx->autoActive[i] = false;
        ctx->autoDirection[i] = GATE_MOTOR_STOP;
    }
}

void FSM_ProcessEvent(FsmContext *ctx, const FsmEvent *event, TickType_t nowTick)
{
    if (event == 0)
    {
        return;
    }

    if (event->type == FSM_EVENT_SAFETY_OBSTACLE)
    {
        /* Safety preempts any pending/held driver intent immediately. */
        ClearDriverCommands(ctx);

        if (ctx->motorDirection == GATE_MOTOR_CLOSE ||
            ctx->state == GATE_STATE_CLOSING)
        {
            ClearAutoAll(ctx);
            ctx->state = GATE_STATE_REVERSING;
            ctx->motorDirection = GATE_MOTOR_OPEN;
            ctx->reverseDeadlineTick = nowTick + REVERSE_DURATION_TICKS;
        }
        return;
    }

    if (event->type == FSM_EVENT_LIMIT_OPEN)
    {
        ClearAutoAll(ctx);
        ctx->state = GATE_STATE_IDLE_OPEN;
        ctx->motorDirection = GATE_MOTOR_STOP;
        return;
    }

    if (event->type == FSM_EVENT_LIMIT_CLOSED)
    {
        ClearAutoAll(ctx);
        ctx->state = GATE_STATE_IDLE_CLOSED;
        ctx->motorDirection = GATE_MOTOR_STOP;
        return;
    }

    if (event->type == FSM_EVENT_CONFLICT)
    {
        ClearAutoAll(ctx);
        StopSafely(ctx, GATE_STATE_STOPPED_MIDWAY);
        return;
    }

    if (ctx->state == GATE_STATE_REVERSING)
    {
        if (event->type == FSM_EVENT_REVERSE_TIMEOUT)
        {
            StopSafely(ctx, GATE_STATE_STOPPED_MIDWAY);
        }
        return;
    }

    switch (event->type)
    {
        case FSM_EVENT_CMD_OPEN_PRESS:
            if (event->source != CMD_SOURCE_NONE)
            {
                ctx->openHeld[(uint32_t)event->source] = true;
                ctx->autoActive[(uint32_t)event->source] = false;
                ctx->autoDirection[(uint32_t)event->source] = GATE_MOTOR_STOP;
            }
            break;

        case FSM_EVENT_CMD_OPEN_RELEASE:
            if (event->source != CMD_SOURCE_NONE)
            {
                ctx->openHeld[(uint32_t)event->source] = false;
            }
            break;

        case FSM_EVENT_CMD_CLOSE_PRESS:
            if (event->source != CMD_SOURCE_NONE)
            {
                ctx->closeHeld[(uint32_t)event->source] = true;
                ctx->autoActive[(uint32_t)event->source] = false;
                ctx->autoDirection[(uint32_t)event->source] = GATE_MOTOR_STOP;
            }
            break;

        case FSM_EVENT_CMD_CLOSE_RELEASE:
            if (event->source != CMD_SOURCE_NONE)
            {
                ctx->closeHeld[(uint32_t)event->source] = false;
            }
            break;

        case FSM_EVENT_CMD_OPEN_AUTO:
            if (event->source != CMD_SOURCE_NONE)
            {
                ctx->autoActive[(uint32_t)event->source] = true;
                ctx->autoDirection[(uint32_t)event->source] = GATE_MOTOR_OPEN;
            }
            break;

        case FSM_EVENT_CMD_CLOSE_AUTO:
            if (event->source != CMD_SOURCE_NONE)
            {
                ctx->autoActive[(uint32_t)event->source] = true;
                ctx->autoDirection[(uint32_t)event->source] = GATE_MOTOR_CLOSE;
            }
            break;

        default:
            break;
    }

    ResolveMotionFromCommands(ctx);
}
