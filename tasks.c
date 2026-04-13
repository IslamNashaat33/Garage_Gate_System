#include "tasks.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "fsm.h"
#include "gpio.h"

#define INPUT_QUEUE_LENGTH            (24U)
#define EVENT_QUEUE_LENGTH            (24U)
#define DEBOUNCE_TICKS                pdMS_TO_TICKS(30U)
#define ONE_TOUCH_MAX_TICKS           pdMS_TO_TICKS(250U)
#define GATE_TASK_LOOP_TICKS          pdMS_TO_TICKS(10U)
#define LED_TASK_PERIOD_TICKS         pdMS_TO_TICKS(20U)
#define STATUS_TASK_PERIOD_TICKS      pdMS_TO_TICKS(500U)

#define PRIORITY_SAFETY_TASK          (tskIDLE_PRIORITY + 4U)
#define PRIORITY_INPUT_TASK           (tskIDLE_PRIORITY + 3U)
#define PRIORITY_GATE_TASK            (tskIDLE_PRIORITY + 2U)
#define PRIORITY_LED_TASK             (tskIDLE_PRIORITY + 2U)
#define PRIORITY_STATUS_TASK          (tskIDLE_PRIORITY + 1U)

typedef enum
{
    DIR_OPEN = 0,
    DIR_CLOSE
} ControlDirection;

static QueueHandle_t g_inputQueue;
static QueueHandle_t g_gateEventQueue;

static SemaphoreHandle_t g_limitOpenSem;
static SemaphoreHandle_t g_limitClosedSem;
static SemaphoreHandle_t g_obstacleSem;
static SemaphoreHandle_t g_stateMutex;

static GateSystemStatus g_status;
static FsmContext g_fsm;

static TickType_t g_lastDebounceTick[BUTTON_COUNT];
static TickType_t g_pressTick[2][2];
static bool g_pressedState[2][2];

static void InputTask(void *params);
static void GateControlTask(void *params);
static void LedControlTask(void *params);
static void SafetyTask(void *params);
static void StatusTask(void *params);

static inline bool IsCommandButton(ButtonId button)
{
    return (button == BUTTON_DRIVER_OPEN) ||
           (button == BUTTON_DRIVER_CLOSE) ||
           (button == BUTTON_SECURITY_OPEN) ||
           (button == BUTTON_SECURITY_CLOSE);
}

static inline CommandSource ButtonToSource(ButtonId button)
{
    if ((button == BUTTON_DRIVER_OPEN) || (button == BUTTON_DRIVER_CLOSE))
    {
        return CMD_SOURCE_DRIVER;
    }

    return CMD_SOURCE_SECURITY;
}

static inline ControlDirection ButtonToDirection(ButtonId button)
{
    if ((button == BUTTON_DRIVER_OPEN) || (button == BUTTON_SECURITY_OPEN))
    {
        return DIR_OPEN;
    }

    return DIR_CLOSE;
}

static void UpdateSharedStatus(void)
{
    if (xSemaphoreTake(g_stateMutex, 0U) == pdTRUE)
    {
        g_status.state = g_fsm.state;
        g_status.motorDirection = g_fsm.motorDirection;

        if (g_fsm.openHeld[CMD_SOURCE_SECURITY] ||
            g_fsm.closeHeld[CMD_SOURCE_SECURITY] ||
            g_fsm.autoActive[CMD_SOURCE_SECURITY])
        {
            g_status.activeSource = CMD_SOURCE_SECURITY;
        }
        else if (g_fsm.openHeld[CMD_SOURCE_DRIVER] ||
                 g_fsm.closeHeld[CMD_SOURCE_DRIVER] ||
                 g_fsm.autoActive[CMD_SOURCE_DRIVER])
        {
            g_status.activeSource = CMD_SOURCE_DRIVER;
        }
        else
        {
            g_status.activeSource = CMD_SOURCE_NONE;
        }

        (void)xSemaphoreGive(g_stateMutex);
    }
}

static void QueueGateEvent(FsmEventType type, CommandSource source, TickType_t tick)
{
    FsmEvent event;

    event.type = type;
    event.source = source;
    event.tick = tick;

    (void)xQueueSend(g_gateEventQueue, &event, 0U);
}

static void CheckAndQueueConflict(TickType_t tick)
{
    bool anyOpenPressed = g_pressedState[CMD_SOURCE_DRIVER][DIR_OPEN] ||
                          g_pressedState[CMD_SOURCE_SECURITY][DIR_OPEN];
    bool anyClosePressed = g_pressedState[CMD_SOURCE_DRIVER][DIR_CLOSE] ||
                           g_pressedState[CMD_SOURCE_SECURITY][DIR_CLOSE];

    if (anyOpenPressed && anyClosePressed)
    {
        QueueGateEvent(FSM_EVENT_CONFLICT, CMD_SOURCE_NONE, tick);
    }
}

static void InputTask(void *params)
{
    GpioInputEvent inputEvent;
    (void)params;

    for (;;)
    {
        if (xQueueReceive(g_inputQueue, &inputEvent, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        if ((inputEvent.tick - g_lastDebounceTick[inputEvent.button]) < DEBOUNCE_TICKS)
        {
            continue;
        }

        g_lastDebounceTick[inputEvent.button] = inputEvent.tick;

        if (IsCommandButton(inputEvent.button))
        {
            CommandSource source = ButtonToSource(inputEvent.button);
            ControlDirection direction = ButtonToDirection(inputEvent.button);
            uint32_t sourceIdx = (uint32_t)source;
            uint32_t dirIdx = (uint32_t)direction;

            if (inputEvent.asserted)
            {
                g_pressTick[sourceIdx][dirIdx] = inputEvent.tick;
                g_pressedState[sourceIdx][dirIdx] = true;
                CheckAndQueueConflict(inputEvent.tick);

                if (direction == DIR_OPEN)
                {
                    QueueGateEvent(FSM_EVENT_CMD_OPEN_PRESS, source, inputEvent.tick);
                }
                else
                {
                    QueueGateEvent(FSM_EVENT_CMD_CLOSE_PRESS, source, inputEvent.tick);
                }
            }
            else
            {
                TickType_t heldTicks = inputEvent.tick - g_pressTick[sourceIdx][dirIdx];
                g_pressedState[sourceIdx][dirIdx] = false;

                if (direction == DIR_OPEN)
                {
                    QueueGateEvent(FSM_EVENT_CMD_OPEN_RELEASE, source, inputEvent.tick);

                    if (heldTicks <= ONE_TOUCH_MAX_TICKS)
                    {
                        QueueGateEvent(FSM_EVENT_CMD_OPEN_AUTO, source, inputEvent.tick);
                    }
                }
                else
                {
                    QueueGateEvent(FSM_EVENT_CMD_CLOSE_RELEASE, source, inputEvent.tick);

                    if (heldTicks <= ONE_TOUCH_MAX_TICKS)
                    {
                        QueueGateEvent(FSM_EVENT_CMD_CLOSE_AUTO, source, inputEvent.tick);
                    }
                }
            }
            continue;
        }

        if (inputEvent.asserted)
        {
            if (inputEvent.button == BUTTON_LIMIT_OPEN)
            {
                (void)xSemaphoreGive(g_limitOpenSem);
            }
            else if (inputEvent.button == BUTTON_LIMIT_CLOSED)
            {
                (void)xSemaphoreGive(g_limitClosedSem);
            }
            else if (inputEvent.button == BUTTON_OBSTACLE)
            {
                (void)xSemaphoreGive(g_obstacleSem);
            }
        }
    }
}

static void GateControlTask(void *params)
{
    FsmEvent event;
    TickType_t nowTick;
    GateMotorDirection previousDirection;

    (void)params;

    FSM_Init(&g_fsm, GPIO_IsClosedLimitActive() ? GATE_STATE_IDLE_CLOSED : GATE_STATE_STOPPED_MIDWAY);

    if (g_fsm.state == GATE_STATE_IDLE_CLOSED)
    {
        g_status.state = GATE_STATE_IDLE_CLOSED;
    }
    else
    {
        g_status.state = GATE_STATE_STOPPED_MIDWAY;
    }

    g_status.motorDirection = GATE_MOTOR_STOP;
    g_status.activeSource = CMD_SOURCE_NONE;

    for (;;)
    {
        previousDirection = g_fsm.motorDirection;
        nowTick = xTaskGetTickCount();

        if (xSemaphoreTake(g_limitOpenSem, 0U) == pdTRUE)
        {
            event.type = FSM_EVENT_LIMIT_OPEN;
            event.source = CMD_SOURCE_NONE;
            event.tick = nowTick;
            FSM_ProcessEvent(&g_fsm, &event, nowTick);
        }

        if (xSemaphoreTake(g_limitClosedSem, 0U) == pdTRUE)
        {
            event.type = FSM_EVENT_LIMIT_CLOSED;
            event.source = CMD_SOURCE_NONE;
            event.tick = nowTick;
            FSM_ProcessEvent(&g_fsm, &event, nowTick);
        }

        if (g_fsm.state == GATE_STATE_REVERSING && nowTick >= g_fsm.reverseDeadlineTick)
        {
            event.type = FSM_EVENT_REVERSE_TIMEOUT;
            event.source = CMD_SOURCE_NONE;
            event.tick = nowTick;
            FSM_ProcessEvent(&g_fsm, &event, nowTick);
        }

        if (xQueueReceive(g_gateEventQueue, &event, GATE_TASK_LOOP_TICKS) == pdTRUE)
        {
            FSM_ProcessEvent(&g_fsm, &event, nowTick);
        }

        if (previousDirection != g_fsm.motorDirection)
        {
            GPIO_GateMotorSetDirection(g_fsm.motorDirection);
        }

        UpdateSharedStatus();
    }
}

static void LedControlTask(void *params)
{
    GateSystemStatus localStatus;
    (void)params;

    for (;;)
    {
        if (xSemaphoreTake(g_stateMutex, portMAX_DELAY) == pdTRUE)
        {
            localStatus = g_status;
            (void)xSemaphoreGive(g_stateMutex);

            if (localStatus.motorDirection == GATE_MOTOR_OPEN)
            {
                GPIO_SetGreenLed(true);
                GPIO_SetRedLed(false);
            }
            else if (localStatus.motorDirection == GATE_MOTOR_CLOSE)
            {
                GPIO_SetGreenLed(false);
                GPIO_SetRedLed(true);
            }
            else
            {
                GPIO_SetGreenLed(false);
                GPIO_SetRedLed(false);
            }
        }

        vTaskDelay(LED_TASK_PERIOD_TICKS);
    }
}

static void SafetyTask(void *params)
{
    FsmEvent event;
    (void)params;

    event.type = FSM_EVENT_SAFETY_OBSTACLE;
    event.source = CMD_SOURCE_NONE;

    for (;;)
    {
        if (xSemaphoreTake(g_obstacleSem, portMAX_DELAY) == pdTRUE)
        {
            event.tick = xTaskGetTickCount();
            (void)xQueueSendToFront(g_gateEventQueue, &event, 0U);
        }
    }
}

static void StatusTask(void *params)
{
    (void)params;

    for (;;)
    {
        /* Hook for telemetry/UART diagnostics if required. */
        vTaskDelay(STATUS_TASK_PERIOD_TICKS);
    }
}

bool Tasks_Init(void)
{
    uint32_t i;
    uint32_t j;

    g_inputQueue = xQueueCreate(INPUT_QUEUE_LENGTH, sizeof(GpioInputEvent));
    g_gateEventQueue = xQueueCreate(EVENT_QUEUE_LENGTH, sizeof(FsmEvent));

    g_limitOpenSem = xSemaphoreCreateBinary();
    g_limitClosedSem = xSemaphoreCreateBinary();
    g_obstacleSem = xSemaphoreCreateBinary();
    g_stateMutex = xSemaphoreCreateMutex();

    if ((g_inputQueue == 0) ||
        (g_gateEventQueue == 0) ||
        (g_limitOpenSem == 0) ||
        (g_limitClosedSem == 0) ||
        (g_obstacleSem == 0) ||
        (g_stateMutex == 0))
    {
        return false;
    }

    for (i = 0U; i < BUTTON_COUNT; i++)
    {
        g_lastDebounceTick[i] = 0U;
    }

    for (i = 0U; i < 2U; i++)
    {
        for (j = 0U; j < 2U; j++)
        {
            g_pressTick[i][j] = 0U;
            g_pressedState[i][j] = false;
        }
    }

    GPIO_RegisterInputQueue(g_inputQueue);

    if (xTaskCreate(SafetyTask, "Safety", configMINIMAL_STACK_SIZE + 96U, 0, PRIORITY_SAFETY_TASK, 0) != pdPASS)
    {
        return false;
    }

    if (xTaskCreate(InputTask, "Input", configMINIMAL_STACK_SIZE + 128U, 0, PRIORITY_INPUT_TASK, 0) != pdPASS)
    {
        return false;
    }

    if (xTaskCreate(GateControlTask, "Gate", configMINIMAL_STACK_SIZE + 128U, 0, PRIORITY_GATE_TASK, 0) != pdPASS)
    {
        return false;
    }

    if (xTaskCreate(LedControlTask, "LED", configMINIMAL_STACK_SIZE + 96U, 0, PRIORITY_LED_TASK, 0) != pdPASS)
    {
        return false;
    }

    if (xTaskCreate(StatusTask, "Status", configMINIMAL_STACK_SIZE + 96U, 0, PRIORITY_STATUS_TASK, 0) != pdPASS)
    {
        return false;
    }

    return true;
}
