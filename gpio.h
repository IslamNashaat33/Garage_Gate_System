#ifndef GPIO_H
#define GPIO_H

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#include "fsm.h"

typedef enum
{
    BUTTON_DRIVER_OPEN = 0,
    BUTTON_DRIVER_CLOSE,
    BUTTON_SECURITY_OPEN,
    BUTTON_SECURITY_CLOSE,
    BUTTON_LIMIT_OPEN,
    BUTTON_LIMIT_CLOSED,
    BUTTON_OBSTACLE,
    BUTTON_COUNT
} ButtonId;

typedef struct
{
    ButtonId button;
    bool asserted;
    TickType_t tick;
} GpioInputEvent;

void GPIO_HardwareInit(void);
void GPIO_RegisterInputQueue(QueueHandle_t queueHandle);
void GPIO_RegisterObstacleSemaphore(SemaphoreHandle_t semHandle);

void GPIO_SetGreenLed(bool on);
void GPIO_SetRedLed(bool on);
void GPIO_GateMotorSetDirection(GateMotorDirection direction);

bool GPIO_IsOpenLimitActive(void);
bool GPIO_IsClosedLimitActive(void);

#endif
