#include "gpio.h"

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"

#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#define LED_RED_PORT            GPIO_PORTF_BASE
#define LED_RED_PIN             GPIO_PIN_1
#define LED_GREEN_PORT          GPIO_PORTF_BASE
#define LED_GREEN_PIN           GPIO_PIN_3

#define DRIVER_OPEN_PORT        GPIO_PORTB_BASE
#define DRIVER_OPEN_PIN         GPIO_PIN_0
#define DRIVER_CLOSE_PORT       GPIO_PORTB_BASE
#define DRIVER_CLOSE_PIN        GPIO_PIN_1
#define SECURITY_OPEN_PORT      GPIO_PORTB_BASE
#define SECURITY_OPEN_PIN       GPIO_PIN_2
#define SECURITY_CLOSE_PORT     GPIO_PORTB_BASE
#define SECURITY_CLOSE_PIN      GPIO_PIN_3

#define LIMIT_OPEN_PORT         GPIO_PORTE_BASE
#define LIMIT_OPEN_PIN          GPIO_PIN_1
#define LIMIT_CLOSED_PORT       GPIO_PORTE_BASE
#define LIMIT_CLOSED_PIN        GPIO_PIN_2
#define OBSTACLE_PORT           GPIO_PORTE_BASE
#define OBSTACLE_PIN            GPIO_PIN_3

static QueueHandle_t g_inputQueueFromIsr;
static SemaphoreHandle_t g_obstacleSemFromIsr;

static inline bool IsAsserted(uint32_t portBase, uint8_t pinMask)
{
    return (GPIOPinRead(portBase, pinMask) == 0U);
}

static void PushIsrEvent(ButtonId button, uint32_t portBase, uint8_t pinMask)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    GpioInputEvent event;

    if (g_inputQueueFromIsr == 0)
    {
        return;
    }

    event.button = button;
    event.asserted = IsAsserted(portBase, pinMask);
    event.tick = xTaskGetTickCountFromISR();

    (void)xQueueSendFromISR(g_inputQueueFromIsr, &event, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void GPIO_HardwareInit(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
    {
    }
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))
    {
    }

    GPIOPinTypeGPIOOutput(LED_RED_PORT, LED_RED_PIN | LED_GREEN_PIN);
    GPIOPinWrite(LED_RED_PORT, LED_RED_PIN | LED_GREEN_PIN, 0U);

    GPIOPinTypeGPIOInput(DRIVER_OPEN_PORT,
                         DRIVER_OPEN_PIN | DRIVER_CLOSE_PIN | SECURITY_OPEN_PIN | SECURITY_CLOSE_PIN);
    GPIOPadConfigSet(DRIVER_OPEN_PORT,
                     DRIVER_OPEN_PIN | DRIVER_CLOSE_PIN | SECURITY_OPEN_PIN | SECURITY_CLOSE_PIN,
                     GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);

    GPIOPinTypeGPIOInput(LIMIT_OPEN_PORT, LIMIT_OPEN_PIN | LIMIT_CLOSED_PIN | OBSTACLE_PIN);
    GPIOPadConfigSet(LIMIT_OPEN_PORT,
                     LIMIT_OPEN_PIN | LIMIT_CLOSED_PIN | OBSTACLE_PIN,
                     GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);

    GPIOIntDisable(DRIVER_OPEN_PORT, DRIVER_OPEN_PIN | DRIVER_CLOSE_PIN | SECURITY_OPEN_PIN | SECURITY_CLOSE_PIN);
    GPIOIntDisable(LIMIT_OPEN_PORT, LIMIT_OPEN_PIN | LIMIT_CLOSED_PIN | OBSTACLE_PIN);

    GPIOIntTypeSet(DRIVER_OPEN_PORT,
                   DRIVER_OPEN_PIN | DRIVER_CLOSE_PIN | SECURITY_OPEN_PIN | SECURITY_CLOSE_PIN,
                   GPIO_BOTH_EDGES);
    GPIOIntTypeSet(LIMIT_OPEN_PORT,
                   LIMIT_OPEN_PIN | LIMIT_CLOSED_PIN | OBSTACLE_PIN,
                   GPIO_BOTH_EDGES);

    GPIOIntClear(DRIVER_OPEN_PORT, DRIVER_OPEN_PIN | DRIVER_CLOSE_PIN | SECURITY_OPEN_PIN | SECURITY_CLOSE_PIN);
    GPIOIntClear(LIMIT_OPEN_PORT, LIMIT_OPEN_PIN | LIMIT_CLOSED_PIN | OBSTACLE_PIN);

    GPIOIntEnable(DRIVER_OPEN_PORT, DRIVER_OPEN_PIN | DRIVER_CLOSE_PIN | SECURITY_OPEN_PIN | SECURITY_CLOSE_PIN);
    GPIOIntEnable(LIMIT_OPEN_PORT, LIMIT_OPEN_PIN | LIMIT_CLOSED_PIN | OBSTACLE_PIN);

    IntEnable(INT_GPIOB);
    IntEnable(INT_GPIOE);
}

void GPIO_RegisterInputQueue(QueueHandle_t queueHandle)
{
    g_inputQueueFromIsr = queueHandle;
}

void GPIO_RegisterObstacleSemaphore(SemaphoreHandle_t semHandle)
{
    g_obstacleSemFromIsr = semHandle;
}

void GPIO_SetGreenLed(bool on)
{
    GPIOPinWrite(LED_GREEN_PORT, LED_GREEN_PIN, on ? LED_GREEN_PIN : 0U);
}

void GPIO_SetRedLed(bool on)
{
    GPIOPinWrite(LED_RED_PORT, LED_RED_PIN, on ? LED_RED_PIN : 0U);
}

void GPIO_GateMotorSetDirection(GateMotorDirection direction)
{
    switch (direction)
    {
        case GATE_MOTOR_OPEN:
            GPIO_SetGreenLed(true);
            GPIO_SetRedLed(false);
            break;

        case GATE_MOTOR_CLOSE:
            GPIO_SetGreenLed(false);
            GPIO_SetRedLed(true);
            break;

        case GATE_MOTOR_STOP:
        default:
            GPIO_SetGreenLed(false);
            GPIO_SetRedLed(false);
            break;
    }
}

bool GPIO_IsOpenLimitActive(void)
{
    return IsAsserted(LIMIT_OPEN_PORT, LIMIT_OPEN_PIN);
}

bool GPIO_IsClosedLimitActive(void)
{
    return IsAsserted(LIMIT_CLOSED_PORT, LIMIT_CLOSED_PIN);
}

void GPIOB_Handler(void)
{
    uint32_t status = GPIOIntStatus(DRIVER_OPEN_PORT, true);
    GPIOIntClear(DRIVER_OPEN_PORT, status);

    if ((status & DRIVER_OPEN_PIN) != 0U)
    {
        PushIsrEvent(BUTTON_DRIVER_OPEN, DRIVER_OPEN_PORT, DRIVER_OPEN_PIN);
    }

    if ((status & DRIVER_CLOSE_PIN) != 0U)
    {
        PushIsrEvent(BUTTON_DRIVER_CLOSE, DRIVER_CLOSE_PORT, DRIVER_CLOSE_PIN);
    }

    if ((status & SECURITY_OPEN_PIN) != 0U)
    {
        PushIsrEvent(BUTTON_SECURITY_OPEN, SECURITY_OPEN_PORT, SECURITY_OPEN_PIN);
    }

    if ((status & SECURITY_CLOSE_PIN) != 0U)
    {
        PushIsrEvent(BUTTON_SECURITY_CLOSE, SECURITY_CLOSE_PORT, SECURITY_CLOSE_PIN);
    }
}

void GPIOE_Handler(void)
{
    uint32_t status = GPIOIntStatus(LIMIT_OPEN_PORT, true);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    GPIOIntClear(LIMIT_OPEN_PORT, status);

    if ((status & LIMIT_OPEN_PIN) != 0U)
    {
        PushIsrEvent(BUTTON_LIMIT_OPEN, LIMIT_OPEN_PORT, LIMIT_OPEN_PIN);
    }

    if ((status & LIMIT_CLOSED_PIN) != 0U)
    {
        PushIsrEvent(BUTTON_LIMIT_CLOSED, LIMIT_CLOSED_PORT, LIMIT_CLOSED_PIN);
    }

    if ((status & OBSTACLE_PIN) != 0U)
    {
        /* Route obstacle directly to SafetyTask to avoid input-queue latency. */
        if (g_obstacleSemFromIsr != 0)
        {
            (void)xSemaphoreGiveFromISR(g_obstacleSemFromIsr, &xHigherPriorityTaskWoken);
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
