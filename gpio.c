#include "gpio.h"

#include <stdint.h>
#include <stdbool.h>

#include "RTE_Components.h"
#include CMSIS_device_header

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#define GPIO_PORTB_BASE_ADDR      (0x40005000UL)
#define GPIO_PORTD_BASE_ADDR      (0x40007000UL)
#define GPIO_PORTE_BASE_ADDR      (0x40024000UL)
#define GPIO_PORTF_BASE_ADDR      (0x40025000UL)

#define GPIO_DATA_OFFSET          (0x3FCUL)
#define GPIO_DIR_OFFSET           (0x400UL)
#define GPIO_IS_OFFSET            (0x404UL)
#define GPIO_IBE_OFFSET           (0x408UL)
#define GPIO_IM_OFFSET            (0x410UL)
#define GPIO_MIS_OFFSET           (0x418UL)
#define GPIO_ICR_OFFSET           (0x41CUL)
#define GPIO_AFSEL_OFFSET         (0x420UL)
#define GPIO_DR2R_OFFSET          (0x500UL)
#define GPIO_PUR_OFFSET           (0x510UL)
#define GPIO_DEN_OFFSET           (0x51CUL)
#define GPIO_AMSEL_OFFSET         (0x528UL)

#define LED_RED_PORT              GPIO_PORTF_BASE_ADDR
#define LED_RED_PIN               (1UL << 1)
#define LED_GREEN_PORT            GPIO_PORTF_BASE_ADDR
#define LED_GREEN_PIN             (1UL << 3)

#define DRIVER_OPEN_PORT          GPIO_PORTB_BASE_ADDR
#define DRIVER_OPEN_PIN           (1UL << 0)
#define DRIVER_CLOSE_PORT         GPIO_PORTB_BASE_ADDR
#define DRIVER_CLOSE_PIN          (1UL << 1)
#define SECURITY_OPEN_PORT        GPIO_PORTD_BASE_ADDR
#define SECURITY_OPEN_PIN         (1UL << 0)
#define SECURITY_CLOSE_PORT       GPIO_PORTD_BASE_ADDR
#define SECURITY_CLOSE_PIN        (1UL << 1)

#define LIMIT_OPEN_PORT           GPIO_PORTE_BASE_ADDR
#define LIMIT_OPEN_PIN            (1UL << 1)
#define LIMIT_CLOSED_PORT         GPIO_PORTE_BASE_ADDR
#define LIMIT_CLOSED_PIN          (1UL << 2)
#define OBSTACLE_PORT             GPIO_PORTE_BASE_ADDR
#define OBSTACLE_PIN              (1UL << 3)

#define SYSCTL_RCGCGPIO_PORTB     (1UL << 1)
#define SYSCTL_RCGCGPIO_PORTD     (1UL << 3)
#define SYSCTL_RCGCGPIO_PORTE     (1UL << 4)
#define SYSCTL_RCGCGPIO_PORTF     (1UL << 5)

#define IRQ_GPIO_PORTB            (1UL)
#define IRQ_GPIO_PORTD            (3UL)
#define IRQ_GPIO_PORTE            (4UL)

#define REG32(addr)               (*((volatile uint32_t *)(addr)))

#define GPIO_REG(base, offset)    REG32((base) + (offset))
#define GPIO_DATA(base)           GPIO_REG((base), GPIO_DATA_OFFSET)
#define GPIO_DIR(base)            GPIO_REG((base), GPIO_DIR_OFFSET)
#define GPIO_IS(base)             GPIO_REG((base), GPIO_IS_OFFSET)
#define GPIO_IBE(base)            GPIO_REG((base), GPIO_IBE_OFFSET)
#define GPIO_IM(base)             GPIO_REG((base), GPIO_IM_OFFSET)
#define GPIO_MIS(base)            GPIO_REG((base), GPIO_MIS_OFFSET)
#define GPIO_ICR(base)            GPIO_REG((base), GPIO_ICR_OFFSET)
#define GPIO_AFSEL(base)          GPIO_REG((base), GPIO_AFSEL_OFFSET)
#define GPIO_DR2R(base)           GPIO_REG((base), GPIO_DR2R_OFFSET)
#define GPIO_PUR(base)            GPIO_REG((base), GPIO_PUR_OFFSET)
#define GPIO_DEN(base)            GPIO_REG((base), GPIO_DEN_OFFSET)
#define GPIO_AMSEL(base)          GPIO_REG((base), GPIO_AMSEL_OFFSET)

static QueueHandle_t g_inputQueueFromIsr;
static SemaphoreHandle_t g_obstacleSemFromIsr;

static inline bool IsAsserted(uint32_t portBase, uint8_t pinMask)
{
    return ((GPIO_DATA(portBase) & pinMask) == 0U);
}

static void ConfigureGpioOutput(uint32_t portBase, uint32_t pinMask)
{
    GPIO_AFSEL(portBase) &= ~pinMask;
    GPIO_AMSEL(portBase) &= ~pinMask;
    GPIO_DIR(portBase) |= pinMask;
    GPIO_DR2R(portBase) |= pinMask;
    GPIO_DEN(portBase) |= pinMask;
}

static void ConfigureGpioInputPullUp(uint32_t portBase, uint32_t pinMask)
{
    GPIO_AFSEL(portBase) &= ~pinMask;
    GPIO_AMSEL(portBase) &= ~pinMask;
    GPIO_DIR(portBase) &= ~pinMask;
    GPIO_PUR(portBase) |= pinMask;
    GPIO_DEN(portBase) |= pinMask;
}

static void ConfigureGpioInterruptBothEdges(uint32_t portBase, uint32_t pinMask)
{
    GPIO_IM(portBase) &= ~pinMask;
    GPIO_IS(portBase) &= ~pinMask;
    GPIO_IBE(portBase) |= pinMask;
    GPIO_ICR(portBase) = pinMask;
    GPIO_IM(portBase) |= pinMask;
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
    uint32_t clockMask = SYSCTL_RCGCGPIO_PORTB |
                         SYSCTL_RCGCGPIO_PORTD |
                         SYSCTL_RCGCGPIO_PORTE |
                         SYSCTL_RCGCGPIO_PORTF;

    SYSCTL->RCGCGPIO |= clockMask;

    while ((SYSCTL->PRGPIO & clockMask) != clockMask)
    {
    }

    ConfigureGpioOutput(LED_RED_PORT, LED_RED_PIN | LED_GREEN_PIN);
    GPIO_DATA(LED_RED_PORT) &= ~(LED_RED_PIN | LED_GREEN_PIN);

    ConfigureGpioInputPullUp(DRIVER_OPEN_PORT, DRIVER_OPEN_PIN | DRIVER_CLOSE_PIN);
    ConfigureGpioInputPullUp(SECURITY_OPEN_PORT, SECURITY_OPEN_PIN | SECURITY_CLOSE_PIN);

    ConfigureGpioInputPullUp(LIMIT_OPEN_PORT, LIMIT_OPEN_PIN | LIMIT_CLOSED_PIN | OBSTACLE_PIN);

    ConfigureGpioInterruptBothEdges(DRIVER_OPEN_PORT, DRIVER_OPEN_PIN | DRIVER_CLOSE_PIN);
    ConfigureGpioInterruptBothEdges(SECURITY_OPEN_PORT, SECURITY_OPEN_PIN | SECURITY_CLOSE_PIN);
    ConfigureGpioInterruptBothEdges(LIMIT_OPEN_PORT,
                                    LIMIT_OPEN_PIN | LIMIT_CLOSED_PIN | OBSTACLE_PIN);

    NVIC->ICPR[0] = (1UL << IRQ_GPIO_PORTB) | (1UL << IRQ_GPIO_PORTD) | (1UL << IRQ_GPIO_PORTE);
    NVIC->ISER[0] = (1UL << IRQ_GPIO_PORTB) | (1UL << IRQ_GPIO_PORTD) | (1UL << IRQ_GPIO_PORTE);
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
    if (on)
    {
        GPIO_DATA(LED_GREEN_PORT) |= LED_GREEN_PIN;
    }
    else
    {
        GPIO_DATA(LED_GREEN_PORT) &= ~LED_GREEN_PIN;
    }
}

void GPIO_SetRedLed(bool on)
{
    if (on)
    {
        GPIO_DATA(LED_RED_PORT) |= LED_RED_PIN;
    }
    else
    {
        GPIO_DATA(LED_RED_PORT) &= ~LED_RED_PIN;
    }
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
    uint32_t status = GPIO_MIS(DRIVER_OPEN_PORT);
    GPIO_ICR(DRIVER_OPEN_PORT) = status;

    if ((status & DRIVER_OPEN_PIN) != 0U)
    {
        PushIsrEvent(BUTTON_DRIVER_OPEN, DRIVER_OPEN_PORT, DRIVER_OPEN_PIN);
    }

    if ((status & DRIVER_CLOSE_PIN) != 0U)
    {
        PushIsrEvent(BUTTON_DRIVER_CLOSE, DRIVER_CLOSE_PORT, DRIVER_CLOSE_PIN);
    }
}

void GPIOD_Handler(void)
{
    uint32_t status = GPIO_MIS(SECURITY_OPEN_PORT);
    GPIO_ICR(SECURITY_OPEN_PORT) = status;

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
    uint32_t status = GPIO_MIS(LIMIT_OPEN_PORT);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    GPIO_ICR(LIMIT_OPEN_PORT) = status;

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
