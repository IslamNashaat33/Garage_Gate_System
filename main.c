#include <stdbool.h>
#include <stdint.h>

#include "RTE_Components.h"
#include CMSIS_device_header

#include "FreeRTOS.h"
#include "task.h"

#include "gpio.h"
#include "tasks.h"

#define SYSCTL_RIS_PLLLRIS        (1UL << 6)

#define SYSCTL_RCC_XTAL_M         (0x1FUL << 6)
#define SYSCTL_RCC_XTAL_16MHZ     (0x15UL << 6)
#define SYSCTL_RCC_OSCSRC_M       (0x3UL << 4)
#define SYSCTL_RCC_OSCSRC_MAIN    (0x0UL << 4)
#define SYSCTL_RCC_USESYSDIV      (1UL << 22)

#define SYSCTL_RCC2_USERCC2       (1UL << 31)
#define SYSCTL_RCC2_DIV400        (1UL << 30)
#define SYSCTL_RCC2_BYPASS2       (1UL << 11)
#define SYSCTL_RCC2_PWRDN2        (1UL << 13)
#define SYSCTL_RCC2_OSCSRC2_M     (0x7UL << 4)
#define SYSCTL_RCC2_OSCSRC2_MAIN  (0x0UL << 4)
#define SYSCTL_RCC2_SYSDIV2_M     (0x7FUL << 22)

#define SYSCLK_80MHZ_DIVIDER_FIELD (4UL << 23)

static void SystemClockInit(void)
{
    /* Configure PLL from 16 MHz crystal to 80 MHz system clock. */
    SYSCTL->RCC2 |= SYSCTL_RCC2_USERCC2;
    SYSCTL->RCC2 |= SYSCTL_RCC2_BYPASS2;

    SYSCTL->RCC &= ~(SYSCTL_RCC_XTAL_M | SYSCTL_RCC_OSCSRC_M);
    SYSCTL->RCC |= (SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV);

    SYSCTL->RCC2 &= ~SYSCTL_RCC2_OSCSRC2_M;
    SYSCTL->RCC2 |= SYSCTL_RCC2_OSCSRC2_MAIN;

    SYSCTL->RCC2 &= ~SYSCTL_RCC2_PWRDN2;
    SYSCTL->RCC2 |= SYSCTL_RCC2_DIV400;

    SYSCTL->RCC2 &= ~SYSCTL_RCC2_SYSDIV2_M;
    SYSCTL->RCC2 |= SYSCLK_80MHZ_DIVIDER_FIELD;

    while ((SYSCTL->RIS & SYSCTL_RIS_PLLLRIS) == 0U)
    {
    }

    SYSCTL->RCC2 &= ~SYSCTL_RCC2_BYPASS2;
}

int main(void)
{
    SystemClockInit();
    __disable_irq();

    GPIO_HardwareInit();

    if (!Tasks_Init())
    {
        for (;;)
        {
        }
    }

    __enable_irq();
    vTaskStartScheduler();

    for (;;)
    {
    }
}
