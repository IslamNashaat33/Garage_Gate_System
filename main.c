#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"

#include "gpio.h"
#include "tasks.h"

static void SystemClockInit(void)
{
    (void)SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |
                              SYSCTL_OSC_MAIN |
                              SYSCTL_USE_PLL |
                              SYSCTL_CFG_VCO_480),
                             80000000U);
}

int main(void)
{
    SystemClockInit();
    IntMasterDisable();

    GPIO_HardwareInit();

    if (!Tasks_Init())
    {
        for (;;)
        {
        }
    }

    IntMasterEnable();
    vTaskStartScheduler();

    for (;;)
    {
    }
}
