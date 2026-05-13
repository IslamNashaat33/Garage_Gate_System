#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"

#include "gpio.h"
#include "tasks.h"
#include "uart_diag.h"

#define SYSTEM_CLOCK_HZ 80000000U

static void SystemClockInit(void)
{
    (void)SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |
                              SYSCTL_OSC_MAIN |
                              SYSCTL_USE_PLL |
                              SYSCTL_CFG_VCO_480),
                             SYSTEM_CLOCK_HZ);
}

int main(void)
{
    SystemClockInit();
    UARTDiag_Init(SYSTEM_CLOCK_HZ);
    UARTDiag_Print("\r\nGarage Gate System UART diagnostics started\r\n");

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
