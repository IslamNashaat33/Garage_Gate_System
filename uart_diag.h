#ifndef UART_DIAG_H
#define UART_DIAG_H

#include <stdint.h>

#include "utils/uartstdio.h"

void UARTDiag_Init(uint32_t systemClockHz);

#define UARTDiag_Print UARTprintf

#endif
