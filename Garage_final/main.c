/* =============================================================================
 * Smart Parking Garage Gate Embedded System
 * MCU  : Tiva-C TM4C123GH6PM (ARM Cortex-M4F)
 * RTOS : FreeRTOS
 * =============================================================================
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "hardware.h"
#include "tasks.h"

/* RTOS object handles definition */
QueueHandle_t     xInputQueue;       /* QueueEvent_t from Input→GateCtrl */
SemaphoreHandle_t xObstacleSem;      /* obstacle ISR → Safety task        */
SemaphoreHandle_t xLimitOpenSem;     /* limit-open  ISR → GateCtrl task   */
SemaphoreHandle_t xLimitCloseSem;    /* limit-close ISR → GateCtrl task   */
SemaphoreHandle_t xGateStateMutex;   /* protects gateState                */


int main(void) {
    // Initialise hardware (clocks, GPIO directions, pull-ups)
    Hardware_Init();

    // Create RTOS synchronisation objects
    xInputQueue     = xQueueCreate(5, sizeof(QueueEvent_t));
    xObstacleSem    = xSemaphoreCreateBinary();
    xLimitOpenSem   = xSemaphoreCreateBinary();
    xLimitCloseSem  = xSemaphoreCreateBinary();
    xGateStateMutex = xSemaphoreCreateMutex();

    /* Abort if any object creation failed */
    configASSERT(xInputQueue     != NULL);
    configASSERT(xObstacleSem    != NULL);
    configASSERT(xLimitOpenSem   != NULL);
    configASSERT(xLimitCloseSem  != NULL);
    configASSERT(xGateStateMutex != NULL);

    xTaskCreate(vSafetyTask,      "Safety",   256, NULL, 4, NULL);
    xTaskCreate(vInputTask,       "Input",    256, NULL, 3, NULL);
    xTaskCreate(vGateControlTask, "GateCtrl", 256, NULL, 2, NULL);
    xTaskCreate(vLEDControlTask,  "LED",      256, NULL, 2, NULL);

    //Enable GPIO interrupts AFTER RTOS objects exist
    Hardware_EnableInterrupts();

    // Start task scheduler
    vTaskStartScheduler();

    for (;;) {}
}