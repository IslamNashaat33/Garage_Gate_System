/* =============================================================================
 * Smart Parking Garage Gate Embedded System
 * MCU  : Tiva-C TM4C123GH6PM (ARM Cortex-M4F)
 * RTOS : FreeRTOS
 * 
 * The system simulates an automated parking gate that can be controlled 
 * from both a driver's panel and a security panel. It includes safety 
 * features such as obstacle detection and motion limits.
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
    // Initialise hardware (clocks, GPIO, UART0 at 115200 baud)
    Hardware_Init();

    // Startup banner – confirms UART is alive before any RTOS task runs
    vprintString("\n\n=== SMART PARKING GARAGE GATE SYSTEM ===\n");
    vprintString("[BOOT] Hardware initialised. RTOS starting...\n");
    vprintString("[BOOT] TC-19: Queue | TC-20: Mutex | TC-21: Semaphore\n");
    vprintString("=========================================\n\n");

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

    xTaskCreate(vSafetyTask,      "Safety",        256, NULL, 4, NULL);  /* Task 1 */
    xTaskCreate(vInputTask,       "Input",         256, NULL, 3, NULL);  /* Task 2 */
    xTaskCreate(vGateControlTask, "GateControl",   256, NULL, 2, NULL);  /* Task 3 */
    xTaskCreate(vLEDControlTask,  "LED",           256, NULL, 2, NULL);  /* Task 4 */
    xTaskCreate(vStatusTask,      "Status",        256, NULL, 1, NULL);  /* Task 5 */

    //Enable GPIO interrupts AFTER RTOS objects exist
    Hardware_EnableInterrupts();

    // Start task scheduler
    vTaskStartScheduler();

    for (;;) {}
}