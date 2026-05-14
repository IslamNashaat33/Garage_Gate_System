#ifndef TASKS_H
#define TASKS_H

#include "hardware.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* RTOS object handles declared as extern so main.c can create them and hardware.c/tasks.c can use them */
extern QueueHandle_t     xInputQueue;
extern SemaphoreHandle_t xObstacleSem;
extern SemaphoreHandle_t xLimitOpenSem;
extern SemaphoreHandle_t xLimitCloseSem;
extern SemaphoreHandle_t xGateStateMutex;

/* Task Prototypes */
void vSafetyTask(void *pvParameters);      /* Task 1 – Safety       (Priority 4) */
void vInputTask(void *pvParameters);       /* Task 2 – Input        (Priority 3) */
void vGateControlTask(void *pvParameters); /* Task 3 – Gate Control (Priority 2) */
void vLEDControlTask(void *pvParameters);  /* Task 4 – LED Control  (Priority 2) */
void vStatusTask(void *pvParameters);      /* Task 5 – Status       (Priority 1) */

#endif /* TASKS_H */
