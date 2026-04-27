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
void vSafetyTask(void *pvParameters);
void vInputTask(void *pvParameters);
void vGateControlTask(void *pvParameters);
void vLEDControlTask(void *pvParameters);

#endif /* TASKS_H */
