#ifndef TASKS_H
#define TASKS_H

#include <stdbool.h>

#include "fsm.h"

typedef struct
{
    GateState state;
    GateMotorDirection motorDirection;
    CommandSource activeSource;
} GateSystemStatus;

bool Tasks_Init(void);

#endif
