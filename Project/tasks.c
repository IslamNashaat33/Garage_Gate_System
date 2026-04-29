#include "tasks.h"
#include "hardware.h"
#include "tm4c123gh6pm.h"

/*
 * Shared safety flag (Safety_Task → Gate_Control_Task).
 * Set to true by Safety_Task after an obstacle event so that
 * Gate_Control_Task ignores all movement commands until it confirms
 * all buttons have been physically released (waitForRelease handshake).
 * Volatile: written by Safety_Task (priority 4), read by GateCtrl (priority 2).
 */
static volatile bool gObstacleStop = false;

/* ── Shared gate state (always access through helpers below) ────────────────*/
static volatile GateState_t gateState = GATE_IDLE_CLOSED;

/* ── Mutex-protected state helpers ─────────────────────────────────────────*/
static void GateState_Set(GateState_t s) {
    xSemaphoreTake(xGateStateMutex, portMAX_DELAY);
    gateState = s;
    xSemaphoreGive(xGateStateMutex);
}

static GateState_t GateState_Get(void) {
    GateState_t s;
    xSemaphoreTake(xGateStateMutex, portMAX_DELAY);
    s = gateState;
    xSemaphoreGive(xGateStateMutex);
    return s;
}

/* Atomically sets state only if current state == expected. Returns true on success. */
static bool GateState_CAS(GateState_t expected, GateState_t next) {
    bool ok = false;
    xSemaphoreTake(xGateStateMutex, portMAX_DELAY);
    if (gateState == expected) { gateState = next; ok = true; }
    xSemaphoreGive(xGateStateMutex);
    return ok;
}


/* ═══════════════════════════════════════════════════════════════════════════
 * TASK 1 – Safety_Task  (Highest Priority = 4)
 *
 * Blocks on xObstacleSem.  When the obstacle sensor fires while the gate
 * is CLOSING it:
 *   1. Immediately stops (STOPPED_MIDWAY)
 *   2. Reverses (REVERSING / Green LED) for exactly 500 ms
 *   3. Stops again (STOPPED_MIDWAY / all LEDs off)
 * ═══════════════════════════════════════════════════════════════════════════*/
void vSafetyTask(void *pvParameters) {
    QueueEvent_t xDummy;

    for (;;) {
        /* Block here until obstacle semaphore is signalled from ISR */
        xSemaphoreTake(xObstacleSem, portMAX_DELAY);

        /* Only act if gate is currently CLOSING */
        if (GateState_Get() == GATE_CLOSING) {

            /* Step 1: Immediate stop */
            GateState_Set(GATE_STOPPED_MIDWAY);

            /*
             * Flush the input queue so any CLOSE commands already sitting
             * in the queue (from a held-down button) cannot restart closing
             * once the GateCtrl task wakes up.
             */
            while (xQueueReceive(xInputQueue, &xDummy, 0) == pdTRUE) {}

            /*
             * Signal Gate_Control_Task to ignore all movement commands
             * until it sees that all buttons have been released.
             */
            gObstacleStop = true;

            /* Brief settling delay */
            vTaskDelay(pdMS_TO_TICKS(50));

            /* Step 2: Reverse for 500 ms */
            GateState_Set(GATE_REVERSING);
            vTaskDelay(pdMS_TO_TICKS(REVERSE_DURATION_MS));

            /* Step 3: Full stop – midway */
            GateState_Set(GATE_STOPPED_MIDWAY);

            /* Flush again: button may still be held during the 500 ms reverse */
            while (xQueueReceive(xInputQueue, &xDummy, 0) == pdTRUE) {}
        }
        /* If obstacle fires in any other state, silently ignore */
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TASK 2 – Input_Task  (High Priority = 3)
 *
 * Polls Port B every 20 ms to debounce buttons, determines whether each
 * press is manual-hold or one-touch, resolves Security vs Driver priority
 * and same-panel conflicts, then sends a ButtonEvent_t to xInputQueue.
 *
 * Priority rules implemented here:
 *   a) Security panel always overrides Driver panel.
 *   b) OPEN + CLOSE on the same panel => EVT_STOP.
 *   c) Manual-hold: sends EVT_OPEN/CLOSE_MANUAL while held, EVT_RELEASE
 *      when released.
 *   d) One-touch: sends EVT_OPEN/CLOSE_AUTO on release (after short press).
 * ═══════════════════════════════════════════════════════════════════════════*/
void vInputTask(void *pvParameters) {
    (void)pvParameters;

    /*
     * Per-button state:
     *   pressStartTick  – tick count when button first went low (pressed)
     *   wasManual       – set true once the hold exceeds HOLD_THRESHOLD_MS
     *                     while the button is still held down; cleared on
     *                     release so every button carries its own flag.
     */
    typedef struct {
        bool     lastState;       /* true = was pressed last poll */
        bool     wasManual;       /* long-press threshold crossed while held */
        uint32_t pressStartTick;  /* tick when button went down            */
    } BtnState_t;

    BtnState_t dOpen  = {false, false, 0};
    BtnState_t dClose = {false, false, 0};
    BtnState_t sOpen  = {false, false, 0};
    BtnState_t sClose = {false, false, 0};
    bool prevSecurityActive = false; /* tracks security panel state across polls */
    bool driverSuppressed   = false; /* ignores driver until released after security override */

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(INPUT_POLL_MS));

        /* ── Read raw pin states (active-LOW, pull-up enabled) ──────────── */
        uint32_t portB = GPIO_PORTB_DATA_R;

        bool dOpenNow  = BTN_PRESSED(portB, DRIVER_OPEN);
        bool dCloseNow = BTN_PRESSED(portB, DRIVER_CLOSE);
        bool sOpenNow  = BTN_PRESSED(portB, SECURITY_OPEN);
        bool sCloseNow = BTN_PRESSED(portB, SECURITY_CLOSE);

        uint32_t nowTick = (uint32_t)xTaskGetTickCount();

        /* ── Record press-start tick on the falling edge ────────────────── */
        if (dOpenNow  && !dOpen.lastState)  { dOpen.pressStartTick  = nowTick; dOpen.wasManual  = false; }
        if (dCloseNow && !dClose.lastState) { dClose.pressStartTick = nowTick; dClose.wasManual = false; }
        if (sOpenNow  && !sOpen.lastState)  { sOpen.pressStartTick  = nowTick; sOpen.wasManual  = false; }
        if (sCloseNow && !sClose.lastState) { sClose.pressStartTick = nowTick; sClose.wasManual = false; }

        /* ── Mark buttons that have crossed the manual-hold threshold ───── */
        if (dOpenNow  && (nowTick - dOpen.pressStartTick)  >= HOLD_THRESHOLD_MS) { dOpen.wasManual  = true; }
        if (dCloseNow && (nowTick - dClose.pressStartTick) >= HOLD_THRESHOLD_MS) { dClose.wasManual = true; }
        if (sOpenNow  && (nowTick - sOpen.pressStartTick)  >= HOLD_THRESHOLD_MS) { sOpen.wasManual  = true; }
        if (sCloseNow && (nowTick - sClose.pressStartTick) >= HOLD_THRESHOLD_MS) { sClose.wasManual = true; }

        /* ── Security panel active state + rising-edge detection ──────── */
        bool securityActive     = sOpenNow || sCloseNow;
        bool securityJustActive = securityActive && !prevSecurityActive;

        /* If security is active, driver is suppressed */
        if (securityActive) {
            driverSuppressed = true;
        }

        ButtonEvent_t evt = EVT_NONE;
        bool isSecurityEvt = false;

        if (securityJustActive) {
            /*
             * Security panel just became active this poll tick.
             * Immediately send STOP to cancel any driver-initiated motion
             * (auto or manual), then let normal security processing take
             * over from the next tick onward.
             */
            evt = EVT_STOP;
            isSecurityEvt = true;

        } else {
            /*
             * Normal processing: check releases first (before securityActive
             * is used as a gate) so a button that was just released is never
             * missed because its pin already reads high.
             */

            /* Security OPEN released */
            if (!sOpenNow && sOpen.lastState) {
                evt = sOpen.wasManual ? EVT_RELEASE : EVT_OPEN_AUTO;
                sOpen.wasManual = false;
                isSecurityEvt = true;
            }
            /* Security CLOSE released */
            else if (!sCloseNow && sClose.lastState) {
                evt = sClose.wasManual ? EVT_RELEASE : EVT_CLOSE_AUTO;
                sClose.wasManual = false;
                isSecurityEvt = true;
            }
            /* Driver OPEN released – suppressed while driverSuppressed is true */
            else if (!dOpenNow && dOpen.lastState && !driverSuppressed) {
                evt = dOpen.wasManual ? EVT_RELEASE : EVT_OPEN_AUTO;
                dOpen.wasManual = false;
                isSecurityEvt = false;
            }
            /* Driver CLOSE released – suppressed while driverSuppressed is true */
            else if (!dCloseNow && dClose.lastState && !driverSuppressed) {
                evt = dClose.wasManual ? EVT_RELEASE : EVT_CLOSE_AUTO;
                dClose.wasManual = false;
                isSecurityEvt = false;
            }
            else {
                /* ── Ongoing hold (no release this tick) ─────────────── */
                if (securityActive) {
                    /* Conflict on security panel */
                    if (sOpenNow && sCloseNow) {
                        evt = EVT_STOP;
                        isSecurityEvt = true;
                    } else if (sOpenNow && sOpen.wasManual) {
                        evt = EVT_OPEN_MANUAL;
                        isSecurityEvt = true;
                    } else if (sCloseNow && sClose.wasManual) {
                        evt = EVT_CLOSE_MANUAL;
                        isSecurityEvt = true;
                    }
                    /* Threshold not yet crossed: send nothing */
                } else if (!driverSuppressed) {
                    /* Driver panel (security not active, driver not suppressed) */
                    if (dOpenNow && dCloseNow) {
                        evt = EVT_STOP;
                        isSecurityEvt = false;
                    } else if (dOpenNow && dOpen.wasManual) {
                        evt = EVT_OPEN_MANUAL;
                        isSecurityEvt = false;
                    } else if (dCloseNow && dClose.wasManual) {
                        evt = EVT_CLOSE_MANUAL;
                        isSecurityEvt = false;
                    }
                }
            }
        }

        /* ── Send event to queue (non-blocking; drop if full) ───────────── */
        if (evt != EVT_NONE) {
            QueueEvent_t qEvt = { evt, isSecurityEvt };
            xQueueSend(xInputQueue, &qEvt, 0);
        }

        /* ── Save states for next poll ──────────────────────────────────── */
        dOpen.lastState      = dOpenNow;
        dClose.lastState     = dCloseNow;
        sOpen.lastState      = sOpenNow;
        sClose.lastState     = sCloseNow;
        prevSecurityActive   = securityActive;

        /*
         * Clear suppression only when security is inactive AND the driver
         * has physically released both buttons. This prevents a driver
         * button held during a security override from suddenly taking
         * effect when security is released.
         */
        if (!securityActive && !dOpenNow && !dCloseNow) {
            driverSuppressed = false;
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TASK 3 – Gate_Control_Task  (Medium Priority = 2)
 *
 * Implements the FSM.  Two sources of inputs:
 *   1. ButtonEvent_t messages from xInputQueue  (from Input_Task)
 *   2. Limit semaphores (xLimitOpenSem, xLimitCloseSem) from the ISR
 *
 * Key design flags:
 *   gateManualMode  – true when the current movement was started by a
 *                     MANUAL hold command.  EVT_RELEASE only stops the gate
 *                     when this is true, so a short-press AUTO command is
 *                     never interrupted by the subsequent button release.
 *
 *   waitForRelease  – set true after an obstacle event (via gObstacleStop).
 *                     While true, all OPEN/CLOSE commands are ignored until
 *                     the task sees an EVT_RELEASE or EVT_STOP, meaning the
 *                     user has physically let go of every button.
 * ═══════════════════════════════════════════════════════════════════════════*/
void vGateControlTask(void *pvParameters) {
    (void)pvParameters;

    QueueEvent_t qEvt;
    ButtonEvent_t evt;
    bool gateManualMode    = false;  /* true = current move started by manual hold */
    bool waitForRelease    = false;  /* true = ignore move cmds until buttons up   */
    bool gateSecurityOwned = false;  /* true = current move owned by Security      */

    for (;;) {
        /* ── 1. Check limit semaphores (non-blocking) ──────────────────── */
        if (xSemaphoreTake(xLimitOpenSem, 0) == pdTRUE) {
            GateState_CAS(GATE_OPENING, GATE_IDLE_OPEN);
            gateManualMode = false;
            gateSecurityOwned = false;
        }
        if (xSemaphoreTake(xLimitCloseSem, 0) == pdTRUE) {
            GateState_CAS(GATE_CLOSING, GATE_IDLE_CLOSED);
            gateManualMode = false;
            gateSecurityOwned = false;
        }

        /*
         * ── 2. Latch the obstacle-stop flag from Safety_Task ────────────
         */
        if (gObstacleStop) {
            gObstacleStop   = false;   /* acknowledge */
            waitForRelease  = true;
            gateManualMode  = false;
            gateSecurityOwned = false; /* reset ownership on obstacle */
        }

        /* ── 3. Wait for the next button event (up to 50 ms) ───────────── */
        if (xQueueReceive(xInputQueue, &qEvt, pdMS_TO_TICKS(50)) != pdTRUE) {
            continue;   /* no event – re-check limits and flags */
        }

        evt = qEvt.type;

        /*
         * ── Priority Enforcement ────────────────────────────────────────
         * If the gate is currently executing a command owned by Security,
         * completely ignore any commands sent by the Driver.
         */
        if (gateSecurityOwned && !qEvt.isSecurity) {
            continue;
        }

        /*
         * ── 4. waitForRelease handshake ─────────────────────────────────
         */
        if (waitForRelease) {
            if (evt == EVT_OPEN_AUTO || evt == EVT_CLOSE_AUTO) {
                /* Short-press release confirms buttons are up.
                   Clear the flag, then fall through to process the event. */
                waitForRelease = false;
                gateManualMode = false;
                /* Do NOT continue – let the switch() handle it below */
            } else {
                /* For RELEASE / STOP: clear flag but discard event. */
                if (evt == EVT_RELEASE || evt == EVT_STOP) {
                    waitForRelease = false;
                }
                /* For MANUAL commands: keep blocking. */
                continue;
            }
        }

        GateState_t current = GateState_Get();

        switch (evt) {

        /* ── OPEN AUTO (one-touch) ─────────────────────────────────────── */
        case EVT_OPEN_AUTO:
            if (current == GATE_IDLE_CLOSED  ||
                current == GATE_STOPPED_MIDWAY ||
                current == GATE_CLOSING) {
                GateState_Set(GATE_OPENING);
                gateManualMode = false;   
                gateSecurityOwned = qEvt.isSecurity;
            }
            break;

        /* ── CLOSE AUTO (one-touch) ─────────────────────────────────────*/
        case EVT_CLOSE_AUTO:
            if (current == GATE_IDLE_OPEN    ||
                current == GATE_STOPPED_MIDWAY ||
                current == GATE_OPENING) {
                GateState_Set(GATE_CLOSING);
                gateManualMode = false;
                gateSecurityOwned = qEvt.isSecurity;
            }
            break;

        /* ── OPEN MANUAL (hold-to-run) ──────────────────────────────────*/
        case EVT_OPEN_MANUAL:
            if (current != GATE_IDLE_OPEN &&
                current != GATE_REVERSING) {
                GateState_Set(GATE_OPENING);
                gateManualMode = true;
                gateSecurityOwned = qEvt.isSecurity;
            }
            break;

        /* ── CLOSE MANUAL (hold-to-run) ─────────────────────────────────*/
        case EVT_CLOSE_MANUAL:
            if (current != GATE_IDLE_CLOSED &&
                current != GATE_REVERSING) {
                GateState_Set(GATE_CLOSING);
                gateManualMode = true;
                gateSecurityOwned = qEvt.isSecurity;
            }
            break;

        /* ── RELEASE ──────────────────────────────────────────────────── */
        case EVT_RELEASE:
            if (gateManualMode &&
                (current == GATE_OPENING || current == GATE_CLOSING)) {
                GateState_Set(GATE_STOPPED_MIDWAY);
                gateManualMode = false;
                gateSecurityOwned = false;
            }
            break;

        /* ── STOP (conflicting buttons on same panel) ───────────────────*/
        case EVT_STOP:
            if (current == GATE_OPENING || current == GATE_CLOSING) {
                GateState_Set(GATE_STOPPED_MIDWAY);
            }
            gateManualMode = false;
            gateSecurityOwned = false;
            break;

        default:
            break;
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TASK 4 – LED_Control_Task  (Medium Priority = 2)
 *
 * Reads gateState every 50 ms and drives the LEDs according to the table:
 * 
 * | State                | LED Color | Pins Active          | Motor Output |
 * |----------------------|-----------|----------------------|--------------|
 * | GATE_IDLE_CLOSED     | Blue      | BLUE_LED             | OFF          |
 * | GATE_IDLE_OPEN       | Cyan      | GREEN_LED + BLUE_LED | OFF          |
 * | GATE_OPENING         | Green     | GREEN_LED            | OPEN (PA2)   |
 * | GATE_CLOSING         | Red       | RED_LED              | CLOSE (PA3)  |
 * | GATE_STOPPED_MIDWAY  | Yellow    | RED_LED + GREEN_LED  | OFF          |
 * | GATE_REVERSING       | Magenta   | RED_LED + BLUE_LED   | OPEN (PA2)   |
 * ═══════════════════════════════════════════════════════════════════════════*/
void vLEDControlTask(void *pvParameters) {
    (void)pvParameters;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(50));

        GateState_t state = GateState_Get();

        switch (state) {
        case GATE_IDLE_CLOSED:
            LED_AllOff();
            LED_Set(BLUE_LED, true);
            Motor_Stop();
            break;

        case GATE_IDLE_OPEN:
            LED_AllOff();
            LED_Set(GREEN_LED | BLUE_LED, true); /* Cyan */
            Motor_Stop();
            break;

        case GATE_OPENING:
            LED_AllOff();
            LED_Set(GREEN_LED, true);
            Motor_SetDir(true, false);
            break;

        case GATE_CLOSING:
            LED_AllOff();
            LED_Set(RED_LED, true);
            Motor_SetDir(false, true);
            break;

        case GATE_STOPPED_MIDWAY:
            LED_AllOff();
            LED_Set(RED_LED | GREEN_LED, true); /* Yellow */
            Motor_Stop();
            break;

        case GATE_REVERSING:
            LED_AllOff();
            LED_Set(RED_LED | BLUE_LED, true); /* Magenta */
            Motor_SetDir(true, false);
            break;

        default:
            LED_AllOff();
            Motor_Stop();
            break;
        }
    }
}
