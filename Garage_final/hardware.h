#ifndef HARDWARE_H
#define HARDWARE_H
/* =============================================================================
 * Hardware map
 *  Outputs  : PF1 = Red LED (closing), PF3 = Green LED (opening)
 *  Port B   : PB0 = Driver OPEN,  PB1 = Driver CLOSE
 *             PB2 = Security OPEN, PB3 = Security CLOSE
 *  Port E   : PE1 = Limit OPEN (fully-open sensor)
 *             PE2 = Limit CLOSED (fully-closed sensor)
 *             PE3 = Obstacle sensor
 * =============================================================================
 */
#include <stdint.h>
#include <stdbool.h>

/* ── Pin definitions ────────────────────────────────────────────────────────*/
#define RED_LED         (1U << 1)   /* PF1 – gate closing  */
#define GREEN_LED       (1U << 3)   /* PF3 – gate opening  */

#define DRIVER_OPEN     (1U << 0)   /* PB0 */
#define DRIVER_CLOSE    (1U << 1)   /* PB1 */
#define SECURITY_OPEN   (1U << 2)   /* PB2 */
#define SECURITY_CLOSE  (1U << 3)   /* PB3 */

#define LIMIT_OPENED    (1U << 1)   /* PE1 */
#define LIMIT_CLOSED    (1U << 2)   /* PE2 */
#define OBSTACLE        (1U << 3)   /* PE3 */

/* Buttons are active-LOW (pull-up resistors enabled).
   A pin reads 0 when pressed, 1 when released.              */
#define BTN_PRESSED(port, mask)   (((port) & (mask)) == 0U)

/* Hold time that distinguishes manual-hold from one-touch.
   < HOLD_THRESHOLD_MS  => one-touch auto mode
   >= HOLD_THRESHOLD_MS => manual (hold-to-run) mode          */
#define HOLD_THRESHOLD_MS   600U   /* < this = short press (auto); >= this = long press (manual) */

/* Reverse duration after obstacle detection (ms) */
#define REVERSE_DURATION_MS 500U

/* Input task poll period (ms) */
#define INPUT_POLL_MS        20U

/* ── FSM States ─────────────────────────────────────────────────────────────*/
typedef enum {
    GATE_IDLE_CLOSED    = 0,
    GATE_IDLE_OPEN,
    GATE_OPENING,
    GATE_CLOSING,
    GATE_STOPPED_MIDWAY,
    GATE_REVERSING
} GateState_t;

/* ── Button event type sent through the queue ───────────────────────────────*/
typedef enum {
    EVT_NONE = 0,
    EVT_OPEN_AUTO,       /* brief press – one-touch open  */
    EVT_CLOSE_AUTO,      /* brief press – one-touch close */
    EVT_OPEN_MANUAL,     /* held – move while held        */
    EVT_CLOSE_MANUAL,    /* held – move while held        */
    EVT_RELEASE,         /* button released in manual mode*/
    EVT_STOP             /* conflicting buttons on same panel */
} ButtonEvent_t;

typedef struct {
    ButtonEvent_t type;
    bool isSecurity;
} QueueEvent_t;

/* ── Hardware Functions ─────────────────────────────────────────────────────*/
void Hardware_Init(void);
void Hardware_EnableInterrupts(void);
void LED_AllOff(void);
void LED_Set(uint32_t mask, bool on);

#endif /* HARDWARE_H */
