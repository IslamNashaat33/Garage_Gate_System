#include "hardware.h"
#include "tm4c123gh6pm.h"
#include "FreeRTOS.h"
#include "semphr.h"

/* Extern references to Semaphores used in ISRs (defined in main.c) */
extern SemaphoreHandle_t xObstacleSem;
extern SemaphoreHandle_t xLimitOpenSem;
extern SemaphoreHandle_t xLimitCloseSem;

/* ── LED helpers ────────────────────────────────────────────────────────────*/
void LED_AllOff(void) {
    GPIO_PORTF_DATA_R &= ~(RED_LED | BLUE_LED | GREEN_LED);
}

void LED_Set(uint32_t mask, bool on) {
    if (on) GPIO_PORTF_DATA_R |=  mask;
    else    GPIO_PORTF_DATA_R &= ~mask;
}

/* ── Motor helpers ──────────────────────────────────────────────────────────*/
void Motor_Stop(void) {
    GPIO_PORTA_DATA_R &= ~(MOTOR_OPEN | MOTOR_CLOSE);
}

void Motor_SetDir(bool opening, bool closing) {
    if (opening) GPIO_PORTA_DATA_R |= MOTOR_OPEN;
    else         GPIO_PORTA_DATA_R &= ~MOTOR_OPEN;

    if (closing) GPIO_PORTA_DATA_R |= MOTOR_CLOSE;
    else         GPIO_PORTA_DATA_R &= ~MOTOR_CLOSE;
}

/* ── Hardware initialisation ────────────────────────────────────────────────*/
void Hardware_Init(void) {
    /* Enable clocks for Port A (bit0), Port B (bit1), Port E (bit4), Port F (bit5) */
    SYSCTL_RCGCGPIO_R |= (1U << 0) | (1U << 1) | (1U << 4) | (1U << 5);
    while ((SYSCTL_PRGPIO_R & ((1U<<0)|(1U<<1)|(1U<<4)|(1U<<5))) !=
                               ((1U<<0)|(1U<<1)|(1U<<4)|(1U<<5))) {}

    /* ── Enable UART0 clock and wait for it to be ready ─────────────────── */
    SYSCTL_RCGCUART_R |= (1U << 0);          /* UART0 clock gate            */
    while ((SYSCTL_PRUART_R & (1U << 0)) == 0U) {}  /* wait for ready        */

    /* ── Port A: PA0 = UART0 RX, PA1 = UART0 TX, PA2/PA3 = Motor outputs ─── */
    GPIO_PORTA_AFSEL_R |=  (1U << 0) | (1U << 1); /* alt function on PA0,PA1 */
    GPIO_PORTA_PCTL_R   = (GPIO_PORTA_PCTL_R
                           & ~0x000000FFU)          /* clear PMC0, PMC1       */
                           |  0x00000011U;           /* PMC0=1, PMC1=1 = UART  */
    GPIO_PORTA_DEN_R   |=  (1U << 0) | (1U << 1); /* digital enable PA0,PA1  */
    GPIO_PORTA_DIR_R   &= ~(1U << 0);             /* PA0 input  (RX)         */
    GPIO_PORTA_DIR_R   |=  (1U << 1);             /* PA1 output (TX)         */

    /* ── Configure UART0: 115200 baud, 8-N-1, FIFOs enabled ────────────── */
    UART0_CTL_R &= ~(1U << 0);                    /* disable UART during setup */
    UART0_CC_R   = 0x00U;                          /* clock source = SysClk   */

    /*
     * Baud-rate divisor: BRD = SysClk / (16 × Baud)
     * Use integer math with rounding:
     *   BRD_64 = (SysClk × 4 + Baud/2) / Baud
     *   IBRD   = BRD_64 / 64
     *   FBRD   = BRD_64 % 64
     * Works correctly whether Keil boots at 16 MHz, 50 MHz, or 80 MHz.
     */
    extern uint32_t SystemCoreClock;
    uint32_t brd64 = ((SystemCoreClock * 4U) + (115200U / 2U)) / 115200U;
    UART0_IBRD_R = brd64 / 64U;
    UART0_FBRD_R = brd64 % 64U;

    UART0_LCRH_R = (0x3U << 5)    /* 8-bit word length (WLEN = 11) */
                 | (1U   << 4);   /* enable TX/RX FIFOs (FEN = 1)  */
    UART0_CTL_R  = (1U << 0)      /* UARTEN: enable UART           */
                 | (1U << 8)      /* TXE:    transmit enable        */
                 | (1U << 9);     /* RXE:    receive enable         */

    /* Port A – Motor outputs (PA2, PA3) */
    GPIO_PORTA_DIR_R  |= (MOTOR_OPEN | MOTOR_CLOSE);
    GPIO_PORTA_DEN_R  |= (MOTOR_OPEN | MOTOR_CLOSE);
    GPIO_PORTA_DATA_R &= ~(MOTOR_OPEN | MOTOR_CLOSE);

    /* Port F – LEDs (outputs) */
    GPIO_PORTF_LOCK_R  = 0x4C4F434BU;   /* unlock */
    GPIO_PORTF_CR_R   |= 0x1FU;
    GPIO_PORTF_DIR_R  |= (RED_LED | BLUE_LED | GREEN_LED);
    GPIO_PORTF_DEN_R  |= (RED_LED | BLUE_LED | GREEN_LED);
    GPIO_PORTF_DATA_R &= ~(RED_LED | BLUE_LED | GREEN_LED);

    /* Port B – Driver / Security buttons (inputs, pull-up) */
    GPIO_PORTB_DIR_R  &= ~(DRIVER_OPEN | DRIVER_CLOSE | SECURITY_OPEN | SECURITY_CLOSE);
    GPIO_PORTB_DEN_R  |=  (DRIVER_OPEN | DRIVER_CLOSE | SECURITY_OPEN | SECURITY_CLOSE);
    GPIO_PORTB_PUR_R  |=  (DRIVER_OPEN | DRIVER_CLOSE | SECURITY_OPEN | SECURITY_CLOSE);

    /* Port E – Limit switches and obstacle sensor (inputs, pull-up) */
    GPIO_PORTE_DIR_R  &= ~(LIMIT_OPENED | LIMIT_CLOSED | OBSTACLE);
    GPIO_PORTE_DEN_R  |=  (LIMIT_OPENED | LIMIT_CLOSED | OBSTACLE);
    GPIO_PORTE_PUR_R  |=  (LIMIT_OPENED | LIMIT_CLOSED | OBSTACLE);
}

/* Called AFTER all RTOS objects are created */
void Hardware_EnableInterrupts(void) {
    /* Port B – both edges on PB0-PB3 (detect press and release) */
    GPIO_PORTB_IS_R  &= ~(DRIVER_OPEN | DRIVER_CLOSE | SECURITY_OPEN | SECURITY_CLOSE);
    GPIO_PORTB_IBE_R |=  (DRIVER_OPEN | DRIVER_CLOSE | SECURITY_OPEN | SECURITY_CLOSE);
    GPIO_PORTB_ICR_R  =  (DRIVER_OPEN | DRIVER_CLOSE | SECURITY_OPEN | SECURITY_CLOSE);
    GPIO_PORTB_IM_R  |=  (DRIVER_OPEN | DRIVER_CLOSE | SECURITY_OPEN | SECURITY_CLOSE);
    /* IRQ1 (GPIO Port B): priority 5 */
    NVIC_PRI0_R = (NVIC_PRI0_R & ~(0x7U << 13)) | (5U << 13);
    NVIC_EN0_R  |= (1U << 1);

    /* Port E – falling edge on PE1, PE2, PE3 (active-low: pressed = 0) */
    GPIO_PORTE_IS_R  &= ~(LIMIT_OPENED | LIMIT_CLOSED | OBSTACLE);
    GPIO_PORTE_IBE_R &= ~(LIMIT_OPENED | LIMIT_CLOSED | OBSTACLE);
    GPIO_PORTE_IEV_R &= ~(LIMIT_OPENED | LIMIT_CLOSED | OBSTACLE); /* falling */
    GPIO_PORTE_ICR_R  =  (LIMIT_OPENED | LIMIT_CLOSED | OBSTACLE);
    GPIO_PORTE_IM_R  |=  (LIMIT_OPENED | LIMIT_CLOSED | OBSTACLE);
    /* IRQ4 (GPIO Port E): priority 5 */
    NVIC_PRI1_R = (NVIC_PRI1_R & ~(0x7U << 5)) | (5U << 5);
    NVIC_EN0_R  |= (1U << 4);
}

/* ── ISR: GPIO Port B (driver & security buttons) ───────────────────────────*/
void GPIOB_Handler(void) {
    GPIO_PORTB_ICR_R = (DRIVER_OPEN | DRIVER_CLOSE | SECURITY_OPEN | SECURITY_CLOSE);
}

/* ── ISR: GPIO Port E (limit switches & obstacle) ───────────────────────────*/
void GPIOE_Handler(void) {
    BaseType_t xWoken = pdFALSE;

    if (GPIO_PORTE_RIS_R & OBSTACLE) {
        GPIO_PORTE_ICR_R = OBSTACLE;
        xSemaphoreGiveFromISR(xObstacleSem, &xWoken);
        /* TC-21: Obstacle semaphore signalled from ISR */
        vprintString("[TC-21][Semaphore] Obstacle semaphore GIVEN from ISR\n");
    }
    if (GPIO_PORTE_RIS_R & LIMIT_OPENED) {
        GPIO_PORTE_ICR_R = LIMIT_OPENED;
        xSemaphoreGiveFromISR(xLimitOpenSem, &xWoken);
        /* TC-21: Limit-Open semaphore signalled from ISR */
        vprintString("[TC-21][Semaphore] Limit-Open semaphore GIVEN from ISR\n");
    }
    if (GPIO_PORTE_RIS_R & LIMIT_CLOSED) {
        GPIO_PORTE_ICR_R = LIMIT_CLOSED;
        xSemaphoreGiveFromISR(xLimitCloseSem, &xWoken);
        /* TC-21: Limit-Close semaphore signalled from ISR */
        vprintString("[TC-21][Semaphore] Limit-Close semaphore GIVEN from ISR\n");
    }

    portYIELD_FROM_ISR(xWoken);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * vprintString  –  UART0 polling debug output
 *
 * Sends a null-terminated string over UART0 at 115200 baud (8-N-1).
 * UART0 TX is wired to the Tiva-C LaunchPad’s built-in ICDI USB-to-serial
 * bridge, so it appears as a virtual COM port on your PC the moment you
 * plug in the USB debug cable – the same cable used for flashing.
 *
 * HOW TO VIEW THE OUTPUT WITH PuTTY:
 *   1. Plug in the Tiva-C board via USB (debug cable).
 *   2. Open Device Manager → Ports (COM & LPT) to find the COM number
 *      (e.g. COM4).  It is labelled “Stellaris Virtual Serial Port”.
 *   3. Open PuTTY, choose “Serial”, set COM port = COM4, Speed = 115200.
 *   4. Click “Open” – keep PuTTY open.
 *   5. Flash the board from Keil (Ctrl+F5 or the Load button).
 *   6. Press the RESET button on the board.
 *   7. Messages appear immediately in PuTTY.
 *
 * NOTE: Do NOT open PuTTY and Keil’s debug session at the same time –
 *       they both try to claim the COM port and will conflict.
 *       Close PuTTY before starting a Keil debug session.
 * ═══════════════════════════════════════════════════════════════════════════*/
void vprintString(const char *pcString) {
    while (*pcString != '\0') {
        /* Wait until TX FIFO has space (TXFF bit in UART0_FR_R is 0 when space) */
        while ((UART0_FR_R & (1U << 5)) != 0U) {}
        /* If \n, send \r first so PuTTY shows a proper new line */
        if (*pcString == '\n') {
            UART0_DR_R = (uint32_t)'\r';
            while ((UART0_FR_R & (1U << 5)) != 0U) {}
        }
        UART0_DR_R = (uint32_t)(*pcString);
        pcString++;
    }
}

