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
    GPIO_PORTF_DATA_R &= ~(RED_LED | GREEN_LED);
}

void LED_Set(uint32_t mask, bool on) {
    if (on) GPIO_PORTF_DATA_R |=  mask;
    else    GPIO_PORTF_DATA_R &= ~mask;
}

/* ── Hardware initialisation ────────────────────────────────────────────────*/
void Hardware_Init(void) {
    /* Enable clocks for Port B (bit1), Port E (bit4), Port F (bit5) */
    SYSCTL_RCGCGPIO_R |= (1U << 1) | (1U << 4) | (1U << 5);
    while ((SYSCTL_PRGPIO_R & ((1U<<1)|(1U<<4)|(1U<<5))) !=
                               ((1U<<1)|(1U<<4)|(1U<<5))) {}

    /* Port F – LEDs (outputs) */
    GPIO_PORTF_LOCK_R  = 0x4C4F434BU;   /* unlock */
    GPIO_PORTF_CR_R   |= 0x1FU;
    GPIO_PORTF_DIR_R  |= (RED_LED | GREEN_LED);
    GPIO_PORTF_DEN_R  |= (RED_LED | GREEN_LED);
    GPIO_PORTF_DATA_R &= ~(RED_LED | GREEN_LED);

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
    }
    if (GPIO_PORTE_RIS_R & LIMIT_OPENED) {
        GPIO_PORTE_ICR_R = LIMIT_OPENED;
        xSemaphoreGiveFromISR(xLimitOpenSem, &xWoken);
    }
    if (GPIO_PORTE_RIS_R & LIMIT_CLOSED) {
        GPIO_PORTE_ICR_R = LIMIT_CLOSED;
        xSemaphoreGiveFromISR(xLimitCloseSem, &xWoken);
    }

    portYIELD_FROM_ISR(xWoken);
}
