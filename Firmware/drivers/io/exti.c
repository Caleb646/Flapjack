#include "hal.h"
#include "target.h"

#include "core/core.h"

#include "drivers/io/exti.h"
#include "drivers/io/gpio.h"

#include <stdint.h>
#include <string.h>

#define EXTI_LINE_COUNT 16U

typedef struct {
    ExtiCallback_t callback;
    void* ctx;
} ExtiLine_t;

/*
 * Indexed by line number, which is the pin number. Written once at
 * registration and read from ISR context thereafter.
 */
static ExtiLine_t s_Lines[EXTI_LINE_COUNT];

static IRQn_Type Exti_IrqForLine (uint32_t line) {

    // EXTI0..EXTI4_IRQn are contiguous (6..10); 5-9 and 10-15 share a vector.
    if (line <= 4U) {
        return (IRQn_Type)((uint32_t)EXTI0_IRQn + line);
    }
    if (line <= 9U) {
        return EXTI9_5_IRQn;
    }
    return EXTI15_10_IRQn;
}

static void Exti_HandleLine (uint32_t line) {

    uint32_t pin = 1UL << line;
    if (__HAL_GPIO_EXTI_GET_IT (pin) == 0U) {
        return;
    }

    /* Clear before dispatching: an edge arriving while the callback runs must
     * leave the line pending again, not be swallowed by a later clear. */
    __HAL_GPIO_EXTI_CLEAR_IT (pin);

    if (s_Lines[line].callback) {
        s_Lines[line].callback (s_Lines[line].ctx);
    }
}

static void Exti_HandleRange (uint32_t first, uint32_t last) {

    for (uint32_t line = first; line <= last; ++line) {
        Exti_HandleLine (line);
    }
}

// Override the weak defaults in startup_stm32h747xihx.s.
void EXTI0_IRQHandler (void) { Exti_HandleLine (0U); }
void EXTI1_IRQHandler (void) { Exti_HandleLine (1U); }
void EXTI2_IRQHandler (void) { Exti_HandleLine (2U); }
void EXTI3_IRQHandler (void) { Exti_HandleLine (3U); }
void EXTI4_IRQHandler (void) { Exti_HandleLine (4U); }
void EXTI9_5_IRQHandler (void) { Exti_HandleRange (5U, 9U); }
void EXTI15_10_IRQHandler (void) { Exti_HandleRange (10U, 15U); }

eSTATUS_t Exti_Register (ExtiConf_t const* pConf) {

    if (!pConf || !pConf->pPort || !pConf->callback) {
        return eSTATUS_NULL_ARG;
    }

    /* One pin per registration: the line is the pin's bit position, so a
     * multi-bit mask has no single line to dispatch to. */
    if (pConf->pin == 0U || (pConf->pin & (uint16_t)(pConf->pin - 1U)) != 0U) {
        return eSTATUS_INVALID_ARG;
    }

    uint32_t line = (uint32_t)__builtin_ctz (pConf->pin);
    if (s_Lines[line].callback) {
        return eSTATUS_ALREADY_INITED;
    }

    GPIO_ENABLE_CLOCK (pConf->pPort);
    __HAL_RCC_SYSCFG_CLK_ENABLE ();

    GPIO_InitTypeDef init = { 0 };
    init.Pin              = pConf->pin;
    init.Mode             = pConf->trigger;
    init.Pull             = pConf->pull;
    init.Speed            = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init (pConf->pPort, &init);

    /* HAL_GPIO_Init unmasks the line, so a pin already asserted at boot leaves
     * it pending. Drop that edge rather than waking the owner before it has
     * anything to read. */
    __HAL_GPIO_EXTI_CLEAR_IT (pConf->pin);

    /* Callback before NVIC enable: a pending edge fires the vector the moment
     * it is enabled, and that must not land on an empty slot. */
    s_Lines[line].callback = pConf->callback;
    s_Lines[line].ctx      = pConf->ctx;

    IRQn_Type irq = Exti_IrqForLine (line);
    HAL_NVIC_SetPriority (irq, pConf->irqPriority, 0U);
    HAL_NVIC_EnableIRQ (irq);

    return eSTATUS_SUCCESS;
}
