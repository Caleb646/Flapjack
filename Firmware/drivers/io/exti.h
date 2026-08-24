#ifndef DRIVERS_IO_EXTI_H
#define DRIVERS_IO_EXTI_H

/*
 * Shared EXTI line dispatcher.
 *
 * A GPIO interrupt cannot be owned by the driver that cares about it the way a
 * UART or SPI interrupt can (uart.c, spi.c, dma.c each define their own
 * handler). On STM32 the EXTI line index IS the pin number, and lines 5-9 and
 * 10-15 share one vector each - so EXTI9_5_IRQHandler belongs to every sensor
 * whose interrupt pin happens to land in that range, not to any one of them.
 * This module owns the seven handlers and fans each edge out to whoever
 * registered that line, the same shape dma.c uses for its stream handlers.
 *
 * Nothing here knows about FreeRTOS. The callback runs in ISR context and is
 * expected to do one thing - signal whatever is waiting - so a task layer can
 * back it with a notification without that choice reaching down here.
 *
 * IRQ PRIORITY: a callback that calls a FreeRTOS `...FromISR` API must be
 * registered with `irqPriority` numerically >= configMAX_SYSCALL_INTERRUPT_
 * PRIORITY (5 on this build, configPRIO_BITS 4). A numerically lower priority
 * preempts the kernel's critical sections and corrupts it intermittently.
 * Not asserted here because that would pull FreeRTOSConfig.h into drivers/;
 * the task supplying the callback asserts it instead. The GPS UART carries the
 * same constraint without going through this file - its signal is raised from
 * the RX ISR (gps.c), so gps_task.c asserts its own priority the same way.
 */

#include "hal.h"

#include "core/core.h"

#include <stdint.h>

/* Runs in ISR context. Keep it to signalling; do no bus traffic here. */
typedef void (*ExtiCallback_t) (void* ctx);

typedef struct {
    GPIO_TypeDef* pPort;
    uint16_t pin;         // single GPIO_PIN_n; its bit position is the EXTI line
    uint32_t trigger;     // GPIO_MODE_IT_RISING / _FALLING / _RISING_FALLING
    uint32_t pull;        // GPIO_NOPULL / GPIO_PULLUP / GPIO_PULLDOWN
    uint8_t irqPriority;  // see IRQ PRIORITY above
    ExtiCallback_t callback;
    void* ctx;
} ExtiConf_t;

/*
 * Configures the pin, claims its EXTI line and enables the vector.
 *
 * Returns eSTATUS_ALREADY_INITED if the line is already claimed: SYSCFG_EXTICR
 * routes one port per line, so a second registration would silently re-point
 * the line and strand the first owner rather than adding to it.
 */
eSTATUS_t Exti_Register (ExtiConf_t const* pConf);

#endif // DRIVERS_IO_EXTI_H
