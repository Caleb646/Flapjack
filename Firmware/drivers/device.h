#ifndef DRIVERS_DEVICE_H
#define DRIVERS_DEVICE_H

/*
 * Types shared by every sensor driver interface, independent of which part is
 * fitted or which bus it sits on.
 *
 * DataReadySignal_t is how a driver tells whoever is waiting that a sample has
 * landed. Drivers get it on their driver struct's cfg and only ever call
 * Notify, from ISR context. What sits on the other side - a task notification,
 * a semaphore, nothing at all - is the caller's business, so no driver has to
 * include FreeRTOS to be woken by one.
 *
 * Not tied to EXTI: the source is a data-ready pin for the IMU, mag and baro
 * (drivers/io/exti.h), but the GPS raises the same signal from the UART ISR
 * when a sentence completes. One signal, several sources.
 *
 * A NULL Notify means the caller is not using interrupts, and the driver must
 * leave the part's interrupt output alone. Read() polls the data registers
 * either way - the signal decides WHEN a read happens, not where the data
 * comes from - so a driver whose caller passes nothing keeps working unchanged.
 *
 * irqPriority travels with the signal because the constraint it exists for
 * belongs to Notify, not to the pin: an ISR that calls a FreeRTOS `...FromISR`
 * API must run at a priority numerically >= configMAX_SYSCALL_INTERRUPT_
 * PRIORITY. The layer that supplies Notify is the one that knows, and is the
 * only layer that can static-assert it.
 */

#include <stdint.h>

typedef void (*DataReadyFn_t) (void* ctx);

typedef struct {
    DataReadyFn_t Notify;   // ISR context. NULL = caller polls; leave the part's INT alone.
    void* ctx;
    uint8_t irqPriority;
} DataReadySignal_t;

#endif // DRIVERS_DEVICE_H
