#ifndef DRIVERS_CORE_LOCK_H
#define DRIVERS_CORE_LOCK_H

#include <stdint.h>

#include "common.h"

typedef uint8_t eIRQ_PRIO_t;
enum {
    eIRQ_PRIO_UNUSED = 0,
    eIRQ_PRIO_HIGHEST,
    eIRQ_PRIO_2,
    eIRQ_PRIO_3,
    eIRQ_PRIO_4,
    eIRQ_PRIO_5,
    eIRQ_PRIO_6,
    eIRQ_PRIO_7,
    eIRQ_PRIO_8,
    eIRQ_PRIO_9,
    eIRQ_PRIO_10,
    eIRQ_PRIO_11,
    eIRQ_PRIO_12,
    eIRQ_PRIO_13,
    eIRQ_PRIO_14,
    eIRQ_PRIO_LOWEST
};

typedef struct SpinLock_s {
    uint8_t semId;
} SpinLock_t;
typedef uint8_t IrqState_t;

static inline IrqState_t Irq_SetState (eIRQ_PRIO_t lvl) {

    uint32_t prevState = __get_BASEPRI ();
    __set_BASEPRI ((uint32_t)lvl << (8U - __NVIC_PRIO_BITS));
    return prevState & 0xFFU;
}

static inline void Irq_RestoreState (IrqState_t prevState) {

    __set_BASEPRI ((uint32_t)prevState);
}

void Plat_SpinLock_Init (SpinLock_t* pLock);
void Plat_SpinLock_Take (SpinLock_t* pLock);
void Plat_SpinLock_Release (SpinLock_t* pLock);

static inline void SpinLock_Init (SpinLock_t* pLock) {

    Plat_SpinLock_Init (pLock);
}

static inline void SpinLock_Take (SpinLock_t* pLock) {

    // while (!__atomic_test_and_set (pLock, __ATOMIC_ACQUIRE)) {
    //     // spin-wait
    // }
    Plat_SpinLock_Take (pLock);
}

static inline void SpinLock_Release (SpinLock_t* pLock) {

    // __atomic_clear (pLock, __ATOMIC_RELEASE);
    Plat_SpinLock_Release (pLock);
}

static inline IrqState_t IrqSpinLock_Take (SpinLock_t* pLock, eIRQ_PRIO_t lvl) {

    IrqState_t state = Irq_SetState (lvl);
    SpinLock_Take (pLock);
    return state;
}

static inline void IrqSpinLock_Release (SpinLock_t* pLock, IrqState_t prevState) {

    SpinLock_Release (pLock);
    Irq_RestoreState (prevState);
}


#endif // DRIVERS_CORE_LOCK_H