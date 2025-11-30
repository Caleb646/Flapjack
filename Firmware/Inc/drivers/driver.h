#ifndef DRIVERS_DRIVER_H
#define DRIVERS_DRIVER_H

#include <stdint.h>

#include "targets/target.h"

#define DRIVER_DECLARE(TYPE, NAME)                       \
    extern TYPE NAME##_Internal TARG_SHARED_MEM_SECTION; \
    static inline TYPE* const NAME##_Get (void) {        \
        return &NAME##_Internal;                         \
    }                                                    \
    static inline TYPE* NAME##_GetMutable (void) {       \
        return &NAME##_Internal;                         \
    }

#define DRIVER_DECLARE_ARRAY(TYPE, NAME, SIZE)                      \
    extern TYPE NAME##_InternalArray[SIZE] TARG_SHARED_MEM_SECTION; \
    static inline TYPE* const NAME##_Get (uint32_t idx) {           \
        if (idx >= (SIZE)) {                                        \
            return NULL;                                            \
        }                                                           \
        return &NAME##_InternalArray[idx];                          \
    }                                                               \
    static inline TYPE* NAME##_GetMutable (uint32_t idx) {          \
        if (idx >= (SIZE)) {                                        \
            return NULL;                                            \
        }                                                           \
        return &NAME##_InternalArray[idx];                          \
    }                                                               \
    static inline TYPE (*NAME##_GetArray (void))[SIZE] {            \
        return &NAME##_InternalArray;                               \
    }

// clang-format off
#define DRIVER_DEFINE(TYPE, NAME)             TYPE NAME##_Internal TARG_SHARED_MEM_SECTION
#define DRIVER_DEFINE_ARRAY(TYPE, NAME, SIZE) TYPE NAME##_InternalArray[SIZE] TARG_SHARED_MEM_SECTION
// clang-format on

#endif // DRIVERS_DRIVER_H