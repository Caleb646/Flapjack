#ifndef CFG_H
#define CFG_H

#include <stdint.h>

#define DEV_ID_TO_CFG_ID(ID) ((ID) - 1U)

#define CFG_DECLARE(TYPE, NAME)                    \
    extern TYPE NAME##_Internal;                   \
    static inline TYPE* const NAME##_Get (void) {  \
        return &NAME##_Internal;                   \
    }                                              \
    static inline TYPE* NAME##_GetMutable (void) { \
        return &NAME##_Internal;                   \
    }

#define CFG_DECLARE_ARRAY(TYPE, NAME, SIZE)                \
    extern TYPE NAME##_InternalArray[SIZE];                \
    static inline TYPE* const NAME##_Get (uint32_t idx) {  \
        if (idx >= (SIZE)) {                               \
            return NULL;                                   \
        }                                                  \
        return &NAME##_InternalArray[idx];                 \
    }                                                      \
    static inline TYPE* NAME##_GetMutable (uint32_t idx) { \
        if (idx >= (SIZE)) {                               \
            return NULL;                                   \
        }                                                  \
        return &NAME##_InternalArray[idx];                 \
    }                                                      \
    static inline TYPE (*NAME##_GetArray (void))[SIZE] {   \
        return &NAME##_InternalArray;                      \
    }

#define CFG_DEFINE(TYPE, NAME)             TYPE NAME##_Internal
#define CFG_DEFINE_ARRAY(TYPE, NAME, SIZE) TYPE NAME##_InternalArray[SIZE]


#endif // CFG_H