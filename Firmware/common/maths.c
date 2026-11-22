#include "common/maths.h"

float clipf32 (float v, float lower, float upper) {
    /*
     * `!(v >= lower)` rather than `v < lower`, so that NaN takes this branch.
     *
     * Every comparison against NaN is false, so the natural form returned NaN
     * unchanged and quietly broke the one guarantee this function exists to
     * make: that its result lies within [lower, upper]. Callers cast that result
     * to an integer - Mixer_MixServos does `(uint16_t)clipf32(us, 500, 2500)` -
     * and a float-to-integer conversion of NaN is undefined behaviour, which on
     * this target lands on 0. A 0 us pulse width is -1.5x full travel, i.e. a
     * servo commanded 135 deg past its own mechanical stop; that exact value
     * (-2.356 rad) is what showed up in the SIL when the flight model diverged
     * and fed NaN back to the FC (KnownIssues 1.14).
     *
     * NaN clamps to `lower` because that is the cheapest way to restore the
     * range guarantee, NOT because full negative deflection is a good failsafe.
     * A caller that needs NaN to mean "neutral" has to say so itself - this
     * function's only promise is that what comes out is in range.
     */
    if (!(v >= lower)) {
        return lower;
    }
    if (v > upper) {
        return upper;
    }
    return v;
}
