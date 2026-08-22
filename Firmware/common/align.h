#ifndef COMMON_ALIGN_H
#define COMMON_ALIGN_H

/*
 * Sensor mounting alignment: the 24 proper rotations that map a sensor die's
 * axes onto the vehicle body frame (FRD - x forward, y right, z down).
 *
 * NAMING. An entry names the rotation the stored matrix PERFORMS ON THE SAMPLE,
 * not the attitude the part is glued down at. Those two are inverses of each
 * other, and naming for the physical mounting means every reader has to invert
 * in their head before they can check a matrix against a bench reading. This
 * codebase has already been bitten twice by exactly that step: the Madgwick
 * Euler extraction shipped conjugated (KnownIssues 2.2), and the BMI323 remap
 * enum acquired two swapped names when its comment block was rekeyed from
 * register values to names. So the convention that removes the inversion is
 * worth more than the one that reads more naturally off the PCB. Pick the entry
 * that makes the six-face static test come out right; do not derive it from how
 * the chip looks on the board.
 *
 * The name reads in APPLICATION ORDER: eALIGN_FLIP_CW270 is the base rotation
 * (flip 180 about x) applied to the sample first, then the yaw (270 clockwise
 * about z, viewed from above). CW is clockwise seen from above, which in FRD is
 * a positive rotation about z - eALIGN_CW90 sends +x to +y.
 *
 * ONLY PROPER ROTATIONS ARE HERE, and that exclusion is the point. A signed
 * axis permutation with det = -1 is a reflection: it mirrors the frame, and a
 * gyro is a pseudovector, so every rotation sense on the mirrored axes comes
 * out backwards while the accelerometer still looks plausible. The BMI323's
 * register format (6 permutations x 3 independent sign bits) can express 24
 * such reflections and offers nothing to stop you selecting one. Enumerating
 * only the 24 valid rotations makes that unrepresentable rather than merely
 * discouraged.
 *
 * This is a rotation and nothing else. The accelerometer's g - a sign
 * convention (level and still reads (0, 0, +9.81); see msgs/proto/defs/sim.proto)
 * is a whole-vector negation, hence det = -1, hence NOT expressible here - it
 * belongs in the driver backend that turns a raw part into the project's units,
 * and it must never reach the gyro.
 */

#include "core/core.h"

#include <stdint.h>

typedef uint8_t eSensorAlign_t;
enum {
    eALIGN_CW0 = 0,
    eALIGN_CW90,
    eALIGN_CW180,
    eALIGN_CW270,

    eALIGN_FLIP_CW0,
    eALIGN_FLIP_CW90,
    eALIGN_FLIP_CW180,
    eALIGN_FLIP_CW270,

    eALIGN_ROLL90_CW0,
    eALIGN_ROLL90_CW90,
    eALIGN_ROLL90_CW180,
    eALIGN_ROLL90_CW270,

    eALIGN_ROLL270_CW0,
    eALIGN_ROLL270_CW90,
    eALIGN_ROLL270_CW180,
    eALIGN_ROLL270_CW270,

    eALIGN_PITCH90_CW0,
    eALIGN_PITCH90_CW90,
    eALIGN_PITCH90_CW180,
    eALIGN_PITCH90_CW270,

    eALIGN_PITCH270_CW0,
    eALIGN_PITCH270_CW90,
    eALIGN_PITCH270_CW180,
    eALIGN_PITCH270_CW270,

    eALIGN_COUNT
};

// Safe to call with pIn == pOut.
void Align_Apply (eSensorAlign_t align, Vec3f const* pIn, Vec3f* pOut);

/*
 * Apply `first`, then `second` - argument order is application order, matching
 * how the entry names read. Used to fold the two alignments a sample actually
 * goes through into the one that gets stored: the die's placement on the PCB
 * (a property of the board) composed with how the board is bolted into the
 * airframe (a property of the build).
 *
 * Always returns a real alignment: these 24 rotations are the rotation group of
 * a cube, so they are closed under composition and no product can fall outside
 * the set. That is what lets the result be an eSensorAlign_t rather than a
 * matrix, so callers keep storing one byte and keep calling Align_Apply.
 */
eSensorAlign_t Align_Compose (eSensorAlign_t first, eSensorAlign_t second);

#endif // COMMON_ALIGN_H
