#include "common/align.h"

#include <string.h>

/*
 * Row-major, out = R * in. Every row is a signed basis vector, so this table is
 * the 24-element rotation group of a cube and is closed under composition.
 *
 * Generated as R = Yaw(psi) * Base for the six bases named in the enum and the
 * four yaws, then checked for det = +1, orthonormality and pairwise
 * distinctness before being pasted here. Tests/UnitTest/test_align.c re-derives
 * every one of those properties through Align_Apply, so a typo in this table
 * fails the suite rather than the vehicle.
 */
STATIC int8_t const s_alignMatrix[eALIGN_COUNT][3][3] = {
    [eALIGN_CW0]            = { {  1,  0,  0 }, {  0,  1,  0 }, {  0,  0,  1 } },
    [eALIGN_CW90]           = { {  0, -1,  0 }, {  1,  0,  0 }, {  0,  0,  1 } },
    [eALIGN_CW180]          = { { -1,  0,  0 }, {  0, -1,  0 }, {  0,  0,  1 } },
    [eALIGN_CW270]          = { {  0,  1,  0 }, { -1,  0,  0 }, {  0,  0,  1 } },

    [eALIGN_FLIP_CW0]       = { {  1,  0,  0 }, {  0, -1,  0 }, {  0,  0, -1 } },
    [eALIGN_FLIP_CW90]      = { {  0,  1,  0 }, {  1,  0,  0 }, {  0,  0, -1 } },
    [eALIGN_FLIP_CW180]     = { { -1,  0,  0 }, {  0,  1,  0 }, {  0,  0, -1 } },
    [eALIGN_FLIP_CW270]     = { {  0, -1,  0 }, { -1,  0,  0 }, {  0,  0, -1 } },

    [eALIGN_ROLL90_CW0]     = { {  1,  0,  0 }, {  0,  0, -1 }, {  0,  1,  0 } },
    [eALIGN_ROLL90_CW90]    = { {  0,  0,  1 }, {  1,  0,  0 }, {  0,  1,  0 } },
    [eALIGN_ROLL90_CW180]   = { { -1,  0,  0 }, {  0,  0,  1 }, {  0,  1,  0 } },
    [eALIGN_ROLL90_CW270]   = { {  0,  0, -1 }, { -1,  0,  0 }, {  0,  1,  0 } },

    [eALIGN_ROLL270_CW0]    = { {  1,  0,  0 }, {  0,  0,  1 }, {  0, -1,  0 } },
    [eALIGN_ROLL270_CW90]   = { {  0,  0, -1 }, {  1,  0,  0 }, {  0, -1,  0 } },
    [eALIGN_ROLL270_CW180]  = { { -1,  0,  0 }, {  0,  0, -1 }, {  0, -1,  0 } },
    [eALIGN_ROLL270_CW270]  = { {  0,  0,  1 }, { -1,  0,  0 }, {  0, -1,  0 } },

    [eALIGN_PITCH90_CW0]    = { {  0,  0,  1 }, {  0,  1,  0 }, { -1,  0,  0 } },
    [eALIGN_PITCH90_CW90]   = { {  0, -1,  0 }, {  0,  0,  1 }, { -1,  0,  0 } },
    [eALIGN_PITCH90_CW180]  = { {  0,  0, -1 }, {  0, -1,  0 }, { -1,  0,  0 } },
    [eALIGN_PITCH90_CW270]  = { {  0,  1,  0 }, {  0,  0, -1 }, { -1,  0,  0 } },

    [eALIGN_PITCH270_CW0]   = { {  0,  0, -1 }, {  0,  1,  0 }, {  1,  0,  0 } },
    [eALIGN_PITCH270_CW90]  = { {  0, -1,  0 }, {  0,  0, -1 }, {  1,  0,  0 } },
    [eALIGN_PITCH270_CW180] = { {  0,  0,  1 }, {  0, -1,  0 }, {  1,  0,  0 } },
    [eALIGN_PITCH270_CW270] = { {  0,  1,  0 }, {  0,  0,  1 }, {  1,  0,  0 } },
};

void Align_Apply (eSensorAlign_t align, Vec3f const* pIn, Vec3f* pOut) {

    /*
     * An out-of-range alignment would index off the end of the table. It cannot
     * happen from the target headers - those are compile-time constants - but
     * identity is the one fallback that leaves the sample readable rather than
     * silently rotated by whatever followed the table in flash.
     */
    if (align >= eALIGN_COUNT) {
        align = eALIGN_CW0;
    }
    int8_t const (*pR)[3] = s_alignMatrix[align];

    // Read the input out first so pIn == pOut is safe.
    float const x = pIn->x;
    float const y = pIn->y;
    float const z = pIn->z;

    pOut->x = ((float)pR[0][0] * x) + ((float)pR[0][1] * y) + ((float)pR[0][2] * z);
    pOut->y = ((float)pR[1][0] * x) + ((float)pR[1][1] * y) + ((float)pR[1][2] * z);
    pOut->z = ((float)pR[2][0] * x) + ((float)pR[2][1] * y) + ((float)pR[2][2] * z);
}

eSensorAlign_t Align_Compose (eSensorAlign_t first, eSensorAlign_t second) {

    if (first >= eALIGN_COUNT) {
        first = eALIGN_CW0;
    }
    if (second >= eALIGN_COUNT) {
        second = eALIGN_CW0;
    }

    // Applying `first` then `second` is the product Rsecond * Rfirst, in that order.
    int8_t product[3][3];
    for (uint8_t i = 0; i < 3U; ++i) {
        for (uint8_t j = 0; j < 3U; ++j) {
            int8_t sum = 0;
            for (uint8_t k = 0; k < 3U; ++k) {
                sum = (int8_t)(sum + (s_alignMatrix[second][i][k] * s_alignMatrix[first][k][j]));
            }
            product[i][j] = sum;
        }
    }

    for (eSensorAlign_t a = 0; a < eALIGN_COUNT; ++a) {
        if (memcmp (s_alignMatrix[a], product, sizeof (product)) == 0) {
            return a;
        }
    }

    /*
     * Unreachable: the group is closed, so the product is always one of the 24.
     * test_align.c asserts that over all 576 pairs, which is what keeps it true.
     */
    return eALIGN_CW0;
}
