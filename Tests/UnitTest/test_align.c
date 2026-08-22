/*
 * Sensor mounting alignment, on the host.
 *
 * The table in common/align.c is 216 hand-transcribable integers, and a typo in
 * it is invisible on a bench: a wrong entry still produces a plausible-looking
 * accelerometer reading, and only the gyro sense or one particular attitude
 * gives it away. So the properties are re-derived here rather than compared
 * against a second copy of the same numbers - every matrix is recovered THROUGH
 * Align_Apply by feeding it basis vectors, so the test covers the accessor and
 * the arithmetic as well as the data.
 *
 * det = +1 is the load-bearing assertion. A signed axis permutation with
 * det = -1 is a reflection, and a reflection inverts the sense of every rotation
 * about a mirrored axis - which corrupts the gyro while leaving the
 * accelerometer looking fine. That is the shape of KnownIssues 2.2, and it is
 * the failure this enum exists to make unrepresentable.
 */

#include "unity/unity.h"

#include "common/align.h"

#include <math.h>

/*
 * Applying the transform to basis vector e_j returns column j of the matrix, so
 * three calls recover the whole thing without align.c exposing its table.
 */
static void RecoverMatrix (eSensorAlign_t align, float R[3][3]) {

    Vec3f const basis[3] = {
        { .x = 1.0F, .y = 0.0F, .z = 0.0F },
        { .x = 0.0F, .y = 1.0F, .z = 0.0F },
        { .x = 0.0F, .y = 0.0F, .z = 1.0F },
    };

    for (int j = 0; j < 3; ++j) {
        Vec3f out = { 0 };
        Align_Apply (align, &basis[j], &out);
        R[0][j] = out.x;
        R[1][j] = out.y;
        R[2][j] = out.z;
    }
}

static float Determinant (float const R[3][3]) {
    return (R[0][0] * ((R[1][1] * R[2][2]) - (R[1][2] * R[2][1]))) -
           (R[0][1] * ((R[1][0] * R[2][2]) - (R[1][2] * R[2][0]))) +
           (R[0][2] * ((R[1][0] * R[2][1]) - (R[1][1] * R[2][0])));
}

void setUp (void) {
}

void tearDown (void) {
}

void test_Align_EveryEntryIsASignedPermutation (void) {

    for (eSensorAlign_t a = 0; a < eALIGN_COUNT; ++a) {
        float R[3][3];
        RecoverMatrix (a, R);

        for (int i = 0; i < 3; ++i) {
            int rowNonZero = 0;
            int colNonZero = 0;
            for (int j = 0; j < 3; ++j) {
                TEST_ASSERT_TRUE_MESSAGE (R[i][j] == 0.0F || R[i][j] == 1.0F || R[i][j] == -1.0F,
                                          "alignment entry is not 0 or +/-1");
                rowNonZero += (R[i][j] != 0.0F) ? 1 : 0;
                colNonZero += (R[j][i] != 0.0F) ? 1 : 0;
            }
            TEST_ASSERT_EQUAL_INT_MESSAGE (1, rowNonZero, "row does not hold exactly one axis");
            TEST_ASSERT_EQUAL_INT_MESSAGE (1, colNonZero, "column does not hold exactly one axis");
        }
    }
}

// The reason the enum exists: no reflections, so no inverted gyro senses.
void test_Align_EveryEntryIsAProperRotation (void) {

    for (eSensorAlign_t a = 0; a < eALIGN_COUNT; ++a) {
        float R[3][3];
        RecoverMatrix (a, R);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE (1e-6F, 1.0F, Determinant (R),
                                          "alignment is a reflection, not a rotation");
    }
}

void test_Align_EveryEntryIsOrthonormal (void) {

    for (eSensorAlign_t a = 0; a < eALIGN_COUNT; ++a) {
        float R[3][3];
        RecoverMatrix (a, R);

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                float dot = 0.0F;
                for (int k = 0; k < 3; ++k) {
                    dot += R[i][k] * R[j][k];
                }
                TEST_ASSERT_FLOAT_WITHIN (1e-6F, (i == j) ? 1.0F : 0.0F, dot);
            }
        }
    }
}

/*
 * 24 names must be 24 different rotations. A duplicate would mean one of the
 * cube rotations is missing from the enum entirely, so a board mounted that way
 * could not be described at all.
 */
void test_Align_EntriesAreAllDistinct (void) {

    for (eSensorAlign_t a = 0; a < eALIGN_COUNT; ++a) {
        float Ra[3][3];
        RecoverMatrix (a, Ra);

        for (eSensorAlign_t b = (eSensorAlign_t)(a + 1U); b < eALIGN_COUNT; ++b) {
            float Rb[3][3];
            RecoverMatrix (b, Rb);

            bool identical = true;
            for (int i = 0; i < 3 && identical; ++i) {
                for (int j = 0; j < 3; ++j) {
                    if (Ra[i][j] != Rb[i][j]) {
                        identical = false;
                        break;
                    }
                }
            }
            TEST_ASSERT_FALSE_MESSAGE (identical, "two alignments name the same rotation");
        }
    }
}

/*
 * The structural checks above pass for any 24 distinct rotations, including a
 * set where two names got swapped - which is exactly what happened to the
 * BMI323 ZXY/XZY enum. These pin names to rotations.
 */
void test_Align_NamedCasesMatchTheirRotation (void) {

    Vec3f const v     = { .x = 1.0F, .y = 2.0F, .z = 3.0F };
    Vec3f const xAxis = { .x = 1.0F, .y = 0.0F, .z = 0.0F };
    Vec3f const zAxis = { .x = 0.0F, .y = 0.0F, .z = 1.0F };
    Vec3f out         = { 0 };

    Align_Apply (eALIGN_CW0, &v, &out);
    TEST_ASSERT_EQUAL_FLOAT (1.0F, out.x);
    TEST_ASSERT_EQUAL_FLOAT (2.0F, out.y);
    TEST_ASSERT_EQUAL_FLOAT (3.0F, out.z);

    // CW is clockwise seen from above, which in FRD (z down) sends +x to +y.
    Align_Apply (eALIGN_CW90, &xAxis, &out);
    TEST_ASSERT_EQUAL_FLOAT (0.0F, out.x);
    TEST_ASSERT_EQUAL_FLOAT (1.0F, out.y);
    TEST_ASSERT_EQUAL_FLOAT (0.0F, out.z);

    Align_Apply (eALIGN_CW180, &xAxis, &out);
    TEST_ASSERT_EQUAL_FLOAT (-1.0F, out.x);
    TEST_ASSERT_EQUAL_FLOAT (0.0F, out.y);

    // FLIP is 180 about x: forward survives, right and down both invert.
    Align_Apply (eALIGN_FLIP_CW0, &v, &out);
    TEST_ASSERT_EQUAL_FLOAT (1.0F, out.x);
    TEST_ASSERT_EQUAL_FLOAT (-2.0F, out.y);
    TEST_ASSERT_EQUAL_FLOAT (-3.0F, out.z);

    // ROLL90 is 90 about x: down goes to the left, i.e. +z to -y.
    Align_Apply (eALIGN_ROLL90_CW0, &zAxis, &out);
    TEST_ASSERT_EQUAL_FLOAT (0.0F, out.x);
    TEST_ASSERT_EQUAL_FLOAT (-1.0F, out.y);
    TEST_ASSERT_EQUAL_FLOAT (0.0F, out.z);

    // PITCH90 is 90 about y: down goes to the front, i.e. +z to +x.
    Align_Apply (eALIGN_PITCH90_CW0, &zAxis, &out);
    TEST_ASSERT_EQUAL_FLOAT (1.0F, out.x);
    TEST_ASSERT_EQUAL_FLOAT (0.0F, out.y);
    TEST_ASSERT_EQUAL_FLOAT (0.0F, out.z);
}

/*
 * The flapjack v1 BMI323 sits at YXZ with all three sign bits negative - the
 * only record of its placement, carried unchanged since the first IMU commit
 * (4ac42ab). Per bmixxx.h, YXZ is "x=y; y=x; z=z", so with the inversions that
 * register config is out = (-in.y, -in.x, -in.z). This asserts eALIGN_FLIP_CW270
 * is that same rotation, which is what makes it the right constant for
 * IMU_ALIGN. Note it is det +1, so the register config carries no part of the
 * accelerometer g - a negation - that is genuinely absent from the hardware
 * path and belongs in the BMI323 backend.
 */
void test_Align_FlapjackImuMatchesBmi323RegisterConfig (void) {

    Vec3f const in = { .x = 1.0F, .y = 2.0F, .z = 3.0F };
    Vec3f out      = { 0 };

    Align_Apply (eALIGN_FLIP_CW270, &in, &out);
    TEST_ASSERT_EQUAL_FLOAT (-2.0F, out.x);
    TEST_ASSERT_EQUAL_FLOAT (-1.0F, out.y);
    TEST_ASSERT_EQUAL_FLOAT (-3.0F, out.z);
}

// devices/imu.c rotates its samples in place, so this is the calling pattern.
void test_Align_IsInPlaceSafe (void) {

    for (eSensorAlign_t a = 0; a < eALIGN_COUNT; ++a) {
        Vec3f const in = { .x = 1.0F, .y = 2.0F, .z = 3.0F };
        Vec3f apart    = { 0 };
        Vec3f inPlace  = in;

        Align_Apply (a, &in, &apart);
        Align_Apply (a, &inPlace, &inPlace);

        TEST_ASSERT_EQUAL_FLOAT (apart.x, inPlace.x);
        TEST_ASSERT_EQUAL_FLOAT (apart.y, inPlace.y);
        TEST_ASSERT_EQUAL_FLOAT (apart.z, inPlace.z);
    }
}

void test_Align_PreservesMagnitude (void) {

    Vec3f const in    = { .x = 0.3F, .y = -9.7F, .z = 1.25F };
    float const inMag = sqrtf ((in.x * in.x) + (in.y * in.y) + (in.z * in.z));

    for (eSensorAlign_t a = 0; a < eALIGN_COUNT; ++a) {
        Vec3f out = { 0 };
        Align_Apply (a, &in, &out);
        float const outMag = sqrtf ((out.x * out.x) + (out.y * out.y) + (out.z * out.z));
        TEST_ASSERT_FLOAT_WITHIN (1e-5F, inMag, outMag);
    }
}

void test_Align_OutOfRangeFallsBackToIdentity (void) {

    Vec3f const in = { .x = 1.0F, .y = 2.0F, .z = 3.0F };
    Vec3f out      = { 0 };

    Align_Apply ((eSensorAlign_t)eALIGN_COUNT, &in, &out);
    TEST_ASSERT_EQUAL_FLOAT (1.0F, out.x);
    TEST_ASSERT_EQUAL_FLOAT (2.0F, out.y);
    TEST_ASSERT_EQUAL_FLOAT (3.0F, out.z);
}

/*
 * The definitional property, over all 576 pairs: composing and then applying
 * must equal applying one after the other. This is the whole contract in one
 * assertion - it pins the argument ORDER (a reversed Align_Compose passes every
 * other test in this file, since the group is closed either way), it proves
 * closure, and it proves the table search finds the right entry.
 */
void test_AlignCompose_MatchesApplyingBothInOrder (void) {

    Vec3f const in = { .x = 1.0F, .y = 2.0F, .z = 3.0F };

    for (eSensorAlign_t a = 0; a < eALIGN_COUNT; ++a) {
        for (eSensorAlign_t b = 0; b < eALIGN_COUNT; ++b) {
            Vec3f stepwise = { 0 };
            Align_Apply (a, &in, &stepwise);
            Align_Apply (b, &stepwise, &stepwise);

            Vec3f composed = { 0 };
            Align_Apply (Align_Compose (a, b), &in, &composed);

            TEST_ASSERT_EQUAL_FLOAT (stepwise.x, composed.x);
            TEST_ASSERT_EQUAL_FLOAT (stepwise.y, composed.y);
            TEST_ASSERT_EQUAL_FLOAT (stepwise.z, composed.z);
        }
    }
}

// A board bolted in square (CFG_BOARD_ALIGN = eALIGN_CW0) must not move the die.
void test_AlignCompose_IdentityIsNeutral (void) {

    for (eSensorAlign_t a = 0; a < eALIGN_COUNT; ++a) {
        TEST_ASSERT_EQUAL_UINT8 (a, Align_Compose (a, eALIGN_CW0));
        TEST_ASSERT_EQUAL_UINT8 (a, Align_Compose (eALIGN_CW0, a));
    }
}

void test_AlignCompose_EveryAlignmentHasExactlyOneInverse (void) {

    for (eSensorAlign_t a = 0; a < eALIGN_COUNT; ++a) {
        int inverses = 0;
        for (eSensorAlign_t b = 0; b < eALIGN_COUNT; ++b) {
            if (Align_Compose (a, b) == eALIGN_CW0) {
                ++inverses;
            }
        }
        TEST_ASSERT_EQUAL_INT_MESSAGE (1, inverses, "alignment does not have exactly one inverse");
    }
}

// Names, not just structure: two 90 degree yaws are one 180 degree yaw.
void test_AlignCompose_NamedCases (void) {

    TEST_ASSERT_EQUAL_UINT8 (eALIGN_CW180, Align_Compose (eALIGN_CW90, eALIGN_CW90));
    TEST_ASSERT_EQUAL_UINT8 (eALIGN_CW270, Align_Compose (eALIGN_CW90, eALIGN_CW180));
    TEST_ASSERT_EQUAL_UINT8 (eALIGN_CW0, Align_Compose (eALIGN_CW90, eALIGN_CW270));

    // The base rotation is applied first, so flip-then-yaw composes to the name.
    TEST_ASSERT_EQUAL_UINT8 (eALIGN_FLIP_CW270, Align_Compose (eALIGN_FLIP_CW0, eALIGN_CW270));
}

void test_AlignCompose_OutOfRangeFallsBackToIdentity (void) {

    TEST_ASSERT_EQUAL_UINT8 (eALIGN_CW90, Align_Compose (eALIGN_CW90, (eSensorAlign_t)eALIGN_COUNT));
    TEST_ASSERT_EQUAL_UINT8 (eALIGN_CW90, Align_Compose ((eSensorAlign_t)eALIGN_COUNT, eALIGN_CW90));
}

int main (void) {
    UNITY_BEGIN ();
    RUN_TEST (test_Align_EveryEntryIsASignedPermutation);
    RUN_TEST (test_Align_EveryEntryIsAProperRotation);
    RUN_TEST (test_Align_EveryEntryIsOrthonormal);
    RUN_TEST (test_Align_EntriesAreAllDistinct);
    RUN_TEST (test_Align_NamedCasesMatchTheirRotation);
    RUN_TEST (test_Align_FlapjackImuMatchesBmi323RegisterConfig);
    RUN_TEST (test_Align_IsInPlaceSafe);
    RUN_TEST (test_Align_PreservesMagnitude);
    RUN_TEST (test_Align_OutOfRangeFallsBackToIdentity);
    RUN_TEST (test_AlignCompose_MatchesApplyingBothInOrder);
    RUN_TEST (test_AlignCompose_IdentityIsNeutral);
    RUN_TEST (test_AlignCompose_EveryAlignmentHasExactlyOneInverse);
    RUN_TEST (test_AlignCompose_NamedCases);
    RUN_TEST (test_AlignCompose_OutOfRangeFallsBackToIdentity);
    return UNITY_END ();
}
