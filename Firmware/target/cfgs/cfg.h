#ifndef CFGS_CFG_H
#define CFGS_CFG_H

// to disable redefinition warnings for default target macros
#pragma GCC system_header

#if defined(SINGLE_CORE)
/* No CM4 exists to drain CM7's ring buffer, so CM7 must own the UART itself. */
#define CFG_PRIMARY_LOGGER                   CM7_CPUID
#else
#define CFG_PRIMARY_LOGGER                   CM4_CPUID
#endif
#define CFG_LOGGER_SHOULD_BLOCK_ON_OVERWRITE 1U
#define LOG_DATA_TYPE_ATTITUDE               "attitude"
#define LOG_DATA_TYPE_PID_ATTITUDE           "pid_attitude"
#define LOG_DATA_TYPE_IMU_CALIB              "imu_calib"
#define LOG_DATA_TYPE_IMU_DATA               "imu_data"
#define LOG_DATA_TYPE_RAW_IMU_DATA           "imu_raw_data"
#define LOG_DATA_TYPE_ACTUATORS              "actuators"

// main control loop update rate in Hz
#define CFG_LOOP_UPDATE_RATE_HZ              200U

#define CFG_GYRO_MEASURE_ERROR_DEGS          5.0F
#define CFG_GYRO_MEASURE_DRIFT_DEGS          0.2F

/*
 * Rate-loop PID.
 *
 * The output IS the normalised mixer command: -1.0 .. +1.0 spans full actuator
 * travel, which is exactly what mixer.c consumes. Error is in deg/s, so every
 * gain has units of "mixer command per deg/s of rate error".
 *
 * The clip is symmetric because a non-negative lower bound leaves every axis
 * unable to correct a negative error, which is not a controller.
 *
 * Gains are the previous values divided by 180. control.c used to clip the PID
 * to +/-5 and then divide by CONTROL_MAX_RATE_DEG_S (180), so the mixer could
 * never see more than 5/180 = 2.8% of travel on any axis. Folding that 180 into
 * the gains keeps the small-signal response identical to what was there before
 * while giving the loop its full authority back - no retune implied by this
 * change on its own.
 *
 * ROLL/PITCH D are 1/10 of the rescaled values, from a measured sweep against
 * the SIL flight model. The actuators are extremely powerful relative to the
 * inertias - full differential thrust is ~9900 deg/s^2 of roll against
 * ixx = 0.012 - so the D term closes an almost-algebraic loop of gain Kd*G. At
 * the old value Kd*G was ~2.8 for roll and ~1.0 for pitch, i.e. >= 1, and the
 * loop limit-cycled at the sample rate: a 10 deg/s disturbance grew to 745
 * deg/s and the vehicle departed the instant it armed.
 *
 * The stability boundary sits between 0.2x and 0.25x, but 0.2x is only stable
 * at 400 Hz - it diverges at 200 and 100 Hz. 0.1x is stable at all three with
 * ~2x margin, which matters because the loop rate follows the IMU and is not
 * fixed. YAW D is left alone: its Kd*G is ~0.007, nowhere near the boundary.
 *
 * Measured with these values (20 deg/s disturbance, no overshoot, identical at
 * 100/200/400 Hz): roll settles in 0.67 s, yaw 0.12 s, pitch 1.47 s.
 *
 * Pitch is the slow axis because its plant gain is ~2.8x lower than roll's, and
 * raising its P would even that out - but pitch authority depends entirely on
 * the ESTIMATED 60 mm rotor height above the CG in tiltrotor.xml. Measure that
 * before tuning pitch, or the gain is fitted to a guess.
 *
 * CAVEAT: the FDM has no actuator dynamics - the tilt servos move instantly.
 * Real servo lag eats phase margin, so treat these as a starting point for a
 * bench/flight tune, not a finished one, and do not raise P blind.
 *
 * CFG_PID_INTEGRAL_LIMIT is in OUTPUT units: it caps how much of the actuator
 * the integrator may claim, independent of the I gain. It used to bound the
 * accumulated error (25 error-seconds), which at these gains let the I-term
 * reach only ~4% of travel; 0.3 gives it enough authority to trim a steady
 * disturbance without letting it dominate the loop.
 */
#define CFG_PID_MAX_VALUE                    1.0F
#define CFG_PID_MIN_VALUE                    (-CFG_PID_MAX_VALUE)
#define CFG_PID_ROLL_P                       0.00111111F
#define CFG_PID_ROLL_I                       0.00166667F
#define CFG_PID_ROLL_D                       0.0000277778F
#define CFG_PID_PITCH_P                      0.00111111F
#define CFG_PID_PITCH_I                      0.00166667F
#define CFG_PID_PITCH_D                      0.0000277778F
#define CFG_PID_YAW_P                        0.00166667F
#define CFG_PID_YAW_I                        0.00027778F
#define CFG_PID_YAW_D                        0.00000083F
#define CFG_PID_THROTTLE_P                   0.00055556F
#define CFG_PID_THROTTLE_I                   0.00111111F
#define CFG_PID_THROTTLE_D                   0.00027778F
#define CFG_PID_INTEGRAL_LIMIT               0.3F

#define CFG_MOTOR_MIN_THROTTLE               0.20F // 20% throttle
#define CFG_MOTOR_MAX_THROTTLE               0.40F // 40% throttle
#define CFG_MOTOR_STARTUP_THROTTLE           0.25F // 25% throttle

/*
 * PID mixing magnitudes for the Motors. They determine how much each PID axis contributes to the motor throttle.
 * For example (PID values are between -1 to 1) if a PID pitch value of 0.5 is received the motor should
 * increase its throttle to maintain the current altitude while allowing the drone to pitch up.
 *
 *
 * PID mixing directions for the Motors. They determine the direction of each PID axis contribution to the motor throttle.
 * For example (PID values are between -1 to 1) if a PID roll value of 0.5
 * is received the left motor should increase its throttle and the right motor should decrease its throttle.
 *
 * NOTE: A PID pitch value should always increase the throttle of both motors regardless of the sign of the PID pitch value.
 */
#endif // CFGS_CFG_H