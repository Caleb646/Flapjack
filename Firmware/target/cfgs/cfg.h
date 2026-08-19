#ifndef CFGS_CFG_H
#define CFGS_CFG_H

// to disable redefinition warnings for default target macros
#pragma GCC system_header

/*
 * CM7 owns the debug UART: it inits the peripheral and runs the SerialLink TX
 * task, and the FreeRTOS objects guarding that link are CM7's. CM4 therefore
 * cannot drive the sinks itself - it hands off via SyncNotifyTaskUartOut and
 * the TX task drains the queue with SyncProcessTasks().
 */
#define CFG_PRIMARY_LOGGER                   CM7_CPUID
#define CFG_LOGGER_SHOULD_BLOCK_ON_OVERWRITE 1U

/*
 * Compile-time log verbosity. Anything above CFG_LOG_LEVEL is removed by the
 * preprocessor - the call, its format string and its arguments all disappear
 * from the image rather than being suppressed at runtime.
 *
 * LOG_DATA and its derivatives sit at DEBUG: they carry no severity of their
 * own, they are the heaviest users of the link, and they are what the GUI
 * plots - so building below DEBUG silently stops those plots updating.
 */
#define CFG_LOG_LEVEL_NONE                   0U
#define CFG_LOG_LEVEL_ERROR                  1U
#define CFG_LOG_LEVEL_WARN                   2U
#define CFG_LOG_LEVEL_INFO                   3U
#define CFG_LOG_LEVEL_DEBUG                  4U

#ifndef CFG_LOG_LEVEL /* -D CFG_LOG_LEVEL=n overrides (board.py --log-level) */
#define CFG_LOG_LEVEL CFG_LOG_LEVEL_DEBUG
#endif
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
 * Vertical complementary filter (common/filter.c), as the standard third-order
 * set for a time constant T: kAlt = 3/T, kVel = 3/T^2, kBias = 1/T^3.
 *
 * T = 1 s. That trades roughly a second of lag behind a step in TRUE altitude
 * for heavy suppression of baro noise, which is the right way round here: the
 * accel path already carries the fast response, so all the baro has to do is
 * stop it drifting. Shorten T only if the baro turns out quieter than expected
 * on real hardware - the SIL's baro is noise-free, so it cannot answer that.
 *
 * The bias clamp is deliberately loose. 2 m/s^2 is far beyond any healthy IMU
 * (a good part sits under 0.5), so it never fights a real bias; it exists only
 * to stop the integrator running away if the datum is wrong or the part failed.
 */
#define CFG_ALT_FILTER_K_ALT                 3.0F
#define CFG_ALT_FILTER_K_VEL                 3.0F
#define CFG_ALT_FILTER_K_BIAS                1.0F
#define CFG_ALT_FILTER_MAX_BIAS_MPS2         2.0F

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
 * Mixer output band. Deliberately NOT the CFG_MOTOR_* values above: those are a
 * bench cap the DShot driver applies on its own (dshot.c), and 0.40 sits below
 * the ~0.50 this airframe needs to hover. The mixer desaturates against the band
 * it is actually allowed to use, so raise the driver cap before flying hardware
 * or it will re-clip the differential this band exists to protect.
 *
 * The idle floor is what keeps roll available at low throttle: differential
 * thrust is the only roll effector on a bicopter and a motor commanded to zero
 * has none. Props therefore spin whenever the vehicle is ARMED, not just when
 * the throttle stick is raised.
 */
#define CFG_MIXER_IDLE_THROTTLE              0.05F
#define CFG_MIXER_MAX_THROTTLE               1.00F

/*
 * Ceiling on total tilt-servo deflection, in normalised travel where 1.0 is full
 * mechanical travel - 90 deg on the sim backend (drivers/servo/sim.c). Pitch and
 * yaw share the tilt servos at weight 1.0 each and each PID output spans +/-1, so
 * an unbounded sum reaches +/-2 and pins the rotors horizontal on any saturated
 * rate loop.
 *
 * 0.3333 is +/-30 deg, and it DOES bind in normal flight - measured, not assumed.
 * Unlimited, the `hover` plan peaks at 0.760 rad (43.5 deg) on the takeoff
 * transient; clamped it peaks at the limit and hover still passes with the same
 * altitude tracking (0.44 m nav error against a 1.5 m budget). Raise it if a
 * measured airframe turns out to need the extra travel, but do not assume the
 * loop only asks for this much when something has gone wrong.
 */
#define CFG_MIXER_SERVO_TRAVEL_LIMIT         0.3333F

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