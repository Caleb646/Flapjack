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

/*
 * How the flight controller is bolted into the airframe (common/align.h),
 * composed onto each sensor's own die placement (IMU_ALIGN / MAG_ALIGN in the
 * board header) to give the die-to-body rotation the device layer applies.
 *
 * It is split from those on purpose. The die placements are facts about the
 * PCB and differ per part; this is a fact about the build and is shared by
 * every sensor on the board. Folding them into one constant per sensor would
 * mean remounting the board required editing each of them in step, and letting
 * the IMU and mag drift out of agreement is exactly what breaks heading.
 */
#define CFG_BOARD_ALIGN                      eALIGN_CW0

/*
 * Madgwick beta, as the gyro measurement error it is derived from:
 * beta = sqrt(3/4) * DEG2RAD(this), i.e. the rate at which the ACCELEROMETER is
 * allowed to drag the estimate. It is not a noise figure - it is how much this
 * filter trusts gravity over the gyro.
 *
 * That trust is the problem, because for a rotorcraft in free flight the
 * accelerometer is not a gravity reference. Specific force points along the
 * thrust axis whatever the attitude, so the accel-implied bank works out as
 * (b/mg)*v - a drag-scaled VELOCITY reading, correct only once drag has
 * balanced and the vehicle has stopped accelerating. Angle mode closes on that
 * estimate (guidance.c), so the faster beta drags it, the harder the loop is
 * driven by a signal carrying no attitude information.
 *
 * BETA IS NOT THE LEVER FOR THAT, and this comment used to claim it was. The
 * argument ran: 5.0 gave 4.33 deg/s of slew regardless of error size, against
 * Betaflight's Mahony at Kp*error (imu_dcm_kp 0.25), so 1.4 "matches a
 * reference implementation at typical in-flight error". Both halves are wrong.
 *
 *   The comparison. Kp 0.25 is 14.3 deg/s per unit sin(error), so Betaflight
 *   corrects at 0.25 deg/s at 1 deg but 2.9 deg/s at 11.5 deg - the error 1.20
 *   actually measures. At that error Betaflight is roughly TWICE as
 *   accel-trusting as this filter, not less. The two curves cross near 5 deg,
 *   which is the only place the old claim held.
 *
 *   The mechanism. Gain sets how fast the estimate converges on the apparent
 *   vertical; it does not choose where that vertical is. Both filters have the
 *   same fixed point - estimated up aligned with measured specific force - so
 *   in trim the estimate ends up there at ANY gain. That is why 5.0 -> 1.4
 *   slowed 1.20 without removing it, and why no further cut will.
 *
 * Moving the fixed point is what CFG_NAV_TRANS_ACCEL_* below does. Beta is now
 * just a convergence-rate knob again, and with the reference corrected there is
 * a case for raising it - unmeasured, so it has not been raised.
 *
 * The cost is convergence time, which the arming interlock pays for
 * (ARM_ATTITUDE_SETTLE_US wants 3 s of stillness after the estimate settles).
 * Measure time-to-arm if this is lowered further.
 */
#define CFG_GYRO_MEASURE_ERROR_DEGS          1.4F
#define CFG_GYRO_MEASURE_DRIFT_DEGS          0.2F

/*
 * Horizontal estimator gains (common/filter.c HorizontalFilter_t), and the
 * kinematic accel correction they exist to feed.
 *
 * THE FAULT. Beta above sets how FAST the attitude filter converges on the
 * accelerometer's apparent vertical. It does not choose where that vertical
 * is. Both this filter and Betaflight's Mahony have the same fixed point -
 * estimated up aligned with measured specific force - so in trim the estimate
 * ends up wherever the accel points, at any gain. That is why cutting beta
 * 5.0 -> 1.4 slowed KnownIssues 1.20 without removing it, and why no further
 * cut will. The fix has to move the fixed point: subtract the vehicle's own
 * translational acceleration so what is left really is gravity.
 *
 * WHERE THE NUMBER COMES FROM. The horizontal filter integrates the IMU and
 * lets GPS argue with the result; its bias state converges on the standing
 * horizontal specific force the vehicle is NOT actually accelerating with, and
 * subtracting that leaves the real acceleration. This replaced differentiating
 * the GPS velocity directly, which is the same idea with the conditioning
 * inverted: the receiver reports velocity at 5 Hz with sigma = 0.1 m/s
 * (SensorGps.xml), so differencing it yields 0.71 m/s^2 of noise against a
 * ~2 m/s^2 signal. Measured, that put +/-1.25 m/s^2 - about 7 deg of false
 * tilt - into the gravity reference during a HOVER and departed the vehicle.
 * Integrating and correcting has the opposite noise behaviour.
 *
 * THE GAINS are the standard critically-damped pair for a time constant T:
 * kVel = 2/T, kBias = 1/T^2, with T = 2 s - long enough to average the
 * receiver's noise, short enough to track the seconds-scale transient 1.20
 * lives in. kPos only trims position and is deliberately slack; nothing flies
 * on position yet.
 *
 * MAX_BIAS is far larger than the vertical filter's 2.0, and for a different
 * reason. Here the bias absorbs ATTITUDE error, not a part's offset: a tilt
 * error theta leaks g*sin(theta) into the rotated horizontal accel, so 5.0
 * covers about 30 deg - the whole of CFG_ANGLE_MAX_DEG. Clamping tighter would
 * stop the state doing the job it exists for.
 *
 * MAX_AGE_MS drops the CORRECTION when fixes stop arriving. The filter keeps
 * integrating either way, but a bias estimate no receiver has argued with for
 * half a second must not keep rotating the gravity reference.
 *
 * MIN_SPEED is the one that stops this correcting things that do not need it.
 * The error being removed is (b/mg)*v, so it VANISHES as the vehicle stops
 * translating - while the noise in the bias estimate does not, because that
 * comes from the receiver's 0.1 m/s velocity noise and is there at any speed.
 * Below this there is nothing to correct and everything to inject, and 0.17
 * m/s^2 of injected horizontal accel is one degree of false tilt.
 *
 * Measured cost of not having it: on the ground, stationary, with the
 * correction ungated, the attitude estimate wandered several degrees instead
 * of settling, and the vehicle NEVER ARMED - mission.c wants the estimate held
 * within 1.0 deg for 3 s. It is gated on the FILTER's velocity rather than the
 * raw fix because that one is smoothed by the same filter this feeds.
 */
#define CFG_NAV_HORIZ_K_POS                  0.5F
#define CFG_NAV_HORIZ_K_VEL                  1.0F
#define CFG_NAV_HORIZ_K_BIAS                 0.25F
#define CFG_NAV_HORIZ_MAX_BIAS_MPS2          5.0F
#define CFG_NAV_HORIZ_AID_MAX_AGE_MS         500U
#define CFG_NAV_HORIZ_AID_MIN_SPEED_MPS      1.0F

/*
 * The translational-acceleration correction for KnownIssues 1.20, differenced
 * from the GPS velocity.
 *
 * WHY NOT THE HORIZONTAL FILTER'S BIAS, which is smoother and right there.
 * Because to correct attitude USING translation, the translation estimate must
 * not itself depend on attitude - otherwise the correction is self-referential.
 * The filter's (accelNed - accelBias) fails that on both terms: accelNed is the
 * accel rotated through qEst, and accelBias is driven by the velocity
 * innovation that same rotated accel feeds. Wiring it that way closed the
 * horizontal filter and the attitude filter around each other, both at
 * seconds-scale time constants, and measured 4.6-5.4 deg of estimate error
 * with the vehicle oscillating at 5.7 deg peak-to-peak - against 2.9 deg for
 * the differencing below, on the same plan at the same 489 Hz loop rate.
 *
 * Differencing GPS velocity is noisier and open-loop. That trade is the whole
 * point: it cannot feed back.
 *
 * CUTOFF is the noise/lag balance and it was swept, not guessed. The receiver
 * reports velocity at 5 Hz with sigma = 0.1 m/s (SensorGps.xml), which
 * differences to 0.71 m/s^2 of noise against a ~2 m/s^2 signal:
 *
 *   cutoff Hz   max tilt injected   limited by
 *      0.05           4.4 - 7.3     lag
 *      0.10           2.5 - 4.4     lag
 *      0.20           1.8 - 2.6     balanced
 *      0.30           2.3 - 2.6     noise
 *      1.00                 5.0     noise
 *
 * Both ends were seen for real. At 1 Hz the vehicle departed and crashed in a
 * HOVER, where the true correction is zero, on differencing noise alone.
 *
 * MIN_SPEED: the error is (b/mg)*v and vanishes as the vehicle stops
 * translating, while the noise does not. Below this there is nothing to correct
 * and everything to inject.
 *
 * MAX_MPS2 is a glitch guard, not a tune - this airframe cannot pull more than
 * g*tan(CFG_ANGLE_MAX_DEG) = 5.7 m/s^2 laterally. An over-range reading
 * DISABLES the correction rather than clipping it, because clipping would
 * quietly keep applying a bad direction at a legal size.
 */
#define CFG_NAV_TRANS_ACCEL_CUTOFF_HZ        0.2F
#define CFG_NAV_TRANS_ACCEL_MIN_SPEED_MPS    1.0F
#define CFG_NAV_TRANS_ACCEL_MAX_MPS2         8.0F

/*
 * What beta is multiplied by when the correction above is NOT available - no
 * fix, a stale one, or an out-of-range reading.
 *
 * READ THIS BEFORE TUNING IT. This is damage limitation, not a fix, and it
 * cannot be made into one. Without an independent velocity source there is no
 * signal on this vehicle that separates "the accelerometer is pointing at a
 * false vertical because we are translating" from "the accelerometer is right
 * and the estimate has drifted" - that is exactly what KnownIssues 1.16 says,
 * and it is why the residual monitor in nav.c is a monitor rather than a gate.
 * The two want opposite responses and the number below picks a point between
 * them for the whole flight:
 *
 *   toward 0   the estimate coasts on the gyro. 1.20 shrinks, and any real
 *              attitude error - bias, the 1.21 yaw scale error, a knock -
 *              persists longer because nothing is pulling it back.
 *   toward 1   the accelerometer keeps its authority, and so does 1.20.
 *
 * Rejected on the way here: gating on the residual itself. It reads large in
 * both cases, so suppressing on it would freeze a genuinely wrong estimate
 * precisely when it most needs correcting. A gyro-rate term was rejected too -
 * it addresses 1.21's rotation-scale error, not this one.
 *
 * 0.5 is a STARTING POINT, not a measured value. Halving beta halves the rate
 * the estimate is dragged at, which is the only thing it is claimed to do.
 * Measuring it wants a rig whose achieved loop rate is stable run to run
 * (CONTROL_RATE_FLOOR in bridge.py) and a --gps-stop probe.
 */
#define CFG_NAV_ACCEL_TRUST_UNAIDED          0.5F

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

/*
 * D-term low-pass corner, in Hz, applied to the measurement the derivative is
 * taken from (common/pid.c). 0 disables it and restores the previous
 * bit-for-bit behaviour.
 *
 * This is NOT a substitute for filtering the gyro itself, and it does not make
 * one unnecessary. The two do different jobs:
 *
 *   - P has flat gain against frequency and I has FALLING gain, so I actually
 *     attenuates noise. D differentiates: its gain RISES at +20 dB/decade, so it
 *     is the only term that amplifies whatever the sensor path leaves behind.
 *   - Filtering is paid for in phase lag, and lag at crossover is the scarce
 *     resource. Filtering the shared gyro hard enough to satisfy D would charge
 *     that lag to P and to the attitude estimator, neither of which needs it.
 *     Betaflight splits it for exactly this reason: gyro low-passes at 250-500
 *     Hz (deliberately mild) and D-term low-passes at 75-150 Hz (aggressive) -
 *     same signal, two cutoffs, because the terms differ in noise gain.
 *
 * What the sensor path does today, which this cannot substitute for: the BMI323
 * is set to ODR 400 Hz, bandwidth ODR/2 = 200 Hz, 16x averaging
 * (drivers/imu/bmi323.c), and devices/imu.c is still a documented pass-through.
 * Note the ODR EQUALS the control-loop rate, so there is no oversampling and
 * anything above 200 Hz has already aliased into the band by the time any
 * software sees it - no filter here or in imu.c can undo that. Fixing it means
 * raising the ODR above the loop rate, which is a sensor-layer change.
 * See ControlResearch.md 2.4.
 *
 * Measured on the host against the same JSBSim tiltrotor the gains were swept
 * on, with gyro noise injected at the READING (the FDM state stays clean, which
 * is what sensor noise actually is). At 400 Hz, 3 deg/s RMS noise, recovering a
 * 10 deg/s roll disturbance:
 *
 *     fc (Hz)   D-term RMS   servo activity   settle (clean gyro)
 *        off       0.0564         0.1384            0.128 s
 *         30       0.0124         0.0383            0.130 s
 *         60       0.0203         0.0562            0.130 s
 *        100       0.0273         0.0722            0.130 s
 *
 * So 60 Hz removes ~64% of the D term's noise energy and ~59% of the resulting
 * servo activity, and costs 2 ms of settling time on a clean gyro. At 10 deg/s
 * of noise - a badly balanced prop - the unfiltered loop is driven to 25.9 deg/s
 * peak by noise alone against the 10 deg/s disturbance; filtered it reaches
 * 15.3.
 *
 * Do NOT read that table as "lower is better". Settling barely moves across the
 * whole range because neither the FDM nor the host replica has any actuator
 * dynamics (KnownIssues 1.9), so neither can charge a filter for phase lag - and
 * lag is what a low cutoff spends. A PT1 lags by atan(f/fc): 4.8 deg at 5 Hz
 * here, but 9.5 deg at 30 Hz and 14 deg at 20 Hz, and real servo lag adds to it.
 * 60 Hz takes most of the available noise rejection while leaving the phase
 * budget mostly intact; going lower needs a model that can price the cost.
 *
 * The SIL cannot show any of this. Scripts/sim/jsbsim/systems models baro and
 * GPS noise but there is no IMU noise model at all, so the emulated gyro arrives
 * clean and the filter has nothing to remove. Re-tune against a real gyro's
 * noise spectrum on the bench.
 */
#define CFG_PID_DTERM_LPF_HZ                 60.0F
/*
 * Angle mode. The roll and pitch sticks command a BANK ANGLE, not a rate: an
 * outer P loop turns the angle error into the rate setpoint the rate PIDs above
 * already close on (guidance.c). Centre stick therefore means LEVEL rather than
 * "stop rotating", and the bank a pilot can reach - and so the vertical thrust
 * it costs - is bounded. Yaw stays a rate command; there is no attitude to
 * level to.
 *
 * Units: deg/s of rate demand per deg of angle error. Error is in DEGREES
 * because nav euler is (filter.c converts on the way out).
 *
 * Swept on the host against the JSBSim tiltrotor at 100/200/400 Hz (the loop is
 * IMU-paced, so one rate proves nothing), driving the real pid.c and mixer.c
 * including the tilt servos' rate limit and lag. These are the fastest values
 * that still overshoot ZERO on a full-scale 30 deg step:
 *
 *              step to 30 deg   recover 45 deg   overshoot
 *   roll         0.57 s           0.62 s          0.00 deg
 *   pitch        2.24 s           2.48 s          0.00 deg
 *
 * The two differ because the plants do, and the gap is not cosmetic: pitch acts
 * through the tilt servos - rate-limited and lagged - and its plant gain is
 * ~2.8x lower, so roll's 4.0 costs pitch 5-7 deg of overshoot. Do not raise
 * pitch to match roll without measuring the rotor height first; pitch authority
 * scales linearly with it and it is still an estimate (see tiltrotor.xml).
 *
 * Also unmodelled: motor/ESC lag. Roll is differential THRUST and the FDM
 * applies it instantly, so roll's margin here is optimistic in a way pitch's
 * is not. Re-check roll against a real airframe before raising it.
 */
#define CFG_ANGLE_ROLL_P                     4.0F
#define CFG_ANGLE_PITCH_P                    2.0F

/*
 * Bank/pitch ceiling in angle mode, in degrees. Full stick commands exactly
 * this. Lift scales with cos(angle), so 30 deg costs 13% of vertical thrust -
 * comfortably inside this airframe's 2:1 margin, and the bound is the point:
 * in the rate loop this replaced, any stick input left a permanent bank whose
 * lift cost nothing put a limit on.
 */
#define CFG_ANGLE_MAX_DEG                    30.0F

/*
 * Altitude-hold gains. Error is in METRES and the output is normalised throttle
 * trim about CFG_HOVER_THROTTLE - not the units the previous values (~1/1800)
 * were scaled for, which gave 0.0006 of throttle for a 1 m error and could not
 * hold anything.
 *
 * Swept on the host against the JSBSim tiltrotor at 100/200/400 Hz (the loop is
 * IMU-paced, so one rate proves nothing), with the MEASUREMENT NOISE MODELLED -
 * see CFG_ALT_HOLD_VZ_DAMPING for why that matters. On a 5 m step from the
 * ground: 0.38 m overshoot, zero steady-state error, 0.02 m of ripple.
 *
 * D is zero deliberately, and must stay zero. This axis takes its damping from
 * CFG_ALT_HOLD_VZ_DAMPING instead; a non-zero D here differentiates the baro
 * correction steps and does nothing else useful.
 */
#define CFG_PID_THROTTLE_P                   0.50F
#define CFG_PID_THROTTLE_I                   0.20F
#define CFG_PID_THROTTLE_D                   0.00F
#define CFG_PID_INTEGRAL_LIMIT               0.3F

#define CFG_MOTOR_MIN_THROTTLE               0.05F // 5% throttle
#define CFG_MOTOR_MAX_THROTTLE               1.00F // 100% throttle
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
 * Altitude hold.
 *
 * CFG_HOVER_THROTTLE is the collective this airframe needs to hold height,
 * and it is a FEEDFORWARD - the throttle PID only trims around it. Without it
 * the integrator has to manufacture the whole hover thrust itself, which is
 * slow and saturates against CFG_PID_INTEGRAL_LIMIT (0.3) well before it
 * gets there.
 *
 * 0.50 is MEASURED, not nominal: stepping the JSBSim tiltrotor open-loop
 * gives +0.09 m/s at 0.50 and +3.02 m/s at 0.55, so the trim point is 0.50
 * and the response either side is steep. Re-measure for a real airframe -
 * and note CFG_MOTOR_MAX_THROTTLE (0.40) sits BELOW this, so hardware cannot
 * hover until that driver cap is raised.
 */
#define CFG_HOVER_THROTTLE                   0.50F

/*
 * Throttle-stick authority in ALTITUDE_HOLD: full deflection walks the target
 * altitude at this rate. The stick commands a RATE, not a height, which is
 * what makes centre mean "hold" rather than "descend to the bottom of the
 * range".
 */
#define CFG_ALT_HOLD_CLIMB_RATE_MPS          1.0F

/*
 * Stick travel either side of centre that still counts as centred. CRSF
 * quantises to ~1 us and no real transmitter sits exactly on 1500, so without
 * a deadband the target creeps whenever the pilot is not touching the stick.
 */
#define CFG_ALT_HOLD_STICK_DEADBAND          0.05F

/*
 * Vertical damping, in throttle per m/s of climb-rate ERROR. This is the
 * altitude loop's D term, taken from the nav filter's climb rate rather than by
 * differentiating its altitude - which is why CFG_PID_THROTTLE_D is 0.
 *
 * "Error" and not "climb rate": control.c subtracts the COMMANDED climb rate
 * (guidance's sp.climb_rate) before applying this gain, so the term is zero in
 * a climb the loop is tracking and only fights vertical motion the pilot did
 * not ask for. Against raw vz this gain opposed the commanded climb too, which
 * cost 0.5 of throttle at the full 1 m/s stick and left the vehicle half a
 * metre under its own target for the whole climb.
 *
 * The distinction is not cosmetic, it is the whole difference between the
 * loop working and not. Both are estimates of the same quantity, but the
 * altitude estimate STEPS when a baro correction lands: measured in the SIL,
 * 0.041 m between 50 Hz corrections. Differentiated over one 2.5 ms control
 * tick that reads as 16.4 m/s, so a D of 0.50 asked for 8.2 of throttle trim
 * against a PID output clip of 1.0 - the term simply sat on the clip and
 * pinned the motors at 1.000 on 6% of frames. nav vel_ned is accelerometer-
 * aided and smooth, and the same damping off it saturates on 0.5%.
 *
 * Do not be tempted to drop the damping instead: thrust to altitude is a
 * double integrator, and a host sweep with this term removed oscillates 6.5 m
 * peak to peak or diverges outright.
 */
#define CFG_ALT_HOLD_VZ_DAMPING              0.50F

/*
 * Ceiling on total tilt-servo deflection, in normalised travel where 1.0 is full
 * mechanical travel - 90 deg on the sim backend (drivers/servo/sim.c). Pitch and
 * yaw share the tilt servos at weight 1.0 each and each PID output spans +/-1, so
 * an unbounded sum reaches +/-2 and pins the rotors horizontal on any saturated
 * rate loop.
 *
 * It is the threshold at which Mixer_MixServos scales the whole servo set down,
 * not a per-servo clip: the servos carry a shared pitch/yaw decomposition, and
 * clipping them one at a time attenuates the two axes by wildly different
 * amounts. See the comment there.
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