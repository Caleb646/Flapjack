# Flapjack — Betaflight Control-Law Comparison: Rate PID, Angle vs Rate, Altitude Hold, Mixing

**Reference:** `Research/betaflight` at `FC_VERSION 2026.12 alpha` (post-`USE_WING` split, so
`alt_hold_multirotor.c` / `autopilot_multirotor.c` are the multirotor paths). Companion to
[KnownIssues.md](KnownIssues.md) — §1.9 there flags the provisional rate tune; this document
quantifies it against a reference implementation, and §2.4 below is where the D-term filter it
used to also flag was designed and shipped.

## Bottom line up front

**The architecture is the same.** Both firmwares are: stick → *(angle mode)* outer P loop on
attitude error → rate setpoint in deg/s → rate PID closing on raw gyro → normalised axis
command → mixer. Altitude hold in both is an outer loop trimming around a **hover-throttle
feedforward**, with the pilot's throttle stick commanding a **climb rate** that walks a target
altitude. The convergence goes all the way down to the tables: Betaflight ships a bicopter mixer,
and its **motor mix is identical to Flapjack's, field for field** (§6.2). Flapjack did not
converge on a different design — it converged on the same one.

**Where Flapjack is genuinely better:** its integrator is the more correct of the two. It has
conditional-integration anti-windup against *output* saturation, and it back-calculates the
stored integral when the I-term clamp bites. Betaflight has neither — it clamps I to a fixed
band and leans on `iterm_relax` and airmode instead. Its motor desaturation is also the same
"scale the differential, then slide the block" algorithm Betaflight's airmode uses, down to the
tie-break (§6.3).

**The five real gaps, in order:**

| # | Gap | Consequence |
|---|-----|-------------|
| 1 | ~~**No gyro or D-term filtering anywhere**~~ — **D path FIXED, see §2.4** | A 60 Hz PT1 on the D path removes 64 % of the D term's noise energy and 59 % of the servo activity it drives, for 2 ms of settling. The **gyro** path (P and I, and the attitude estimator) is still unfiltered — BF has 2 gyro LPFs + 3 dynamic notches there. |
| 2 | ~~**Altitude damping is `-k·vz`, not `k·(vz_target − vz)`**~~ — **FIXED, see §5.3** | Was: a commanded 1 m/s climb fought its own damping term. Measured in the SIL at **−0.52 m** of climb lag and **+0.34 m** of balloon on stick release; now **−0.04 m** and **+0.09 m**. |
| 3 | ~~**The servo mixer clips per actuator; the motor mixer does not**~~ — **FIXED, see §6.4** | Independent clipping hit the subordinate axis ~5× harder than the dominant one. Measured in flight: pitch kept 81–87 % against yaw's 97–98 %; uniform scaling keeps 94–96 % of both. Smaller than first claimed — saturation is rare and shallow on this tune. |
| 4 | **No rate-loop feedforward** | Both the rate loop and the altitude loop are pure feedback. BF has `Kf` on rate and `altitudeF` on climb rate. |
| 5 | **No tilt compensation in altitude hold** | cfg.h itself notes 30° of bank costs 13 % of lift; nothing gives it back. BF multiplies throttle by `1/cos(tilt)`. |

**Two calibration observations** (not bugs — measurements worth knowing):

- Normalised to the same units, Flapjack's rate **I gain is ~12× lower** than Betaflight's
  (~70× on yaw): an integral time constant of 0.67 s against Betaflight's 0.074 s.
- Flapjack's **altitude loop is ~18× higher loop gain** than Betaflight's default, after
  correcting for hover throttle. Defensible against a noise-free SIL baro; it will not survive a
  real baro in prop wash.

---

## 1. Loop architecture and timing

|  | Flapjack | Betaflight |
|---|---|---|
| Rate loop rate | IMU-paced, whatever the driver delivers | Fixed `targetPidLooptime` (typ. 8 kHz / 3.2 kHz) |
| `dt` source | **Measured**, clipped to [0.5 ms, 20 ms] | **Fixed** `pidRuntime.dT`, jitter deliberately ignored |
| Angle loop | Separate task (`guidance`), nav-paced | Inline in `pidController`, same rate as the rate loop |
| Altitude estimate | `nav` task, IMU-paced (predict) + baro-paced (correct) | `TASK_ALTITUDE` at **100 Hz** |
| Altitude hold | `control` task, IMU-paced | `TASK_ALTHOLD` at **100 Hz** |
| Structure | 4 tasks over umsg pub/sub | One monolithic `pidController()` + scheduler tasks |

Betaflight's use of a **fixed `dt`** is a deliberate decision with a comment attached
([pid.c:1331](Research/betaflight/src/main/flight/pid.c#L1331)):

> `Divide rate change by dT to get differential (ie dr/dt). dT is fixed and calculated from the
> target PID loop time. This is done to avoid DTerm spikes that occur with dynamically calculated
> deltaT whenever another task causes the PID loop execution to be delayed.`

Flapjack measures `dt` and clips it ([control.c:127](Firmware/tasks/control/control.c#L127)),
with its own comment explaining the clip. Both approaches defend against the same failure; the
difference is that a measured `dt` still lets scheduling jitter modulate the D term
proportionally, where a fixed `dt` converts jitter into a *slope* error the LPF then absorbs.
Flapjack's choice is the more defensible one for a loop whose rate genuinely varies with the IMU
— but it is only equivalent to Betaflight's once there is a D-term LPF in front of it (§2.4).

One dead knob: **`CFG_LOOP_UPDATE_RATE_HZ` (200) has no reader.** `grep` finds only its own
definition at [cfg.h:42](Firmware/target/cfgs/cfg.h#L42). The loop rate is set entirely by the
IMU driver. Not touching it here (Rule 3), but it is misleading as written.

---

## 2. The rate PID

### 2.1 Term by term

| Term | Flapjack `Pid_UpdateAxis` | Betaflight `pidController` |
|---|---|---|
| P | `Kp · error` | `Kp · error · tpaFactor`, + anti-gravity P boost, + yaw P LPF (100 Hz) |
| I | `Ki · ∫error dt`, **conditional integration** + back-calculated clamp | `Ki · ∫error dt` with `iterm_relax`, iterm rotation, anti-gravity Ki boost, fixed clamp |
| D | `−Kd · d(gyro_filtered)/dt`, **PT1 60 Hz, measured dt** | `−Kd · d(gyro_filtered)/dt · tpaFactor · dMax`, fixed dt |
| F | *(none)* | `Kf · Δsetpoint`, with boost / jitter / averaging shaping |
| S | *(none)* | Setpoint term (fixed-wing only) |
| Output | `clip(P+I+D, ±1.0)` | `clip(P+I+D+F+S, ±pidSumLimit)` then `/1000` |

Both differentiate the **measurement, not the error**, and both carry the same sign
(`−Kd·d(gyro)/dt`). Flapjack's comment on why ([pid.c:113](Firmware/common/pid.c#L113)) describes
exactly the positive-feedback failure Betaflight's structure also avoids. That one is settled and
correct on both sides.

### 2.2 Gains, in the same units

Betaflight's `pidSum` is divided by `PID_MIXER_SCALING` (1000) before mixing
([mixer.c:700](Research/betaflight/src/main/flight/mixer.c#L700)), and its motor mix weights are
±1 like Flapjack's. So `PTERM_SCALE·P/1000` is directly comparable to `CFG_PID_ROLL_P` — both are
"normalised motor command per deg/s of rate error".

| Axis / term | Betaflight default | Flapjack | BF ÷ FJ |
|---|---|---|---|
| Roll P  | 45 → **1.441e-3** | **1.111e-3** | 1.30 |
| Pitch P | 47 → **1.505e-3** | **1.111e-3** | 1.35 |
| Yaw P   | 45 → **1.441e-3** | **1.667e-3** | 0.86 |
| Roll I  | 80 → **1.955e-2** | **1.667e-3** | **11.7** |
| Pitch I | 84 → **2.053e-2** | **1.667e-3** | **12.3** |
| Yaw I   | 80 → **1.955e-2** | **2.778e-4** | **70** |
| Roll D  | 30 → **1.587e-5** | **2.778e-5** | 0.57 |
| Pitch D | 34 → **1.799e-5** | **2.778e-5** | 0.65 |
| Yaw D   | 0 → **0** | **8.3e-7** | — |
| Roll F  | 120 → **1.651e-5** | **none** | — |

*(Scales: `PTERM_SCALE 0.032029`, `ITERM_SCALE 0.244381`, `DTERM_SCALE 0.000529`,
`FEEDFORWARD_SCALE 0.013754` — [pid.h:46](Research/betaflight/src/main/flight/pid.h#L46).
Defaults `{45,80,30,120}` roll, `{47,84,34,125}` pitch, `{45,80,0,120}` yaw.)*

Caveat first: these are different airframes. A 5" quad and a tilt-rotor bicopter have different
inertias, effector authorities and lags, so the ratios are **not** "you are mistuned". What they
show is the *shape* of the tune:

- **P agrees within 35 %.** Two independently-derived tunes landing this close on the dominant
  term is a good sign for Flapjack's SIL sweep.
- **I is an order of magnitude lower.** As an integral time constant `Ti = Kp/Ki`: Flapjack roll
  is **0.67 s**, Betaflight roll is **0.074 s**. Flapjack's integrator is a slow trim; Betaflight's
  is a working part of the loop bandwidth. Yaw is the extreme case at `Ti = 6.0 s` — Flapjack's
  yaw axis has almost no integral action at all.
- **D is 1.5–1.75× higher, and unfiltered.** See §2.4.

### 2.3 Anti-windup — Flapjack wins this one

Flapjack ([pid.c:87-119](Firmware/common/pid.c#L87)) does two things Betaflight does not:

1. **Conditional integration against output saturation.** It computes the provisional output and
   refuses to integrate when the output is already pinned *and* the error would push it further
   out — while always integrating errors that pull it back toward the linear region.
2. **Back-calculation on the I clamp.** When `iTerm` hits `integralLimit`, it writes
   `integral = iTerm/Ki` back into the state, so the raw accumulator cannot run away behind a
   clamped output.

Betaflight does neither. Its I handling is
`pidData[axis].I = constrainf(previousIterm + iTermChange, ±itermLimit)` with
`itermLimit = 0.01 · itermWindup · pidSumLimit = 0.8 · 500 = 400`, i.e. **40 % of motor range**
(Flapjack's `CFG_PID_INTEGRAL_LIMIT` is 0.3 → **30 %**; comparable). It compensates with:

- **`iterm_relax`** ([pid.c:844](Research/betaflight/src/main/flight/pid.c#L844)): a high-passed
  setpoint measure scales the integrated error down during fast stick moves, so I does not wind
  up chasing a transient it was never going to catch. The threshold tightens 5× in angle mode.
- **`zeroThrottleItermReset`** and airmode, which keep the mixer out of hard saturation.

Flapjack's mixer also helps here: `Mixer_MixMotors` **slides** the motor set into the band as a
block rather than clipping each motor ([mixer.c:163](Firmware/tasks/control/mixer.c#L163)),
spending collective thrust (which the altitude loop recovers) instead of differential (which
nothing recovers). Same instinct as Betaflight's airmode, implemented differently and arguably
more cleanly for a two-motor airframe.

**Nothing to change here.** `iterm_relax` is the one piece worth borrowing, and it is low priority.

### 2.4 The filtering gap — the largest single difference

Flapjack has **no signal conditioning at all** between the gyro and the D term.
[devices/imu.c:48](Firmware/devices/imu.c#L48) is explicit:

```c
// TODO: apply low-pass filtering (common/filter.h). Pass-through for now.
pSensor->accelFiltered = pSensor->accel;
pSensor->gyroFiltered  = pSensor->gyro;
```

Betaflight's chain before `Kd` is applied, at defaults:

| Stage | Default |
|---|---|
| Gyro LPF1 (dynamic) | PT1, 250 → 500 Hz, throttle-scheduled |
| Gyro LPF2 (static) | PT1, 500 Hz |
| Dynamic notch | **3 notches**, 100–600 Hz tracking, Q 3.0 |
| RPM filter | per-motor harmonic notches (with DShot telemetry) |
| D-term notch | off by default |
| D-term LPF1 (dynamic) | PT1, 75 → 150 Hz, throttle-scheduled |
| D-term LPF2 (static) | PT1, 150 Hz |

That is **two cascaded low-passes at 75–150 Hz on the D path alone**, on top of everything the
gyro path already did. Flapjack applies a D gain 1.6× larger to the raw signal.

Why this matters more for Flapjack than the ratio suggests: pitch and yaw act through
**rate-limited, lagged tilt servos**. Unfiltered D plus actuator lag is the exact mechanism
behind the limit cycle already documented at [cfg.h:83-97](Firmware/target/cfgs/cfg.h#L83) — the
D loop gain `Kd·G` reached ~2.8 and the vehicle departed on arming. That was solved by cutting D
to 1/10, which buys margin by throwing away damping. A D-term LPF buys the same margin by
removing the noise that forced the cut, and lets some of the D back.

The SIL cannot show you this: JSBSim's gyro is clean. It is a hardware-only failure mode, which
is exactly why it is worth building before hardware rather than after.

**Recommended minimum**: a PT1 on the D path, nothing else. A dynamic notch is not worth it at a
200–400 Hz loop rate — most prop noise is above Nyquist, which is itself an argument for running a
*lower* D gain than a Betaflight-rate loop would.

#### Implemented and measured

`common/filter.c`'s `LowPassFilter_t` was a dead stub — an `Init` that only range-checked an
`alpha`, no state, no `Update`, no callers. It is now a real PT1 parameterised by **cutoff, not
alpha**: a fixed alpha pins the corner to the sample rate (`f_c = α / (2π·dt·(1−α))`), and every
loop here is sensor-paced. The same build has been measured between 257 and 366 Hz in the SIL on
host load alone, which would have moved a fixed-alpha corner by 40 %.

`Pid_UpdateAxis` filters the **measurement** and differences that (Betaflight's ordering; it never
forms the raw difference at all), so `prevMeasurement` now holds the filtered value. P and I still
close on the raw signal — D is the only term whose gain rises with frequency, so it is the only
one worth paying phase for. `CFG_PID_DTERM_LPF_HZ = 0` is exact pass-through. A new
`Pid_ResetAxis` clears integrator, previous sample and filter state together, so the two reset
sites in `control.c` cannot miss a newly added piece of state.

**Measured on a deterministic host replica** of the current `Pid_UpdateAxis` + mixer against the
same JSBSim tiltrotor the gains were swept on, with gyro noise injected at the *reading* (the FDM
state stays clean — which is what sensor noise actually is). 400 Hz, recovering a 10 °/s roll
disturbance:

| gyro noise | D-term RMS (off → 60 Hz) | servo activity (off → 60 Hz) |
|---|---|---|
| 0 (clean) | 0.00029 → 0.00028 | 0 → 0 |
| 0.5 °/s | 0.00940 → **0.00340** (−64 %) | 0.0231 → **0.0094** (−59 %) |
| 3 °/s | 0.05636 → **0.02033** (−64 %) | 0.1384 → **0.0562** (−59 %) |
| 10 °/s (bad prop) | peak rate **25.9 °/s** → **15.3 °/s** against a 10 °/s disturbance | 0.447 → 0.187 |

Cost on a clean gyro, same disturbance: settling 0.120 → 0.120 s at 100 Hz, 0.125 → 0.125 s at
200 Hz, 0.128 → 0.130 s at 400 Hz. No divergence at any rate.

**Why 60 Hz and not lower**, given 30 Hz measures better on noise: settling barely moves anywhere
in 20–150 Hz, because the replica has *no* actuator dynamics and the FDM's are representative
rather than measured (KnownIssues §1.9) — so neither can honestly charge a filter for phase lag,
which is precisely what a low cutoff spends. A PT1 lags `atan(f/fc)`: 4.8° at 5 Hz for 60 Hz, but 9.5° at 30 Hz and 14° at
20 Hz, and real servo lag adds to that. 60 Hz takes most of the available rejection while leaving
the phase budget intact. Going lower needs a model that can price the cost.

**The SIL contributed nothing to the cutoff choice, by construction.**
`Scripts/sim/jsbsim/systems/` models baro and GPS noise but there is no IMU noise model at all, so
the emulated gyro arrives clean and the filter has nothing to remove. SIL runs confirm the C code
integrates and flies (`alt_hold` passes, attitude settles, no NaN) and nothing more. Adding a gyro
noise model is the obvious follow-on — it would move this whole question inside the SIL.

### 2.5 Feedforward — absent

Betaflight computes `Kf · Δsetpoint` per axis (default F=120/125, comparable in magnitude to its
own D gain) with dedicated smoothing, jitter rejection and a boost term. It exists so the craft
follows stick *steps* without waiting for error to build.

Flapjack has none. This is a "feel" feature more than a stability one — a bicopter with a
rate-limited pitch servo will not track a stick step regardless — so it ranks below the filtering
work. Notably, Betaflight **zeroes rate-loop feedforward in angle mode**
([pid.c:1310](Research/betaflight/src/main/flight/pid.c#L1310)) because the angle loop carries its
own feedforward path, so an angle-mode-only vehicle loses less by not having it.

### 2.6 Output limits and gain scheduling

| | Flapjack | Betaflight |
|---|---|---|
| Per-axis clip | ±1.0 (**full** motor range) | ±500/1000 = ±0.5 roll/pitch, ±0.4 yaw |
| TPA | none | `tpa_rate 65`, breakpoint 1350, on **D** by default |
| Low-throttle TPA | none | `tpa_low_rate 20`, breakpoint 1050 |
| D_max | none | 40/46 with gyro- and setpoint-triggered boost |
| Anti-gravity | none | P and I boost on throttle transients |
| Thrust linearisation | none | optional |

Two things stand out. **Betaflight caps each axis at half the motor range**, Flapjack at all of it
— Flapjack's single-axis authority is 2× Betaflight's before the mixer's desaturation logic even
runs. And **Betaflight schedules D down with throttle** (TPA), because prop noise and thrust both
rise with throttle. Flapjack has one fixed gain across the range.

TPA is the second-cheapest borrow after the D LPF: a scalar multiply on D, aimed at the same
high-throttle noise problem.

---

## 3. Angle vs rate

### 3.1 The structure is identical

Both are a **cascade**: outer P on angle error produces a rate setpoint, inner rate PID closes it.
Both leave **yaw as a rate command** in angle mode. Both fall back to rate when attitude is
unavailable (Flapjack explicitly at [guidance.c:115-121](Firmware/tasks/guidance/guidance.c#L115);
Betaflight via `sensors(SENSOR_ACC)` gating at
[core.c:1056](Research/betaflight/src/main/fc/core.c#L1056)).

```
Flapjack (guidance.c)                 Betaflight (pidLevel)
  err  = stick·30° − euler              err  = stick·60° − attitude
  rate = clip(4.0·err, ±180 °/s)        rate = 5.0·err + angleFF
                                        rate += yawSetpoint·sin(otherAxisTarget)·earthRef
                                        rate = pt3(rate, 50 Hz)
```

### 3.2 Gains and limits

| | Flapjack | Betaflight |
|---|---|---|
| Angle gain | roll **4.0**, pitch **2.0** °/s per ° | **5.0** (`PID_LEVEL.P 50 / 10`) both axes |
| Angle limit | **30°** (`CFG_ANGLE_MAX_DEG`) | **60°** (`angle_limit`) |
| Rate clamp on angle output | ±180 °/s hard clip | none explicit; pt3 + rate limits bound it |
| Stick → angle map | **linear**, `stick · 30°` | via the **rate curve**: `angleLimit · setpointRate / maxRcRate` |
| Angle feedforward | none | `angleLimit · FF · angleFeedforwardGain / maxRcRate`, pt3-smoothed |
| Output smoothing | none | **pt3 at 50 Hz** (`ATTITUDE_CUTOFF_HZ`) |
| Earth-referenced yaw | none | `angle_earth_ref 100` (full) |
| Horizon mode | none | crossfade angle↔acro by `horizonLevelStrength` |

The angle gains agree closely (4.0/2.0 vs 5.0) — again a good sign. The pitch/roll asymmetry is
Flapjack-specific and well justified at [cfg.h:126-157](Firmware/target/cfgs/cfg.h#L126): pitch
acts through the tilt servos with ~2.8× less plant gain.

### 3.3 The one thing worth copying: smoothing the angle-loop output

The pt3 smoother at 50 Hz ([pid.c:621](Research/betaflight/src/main/flight/pid.c#L621)) carries a
comment that reads like it was written for Flapjack:

> `smooth final angle rate output to clean up attitude signal steps (500hz), GPS steps (10 or
> 100hz), RC steps etc`

Flapjack's `guidance` task runs at the nav rate (IMU-paced, 200–400 Hz) but its RC input arrives
at **50 Hz** and is read latest-value-cached
([guidance.c:56](Firmware/tasks/guidance/guidance.c#L56)). So the angle target **steps** every
~4–8 frames while the rate setpoint is recomputed every frame. `CFG_ANGLE_ROLL_P` is 4.0, so a
single CRSF quantisation step is negligible — but a transmitter moving quickly produces steps of
several degrees, i.e. tens of °/s of instantaneous rate-setpoint jump, into a rate loop with **no
feedforward and no D filtering**. That step reaches the D term as `Δgyro` a frame later.

Earth-referenced yaw matters less for a bicopter than a quad, but the cross-axis coupling it
compensates (yaw input tilting the roll/pitch reference while banked) is real on any airframe.
Low priority.

### 3.4 One asymmetry worth noting

Betaflight's stick→angle map goes **through the rate curve** (`applyRates` → `setpointRate`, then
scaled by `angleLimit/maxRcRate`), so the pilot's expo and rate feel carry into angle mode.
Flapjack maps the stick linearly. Neither is wrong: Betaflight's means one set of curve settings
governs both modes; Flapjack's means angle mode is predictable independent of any rate curve
(which it does not have anyway).

---

## 4. Altitude estimation

| | Flapjack `AltitudeFilter` | Betaflight `positionKalman_t` |
|---|---|---|
| Form | 3rd-order complementary, fixed gains | **2-state Kalman**, `[pos, vel]`, live covariance |
| States | `alt`, `vz`, `accelBias` | `x[0]=pos`, `x[1]=vel` (+ separate offset calibration) |
| Predict | `alt += vz·dt + ½a·dt²`, IMU-paced | same kinematics, **fixed 100 Hz** |
| Correct | `alt += kAlt·e·dt` etc., baro-paced, gated on a *new* sample | `kalmanUpdatePosition(measured, R)`, Joseph-form covariance |
| Accel bias | explicit state, `kBias`, clamped ±2 m/s² | not a state; handled by `crossCalibrateOffsets` on sensor offsets |
| Sensors | baro only (vertical); GPS is horizontal-only and unfiltered | baro + GPS alt + rangefinder + optical flow, each with its own `R` |
| Tuning | T = 1 s → `kAlt 3, kVel 3, kBias 1` | `Q_ACCEL_Z 20000`, `R_BARO_ALT 1500` (cm² units) |
| Validity | `NAV_VALID_BARO_ALT` flag | `isValidZ` (2 s measurement timeout) + `trustZ` from covariance |
| Datum | averaged over 50 samples **at boot** | first baro sample, re-reset **on arm and disarm** |
| Output for control | `alt`, `vz` raw | **raw KF** for control, pt2-filtered separately for OSD/vario |

Flapjack's filter is the correct textbook structure for what it has, and `AltitudeFilter_Correct`
correctly identifies the gain-multiplication trap that comes from re-applying a stale baro sample
— Betaflight sidesteps that by running the whole estimator at a fixed 100 Hz and letting the baro
task set `R`.

Three differences worth acting on:

1. **Betaflight resets the estimator on arm/disarm**
   ([position.c:115](Research/betaflight/src/main/flight/position.c#L115)). Flapjack fixes the
   datum at boot and never re-takes it, with a stated reason: nav would have to subscribe to
   `umsg_mission_state_t` to see arming, closing a layer cycle. That reasoning is sound, but the
   *consequence* — a long pre-arm sit lets baro drift accumulate into the datum — is unaddressed.
   A boot-time datum with a slow re-zero while disarmed and stationary gets most of the benefit
   without the dependency.
2. **Betaflight separates the control signal from the display signal.** `getAltitudeCmControl()`
   returns the raw KF output; `getAltitudeCm()` returns a pt2-filtered version for OSD and vario.
   Flapjack publishes one `alt` for both, so any smoothing added for telemetry would land in the
   control path.
3. **`trustZ`** is a genuinely useful idea with no Flapjack analogue: a continuous 0–1 confidence
   derived from the covariance rather than a binary flag. `NAV_VALID_BARO_ALT` goes true 50 baro
   samples after boot and never goes false again.

---

## 5. Altitude hold

### 5.1 Structure — the same skeleton

|  | Flapjack | Betaflight |
|---|---|---|
| Stick meaning | **climb rate**, walks `altTarget` | **climb rate**, walks `targetAltitudeCm` |
| Deadband | ±0.05 normalised, about stick centre | `altHoldConfig()->deadband`, **asymmetric about hover PWM** |
| Climb rate at full stick | `CFG_ALT_HOLD_CLIMB_RATE_MPS` = **1.0 m/s** | `climbRate · 10` = **5 m/s** default |
| Target capture on entry | yes, `altTarget = alt` | yes, `altHoldReset()` |
| Hover feedforward | `CFG_HOVER_THROTTLE` = **0.50** | `hoverThrottle` 1275 PWM ≈ **0.275**, or captured from the stick on entry |
| Integrator reset | on entry and on arm | on entry and on exit |
| Target runaway guard | **none** | **yes** — target only advances if within `maxClimbRate · 1 s` of current altitude |
| Requires angle mode | no (independent) | **yes** — `ALT_HOLD_MODE` force-enables `ANGLE_MODE` |
| Validity gate | `NAV_VALID_BARO_ALT`, **re-checked in control** | `isAltitudeAvailable()` + `wasThrottleRaised()` at mode entry |

Flapjack re-checking `NAV_VALID_BARO_ALT` in `control` rather than trusting guidance
([control.c:161](Firmware/tasks/control/control.c#L161)) is the right call, for the right stated
reason. Betaflight's equivalent guard exists only at mode-enable time.

Betaflight's **target runaway guard** is worth taking outright, and it is three lines
([alt_hold_multirotor.c:145](Research/betaflight/src/main/flight/alt_hold_multirotor.c#L145)):

```c
if (fabsf(getAltitudeCmControl() - altHold.targetAltitudeCm) < maxClimbRate * 1.0f /* s */) {
    altHold.targetAltitudeCm += altHold.targetVelocity * taskIntervalSeconds;
}
```

Flapjack walks `altTarget` unconditionally at
[guidance.c:149](Firmware/tasks/guidance/guidance.c#L149). If the vehicle cannot climb — thrust
saturated, prop damage, the mixer sliding collective away to preserve differential — the target
runs away from the vehicle at 1 m/s for as long as the stick is held, and every metre of that is
error the loop will try to spend on stick release.

### 5.2 Gains, normalised

Betaflight's `throttleOffset` is in PWM units on a ~1000-unit full-scale throttle span, so
`PWM/1000 ≈` normalised throttle. Flapjack's units are normalised throttle directly.

| | Betaflight | Flapjack | BF ÷ FJ |
|---|---|---|---|
| P | 30 × 0.005 = 0.15 /cm → **0.015 /m** | **0.50 /m** | 0.030 |
| I | 30 × 0.002 = 0.06 → **0.006 /(m·s)** | **0.20 /(m·s)** | 0.030 |
| I limit | 150 PWM → **0.15** | **0.30** | 0.5 |
| D | 30 × 0.01 = 0.3 /(cm/s) → **0.030 /(m/s)** | **0.50 /(m/s)** (`VZ_DAMPING`) | 0.060 |
| F | 30 × (0.1/30) = **0.010 /(m/s)** | **none** | — |
| Hover | **0.275** | **0.50** | — |

The hover throttles differ, and that accounts for some of the gap: with thrust ≈ k·throttle², the
normalised plant gain near hover is `2/hover`, so Betaflight's quad is 0.5/0.275 = **1.8×** more
responsive per unit of throttle. That leaves Flapjack's altitude loop at roughly **18× the loop
gain** of Betaflight's default.

Two consequences worth stating plainly:

- **Flapjack's altitude P term saturates at 1.0 m of error.** `trim` is clipped to ±1.0 and hover
  is 0.5, so `0.5 · error` reaches full *throttle* (`0.5 + 0.5`) at 1 m and the PID's own clip at
  2 m. Betaflight's P does not saturate until ~41 m. Beyond 1 m Flapjack's altitude hold is
  effectively bang-bang — which the anti-windup handles correctly, but it means the linear tuning
  only describes behaviour inside ±1 m.
- The tune was swept against a **noise-free SIL baro**, which cfg.h says outright
  ([cfg.h:55](Firmware/target/cfgs/cfg.h#L55)). A real BMP390 in prop wash will not support it.
  Betaflight's defaults are what survived contact with real barometers on a very large number of
  airframes. Expect to divide by something between 3 and 10 on hardware.

### 5.3 The damping term — the real finding

This is the one structural difference that changes behaviour today, in the SIL, with no hardware
involved.

**Betaflight** ([autopilot_multirotor.c:307-320](Research/betaflight/src/main/flight/autopilot_multirotor.c#L307)):

```c
float velocityError = targetVerticalVelocity - verticalVelocity;   // TARGET minus actual
const float altitudeD = velocityError * altitudeKd * dBoost;
const float altitudeF = targetVerticalVelocity * altitudeKf;
```

**Flapjack** ([control.c:180](Firmware/tasks/control/control.c#L180)):

```c
float trim = Pid_UpdateAxis(&pPid->axes[AXIS_IDX_THROTTLE], alt, sp.vel_b[2], dt);
trim -= CFG_ALT_HOLD_VZ_DAMPING * vz;      // pure rate damping, no target
```

Betaflight's is a proper cascade: the outer loop produces a **target velocity**, and the inner
term drives velocity *error* to zero. Flapjack's is a pure rate damper — it opposes **all**
vertical motion, including the motion the pilot just asked for.

Working the numbers for a commanded 1 m/s climb (full stick, `CFG_ALT_HOLD_CLIMB_RATE_MPS = 1.0`):

- A steady climb needs roughly hover thrust, so the *net* trim about `CFG_HOVER_THROTTLE` should
  settle near zero.
- But steady climb ⇒ `vz = 1.0` ⇒ the damping term contributes **−0.50** of normalised throttle,
  so the PID must supply **+0.50** just to cancel it: `0.5·err + I = 0.50`.
- With `I = 0` at climb entry that is **`err = 1.0 m` of lag**. `I` then winds up at
  `0.2 · err` per second until it hits its **0.30** clamp, which settles the lag at
  **`err = 0.4 m`** — with the integrator pinned at its limit for the rest of the climb.

Then the pilot centres the stick. The target stops moving, the vehicle is 0.4 m low, `vz` decays,
the damping term releases its −0.50 — and the integrator is still sitting at +0.30 with nothing
to unwind it except the resulting overshoot. That is a textbook climb-then-balloon.

Betaflight's `dBoost`
([autopilot_multirotor.c:314](Research/betaflight/src/main/flight/autopilot_multirotor.c#L314)) is
a second refinement: D is *increased* above 5 m/s (2× at 10 m/s), so fast descents get extra
braking without over-damping the near-hover response.

**The fix is small and local.** Guidance already computes the commanded climb rate; publish it
alongside the target altitude and use it:

```c
trim -= CFG_ALT_HOLD_VZ_DAMPING * (vz - sp.climb_rate);
```

One new field on `umsg_guidance_setpoints_t`, one line in guidance, one line in control. It
removes the 0.4 m lag, unpins the integrator, and delivers Betaflight's `altitudeF` climb-rate
feedforward essentially for free. It does not disturb the hover case at all — with
`climb_rate = 0` the expression is identical to today's.

#### Implemented and measured

Shipped as `climb_rate` on `umsg_guidance_setpoints_t` — published by
[guidance.c](Firmware/tasks/guidance/guidance.c) as exactly `d(vel_b[2])/dt`, consumed by
[control.c](Firmware/tasks/control/control.c). The invariant is the load-bearing part and is
written down in [Firmware/CLAUDE.md](Firmware/CLAUDE.md): a `climb_rate` the target is not really
moving at becomes a feedforward the loop will chase, so it is computed inside the same `armed`
test that gates the target integration, and forced to zero in the throttle-passthrough fallback.

Measured with `Scripts/sim/plans/alt_hold.yaml` in the Renode SIL (400 Hz loop, noise on;
the loop is 500 Hz now — see KnownIssues 1.17, the gains did not need retuning), against the
target trajectory reconstructed from the plan. Two runs after the change, to separate the effect
from SIL run-to-run variance:

| | baseline (`-k·vz`) | fix run 1 | fix run 2 |
|---|---|---|---|
| Climb lag, τ 7–11 s (truth − target) | **−0.524 m** | −0.037 m | **+0.012 m** |
| Balloon after stick centres @ 11 s | **+0.342 m** | +0.091 m | **+0.038 m** |
| Balloon after stick centres @ 23 s | **+0.336 m** | +0.040 m | +0.129 m |
| Time to lift off | 6.86 s | 6.08 s | 5.96 s |
| Peak motor command | 0.589 | 0.791 | 0.807 |

The climb lag is gone (−0.52 m → ~0) and the balloon is down by 73–89 %. The higher peak motor
command is the expected consequence, not a regression: the damping term is no longer eating half
of the throttle authority during a commanded climb, so the loop can actually spend it. Nothing is
near the 1.000 pin the plan checks for.

Two residuals worth recording, neither introduced by this change:

- **A standing +0.28…+0.38 m offset above target persists in every phase, in all three runs.** That
  is the nav-versus-truth altitude bias — the bridge reports `nav alt max err ≈ 0.5 m` identically
  before and after. The loop holds *its own* altitude estimate on target; truth sits above by the
  estimator's bias. The change's real effect is that the tracking error is now a roughly constant
  offset instead of swinging from −0.52 m (climbing) to +0.38 m (holding).
- **Climb 2 (τ 20–23 s, 0.6 m/s) looked *better* in the baseline** (−0.04 m vs +0.30 m). That was
  coincidence, not merit: at 0.6 m/s the old damping term was −0.30, exactly cancelling the +0.3 m
  standing offset. At 1.0 m/s it was −0.50, which the integrator could not cover — `CFG_PID_INTEGRAL_LIMIT`
  is 0.30 — leaving the rest to P as the 0.52 m lag. The 0.6 m/s case never exposed the bug.

### 5.4 Tilt compensation — missing

Betaflight ([autopilot_multirotor.c:329](Research/betaflight/src/main/flight/autopilot_multirotor.c#L329)):

```c
const float tiltMultiplier = 1.0f / fmaxf(getCosTiltAngle(), 0.5f);
throttleOffset *= tiltMultiplier;
```

Flapjack has nothing equivalent. [cfg.h:160](Firmware/target/cfgs/cfg.h#L160) states the problem
itself: *"Lift scales with cos(angle), so 30 deg costs 13% of vertical thrust."* At the angle
limit that is a 13 % thrust deficit the integrator has to discover and cancel — and it has only
0.30 of authority to begin with, most of which the climb case (§5.3) is already consuming.

This is **not** the same as the mixer's existing `pMix[i].pitch * ABS_F32(pidPitch)` collective
term ([mixer.c:126](Firmware/tasks/control/mixer.c#L126)), which compensates lift lost to **rotor
tilt** relative to the airframe. Bank of the whole airframe is uncompensated. `nav` already
publishes the attitude quaternion, and the bottom-row element `Nav_VerticalAccelUp` already uses
is exactly the cosine wanted: `cosTilt = 1 − 2(q2² + q3²)`. One line.

### 5.5 Deadband shape

Small but real: Betaflight's throttle deadband is **asymmetric about the hover PWM**, not about
stick centre ([alt_hold_multirotor.c:110-111](Research/betaflight/src/main/flight/alt_hold_multirotor.c#L110)),
so the "hold" position of the stick is where hover actually is. Flapjack's is symmetric about
1500 µs and assumes hover sits at stick centre. That is currently true (`CFG_HOVER_THROTTLE` is
0.50 and 1500 µs maps to 0.50), so it is harmless today — but it silently couples the deadband to
the hover thrust, and if the airframe's hover throttle ever changes, the deadband lands in the
wrong place.

### 5.6 Incidental: two stale comments about the motor cap

While reading the altitude-hold config: [cfg.h:192](Firmware/target/cfgs/cfg.h#L192) and
[cfg.h:217](Firmware/target/cfgs/cfg.h#L217) both warn that `CFG_MOTOR_MAX_THROTTLE` **(0.40)**
sits below the ~0.50 hover throttle, so "hardware cannot hover until that driver cap is raised".
The define is now **1.00F** ([cfg.h:187](Firmware/target/cfgs/cfg.h#L187)). The cap was raised;
the two comments were not. Flagging, not changing (Rule 3).

---

## 6. Motor and servo mixing

Betaflight ships a **bicopter mixer**, so for once this is a like-for-like comparison rather than
a quad-versus-tiltrotor analogy.

### 6.1 Structure

|  | Flapjack | Betaflight |
|---|---|---|
| Entry point | `Mixer_MixMotors` + `Mixer_MixServos`, back to back in `Control_Update` | `mixTable()` then `writeServos()` in `subTaskMotorUpdate` |
| Motor mix input | `pidData[4]`, each already clipped to ±1 by `Pid_UpdateAxis` | `pidData[].Sum` clipped to `pidSumLimit`, then `/1000` |
| Servo mix input | the same `pidData[4]` | `pidData[].Sum × 0.7` (`PID_SERVO_MIXER_SCALING`), **not** `pidSumLimit`-clipped |
| Motor mix struct | `MotorMix_t { throttle, roll, pitch, yaw }` | `motorMixer_t { throttle, roll, pitch, yaw }` — **identical** |
| Servo mix struct | `ServoMix_t { targetServo, inputIndex, weight }` | `servoMixer_t { targetChannel, inputSource, rate, speed, min, max, box }` |
| Profile selection | `Mixer_Init_(eMIXER_PROFILE_TILT_ROTOR, …)`, with a `TODO` to read it from cfg | runtime `getMixerMode()`, 20+ built-ins plus custom |
| Desaturation strategies | one | four (`LEGACY`, `LINEAR`, `DYNAMIC`, `EZLANDING`) |

Betaflight's servo path reading **raw, unclipped** `pidData[].Sum` is worth noting as the one
place Flapjack is straightforwardly safer: `Pid_UpdateAxis` clips to ±1 before anything sees it,
so there is no path by which a runaway PID sum reaches a servo un-bounded.

### 6.2 The bicopter tables — motors identical, servos disagree

**The motor mix is the same table, field for field.**

Flapjack [mixer.c:38-41](Firmware/tasks/control/mixer.c#L38):

```c
{ .throttle = 1.0F, .roll =  1.0F, .pitch = 0.0F, .yaw = 0.0F }, // Left
{ .throttle = 1.0F, .roll = -1.0F, .pitch = 0.0F, .yaw = 0.0F }, // Right
```

Betaflight [mixer_init.c:105-108](Research/betaflight/src/main/flight/mixer_init.c#L105) — struct
order is `{throttle, roll, pitch, yaw}`:

```c
{ 1.0f,  1.0f,  0.0f,  0.0f },          // LEFT
{ 1.0f, -1.0f,  0.0f,  0.0f },          // RIGHT
```

**The servo mixes do not agree — pitch and yaw are swapped.**

Betaflight [servos.c:143-148](Research/betaflight/src/main/flight/servos.c#L143):

| Servo | Yaw | Pitch |
|---|---|---|
| `SERVO_BICOPTER_LEFT` | **+100** | **−100** |
| `SERVO_BICOPTER_RIGHT` | **+100** | **+100** |

→ yaw is **collective** tilt (same sign both sides), pitch is **differential** tilt.

Flapjack [mixer.c:43-49](Firmware/tasks/control/mixer.c#L43):

| Servo | Yaw | Pitch |
|---|---|---|
| `eSERVO_LEFT_MOTOR_ID` | **+1.0** | **−1.0** |
| `eSERVO_RIGHT_MOTOR_ID` | **−1.0** | **−1.0** |

→ pitch is **collective** tilt, yaw is **differential** tilt.

Both firmwares agree the rotors are lateral (the motor mix makes roll differential *thrust* in
both). For lateral rotors on fore/aft-tilting servos:

- tilt **both** forward → the thrust vector tilts forward, and with the rotors above the CG that
  is a nose-down moment → **pitch**;
- tilt **left forward, right aft** → a couple about the vertical axis → **yaw**.

Flapjack's table is the one that matches that, and it matches its own documented geometry at
[mixer.c:24-36](Firmware/tasks/control/mixer.c#L24). Betaflight's is inverted on both axes.

Before treating that as a Betaflight bug worth reporting: `MIXER_BICOPTER` sits behind
`USE_UNCOMMON_MIXERS`, is inherited essentially unchanged from MultiWii, and is not part of the
maintained or flight-tested path — Betaflight is a quad firmware. The practical conclusion is
narrower and more useful: **Betaflight is not a cross-check for the bicopter servo mix.** It
disagrees, and Flapjack's is the version with the physics written down next to it.

### 6.3 Motor desaturation — the same algorithm, arrived at independently

This is the closest correspondence in either codebase. Flapjack's `Mixer_MixMotors` and
Betaflight's default `applyMixerAdjustment` implement the same two steps in the same order:

**Step 1 — scale the differential down if its range exceeds the band**, so every motor loses
authority by the same factor and the response stays symmetric:

```c
/* Flapjack */                          /* Betaflight (LEGACY) */
float band  = max - min;                motorMixNormalizationFactor =
float range = diffMax - diffMin;            motorMixRange > 1.0f
if (range > band) scale = band / range;     ? airmodeTransitionPercent / motorMixRange
                                            : airmodeTransitionPercent;
```

**Step 2 — slide the whole set with a common offset instead of clipping motors individually:**

```c
/* Flapjack: compute an offset, floor wins */
if (mixedMax > max)            offset = max - mixedMax;
if (mixedMin + offset < min)   offset = min - mixedMin;

/* Betaflight: constrain throttle, floor wins */
throttle = constrainf(throttle, -normalizedMotorMixMin, 1.0f - normalizedMotorMixMax);
```

Both give the **floor priority** when the two bounds conflict — Flapjack because the min test is
applied second and overrides, Betaflight because `constrainf` tests `< low` first and so returns
the low bound when the interval is empty. Flapjack's comment at
[mixer.c:109-113](Firmware/tasks/control/mixer.c#L109) ("*clipping each motor independently
spends the difference first… the roll response halves and goes asymmetric exactly where it is
needed most*") is the same argument that produced Betaflight's airmode.

Three real differences:

1. **What lands in the "differential" bucket.** Betaflight puts roll + pitch + yaw into
   `motorMix` and adds throttle afterwards — on a quad all three axes are motor effectors.
   Flapjack puts roll + yaw into `differential` and throttle + `|pitch|` into `collective`,
   because on a bicopter pitch is a *servo* effector and its motor term is lift compensation, not
   a moment. That split is airframe-correct and has no Betaflight analogue.
2. **Airmode.** Betaflight halves mix authority below half throttle when airmode is off
   (`scaleAirmodeTransition`). Flapjack always runs full authority with a 5 % idle floor whenever
   armed — effectively "airmode permanently on". That is the right call for a bicopter, where
   differential thrust is the only roll effector, and it matches what Betaflight does with airmode
   enabled, which is what anyone flying acro runs anyway.
3. **Strategy count.** Betaflight's `LINEAR`/`DYNAMIC` variants trade throttle range for mix
   authority differently, and `EZLANDING` deliberately gives up authority near the ground.
   Flapjack has one strategy, which is the correct amount for one airframe.

### 6.4 Servo saturation — the motor fix has not been applied to the servos

Flapjack's servo mixer clamps each servo's **summed** travel independently
([mixer.c:217](Firmware/tasks/control/mixer.c#L217)):

```c
float travel = clipf32 (mixedInputs[i], -pMixer->servoTravelLimit, pMixer->servoTravelLimit);
```

That is exactly the per-actuator clipping that §6.3 was written to avoid on the motors — and it
has the same consequence. With `left = −pitch + yaw` and `right = −pitch − yaw`, clipping the two
servos independently does not attenuate pitch and yaw proportionally; it **preserves whichever
axis is larger and annihilates the other.** Worked against the actual limit of 0.3333:

| Commanded | Left | Right | Pitch delivered | Yaw delivered |
|---|---|---|---|---|
| pitch 0.2, yaw 0.5 | +0.300 | −0.333 | 0.017 (**−92 %**) | 0.317 (−37 %) |
| pitch 0.3, yaw 0.3 | 0.000 | −0.333 | 0.167 (−44 %) | 0.167 (−44 %) |
| pitch 0.5, yaw 0.2 | −0.300 | −0.333 | 0.317 (−37 %) | 0.017 (**−92 %**) |

(Decomposed back out as `pitch = −(L+R)/2`, `yaw = (L−R)/2`.)

Those three rows are **illustrative of the mechanism at deep saturation, not measured** — see the
measured section below, which is a good deal less dramatic. The direction of the effect is the
robust part: whichever axis is asking for less is the one that disappears, and pitch is already
this airframe's slow, low-authority axis (§2.2, §3.2).

#### Implemented and measured

Shipped in [mixer.c](Firmware/tasks/control/mixer.c) as a **single scale factor across the servo
set** rather than a per-servo clip:

```c
float scale = (maxAbs > limit) ? limit / maxAbs : 1.0f;   /* then travel = mix[i] * scale */
```

Uniform scaling attenuates every axis by the same fraction, so the commanded direction in the
(pitch, yaw) plane survives. There is deliberately **no counterpart to the motor mixer's second
step**, the common offset: the motor band is asymmetric about its operating point so sliding
within it spends collective thrust, whereas this limit is symmetric about centre, where the same
offset would just be an uncommanded collective tilt — a pitch command, not a desaturation. The
`clipf32` after the scale is kept as the **NaN guard**, not as the limiter (KnownIssues §1.14, §2.25).

**Measured, and it corrects the severity claim above.** The pre-limit servo pair was logged from
the current (clipping) build on every saturating frame, flying a plan built for this — full pitch
plus 0.8 yaw, held simultaneously, which nothing in `Scripts/sim/plans` does. Both policies were
then evaluated offline on those real commands:

| | pitch retained | yaw retained | disparity |
|---|---|---|---|
| per-servo clip (before) | mean 0.866, min **0.814** | mean 0.977, min 0.967 | mean 0.111, max **0.153** |
| uniform scale (after) | mean 0.960, min 0.944 | mean 0.960, min 0.944 | **0** by construction |

The mechanism is confirmed — the subordinate axis is attenuated ~5× harder than the dominant one,
and the fix makes the two equal — but **the magnitude is far smaller than the worked table
suggests, and saturation is rare**: 7 saturating frames in a 30 s aggressive manoeuvre (~0.06 % of
control frames; the loop ran at 400 Hz then, 500 Hz now), never more than **5.6 %** past the limit. So the real numbers are
"pitch loses 19 % where yaw loses 3 %", not "pitch loses 92 %".

Two claims in the original write-up do not survive this:

- **"the limit does bind in normal flight"** leaned on
  [cfg.h:258-271](Firmware/target/cfgs/cfg.h#L258), which records the `hover` plan peaking at
  0.760 rad. That figure was measured *unlimited* and on an earlier tune; on the current tune the
  limit is reached only on deliberately combined full-scale inputs.
- **Ranking this second.** It is correct, cheap and now done, but on severity it sits below the
  D-term filter (§2.4). It matters more as the tune gets more aggressive, or if
  `CFG_MIXER_SERVO_TRAVEL_LIMIT` is lowered — both of which push saturation deeper, where the
  disparity grows toward the illustrative table.

Betaflight's equivalent lever is per-rule `min`/`max` (percent of servo travel), which lets each
axis *reserve* a share rather than race for it — a strictly richer policy than one global factor,
and the thing to reach for if pitch ever needs a guaranteed floor.

### 6.5 Servo output path

|  | Flapjack | Betaflight |
|---|---|---|
| Per-rule gain | single `weight` | `rate` (±125 %) |
| Per-rule travel limits | none | `min` / `max`, % of servo travel |
| Per-rule slew limit | none | `speed` |
| Conditional rules | none | `box` (bind a rule to an aux switch) |
| Total travel clamp | `CFG_MIXER_SERVO_TRAVEL_LIMIT` 0.3333 → ±30° | per-servo `min`/`max`, default 1000–2000 µs |
| Pulse conversion | `1500 + travel × 1000` µs, clamped 500–2500 | `middle + rate% × servo`, clamped to min/max |
| Centre | fixed `SERVO_CENTER_US_DC` 1500 µs | per-servo `middle`, or forwarded from an RC channel |
| Output filter | none | optional `servo_lowpass_freq` (off by default) |
| Reversal | sign in the mix weight | `reversedSources` bitfield per servo |

The one entry worth acting on is **`speed`**, Betaflight's per-rule slew limit. Both
[cfg.h:104-107](Firmware/target/cfgs/cfg.h#L104) and KnownIssues §1.9 note that the FDM's servo
dynamics are representative rather than measured, and that real servo lag will eat phase margin. A slew limit does not model that lag
— it *bounds the command* so the mixer cannot ask for more than the servo can deliver. That
matters for the anti-windup specifically: `Pid_UpdateAxis` keys its conditional integration on its
own **output clip**, so a PID whose output is well inside ±1 while the servo physically cannot
follow will keep integrating against an actuator that is not tracking. A slew limit in the mixer
does not fix that by itself, but it is the cheapest way to make commanded and achievable travel
agree.

### 6.6 Motor endpoints and the disarmed state

|  | Flapjack | Betaflight |
|---|---|---|
| Mixer band | `CFG_MIXER_IDLE_THROTTLE` 0.05 → `CFG_MIXER_MAX_THROTTLE` 1.00 | `motorOutputLow`…`motorOutputHigh`, normalised |
| Driver clamp | `dshot.c` re-clips to `CFG_MOTOR_MIN_THROTTLE` 0.05 … `CFG_MOTOR_MAX_THROTTLE` 1.00 | `constrainf(motorOutput, motorRangeMin, motorRangeMax)` |
| Idle | fixed 5 % whenever armed | `dshot_idle_value`, plus optional **dynamic idle** (RPM-closed-loop PID that raises the floor) |
| Runtime endpoint movement | none | 3D mode, dyn idle, `motor_output_limit`, **vbat sag compensation** all move the endpoints live |
| Thrust linearisation | none | optional |
| Motors when disarmed | `Motors_Disarm()` on the edge; nothing written after | `motor[i] = motor_disarmed[i]` every frame |
| Servos when disarmed | **never written** | written every frame; tricopter has `tri_unarmed_servo` to opt out |

Two observations:

- **Flapjack's two motor clamps are currently identical** (0.05–1.00 in both the mixer band and
  the DShot driver), so the driver clamp is a no-op. Betaflight's equivalent double-clamp earns
  its keep because `motorRangeMin`/`Max` genuinely move at runtime. Harmless as it stands, but the
  duplication is what let the stale-comment drift in §5.6 go unnoticed.
- **`Control_Update` returns before `Servos_Write` when disarmed**
  ([control.c:190-194](Firmware/tasks/control/control.c#L190)), and `Servos_Write` has no other
  caller. So the mix is computed and thrown away, and the tilt servos hold whatever the last armed
  frame commanded — potentially rotors left tilted well off vertical on the ground. Betaflight
  writes servos unconditionally and makes "hold the servo still while disarmed" an explicit
  per-airframe option rather than a side effect of the early return. Whichever behaviour is
  wanted here, it is currently implicit.

---

## 7. Scorecard

| Item | Flapjack | Betaflight | Verdict |
|---|---|---|---|
| D on measurement, correct sign | ✅ | ✅ | Match |
| Output-saturation anti-windup | ✅ | ❌ | **Flapjack better** |
| Integral back-calculation on clamp | ✅ | ❌ | **Flapjack better** |
| NaN / zero-`dt` hardening | ✅ | n/a (fixed dt) | **Flapjack better** |
| **Motor** desaturation: scale then slide | ✅ | ✅ (airmode) | Match — same algorithm, same tie-break |
| Bicopter motor mix table | ✅ | ✅ | **Identical, field for field** |
| Servo mix bounded before the driver | ✅ (PID clip ±1) | ⚠️ raw `pidSum × 0.7` | **Flapjack better** |
| Angle→rate cascade, yaw stays rate | ✅ | ✅ | Match |
| Fallback to rate on lost attitude | ✅ | ✅ | Match |
| Hover-throttle feedforward | ✅ | ✅ | Match |
| Climb-rate stick with target capture | ✅ | ✅ | Match |
| Re-validating altitude in the control task | ✅ | ⚠️ mode entry only | **Flapjack better** |
| D-term filtering | ✅ (PT1, 60 Hz) | ✅✅ (2 LPFs) | **Fixed** — §2.4 |
| Gyro-path filtering (P, I, estimator) | ❌ | ✅✅✅ (2 LPFs + 3 notches) | Remaining gap |
| Altitude damping on velocity *error* | ✅ | ✅ | **Fixed** — §5.3 |
| **Servo** desaturation: proportional, not per-actuator | ✅ | ✅ (per-rule min/max) | **Fixed** — §6.4 |
| Rate / altitude feedforward | ❌ | ✅ | Gap |
| Tilt compensation in alt hold | ❌ | ✅ | Gap |
| Target-runaway guard | ❌ | ✅ | Gap (3 lines) |
| TPA / gain scheduling | ❌ | ✅ | Gap |
| Servo slew limit | ❌ | ✅ (`speed`) | Gap; matters with real tilt servos |
| Explicit disarmed servo behaviour | ❌ implicit | ✅ | Gap |
| I-term relax | ❌ | ✅ | Minor |
| Angle-rate output smoothing | ❌ | ✅ | Minor; matters at 50 Hz RC |
| Estimator confidence, not just a flag | ❌ | ✅ (`trustZ`) | Minor |
| Separate control vs display altitude | ❌ | ✅ | Minor |
| Runtime motor endpoints (sag comp, dyn idle) | ❌ | ✅ | Out of scope |
| Earth-referenced yaw, horizon, acro trainer | ❌ | ✅ | Out of scope |
| Bicopter **servo** mix sign convention | ✅ matches its geometry | ⚠️ inverted, unmaintained path | Not a usable cross-check |

## 8. Ranked recommendations

1. ~~**Fix the altitude damping term to use velocity error**~~ — **DONE** (§5.3). Shipped as
   `climb_rate` on `umsg_guidance_setpoints_t`; SIL-measured climb lag −0.52 m → ~0, balloon
   +0.34 m → +0.04…+0.09 m.
2. ~~**Apply the motor mixer's desaturation logic to the servos**~~ — **DONE** (§6.4). One scale
   factor across the servo set instead of a per-servo clip. The effect is real but modest: in
   flight, clipping kept 81–87 % of the pitch command against 97–98 % of the yaw, where scaling
   keeps 94–96 % of both. **Severity was overstated in the original write-up** — saturation is
   rare (7 frames in a 30 s aggressive manoeuvre) and shallow (≤5.6 % past the limit), so on
   severity alone this belonged below the D-term filter rather than above it.
3. ~~**Add a PT1 low-pass on the D path**~~ — **DONE** (§2.4), at 60 Hz. Measured on a host replica
   with injected gyro noise: −64 % D-term noise energy, −59 % servo activity, for 2 ms of
   settling. The SIL could not contribute — it has no IMU noise model.
4. **Add the target-runaway guard** to `Guidance_Update` (§5.1). Three lines, copied directly.
5. **Add tilt compensation** to the altitude loop (§5.4). One line off the existing quaternion.
6. **Back the altitude gains off before hardware** (§5.2). ~18× Betaflight's loop gain, tuned
   against a noise-free baro.
7. **Decide the disarmed servo behaviour explicitly** (§6.6). Right now the tilt servos are never
   written while disarmed, as a side effect of an early return.
8. **Reconsider the rate I gains** (§2.2). `Ti = 0.67 s` on roll/pitch and `6.0 s` on yaw is very
   slow trim against Betaflight's 0.074 s. Worth a SIL sweep — the anti-windup is already good
   enough to support more integral action than it is currently being asked to provide.
9. **pt3-smooth the angle-loop rate output** (§3.3). Matters because RC arrives at 50 Hz into a
   loop running 4–8× faster.
10. Optional, later: a servo slew limit (§6.5), TPA on D, rate feedforward, `iterm_relax`, a
    `trustZ`-style confidence signal.
