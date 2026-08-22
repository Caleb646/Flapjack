# Flapjack — Problems Found Building the Renode SIL

Everything uncovered while standing up software-in-the-loop emulation (Renode CM7 + JSBSim
bridge). How the rig itself works is [SilResearch.md](SilResearch.md); control-law analysis is
[ControlResearch.md](ControlResearch.md). This is the defect index.

**The distinction that matters:** most of these are **pre-existing firmware faults that affect
real hardware**. The emulator did not cause them, it made them observable. They are marked
**HW** below. Emulation-only issues are marked **SIL**.

**Section numbers here are load-bearing** — source comments in `bridge.py`, `mixer.c`, `main.c`,
`align.h`, `core_shared.c`, `gps_task.c`, `serial_link.c` and `tiltrotor.xml` cite them. Never
renumber; retire an entry to a one-line redirect instead.

---

## 1. Open — firmware, affects real hardware

### 1.9 The rate tune is provisional — estimated geometry, unmeasured actuators **HW**

The loop flies closed-loop in the SIL (§2.15) and the two defects this entry used to carry are
closed — output-saturation anti-windup landed in §2.26, and the D term now has a 60 Hz PT1
(ControlResearch §2.4). What stops the gains being final is that they are fitted to numbers
nobody has measured:

- **Pitch is the slow axis** (1.47 s to settle a 20 °/s disturbance, against 0.67 s roll and
  0.12 s yaw) because its plant gain is ~2.8× lower. Evening that out means raising pitch P —
  but pitch authority scales linearly with the *estimated* 60 mm rotor height (§3.8), so tuning
  it now fits a gain to a guess. **Measure the rotor height first.**
- **The FDM's servo dynamics are representative, not measured.** §3.12 gave the tilt servos a
  10.5 rad/s rate limit and a lag of 50, which is a mid-range hobby digital servo rather than
  *this* airframe's. Real servo lag eats phase margin and the D term pays for it, so do not raise
  P or D past a bench tune's starting point without re-checking stability against bench figures.
- **`ROTOR_TMAX_LBS` is a guess** (1.874 lbf, 850 g per rotor), so the whole plant gain is scaled
  by an unmeasured number.

---

### 1.10 The debug UART has no interrupt or DMA TX path **HW**

`uart.c` has no `HAL_UART_Transmit_IT` / `_DMA` at all - every write is a polled
`HAL_UART_Transmit(..., HAL_MAX_DELAY)` that spins until the last byte clears. Actuator frames
are written synchronously from the control task (~0.33 ms each, twice per 400 Hz cycle), and
buffered log text is spun out by the `sltx` task. Neither is *wrong*, but both burn a core
where an interrupt-driven TX would sleep. `HAL_UART_Transmit_IT` plus a TC notification is the
contained fix.

### 1.11 The logger has no line atomicity **HW**

`LoggerWriteChar_*` writes one character at a time into a shared ring buffer with no mutex and
no critical section, so two tasks logging concurrently interleave characters *within* a line.
[serial_link.c](Firmware/drivers/serial/serial_link.c) serialises at the sink boundary - a
whole sink call is copied under one lock - which stops a log line from being split by a *frame*,
but it cannot reassemble a line the logger already shredded. This gets more visible now that sim
builds can log again (§2.17).

### 1.12 `tfp_printf` silently ignores `%f` **HW**

`tfp_format` ([format.c:121](Firmware/core/log/format.c#L121)) implements `u d x c s %` and
nothing else. A `%f` prints **nothing at all**, and worse, does not consume its `va_arg` - so any
argument after it is read from the wrong slot. There is currently exactly one `%f` in the tree
and it has been corrected to scaled-integer form, but the trap is still armed for the next
person. The `LOG_*_FLOATS` macros exist precisely because of this; either implement `%f` or make
it a compile error.

### 1.13 The SIL's final attitude is not repeatable, so it is a weak regression signal **SIL**

`eMISSION_MODE_MANUAL` is a **rate** loop - centred sticks command zero rate, not zero attitude -
so nothing pulls the vehicle back to level and the attitude it settles at is whatever the takeoff
transient left. Six 70 s runs, three per build, on an unmodified HEAD and on the serial-link
branch:

| run | HEAD | serial-link |
|---|---|---|
| 1 | `-0.0 / +0.0 / -0.0` | `-3.3 / +2.0 / -3.4` |
| 2 | `-3.7 / +2.3 / -2.5` | `-3.7 / +2.2 / -3.0` |
| 3 | `-4.4 / +2.6 / -2.5` | `-0.0 / -0.0 / -0.1` |

Same spread on both. **§2.15's "holds 0.0 / 0.0 / 0.0 for the entire armed flight" is one draw
from this distribution, not an invariant** - reading it as a pass/fail gate produces false
regressions, which it did once during this work. Use NaN count, `imu#` rate and CRC failures as
the gate; treat attitude as a distribution needing several runs.

### 1.14 `roll_pitch` diverges to NaN — the rate loop saturates and the model amplifies it **HW/SIL**

`sim --plan Scripts/sim/plans/roll_pitch.yaml` ends in NaN every run. Confirmed pre-existing by
stashing the estimator work and rebuilding: unmodified HEAD fails identically. One of the three
shipped plans is therefore unusable, and `hover`/`yaw_step` pass only because neither rolls far.

Traced with a per-tick capture of both sides of the link. Plan time is offset ~3 s because arming
is a barrier, so the roll doublet lands at sim t≈11.03:

| sim t | state |
|---|---|
| 11.03 | roll doublet applied. Commanded 54 °/s, achieved 58 °/s — **the rate loop tracks correctly** |
| 11.0–11.6 | rolls steadily to 28°. Expected: manual mode is a *rate* loop (§1.13), nothing holds attitude |
| 11.6–12.17 | uncommanded pitch rate builds, q → 167 °/s, θ → 32°. Cross-coupling the tune cannot reject |
| **12.1725→12.1975** | **one 25 ms window: p 141 → 1630 °/s, r +11 → −2845 °/s** |
| 12.19+ | servos slam to ±1.571 rad (their limits), motors bang-bang 0/1 every tick |
| →19.34 | rates grow 4e4 → 7e12 → 6e20 → 4e36 → NaN |

**The model produced torque it does not have.** That 25 ms window is 59,560 °/s² of roll against
the ~9,900 °/s² of full differential thrust (§1.9) — 6×. Yaw was worse: 114,240 °/s² against
`izz` = 0.014 kg·m² needs ~28 N·m, which on a 0.85 kg airframe with 0.25 m arms is ~112 N, 13× the
vehicle's own weight. That is an integrator failing, not physics.

The NaN surfaced first in the FC's `MotorCmd`, which is misleading: the firmware computes in
float32 (overflows ~3.4e38) and JSBSim in float64 (~1.8e308), so the FC merely ran out of exponent
first. The FDM's rates had already passed 4e36.

Not caused by the estimator work — `guidance.c`, `control.c` and `mission.c` read only
`NAV_VALID_ATTITUDE` out of `umsg_nav_state_t`; nothing consumes `pos_ned`, `vel_ned` or `alt`.

### What has been fixed, and what has not

Three contributing defects are closed, and each was real on its own terms:

- **§3.12** — the FDM's tilt servos now have rate and lag dynamics, so a full-scale reversal takes
  150 ms instead of one 2.5 ms frame.
- **§2.25** — `clipf32()` passed NaN straight through, and the mixer cast it to a 0 µs pulse, i.e.
  a servo commanded 135° past its stop. The SIL's `servo range` is now bounded to ±π/2 exactly.
- **§2.26** — the rate PID now has output-saturation anti-windup.

**`roll_pitch` still diverges, and the remaining cause is the tune, not a defect.** Re-measured
with all three fixes in: `hover` and `yaw_step` pass cleanly, and `roll_pitch` departs *during*
the roll doublet at about 28° of bank — in **pitch and yaw**, not roll:

```
+4.5   +0.0   -0.2      roll doublet begins
+28.0  +9.1   -1.2      uncommanded pitch appearing
-1.4  -70.8 +142.2      departed
```

That is §1.9's pitch-authority problem arriving: pitch is the slow axis at ~2.8× lower plant gain,
and a roll input raises a pitch disturbance it cannot reject. **Fixing it means raising pitch P,
and §1.9 is explicit that pitch authority scales linearly with the *estimated* 60 mm rotor
height — so tuning it now fits a gain to a guess.** Measure the airframe first; this is the bench
tune, not a bug.

Worth noting the model fix made the SIL *stricter*, not more forgiving: the old instantaneous
servos flattered the tune, and the departure is what a realistic actuator exposes.

### 1.15 Baro altitude carries a datum-elevation scale error **HW** — **fixed, see §2.27**

*Retired. The full write-up, including what the fix trades away, is §2.27.*

### 1.16 The SIL cannot catch an estimator that is confidently wrong **SIL**

The failure a departure gate exists for is one where the estimate reads level while the vehicle is
not — so gating on the estimate would check it against itself and pass. `bridge.py`'s attitude
checks are therefore on **FDM truth**, not on `Telemetry.euler`, and that is the only reason they
mean anything.

**On hardware there is no truth signal**, so the same class of fault is undetectable in flight.
Nothing in the firmware cross-checks the attitude estimate against an independent source — there
is no innovation/residual monitor, no accel-magnitude gate, no "estimate disagrees with raw
gravity vector" alarm. §2.2 (attitude inverted on all three axes) is exactly this shape and was
caught only because the SIL had truth to compare against.

Recorded here because it is a standing limit on what a green SIL run proves, not a defect with a
patch attached. The cheap partial answer is a residual monitor in `nav.c`; the honest answer is
that a bench check against a known orientation stays mandatory before first flight.

### 1.17 The SIL delivers ~92 % of the nominal control rate, whatever the rate **SIL**

`alt_hold` measures the achieved actuator-frame rate, and it lands consistently short:

| Config | Achieved | Nominal | Ratio |
|---|---|---|---|
| `CONTROL_RATE_HZ` 400, tick 2000 | 366 Hz | 400 | 91.5 % |
| `CONTROL_RATE_HZ` 500, tick 1000 | 461 Hz | 500 | 92.2 % |

Both plans passed on every criterion, with near-identical attitude numbers (peak pitch 1.2° in
each; settle pk-pk 1.17° at 400 Hz, 1.08° at 500 Hz).

**The ratio is the same at both settings, so this is not caused by the rate.** Raising the rate
delivered proportionally — 366 Hz achieved became 461 Hz achieved. 366 also sits exactly at the top
of the 250–366 Hz band `common/filter.h` already records as the SIL's observed spread "depending
only on host load", so this has been true the whole time and was simply never written down as a
number.

It is not Renode falling behind: it reported virtual time equal to host time to the microsecond
over the run (114.885 s each way), with `Current load: 0.9995` — right at the edge, but keeping up.

Why it matters and why it does not: **it does not invalidate the gains**, because the PID takes dt
per call and the PT1 filters are parameterised by cutoff rather than by a fixed coefficient,
precisely so a wandering loop rate cannot move the corner (see `filter.h`). What it does mean is
that **the SIL is not the place to measure timing margin**, and that any future check asserting a
loop rate must assert the achieved rate, not the configured one.

Worth settling on hardware: `platform.c:133` sets `SYSCLKSource = RCC_SYSCLKSOURCE_HSI`, so SYSCLK
is **HSI at 64 MHz** and the configured PLL is never selected. That is firmware, not emulation, so
the real board runs at 64 MHz too. Whether the shortfall is host load (as `filter.h` assumes) or a
genuinely tight CPU budget is answerable only by measuring the achieved rate on the bench.

---

### 1.18 `dtBaro` is measured at consumption, not at measurement **HW**

[nav.c:151](Firmware/tasks/nav/nav.c#L151) derives the altitude filter's correction interval from
when **Nav** happened to consume the sample:

```c
float dtBaro         = (float)(usNow - s_Nav.usLastBaroTime) / 1000000.0F;
s_Nav.usLastBaroTime = usNow;
```

Nothing in that chain carries the time the part actually took the measurement. The BMP390
free-runs at its programmed 50 Hz ODR, `Baro_Task` polls it every 10 ms
([baro_task.c:18](Firmware/tasks/baro/baro_task.c#L18)) and reports `eSTATUS_BUSY` when there is
nothing new, and Nav drains the topic at 400 Hz. So a sample measured on a clean 20 ms cadence is
picked up 0–10 ms late, jittering, and `dtBaro` swings roughly ±50 % around 20 ms. `vz` is what
that dt divides into, so the jitter lands directly on the climb-rate estimate.

**This is hardware behaviour, not an emulation artefact.** A real BMP390 at 50 Hz against a 100 Hz
poll jitters identically. It was invisible until the sim baro backend was retired (§3.x / SIL
§1.1) because that backend blocked on a semaphore given the instant a `BaroData` frame arrived,
so `dtBaro` was a clean 20 ms that no board ever delivers.

Measured on `hover.yaml`, changing nothing but how fast the emulated part is fed:

| baro push rate | poll rate | `vz` max err | gate (1.0 m/s) |
|---|---|---|---|
| 50 Hz (the part's real ODR) | 100 Hz | **2.01 m/s** | FAIL |
| 100 Hz (every poll finds a fresh sample) | 100 Hz | **0.75 m/s** | pass |

The pressure itself is not in question: both runs echoed with **0 mismatched** against the
loopback, so the value path — model, SPI, calibration decode, compensation polynomial, publish —
is exact. Only the timestamp is wrong.

Do **not** "fix" this by pushing baro at 100 Hz. `bmp390.c` programs the part for a 50 Hz ODR, so
feeding it faster models a part the board does not have and buys a green gate with a lie. The
honest fixes are firmware:

- Stamp the sample when the driver reads it (`Baro_Task` already calls `GetMicroseconds()` into
  `usLastUpdate`) and carry that into the topic, so Nav differentiates against measurement time.
- Or have Nav use the nominal ODR period rather than an observed interval, which is cruder but
  removes the jitter outright.

Until one of those lands, the `nav climb rate` gate fails on `hover.yaml` for a known reason.

---

### 1.19 Every sensor task gives up permanently if its part is not ready at boot **HW**

All three suspend forever on a single failed init, and are now identical in shape:

```c
if (STATUS_FAIL (Mag_Init (&s_mag))) {          // imu_task.c and baro_task.c are the same
    LOG_ERROR ("MAG unavailable; task exiting");
    vTaskSuspend (NULL);
}
```

The IMU briefly had a retry loop for this (§2.32); it was removed deliberately, so that the three
sensor tasks behave the same way rather than one of them being special. That makes the behaviour
consistent and this entry the single place it is tracked — it does not make it safe.

The exposure is a part still coming out of reset when its task first runs. All three come off the
same power ramp. The MMC5983's reset settle is 15 ms
([mmc5983.c](Firmware/drivers/mag/mmc5983.c) `RESET_SETTLE_US`) and the BMP390's is 2 ms, both
measured from a part that has already powered up; the BMI323 is the slowest, since a failing
`IMUSoftReset` spends ~2.5 s waiting on the feature engine. Losing the IMU costs the aircraft the
whole flight, losing the mag costs the heading reference, and losing the baro costs the altitude
estimate and `NAV_VALID_BARO_ALT` with it.

**The SIL cannot catch this, by construction.** Every one of the three inits reads a chip id,
writes config and reads it back, and the baro also reads its calibration block — the emulated
parts serve all of that from their register files whether or not the bridge has connected
(`Scripts/renode/{BMI323,MMC5983,BMP390}.cs`, the `activeSample == null` branch in `Read`). Only
`STATUS` is gated on a pushed sample, and no init reads it. So init cannot fail under the SIL and
the failure path is never executed. Only a bench test, or a "part not present" mode in the models,
reaches it.

If retrying is ever reinstated, note §2.33's warning: the IMU's loop needed the
context-allocation fix first, and `ImuDrv_Init` / `MagDrv_Init` / `BaroDrv_Init` all `Allocate()`
a context per call, so a naive loop leaks one per attempt on `heap_1`.

---

## 2. Fixed — firmware, affected real hardware

### 2.1 `umsg_publish()` aliased the caller's buffer; `peek` read dangling stack **HW**

`umsg_publish()` stored the caller's pointer (`msg->msg_value = data`) instead of copying;
`umsg_peek()` read through it. `Rc_Update()` published a **stack local**, `vTaskDelay()` reused
that stack, and `guidance.c` peeked dead bytes — which decoded to `channels[3]` ≈ 42553 µs.

That one value produced **both** reported actuator symptoms: throttle 41.55 → clipped to motor
1.000, and the same 41.55 → servo 41553 µs → 62.9151 rad. It looked deterministic because
`vTaskDelay`'s frame is identical every iteration.

Queue subscribers were always safe — `umsg_port_send` copies. Only `peek` read the alias.

**Fixed by deleting `peek` entirely.** All seven consumers now subscribe (length 1, giving
latest-value semantics via `xQueueOverwrite`) and `receive(..., 0)` into a local cache. The
`msg_value` field and a leaked `umsg_subscribe` malloc are gone. Published data no longer needs
static lifetime, because nothing retains the pointer.

### 2.2 Attitude was inverted on all three axes **HW**

[filter.c](Firmware/common/filter.c) transcribed the Euler formulas from page 6 of the Madgwick
report verbatim. Those give angles for `ˢᴱq` — Earth relative to *sensor*. Flight code wants
`ᴱˢq`, the conjugate. Per-axis test before → after:

| input | before | after |
|---|---|---|
| roll +20 | **−20** | +20 |
| pitch +15 | **−15** | +15 |
| yaw +30 | **−30** | +30 |

All three negated is the signature of a conjugate quaternion, not three sign typos. Substituting
`(q1, −q2, −q3, −q4)` flips the second term in each expression; the corrected roll form reduces
to the standard aerospace ZYX extraction, which is an independent check that it is right.

**This would have been a very bad first-flight surprise.**

### 2.3 `GetMicroseconds()` returned exactly 2× real time **HW**

Measured after exactly 1.0 virtual second: `uwTick * 1000` = 1,000,000 µs and
`DWT->CYCCNT / (SystemCoreClock/1e6)` = 999,608 µs — both measure time since boot, and
[core_shared.c](Firmware/core/core_shared.c) summed them as though the second were a sub-ms
remainder. Corrupted every `dt`: the filter step, `pid.c`, `filter.c`.

Observed effect: with doubled `dt` the attitude never converged — it precessed and then went
chaotic through ±180° despite a *static* input with zero gyro rates.

That fix corrected the *scale* of the reading but kept the tick + `SysTick->VAL` construction,
which turned out to be unfixable in that form — see §2.8. The function is now DWT-only.

### 2.4 `Rc_Task` starved every sensor task **HW**

[rc.c](Firmware/tasks/rc/rc.c) was an unthrottled `while(1)`, and `Rc_Update()` never blocks —
`umsg_publish` bottoms out in `xQueueSendToBack(..., 0)`. Permanently ready at priority 3, above
`Imu_Task` (2), `Mag_Task` (1), `SimTelemetry_Task` (1).

Measured over 0.5 s: `Rc_Update` **725,309** calls; `Imu_Task`, `Mag_Task`, `SimTelemetry_Task`
**zero**. Priority arithmetic — identical on real silicon. Fixed with a 50 Hz `vTaskDelay`.

### 2.5 Mixer servo output was unclamped and ~20× out of range **HW**

`mixer.c` accumulated signed mix contributions straight into a **`uint16_t`** (negatives wrapped
to ~65k), treated 0 µs as neutral instead of `SERVO_CENTER_US_DC`, and never clamped. Now
accumulates in float, centres on 1500 µs, and clips to the 500–2500 µs travel. Clamp verified to
engage: a mix summing to 2583 µs holds at exactly 2500.

### 2.6 `Mag_Task` busy-spun; sim mag backend never blocked **HW/SIL**

`SimLink_GetMag()` returned immediately, so `Mag_Task` spun at priority 1 (~639k `Mag_Update`
per 0.5 s). Fixed properly rather than rate-limited: `sim_link` now signals a **second** binary
semaphore per `SensorData` and exposes `SimLink_WaitMag()`, so the mag backend blocks exactly as
the IMU one does. Two semaphores are required — a binary semaphore wakes exactly one waiter, so
sharing would let the consumers steal samples from each other.

`SimLink_WaitSensor` was renamed `SimLink_WaitImu` to match; `SimLink_GetMag` removed.

### 2.7 Single-core builds were silent **SIL**

`CFG_PRIMARY_LOGGER` was `CM4_CPUID`, so CM7 wrote to a ring buffer and signalled **CM4** to
drive the UART. With no CM4 nothing drained it — zero log output. Now conditional on
`SINGLE_CORE`.

---

### 2.8 `GetMicroseconds()` went backwards; that alone destroyed the attitude estimate **HW**

This was §1.2 — "attitude drifts to ~162° with a fixed input, unexplained". It is not an
estimator fault at all. Measured, not inferred.

**The filter is correct.** Driven on the host with the exact dry-run inputs (static 20° roll,
zero gyro, the bridge's synthesized mag) at a *constant* `dt`, `MadgwickFilter_Update` converges
to 20.00° and holds it to two decimals for 300 s, in both 6DOF and 9DOF. So only `dt` or the
sample data could move it — and in `--dry-run` every sample is byte-identical, which leaves very
little room.

Temporary instrumentation in `Nav_Update` (max `dt`, count of `dt` > 50 ms, count of samples
deviating from the constant) settled it:

| build | max dt | dt spikes / 70 s | bad samples |
|---|---|---|---|
| before | **4295.0 s** | 13 and climbing | 0 |
| after | 4.4 s (the one boot gap) | 1, never increments | 0 |

4295 s is `2^32 / 1e6` — an unsigned underflow. `GetMicroseconds()` built its result from
`HAL_GetTick()` **plus** `SysTick->VAL`, two registers that cannot be sampled coherently under an
RTOS: the tick only advances when the SysTick ISR runs, and FreeRTOS masks interrupts for every
critical section, so SysTick wraps and reloads while the tick is still stale. The reading then
lands up to a millisecond in the past. Every caller takes an unsigned difference, so one
backwards step becomes a `dt` of 4295 s; the filter integrates a single enormous step and the
quaternion ends up on an arbitrary orientation. It fired roughly **once every five seconds**.

Two more defects in the same expression: re-reading the tick does not help (both reads return
the same stale value), and `SysTick->LOAD / 1000U` truncates 63999/1000 to **63**, letting
`usTime` reach 1015 so the reading overshoots the millisecond and then steps back.

**Fixed by deriving the clock from `DWT->CYCCNT`** — one free-running register, so a single read
is inherently coherent. It wraps every 67 s at 64 MHz, too soon to divide down directly (the
quotient would wrap at ~67e6 instead of 2^32 and break the callers' difference arithmetic), so
whole microseconds are accumulated and the leftover cycles carried, under a PRIMASK-guarded
critical section. DWT is already enabled by `DWTInit()` and modelled in the Renode overlay. It
must be called at least once per 67 s, which the GNC loop does hundreds of times a second.

`Nav_Update` also seeds its timestamp on the first sample rather than at `Nav_Init()`: the link
takes ~4.4 s to come up, and integrating that gap as one `dt` threw the estimate ~14° past the
truth and cost several seconds of beta-limited slewing.

Verified over a 150 s `--dry-run`: the estimate converges 0 → 20.0° in ~3 s with no overshoot and
holds `+20.0 / +0.0 / +0.0` for the rest of the run. Also clean at 50, 100, 200 and 400 Hz.

**Corrections to the original report.** "Holds +20.0 for tens of seconds" does not reproduce — on
the current tree the estimate leaves 20° within a second or two, on an unmodified HEAD build as
well. The `imu#` of first deviation varies run to run, as a race should; the 13926–19376 figures
are samples of that distribution rather than a stable signature. A rate-ceiling theory raised
during this investigation — that ≥300 Hz overran the sim link and let corrupt frames through —
was **wrong**: `bad samples` was 0 at every rate, and 400 Hz is clean once the clock is fixed.
The apparent rate dependence was only that a higher rate calls `GetMicroseconds()` more often and
so hits the race sooner.

### 2.9 `mapf32()` was broken for every ascending range **HW**

`if ((fromMin - fromMax) < 0.0001F)` was missing the absolute value, so for any
`fromMin < fromMax` the difference is negative, the guard fires, and the function returns
`toMin` immediately. Now `ABS_F32 (fromMax - fromMin)`. Its only caller was dead code inside
`Mixer_MixServos` (an `inputs[]` array computed and never read), removed as part of §2.11, so
`mapf32` now has no callers at all — the fix is for the next one.

### 2.10 Out-of-bounds writes in the mixer **HW**

`Mixer_MixServos` indexed `mixedInputs[SERVO_ID_TO_IDX(...)]` unchecked; the airplane profile
names servo IDs up to 6 against `BRD_SERVO_COUNT` = 2, and `eSERVO_ID_NONE` underflows to a huge
index. Both are now bound-checked.

`Mixer_MixMotors` had the same shape and was **not** merely latent: it iterates
`pProfile->motorCount` (2 for the tilt and airplane profiles) into `motorOutputs[BRD_MOTOR_COUNT]`,
and nucleo-h747zi has `BRD_MOTOR_COUNT` = 1 — so that board's build wrote one element past the
array. Now clamped to the board's count.

### 2.11 Tilt-rotor mixing is now a real bicopter mix **HW**

`g_TiltServoMix` fed ROLL + PITCH + YAW *and THROTTLE* into both tilt servos at equal weight, so
tilt angle tracked the throttle stick. `ServoMix_t` had no weight field, so every contribution
was +1 and no differential mix was expressible at all.

`ServoMix_t` gained a signed `float weight`, and the tilt profile now uses the standard bicopter
allocation, derived from the airframe geometry (rotors on the lateral axis at y = ±0.25 m, each
on a fore/aft tilt servo):

| axis | mechanism | mix |
|---|---|---|
| roll | differential **thrust** | motor L `+1.0`, R `−1.0` |
| yaw | differential **tilt** | servo L `+1.0`, R `−1.0` |
| pitch | collective **tilt** | servo L and R `−1.0` — rotors sit above the CG, so tilting forward pitches the nose down |

Throttle no longer reaches the tilt servos. The `TEST` and `AIRPLANE` profiles carry explicit
`1.0` weights, preserving their behaviour exactly. The tilt profile's `servoCount` claimed 3 for
a 2-servo airframe; corrected, though the field is currently unread.

Signs were derived from the moment arms and cross-checked against the FDM — which is how §4.7
came to light. The pitch row cannot be validated in the SIL until that is addressed.

### 2.12 The rate loop could only ever command 2.8 % of travel **HW**

This was §1.1. The sign half was fixed earlier; this is the magnitude.

`Pid_UpdateAxis` clipped to `CFG_PID_MIN/MAX_VALUE` = ±5.0, and `control.c` then applied
`clipf32(output, ±CONTROL_MAX_RATE_DEG_S) / CONTROL_MAX_RATE_DEG_S` with
`CONTROL_MAX_RATE_DEG_S = 180`. The ±5 clip bound first, so no axis could ever ask the mixer for
more than **5/180 = 2.8 %** of actuator travel. The integrator saturated even that on its own
(limit 25 × I 0.3 = 7.5 against a ±5 clip), so with no anti-windup the output sat pinned and
kept the wrong sign after the error reversed.

**The PID output is now the normalised mixer command directly.** `CFG_PID_MAX_VALUE` is 1.0,
`CFG_PID_MIN_VALUE` is `-CFG_PID_MAX_VALUE`, and `control.c` uses the return value as-is —
`CONTROL_MAX_RATE_DEG_S` is gone. Every gain was divided by 180 (the factor that used to be
applied downstream), so the small-signal response is **identical** to before and the change
implies no retune by itself; it only removes the ceiling. Gain quality is a separate matter —
see §1.8.

### 2.13 Arming now waits for the attitude estimate to settle — in the firmware **HW**

`Mission_IsArmable()` checked `NAV_VALID_ATTITUDE`, but that bit is set from the very first IMU
sample: it means "the filter is running", not "the filter has converged". Arming on it hands the
rate loop an attitude error of tens of degrees.

`tasks/mission/mission.c` now also requires the estimate to have stopped moving — every Euler
axis within `ARM_ATTITUDE_STABLE_DEG` (1.0°) of a reference for `ARM_ATTITUDE_SETTLE_US` (3 s).
Any axis leaving the band re-arms the reference and restarts the clock, so the interlock also
blocks arming while the airframe is being handled. The comparison is wrap-safe, and non-finite
Euler values invalidate the window rather than passing it.

Two consequences handled with it:

- **A raised arm switch is now a standing request, not an edge.** The gate can take seconds to
  open, and the old edge-triggered path consumed the request and never retried while the pilot
  held the switch up — it would have hung disarmed forever. The request clears as soon as the
  switch leaves the armed region. A shell arm stays one-shot: it is an explicit command, so a
  rejection is reported and dropped.
- **Rejections name the reason** (`no valid attitude estimate` / `attitude estimate still
  settling` / `throttle not at minimum`) and log only when the reason changes, so a waiting
  request does not spam the log every iteration.

This supersedes the bridge-side gate added earlier as §3.5 — the interlock belongs in the
firmware, where it protects real hardware too, not just the SIL. `bridge.py` is back to acting
like a pilot: throttle down, switch up after `--arm-delay` (2.0 s), wait for `Telemetry.armed`.
`--arm-settle` / `--arm-settle-deg` are gone.

Verified in the SIL: the FC holds `armed=False` through convergence and the settle window, then
arms on its own with the switch held.

### 2.14 The mission task's stack was overflowing **HW**

Found by the above wedging the sim link: telemetry stopped after a single frame, with no fault
logged. `STACK_MISSION` was 128 words (**512 B**), and `Mission_Update` already holds a
`umsg_rc_input_t` (68 B), a `umsg_nav_state_t` (72 B), an arming request and the outgoing mission
state on that stack at once — ~170 B of message structs before any call frame — and it logs,
which is printf-shaped. Adding the settle tracker tipped it over.

Raised to 256 words (1 KB). Cross-checked against the heap measurement in §3.7: used heap rose by
exactly 512 B, the size of the bump. Nothing detected this — see §1.7.

### 2.15 The rate loop now flies — three separate faults were stacked on it **HW**

This was §1.8. The first closed-loop SIL run (full JSBSim, not `--dry-run`) departed to −119° of
roll the instant it armed and put the FDM into NaN. Three independent defects, found in order:

**a. The D term had the wrong sign.** `Pid_UpdateAxis` computed `derivative = d(error)/dt` and
then applied `- d * derivative`. With a steady setpoint `d(error)/dt = -d(rate)/dt`, so that
evaluates to `+d * d(rate)/dt` — positive feedback on exactly the motion D exists to damp.
Fixed by differentiating the **measurement** instead and keeping the minus, which is the
textbook derivative-on-measurement form: it restores the damping sign *and* removes the
derivative kick a stick step used to inject. The first sample is skipped so a stale
`prevMeasurement` cannot produce a false spike.

**b. D was ~28× too large for the plant.** The actuators are enormous relative to the inertias:
full differential thrust is ~9900 °/s² of roll against `ixx = 0.012`. The D term therefore closes
a nearly algebraic loop of gain `Kd·G`, which was **~2.8 for roll and ~1.0 for pitch** — at or
past 1, so the loop limit-cycled at the sample rate. Measured against the FDM, a 10 °/s
disturbance grew to **745 °/s**.

Swept against the model: the boundary sits between 0.2× and 0.25×, but **0.2× is only stable at
400 Hz — it diverges at 200 and 100 Hz**, which would have been a trap since the loop rate
follows the IMU. Shipped 0.1× for roll and pitch, stable and near-identical at 100/200/400 Hz.
Yaw D was left alone (`Kd·G ≈ 0.007`, nowhere near the boundary).

**c. `dt` could be exactly zero, and the PID divided by it.** With (a) and (b) fixed the vehicle
stopped tumbling but the FDM still went NaN about a second after arming. `GetMicroseconds()` has
1 µs resolution and the loop is paced by queued IMU samples, so when a burst drains, two
iterations land in the same microsecond. `(current - prev)/0` is ±inf, or NaN when the rate has
not changed — and the mixer forwards it: inf clips to full deflection, NaN propagates into the
flight model. `Pid_UpdateAxis` now rejects a non-positive or NaN `dt` (holding P plus the
existing I-term and leaving its state untouched), and `control.c` bounds `dt` to
[0.5 ms, 20 ms] so neither a duplicated iteration nor a stall scales the I and D terms.

The I-term clamp also moved into output units: `integralLimit` now caps the I-term's
*contribution* with back-calculation anti-windup, instead of bounding raw accumulated error where
the ceiling depended on the I gain (25 error-seconds × 0.00167 was only ~4 % of travel). Set to
0.3.

**Result — the first closed-loop flight of the full stack.** 120 s run, ground start, arms on its
own, lifts off, and holds `0.0 / 0.0 / 0.0` for the entire armed flight with **zero** NaN. The
`--dry-run` case still holds +20.0°. Measured disturbance rejection (20 °/s, no overshoot,
unchanged at 100/200/400 Hz): roll 0.67 s, yaw 0.12 s, pitch 1.47 s. Remaining caveats in §1.9.

### 2.16 Stack sizes are now measured, and overflows are trapped **HW**

This was §1.7. `configCHECK_FOR_STACK_OVERFLOW` is now `2` on both cores, with a
`vApplicationStackOverflowHook` in `main.c` that records the task name in
`g_pOverflowedTaskName` and halts via `CriticalErrorHandler()`. It deliberately does not log —
the logger is printf-shaped and wants the stack that just ran out. It earned its keep
immediately, catching a 640-byte `TaskStatus_t[16]` that the diagnostic below had put on the
telemetry task's stack.

Every task's `uxTaskGetStackHighWaterMark()` was then sampled over a SIL run. Peak usage in
words, against the old allocation:

| task | used | was | now |
|---|---|---|---|
| imu | 99 | 128 (77 %) | 256 |
| mag | 82 | 128 (64 %) | 256 |
| rc | 82 | 128 (64 %) | 256 |
| mission | **158** | **128 — overflowing** | 384 |
| guidance | 142 | 256 (55 %) | 384 |
| control | 253 | 512 (49 %) | 512 |
| nav | 213 | 512 (42 %) | 512 |
| simrx | 236 | 512 (46 %) | 512 |
| idle | 24 | 128 (19 %) | 128 |

The mission row is independent confirmation of §2.14: 158 words used against 128 allocated. `imu`
was one `LOG_ERROR` away from the same fate. Anything that can log now gets roughly 2× its
measured peak. `STACK_RX` is CM4-only and the single-core SIL never runs it, so it is unmeasured
and left at 128.

### 2.17 The debug UART could serve only one of its three consumers **HW**

`Uart_t` holds exactly one `rxCallback` and every `UartPort_Init` clobbers the previous one
([uart.c:220](Firmware/drivers/serial/uart.c#L220)); `UartPort_Write` is an unguarded blocking
poll on a shared handle. So the logger (TX, ASCII), the shell (RX, `[len][pb]`) and the sim link
(RX+TX, `[AA][55]…`, and a different baud) could not coexist, and `main.c` resolved it by
compile-time exclusion: `SIM_HIL` started the sim link *instead of* the debug serial and skipped
the shell.

The cost was that a `-D sim` build was **blind**. `LOG_*` still ran the full `tfp_printf`
formatting and burned the stack, then discarded every byte because no sink was registered - the
only channel out was the three `Telemetry.euler` floats.

Fixed by [serial_link.c](Firmware/drivers/serial/serial_link.c), which owns the UART outright;
`sim_link` and `shell` became clients. Log text stays raw ASCII and binary stays framed, which is
unambiguous on the wire because log output is 7-bit ASCII and the frame magic is `0xAA` - so a
text byte can never open a frame, and a plain terminal still shows readable logs. Frames are
written synchronously (rate-limited by their producers, so a queue would only add latency); only
text is buffered, drained in 32-byte chunks by the `sltx` task so a log line can delay a frame by
at most ~0.7 ms. Verified in a 130 s closed-loop SIL run: logs, telemetry and a shell `set_pid`
all on one wire, zero NaN, zero dropped frames, zero dropped log bytes.

### 2.18 `Shell_Update()` had no caller - the shell had never run **HW**

`Shell_RxCb` filled `s_Buf` from the UART ISR and set `s_HasMsg`, and **nothing anywhere in the
tree ever drained it** - `Shell_Update` had no caller. Meanwhile its `UartPort_Init` still
claimed the debug UART's only RX callback slot in every hardware build, so the shell was
simultaneously dead and harmful: it could not act on a command, but it stopped anything else
from receiving one.

Commands now ride the common framing as msg id 6 and are decoded in the SerialLink RX task, so
the handler finally has a caller. Confirmed by sending `set_pid` mid-flight in the SIL and
watching the FC log the change. Two framings on one inbound stream could not be disambiguated -
a shell frame whose length byte is `0xAA` is genuinely ambiguous - so the move was required, not
cosmetic; `Scripts/gui/conf.py` frames the same way.

### 2.19 `SyncProcessTasks()` had no caller - CM4 log messages never reached the UART **HW**

The other half of §2.7. `CFG_PRIMARY_LOGGER` was `CM4_CPUID` in dual-core builds, so CM7 wrote
to its ring buffer and signalled CM4 via `SyncNotifyTaskUartOut` - onto a queue that
`SyncProcessTasks()` drains, and **that function had no caller anywhere**
([sync.c:150](Firmware/core/sync.c#L150)). CM7 filled its 4 KB ring and then spun in
`LoggerWriteChar_Blocking`.

Forced into scope by §2.17: the FreeRTOS objects guarding the link belong to CM7, so CM4 must not
drive the sinks itself. `CFG_PRIMARY_LOGGER` is now unconditionally `CM7_CPUID` and the `sltx`
task calls `SyncProcessTasks()` on each wakeup, which finally gives it a caller.

### 2.20 `Rx_Task` was an unthrottled spin loop - it starved the whole GNC loop on CM7 **SIL/HW**

`Rc_Task` in §2.4 was fixed with a 50 Hz `vTaskDelay`; `Rx_Task`
([tasks/rx/rx_task.c](Firmware/tasks/rx/rx_task.c)) had the identical shape and was left alone,
because it only ever ran on CM4, where there is nothing else to starve.

Putting it on CM7 under `SINGLE_CORE` - so a single-core build still gets CRSF - made it matter
immediately. `Rx_Update()` polls and never blocks, so at `TASK_PRIORITY_RX` = 3 the task is
permanently ready and starves everything below it: `imu` (2), `mag` (1), `simtlm` (1) and
`sltx` (1). The symptom is total, not partial - the FC logs its two boot lines and then goes
silent, because even the log TX task cannot run.

Measured, with and without the CM7 task:

| | telemetry samples / 45 s | heap free | after `vTaskStartScheduler()` |
|---|---|---|---|
| with `Rx_Task` on CM7 | **0** | 15976 | silent |
| without it | 83 | 16584 | arms and flies |

The 608-byte heap delta is exactly its 128-word stack plus TCB, which is what identified it.

Fixed the same way as §2.4: a 50 Hz `vTaskDelay` matching `Rc_Task`. Verified in a 90 s
closed-loop SIL run with the CM7 task present - 171 telemetry samples, zero NaN, arms and holds
attitude, and a shell `set_pid` still lands mid-flight.

**Resolved by §2.23.** This used to note that `Rx_Task` was *redundant* under `-D sim`, because
`SimLink_OnRc` wrote `g_Rx` directly while `Rx_Task` polled USART3 - two writers on one buffer.
The shortcut is gone: the bridge feeds real CRSF to USART3, so `Rx_Task` is now the only writer
and the sole RC path in every build.

### 2.21 The CRSF receiver path had never decoded a frame — five spec violations **HW**

Standing the SIL's RC path up on the real receiver driver (SilResearch §1.1) meant reading
`drivers/rx/crsf.c` against the TBS CRSF specification for the first time. It did not conform in
five places, and **none of this is emulator-specific — it is what the aircraft would fly with**.

The path had evidently never been exercised end to end: with the first two faults below, a real
receiver could not have armed the vehicle in any predictable way.

| | Fault | Effect |
|---|---|---|
| C1 | `CRSF_CHANNEL_MIN/MAX` were 508/1496 | Wrong channel scaling — see below |
| C2 | `int channel_N : 11` (signed) | Everything above 1023 ticks read negative and clamped to full deflection |
| C3 | `frameLen != sizeof(CrsfChannelsPayload_t)` | Compared the length byte (24) against the payload size (22) — **every RC frame rejected** |
| C4 | Length range checked against `CRSF_MAX_PAYLOAD_SIZE` (58) | Spec range is 2–62; legal frames of 59–62 discarded |
| C5 | `(1000000 / 416666) * …` | Leading integer division truncates to 2, so the inter-frame timeout was 2048 µs instead of 3072 |

**C1 is the one that matters in flight.** The spec fixes the conversion exactly —
`TICKS_TO_US(x) = (x - 992) * 5 / 8 + 1500`, so 1000–2000 µs is ticks 192–1792. 508/1496 are
1198 µs and 1815 µs under that formula: a 617 µs slice of stick travel stretched across the full
output range with everything outside it clamped. Errors reached 193 µs mid-stick, and a *centred*
stick read 1489 µs rather than 1500 — which `Guidance_Update` turns into a standing
**−0.069 rad/s (−3.96 °/s) rate demand on roll, pitch and yaw with the sticks centred**.

The formula itself was already right; only the two constants were wrong. With 192/1792 it reduces
to `0.625·v + 880`, algebraically identical to the spec, and matches it **within 1 µs** across the
full 172–1811 transmitter range (a rounding-direction difference below centre), exactly at all
three endpoints.

**C2 and C3 were dangerous together.** Fix only C3 and the first test looks like it passes: the
arming gesture is throttle-low (508 ticks) and AUX1-high (1496 ticks), and while throttle maps
correctly, AUX1 reads −552, wraps through `Crsf_MapChannel`'s `uint32_t` parameter, and clears
`ARM_AUX_THRESHOLD` **by accident**. Centred sticks are also under 1023 and map correctly. So a
hover smoke test arms and holds level, and the bug only detonates the first time a stick goes past
~1522 µs — where it snaps to full deflection.

Note the spec's own code sample declares these fields `int`. The sample is wrong: signed 11-bit
tops out at 1023 while the spec documents channel values to 1811, so it cannot represent its own
range. `unsigned` is what matches the documented behaviour, and is what Betaflight uses.

**Fixed**, and verified by `Tests/UnitTest/test_crsf.c`, which was written against the spec first
and failed 11 of 17 on the unmodified tree. C2 confirmed on the target: the Release `crsf.c.obj`
went from 16 `sbfx` / 0 `ubfx` to **0 `sbfx` / 20 `ubfx`**.

Conforming already, and left alone: the CRC8 table (byte-identical to the spec's 0xD5 table), CRC
coverage and position, frame length semantics, the 64-byte cap, the 416666 baud default, and the
`Crsf_Bind` frame's two hardcoded CRCs (0x9E command / 0xE8 packet — both recomputed and correct).
The sync byte is deliberately *not* validated: the spec permits 0xC8, 0x00 or any device address
there, so a strict check would itself be non-conformant — which is why C5, the only legitimate
resync mechanism left, mattered more than its magnitude suggests.

### 2.22 RC link loss is now detected **HW**

There was no RC timeout of any kind: `g_Rx.channels` held its last value forever, and `Rx_Task`
discarded `Rx_Update`'s return status. A lost link left the vehicle flying its last commanded
rates indefinitely, with nothing anywhere reporting it.

The CRSF spec is explicit — a receiver in failsafe simply stops transmitting, and recommends the
FC waits one second before reacting. `Rx_Update` now timestamps each good frame, derives
`Rx_IsLinkUp()` against `RX_LINK_TIMEOUT_US` (1 s), logs once per transition, and `Rc_Update`
publishes `link_quality` from it instead of a hardcoded 0. `Telemetry.rc_link_up` carries it to
the bridge, which prints `rc=up` / `rc=DOWN`.

Measured in the SIL by cutting CRSF mid-flight (`sim --rc-stop`): the transition lands one second
later, with exactly one log line.

**Detection only — nothing acts on it**, by decision. The channels are deliberately not scrubbed:
disarming an airborne multirotor drops it, and a controlled descent needs an altitude estimate
`nav.c` does not produce (it publishes `alt = 0.0f`). Choosing the action is open work; see
§4.12.

**Consequence:** `Rx_Task` now logs, so `STACK_RX` moved 128 → 256 words. Measured with
`uxTaskGetStackHighWaterMark` over a SIL run: **109-word peak**. Against the old 128 that is 85 %
occupancy — worse than the 77 % that §2.16 already calls one `LOG_ERROR` from overflow.

### 2.23 The SIL bypassed the entire receiver path; it no longer does **SIL**

`SimLink_OnRc` decoded an `RcInput` sim-link frame straight into `g_Rx.channels`, so every SIL run
skipped the UART ISR, CRSF deframing, the CRC, the 11-bit unpack and the channel mapping — which
is precisely why §2.21's five faults survived so long. §2.20's "still open" note (two writers on
one buffer, `Rx_Task` redundant under `SIM_HIL`) is resolved by the same change.

The bridge now sends spec-conformant 0x16 frames to USART3, which Renode exposes on port 4001
alongside the sim link on 4000. `SimLink_OnRc`, `SIM_MSG_RC`, `message RcInput` and its nanopb
option are gone; **frame id 2 is retired and must not be reused** (an older bridge would still be
sending it). `SIM_PB_H_MAX_SIZE` consequently dropped 96 → 45; `SERIAL_LINK_MAX_PAYLOAD` was left
at 96 as harmless headroom, with its now-false "RcInput_size" comment corrected.

Renode needed nothing beyond a frequency pin: `usart3` is a real `UART.STM32F7_USART` in the
shipped platform, wired `exti@28 -> nvic@39`, and `Crsf_Init` was already claiming it on every SIL
boot.

Verified with a negative control — **with no `--rc-port`, zero armed transitions** — which is what
proves the arming in the other runs comes from the CRSF path and nothing else.

### 2.24 The host unit-test suite could not build, and had not for some time **SIL**

Supersedes §4.11, which reported only the `test_filter.c` API drift. The problem was broader:
**every** test includes `"unity/unity.h"`, but `Tests/CMakeLists.txt` fetched Unity to
`_deps/unity-src/src/`, where that path cannot resolve — so no target compiled, including the ones
§4.11 assumed were fine. On top of that `spi.c` and `uart.c` no longer build against
`stubs/hal_stub.c` at all (missing `LL_SPI_*` and `__HAL_UART_*`), and the vendored `Tests/unity/`
(2.6.1) and the fetched copy (2.5.2) were different versions.

Rebuilt around the vendored Unity — which is what the includes always expected, and which drops
the network dependency from configure. The old tests were removed rather than left as decoration;
`test_crsf.c` is the current suite. See `Tests/README.md`.

**One trap worth recording.** MinGW defaults to `-mms-bitfields` (MSVC bitfield layout), under
which `CrsfChannelsPayload_t` packs to **32 bytes** instead of the 22 ARM EABI produces. The host
would then test a wire format the firmware never emits — and briefly did: the first run "passed"
an oversized-frame length check purely because 32 coincided with the frame length under test. The
build now forces `-mno-ms-bitfields` and the test static-asserts the size. Any future host test of
a wire-layout struct needs the same.

### 2.25 `clipf32()` passed NaN through, and the mixer turned that into a servo past its stop **HW**

`clipf32(v, lower, upper)` was the natural `if (v < lower) ... if (v > upper) ...`. Every
comparison against NaN is false, so NaN fell through both branches and came back **unchanged** —
breaking the single guarantee the function exists to make.

That matters because callers cast the result to an integer.
[mixer.c](Firmware/tasks/control/mixer.c) does `(uint16_t)clipf32(us, 500, 2500)`, and a
float-to-integer conversion of NaN is undefined behaviour that lands on 0 here. A 0 µs pulse is
−1.5× full travel: **a servo commanded 135° past its own mechanical stop**. The SIL reported it as
`servo range -2.356 .. +1.571 rad`, and −2.356 is exactly −1.5 × π/2 — the arithmetic signature of
the cast, not a coincidence.

Fixed by ordering the first test as `!(v >= lower)`, which NaN takes. One character, and the range
guarantee holds for every caller including the PID output clamp. NaN clamps to `lower` because that
is the cheapest way to restore the invariant, **not** because full negative deflection is a good
failsafe — a caller needing NaN to mean "neutral" has to say so itself.

Verified: the SIL's servo range is now `-1.571 .. +1.571` exactly, on a run where the FDM still
diverges and still feeds NaN back.

### 2.26 The rate PID had no output-saturation anti-windup **HW**

Carried from §1.9. The existing clamp bounded the I-term's *contribution*, but nothing stopped the
integral growing while the total output was already pinned at `CFG_PID_MIN/MAX_VALUE`. The actuator
cannot deliver past full travel, so every error-second past that point is stored lag that has to be
unwound before the loop can respond the other way.

Fixed by conditional integration: integrate unless the output is already saturated **and** this
error would push it further out. Errors that bring it back toward the linear region are always
integrated, so recovery is immediate rather than waiting out the accumulated excess.

This became load-bearing rather than theoretical once the FDM got realistic servo rate limits
(§3.12) — with instantaneous servos the loop left saturation fast enough that windup rarely showed.

The other half of §1.9's tail, the unfiltered D term, is closed too: `Pid_UpdateAxis` now filters
the measurement through a 60 Hz PT1 before differencing it. Measured on a host replica with
injected gyro noise — −64 % D-term noise energy, −59 % servo activity, for 2 ms of settling. See
ControlResearch §2.4 for the cutoff derivation and why the SIL could not contribute to it.

### 2.27 Baro altitude carried a datum-elevation scale error **HW**

Was §1.15. `Baro_PressureToAltitude()` used the textbook 44330 coefficient, which is not a constant
at all — **288.15 K / 0.0065 K/m *is* 44330**, i.e. the sea-level special case of T0/L. Referencing
it to a datum that is not at sea level left a pure scale error, because the air column being
measured is colder and denser than the formula assumes: **+0.23 % of height per 1000 m of field
elevation**, so a true 100 m climb read 102.31 m from a 1000 m field.

Fixed by feeding the datum's own measured temperature, which `umsg_sensors_baro_t` already carried
and nothing consumed. `Nav_UpdateBaro` now averages temperature alongside pressure over the same
50-sample warmup. Verified below a millimetre from 0 to 2000 m of field elevation.

**What it trades.** Accuracy now rides on the temperature reading rather than on field elevation,
at dT/T0 ≈ 0.35 % per °C. Break-even against the old constant is roughly **0.65 °C per 100 m of
field elevation** — so this is a clear win at altitude, a wash near sea level, and a net *loss* if
the reading is worse than that. Which matters, because a barometer reports its own **die**
temperature, not ambient, and a part in a warm enclosure reads high. Two things keep it honest: the
datum is taken at boot, when the board is closest to ambient, and
`test_PressureToAltitude_ScalesWithDatumTemperature` pins the sensitivity so the trade cannot be
forgotten.

**The SIL cannot see this failure** — JSBSim reports true air temperature with no self-heating
model. Setting a `<bias>` on `sensor/baro/temp_C` would make it visible, and is the obvious next
experiment; see §3.13 for the same lesson about noise.

---

### 2.28 `vTaskDelete(NULL)` on a `heap_1` build deadlocked the whole FC **HW**

Five tasks — imu, mag, baro, gps, rc — ended a failed init with `vTaskDelete (NULL)`. Both cores
build `heap_1`, whose `vPortFree` is `configASSERT (pv == NULL)`, and `configASSERT` is
`taskDISABLE_INTERRUPTS(); for(;;);`. A self-deleting task defers its TCB and stack free to the
idle task, so **the idle task hit that assert and spun forever with interrupts off**. No fault, no
log line, no watchdog — the flight controller simply stopped.

On hardware that means **one loose sensor connector deadlocks the aircraft** rather than degrading
to the sensors that still work.

It stayed invisible for two independent reasons. A `-D sim` build's sensor backends always
initialise, so no task ever exited; and on the default profile `Imu_Task` free-ran at priority 2
(§2.29) and starved mag and baro before they could reach their exit path. Fixing §2.29 exposed it
within a second of boot.

Fixed: `vTaskSuspend (NULL)`. The task parks, nothing is freed, and the leaked TCB and stack are
exactly what `heap_1` implies anyway. Each site carries a one-line comment saying why it is not
`vTaskDelete`, because the "fix" back is obvious and wrong.

### 2.29 `Imu_Task` never blocked, and starved every priority-1 task **HW**

The loop was a bare `while (true)` around `Imu_Update` with no data-ready wait and no delay. At
`TASK_PRIORITY_SENSOR_IMU` (2) that permanently starved everything at priority 1: the log drain,
`Mag_Task` and `Baro_Task`. Measured under the SIL, the task free-ran at **6,558 Hz** — sixteen
times what the control loop consumes — and the boot log died immediately after "Heap Free Size".

Proof it was starvation rather than a dead logger: remove the IMU model so `Imu_Task` exits at
init, and the same image emits twelve further lines including the mag and baro failures.

Same class as §2.4 and §2.6, and the last of the three. The sim backends blocked on a semaphore, so
only the real driver path was affected — which is why the SIL never saw it until the IMU moved onto
the real driver.

Fixed: `vTaskDelay (pdMS_TO_TICKS (2))`, capping the loop at 500 Hz.

### 2.30 `IMUUpdatefromPolling` busy-waited up to 1000 times per update **HW**

The data-ready poll looped up to 1000 times, `DelayMicroseconds (10)` between attempts. That is a
spin on DWT CYCCNT with no yield point, so **a single miss held the CPU for >10 ms** — five whole
periods of the capped task — at priority 2.

It cost transactions as well as time: against a part that never asserted data-ready, the driver
issued **1000 SPI transactions per update instead of 3**, 60 % of all measured SPI traffic.

Fixed: one STATUS read, then whichever of accel and gyro it reports ready. "No new sample" is
normal above the configured ODR, so it returns `eSTATUS_BUSY` and the task publishes nothing.
The accel-configuration recovery that used to hang off the poll timeout is kept, now triggered by
`DATA_MISS_LIMIT` (50) consecutive misses — 100 ms at 500 Hz — so an ordinary rate mismatch no
longer looks like a part that has stopped.

### 2.31 `Delay()` busy-waited, starving everything below the caller **HW**

`Delay (ms)` was `DelayMicroseconds (ms * 1000)`, i.e. a CYCCNT spin. `IMUSoftReset` alone spends
~2.5 s in it (20 × `Delay (100)` polling the feature engine), all of it holding the core at the
caller's priority. With `Imu_Task` retrying a failed init (§2.32, since reverted) that became
continuous.

Fixed: `Delay` yields when there is a scheduler to yield to —
`xTaskGetSchedulerState () == taskSCHEDULER_RUNNING && !xPortIsInsideInterrupt ()` — and spins
otherwise, so pre-scheduler and ISR callers are unchanged. `+1` tick, because `vTaskDelay` counts
whole ticks from the *next* boundary and every caller is satisfying a settling time, which is a
minimum.

Measured on the retry case: 40 virtual seconds took **>2 min of host time before, 40.0 s after**,
cumulative load 1.000 with the core reported `Idle` instead of executing NOPs.

`DelayBusyWait (ms)` is the explicit spin, for waits the scheduler must not stretch. DShot arming
uses it — the ESC is timing that frame cadence — which keeps arming at exactly 350 ms.

### 2.32 `Imu_Task` gave up permanently if the part was not ready at boot **HW**

A single failed `Imu_Init` suspended the task forever, so a part still coming out of reset cost
the aircraft its IMU for the whole flight.

Fixed at the time: retry until it succeeds, `vTaskDelay (pdMS_TO_TICKS (20))` between attempts.

**Since REVERTED, deliberately — the behaviour is open again and tracked in §1.19.** The retry
loop was removed so that all three sensor tasks fail the same way (suspend) instead of the IMU
alone retrying. Do not read this entry as describing current code.

Two things it got wrong while it stood, worth keeping so they are not repeated:

- The parenthetical claiming the SIL exercised it ("under the SIL, a sample source not connected
  yet") was false. `IMU_Init_` reads the chip id, the soft-reset status and the config read-back,
  and the emulated part serves all of those from its register file whether or not the bridge has
  connected. Only `STATUS` is gated on a pushed sample, and init never reads it — so a green SIL
  run was never evidence the retry path worked.
- The retry cadence was ~2.7 s, not the 50 Hz the `vTaskDelay` suggests, for the reason below.

That change required §2.33 first — without it the retry loop caused the failure it exists to
prevent.

**The retry cadence was ~2.7 s, not the 50 Hz the delay suggests**: a failing `IMUSoftReset` spends
~2.5 s in `Delay()` waiting on the feature engine. Since §2.31 that wait no longer costs CPU, only
time, but shortening it is the only way to retry faster. Relevant again to anyone reinstating a
retry loop under §1.19.

### 2.33 `ImuDrv_Init` allocated a fresh context on every call **HW**

`pOutDriver->ctx = Allocate (sizeof (Bmi323_t))`, and `Allocate` is a bump allocator over a fixed
pool with **no free**. Harmless while init ran once; with the §2.32 retry loop it consumed the
shared pool one context per attempt until `Allocate` returned NULL and init failed permanently —
the retry loop would have caused the outage it exists to prevent.

Fixed: one static instance. There is one IMU. Verified across 15 retries against an absent part:
the pool counter held at 20 bytes.


## 3. Fixed — host tooling and flight model

### 3.1 The bridge could never arm

It raised AUX1 **and** throttle to hover (1500 µs) in the same frame, but
[mission.c:14](Firmware/tasks/mission/mission.c#L14) requires `ARM_THROTTLE_MAX = 1100`. Every
arm request was refused.

**The firmware is correct** — that interlock stops the aircraft leaping off the bench the instant
it arms, and every mainstream flight stack has it. The bridge now mimics a pilot: throttle low
with AUX1 high, wait for `Telemetry.armed`, then hover.

### 3.2 `tiltrotor.xml` had never parsed

Line 14 sat inside an XML comment containing `--port` / `--dry-run`, and **`--` is illegal inside
an XML comment**. The model had therefore never loaded and the closed loop had never run. (Worth
knowing: this trap bites easily — writing CLI flags or `----` rules inside XML comments.)

### 3.3 The FDM diverged to NaN in 0.215 s

Three faults, isolated by stepping JSBSim standalone with **no FC attached**:

- thrust on the wrong axis — `propulsion/engine[N]/pitch-angle-rad` is **absolute and overrides**
  the thruster's `<orient>`, so `<orient>pitch=90</orient>` did nothing once the bridge wrote the
  property;
- thrust-to-weight ~12:1 at half throttle (2 kg airframe, 10 in prop, 700 W);
- the Ct table's last row was J = 1.2, and the solver extrapolated off the end.

Rescaled to the real airframe (0.85 kg, 7 in, 1000 KV assumed on 6S). But `FGPropeller` then
diverged for a *fourth* reason — the advance ratio ran to 1e8 within 0.05 s, regardless of thrust
direction, with RPM exceeding `maxrpm`.

**Replaced `FGPropeller` with `<external_reactions>`**: four body-axis forces scaled by
`cos(tilt)`/`sin(tilt)`. No advance ratio, no rpm ODE, and JSBSim clamps table lookups at their
endpoints. Verified: hover at exactly 50 % throttle, 2:1 at full, tilt rotates correctly, 13+ s
of powered closed-loop flight with attitude held at 0.0°.

`ROTOR_TMAX = 1.874 lbf` (850 g/rotor) is the one number to replace with thrust-stand data.

### 3.4 The gear geometry made any ground contact diverge

This was §4.1, "NaNs on hard ground impact". It was far worse than a 50 ft drop: **a 0.5 ft drop
NaN'd within a second**, so the model could not be set down at all.

The contacts did not straddle the CG. Both mains sat at `x = 0` — exactly the CG station — with
a single third contact 0.25 m away carrying an unbalanced moment. With `iyy = 0.008 kg*m2`, a few
newtons at that arm is ~1100 rad/s², and the model picked up **2.83 rad/s of pitch rate in one
2.5 ms step** on contact, then integrated straight to NaN. Two further traps: `<location>` inside
`<contact>` is the **structural** frame (+X aft, +Z up), the opposite sense in X and Z to the body
frame used everywhere else — so the contact named `TAIL` at `x = -0.25` was actually at the nose;
and the spring/damping were given in `N/M` and `N/M/SEC` and sized for a much heavier airframe.

Fixed by putting the mains 0.15 m ahead of the CG and the tailwheel 0.25 m behind it, and sizing
the gear for this 0.85 kg airframe in JSBSim's native `LBS/FT` (25 lbf/ft and 1.5 lbf/(ft/s) per
contact — ~10 mm static deflection, ~6 Hz heave, just past critical damping). Verified with no FC
attached:

| drop | result |
|---|---|
| 0.5 ft | settles, 6.4 g peak |
| 2 ft | settles, 20.8 g peak |
| 10 ft | settles, 54.4 g peak |
| 50 ft | settles, 120 g peak — no NaN |

All four settle to the same attitude (θ = +0.21°, φ = 0.00°) and compression (0.0233 ft). A full
throttle-driven takeoff → hover → descent → landing cycle also completes with no NaN.

### 3.5 Arming now waits for the attitude estimate to settle

This was §4.2. `bridge.py` gated arming on a 1.0 s fixed delay, which armed into an unconverged
estimate.

**Superseded by §2.13**: the interlock now lives in the mission task, where it protects real
hardware too. The bridge-side settle gate has been removed again and `bridge.py` is back to a
plain `--arm-delay` (2.0 s) before raising the switch, then waiting for the FC to report armed.

With §2.8 fixed the estimate converges in about 3 s, so the gate opens promptly instead of being
the tens-of-seconds wait the original note assumed.

### 3.6 Doc and tooling cleanups

- **`Firmware/CLAUDE.md`** (was §4.3): the umsg regeneration command pointed at
  `-d Firmware/msgs/umsg -o Vendor/umsg/umsg_lib`, which edits the wrong tree. Now documents
  `python Scripts/board.py gen` with the real underlying invocation, and notes that the umsg
  *core* is not generated. The layer table's `inc/…` / `src/…` paths were replaced with the actual
  `tasks/**` layout, and a trailing claim that sensor wrappers have no `_Init` was dropped.
- **`umsg_gen.py`** (was §4.4): removed the `core_path` variable that was computed and never used,
  along with the now-unused `shutil` import. The duplication it hinted at remains — see §4.9.
- **`Vendor/umsg`** (was §4.5): dropped the `test_peek` CMake target and the `peek` references in
  `README.md` and `umsg_graph.py`. This does not make the standalone build work — see §4.8.
- **Build artifacts** (was §4.6): single- and dual-core images no longer share a directory.
  `--single-core` builds go to `Build/<board>/<config>-single-core/`, so a stale `cm4.elf` can no
  longer sit beside a single-core `cm7.elf`. `renode` looks only in the single-core tree and
  `flash` only in the dual-core one, and `flash` now checks both ELFs exist before invoking
  openocd instead of passing missing paths to it.

**These touch the `Vendor/umsg` submodule** (`CMakeLists.txt`, `README.md`, `umsg_gen.py`,
`umsg_graph.py`, and `umsg_gen/templates/msg.{c,h}.j2` - see §3.10). They are uncommitted in the
submodule working tree and need their own commit there plus a pointer bump here.

### 3.7 Heap headroom is no longer a constraint

This was §1.6, written when `configTOTAL_HEAP_SIZE` was 15360 with 968 bytes free. CM7 has since
been doubled to **32768**.

Measured rather than assumed — `xPortGetFreeHeapSize()` surfaced through sim telemetry:
**17864 bytes free**, i.e. 14904 used, 54.5 % headroom. The used figure is 512 B above the
previous 14392, which is exactly the `STACK_MISSION` bump in §2.14 and cross-checks the reading.
`heap_1` never reclaims, so this is the steady-state number once every task and queue is created.

The old warning that "roughly two more `nav_state` subscriptions would exhaust it" no longer
applies — there is room for dozens. CM4 is untouched at 15360.

### 3.8 The FDM rotors now sit above the CG, so pitch is controllable

This was §4.7, found while deriving the §2.11 mixing signs. Both rotors were declared at
structural `(0, ±0.25, 0)` — coincident with the CG in X *and* Z — so neither thrust component
had a moment arm about Y and the pitch axis had **zero** authority: 0.3 rad of collective tilt
produced exactly 0.00000 rad/s of pitch rate.

Rotors raised to structural `z = +0.06` (60 mm above the CG; structural +Z is up). Measured with
no FC attached:

| input | before | after |
|---|---|---|
| collective +0.3 rad | q = 0.00000 | q = **−8.92** rad/s (nose down) |
| collective −0.3 rad | q = 0.00000 | q = **+8.92** rad/s (nose up) |
| differential ±0.3 rad | r = +21.8 | r = +21.8, q = 0.00000 |

Nose-down for rotors-forward is the sign the mixer's −1.0 pitch weight assumes, and the axes are
cleanly decoupled. The §3.4 gear regression still passes (0.5 ft and 50 ft drops settle to
θ = +0.208°).

**60 mm is an estimate, not a measurement** — the repo has no airframe CAD, only PCB designs.
Pitch response scales linearly with it, so replace it with the real rotor-plane height before
trusting SIL pitch tuning. Flagged inline in `tiltrotor.xml` alongside `ROTOR_TMAX_LBS`.

### 3.9 umsg: the dead standalone build now says so, and the core has one source of truth

This was §4.8 and §4.9.

**§4.8** — the missing `tests/`, `msg_defs/` and `FreeRTOS-Kernel/` were not lost, they were
deliberately deleted (`4d93776` "Removed peek functionality and unneeded folders files.",
`3800750` "removed freertos submodule"). So the top-level `CMakeLists.txt` is vestigial. Rather
than delete someone else's vendored file, it now checks for those directories **before**
`project()` and fails with an explanation naming what is missing and pointing consumers at the
core sources. Previously it failed deep inside `add_subdirectory` after a silent no-op
generation; the `execute_process` also gained `COMMAND_ERROR_IS_FATAL ANY`.

**§4.9** — `umsg_gen.py` regained the `core_path` it used to compute and discard, now behind an
opt-in `--with-core` that copies the packaged core into the output. That makes
`umsg_gen/umsg_gen/core/` the single source of truth and `umsg_lib/core/` a refreshable copy
instead of a hand-maintained twin:

```
python umsg_gen/umsg_gen/umsg_gen.py -d msg_defs -o umsg_lib --with-core
```

Verified the refresh reproduces the committed `umsg_lib/core` byte-for-byte, confirming it is
purely derived. Opt-in matters: unconditional copying would drop an unused core into
`Firmware/msgs/umsg/`, where `GENERATED_MSG_SRCS` globs `*.c` recursively and would try to
compile `port_posix.c` into the ARM image. Confirmed `board.py gen` produces no core directory
and no change to `Firmware/msgs/`. Documented in the submodule README.

---

### 3.10 Generated umsg files carried a timestamp, so every regen looked like a change

`umsg_gen.py` rendered `// Generated with umsg_gen on {{ date }}` into every `.c` and `.h`, so
running `board.py gen` on a different day rewrote all 14 generated files with a one-line diff
and nothing else. That is pure noise in `git status`, and it fires on any `build -f c` (clean
implies regen), which is how it kept reappearing mid-review.

The banner now reads `// Generated with umsg_gen - do not edit by hand` - it still marks the
files as generated, it just no longer encodes *when*. The `date=` kwarg and the `datetime`
import are gone with it.

Verified by generating twice in a row and comparing checksums: byte-identical. The one-time
banner change itself is a 14-file diff that needs committing once.

### 3.11 The pre-arm hold fed the FC a free-falling accelerometer

`--hold-until-armed` freezes the FDM at its initial condition, but the bridge still read
`accelerations/{u,v,w}dot` from the un-stepped model. A model that has never been stepped has not
resolved its gear reaction, so it reports the only force it knows about — gravity — and `wdot`
comes back as a full −32.2 ft/s². `synthesize_sensors()`'s `g_body - a_lin` then cancels to
**exactly (0, 0, 0)**.

Nothing physical reads 0 g sitting on its gear. The FC was told it was in free fall for the entire
pre-arm window and faithfully integrated it: the altitude estimate reached **−7.7 m/s and −2.0 m**
before the first real FDM step rescued it. The comment above the code claimed "the FC still gets a
valid level-and-still sample stream", which is precisely what it did not.

It also handed the Madgwick filter a zero-length accel vector for that whole window, which is part
of why attitude converges over the first seconds *after* arming rather than starting converged.

**Fixed**: a held vehicle is not accelerating, by definition, so linear acceleration is forced to
zero during the hold and the accelerometer reads its 1 g. Altitude error against FDM truth over a
`hover` run went from **2.63 m / 8.24 m/s to 0.01 m / 0.05 m/s**.

### 3.12 The FDM's tilt servos moved instantly

`fcs/tilt-cmd-rad[N]` fed the rotor force functions straight through `<cos>`/`<sin>` with no
`<actuator>`, so a commanded tilt was applied in the frame it arrived — a full ±π/2 reversal inside
one 2.5 ms step. Item 2 of the attack order had this as fidelity work ("flatters any tune"); §1.14
showed it was worse than that, because the integrator answered those step discontinuities with
6–13× the torque the airframe possesses.

Fixed with a `<channel name="actuators">` carrying one `<actuator>` per tilt servo, and the force
functions now read `fcs/tilt-pos-rad[N]` — the servo's *position* — never the command.

`rate_limit` 10.5 rad/s is 0.10 s per 60°, a mid-range hobby digital servo; `lag` 50 adds the
electrical delay on top. Measured standalone with no FC attached: slew comes out at exactly
10.500 rad/s, a 90° step takes 150 ms, and under ±π/2 commanded *every frame* the per-frame slew is
clamped to 10.5 rad/s against **1257 rad/s** unlimited — a 120× reduction in the discontinuity the
integrator sees.

Both numbers are representative, not measured. Replace them with bench figures for the actual servo
before trusting a tune taken against this model.

### 3.13 Every JSBSim sensor error model was set to zero

Was §4.15. `SensorBaro.xml` and `SensorGps.xml` were vendored with `<lag>`, `<noise>`,
`<drift_rate>`, `<bias>` and `<delay>` all zero, so the SIL fed **perfect** sensors — which meant
the vertical complementary filter, whose entire job is rejecting baro noise and absorbing accel
bias, was never actually exercised. Its 0.01 m result measured roundoff, and *any*
`CFG_ALT_FILTER_K_*` would have passed.

Now enabled:

| term | value | why |
|---|---|---|
| baro noise | 2.0 Pa (~17 cm) | the term the filter exists to reject |
| baro bias | 20 Pa (~1.7 m) | deliberately large and harmless — the boot datum cancels it exactly, so tracking truth *proves* the datum works |
| baro drift | 0.05 Pa/s | the term that does **not** cancel; slow weather-like wander |
| baro temp noise | 0.1 °C | now scales the whole altitude, via §2.27 |
| GPS lat/lon | 2.5e-7 rad (~1.6 m) | consumer-receiver horizontal error |
| GPS alt | 3.0 m | nothing consumes it yet; for whoever blends it later |
| GPS velocity | 0.1 m/s | reaches `vel_ned[0..1]` through speed/course |

Re-measured gates, and this is the payoff — the numbers are now informative:

| | before | after |
|---|---|---|
| altitude | 0.01 m | **0.46 m** |
| climb rate | 0.05 m/s | **0.26 m/s** |
| position | 0.01 m | 0.00 m |

`NAV_ALT_EPS_M` raised 0.5 → 1.5 accordingly. Baro temperature `<bias>` is deliberately left at
zero: modelling die self-heating there is how §2.27's failure mode becomes visible, and that is a
separate experiment rather than part of turning noise on.

### 3.14 `nmea.py` died on a NaN coordinate, hiding the real failure

Was §4.16. A diverged FDM hands back NaN, and `nmea._coord()`'s `int(degrees)` raised
`ValueError`, killing the bridge mid-run so `Checks.report()` never printed. The verdict was a
traceback rather than a result, and §1.14 stayed unexplained longer than it needed to because the
actual failure was hidden behind a crash in the sensor encoder.

Fixed by going quiet instead: non-finite FDM position stops GPS transmission for the rest of the
run, warns once, and counts the ticks. A receiver that stops producing fixes is something the FC
already handles (`GPS_FIX_TIMEOUT_US` → `Gps_HasFix`), and the run now reaches its summary. The
tick count is reported as its own failure line, distinct from FC-side NaN.

### 3.15 The nav position gate measured GPS noise instead of the projection

Found by turning on §3.13, and the reason that entry was worth doing beyond the tuning question.

The gate compared the FC's `nav_pos_ned` against a fresh read of `sensor/gps/lat_rad`. With noise
off, that property was slowly varying and the comparison looked exact. With noise on it is a new
random draw every tick, so sampling it at telemetry time differenced **two independent ~1.6 m
draws** and reported 8.8 m of "estimator error" on a hovering vehicle. Comparing against the
bridge's most recent transmission instead was no better — the FC's telemetry necessarily lags it by
a poll and a decode.

Fixed by comparing the FC's projection against the FC's **own echoed fix from the same telemetry
frame**, differentially against the first sample. Same frame means zero skew by construction, and
differential means the FC's choice of origin cancels rather than having to be guessed from outside.
Error dropped to 0.00 m, and what is being measured is now exactly what the gate always claimed:
whether the flat-earth projection was applied to the right fields, with the right signs and scale.

One frame of skew is still tolerated, because it is structural: `SimTelemetry_Task` reads
`umsg_nav_state_t` before it drains `umsg_sensors_gps_t`, so a fix landing between those two reads
pairs an old position with a new echo. Bounded at exactly one fix — hence two candidates, not a
deque; allowing more would let the gate find a match for anything.

---

## 4. Open — sim and tooling

### 4.10 `umsg_lib`'s committed example messages still reference `peek` **SIL**

`umsg_lib/inc/umsg_*.h` and `umsg_lib/src/*.c` (battery, control, example, sensors, test,
unused) are committed *generated* output from before `peek` was removed. They are regenerated
from `msg_defs/`, which was deleted (§3.9), so they cannot be refreshed as things stand — either
restore a set of example definitions or delete the stale output.

Harmless to Flapjack, which compiles only `umsg_lib/core/` plus its own `Firmware/msgs/umsg/*.c`.

### 4.11 The host unit-test suite is stale **SIL** — **superseded by §2.24**

*Retired; the suite was rebuilt. See §2.24 and `Tests/README.md`.*

What still stands: `Tests/` is the natural home for replaying a module against controlled inputs,
the technique that proved the estimator correct in §2.8 and would have caught the D-term sign
error in §2.15. `filter.c` and `pid.c` do have targets now (`test_altitude.c`, `test_align.c`);
`pid.c` itself still has none.

### 4.12 RC failsafe detects but does not act **HW**

§2.22 added detection; what to *do* about a lost link is undecided and deliberately unimplemented.
The three candidates: report only (what ships today), force a disarm posture (~10 lines reusing the
existing AUX hysteresis, but it drops an airborne multirotor), or a real controlled-descent mode.
The third is the right answer and is **blocked** on the missing altitude estimate — `nav.c`
publishes `alt = 0.0f`, `Baro_Update()` is a stub, and no task calls `SensorGps_Update()`.

### 4.13 The SIL is not yet reproducible enough for a golden-trajectory gate **SIL**

`sim --plan` gates on physics invariants (no NaN, motors inside [0,1] and not pinned, servos inside
±π/2, actually armed), which is the right first step per §1.13's warning about attitude
repeatability. Asserting on a recorded trajectory needs tighter run-to-run behaviour than wall-clock
pacing gives. Lock-step bridging - send `SensorData` for step N, wait for that step's
`MotorCmd`/`ServoCmd`, then advance - is bridge-only and needs no firmware change; SilResearch
§3 already flagged it as the reproducibility play. Note it cannot be made bit-exact while
`Rx_Task`/`Rc_Task`/`SimTelemetry_Task` run on `vTaskDelay` virtual time, so budget ±1 RC frame of
jitter rather than chasing equality.

### 4.14 Two timeout paths are unreachable in both test environments **SIL**

The CRSF inter-frame timeout (§2.21 C5) and the RC link timeout (§2.22) are both time-gated.
`GetMicroseconds()` returns 0 under `UNIT_TEST`, so neither is reachable from a host test; and
Renode does not pace UART bytes at the baud rate, so the CRSF one is unreachable in the SIL too.
The RC link timeout *is* covered in the SIL (`sim --rc-stop`). Covering the CRSF one needs a time
seam that does not exist.

### 4.15 Every JSBSim sensor error model is set to zero **SIL** — **fixed, see §3.13**

*Retired. §3.13 has the enabled terms, the reasoning behind each value, and the re-measured gates.*

Still true, and the reason this entry is worth keeping as a pointer: JSBSim's *magnetometer* is not
merely noiseless but **wrong** (|B| = 32 µT against ~48 µT real, Z sign inverted), so it is
deliberately not used — SilResearch §5.4. And there is still **no IMU noise model at all**, which
is why the D-term filter cutoff had to be chosen on a host replica rather than in the SIL.

### 4.16 `nmea.py` dies on a NaN coordinate **SIL** — **fixed, see §3.14**

*Retired. See §3.14.*

### 4.17 Tilt reaches the FDM unclamped; throttle does not **SIL**

`bridge.py` clamps throttle, `fdm[PROP_THROTTLE[i]] = max(0.0, min(1.0, thr))`, but writes tilt
straight through, `fdm[PROP_TILT[i]] = a`. The asymmetry is deliberate per the comment ("passed
through unmodified") and it did **not** cause §1.14 — the FC's angles stayed inside its own ±π/2
servo limits throughout. But there is no guard if one ever does not, and §2.5 is precedent for the
mixer emitting servo commands ~20× out of range.

Clamping to ±π/2 would match the invariant `Checks.report()` already asserts on.

### 4.18 The nav position gate is only as strong as the plan's trajectory **SIL**

The estimator gate in `bridge.py` now checks the FC's `nav_pos_ned` against its own echoed fix
differentially (§3.15) and passes at 0.00 m on both `hover` and `yaw_step`. The *check* is exact —
but none of the three plans translates far, so north and east are both near zero for the whole run
and **the gate still cannot catch them being swapped or negated**. It does catch a stuck or absent
estimate, and the validity-bit check catches the estimator never coming up.

A plan that flies a leg out and back would close it. Same class of gap as §4.13: the assertion is
sound, the excitation is not.

### 4.19 `sim_pb2.py` round-trips through git with different bytes than the generator writes **SIL**

The generator writes LF; a git checkout of the same file writes CRLF — 44 lines, 44 bytes,
confirmed at byte level. `git diff` normalises and shows nothing, so the file looks clean while its
checksum has moved. `board.py gen` is otherwise deterministic: `sim.pb.h` and `sim.pb.c` came back
byte-identical across four consecutive runs, and only `sim_pb2.py` alternates between two hashes
depending on whether the generator or git wrote it last.

Harmless today, and invisible to git by construction. It would matter to anything that checksums or
caches generated artifacts, and it cost time this round chasing a hash change that turned out not
to be a content change. Same family as §3.10; a `.gitattributes` entry pinning `*_pb2.py` to LF
would settle it.

---

## Suggested order of attack

**The full stack flies closed-loop in the SIL, and can log while doing it.** Every defect on the
original open lists is closed; §§2 and 3 are that record. What is left is measurement and fidelity
work, not bug fixing.

Open items are §1.9, §1.10–§1.14, §1.16 and §4.10–§4.19. In priority order:

1. **Measure the airframe.** Rotor-plane height above the CG (an estimated 60 mm) and
   `ROTOR_TMAX_LBS` (1.874 lbf assumed). Pitch authority scales linearly with the first, so the
   pitch gain cannot be responsibly finished until it is known (§1.9) — and `roll_pitch`
   diverging (§1.14) is blocked on exactly the same measurement.
2. **Bench-measure the tilt servos** and replace §3.12's representative rate limit and lag. The
   D-term filter cutoff was chosen at 60 Hz specifically because neither the FDM nor the host
   replica could price phase lag (ControlResearch §2.4); real figures reopen that choice.
3. **Add a gyro/accel noise model to the SIL.** The one measurement the rig cannot make today.
   It would move the D-term cutoff question, and estimator robustness generally, inside the SIL.
4. **§4.12** — decide what RC failsafe should *do*. Detection ships; the action does not.
5. **§4.13 / §4.18** — lock-step bridging and a plan that flies a leg out and back, which
   together are what a golden-trajectory gate needs.
6. **§4.10** — the stale `peek`-era example messages in `umsg_lib`.

Then: bench tune, then first flight.

## Verification note

Measured rather than inspected, across both rounds:

- **§2.8** — host replay of `filter.c` at constant `dt` (300 s, stable to 0.01°), then temporary
  `dt` instrumentation in `Nav_Update` before and after, then a 150 s `--dry-run` under Renode at
  50/100/200/400 Hz. An unmodified HEAD build served as the A/B control.
- **§2.13/§2.14** — SIL run confirming the FC holds `armed=False` through convergence and the
  settle window, then arms itself with the switch held; the stack overflow was isolated by the
  telemetry stall it caused and confirmed by the heap delta in §3.7.
- **§3.4/§3.8** — JSBSim stepped standalone with no FC attached: 0.5/2/10/50 ft drops, a full
  takeoff-to-landing cycle, and collective/differential tilt commanded directly.
- **§3.7** — `xPortGetFreeHeapSize()` read out over sim telemetry.
- **§2.15** — gains swept against the flight model with a host replica of the exact control law
  (guidance → `Pid_UpdateAxis` → `Mixer_Mix*` → `UsToAngleRad`), at 100/200/400 Hz, then
  confirmed with the real firmware under Renode: 120 s closed-loop flight, zero NaN. Each of the
  three faults was isolated by a separate experiment rather than inferred — the replica stayed
  stable under dt *jitter*, which is what narrowed the last one down to dt being exactly zero.
  The replica and its sweeps live in `Build/ratetune.py` and `Build/sweep2.py` — gitignored and
  disposable, so `build -f c` wipes them. Promote them to `Scripts/` if tuning that way is worth
  keeping.
- **§2.16** — `uxTaskGetStackHighWaterMark()` sampled per task over a SIL run.
- **§1.14/§3.11** — a temporary per-tick capture of both sides of the link (FC servo/motor
  commands, FDM attitude/rates/altitude, and the FC's own reported euler) written to CSV for the
  whole run, then read back offline. §1.14 was confirmed pre-existing by `git stash` + rebuild on
  unmodified HEAD; §3.11's fix was confirmed by the altitude error over a `hover` run dropping
  from 2.63 m to 0.01 m. The instrumentation was removed afterwards.
- **§2.27** — the ISA formula evaluated against JSBSim's own atmosphere at 0/100/500/1000/2000 m
  datum elevations, and pinned in `Tests/UnitTest/test_altitude.c`.
- **§4.19** — `md5sum` across four consecutive `board.py gen` runs, then a `git stash push`/`pop`
  round trip on the single file, with the CRLF confirmed by `od -c`.
- **§2.9/§2.10/§2.11/§2.12** — build-verified on both boards and both driver profiles. The tilt
  mix signs are derived from the moment arms, and both the pitch sign (§3.8) and the whole
  allocation (§2.15) are now confirmed in closed-loop flight.

Two things still not independently reviewed, carried from the first round: the mixer and
`peek`-removal diffs have not had a line-by-line read, particularly `control.c`'s nav caching.
`mission.c`'s arming path has since been substantially rewritten (§2.13) and exercised in the
SIL, so that half is covered.
