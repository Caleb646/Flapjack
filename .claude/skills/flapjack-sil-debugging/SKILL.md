---
name: flapjack-sil-debugging
description: How to find a GNC bug in the Flapjack SIL (Renode CM7 + JSBSim). Use when the attitude estimate drifts, jumps or goes NaN; the vehicle departs the instant it arms; the sim link wedges or telemetry stops; a task hangs with no fault; or PID/mixer gains need sweeping. Covers getting observability out of a `-D sim` build (logs, telemetry and shell all share one UART), isolating firmware faults from flight-model faults, and tuning gains without a Renode round trip. For plain command syntax use the flapjack-tooling skill instead.
---

# Debugging the Flapjack GNC stack in the SIL

Command syntax lives in **flapjack-tooling**. This is about method: how to see anything, and
how to tell which layer is lying to you.

## Two facts that shape every investigation

1. **A `-D sim` build logs normally.** `LOG_*` and the shell both work — the debug UART carries
   ASCII log text interleaved with binary sim frames, and `bridge.py` prints the text to stdout
   as it arrives. So reach for a log line first; the telemetry-hijack trick below is now only
   for values you need at full loop rate, not for basic visibility. Note `%f` prints nothing
   (KnownIssues §1.12) — scale to an integer. If logs are missing, check `--log-level`, which
   compiles them out.
2. **In `--dry-run`, every sensor sample is byte-identical.** The bridge synthesises a fixed
   20° roll with zero gyro rates. So if the estimate moves, the cause is *either* `dt` *or*
   corrupted data — nothing else can. This collapses the search space enormously; reach for
   `--dry-run` before the full FDM whenever the symptom allows it.

## Getting observability: hijack the telemetry fields

This is the highest-value technique here and it is not obvious. Temporarily repurpose the three
`euler` floats (and `imu_count`, a `uint32`) to carry whatever you need to see, in
`Firmware/tasks/sim/sim_telemetry.c`:

```c
/* TEMPORARY DIAGNOSTIC - remove. euler[] carries dt stats instead of attitude. */
extern volatile float    g_DiagMaxDt;
extern volatile uint32_t g_DiagDtSpikes;
float diag[3] = { g_DiagMaxDt, (float)g_DiagDtSpikes, 0.0f };
SimLink_SendTelemetry (diag, mission.armed != 0U, SimLink_GetSensorCount ());
```

The bridge prints them as `[FC] euler(deg)= ...`, so no host-side change is needed. Found this
way, in one session: a `dt` of **4295 s** (`2^32/1e6`, an unsigned underflow), free-heap bytes,
and every task's stack high-water mark.

Discipline that matters:

- **Back up the file first** (`cp Firmware/tasks/sim/sim_telemetry.c Build/`) and restore it when
  done; confirm with `git diff --stat` that the revert is clean.
- **Keep instrumentation stack-cheap.** A `TaskStatus_t status[16]` local (640 B) overflowed the
  telemetry task's own stack. Use `static` for anything large.
- `imu_count` can smuggle four packed chars if you need to label rows:
  `tag |= ((uint32_t)(uint8_t)name[k]) << (8U * k);`
- Telemetry runs at 50 Hz but the bridge only *prints* every 0.5 s. If you are cycling through
  a list of values, hold each one for ~1 s or you will sample it once and miss the rest.

## Isolation ladder — go down it in order

Each rung removes a layer. Do not skip to the full SIL.

**1. Replay one module on the host, with inputs you control.** Proved `filter.c` was correct by
driving it at a *constant* `dt` with the exact dry-run inputs: it converged to 20.00° and held
for 300 s. That single result redirected the whole investigation from the estimator to the
timebase. The home for this is `Tests/UnitTest/` + `Tests/CMakeLists.txt`.

**Note on the state of that harness:** it was rebuilt around `test_crsf.c` and now compiles only
`crsf.c` against the host. The previous tests (`test_filter.c` and the rest) were removed — they
had rotted past the point of being runnable: every one of them included `"unity/unity.h"` against
an include path where it could not resolve, and `spi.c` / `uart.c` no longer compile against
`stubs/hal_stub.c` at all (missing `LL_SPI_*` and `__HAL_UART_*`). So replaying `filter.c` or
`pid.c` on the host is still the right first rung, but you will have to add the target back to
`Tests/CMakeLists.txt` (pattern: a new `add_library(... OBJECT ...)` plus `add_test_exe`) rather
than uncommenting something. Unity is vendored at `Tests/unity/`; the build is
`cmake -S Tests -B Build/UnitTest -G "MinGW Makefiles"` then `ctest --test-dir Build/UnitTest`.

**2. Step JSBSim standalone, with no FC attached.** The only clean way to separate a flight-model
fault from a firmware fault:

```python
import jsbsim
fdm = jsbsim.FGFDMExec("Scripts/sim/jsbsim"); fdm.set_debug_level(0)
fdm.load_model("tiltrotor"); fdm.set_dt(1/400.0)
fdm["ic/h-agl-ft"] = 50.0; fdm.run_ic()
for i in range(4000):
    fdm.run()
```

Found the gear geometry that NaN'd on a 0.5 ft drop, and that collective tilt produced *exactly*
0.00000 rad/s of pitch because both rotors sat coincident with the CG.

**3. A/B against unmodified HEAD.** `git stash push -- Firmware/`, rebuild, re-run, `git stash
pop`. Cheap, and it settles "did I cause this?" definitively — the 162° attitude drift reproduced
identically on a clean HEAD build, which ruled out every local edit at once.

**4. Only then the full SIL.**

## Sweeping gains without a Renode round trip

A Renode run is ~2 minutes; a host sweep is seconds. Replicate the control law in Python against
the *same* JSBSim model — guidance → `Pid_UpdateAxis` → `Mixer_Mix*` → `UsToAngleRad` — and sweep
there, then confirm the chosen values on the real firmware. Mirror the firmware exactly, including
the clips (`CFG_PID_MIN/MAX_VALUE`, motor `[0,1]`, servo `[500,2500] us`).

**Always sweep across loop rates (100/200/400 Hz), not just 400.** The rate loop is IMU-paced, so
its period is not fixed. A D gain that looked fine at 400 Hz diverged at 200 and 100 — testing one
rate would have shipped it.

Useful plant numbers for this airframe: full differential thrust is ~9900 °/s² of roll against
`ixx = 0.012`, so the D term closes a nearly algebraic loop of gain `Kd·G`. Keep `Kd·G` well under
1 or it limit-cycles at the sample rate.

## Running the SIL cleanly

- **Restart Renode between runs.** A still-running emulator carries diverged filter state into the
  next run. An early measurement was contaminated exactly this way — the estimate had already
  blown up in a previous run and never recovered.
- **`PYTHONUNBUFFERED=1` and redirect to a file.** Piping the bridge buffers its output, so when
  `timeout` kills it you get *nothing*. `PYTHONUNBUFFERED=1 timeout 90 python Scripts/board.py sim
  ... > Build/run.log 2>&1`.
- **The FC does not consume the full `--rate`.** Under Renode the effective sample rate is well
  below the bridge's setting. Derive it from `imu#` deltas rather than assuming.
- **Arming takes ~6 s** from cold: the estimate needs ~3 s to converge and the firmware's own
  interlock (`tasks/mission/mission.c`) then wants 3 s of stillness. Telemetry showing
  `armed=False` early is normal, not a hang.
- **`armed=False` *forever* usually means you forgot `--rc-port`.** RC now arrives only as real
  CRSF on USART3; there is no sim-link fallback. With no RC the FC holds its boot defaults, sees
  no arm switch, and sits disarmed indefinitely while everything else looks healthy — telemetry
  flows, the estimate converges, `imu#` climbs. Telemetry carries `rc_link_up`, printed by the
  bridge as `rc=up` / `rc=DOWN`; check that before suspecting the arming interlock.
- **A lost RC link is reported, not acted on.** After `RX_LINK_TIMEOUT_US` (1 s) the FC logs
  `Rx: RC link lost` and `rc_link_up` goes false, but the channels keep their last values and the
  vehicle keeps flying them. That is deliberate (there is no failsafe behaviour yet), so a run
  that "keeps flying after the link dies" is not a bug.

## Traps

- **Non-finite values reach the FDM silently.** `clipf32` passes NaN through (all comparisons
  false) while `+inf` clamps to the upper bound. So a NaN poisons JSBSim, and an inf becomes full
  actuator deflection. Both trace back to a division by a zero `dt`.
- **`GetMicroseconds()` has 1 µs resolution** and the loop is paced by queued IMU samples, so two
  iterations can land in the same microsecond. Any `x / dt` needs a guard.
- **A silent hang is probably a stack overflow.** `configCHECK_FOR_STACK_OVERFLOW` is on and
  `vApplicationStackOverflowHook` (in `main.c`) parks the task name in `g_pOverflowedTaskName`
  before halting — read it in the debugger. The symptom is telemetry simply stopping.

## Anti-patterns

- **After two failed hypotheses, stop theorising and instrument.** Time was lost this way twice:
  first blaming sim-link saturation, then `dt` *jitter*. Measuring `dt` directly settled it in one
  run — it was `dt` being exactly *zero*, which neither theory predicted.
- **A rate-dependent symptom suggests a race, not a bandwidth limit.** Faster loops hit a race
  sooner, which looks exactly like a throughput ceiling. It is not.
- **Do not trust a clean result from a single loop rate or a single run.** Races and
  discretisation faults are both intermittent by nature.
