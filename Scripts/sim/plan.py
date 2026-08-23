"""Flight plans — scripted RC stick sequences for the SIL.

A plan is what a pilot would do: a list of timed segments, each holding or
ramping named channels. The bridge evaluates it once per tick and sends the
result as CRSF, so the firmware sees sticks moving exactly as it would in flight.

    plan:
      - {t: 0.0, dur: 2.0}                            # hold whatever is current
      - {t: 2.0, dur: 3.0, set:  {throttle: 1600}}    # step the throttle up
      - {t: 5.0, dur: 4.0, ramp: {yaw: 1700}}         # ease yaw over the segment
      - {t: 9.0, dur: 2.0, set:  {yaw: 1500}}

Channels are named per Firmware/drivers/rx/rx.h (roll, pitch, yaw, throttle,
aux1..aux12) and given in microseconds. `set` applies at the start of the
segment; `ramp` interpolates linearly from the value already held to the target
across the segment. Anything unmentioned holds.

PHASES, AND WHY THEY EXIST
--------------------------
A `plan:` is stick input and nothing else - it can be flown, but the only thing
asserted about it afterwards is that the vehicle did not depart or go NaN. A
`phases:` plan is the same stick input carrying its own PASS/FAIL criteria:

    phases:
      - name: climb_in_roll_right
        set:    {roll: 1700, throttle: 1800}   # 12 deg of bank, 0.6 m/s climb
        settle: 4.0                            # transient - NOT gated
        hold:   25.0                           # gates live on every sample here
        gates:
          roll_deg:     {target: cmd, tol: 2.0}
          climb_mps:    {target: cmd, tol: 0.15}
          pitch_deg:    {target: 0.0, tol: 3.0}
          yaw_rate_dps: {target: 0.0, tol: 5.0}

`set`/`ramp` are applied over `settle`, then the sticks are held still for
`hold`. Splitting the two is the whole point: "did it get there" and "did it
STAY there" are different questions, and only the second one is a stability
check. Phases run back to back in the order given.

`target: cmd` compares against what the sticks were ASKING for, via
`commanded()` below. `target: <number>` compares against a constant - which is
how an axis nobody commanded gets asserted, and that is the cross-coupling
check: a roll input that drags pitch with it fails `pitch_deg` even though the
vehicle is tracking roll perfectly.

Gate names are validated at load time (GATE_NAMES). A typo has to be an error
rather than a gate that silently never fires - a green gate that asserts nothing
is worse than no gate, because someone will trust it.

TWO THINGS THAT MAKE RUNS COMPARABLE
------------------------------------
**Arming is a barrier, not a timestamp.** The FC will not arm until its attitude
estimate has held still for 3 s (ARM_ATTITUDE_SETTLE_US in mission.c), and how
long that takes is not fixed - it was measured at ~24 s once. So the plan clock
does not start until the FC reports armed. Everything before that is the
pre-arm hold: throttle low, arm switch up, which is the gesture the FC's
interlock is waiting for.

**t is simulation time, not wall clock.** The bridge steps JSBSim exactly once
per tick with dt = 1/rate, so tick_count * dt is an exact, drift-free clock.
Pacing off time.perf_counter() would make the plan land differently under host
load.
"""

import math
from pathlib import Path

# Channel name -> index, mirroring RC_CHANNEL_IDX_* in drivers/rx/rx.h.
CHANNELS = {
    "roll": 0,
    "pitch": 1,
    "yaw": 2,
    "throttle": 3,
    "aux1": 4,
    "aux2": 5,
}
for _i in range(3, 13):
    CHANNELS[f"aux{_i}"] = _i + 3

RC_MIN, RC_MID, RC_MAX = 1000, 1500, 2000

# Sticks centred, throttle down, arm switch low - what the plan starts from.
NEUTRAL = [RC_MID] * 16
NEUTRAL[CHANNELS["throttle"]] = RC_MIN
NEUTRAL[CHANNELS["aux1"]] = RC_MIN


# Stick -> what the FC is actually being asked for, mirroring
# tasks/guidance/guidance.c. Roll and pitch command an ANGLE (centred = level),
# yaw commands a RATE, and throttle commands a CLIMB RATE about a held target.
#
# Keep these in step with Firmware/target/cfgs/cfg.h. They are duplicated rather
# than parsed out of the header because a gate that silently tracked a changed
# gain would assert nothing - if cfg.h moves and this does not, the gates should
# fail loudly and get updated deliberately.
ANGLE_MAX_DEG = 30.0            # CFG_ANGLE_MAX_DEG
YAW_RATE_MAX_DPS = 180.0        # GUIDANCE_MAX_RATE_RAD_S, pi rad/s
CLIMB_RATE_MPS = 1.0            # CFG_ALT_HOLD_CLIMB_RATE_MPS
CLIMB_STICK_DEADBAND = 0.05     # CFG_ALT_HOLD_STICK_DEADBAND, on the NORMALISED stick

# Gates the bridge knows how to evaluate. Split by what they measure:
#
#   per-sample - checked on every tick of a phase's hold window, so a gate that
#     holds on average but breaks briefly still fails. This is what makes
#     "maintains the climb AND the roll" a real assertion rather than an
#     end-state comparison.
#   summary    - one number per phase, measured across the whole hold. These are
#     the drift checks, and they are the only reason a long phase is worth
#     flying: nothing a 20 s hold does can express them.
#   estimate   - the FC's OWN attitude estimate against FDM truth, per sample.
#     Nothing else in the rig compares those two, which is what KnownIssues 1.16
#     is about: every other attitude gate is on truth precisely because an
#     estimate checked against itself passes while the vehicle is somewhere else.
#     These close that loop directly. On hardware there is no truth signal, so
#     this class of fault is only ever visible here.
GATE_SAMPLE = ("roll_deg", "pitch_deg", "yaw_rate_dps", "climb_mps")
GATE_ESTIMATE = ("est_roll_deg", "est_pitch_deg", "est_yaw_deg")
GATE_SUMMARY = ("alt_drift_m", "heading_drift_dps")
GATE_NAMES = GATE_SAMPLE + GATE_ESTIMATE + GATE_SUMMARY


def commanded(channels) -> dict:
    """What the sticks are ASKING for, in the units the gates compare against.

    This is the reference half of every `target: cmd` gate, and it is computed
    from the PLAN rather than read back from the FC on purpose: a reference
    derived from the thing under test cannot catch the thing under test being
    wrong (KnownIssues 1.16 makes the same argument for gating on FDM truth
    rather than on Telemetry.euler).
    """
    roll_n = (channels[CHANNELS["roll"]] - 1500.0) / 500.0
    pitch_n = (channels[CHANNELS["pitch"]] - 1500.0) / 500.0
    yaw_n = (channels[CHANNELS["yaw"]] - 1500.0) / 500.0
    throt_n = (channels[CHANNELS["throttle"]] - 1500.0) / 500.0

    # Bounded as a VECTOR, not per axis - guidance.c does the same, because
    # clipping each separately rotates the commanded direction and lets the two
    # sticks together reach 42 deg on the diagonal.
    roll_deg = roll_n * ANGLE_MAX_DEG
    pitch_deg = pitch_n * ANGLE_MAX_DEG
    mag = math.hypot(roll_deg, pitch_deg)
    if mag > ANGLE_MAX_DEG:
        scale = ANGLE_MAX_DEG / mag
        roll_deg *= scale
        pitch_deg *= scale

    climb = 0.0 if abs(throt_n) < CLIMB_STICK_DEADBAND else throt_n * CLIMB_RATE_MPS

    return {
        "roll_deg": roll_deg,
        "pitch_deg": pitch_deg,
        "yaw_rate_dps": yaw_n * YAW_RATE_MAX_DPS,
        "climb_mps": climb,
    }


class Gate:
    """One PASS/FAIL criterion on one quantity, over one phase's hold window."""

    __slots__ = ("name", "target", "tol")

    def __init__(self, name, spec):
        if name not in GATE_NAMES:
            raise ValueError(f"unknown gate {name!r}; known: {', '.join(GATE_NAMES)}")
        if not isinstance(spec, dict) or "tol" not in spec:
            raise ValueError(f"gate {name!r}: expected a mapping with a 'tol'")
        self.name = name
        self.tol = float(spec["tol"])
        if self.tol <= 0.0:
            raise ValueError(f"gate {name!r}: tol must be positive")

        target = spec.get("target", 0.0)
        if name in GATE_SUMMARY or name in GATE_ESTIMATE:
            # Neither kind has a target to name: a drift gate measures change
            # across the hold, and an estimate gate is compared against truth at
            # the same instant. Naming one is a mistake worth catching rather
            # than quietly ignoring.
            if "target" in spec:
                kind = "drift" if name in GATE_SUMMARY else "estimate"
                raise ValueError(f"gate {name!r}: {kind} gates take only 'tol', not 'target'")
            self.target = 0.0
        elif target == "cmd":
            self.target = "cmd"
        else:
            self.target = float(target)

    @property
    def is_summary(self) -> bool:
        return self.name in GATE_SUMMARY

    @property
    def is_estimate(self) -> bool:
        return self.name in GATE_ESTIMATE


class Phase:
    """A named stretch of a plan: reach a stick position, then hold it and measure.

    `settle` is the transient and is deliberately not gated - the vehicle is
    still getting there. `hold` is the measurement window.
    """

    __slots__ = ("name", "t0", "settle", "hold", "gates")

    def __init__(self, name, t0, settle, hold, gates):
        self.name = name
        self.t0 = float(t0)
        self.settle = float(settle)
        self.hold = float(hold)
        self.gates = gates
        if self.hold <= 0.0:
            raise ValueError(f"phase {name!r}: hold must be positive - "
                             f"a phase with no hold window asserts nothing")

    @property
    def hold_start(self) -> float:
        return self.t0 + self.settle

    @property
    def end(self) -> float:
        return self.t0 + self.settle + self.hold

    def holding(self, t: float) -> bool:
        return self.hold_start <= t < self.end


def _resolve(name):
    if name not in CHANNELS:
        raise ValueError(f"unknown channel {name!r}; known: {', '.join(sorted(CHANNELS))}")
    return CHANNELS[name]


class Segment:
    __slots__ = ("t", "dur", "set", "ramp")

    def __init__(self, t, dur, set_=None, ramp=None):
        self.t = float(t)
        self.dur = float(dur)
        self.set = {_resolve(k): int(v) for k, v in (set_ or {}).items()}
        self.ramp = {_resolve(k): int(v) for k, v in (ramp or {}).items()}


class Plan:
    """Timed RC stick sequence, evaluated at a point in plan time."""

    def __init__(self, segments, name="plan", phases=()):
        self.name = name
        self.segments = sorted(segments, key=lambda s: s.t)
        for a, b in zip(self.segments, self.segments[1:]):
            if a.t + a.dur > b.t + 1e-9:
                raise ValueError(f"{name}: segment at t={a.t} overruns the one at t={b.t}")
        self.duration = max((s.t + s.dur for s in self.segments), default=0.0)
        # Empty for a plain `plan:` - such a plan still flies, it just carries no
        # criteria of its own and is judged only by the run-wide invariants.
        self.phases = tuple(phases)

    def phase_at(self, t: float):
        """The phase covering plan time `t`, or None."""
        for ph in self.phases:
            if ph.t0 <= t < ph.end:
                return ph
        return None

    @property
    def total_time(self) -> float:
        return self.duration

    def channels_at(self, t: float) -> list:
        """Stick positions (us) at plan time `t`, in seconds since arming.

        Replayed from the start each call rather than kept as running state, so
        the result depends only on `t` - the same `t` always gives the same
        sticks, whatever order they were asked for.
        """
        ch = list(NEUTRAL)
        # An armed plan holds the arm switch up; dropping it would disarm.
        ch[CHANNELS["aux1"]] = RC_MAX

        for seg in self.segments:
            if t < seg.t:
                break
            for idx, value in seg.set.items():
                ch[idx] = value
            if not seg.ramp:
                continue
            # Fraction of this segment elapsed, clamped so a finished segment
            # leaves its ramp targets applied in full.
            frac = 1.0 if seg.dur <= 0 else min(1.0, (t - seg.t) / seg.dur)
            for idx, target in seg.ramp.items():
                start = ch[idx]
                ch[idx] = int(round(start + (target - start) * frac))

        return [max(RC_MIN, min(RC_MAX, v)) for v in ch]


def load(path) -> Plan:
    """Read a plan from YAML (or JSON, which is a subset of it)."""
    path = Path(path)
    text = path.read_text()
    try:
        import yaml
        doc = yaml.safe_load(text)
    except ImportError:
        import json
        doc = json.loads(text)

    if not isinstance(doc, dict) or ("plan" not in doc and "phases" not in doc):
        raise ValueError(f"{path}: expected a top-level 'plan:' or 'phases:' list")
    if "plan" in doc and "phases" in doc:
        raise ValueError(f"{path}: use 'plan:' or 'phases:', not both")

    name = doc.get("name", path.stem)
    if "plan" in doc:
        segments = [
            Segment(entry.get("t", 0.0), entry.get("dur", 0.0), entry.get("set"), entry.get("ramp"))
            for entry in doc["plan"]
        ]
        return Plan(segments, name=name)

    segments, phases, t = [], [], 0.0
    for i, entry in enumerate(doc["phases"]):
        pname = entry.get("name", f"phase{i}")
        settle = float(entry.get("settle", 0.0))
        hold = float(entry.get("hold", 0.0))
        gates = {k: Gate(k, v) for k, v in (entry.get("gates") or {}).items()}
        try:
            phases.append(Phase(pname, t, settle, hold, gates))
        except ValueError as exc:
            raise ValueError(f"{path}: {exc}") from None

        # Two segments, so the stick move lands inside `settle` and the hold
        # window is genuinely still. A zero-length settle still applies `set`,
        # because channels_at() applies a segment from its start instant.
        segments.append(Segment(t, settle, entry.get("set"), entry.get("ramp")))
        segments.append(Segment(t + settle, hold))
        t += settle + hold

    return Plan(segments, name=name, phases=phases)
