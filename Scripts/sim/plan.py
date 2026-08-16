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

    def __init__(self, segments, name="plan"):
        self.name = name
        self.segments = sorted(segments, key=lambda s: s.t)
        for a, b in zip(self.segments, self.segments[1:]):
            if a.t + a.dur > b.t + 1e-9:
                raise ValueError(f"{name}: segment at t={a.t} overruns the one at t={b.t}")
        self.duration = max((s.t + s.dur for s in self.segments), default=0.0)

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

    if not isinstance(doc, dict) or "plan" not in doc:
        raise ValueError(f"{path}: expected a top-level 'plan:' list")

    segments = [
        Segment(entry.get("t", 0.0), entry.get("dur", 0.0), entry.get("set"), entry.get("ramp"))
        for entry in doc["plan"]
    ]
    return Plan(segments, name=doc.get("name", path.stem))
