#!/usr/bin/env python3
"""JSBSim <-> Flapjack HIL bridge.

Pushes synthesized IMU, magnetometer and barometer samples into the emulated
parts on their own sockets, plays a pilot's arm-and-hover stick sequence as real
CRSF frames on the FC's separate RX UART, feeds real NMEA to its GPS UART, and
applies the servo/motor commands the FC sends back to a JSBSim tilt-rotor model.

No sensor value is handed to the firmware pre-decoded any more: the debug UART
carries only the FC's own output (actuators, telemetry, logs, shell).

Timing is free-running real-time: sensors are streamed at a fixed rate and the
FDM is stepped to wall-clock; the FC consumes the latest sample and replies
asynchronously.

Wire format (matches Firmware/drivers/serial/serial_link.c):
    [0xAA][0x55][msg_id][len][payload(protobuf)][crc8]
    crc8: poly 0x07, init 0x00, over (msg_id, len, payload).

The FC also interleaves plain ASCII debug logs on the same wire; anything that
is not a frame is log text and is echoed to stdout.

Run the sensor/link path WITHOUT a working FDM first:
    python bridge.py --port COM7 --dry-run
Then the full sim:
    python bridge.py --port COM7
"""

import argparse
import collections
import json
import math
import socket
import struct
import sys
import time
from pathlib import Path

from proto import flapjack_pb2, sim_pb2
from link.framing import frame, FrameParser
from link.serial_io import open_port
from sim import crsf
from sim import nmea
from sim import plan as planlib

# --- frame ids (keep in sync with sim_link.h / serial_link.h) ----------------
# 1 (SensorData), 2 (RcInput) and 7 (BaroData) are retired and must not be
# reused: every sensor now reaches the FC on a wire it has to parse or a part it
# has to read. The sim link carries only what the FC sends back, plus the shell.
MSG_SERVO = 3
MSG_MOTOR = 4
MSG_TELEMETRY = 5
MSG_SHELL_CMD = 6

# --- SIL port map ------------------------------------------------------------
# The SIL wiring never varies, so it is the default rather than six flags on
# every invocation. `board.py renode` puts USART1/3/2 on 4000/4001/4002, and the
# three emulated parts open 4010/4011/4012 themselves from the `port:` properties
# in Scripts/renode/flapjack_h7_cm7.repl. Keep this block in step with that file.
SIL_HOST = "localhost"
SIL_LINK_PORT = 4000
SIL_RC_PORT = 4001
SIL_GPS_PORT = 4002
SIL_IMU_PORT = 4010
SIL_MAG_PORT = 4011
SIL_BARO_PORT = 4012

# Spellings that turn a defaulted wire back OFF, so "run without GPS" and "run
# without RC" stay reachable - they are real test cases (a receiver that never
# appears, a board with no fix), and defaulting them on would have quietly
# removed them.
PORT_OFF = {"none", "off", "no", "false", "-", ""}


def sil_url(port: int) -> str:
    return f"socket://{SIL_HOST}:{port}"


def resolve_ports(args):
    """Fill in the SIL sockets for every wire the caller did not name.

    Only when the FC's own link is a socket, i.e. when this is a SIL run. Against
    a real board the companion wires are separate physical ports that nobody can
    guess, so they stay off unless named - which is also what keeps an existing
    hardware command line behaving exactly as it did.
    """
    if args.port is None:
        args.port = sil_url(SIL_LINK_PORT)
    is_sil = args.port.startswith("socket://")

    for attr, port in (("rc_port", SIL_RC_PORT), ("gps_port", SIL_GPS_PORT),
                       ("imu_port", SIL_IMU_PORT), ("mag_port", SIL_MAG_PORT),
                       ("baro_port", SIL_BARO_PORT)):
        value = getattr(args, attr)
        if value is not None and value.strip().lower() in PORT_OFF:
            setattr(args, attr, None)
        elif value is None and is_sil:
            setattr(args, attr, sil_url(port))
    return args


# --- RC channel layout (matches drivers/rx/rx.h) -----------------------------
RC_MIN, RC_MID, RC_MAX = 1000, 1500, 2000
CH_ROLL, CH_PITCH, CH_YAW, CH_THROTTLE, CH_AUX1 = 0, 1, 2, 3, 4

# --- JSBSim property map (adjust to match the FDM) ---------------------------
# These are the single place to tweak if the model uses different names.
PROP_PHI = "attitude/phi-rad"
PROP_THETA = "attitude/theta-rad"
PROP_PSI = "attitude/psi-rad"
PROP_P = "velocities/p-rad_sec"
PROP_Q = "velocities/q-rad_sec"
PROP_R = "velocities/r-rad_sec"
PROP_UDOT = "accelerations/udot-ft_sec2"
PROP_VDOT = "accelerations/vdot-ft_sec2"
PROP_WDOT = "accelerations/wdot-ft_sec2"
# Barometer, from JSBSim's own SensorBaro system (vendored into
# jsbsim/systems/). The _true_ variants are the uncorrupted values; switch to
# these post-error ones by turning noise/bias/lag on in that XML, not here.
PROP_BARO_PA = "sensor/baro/presStatic_Pa"
PROP_BARO_TEMP_C = "sensor/baro/temp_C"
# GPS, from JSBSim's SensorGps system. lat/long are GEODETIC - position/lat-gc-*
# is geocentric and differs by ~0.19 deg (about 20 km) at mid latitudes.
PROP_GPS_LAT_RAD = "sensor/gps/lat_rad"
PROP_GPS_LON_RAD = "sensor/gps/long_rad"
PROP_GPS_ALT_M = "sensor/gps/alt_m"
PROP_GPS_VN = "sensor/gps/vNorth_mps"
PROP_GPS_VE = "sensor/gps/vEast_mps"
# Inputs we drive: throttle [0..1] and thrust-vector (tilt) gimbal pitch [rad].
PROP_THROTTLE = ["fcs/throttle-cmd-norm[0]", "fcs/throttle-cmd-norm[1]"]
# Tilt is passed through unmodified: the FDM defines 0 rad as rotors-up, which
# matches the FC's neutral servo angle (drivers/servo/sim.c, centre 1500 us).
PROP_TILT = ["fcs/tilt-cmd-rad[0]", "fcs/tilt-cmd-rad[1]"]

G = 9.81
FT2M = 0.3048
RAD2DEG = 180.0 / math.pi

# --- sensor mounting -------------------------------------------------------
# This link stands in for the parts soldered to the board, and a part has no
# knowledge of the body frame - it reports what its own die sees. So the bridge
# rotates the FDM's body-frame truth INTO the die frame here, and the firmware's
# IMU_ALIGN / MAG_ALIGN rotate it back out (Firmware/common/align.c). That round
# trip is what puts the alignment path on the SIL's critical path; with the
# bridge emitting body frame it was skipped entirely and align.c was exercised
# only by test_align.c.
#
# These duplicate Firmware/target/flapjack_v1/flapjack_v1.h ON PURPOSE, and must
# be kept in step with it by hand. Taking the alignment from the firmware
# instead - by parsing the header, or having the FC report it - would make the
# round trip cancel by construction and assert nothing. Two independent
# statements are what let a mismatch fail the run.
#
# NOTE what this does and does not prove. It exercises the alignment MACHINERY:
# composition order, Align_Apply, the device wiring. It cannot tell you the
# CONSTANT is right - both sides wrong the same way still cancels. Whether
# FLIP_CW270 describes the real board is a six-face bench test, not a SIL run.
IMU_ALIGN = "FLIP_CW270"
MAG_ALIGN = "CW0"

_COS = {0: 1, 90: 0, 180: -1, 270: 0}
_SIN = {0: 0, 90: 1, 180: 0, 270: -1}
# Base rotation each alignment name starts from, as (axis, degrees).
_ALIGN_BASES = {
    "": ("z", 0),
    "FLIP": ("x", 180),
    "ROLL90": ("x", 90),
    "ROLL270": ("x", 270),
    "PITCH90": ("y", 90),
    "PITCH270": ("y", 270),
}


def _rot(axis, deg):
    c, s = _COS[deg], _SIN[deg]
    if axis == "x":
        return ((1, 0, 0), (0, c, -s), (0, s, c))
    if axis == "y":
        return ((c, 0, s), (0, 1, 0), (-s, 0, c))
    return ((c, -s, 0), (s, c, 0), (0, 0, 1))


def _matmul(A, B):
    return tuple(tuple(sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def align_matrix(name):
    """R for an eSensorAlign_t name, where body = R @ die.

    Same construction as the table in Firmware/common/align.c: the base rotation
    is applied to the sample first, then the yaw. Names read in that order.
    """
    if name.startswith("CW"):
        base_key, yaw = "", int(name[2:])
    else:
        base_key, _, yaw_str = name.partition("_CW")
        yaw = int(yaw_str)
    if base_key not in _ALIGN_BASES or yaw not in _COS:
        raise ValueError(f"unknown sensor alignment {name!r}")
    return _matmul(_rot("z", yaw), _rot(*_ALIGN_BASES[base_key]))


def _transpose(M):
    return tuple(zip(*M))


def _apply(M, v):
    return [sum(M[i][k] * v[k] for k in range(3)) for i in range(3)]


# body -> die is the inverse of the mount. These are orthonormal, so the
# transpose IS the inverse, exactly and without arithmetic.
_IMU_BODY_TO_DIE = _transpose(align_matrix(IMU_ALIGN))
_MAG_BODY_TO_DIE = _transpose(align_matrix(MAG_ALIGN))

# --- flight GUI feed (--gui-port) -------------------------------------------
# The GUI cannot read the FC itself during a SIL run - the bridge owns that
# wire - so it takes both the FDM's truth and the FC's own telemetry from here,
# on its own UDP port. Fire-and-forget by design: a GUI that is not running, or
# one that has fallen behind, must never pace the sim.
GUI_HOST = "127.0.0.1"
GUI_RATE = 50.0
# Matches SIM_LINK_PORT in Scripts/gui/app.py, which is the other half of this
# link; keep the two in step. On by default because the feed costs a datagram
# nobody has to read - pass --gui-port 0 to turn it off.
GUI_PORT = 5005
# The GUI can also fly the sim: it answers this feed on the same socket, so the
# pair needs no second port and no address to configure - the bridge replies to
# whoever is listening to it. Sticks go stale after GUI_RC_TIMEOUT seconds, so a
# closed or wedged GUI hands control back instead of leaving the last throttle
# latched forever.
GUI_RC_TIMEOUT = 0.5
# The four sticks only. AUX2 used to be here as the flight-mode switch; there is
# one mode now (tasks/mission/mission.c) and nothing reads it. AUX1 is excluded
# for a different reason - see apply_gui_rc.
GUI_RC_CHANNELS = ("roll", "pitch", "yaw", "throttle")

# Loopback tolerance on an echoed GPS coordinate, in degrees. ~1.1 cm: well
# below NMEA's own ~1.9 cm quantisation step (so it cannot mask a wrong field)
# and well below a float32 narrowing, which would show as ~2.2e-6 deg.
GPS_COORD_EPS = 1e-7

# Satellite count the synthesized fix reports. Constant, and echoed back in
# Telemetry, so it doubles as a cheap check that GGA's integer fields survive
# the trip and not just its coordinates.
GPS_SATS = 9

# NAV_VALID_* bits from Firmware/msgs/umsg/defs/nav.json. Duplicated here rather
# than imported because umsg generates C, not Python; keep in step with that file.
NAV_VALID_ATTITUDE = 1 << 0
NAV_VALID_POSITION = 1 << 1
NAV_VALID_VELOCITY = 1 << 2
NAV_VALID_BARO_ALT = 1 << 3

# Gates on the ESTIMATOR, not on the sensor loopback - these compare what the FC
# worked out against what the FDM knows, so unlike the loopback checks there is
# no bit-exact answer and the tolerance is an engineering judgement.
#
# These are set from MEASUREMENT, not from guesswork. Observed maxima over a
# full run, FC estimate against FDM truth, with the sensor error models in
# jsbsim/systems/Sensor*.xml ENABLED (2.0 Pa baro noise + 20 Pa bias +
# 0.05 Pa/s drift, ~1.6 m GPS):
#
#            hover      yaw_step     (noise-free, for reference)
#   alt      0.46 m     0.43 m        0.01 / 0.02 m
#   vz       0.26 m/s   0.28 m/s      0.05 / 0.13 m/s
#   pos      0.00 m     0.00 m        0.01 / 0.05 m
#
# Altitude and climb rate went up ~40x when the noise went on, which is the
# whole point: with perfect sensors those numbers measured roundoff, and ANY
# CFG_ALT_FILTER_K_* would have passed. They now measure how much baro noise the
# complementary filter actually lets through, so they are a real constraint on
# the tuning rather than a formality.
#
# Position went the other way - to zero - because the check is now made against
# the FC's own echoed fix from the SAME telemetry frame, differentially. Noise is
# common to both sides and cancels exactly, which is what leaves the projection
# itself as the only thing being measured.
NAV_ALT_EPS_M = 1.5

# Climb rate is the noisier channel - it is the derivative the accel path
# supplies and the baro only trims - so it gets the loosest gate relative to its
# measured error.
NAV_VZ_EPS_MPS = 1.0

# Horizontal position is unfiltered, straight off the fix, so the only error is
# NMEA quantisation (~2 cm) plus the flat-earth projection (sub-cm at these
# ranges). Worth knowing about this gate: it is only as strong as the
# trajectory. A plan that barely translates cannot catch north and east being
# swapped, because both are near zero - that needs a plan that actually flies
# somewhere, which none of the current three really do.
NAV_POS_EPS_M = 0.5

# Attitude gates. Both are on FDM TRUTH, not on the estimate: the failure these
# exist for is one where the estimate reads level while the vehicle is not, so
# checking the estimate against itself would pass it (KnownIssues 1.16).
#
# Departure bound. CFG_ANGLE_MAX_DEG is 30 and no shipped plan commands more than
# half stick, so anything past 45 deg is the vehicle leaving, not the pilot. Kept
# well clear of the commanded range deliberately - KnownIssues 1.13 warns against
# plan-specific attitude assertions, and this is a physics bound, not a tracking
# one.
ATTITUDE_MAX_DEG = 45.0

# Ring-down bound: peak-to-peak over the last ATTITUDE_SETTLE_S of the run. Every
# shipped plan ends with the sticks centred for at least 5 s, so a vehicle that is
# still moving here is one that will not settle - which is the whole signature of
# closing the angle loop on an accel-referenced estimate. Measured separation is
# wide: 0.03-0.13 deg on runs that settle, 0.9-24.9 deg on runs that do not.
ATTITUDE_SETTLE_S = 5.0
ATTITUDE_SETTLE_PKPK_DEG = 3.0

# ...and only while AIRBORNE. Three of the four shipped plans (hover, yaw_step,
# roll_pitch) only ramp to 1500 us, which is EXACTLY this airframe's hover
# throttle, so they never break ground - they sit on their gear for the whole
# run. Attitude there is gear interaction, not the control loop, and gating on it
# fails plans for a reason that has nothing to do with flight. 1.5 ft is clear of
# the gear and far below any hover this sim reaches.
ATTITUDE_AIRBORNE_FT = 1.5

# Loopback tolerance for the baro round trip, in Pa. The chain is bridge float32
# -> emulated BMP390 inverse compensation -> 24-bit ADC counts -> bmp390.c's
# float32 compensation, whose worst error over -20..60 C and 50..115 kPa
# measures 0.016 Pa. This is ~30x that, and still three orders of magnitude
# below what any real decode fault would cost.
BARO_PA_EPS = 0.5

# Reference Earth field in NED, ~60 deg inclination, 0 declination, in GAUSS.
# A magnitude rather than a unit vector because the field now goes into an
# emulated MMC5983 with a real +/-8 G full scale: a unit vector would assert
# that Earth saturates the part, and mmc5983.c's normalise-to-full-scale would
# hand the estimator a vector 16x too long. 0.50 G (50 uT) is a representative
# mid-latitude total field. Nothing downstream depends on the magnitude -
# MadgwickFilter_Update_9DOF_ renormalises - but the SCALE is now a real,
# testable property of the path rather than an assumption.
_INCL = math.radians(60.0)
_B_GAUSS = 0.50
B_NED = tuple(_B_GAUSS * c for c in (math.cos(_INCL), 0.0, math.sin(_INCL)))


def dcm_body_from_ned(phi, theta, psi):
    cphi, sphi = math.cos(phi), math.sin(phi)
    cth, sth = math.cos(theta), math.sin(theta)
    cpsi, spsi = math.cos(psi), math.sin(psi)
    return (
        (cth * cpsi, cth * spsi, -sth),
        (sphi * sth * cpsi - cphi * spsi, sphi * sth * spsi + cphi * cpsi, sphi * cth),
        (cphi * sth * cpsi + sphi * spsi, cphi * sth * spsi - sphi * cpsi, cphi * cth),
    )


def synthesize_sensors(phi, theta, psi, p, q, r, udot, vdot, wdot):
    """Return (accel[3] m/s^2, gyro[3] deg/s, mag[3] gauss) in each part's DIE frame.

    Two conversions separate what a part reports from what the FC wants, and
    both are undone in Firmware/devices/imu.c:

    1. Sign. Accelerometer = specific force as a real part measures it (a - g).
       Gravity points down, which is +z in FRD, so LEVEL & STILL is (0, 0, -9.81)
       in BODY frame - a part reads +1g along whichever axis points up, and FRD z
       points down. This is also the sign JSBSim's own accelerometer model uses
       (SilResearch 5.3 measured -9.800 against this function's former
       +9.789), so the two now agree instead of differing by a sign on Z. If your
       FDM's signs differ, this is the one place to flip.

    2. Frame. A sensor knows nothing about the body frame, so the body-frame
       values computed here are rotated into each die's own frame on the way
       out - see _IMU_BODY_TO_DIE above.
    """
    R = dcm_body_from_ned(phi, theta, psi)
    # Body linear acceleration, minus gravity projected into body.
    g_body = (-G * math.sin(theta), G * math.sin(phi) * math.cos(theta), G * math.cos(phi) * math.cos(theta))
    a_lin = (udot * FT2M, vdot * FT2M, wdot * FT2M)
    accel = [a_lin[k] - g_body[k] for k in range(3)]
    gyro = [p * RAD2DEG, q * RAD2DEG, r * RAD2DEG]
    mag = [sum(R[k][j] * B_NED[j] for j in range(3)) for k in range(3)]
    # Body -> die. Last step, so everything above stays readable as body frame.
    return (_apply(_IMU_BODY_TO_DIE, accel),
            _apply(_IMU_BODY_TO_DIE, gyro),
            _apply(_MAG_BODY_TO_DIE, mag))


def as_f32(x: float) -> float:
    """Round to IEEE 754 binary32, i.e. to exactly what a protobuf float carries.

    The loopback check compares the FC's echo against what was sent; both hops
    are binary32, so rounding here makes that comparison exact instead of
    approximate.
    """
    return struct.unpack("f", struct.pack("f", x))[0]


# --- sensor pushes ----------------------------------------------------------
# None of these are sim-link frames. They go to the Renode peripheral models
# (Scripts/renode/*.cs), not to the FC, and the FC reads the values back over
# emulated SPI through its real drivers. Each is 0xAA 0x55 then a little-endian
# float32 payload; SamplePushListener.cs resynchronises on the header, so a
# bridge that starts mid-stream is fine.
#
# The values are the same die-frame physical quantities the sim link used to
# carry - devices/imu.c and devices/mag.c undo the die rotation exactly as they
# do on hardware - so there is nothing extra to compute here.


def make_imu_push(accel, gyro) -> bytes:
    """accel xyz (m/s^2, specific force) then gyro xyz (deg/s), IMU die frame."""
    return struct.pack("<BB6f", 0xAA, 0x55, *accel, *gyro)


def make_mag_push(mag) -> bytes:
    """Field xyz in GAUSS, MAG die frame. The model scales to the part's
    +/-8 G full scale and packs the 18-bit unsigned counts."""
    return struct.pack("<BB3f", 0xAA, 0x55, *mag)


def make_baro_push(pressure_pa, temperature_c) -> bytes:
    """Static pressure (Pa) and temperature (degC). The model inverts the
    BMP390 compensation polynomial to find the raw counts bmp390.c will turn
    back into these."""
    return struct.pack("<BB2f", 0xAA, 0x55, pressure_pa, temperature_c)


def make_gui_packet(sim_t, euler_rad, truth_alt_m, telemetry, servo_rad, motor,
                    truth_vel_ned=None) -> bytes:
    """One JSON datagram of sim state for the flight GUI.

    This feeds a display, not the firmware, so it carries display units rather
    than the wire's: degrees, and altitude in metres positive UP (the wire is
    radians and NED, down positive). `nav` is null until the FC's first
    Telemetry frame, so the GUI can tell "no estimate yet" from "estimate of 0".

    `truth.alt` and `nav.alt` share the FC's own pressure datum as their zero -
    see _Checks.nav_alt_datum_m - which is what makes the two comparable on one
    set of axes.
    """
    phi, theta, psi = euler_rad
    state = {
        "t": round(sim_t, 4),
        "truth": {
            "roll": phi * RAD2DEG,
            "pitch": theta * RAD2DEG,
            "yaw": psi * RAD2DEG,
            "alt": truth_alt_m,
        },
        "nav": None,
        # Horizontal velocity, m/s NED. The guidance loop commands this
        # directly now, so a run cannot be judged without seeing it.
        "vel": list(truth_vel_ned) if truth_vel_ned else None,
        "servo": [a * RAD2DEG for a in servo_rad],
        "motor": list(motor),
    }
    if telemetry is not None:
        state["nav"] = {
            "roll": telemetry.euler[0],
            "pitch": telemetry.euler[1],
            "yaw": telemetry.euler[2],
            # NED, down positive - the same negation the nav gate makes.
            "alt": -telemetry.nav_pos_ned[2],
            "valid": telemetry.nav_valid,
            "armed": telemetry.armed,
        }
    return json.dumps(state).encode("ascii")


def poll_gui_rc(sock):
    """The newest stick positions the GUI has sent, or None if it sent none.

    Drains the queue rather than reading one datagram: only the newest sticks
    matter, an older one is a position the pilot has already moved off.
    """
    latest = None
    while True:
        try:
            data, _ = sock.recvfrom(512)
        except OSError:
            # Either nothing pending (the socket is non-blocking) or Windows
            # reporting an earlier send's ICMP port-unreachable as
            # WSAECONNRESET. Neither is a datagram.
            break
        try:
            rc = json.loads(data).get("rc")
        except (ValueError, AttributeError):
            continue
        if isinstance(rc, dict):
            latest = rc
    return latest


def apply_gui_rc(channels, rc):
    """Overwrite the GUI-driven channels in `channels` (us), leaving the rest.

    AUX1 is deliberately not one of them: the arm switch stays with the bridge's
    arming gesture, so the FC's interlock is exercised the same way whether or
    not a GUI is attached.
    """
    for name in GUI_RC_CHANNELS:
        us = rc.get(name)
        if us is not None:
            channels[planlib.CHANNELS[name]] = max(RC_MIN, min(RC_MAX, int(us)))


def rc_channels(throttle_norm: float, armed: bool) -> list:
    """The 16 stick positions, in microseconds, a pilot would be holding."""
    ch = [RC_MID] * 16
    ch[CH_THROTTLE] = int(RC_MIN + throttle_norm * (RC_MAX - RC_MIN))
    ch[CH_AUX1] = RC_MAX if armed else RC_MIN  # arm switch high
    return ch


class _Gates:
    """Per-phase PASS/FAIL criteria, evaluated against FDM TRUTH as the run goes.

    _Checks below asserts what is true of any correct flight. This asserts what
    is true of THIS plan: that while the sticks were held still, the vehicle
    tracked what they were asking for and did not drift.

    Three properties are deliberate:

    **The reference comes from the plan, not from the FC.** `plan.commanded()`
    turns stick microseconds into the angle/rate/climb the firmware should be
    closing on. Comparing the FC's own setpoint echo against the FC's own
    estimate would pass a vehicle that is confidently wrong - the same argument
    KnownIssues 1.16 makes for gating on truth.

    **Every sample counts, not just the last one.** A gate is checked on each
    tick of the hold window, so a bank that holds on average but breaks briefly
    still fails. That is what "maintains the climb AND the roll" has to mean.

    **A gate with no samples is a failure, not a pass.** A phase shorter than a
    tick, or a plan that aborted before reaching it, must not report green.
    """

    def __init__(self, flight_plan, keep_going=False, report_path=None,
                 trace_path=None):
        self.plan = flight_plan
        self.keep_going = keep_going
        self.report_path = report_path
        # Per-sample commanded-vs-truth trace. Off by default; it is a
        # CALIBRATION tool, not part of the verdict. `worst` and the time it
        # happened cannot tell a slow settle from an overshoot that recovers,
        # and those two want opposite fixes - a longer settle, or a gain.
        self.trace = None
        if trace_path:
            self.trace = open(trace_path, "w", buffering=1)
            self.trace.write("t,phase,window,roll_cmd,roll,pitch_cmd,pitch,"
                             "yawrate_cmd,yawrate,climb_cmd,climb,alt,est_roll,est_pitch,est_yaw,vdot,wdot,heading\n")
        self.enabled = bool(flight_plan is not None and flight_plan.phases)
        self.abort = False
        self.first = None            # first violation, as a dict
        self.cur = None              # phase being flown
        self.entry = None            # (alt_m, heading_deg, plan_t) at hold entry
        self.last = None             # most recent truth sample within the hold
        # phase name -> gate name -> accumulator
        self.acc = {}
        if self.enabled:
            for ph in flight_plan.phases:
                self.acc[ph.name] = {
                    g: {"worst": 0.0, "limit": gate.tol, "n": 0, "t": None,
                        "target": None, "actual": None, "summary": gate.is_summary}
                    for g, gate in ph.gates.items()
                }

    def _violate(self, phase, gate, plan_t, target, actual, err, tol):
        rec = self.acc[phase][gate]
        if err > rec["worst"]:
            rec.update(worst=err, t=plan_t, target=target, actual=actual)
        if err <= tol:
            return
        if self.first is None:
            self.first = {"phase": phase, "gate": gate, "t": plan_t,
                          "target": target, "actual": actual, "err": err, "tol": tol}
            # Loud, and at the moment it happens - not folded into a summary
            # minutes of log-spam later. The run stops here unless told not to,
            # so this line is the last thing on the terminal.
            # flush because stdout block-buffers the moment it is redirected
            # to a file - which is every CI run and every backgrounded run.
            # Unbuffered, this line lands minutes later beside the summary,
            # which is the exact failure it exists to avoid.
            print(f"\n[gate] VIOLATION {phase}/{gate} at plan t={plan_t:.2f}s: "
                  f"expected {target:+.3f}, measured {actual:+.3f}, "
                  f"error {err:.3f} > tol {tol:.3f}", flush=True)
            if not self.keep_going:
                print("[gate] aborting the run - pass --keep-going to survey the "
                      "remaining phases instead", flush=True)
                self.abort = True

    # est_* gate -> which euler component, in both the FC's array and the
    # truth dict below.
    _EST_IDX = {"est_roll_deg": 0, "est_pitch_deg": 1, "est_yaw_deg": 2}
    _EST_TRUTH = ("roll_deg", "pitch_deg", "heading_deg")

    def update(self, plan_t, channels, truth, est=None):
        """Evaluate the active phase's gates against one truth sample.

        `est` is the FC's own euler estimate in degrees, or None when no
        telemetry has arrived yet or NAV_VALID_ATTITUDE is clear. None
        leaves the est_* gates with no samples, which reports NOT RUN and
        fails - an estimate gate that quietly skipped would be worthless.
        """
        if not self.enabled:
            return
        ph = self.plan.phase_at(plan_t)
        if ph is not self.cur:
            self._close()
            self.cur, self.entry, self.last = ph, None, None

        if self.trace is not None and ph is not None:
            c = planlib.commanded(channels)
            e = tuple(f"{v:.3f}" for v in est) if est is not None else ("", "", "")
            self.trace.write(
                f"{plan_t:.3f},{ph.name},{'hold' if ph.holding(plan_t) else 'settle'},"
                f"{c['roll_deg']:.3f},{truth['roll_deg']:.3f},"
                f"{c['pitch_deg']:.3f},{truth['pitch_deg']:.3f},"
                f"{c['yaw_rate_dps']:.3f},{truth['yaw_rate_dps']:.3f},"
                f"{c['climb_mps']:.3f},{truth['climb_mps']:.3f},{truth['alt_m']:.3f},{e[0]},{e[1]},{e[2]},{truth['vdot_mps2']:.4f},{truth['wdot_mps2']:.4f},{truth['heading_deg']:.3f}\n")

        if ph is None or not ph.holding(plan_t):
            return

        if self.entry is None:
            self.entry = (truth["alt_m"], truth["heading_deg"], plan_t)
        self.last = (truth["alt_m"], truth["heading_deg"], plan_t)

        cmd = planlib.commanded(channels)
        for name, gate in ph.gates.items():
            if gate.is_summary:
                continue
            if gate.is_estimate:
                if est is None:
                    continue
                i = self._EST_IDX[name]
                target = truth[self._EST_TRUTH[i]]
                actual = est[i]
                # Wrapped: heading is 0..360 on one side and may be -180..180 on
                # the other, so a raw difference reads ~360 for two values that
                # agree. Roll wraps at +/-180 for the same reason. Pitch cannot
                # wrap in practice, but taking the same path keeps one rule.
                err = abs(_wrap180(actual - target))
            else:
                target = cmd[name] if gate.target == "cmd" else gate.target
                actual = truth[name]
                err = abs(actual - target)
            self.acc[ph.name][name]["n"] += 1
            self._violate(ph.name, name, plan_t, target, actual, err, gate.tol)

    def _close(self):
        """Evaluate the finished phase's drift gates across its whole hold."""
        if self.cur is None or self.entry is None or self.last is None:
            return
        a_alt, a_hdg, a_t = self.entry
        b_alt, b_hdg, b_t = self.last
        span = max(b_t - a_t, 1e-6)
        for name, gate in self.cur.gates.items():
            if not gate.is_summary:
                continue
            if name == "alt_drift_m":
                actual = b_alt - a_alt
            else:   # heading_drift_dps - wrapped, so 359 -> 1 is 2 deg, not 358
                actual = _wrap180(b_hdg - a_hdg) / span
            self.acc[self.cur.name][name]["n"] += 1
            self._violate(self.cur.name, name, b_t, 0.0, actual,
                          abs(actual), gate.tol)

    def report(self) -> int:
        """Print the phase x gate table, write the JSON artifact, return an exit code."""
        if not self.enabled:
            return 0
        self._close()

        rows, failed, no_data, unreached = [], 0, 0, []
        for ph in self.plan.phases:
            gates = self.acc[ph.name]
            if not gates:
                continue
            if all(rec["n"] == 0 for rec in gates.values()):
                unreached.append(ph.name)
            for name, rec in gates.items():
                if rec["n"] == 0:
                    state = "NOT RUN"
                    no_data += 1
                elif rec["worst"] > rec["limit"]:
                    state = "FAIL"
                    failed += 1
                else:
                    state = "PASS"
                rows.append((ph.name, name, rec["worst"], rec["limit"], rec["n"], state))

        print()
        print("[gate] ------------------------------------------------------------")
        print(f"[gate] {'phase':<22}{'gate':<16}{'worst':>8}{'limit':>8}{'n':>7}")
        for phase, name, worst, limit, n, state in rows:
            print(f"[gate] {phase:<22}{name:<16}{worst:>8.3f}{limit:>8.3f}{n:>7}  {state}")
        if unreached:
            print(f"[gate] not reached: {', '.join(unreached)}")
        if self.first is not None:
            f = self.first
            print(f"[gate] FIRST FAILURE  {f['phase']}/{f['gate']} at plan t={f['t']:.2f}s")
            print(f"[gate]   expected {f['target']:+.3f}, measured {f['actual']:+.3f}, "
                  f"error {f['err']:.3f} > tol {f['tol']:.3f}")

        # A gate that never ran is a failure. A plan can only get quieter by
        # accident - a phase shorter than a tick, an abort upstream - and a
        # green line for a criterion nobody evaluated is the exact way a
        # regression suite stops meaning anything.
        bad = failed + no_data
        verdict = "PASS" if bad == 0 else f"FAIL ({failed} failed, {no_data} never ran)"
        print(f"[gate] {verdict}")

        if self.report_path:
            doc = {
                "plan": self.plan.name,
                "verdict": "pass" if bad == 0 else "fail",
                "first_failure": self.first,
                "phases": [
                    {"name": ph.name, "settle": ph.settle, "hold": ph.hold,
                     "gates": {n: {k: v for k, v in rec.items() if k != "summary"}
                               for n, rec in self.acc[ph.name].items()}}
                    for ph in self.plan.phases
                ],
            }
            path = Path(self.report_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(doc, indent=2))
            print(f"[gate] wrote {path}")
        if self.trace is not None:
            self.trace.close()
            print(f"[gate] wrote {self.trace.name}")
        return 1 if bad else 0


def _wrap180(deg: float) -> float:
    """Signed angle difference in (-180, 180]."""
    return (deg + 180.0) % 360.0 - 180.0


class _Checks:
    """Physics-level invariants, gathered as the run goes.

    Deliberately not a golden trajectory. KnownIssues 1.13 recorded that the
    then-current *rate* loop settled at an unrepeatable attitude, so asserting on
    it produced false regressions. The loop holds an angle now and the reasoning
    has not been re-derived for it - so these stay what is true of any correct
    flight, and they are the class of fault the SIL has actually caught (motors
    pinned at 1.000, servo commands 20x out of range).
    """

    def __init__(self):
        self.nan = 0
        self.motor_min, self.motor_max = 1e9, -1e9
        self.servo_min, self.servo_max = 1e9, -1e9
        self.alt_min, self.alt_max = 1e9, -1e9
        self.motor_pinned = 0
        self.samples = 0
        # Truth attitude while ARMED, in degrees. The window holds the tail of the
        # run so report() can measure whether it ever stopped moving.
        # Actuator frames, for deriving the CONTROL loop's real rate. It paces
        # itself now (CONTROL_RATE_HZ in control.c) rather than running once per
        # guidance setpoint, so "is it keeping up" is a question worth answering
        # from the wire instead of assuming the configured period.
        self.motor_frames = 0
        self.t_first = None
        self.t_last = None
        self.att_peak = [0.0, 0.0]
        self.att_window = collections.deque()

        # --- sensor loopback (see sim.proto's Telemetry comment) -------------
        # Recent pressures as actually pushed, rounded to float32.
        #
        # This used to be a BIT-EXACT comparison, and can no longer be: the
        # value now goes into an emulated BMP390 as a 24-bit ADC count and comes
        # back through the driver's float32 compensation polynomial. What is
        # left is still a strong check - BARO_PA_EPS is ~30x the measured
        # round-trip error, and a decode fault (a mis-scaled coefficient, a
        # swapped byte, a stale slot) is out by kilopascals, not millipascals.
        self.baro_sent = collections.deque(maxlen=64)
        self.baro_frames = 0
        self.baro_echoed = 0
        self.baro_miss = 0
        self.baro_last_count = 0

        # GPS cannot use the bit-exact comparison above: NMEA quantises the
        # coordinate to 5 decimal places of a minute (~1.9 cm), so what comes
        # back is what the wire could represent, not what the FDM held. These
        # are the post-quantisation values, compared within GPS_COORD_EPS -
        # tight enough that narrowing the chain back to float32 (~2.2e-6 deg at
        # this latitude) fails, loose enough to absorb double rounding.
        self.gps_sent = collections.deque(maxlen=32)
        self.gps_sentences = 0
        self.gps_echoed = 0
        self.gps_miss = 0
        self.gps_last_count = 0
        # Set once --gps-stop cuts the wire. After that the FC is expected to
        # report the fix lost exactly once (sats -> 0), which is a correct
        # report rather than a decode mismatch - but only after the cut. The
        # same value arriving while NMEA is still streaming IS a fault.
        self.gps_stopped = False
        self.gps_lost_reports = 0
        # Ticks on which the FDM handed back a non-finite position, so GPS could
        # not be encoded. Non-zero means the flight model diverged; it is
        # reported as its own line rather than folded into the NaN count, which
        # counts FC-side NaN.
        self.gps_nonfinite = 0
        # First (echoed fix, reported position) pair seen with NAV_VALID_POSITION
        # set. The projection gate is differential against this, so the FC's own
        # origin cancels instead of having to be reconstructed.
        self.nav_pos_ref = None
        self.nav_pos_prev = []

        # --- estimator gates (see the nav_* comment in sim.proto) ------------
        # The opposite of the loopback above: nothing here was sent to the FC,
        # so these compare the FC's own estimate against the FDM's truth.
        #
        # Truth for altitude is height above the point where the FC took its
        # pressure datum, NOT h-agl - the datum is whatever the baro averaged in
        # the first second of firmware boot. So the reference is captured at the
        # exact frame NAV_VALID_BARO_ALT first appears, which is by construction
        # the moment the datum exists. The FDM is frozen at its IC until the FC
        # arms, so that capture is not a race.
        self.nav_alt_datum_m = None
        self.nav_alt_samples = 0
        self.nav_alt_err_max = 0.0
        self.nav_vz_err_max = 0.0

        # Same idea horizontally: the origin is the FC's first usable fix.
        self.nav_origin = None       # (lat_deg, lon_deg)
        self.nav_pos_samples = 0
        self.nav_pos_err_max = 0.0

        self.nav_valid_seen = 0      # union of every bitmask observed

    def motors(self, values):
        self.motor_frames += 1
        for v in values:
            if v != v:
                self.nan += 1
                continue
            self.motor_min = min(self.motor_min, v)
            self.motor_max = max(self.motor_max, v)
            if v >= 0.999:
                self.motor_pinned += 1

    def servos(self, values):
        for v in values:
            if v != v:
                self.nan += 1
                continue
            self.servo_min = min(self.servo_min, v)
            self.servo_max = max(self.servo_max, v)

    def baro_tx(self, pressure_pa):
        """Record a pushed pressure as float32, the way the push carries it."""
        self.baro_frames += 1
        self.baro_sent.append(as_f32(pressure_pa))

    def baro_rx(self, telemetry):
        """Check one Telemetry echo against what was actually sent."""
        if telemetry.baro_count == 0:
            return                      # nothing decoded yet; not a failure
        if telemetry.baro_count == self.baro_last_count:
            return                      # same sample re-reported; only count fresh ones
        self.baro_last_count = telemetry.baro_count
        self.baro_echoed += 1
        if not any(abs(telemetry.baro_pa - sent) <= BARO_PA_EPS for sent in self.baro_sent):
            self.baro_miss += 1

    def gps_tx(self, lat, lon, sats):
        """Record a positional sentence as the wire will actually carry it.

        Called for GGA *and* RMC: both carry a position, so the FC publishes on
        both, and counting only fixes would report half as many sent as echoed.
        RMC has no satellite field, so the count it echoes is the one the last
        GGA set - which is why the same `sats` is recorded for both.
        """
        self.gps_sentences += 1
        self.gps_sent.append((nmea.coord_on_wire(lat, True),
                              nmea.coord_on_wire(lon, False),
                              sats))

    def gps_rx(self, telemetry):
        if telemetry.gps_count == 0:
            return
        if telemetry.gps_count == self.gps_last_count:
            return
        self.gps_last_count = telemetry.gps_count
        self.gps_echoed += 1

        # A fix-loss report: the FC republishes the last known position with the
        # quality fields cleared once the fix goes stale. Expected after the wire
        # is cut, and a fault before it - sats never legitimately reaches 0 while
        # sentences carrying a fix are still arriving.
        if telemetry.gps_sats == 0:
            if self.gps_stopped:
                self.gps_lost_reports += 1
            else:
                self.gps_miss += 1
            return

        for lat, lon, sats in self.gps_sent:
            if (abs(telemetry.gps_lat - lat) < GPS_COORD_EPS
                    and abs(telemetry.gps_lon - lon) < GPS_COORD_EPS
                    and telemetry.gps_sats == sats):
                return
        self.gps_miss += 1

    def nav_rx(self, telemetry, fdm):
        """Compare the FC's position/velocity estimate against the FDM."""
        if fdm is None:
            return

        valid = telemetry.nav_valid
        self.nav_valid_seen |= valid

        # NED, down positive - altitude is the negated third element. Reading it
        # the other way round is exactly the sign fault this gate exists to
        # catch, so it is spelled out rather than folded into the comparison.
        est_alt = -telemetry.nav_pos_ned[2]
        est_vz = -telemetry.nav_vel_ned[2]

        if valid & NAV_VALID_BARO_ALT:
            h_sl = fdm["position/h-sl-meters"]
            if self.nav_alt_datum_m is None:
                self.nav_alt_datum_m = h_sl
            true_alt = h_sl - self.nav_alt_datum_m
            # h-dot is the FDM's climb rate, positive up, in ft/s.
            true_vz = fdm["velocities/h-dot-fps"] * FT2M

            self.nav_alt_samples += 1
            self.nav_alt_err_max = max(self.nav_alt_err_max, abs(est_alt - true_alt))
            self.nav_vz_err_max = max(self.nav_vz_err_max, abs(est_vz - true_vz))

        if (valid & NAV_VALID_POSITION) and telemetry.gps_sats:
            # Checked against the FC's OWN echoed fix, from the SAME telemetry
            # frame - not against a fresh FDM read, and not against the bridge's
            # last transmission.
            #
            # Both of those alternatives were tried and both measure noise rather
            # than firmware. sensor/gps/lat_rad is a fresh random draw every tick
            # once the error models are on, and the FC's telemetry necessarily
            # lags the bridge's most recent send by a poll and a decode - so
            # either comparison differences two independent ~1.6 m draws and
            # reports 6-9 m of "estimator error" on a vehicle that is hovering.
            # Same frame means zero skew by construction.
            #
            # It is also DIFFERENTIAL, against the first sample rather than an
            # absolute origin, so the FC's own choice of origin cancels out
            # instead of having to be guessed at from outside.
            #
            # What survives is exactly what this gate always claimed to test: did
            # the FC apply the flat-earth projection to the right fields, with
            # the right signs, at the right scale.
            if self.nav_pos_ref is None:
                self.nav_pos_ref = (telemetry.gps_lat, telemetry.gps_lon,
                                    telemetry.nav_pos_ned[0], telemetry.nav_pos_ned[1])
            lat0, lon0, n0, e0 = self.nav_pos_ref
            actual_dn = telemetry.nav_pos_ned[0] - n0
            actual_de = telemetry.nav_pos_ned[1] - e0

            # One frame of skew is structural and has to be tolerated.
            # SimTelemetry_Task reads umsg_nav_state_t first and drains
            # umsg_sensors_gps_t after, so a fix landing between those two reads
            # puts an OLD nav position alongside a NEW echoed fix. Bounded at
            # exactly one fix - hence two candidates, not a deque: allowing more
            # would let the gate find a match for anything.
            best = None
            for lat, lon in [(telemetry.gps_lat, telemetry.gps_lon)] + self.nav_pos_prev:
                expect_dn = math.radians(lat - lat0) * 6371000.0
                expect_de = (math.radians(lon - lon0) * 6371000.0
                             * math.cos(math.radians(lat0)))
                e = math.hypot(actual_dn - expect_dn, actual_de - expect_de)
                best = e if best is None else min(best, e)
            self.nav_pos_prev = [(telemetry.gps_lat, telemetry.gps_lon)]

            self.nav_pos_samples += 1
            self.nav_pos_err_max = max(self.nav_pos_err_max, best)

    def state(self, alt, euler, armed=False, sim_t=0.0):
        self.samples += 1
        if self.t_first is None:
            self.t_first = sim_t
        self.t_last = sim_t
        if alt != alt or any(e != e for e in euler):
            self.nan += 1
            return
        self.alt_min = min(self.alt_min, alt)
        self.alt_max = max(self.alt_max, alt)

        # Disarmed, or still on the gear, the attitude says nothing about the
        # loop - see ATTITUDE_AIRBORNE_FT. A plan that never flies simply
        # produces no window, and report() then has nothing to assert.
        if not armed or alt < ATTITUDE_AIRBORNE_FT:
            return
        roll, pitch = math.degrees(euler[0]), math.degrees(euler[1])
        self.att_peak[0] = max(self.att_peak[0], abs(roll))
        self.att_peak[1] = max(self.att_peak[1], abs(pitch))
        self.att_window.append((sim_t, roll, pitch))
        while self.att_window and (sim_t - self.att_window[0][0]) > ATTITUDE_SETTLE_S:
            self.att_window.popleft()

    def _settle_pkpk(self):
        """Peak-to-peak roll and pitch over the tail of the run, or None."""
        if len(self.att_window) < 2:
            return None
        rolls = [w[1] for w in self.att_window]
        pitches = [w[2] for w in self.att_window]
        return (max(rolls) - min(rolls), max(pitches) - min(pitches))

    def report(self, armed_ever: bool) -> int:
        """Print a pass/fail summary; return a process exit code."""
        fails = []
        if not armed_ever:
            fails.append("FC never armed")
        if self.nan:
            fails.append(f"{self.nan} NaN samples")
        if self.gps_nonfinite:
            fails.append(f"FDM position went non-finite on {self.gps_nonfinite} ticks "
                         f"- the flight model diverged (see KnownIssues 1.14)")
        if self.motor_pinned:
            fails.append(f"motor saturated at 1.000 on {self.motor_pinned} samples")
        if self.motor_max > 1.0 or self.motor_min < 0.0:
            fails.append(f"motor out of [0,1]: {self.motor_min:.3f}..{self.motor_max:.3f}")
        if max(abs(self.servo_min), abs(self.servo_max)) > 1.5708:
            fails.append(f"servo outside +/-pi/2: {self.servo_min:.3f}..{self.servo_max:.3f}")

        # Sensor loopback. Silent if the sensor was never streamed this run, so
        # a --dry-run or a plan that predates baro support still passes.
        if self.baro_frames:
            if self.baro_echoed == 0:
                fails.append(f"baro: {self.baro_frames} samples pushed, FC read none "
                             f"(check Baro_Task, the BMP390 model and the SPI5 mux)")
            elif self.baro_miss:
                fails.append(f"baro loopback mismatch on {self.baro_miss}/{self.baro_echoed} "
                             f"echoes - the FC reported a pressure that was never sent")
        if self.gps_sentences:
            if self.gps_echoed == 0:
                fails.append(f"gps: {self.gps_sentences} sentences sent, FC decoded none "
                             f"(check Gps_Task / the NMEA wire on USART2)")
            elif self.gps_miss:
                fails.append(f"gps loopback mismatch on {self.gps_miss}/{self.gps_echoed} "
                             f"echoes - the FC reported a fix that was never sent")
            # --gps-stop asserts the failsafe: the FC must announce the loss, not
            # leave a stale fix sitting in every subscriber's cache looking live.
            if self.gps_stopped and self.gps_lost_reports == 0:
                fails.append("gps: NMEA was cut but the FC never reported the fix lost "
                             "(check GPS_FIX_TIMEOUT_US / Gps_HasFix)")

        # Attitude gates - see ATTITUDE_MAX_DEG / ATTITUDE_SETTLE_PKPK_DEG.
        if self.att_peak[0] > ATTITUDE_MAX_DEG or self.att_peak[1] > ATTITUDE_MAX_DEG:
            fails.append(f"attitude departed: peak roll {self.att_peak[0]:.1f} deg, "
                         f"pitch {self.att_peak[1]:.1f} deg (limit {ATTITUDE_MAX_DEG:.0f})")
        pkpk = self._settle_pkpk()
        if pkpk is not None and max(pkpk) > ATTITUDE_SETTLE_PKPK_DEG:
            fails.append(f"attitude never settled: last {ATTITUDE_SETTLE_S:.0f}s peak-to-peak "
                         f"roll {pkpk[0]:.1f} deg, pitch {pkpk[1]:.1f} deg "
                         f"(limit {ATTITUDE_SETTLE_PKPK_DEG:.1f})")

        # Estimator gates. Each is silent unless its INPUT was actually streamed,
        # so a --dry-run or an older plan still passes - but once the sensor was
        # fed, "the estimator never produced anything" is a failure rather than a
        # quiet skip. That distinction is the whole point: asserting against a
        # field nobody populated is how a green gate comes to mean nothing.
        if self.baro_frames:
            if not (self.nav_valid_seen & NAV_VALID_BARO_ALT):
                fails.append("nav: baro streamed but NAV_VALID_BARO_ALT never set - "
                             "the altitude estimate never came up (check the datum "
                             "warmup in Nav_UpdateBaro)")
            elif self.nav_alt_err_max > NAV_ALT_EPS_M:
                fails.append(f"nav altitude off by {self.nav_alt_err_max:.2f} m "
                             f"(limit {NAV_ALT_EPS_M:.1f}) - check the sign of "
                             f"Nav_VerticalAccelUp and the pos_ned[2] negation")
            elif self.nav_vz_err_max > NAV_VZ_EPS_MPS:
                fails.append(f"nav climb rate off by {self.nav_vz_err_max:.2f} m/s "
                             f"(limit {NAV_VZ_EPS_MPS:.1f})")
        if self.gps_sentences and not self.gps_stopped:
            if not (self.nav_valid_seen & NAV_VALID_POSITION):
                fails.append("nav: GPS streamed but NAV_VALID_POSITION never set - "
                             "the fix reached the topic but not the estimate")
            elif self.nav_pos_err_max > NAV_POS_EPS_M:
                fails.append(f"nav position off by {self.nav_pos_err_max:.2f} m "
                             f"(limit {NAV_POS_EPS_M:.1f}) - check the north/east "
                             f"projection in Nav_UpdateGps")

        print()
        print("[checks] ---------------------------------------------")
        print(f"[checks] samples      {self.samples}")
        print(f"[checks] motor range  {self.motor_min:.3f} .. {self.motor_max:.3f}")
        print(f"[checks] servo range  {self.servo_min:+.3f} .. {self.servo_max:+.3f} rad")
        print(f"[checks] altitude     {self.alt_min:.1f} .. {self.alt_max:.1f} ft")
        if self.baro_frames:
            print(f"[checks] baro         {self.baro_frames} sent, {self.baro_echoed} echoed, "
                  f"{self.baro_miss} mismatched")
        if self.gps_sentences:
            lost = f", {self.gps_lost_reports} fix-loss report(s)" if self.gps_stopped else ""
            print(f"[checks] gps          {self.gps_sentences} sent, {self.gps_echoed} echoed, "
                  f"{self.gps_miss} mismatched{lost}")
        if self.nav_alt_samples:
            print(f"[checks] nav alt      {self.nav_alt_samples} samples, "
                  f"max err {self.nav_alt_err_max:.2f} m, "
                  f"vz max err {self.nav_vz_err_max:.2f} m/s")
        if self.nav_pos_samples:
            print(f"[checks] nav pos      {self.nav_pos_samples} samples, "
                  f"max err {self.nav_pos_err_max:.2f} m")
        if self.baro_frames or self.gps_sentences:
            print(f"[checks] nav valid    0x{self.nav_valid_seen:02x} "
                  f"(attitude={bool(self.nav_valid_seen & NAV_VALID_ATTITUDE)} "
                  f"baro_alt={bool(self.nav_valid_seen & NAV_VALID_BARO_ALT)} "
                  f"position={bool(self.nav_valid_seen & NAV_VALID_POSITION)} "
                  f"velocity={bool(self.nav_valid_seen & NAV_VALID_VELOCITY)})")
        if self.motor_frames and self.t_first is not None and self.t_last > self.t_first:
            print(f"[checks] control     {self.motor_frames} actuator frames over "
                  f"{self.t_last - self.t_first:.1f}s sim = "
                  f"{self.motor_frames / (self.t_last - self.t_first):.0f} Hz")
        pkpk = self._settle_pkpk()
        if pkpk is not None:
            print(f"[checks] attitude     peak roll {self.att_peak[0]:.1f} / pitch "
                  f"{self.att_peak[1]:.1f} deg; last {ATTITUDE_SETTLE_S:.0f}s pk-pk "
                  f"{pkpk[0]:.2f} / {pkpk[1]:.2f} deg")
        print(f"[checks] NaN samples  {self.nan}")
        if self.gps_nonfinite:
            print(f"[checks] FDM diverged {self.gps_nonfinite} non-finite ticks")
        for f in fails:
            print(f"[checks] FAIL: {f}")
        print("[checks] " + ("PASS" if not fails else f"FAIL ({len(fails)})"))
        return 1 if fails else 0


def main(argv=None):
    ap = argparse.ArgumentParser(prog="board.py sim", description="JSBSim <-> Flapjack HIL bridge")
    ap.add_argument("--port", default=None,
                    help=f"the FC's sim link. Defaults to {sil_url(SIL_LINK_PORT)}, i.e. a "
                         "SIL run against `board.py renode`; give a real device (COM7, "
                         "/dev/ttyUSB0) to drive a board. When this is a socket every "
                         "other wire below defaults to its SIL port too, so a plain "
                         "`board.py sim` wires the whole rig. Pass `none` to any of them "
                         "to leave that wire off.")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--rc-port", default=None,
                    help=f"port carrying CRSF frames to the FC's RX UART (USART3). "
                         f"Defaults to {sil_url(SIL_RC_PORT)} on a SIL run. This is the "
                         "only RC path - with `none` the FC gets no sticks and will "
                         "never arm.")
    ap.add_argument("--rc-baud", type=int, default=416666,
                    help="CRSF baud (spec default for dual-wire full duplex); ignored "
                         "for a socket:// URL, and Renode does not pace bytes anyway")
    ap.add_argument("--rc-rate", type=float, default=50.0,
                    help="CRSF frame rate (Hz). A real link runs 50-150; the FC polls "
                         "Rx_Task at 50, so faster only wastes frames.")
    ap.add_argument("--rc-stop", type=float, default=None, metavar="SECONDS",
                    help="stop sending CRSF this many seconds in, imitating a receiver "
                         "dropping into failsafe. The FC should report rc_link_up=False "
                         "about a second later.")
    ap.add_argument("--rate", type=float, default=400.0, help="sensor stream rate (Hz)")
    ap.add_argument("--imu-port", default=None,
                    help=f"port carrying samples to the emulated BMI323. Defaults to "
                         f"{sil_url(SIL_IMU_PORT)} on a SIL run. Under Renode every "
                         "sensor is its real driver talking to an emulated part, so "
                         "this is the only accel/gyro path; with `none` the FC logs "
                         "'IMU stopped producing data' and never gets an attitude.")
    ap.add_argument("--mag-port", default=None,
                    help=f"port carrying field samples to the emulated MMC5983. Defaults "
                         f"to {sil_url(SIL_MAG_PORT)} on a SIL run. Only mag path; with "
                         "`none` the attitude estimate runs 6DOF and the heading is free "
                         "to drift.")
    ap.add_argument("--baro-port", default=None,
                    help=f"port carrying pressure/temperature to the emulated BMP390. "
                         f"Defaults to {sil_url(SIL_BARO_PORT)} on a SIL run. Only baro "
                         "path; with `none` NAV_VALID_BARO_ALT never sets and there is "
                         "no altitude.")
    ap.add_argument("--mag-rate", type=float, default=100.0,
                    help="magnetometer push rate in Hz (default 100, the MMC5983 "
                         "continuous rate mmc5983.c programs). The emulated part raises "
                         "drdy on every push, so this IS the FC's mag interrupt rate - "
                         "pushing faster models a part the board does not have.")
    ap.add_argument("--baro-rate", type=float, default=50.0,
                    help="barometer push rate (Hz). Its own rate rather than the "
                         "400 Hz sensor stream - a real barometer is a 10-50 Hz part, "
                         "and the emulated one only raises drdy_press when a fresh "
                         "sample lands, so this really does pace Baro_Task. 0 "
                         "disables baro entirely.")
    ap.add_argument("--gps-port", default=None,
                    help=f"port carrying NMEA sentences to the FC's GPS UART (USART2). "
                         f"Defaults to {sil_url(SIL_GPS_PORT)} on a SIL run. Like "
                         "--rc-port this is the only GPS path: the sim link carries no "
                         "GPS frame, so the SIL exercises the real byte assembler and "
                         "minmea.")
    ap.add_argument("--gps-baud", type=int, default=115200,
                    help="GPS baud (matches GPS_BAUD_RATE); ignored for a socket:// URL")
    ap.add_argument("--gps-stop", type=float, default=None, metavar="SECONDS",
                    help="stop sending NMEA this many seconds in, imitating a dead "
                         "antenna or an unplugged receiver. The FC should report the "
                         "fix lost (sats -> 0) about GPS_FIX_TIMEOUT_US later, rather "
                         "than leaving a stale fix looking current.")
    ap.add_argument("--gps-rate", type=float, default=10.0,
                    help="GPS fix rate (Hz). Each fix is a GGA and an RMC, emitted "
                         "half a period apart rather than back to back - the driver "
                         "holds only one assembled sentence, so a burst would lose "
                         "the first. 0 disables GPS entirely.")
    ap.add_argument("--model", default="tiltrotor", help="JSBSim aircraft model name")
    ap.add_argument("--root", default=str(Path(__file__).parent / "jsbsim"),
                    help="JSBSim root dir (contains aircraft/, engine/)")
    ap.add_argument("--hover-throttle", type=float, default=0.5)
    ap.add_argument("--plan", default=None,
                    help="flight plan (YAML) to fly once the FC arms, e.g. "
                         "Scripts/sim/plans/yaw_step.yaml. The bridge exits with the "
                         "check summary when the plan finishes.")
    ap.add_argument("--keep-going", action="store_true",
                    help="on a gate violation, carry on flying instead of stopping. "
                         "Off by default: a departed vehicle makes every later phase "
                         "meaningless, and stopping puts the failure at the END of the "
                         "log instead of burying it under the rest of the run.")
    ap.add_argument("--gate-report", default="Build/sil_report.json", metavar="PATH",
                    help="write the per-phase gate results here as JSON, for diffing "
                         "one build against another. Empty string disables.")
    ap.add_argument("--gate-trace", default=None, metavar="PATH",
                    help="write a per-sample commanded-vs-truth CSV for every phase, "
                         "settle windows included. For CALIBRATION: it is what "
                         "separates a settle that is too short from an overshoot, "
                         "which the pass/fail summary cannot do.")
    ap.add_argument("--hold-until-armed", action=argparse.BooleanOptionalAction, default=True,
                    help="freeze the FDM at its initial condition until the FC arms, so "
                         "the model is not integrating ground contact for the ~20 s the "
                         "attitude estimate takes to settle")
    ap.add_argument("--arm-delay", type=float, default=2.0,
                    help="seconds to settle before raising the arm switch (the FC "
                         "holds the request until its own arming gate opens)")
    ap.add_argument("--gui-port", type=int, default=GUI_PORT, metavar="PORT",
                    help="publish sim state (FDM truth + FC telemetry + actuator "
                         f"commands) as JSON over UDP to {GUI_HOST}:PORT at "
                         f"{GUI_RATE:.0f} Hz, for `board.py gui`, and accept that "
                         "GUI's stick overrides in reply; 0 disables")
    ap.add_argument("--dry-run", action="store_true",
                    help="no JSBSim: emit a fixed 20deg-roll attitude to validate the link")
    ap.add_argument("--send-pid", metavar="AXIS:GAIN:VALUE", default=None,
                    help="send one shell set_pid command over the same link, e.g. "
                         "roll:kp:0.5 - exercises logging, sim frames and the shell together")
    ap.add_argument("--pid-delay", type=float, default=3.0,
                    help="seconds to wait before sending --send-pid")
    args = ap.parse_args(argv)
    resolve_ports(args)

    flight_plan = planlib.load(args.plan) if args.plan else None
    # A phases plan is a flight test, and --dry-run has no flight model to test
    # against. Refusing up front beats the alternative: every gate reporting
    # NOT RUN forty minutes later, which reads like a firmware failure.
    if flight_plan is not None and flight_plan.phases and args.dry_run:
        raise SystemExit(f"--dry-run cannot fly '{flight_plan.name}': its gates need "
                         f"FDM truth. Use a plain 'plan:' file to exercise the link.")

    pending_pid = None
    if args.send_pid:
        axis_s, gain_s, value_s = args.send_pid.split(":")
        pending_pid = (
            flapjack_pb2.Axis.Value(axis_s.upper()),
            flapjack_pb2.PidGainType.Value(gain_s.upper()),
            float(value_s),
        )

    ser = open_port(args.port, args.baud, timeout=0)
    parser = FrameParser()
    dt = 1.0 / args.rate

    # CRSF on its own wire drives Rx_Task -> Crsf_ProcessFrame -> Rc_Update, i.e.
    # the path the aircraft actually flies. There is no longer a shortcut past it.
    rc_ser = open_port(args.rc_port, args.rc_baud, timeout=0) if args.rc_port else None
    rc_period = 1.0 / args.rc_rate if args.rc_rate > 0 else 0.0
    next_rc = 0.0

    # Samples are pushed into the emulated parts rather than handed to the FC,
    # so a SIL run exercises the register maps, the SPI framing and the
    # chip-select timing instead of stepping over all three. Mag and baro share
    # SPI5 behind a multiplexer, so this also exercises bus arbitration between
    # two devices on separate chip selects.
    imu_ser = open_port(args.imu_port, args.baud, timeout=0) if args.imu_port else None
    mag_ser = open_port(args.mag_port, args.baud, timeout=0) if args.mag_port else None
    baro_ser = open_port(args.baro_port, args.baud, timeout=0) if args.baro_port else None

    baro_period = 1.0 / args.baro_rate if args.baro_rate > 0 else 0.0
    mag_period = 1.0 / args.mag_rate if args.mag_rate > 0 else 0.0
    next_baro = 0.0
    next_mag = 0.0

    # NMEA on its own wire drives Gps_DataReceivedHandler -> minmea ->
    # Gps_Update_, i.e. the path a real receiver drives. No sim-link shortcut.
    gps_ser = open_port(args.gps_port, args.gps_baud, timeout=0) if args.gps_port else None
    gps_period = 1.0 / args.gps_rate if args.gps_rate > 0 else 0.0
    next_gps = 0.0
    gps_send_rmc = False   # alternate GGA / RMC, half a period apart

    # Simulation time: the FDM advances exactly one dt per tick, so this is an
    # exact clock. Wall time only paces the sleep - pacing the plan off it would
    # let host load move where the sticks land.
    tick = 0
    sim_t = 0.0
    arm_sim_t = None          # sim time at which the FC armed; plan t=0
    checks = _Checks()
    gates = _Gates(flight_plan, keep_going=args.keep_going,
                   report_path=args.gate_report, trace_path=args.gate_trace)

    fdm = None
    if not args.dry_run:
        import jsbsim
        fdm = jsbsim.FGFDMExec(args.root)
        fdm.set_dt(dt)
        if not fdm.load_model(args.model):
            raise SystemExit(f"failed to load JSBSim model '{args.model}' from {args.root}")
        # Start near the ground, engines running.
        fdm["ic/h-sl-ft"] = 1.0
        fdm.run_ic()

    gui_sock = None
    if args.gui_port:
        gui_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Also the receive side, for the GUI's stick overrides. Never blocks:
        # the sim's pace must not depend on a viewer having something to say.
        gui_sock.setblocking(False)
    gui_period = 1.0 / GUI_RATE
    next_gui = 0.0
    # Latest FC output, held between publishes: the FC replies asynchronously,
    # so a frame type that did not arrive this tick is stale, not absent.
    last_servo = []
    last_motor = []
    last_tm = None
    # Sticks the GUI is flying with, and when they last arrived.
    gui_rc = None
    gui_rc_at = 0.0

    log_buf = b""
    t0 = time.perf_counter()
    next_tick = t0
    armed_sent = False
    hover_sent = False
    fc_armed = False
    last_print = t0

    print(f"[bridge] {'DRY-RUN' if args.dry_run else 'JSBSim'} on {args.port}@{args.baud}, {args.rate:.0f} Hz")
    if imu_ser is not None:
        print(f"[bridge] IMU samples on {args.imu_port}, {args.rate:.0f} Hz")
    else:
        print("[bridge] WARNING: no --imu-port, so the emulated BMI323 gets no samples "
              "and the FC never gets an attitude")
    if mag_ser is not None:
        print(f"[bridge] mag samples on {args.mag_port}, {args.mag_rate:.0f} Hz")
    else:
        print("[bridge] WARNING: no --mag-port, so the emulated MMC5983 gets no samples "
              "and the heading is unreferenced")
    if baro_ser is not None and baro_period:
        print(f"[bridge] baro samples on {args.baro_port}, {args.baro_rate:.0f} Hz")
    else:
        print("[bridge] WARNING: no --baro-port, so the emulated BMP390 gets no samples "
              "and the FC has no altitude")
    if rc_ser is not None:
        print(f"[bridge] CRSF RC on {args.rc_port}@{args.rc_baud}, {args.rc_rate:.0f} Hz")
    else:
        print("[bridge] WARNING: no --rc-port, so the FC receives no RC and will not arm")
    if gui_sock is not None:
        print(f"[bridge] GUI feed on udp://{GUI_HOST}:{args.gui_port}, {GUI_RATE:.0f} Hz "
              "(replies fly the sticks)")
    while True:
        now = time.perf_counter()

        # --- read FC -> PC frames (and the log text interleaved with them) ---
        data = ser.read(256)
        if data:
            for msg_id, payload in parser.feed(data):
                if msg_id == MSG_SERVO:
                    sc = sim_pb2.ServoCmd(); sc.ParseFromString(payload)
                    checks.servos(sc.angle)
                    last_servo = list(sc.angle)
                    if fdm:
                        for i, a in enumerate(sc.angle[:2]):
                            fdm[PROP_TILT[i]] = a
                elif msg_id == MSG_MOTOR:
                    mc = sim_pb2.MotorCmd(); mc.ParseFromString(payload)
                    checks.motors(mc.throttle)
                    last_motor = list(mc.throttle)
                    if fdm:
                        for i, thr in enumerate(mc.throttle[:2]):
                            fdm[PROP_THROTTLE[i]] = max(0.0, min(1.0, thr))
                elif msg_id == MSG_TELEMETRY:
                    tm = sim_pb2.Telemetry(); tm.ParseFromString(payload)
                    fc_armed = tm.armed
                    last_tm = tm
                    checks.baro_rx(tm)
                    checks.gps_rx(tm)
                    checks.nav_rx(tm, fdm)
                    if now - last_print > 0.5:
                        last_print = now
                        print(f"[FC] euler(deg)={tm.euler[0]:+6.1f} {tm.euler[1]:+6.1f} "
                              f"{tm.euler[2]:+6.1f} armed={tm.armed} imu#={tm.imu_count} "
                              f"rc={'up' if tm.rc_link_up else 'DOWN'} "
                              f"baro={tm.baro_pa:.1f}Pa#{tm.baro_count} "
                              f"gps={tm.gps_lat:.5f},{tm.gps_lon:.5f}"
                              f"/{tm.gps_sats}sat#{tm.gps_count}")
            # Everything that was not a frame is FC log text. It shares this
            # wire with the binary above; see serial_link.h. Held back until a
            # newline so the bridge's own prints cannot land mid-line.
            log_buf += parser.take_text()
            if b"\n" in log_buf:
                done, _, log_buf = log_buf.rpartition(b"\n")
                # Decoded as ASCII with errors ignored, deliberately: log output
                # is 7-bit ASCII by contract, so a non-ASCII byte here is a
                # resync artifact (e.g. connecting mid-frame), not a message.
                # It also keeps a stray byte from killing the bridge on a
                # console whose encoding cannot represent U+FFFD.
                line = done.decode("ascii", "ignore").strip("\r\n")
                if line:
                    print(line)

        # --- shell command over the same link, once the FC is up ---
        if pending_pid and now - t0 > args.pid_delay:
            axis, gain, value = pending_pid
            cmd = flapjack_pb2.Command()
            cmd.set_pid.axis = axis
            cmd.set_pid.gain = gain
            cmd.set_pid.value = value
            ser.write(frame(MSG_SHELL_CMD, cmd.SerializeToString()))
            print(f"[bridge] sent set_pid {args.send_pid}")
            pending_pid = None

        # --- step the model / synthesize attitude ---
        if args.dry_run:
            phi, theta, psi = math.radians(20.0), 0.0, 0.0
            p = q = r = udot = vdot = wdot = 0.0
        else:
            # Frozen at the initial condition until the FC arms. Otherwise the
            # model spends the ~20 s the attitude estimate needs to settle
            # integrating ground contact, which KnownIssues records as a route
            # to NaN. The FC still gets a valid level-and-still sample stream,
            # which is what lets the filter converge in the first place.
            held = not (fc_armed or not args.hold_until_armed)
            if not held:
                fdm.run()
            phi = fdm[PROP_PHI]; theta = fdm[PROP_THETA]; psi = fdm[PROP_PSI]
            p = fdm[PROP_P]; q = fdm[PROP_Q]; r = fdm[PROP_R]
            if held:
                # A model that has never been stepped has not resolved its gear
                # reaction yet, so it reports the only force it knows about:
                # gravity. wdot comes back as a full -32.2 ft/s^2, which is the
                # model saying the vehicle is in free fall while it sits parked
                # on its gear. Nothing physical reads that.
                #
                # A held vehicle is not accelerating, by definition, so the
                # honest linear acceleration during the hold is zero and the
                # accelerometer reads its 1 g. Taking the frozen model's word
                # instead fed the FC free fall for the whole pre-arm window,
                # which the altitude estimator faithfully integrated to -7.7 m/s
                # before the first real step rescued it.
                udot = vdot = wdot = 0.0
            else:
                udot = fdm[PROP_UDOT]; vdot = fdm[PROP_VDOT]; wdot = fdm[PROP_WDOT]

        accel, gyro, mag = synthesize_sensors(phi, theta, psi, p, q, r, udot, vdot, wdot)
        if imu_ser is not None:
            imu_ser.write(make_imu_push(accel, gyro))
        # Mag rides the sensor tick, but the part is what paces Mag_Task: it only
        # raises MEAS_M_DONE for a sample the driver has not already read, and
        # Mag_Task polls at 100 Hz, so pushing at 400 Hz costs the FC nothing.
        if mag_ser is not None and mag_period and now >= next_mag:
            next_mag = now + mag_period
            mag_ser.write(make_mag_push(mag))

        # --- barometer, at its own rate ---
        # Under --dry-run the pressure is swept rather than held: a constant
        # would satisfy the loopback check even if the FC latched one sample and
        # stopped updating, so a moving value is what makes the bring-up test
        # actually prove the chain is live. +/-200 Pa is about +/-17 m.
        if baro_period and now >= next_baro:
            next_baro = now + baro_period
            if args.dry_run:
                baro_pa = 101325.0 - 200.0 * math.sin(2.0 * math.pi * (now - t0) / 10.0)
                baro_temp_c = 15.0
            else:
                baro_pa = fdm[PROP_BARO_PA]
                baro_temp_c = fdm[PROP_BARO_TEMP_C]
            if baro_ser is not None:
                baro_ser.write(make_baro_push(baro_pa, baro_temp_c))
                checks.baro_tx(baro_pa)

        # --- GPS: real NMEA on its own wire ---
        # GGA and RMC go out half a fix period apart rather than back to back.
        # drivers/gps/gps.c holds exactly one assembled sentence, and Renode does
        # not pace bytes at the baud rate, so a burst of two would land together
        # and the second would overwrite the first before Gps_Task polled.
        if gps_ser is not None and gps_period and now >= next_gps:
            next_gps = now + gps_period / 2.0
            if args.dry_run:
                # Fixed position; the sweep that makes the baro check meaningful
                # is not needed here because the fix carries a satellite count
                # and both coordinates, so a stale echo cannot match by accident.
                gps_lat, gps_lon, gps_alt = 37.6189, -122.3750, 100.0
                gps_vn = gps_ve = 0.0
            else:
                gps_lat = fdm[PROP_GPS_LAT_RAD] * RAD2DEG
                gps_lon = fdm[PROP_GPS_LON_RAD] * RAD2DEG
                gps_alt = fdm[PROP_GPS_ALT_M]
                gps_vn = fdm[PROP_GPS_VN]
                gps_ve = fdm[PROP_GPS_VE]
            # A diverged FDM hands back NaN, and nmea's ddmm.mmmmm encoder does
            # int(degrees) on it - which raises, kills the bridge mid-run and
            # takes Checks.report() with it, hiding the actual failure behind a
            # crash in the sensor encoder. Go quiet instead: a receiver that has
            # stopped producing fixes is a real thing the FC already handles
            # (GPS_FIX_TIMEOUT_US -> Gps_HasFix), and staying alive means the run
            # still reaches its verdict.
            if not all(map(math.isfinite, (gps_lat, gps_lon, gps_alt, gps_vn, gps_ve))):
                if not checks.gps_nonfinite:
                    print("[bridge] WARNING: FDM position is not finite - GPS has gone "
                          "silent for the rest of this run (the FDM has diverged)")
                checks.gps_nonfinite += 1
            else:
                if gps_send_rmc:
                    speed = math.hypot(gps_vn, gps_ve)
                    course = math.degrees(math.atan2(gps_ve, gps_vn)) % 360.0
                    gps_ser.write(nmea.rmc(gps_lat, gps_lon, speed, course).encode("ascii"))
                else:
                    gps_ser.write(nmea.gga(gps_lat, gps_lon, gps_alt, sats=GPS_SATS).encode("ascii"))
                checks.gps_tx(gps_lat, gps_lon, GPS_SATS)
            gps_send_rmc = not gps_send_rmc

        # --- sticks from the GUI, when someone is flying this run by hand ---
        if gui_sock is not None:
            rc = poll_gui_rc(gui_sock)
            if rc is not None:
                if gui_rc is None:
                    print("[bridge] flying the GUI's sticks")
                gui_rc, gui_rc_at = rc, now
            elif gui_rc is not None and now - gui_rc_at > GUI_RC_TIMEOUT:
                print("[bridge] GUI sticks stale - back to the bridge's own")
                gui_rc = None

        # --- RC: mimic a real pilot's arming gesture ---
        # The FC refuses to arm unless throttle is at or below ARM_THROTTLE_MAX
        # (1100us) while AUX1 is high - a safety interlock so nothing spins up
        # the instant it arms. Raising throttle and the arm switch together (as
        # this used to) is rejected forever. Hold throttle down with the switch
        # up, wait for the FC to report armed, and only then go to hover.
        # The FC owns the arming interlock: it refuses to arm until the attitude
        # estimate has stopped moving and throttle is at or below
        # ARM_THROTTLE_MAX (1100us), and it holds a raised switch as a standing
        # request until its own gate opens (tasks/mission/mission.c). So the
        # bridge just behaves like a pilot - throttle down, switch up, wait.
        if not fc_armed:
            # Pre-arm hold. The plan clock has not started - arming is a barrier,
            # not a timestamp, because the FC's attitude-settle gate takes an
            # amount of time that is not fixed run to run.
            if now - t0 < args.arm_delay:
                channels = rc_channels(0.0, armed=False)        # settle, disarmed
            else:
                if not armed_sent:
                    print("[bridge] arm switch high, throttle low - waiting for FC to arm")
                    armed_sent = True
                channels = rc_channels(0.0, armed=True)         # throttle LOW + AUX1 high
        else:
            if arm_sim_t is None:
                arm_sim_t = sim_t
                what = f"flying plan '{flight_plan.name}'" if flight_plan else "going to hover throttle"
                print(f"[bridge] FC armed at sim t={sim_t:.1f}s - {what}")
                hover_sent = True
            plan_t = sim_t - arm_sim_t
            if flight_plan is not None:
                if plan_t > flight_plan.total_time:
                    print(f"[bridge] plan '{flight_plan.name}' complete at t={plan_t:.1f}s")
                    return checks.report(armed_ever=True) | gates.report()
                channels = flight_plan.channels_at(plan_t)
                # Gated against TRUTH, and against the plan's own commanded
                # value - never against the FC's echo of either. The FC's own
                # estimate rides along as `est` for the est_* gates, which are
                # the one place the two are compared head to head.
                est_euler = None
                if last_tm is not None and (last_tm.nav_valid & NAV_VALID_ATTITUDE):
                    est_euler = (last_tm.euler[0], last_tm.euler[1], last_tm.euler[2])
                if fdm is not None:
                    gates.update(plan_t, channels, {
                        "roll_deg": phi * RAD2DEG,
                        "pitch_deg": theta * RAD2DEG,
                        "yaw_rate_dps": r * RAD2DEG,
                        "climb_mps": fdm["velocities/h-dot-fps"] * FT2M,
                        "alt_m": fdm["position/h-sl-meters"],
                        "heading_deg": psi * RAD2DEG,
                        # Body lateral/vertical acceleration, for reconstructing
                        # what the accelerometer sees. An accel-referenced filter
                        # can only ever measure the APPARENT vertical, so this is
                        # what separates "the estimate is broken" from "the accel
                        # cannot see bank while the vehicle is translating".
                        "vdot_mps2": vdot * FT2M,
                        "wdot_mps2": wdot * FT2M,
                    }, est=est_euler)
                    if gates.abort:
                        return checks.report(armed_ever=True) | gates.report()
            else:
                channels = rc_channels(args.hover_throttle, armed=True)

        # Last word, over the hover hold and over a plan alike: the sticks are
        # whatever the pilot is holding, and a GUI attached to a plan run is a
        # deliberate act. Only the four sticks move - AUX1 stays as computed
        # above, so arming and disarming still follow the FC's own interlock.
        if gui_rc is not None:
            apply_gui_rc(channels, gui_rc)

        # Paced to a realistic link rate rather than the sensor rate: one whole
        # frame per write, never split, so the FC's length-driven deframer sees
        # exactly what a receiver would put on the wire.
        if args.rc_stop is not None and rc_ser is not None and now - t0 >= args.rc_stop:
            print(f"[bridge] cutting CRSF at t={now - t0:.1f}s - receiver failsafe")
            rc_ser.close()
            rc_ser = None

        if args.gps_stop is not None and gps_ser is not None and now - t0 >= args.gps_stop:
            print(f"[bridge] cutting NMEA at t={now - t0:.1f}s - dead receiver")
            gps_ser.close()
            gps_ser = None
            checks.gps_stopped = True

        if rc_ser is not None and now >= next_rc:
            next_rc = now + rc_period
            rc_ser.write(crsf.rc_frame(channels))

        if fdm is not None:
            checks.state(fdm["position/h-agl-ft"], (phi, theta, psi), fc_armed, sim_t)

        # --- publish state to the flight GUI, on its own wire ---
        if gui_sock is not None and now >= next_gui:
            next_gui = now + gui_period
            # Truth altitude is referenced to the FC's own pressure datum so it
            # shares a zero with the estimate the GUI plots it against. Before
            # that datum exists the FDM is still frozen at its IC, so ground
            # level is both the honest answer and the same one the datum will be.
            h_sl = fdm["position/h-sl-meters"] if fdm is not None else 0.0
            datum = checks.nav_alt_datum_m
            truth_alt = h_sl - (datum if datum is not None else h_sl)
            # A datagram nobody is listening for draws an ICMP port-unreachable,
            # which Winsock then reports as an error on the NEXT send. The GUI
            # not being up is the normal case, so drop the packet rather than
            # let a viewer end a flight.
            try:
                gui_sock.sendto(
                    make_gui_packet(sim_t, (phi, theta, psi), truth_alt,
                                    last_tm, last_servo, last_motor,
                                    (fdm["velocities/v-north-fps"] * FT2M,
                                     fdm["velocities/v-east-fps"] * FT2M)
                                    if fdm is not None else None),
                    (GUI_HOST, args.gui_port))
            except OSError:
                pass

        # --- pace to real time ---
        tick += 1
        sim_t = tick * dt
        next_tick += dt
        sleep = next_tick - time.perf_counter()
        if sleep > 0:
            time.sleep(sleep)
        else:
            next_tick = time.perf_counter()  # fell behind; resync


if __name__ == "__main__":
    sys.exit(main())
