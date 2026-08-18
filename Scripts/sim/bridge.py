#!/usr/bin/env python3
"""JSBSim <-> Flapjack HIL bridge.

Streams synthesized IMU+mag to the flight controller over the (binary) debug
UART, plays a pilot's arm-and-hover stick sequence as real CRSF frames on the
FC's separate RX UART, and applies the servo/motor commands the FC sends back to
a JSBSim tilt-rotor model.

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
import math
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
MSG_SENSOR = 1
# 2 is retired (was RcInput). RC reaches the FC as real CRSF on its own wire.
MSG_SERVO = 3
MSG_MOTOR = 4
MSG_TELEMETRY = 5
MSG_SHELL_CMD = 6
MSG_BARO = 7

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

# Loopback tolerance on an echoed GPS coordinate, in degrees. ~1.1 cm: well
# below NMEA's own ~1.9 cm quantisation step (so it cannot mask a wrong field)
# and well below a float32 narrowing, which would show as ~2.2e-6 deg.
GPS_COORD_EPS = 1e-7

# Satellite count the synthesized fix reports. Constant, and echoed back in
# Telemetry, so it doubles as a cheap check that GGA's integer fields survive
# the trip and not just its coordinates.
GPS_SATS = 9

# Reference Earth field in NED (unit), ~60 deg inclination, 0 declination.
_INCL = math.radians(60.0)
B_NED = (math.cos(_INCL), 0.0, math.sin(_INCL))


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
    """Return (accel[3] m/s^2, gyro[3] deg/s, mag[3] unit) in body FRD.

    Accelerometer = specific force in the FC's 'up-positive' convention, so
    LEVEL & STILL -> (0, 0, +9.81). If your FDM's signs differ, this is the one
    place to flip.
    """
    R = dcm_body_from_ned(phi, theta, psi)
    # Gravity projected into body, minus body linear acceleration.
    g_body = (-G * math.sin(theta), G * math.sin(phi) * math.cos(theta), G * math.cos(phi) * math.cos(theta))
    a_lin = (udot * FT2M, vdot * FT2M, wdot * FT2M)
    accel = [g_body[k] - a_lin[k] for k in range(3)]
    gyro = [p * RAD2DEG, q * RAD2DEG, r * RAD2DEG]
    mag = [sum(R[k][j] * B_NED[j] for j in range(3)) for k in range(3)]
    norm = math.sqrt(sum(c * c for c in mag)) or 1.0
    mag = [c / norm for c in mag]
    return accel, gyro, mag


def as_f32(x: float) -> float:
    """Round to IEEE 754 binary32, i.e. to exactly what a protobuf float carries.

    The loopback check compares the FC's echo against what was sent; both hops
    are binary32, so rounding here makes that comparison exact instead of
    approximate.
    """
    return struct.unpack("f", struct.pack("f", x))[0]


def make_sensor_frame(accel, gyro, mag) -> bytes:
    msg = sim_pb2.SensorData(accel=accel, gyro=gyro, mag=mag)
    return frame(MSG_SENSOR, msg.SerializeToString())


def make_baro_frame(pressure_pa, temperature_c) -> bytes:
    msg = sim_pb2.BaroData(pressure_pa=pressure_pa, temperature_c=temperature_c)
    return frame(MSG_BARO, msg.SerializeToString())


def rc_channels(throttle_norm: float, armed: bool) -> list:
    """The 16 stick positions, in microseconds, a pilot would be holding."""
    ch = [RC_MID] * 16
    ch[CH_THROTTLE] = int(RC_MIN + throttle_norm * (RC_MAX - RC_MIN))
    ch[CH_AUX1] = RC_MAX if armed else RC_MIN  # arm switch high
    return ch


class _Checks:
    """Physics-level invariants, gathered as the run goes.

    Deliberately not a golden trajectory. KnownIssues 1.13 records that the
    manual mode is a *rate* loop, so the attitude a run settles at is not
    repeatable - asserting on it produces false regressions. These are the
    things that are true of any correct flight, and they are exactly the class
    of fault the SIL has actually caught (motors pinned at 1.000, servo commands
    20x out of range).
    """

    def __init__(self):
        self.nan = 0
        self.motor_min, self.motor_max = 1e9, -1e9
        self.servo_min, self.servo_max = 1e9, -1e9
        self.alt_min, self.alt_max = 1e9, -1e9
        self.motor_pinned = 0
        self.samples = 0

        # --- sensor loopback (see sim.proto's Telemetry comment) -------------
        # Recent values as actually put on the wire, i.e. rounded to float32 by
        # protobuf. The FC echoes what it decoded, and both directions are IEEE
        # 754 binary32, so a correct chain reproduces one of these BIT-EXACTLY -
        # no tolerance required, and any mangling (wrong field, lost precision,
        # stale slot, discarded parse) shows up as a miss.
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

    def motors(self, values):
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
        """Record a BaroData frame as float32, the way the wire carries it."""
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
        if telemetry.baro_pa not in self.baro_sent:
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

    def state(self, alt, euler):
        self.samples += 1
        if alt != alt or any(e != e for e in euler):
            self.nan += 1
            return
        self.alt_min = min(self.alt_min, alt)
        self.alt_max = max(self.alt_max, alt)

    def report(self, armed_ever: bool) -> int:
        """Print a pass/fail summary; return a process exit code."""
        fails = []
        if not armed_ever:
            fails.append("FC never armed")
        if self.nan:
            fails.append(f"{self.nan} NaN samples")
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
                fails.append(f"baro: {self.baro_frames} frames sent, FC decoded none "
                             f"(check the baro task / SIM_MSG_BARO handler)")
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
        print(f"[checks] NaN samples  {self.nan}")
        for f in fails:
            print(f"[checks] FAIL: {f}")
        print("[checks] " + ("PASS" if not fails else f"FAIL ({len(fails)})"))
        return 1 if fails else 0


def main(argv=None):
    ap = argparse.ArgumentParser(prog="board.py sim", description="JSBSim <-> Flapjack HIL bridge")
    ap.add_argument("--port", required=True, help="serial port, e.g. COM7 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--rc-port", default=None,
                    help="port carrying CRSF frames to the FC's RX UART (USART3), e.g. "
                         "socket://localhost:4001. This is the only RC path - without "
                         "it the FC gets no sticks and will never arm.")
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
    ap.add_argument("--baro-rate", type=float, default=50.0,
                    help="BaroData frame rate (Hz). Its own frame at its own rate "
                         "rather than riding the 400 Hz sensor stream - a real "
                         "barometer is a 10-50 Hz part. 0 disables baro entirely.")
    ap.add_argument("--gps-port", default=None,
                    help="port carrying NMEA sentences to the FC's GPS UART (USART2), "
                         "e.g. socket://localhost:4002. Like --rc-port this is the "
                         "only GPS path: the sim link carries no GPS frame, so that "
                         "the SIL exercises the real byte assembler and minmea.")
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
    ap.add_argument("--hold-until-armed", action=argparse.BooleanOptionalAction, default=True,
                    help="freeze the FDM at its initial condition until the FC arms, so "
                         "the model is not integrating ground contact for the ~20 s the "
                         "attitude estimate takes to settle")
    ap.add_argument("--arm-delay", type=float, default=2.0,
                    help="seconds to settle before raising the arm switch (the FC "
                         "holds the request until its own arming gate opens)")
    ap.add_argument("--dry-run", action="store_true",
                    help="no JSBSim: emit a fixed 20deg-roll attitude to validate the link")
    ap.add_argument("--send-pid", metavar="AXIS:GAIN:VALUE", default=None,
                    help="send one shell set_pid command over the same link, e.g. "
                         "roll:kp:0.5 - exercises logging, sim frames and the shell together")
    ap.add_argument("--pid-delay", type=float, default=3.0,
                    help="seconds to wait before sending --send-pid")
    args = ap.parse_args(argv)

    flight_plan = planlib.load(args.plan) if args.plan else None

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

    baro_period = 1.0 / args.baro_rate if args.baro_rate > 0 else 0.0
    next_baro = 0.0

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

    log_buf = b""
    t0 = time.perf_counter()
    next_tick = t0
    armed_sent = False
    hover_sent = False
    fc_armed = False
    last_print = t0

    print(f"[bridge] {'DRY-RUN' if args.dry_run else 'JSBSim'} on {args.port}@{args.baud}, {args.rate:.0f} Hz")
    if rc_ser is not None:
        print(f"[bridge] CRSF RC on {args.rc_port}@{args.rc_baud}, {args.rc_rate:.0f} Hz")
    else:
        print("[bridge] WARNING: no --rc-port, so the FC receives no RC and will not arm")
    while True:
        now = time.perf_counter()

        # --- read FC -> PC frames (and the log text interleaved with them) ---
        data = ser.read(256)
        if data:
            for msg_id, payload in parser.feed(data):
                if msg_id == MSG_SERVO:
                    sc = sim_pb2.ServoCmd(); sc.ParseFromString(payload)
                    checks.servos(sc.angle)
                    if fdm:
                        for i, a in enumerate(sc.angle[:2]):
                            fdm[PROP_TILT[i]] = a
                elif msg_id == MSG_MOTOR:
                    mc = sim_pb2.MotorCmd(); mc.ParseFromString(payload)
                    checks.motors(mc.throttle)
                    if fdm:
                        for i, thr in enumerate(mc.throttle[:2]):
                            fdm[PROP_THROTTLE[i]] = max(0.0, min(1.0, thr))
                elif msg_id == MSG_TELEMETRY:
                    tm = sim_pb2.Telemetry(); tm.ParseFromString(payload)
                    fc_armed = tm.armed
                    checks.baro_rx(tm)
                    checks.gps_rx(tm)
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
            if fc_armed or not args.hold_until_armed:
                fdm.run()
            phi = fdm[PROP_PHI]; theta = fdm[PROP_THETA]; psi = fdm[PROP_PSI]
            p = fdm[PROP_P]; q = fdm[PROP_Q]; r = fdm[PROP_R]
            udot = fdm[PROP_UDOT]; vdot = fdm[PROP_VDOT]; wdot = fdm[PROP_WDOT]

        accel, gyro, mag = synthesize_sensors(phi, theta, psi, p, q, r, udot, vdot, wdot)
        ser.write(make_sensor_frame(accel, gyro, mag))

        # --- barometer, on its own frame at its own rate ---
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
            ser.write(make_baro_frame(baro_pa, baro_temp_c))
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
            if gps_send_rmc:
                speed = math.hypot(gps_vn, gps_ve)
                course = math.degrees(math.atan2(gps_ve, gps_vn)) % 360.0
                gps_ser.write(nmea.rmc(gps_lat, gps_lon, speed, course).encode("ascii"))
            else:
                gps_ser.write(nmea.gga(gps_lat, gps_lon, gps_alt, sats=GPS_SATS).encode("ascii"))
            checks.gps_tx(gps_lat, gps_lon, GPS_SATS)
            gps_send_rmc = not gps_send_rmc

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
                    return checks.report(armed_ever=True)
                channels = flight_plan.channels_at(plan_t)
            else:
                channels = rc_channels(args.hover_throttle, armed=True)

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
            checks.state(fdm["position/h-agl-ft"], (phi, theta, psi))

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
