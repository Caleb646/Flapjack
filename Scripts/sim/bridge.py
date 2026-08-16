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
import math
import sys
import time
from pathlib import Path

from proto import flapjack_pb2, sim_pb2
from link.framing import frame, FrameParser
from link.serial_io import open_port
from sim import crsf
from sim import plan as planlib

# --- frame ids (keep in sync with sim_link.h / serial_link.h) ----------------
MSG_SENSOR = 1
# 2 is retired (was RcInput). RC reaches the FC as real CRSF on its own wire.
MSG_SERVO = 3
MSG_MOTOR = 4
MSG_TELEMETRY = 5
MSG_SHELL_CMD = 6

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
# Inputs we drive: throttle [0..1] and thrust-vector (tilt) gimbal pitch [rad].
PROP_THROTTLE = ["fcs/throttle-cmd-norm[0]", "fcs/throttle-cmd-norm[1]"]
# Tilt is passed through unmodified: the FDM defines 0 rad as rotors-up, which
# matches the FC's neutral servo angle (drivers/servo/sim.c, centre 1500 us).
PROP_TILT = ["fcs/tilt-cmd-rad[0]", "fcs/tilt-cmd-rad[1]"]

G = 9.81
FT2M = 0.3048
RAD2DEG = 180.0 / math.pi

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


def make_sensor_frame(accel, gyro, mag) -> bytes:
    msg = sim_pb2.SensorData(accel=accel, gyro=gyro, mag=mag)
    return frame(MSG_SENSOR, msg.SerializeToString())


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

        print()
        print("[checks] ---------------------------------------------")
        print(f"[checks] samples      {self.samples}")
        print(f"[checks] motor range  {self.motor_min:.3f} .. {self.motor_max:.3f}")
        print(f"[checks] servo range  {self.servo_min:+.3f} .. {self.servo_max:+.3f} rad")
        print(f"[checks] altitude     {self.alt_min:.1f} .. {self.alt_max:.1f} ft")
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
                    if now - last_print > 0.5:
                        last_print = now
                        print(f"[FC] euler(deg)={tm.euler[0]:+6.1f} {tm.euler[1]:+6.1f} "
                              f"{tm.euler[2]:+6.1f} armed={tm.armed} imu#={tm.imu_count} "
                              f"rc={'up' if tm.rc_link_up else 'DOWN'}")
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
