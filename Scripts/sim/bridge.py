#!/usr/bin/env python3
"""JSBSim <-> Flapjack HIL bridge.

Streams synthesized IMU+mag to the flight controller over the (binary) debug
UART, injects a fixed "armed + hover" RC command at startup, and applies the
servo/motor commands the FC sends back to a JSBSim tilt-rotor model.

Timing is free-running real-time: sensors are streamed at a fixed rate and the
FDM is stepped to wall-clock; the FC consumes the latest sample and replies
asynchronously.

Wire format (matches Firmware/drivers/sim_link/sim_link.c):
    [0xAA][0x55][msg_id][len][payload(protobuf)][crc8]
    crc8: poly 0x07, init 0x00, over (msg_id, len, payload).

Run the sensor/link path WITHOUT a working FDM first:
    python bridge.py --port COM7 --dry-run
Then the full sim:
    python bridge.py --port COM7
"""

import argparse
import math
import time
from pathlib import Path

from proto import sim_pb2
from link.framing import frame, FrameParser
from link.serial_io import open_port

# --- frame ids (keep in sync with sim_link.h) --------------------------------
MSG_SENSOR = 1
MSG_RC = 2
MSG_SERVO = 3
MSG_MOTOR = 4
MSG_TELEMETRY = 5

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


def make_rc_frame(throttle_norm: float, armed: bool) -> bytes:
    ch = [RC_MID] * 16
    ch[CH_THROTTLE] = int(RC_MIN + throttle_norm * (RC_MAX - RC_MIN))
    ch[CH_AUX1] = RC_MAX if armed else RC_MIN  # arm switch high
    msg = sim_pb2.RcInput(channels=ch)
    return frame(MSG_RC, msg.SerializeToString())


def main(argv=None):
    ap = argparse.ArgumentParser(prog="board.py sim", description="JSBSim <-> Flapjack HIL bridge")
    ap.add_argument("--port", required=True, help="serial port, e.g. COM7 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--rate", type=float, default=400.0, help="sensor stream rate (Hz)")
    ap.add_argument("--model", default="tiltrotor", help="JSBSim aircraft model name")
    ap.add_argument("--root", default=str(Path(__file__).parent / "jsbsim"),
                    help="JSBSim root dir (contains aircraft/, engine/)")
    ap.add_argument("--hover-throttle", type=float, default=0.5)
    ap.add_argument("--arm-delay", type=float, default=2.0,
                    help="seconds to settle before raising the arm switch (the FC "
                         "holds the request until its own arming gate opens)")
    ap.add_argument("--dry-run", action="store_true",
                    help="no JSBSim: emit a fixed 20deg-roll attitude to validate the link")
    args = ap.parse_args(argv)

    ser = open_port(args.port, args.baud, timeout=0)
    parser = FrameParser()
    dt = 1.0 / args.rate

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

    t0 = time.perf_counter()
    next_tick = t0
    armed_sent = False
    hover_sent = False
    fc_armed = False
    last_print = t0

    print(f"[bridge] {'DRY-RUN' if args.dry_run else 'JSBSim'} on {args.port}@{args.baud}, {args.rate:.0f} Hz")
    while True:
        now = time.perf_counter()

        # --- read FC -> PC frames ---
        data = ser.read(256)
        if data:
            for msg_id, payload in parser.feed(data):
                if msg_id == MSG_SERVO:
                    sc = sim_pb2.ServoCmd(); sc.ParseFromString(payload)
                    if fdm:
                        for i, a in enumerate(sc.angle[:2]):
                            fdm[PROP_TILT[i]] = a
                elif msg_id == MSG_MOTOR:
                    mc = sim_pb2.MotorCmd(); mc.ParseFromString(payload)
                    if fdm:
                        for i, thr in enumerate(mc.throttle[:2]):
                            fdm[PROP_THROTTLE[i]] = max(0.0, min(1.0, thr))
                elif msg_id == MSG_TELEMETRY:
                    tm = sim_pb2.Telemetry(); tm.ParseFromString(payload)
                    fc_armed = tm.armed
                    if now - last_print > 0.5:
                        last_print = now
                        print(f"[FC] euler(deg)={tm.euler[0]:+6.1f} {tm.euler[1]:+6.1f} "
                              f"{tm.euler[2]:+6.1f} armed={tm.armed} imu#={tm.imu_count}")

        # --- step the model / synthesize attitude ---
        if args.dry_run:
            phi, theta, psi = math.radians(20.0), 0.0, 0.0
            p = q = r = udot = vdot = wdot = 0.0
        else:
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
        if now - t0 < args.arm_delay:
            ser.write(make_rc_frame(0.0, armed=False))          # settle, disarmed
        elif not fc_armed:
            if not armed_sent:
                print("[bridge] arm switch high, throttle low - waiting for FC to arm")
                armed_sent = True
            ser.write(make_rc_frame(0.0, armed=True))           # throttle LOW + AUX1 high
        else:
            if not hover_sent:
                print("[bridge] FC armed - going to hover throttle")
                hover_sent = True
            ser.write(make_rc_frame(args.hover_throttle, armed=True))

        # --- pace to real time ---
        next_tick += dt
        sleep = next_tick - time.perf_counter()
        if sleep > 0:
            time.sleep(sleep)
        else:
            next_tick = time.perf_counter()  # fell behind; resync


if __name__ == "__main__":
    main()
