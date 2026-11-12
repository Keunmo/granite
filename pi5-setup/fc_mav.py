#!/usr/bin/env python3
"""Read and set the FC UART parameters, and measure the FC IMU rate.

The FC is a UVify V6X with ArduPilot. The tool uses pymavlink. When Python
cannot import pymavlink, the tool extracts the wheels of resources/python
(pymavlink 2.4.50 for CPython 3.12 aarch64 = Ubuntu 24.04 on the Pi 5) to
~/.cache/pi5-setup/python and imports from there. No internet connection is
necessary.

Commands:

    params  Read SERIAL3_PROTOCOL, SERIAL3_BAUD, SERIAL3_OPTIONS and
            BRD_SER3_RTSCTS, and show the target values.
    apply   Write the target values, read them back, and reboot the FC
            (--reboot). The FC uses the new values only after a reboot.
    imu     Request HIGHRES_IMU at --rate Hz and measure the received rate.

Connections:

    --conn /dev/ttyACM0                 FC USB port (for params and apply).
                                        Stop a router that uses this port.
    --conn udpin:127.0.0.1:14601        mavlink-routerd endpoint (for imu).
                                        Stop capture and the VIO first.
"""

import argparse
import glob
import os
import platform
import sys
import time
import zipfile

os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", "ardupilotmega")


def add_bundled_wheels():
    """Extract the wheels of resources/python once and add them to sys.path."""
    here = os.path.dirname(os.path.realpath(__file__))
    wheels = sorted(glob.glob(os.path.join(here, "resources", "python", "*.whl")))
    tag = f"cp{sys.version_info.major}{sys.version_info.minor}"
    arch = platform.machine()
    if not wheels or not all(tag in os.path.basename(w) and arch in os.path.basename(w)
                             for w in wheels):
        return False
    site = os.path.join(os.path.expanduser("~"), ".cache", "pi5-setup", "python", tag)
    stamp = os.path.join(site, ".wheels")
    names = "\n".join(os.path.basename(w) for w in wheels)
    if not os.path.exists(stamp) or open(stamp).read() != names:
        os.makedirs(site, exist_ok=True)
        for w in wheels:
            with zipfile.ZipFile(w) as z:
                z.extractall(site)
        with open(stamp, "w") as f:
            f.write(names)
    sys.path.insert(0, site)
    return True


try:
    from pymavlink import mavutil
except ImportError:
    if not add_bundled_wheels():
        sys.exit("pymavlink is not installed, and resources/python has no wheel for "
                 f"Python {sys.version_info.major}.{sys.version_info.minor} "
                 f"{platform.machine()}")
    from pymavlink import mavutil

MAV_AUTOPILOT_ARDUPILOTMEGA = 3
MSG_ID_HIGHRES_IMU = 105


def target_params(flow_control):
    # SERIAL3 = FC UART2 connector (USART2). See docs/fc-v6x-rpi5-uart-wiring.md.
    return [
        ("SERIAL3_PROTOCOL", 2),  # MAVLink2
        ("SERIAL3_BAUD", 921),  # 921600 baud
        ("SERIAL3_OPTIONS", 0),  # 4 = half-duplex: TX and RX on one wire
        ("BRD_SER3_RTSCTS", 1 if flow_control else 0),
    ]


def connect(args):
    print(f"connect {args.conn}")
    m = mavutil.mavlink_connection(args.conn, baud=args.baud,
                                   source_system=args.sysid,
                                   source_component=args.compid)
    hb = m.wait_heartbeat(timeout=args.timeout)
    if hb is None:
        sys.exit(f"no HEARTBEAT in {args.timeout} s")
    # Use the HEARTBEAT of the autopilot, not of a GCS on the same link.
    while hb.get_srcComponent() != 1 or hb.autopilot == 8:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=args.timeout)
        if hb is None:
            sys.exit("no HEARTBEAT from the autopilot component")
    m.target_system = hb.get_srcSystem()
    m.target_component = hb.get_srcComponent()
    print(f"HEARTBEAT sys {m.target_system} comp {m.target_component} "
          f"autopilot {hb.autopilot} type {hb.type}")
    if hb.autopilot != MAV_AUTOPILOT_ARDUPILOTMEGA:
        sys.exit("the FC is not ArduPilot (autopilot != 3). The parameter "
                 "names of this tool are for ArduPilot.")
    return m


def param_name(msg):
    pid = msg.param_id
    if isinstance(pid, bytes):
        pid = pid.decode(errors="replace")
    return pid.rstrip("\x00")


def read_param(m, name, timeout, tries=3):
    for _ in range(tries):
        m.param_fetch_one(name)
        end = time.time() + timeout
        while time.time() < end:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True,
                               timeout=end - time.time())
            if msg is not None and param_name(msg) == name:
                return msg
    return None


def write_param(m, name, value, ptype, timeout, tries=3):
    for _ in range(tries):
        m.param_set_send(name, float(value), parm_type=ptype)
        end = time.time() + timeout
        while time.time() < end:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True,
                               timeout=end - time.time())
            if msg is not None and param_name(msg) == name:
                return msg
    return None


def cmd_params(m, args):
    bad = 0
    for name, want in target_params(not args.no_flow_control):
        msg = read_param(m, name, args.timeout)
        if msg is None:
            print(f"{name:18s} no reply")
            bad += 1
            continue
        have = int(round(msg.param_value))
        mark = "OK" if have == want else "CHANGE"
        bad += have != want
        print(f"{name:18s} {have:>6d}  target {want:>6d}  {mark}")
    return bad


def cmd_apply(m, args):
    plan = []
    for name, want in target_params(not args.no_flow_control):
        msg = read_param(m, name, args.timeout)
        if msg is None:
            sys.exit(f"{name}: no reply. Nothing is written.")
        plan.append((name, int(round(msg.param_value)), want, msg.param_type))
    for name, have, want, _ in plan:
        print(f"{name:18s} {have:>6d} -> {want:>6d}")
    if all(have == want for _, have, want, _ in plan):
        print("all values are correct. Nothing is written.")
    else:
        if not args.yes:
            if input("write these values to the FC? [y/N] ").strip().lower() != "y":
                sys.exit("cancelled")
        for name, have, want, ptype in plan:
            if have == want:
                continue
            msg = write_param(m, name, want, ptype, args.timeout)
            if msg is None or int(round(msg.param_value)) != want:
                sys.exit(f"{name}: write failed")
            print(f"{name:18s} = {want}")
    if args.reboot:
        print("reboot the FC")
        m.reboot_autopilot()
    else:
        print("reboot the FC to use the new values (apply --reboot, or power cycle)")


def cmd_imu(m, args):
    interval_us = int(1e6 / args.rate) if args.rate > 0 else 0
    if args.rate > 0:
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                MSG_ID_HIGHRES_IMU, interval_us, 0, 0, 0, 0, 0)
        ack, end = None, time.time() + args.timeout
        while time.time() < end:
            msg = m.recv_match(type="COMMAND_ACK", blocking=True,
                               timeout=end - time.time())
            if msg is not None and msg.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
                ack = msg
                break
        result = ack.result if ack is not None else "no reply"
        print(f"SET_MESSAGE_INTERVAL HIGHRES_IMU {args.rate} Hz: "
              f"COMMAND_ACK {result} (0 = accepted)")
    stamps = []
    end = time.time() + args.seconds
    while time.time() < end:
        msg = m.recv_match(type="HIGHRES_IMU", blocking=True, timeout=1.0)
        if msg is not None:
            stamps.append(msg.time_usec)
    if len(stamps) < 2:
        print(f"HIGHRES_IMU: {len(stamps)} messages in {args.seconds} s")
        return 1
    span = (stamps[-1] - stamps[0]) * 1e-6
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    rate = (len(stamps) - 1) / span if span > 0 else 0.0
    print(f"HIGHRES_IMU: {len(stamps)} messages, {rate:.1f} Hz (FC time), "
          f"max gap {max(gaps) * 1e-3:.1f} ms")
    return 0 if args.rate <= 0 or rate >= 0.95 * args.rate else 1


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--conn", default="/dev/ttyACM0", help="pymavlink connection string")
    p.add_argument("--baud", type=int, default=115200, help="serial baud (USB ignores it)")
    p.add_argument("--sysid", type=int, default=255, help="source system id")
    p.add_argument("--compid", type=int, default=190, help="source component id")
    p.add_argument("--timeout", type=float, default=5.0, help="reply timeout [s]")
    p.add_argument("--no-flow-control", action="store_true",
                   help="target BRD_SER3_RTSCTS = 0 (only TX, RX and GND are wired)")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("params", help="read the UART parameters")
    a = sub.add_parser("apply", help="write the UART parameters")
    a.add_argument("--yes", action="store_true", help="do not ask before the write")
    a.add_argument("--reboot", action="store_true", help="reboot the FC after the write")
    i = sub.add_parser("imu", help="measure the HIGHRES_IMU rate")
    i.add_argument("--rate", type=float, default=200.0, help="requested rate [Hz], 0 = no request")
    i.add_argument("--seconds", type=float, default=10.0, help="measurement time [s]")
    args = p.parse_args()

    m = connect(args)
    if args.cmd == "params":
        return 1 if cmd_params(m, args) else 0
    if args.cmd == "apply":
        cmd_apply(m, args)
        return 0
    return cmd_imu(m, args)


if __name__ == "__main__":
    sys.exit(main())
