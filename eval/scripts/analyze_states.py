#!/usr/bin/env python3
"""Time series of one Granite run (granite_states.csv) against the GT: speed ratio |v_est| / |v_gt|, tilt error
(angle between estimated and true gravity in the IMU frame), accelerometer and gyroscope bias error, support.

Usage: analyze_states.py RUN_DIR [--at 0,1,2,5,10,20,30,40,50] (seconds after the first state)
"""
import argparse
import math
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from metrics import quat, rotvec, IMU2RIG_ROT  # noqa: E402

RAW = os.path.expanduser("~/multipleye_docker/data/unreal_data_city_sky")


def load(run):
    rows = np.genfromtxt(os.path.join(run, "granite_states.csv"), delimiter=",", names=True, dtype=None,
                         encoding=None)
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--at", default="0,0.5,1,2,3,5,10,15,20,30,40,50")
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()
    s = load(a.run)
    g = np.loadtxt(os.path.join(RAW, "gt_poses.txt"))
    gt_t = g[:, 7] * 1e-9
    t0_frame = gt_t[0]
    gp = g[:, 4:7]
    gv = np.gradient(gp, gt_t, axis=0)
    ib = np.loadtxt(os.path.join(RAW, "imu_gt.csv"), delimiter=",", skiprows=1)
    R_RI = rotvec(IMU2RIG_ROT)
    t = s["t_ns"] * 1e-9
    v = np.column_stack([s["vx"], s["vy"], s["vz"]])
    vg = np.column_stack([np.interp(t, gt_t, gv[:, k]) for k in range(3)])
    ba = np.column_stack([s["bax"], s["bay"], s["baz"]])
    bg = np.column_stack([s["bgx"], s["bgy"], s["bgz"]])
    bat = np.column_stack([np.interp(t, ib[:, 0] * 1e-9, ib[:, k]) for k in (1, 2, 3)])
    bgt = np.column_stack([np.interp(t, ib[:, 0] * 1e-9, ib[:, k]) for k in (4, 5, 6)])
    tilt = []
    for i in range(len(t)):
        Rwi = quat([s["qx"][i], s["qy"][i], s["qz"][i], s["qw"][i]])
        ge = Rwi.T @ np.array([0, 0, -1.0])
        j = min(np.searchsorted(gt_t, t[i]), len(gt_t) - 1)
        gg = R_RI.T @ rotvec(g[j, 1:4]).T @ np.array([0, 1.0, 0])
        tilt.append(math.degrees(math.acos(np.clip(ge @ gg, -1, 1))))
    tilt = np.array(tilt)
    sr = np.linalg.norm(v, axis=1) / np.maximum(np.linalg.norm(vg, axis=1), 0.3)
    init = ""
    log = os.path.join(a.run, "run.log")
    if os.path.exists(log):
        m = re.search(r"MONO_VIO_INITIALIZED (.*)", open(log).read())
        init = m.group(1) if m else "no init"
    print(f"{a.run}: first state {t[0] - t0_frame:.2f} s; {init}")
    if a.quiet:
        return
    print("  t_rel  t_abs  speed_ratio  |v_gt|  tilt_deg  |ba_err|  |bg_err|  support")
    for dt in [float(x) for x in a.at.split(",")]:
        i = min(np.searchsorted(t, t[0] + dt), len(t) - 1)
        print(f"  {dt:5.1f} {t[i] - t0_frame:6.1f}  {sr[i]:10.3f}  {np.linalg.norm(vg[i]):6.2f}  {tilt[i]:8.2f}"
              f"  {np.linalg.norm(ba[i] - bat[i]):8.3f}  {np.linalg.norm(bg[i] - bgt[i]):8.4f}  {s['support'][i]}")
    sup, cnt = np.unique(s["support"], return_counts=True)
    print("  support counts:", dict(zip(sup, cnt.tolist())))


if __name__ == "__main__":
    main()
