#!/usr/bin/env python3
"""Relative rotation error [deg] of an estimate (TUM, T_w_i) against gt_poses.txt over consecutive DT windows;
prints the windows above --min."""
import argparse, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from metrics import quat, rotvec, angle, IMU2RIG_ROT
ap = argparse.ArgumentParser(); ap.add_argument("est"); ap.add_argument("--gt", default=os.path.expanduser("~/multipleye_docker/data/unreal_data_city_sky/gt_poses.txt"))
ap.add_argument("--dt", type=float, default=1.0); ap.add_argument("--min", type=float, default=1.0)
a = ap.parse_args()
e = np.loadtxt(a.est); g = np.loadtxt(a.gt); gt_t = g[:, 7] * 1e-9; R_RI = rotvec(IMU2RIG_ROT)
def gR(t): return rotvec(g[min(np.searchsorted(gt_t, t), len(gt_t) - 1), 1:4])
t = e[:, 0]; s = t[0]; tot = []
while s + a.dt <= t[-1]:
    i = np.searchsorted(t, s); j = min(np.searchsorted(t, s + a.dt), len(t) - 1)
    de = (quat(e[i, 4:8]) @ R_RI.T).T @ (quat(e[j, 4:8]) @ R_RI.T); dg = gR(t[i]).T @ gR(t[j])
    err = angle(dg.T @ de); rot_gt = angle(dg)
    if err >= a.min: print(f"t={s - gt_t[0]:6.1f} err={err:6.2f} deg gt_rot={rot_gt:6.2f} deg")
    s += a.dt
