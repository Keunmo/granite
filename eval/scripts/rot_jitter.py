#!/usr/bin/env python3
"""Frame-to-frame rotation error [deg] (estimate T_w_i vs GT) per 1 s bin: the rms in each bin.
A healthy run follows the gyro and stays near 0.01 deg."""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from metrics import quat, rotvec, angle, IMU2RIG_ROT
e = np.loadtxt(sys.argv[1]); g = np.loadtxt(os.path.expanduser("~/multipleye_docker/data/unreal_data_city_sky/gt_poses.txt"))
gt_t = g[:, 7] * 1e-9; R_RI = rotvec(IMU2RIG_ROT); t0 = gt_t[0]
def gR(t): return rotvec(g[min(np.searchsorted(gt_t, t), len(gt_t) - 1), 1:4]) @ R_RI
t = e[:, 0]
err = np.array([angle((gR(t[i]).T @ gR(t[i + 1])).T @ (quat(e[i, 4:8]).T @ quat(e[i + 1, 4:8]))) for i in range(len(t) - 1)])
tb = t[1:] - t0
for a in np.arange(np.floor(tb[0]), tb[-1], float(sys.argv[2]) if len(sys.argv) > 2 else 1.0):
    m = (tb >= a) & (tb < a + 1)
    if m.any(): print(f"{a:6.1f} {np.sqrt(np.mean(err[m] ** 2)):7.3f}")
