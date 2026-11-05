#!/usr/bin/env python3
"""Relative rotation error [deg] over 30 s windows: estimate T_w_i (TUM) converted to the rig frame vs gt_poses.txt
(xeye: id rx ry rz tx ty tz t_ns, assumed rig-to-world axis-angle). Also the gravity tilt error is not separable
here; this reports the angle of (dR_gt^T dR_est) per window."""
import sys
import numpy as np

def rotvec(r):
    a = np.linalg.norm(r)
    if a < 1e-12:
        return np.eye(3)
    k = r / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K

def quat(q):  # x y z w
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def angle(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))

est = np.loadtxt(sys.argv[1]); g = np.loadtxt(sys.argv[2])
R_RI = rotvec(np.array([-1.2091995761561452] * 3))
gt_t = g[:, 7] * 1e-9; t0 = gt_t[0]
def gt_R(t):
    i = min(np.searchsorted(gt_t, t), len(gt_t) - 1)
    return rotvec(g[i, 1:4])
def est_R(i):
    return quat(est[i, 4:8]) @ R_RI.T
ts = est[:, 0]
out = []
s = ts[0]
while s + 30 < ts[-1]:
    i = np.searchsorted(ts, s); j = np.searchsorted(ts, s + 30)
    dg = gt_R(ts[i]).T @ gt_R(ts[j]); de = est_R(i).T @ est_R(j)
    out.append((s - t0, angle(dg.T @ de)))
    s += 30
for a, b in out:
    print("t=%5.0f-%5.0f s rotation error over 30 s = %.2f deg" % (a, a + 30, b))
print("max %.2f deg, mean %.2f deg" % (max(b for _, b in out), np.mean([b for _, b in out])))
