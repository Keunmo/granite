#!/usr/bin/env python3
"""Scale ratio (estimated / GT path length) per window and the largest one-frame jumps of a Granite TUM trajectory."""
import sys
import numpy as np

est = np.loadtxt(sys.argv[1])
g = np.loadtxt(sys.argv[2])
t0 = g[0, 7] * 1e-9
gt = g[:, 7] * 1e-9
gp = g[:, 4:7]
gi = np.column_stack([np.interp(est[:, 0], gt, gp[:, k]) for k in range(3)])
d = np.linalg.norm(np.diff(est[:, 1:4], axis=0), axis=1)
gd = np.linalg.norm(np.diff(gi, axis=0), axis=1)
dt = np.diff(est[:, 0])
gv = gd / np.maximum(dt, 1e-6)
ev = d / np.maximum(dt, 1e-6)
print("first_state_s=%.2f" % (est[0, 0] - t0))
ts = est[1:, 0] - t0
s = ts[0]
win = 30.0
while s < ts[-1]:
    m = (ts >= s) & (ts < s + win)
    if m.sum() > 5:
        print("window %5.0f-%5.0f s  est/gt path ratio %.2f" % (s, s + win, d[m].sum() / max(gd[m].sum(), 1e-6)))
    s += win
jumps = np.argsort(d - gd)[-3:][::-1]
for i in jumps:
    print("largest step excess t=%.2f s est_step=%.2f m gt_step=%.2f m" % (ts[i], d[i], gd[i]))
print("total est/gt path ratio %.3f" % (d.sum() / gd.sum()))
