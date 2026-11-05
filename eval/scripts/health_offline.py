#!/usr/bin/env python3
"""Apply the mono VIO health rule offline to granite_states.csv (columns prev_corr_rot, prev_corr_pos) of runs.
Rule (as MonoVioEstimator): from settle + window seconds after the first state, over the last window seconds,
rms rotation correction > ROT deg and rms translation correction < POS m -> unhealthy.
Usage: health_offline.py GLOB [--rot 0.07 --pos 0.005 --settle 3 --window 2]"""
import argparse, glob, json, os
import numpy as np
ap = argparse.ArgumentParser(); ap.add_argument("glob"); ap.add_argument("--rot", type=float, default=0.07)
ap.add_argument("--pos", type=float, default=0.005); ap.add_argument("--settle", type=float, default=3.0)
ap.add_argument("--window", type=float, default=2.0); ap.add_argument("--big", type=float, default=2.0)
a = ap.parse_args()
tab = {"TP": 0, "FP": 0, "TN": 0, "FN": 0}
for r in sorted(glob.glob(a.glob)):
    try:
        d = json.load(open(r + "/metrics.json"))
        s = np.genfromtxt(r + "/granite_states.csv", delimiter=",", names=True, dtype=None, encoding=None)
    except Exception:
        continue
    t = s["t_ns"] * 1e-9; rot = np.degrees(s["prev_corr_rot"]); pos = s["prev_corr_pos"]
    flag = None
    for i in range(len(t)):
        if t[i] - t[0] < a.settle + a.window: continue
        m = (t > t[i] - a.window) & (t <= t[i]) & np.isfinite(rot) & np.isfinite(pos)
        if m.sum() < 10: continue
        if np.sqrt(np.mean(rot[m] ** 2)) > a.rot and np.sqrt(np.mean(pos[m] ** 2)) < a.pos:
            flag = t[i] - t[0]; break
    bad = d.get("ate_m", 1e9) > a.big
    tab[("TP" if flag is not None else "FN") if bad else ("FP" if flag is not None else "TN")] += 1
    print(f"{os.path.basename(r):22s} ate {d.get('ate_m', -1):8.2f} first {d.get('first_pose_s', -1):6.2f} "
          f"flag {'-' if flag is None else '%.1f s after init' % flag}")
print(tab)
