#!/usr/bin/env python3
"""Reliability metrics of a metric VIO trajectory (TUM: t x y z qx qy qz qw) against an xeye gt_poses.txt
(id rx ry rz tx ty tz t_ns, rig to world).

ATE RMSE (SE3) and the Sim3 scale use evaluate_trajectory.py of openvio (align, associate) unchanged.
Failure criteria (one is enough):
  - scale5: a 5 s window with est path length / GT path length outside [0.9, 1.1]. Windows with a GT path below
    --min-window-path (default 2 m) are skipped: the ratio of two short paths is dominated by pose noise.
  - jump: a frame step longer than 3 x GT step + --jump-floor (default 0.05 m, the noise floor near hover).
  - diverged: NaN/inf, |p| > 1e4 m, or tracking stops (no pose for > 1 s, or the last pose > 1 s before the
    last frame).
Also reported: 30 s window rotation error (rig rotation from T_w_i and imu2rig), final-pose APE / GT path length,
first-pose delay (first pose time - first frame time).
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.expanduser("~/workspace/openvio/scripts/python"))
from evaluate_trajectory import align, associate, load_trajectory  # noqa: E402

IMU2RIG_ROT = np.array([-1.2091995761561452] * 3)  # capture.yaml imu0 imu2rig_pose rotation


def rotvec(r):
    a = np.linalg.norm(r)
    if a < 1e-12:
        return np.eye(3)
    k = r / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


def quat(q):  # x y z w
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def angle(R):
    return math.degrees(math.acos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("est")
    ap.add_argument("gt")
    ap.add_argument("--cam", default="")
    ap.add_argument("--frames", help="EuRoC cam0/data.csv: first/last frame time")
    ap.add_argument("--t-first-frame", type=float, help="instead of --frames")
    ap.add_argument("--t-last-frame", type=float)
    ap.add_argument("--min-window-path", type=float, default=2.0)
    ap.add_argument("--jump-floor", type=float, default=0.05)
    ap.add_argument("--skip", type=float, default=0.0,
                    help="ignore the first SKIP seconds after the first pose (settling after initialization)")
    ap.add_argument("--json")
    a = ap.parse_args()
    out = {"est": a.est}
    g = np.loadtxt(a.gt)
    gt_t = g[:, 7] * 1e-9
    if a.frames:
        ft = np.loadtxt(a.frames, delimiter=",", comments="#", usecols=0) * 1e-9
        t_first, t_last = ft[0], ft[-1]
    else:
        t_first = a.t_first_frame if a.t_first_frame is not None else gt_t[0]
        t_last = a.t_last_frame if a.t_last_frame is not None else gt_t[-1]
    fail = []
    try:
        raw = np.atleast_2d(np.loadtxt(a.est))
    except Exception:
        raw = np.zeros((0, 8))
    if len(raw) < 10:
        out.update(fail=["no_output"], n=len(raw))
        print(json.dumps(out))
        if a.json:
            json.dump(out, open(a.json, "w"))
        return
    finite = np.all(np.isfinite(raw[:, 1:8]), axis=1)
    big = np.any(np.abs(raw[:, 1:4]) > 1e4, axis=1)
    if not finite.all() or big.any():
        fail.append("diverged_value")
        bad = np.flatnonzero(~finite | big)
        out["first_bad_t"] = raw[bad[0], 0] - t_first
        raw = raw[: bad[0]]
    est = raw
    if a.skip > 0:
        out["first_pose_s_all"] = float(est[0, 0] - t_first)
        est = est[est[:, 0] >= est[0, 0] + a.skip]
    t = est[:, 0]
    gaps = np.diff(t)
    out["max_gap_s"] = float(gaps.max())
    out["last_pose_lag_s"] = float(t_last - t[-1])
    if gaps.max() > 1.0 or t_last - t[-1] > 1.0:
        fail.append("tracking_stopped")
    out["first_pose_s"] = float(t[0] - t_first)
    out["first_pose_gt_s"] = float(t[0] - gt_t[0])
    out["n"] = len(t)

    # ATE (openvio definition)
    traj, _ = load_trajectory(a.gt)
    times, pos, gtp = associate(est[:, :4], traj, -1e18)
    aligned, _, _ = align(pos, gtp)
    ape = np.linalg.norm(aligned - gtp, axis=1)
    length = float(np.sum(np.linalg.norm(np.diff(gtp, axis=0), axis=1)))
    _, scale, _ = align(pos, gtp, allow_scale=True)
    out["ate_m"] = float(np.sqrt(np.mean(ape ** 2)))
    out["sim3_scale"] = float(scale)  # multiply the estimate by this to match GT
    out["gt_path_m"] = length
    out["end_err_pct"] = float(ape[-1] / length * 100)

    # 5 s window scale (path length ratio)
    d = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    gd = np.linalg.norm(np.diff(gtp, axis=0), axis=1)
    ts = times[1:]
    ratios = []
    s = times[0]
    while s + 5.0 <= times[-1] + 1e-9:
        m = (ts > s) & (ts <= s + 5.0)
        if gd[m].sum() >= a.min_window_path:
            ratios.append((s - t_first, d[m].sum() / gd[m].sum()))
        s += 5.0
    r = np.array([x[1] for x in ratios]) if ratios else np.array([np.nan])
    out["scale5_min"] = float(np.nanmin(r))
    out["scale5_max"] = float(np.nanmax(r))
    bad5 = [(round(tt, 1), round(v, 3)) for tt, v in ratios if not 0.9 <= v <= 1.1]
    out["scale5_bad"] = len(bad5)
    out["scale5_windows"] = len(ratios)
    out["scale5_first_bad"] = bad5[:3]
    if bad5:
        fail.append("scale5")

    # jumps
    excess = d - (3 * gd + a.jump_floor)
    k = int(np.argmax(d / np.maximum(gd, 1e-3)))
    out["max_step_m"] = float(d.max())
    out["max_step_ratio"] = float(d[k] / max(gd[k], 1e-3))
    out["max_step_ratio_t"] = float(ts[k] - t_first)
    out["jumps"] = int((excess > 0).sum())
    if out["jumps"]:
        j = int(np.argmax(excess))
        out["worst_jump"] = [round(float(ts[j] - t_first), 2), round(float(d[j]), 3), round(float(gd[j]), 3)]
        fail.append("jump")

    # 30 s rotation error
    R_RI = rotvec(IMU2RIG_ROT)
    def gt_R(tt):
        i = min(np.searchsorted(gt_t, tt), len(gt_t) - 1)
        return rotvec(g[i, 1:4])
    rot = []
    s = t[0]
    while s + 30 <= t[-1]:
        i = np.searchsorted(t, s)
        j = min(np.searchsorted(t, s + 30), len(t) - 1)
        dg = gt_R(t[i]).T @ gt_R(t[j])
        de = (quat(est[i, 4:8]) @ R_RI.T).T @ (quat(est[j, 4:8]) @ R_RI.T)
        rot.append(angle(dg.T @ de))
        s += 30
    out["rot30_max_deg"] = float(max(rot)) if rot else float("nan")

    out["fail"] = fail
    out["ok"] = not fail
    if a.json:
        json.dump(out, open(a.json, "w"), indent=1)
    keys = ["cam", "ok", "ate_m", "sim3_scale", "scale5_min", "scale5_max", "scale5_bad", "jumps", "rot30_max_deg",
            "end_err_pct", "first_pose_s"]
    out["cam"] = a.cam
    print(" ".join(f"{k}={out[k]:.3f}" if isinstance(out[k], float) else f"{k}={out[k]}" for k in keys),
          "fail=" + ",".join(fail))


if __name__ == "__main__":
    main()
