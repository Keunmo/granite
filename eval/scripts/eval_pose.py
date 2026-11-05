#!/usr/bin/env python3
"""Pose accuracy verdict of a metric VIO trajectory against the xeye gt_poses.txt (same metrics for Granite and openvio).

Estimate: TUM (t [s] x y z qx qy qz qw) of the IMU pose T_w_i (Granite T_w_i; openvio q_ItoG, p). GT: xeye pose file
(id rx ry rz tx ty tz t_ns), rig-to-world axis-angle and rig position. The IMU-to-rig translation is 0 (imu2rig_pose),
so positions compare directly; orientations compare after R_w_rig = R_w_i R_RI^T.

Criteria (fixed before the runs; FAIL if any is violated):
  C1 Sim3 scale        |s - 1| <= 0.05     s from the Sim(3) alignment of the whole run (metric scale error <= 5 %)
  C2 window scale      0.90 <= r <= 1.10   r = |p_est(t+5)-p_est(t)| / |p_gt(t+5)-p_gt(t)|, 5 s windows (step 2.5 s)
                                           with GT displacement >= 2 m (shorter ones are too noisy)
  C3 scale jump        |r_i - r_(i-1)| <= 0.10 between consecutive windows
  C4 position jump     |dp_est| <= 3 |v_gt| dt + 0.10 m for every consecutive pose pair (dt = their time step)
  C5 ATE               ATE RMSE (SE3) <= 1.0 % of the GT path length of the run
  C6 end drift         final-pose APE (SE3 aligned) <= 1.0 % of the GT path length
  C7 rotation drift    relative rotation error over 5 s windows <= 3.0 deg, and the orientation error at the end
                       (SE3-aligned rotation) <= 3.0 deg
  C8 divergence        all poses finite and ATE < 10 % of path (a violation is reported as DIVERGED)
  Settling: C2, C3, C4 and the 5 s part of C7 use the poses from first pose + 10 s on (--settle). The first 10 s are
  reported separately (transient_*). Reason: both systems converge after initialization (openvio 3.8-4.8 deg per
  5 s in 8-18 s, Granite Pi 5 a 1.3 m step 0.5 s after initialization); the same allowance is given to both.
  This allowance and the 3.0 deg limit of C7 were chosen after looking at openvio Pi 4 1000-frame runs (stable 5 s
  rotation error 2.0-2.1 deg) and the Pi 5 Granite runs (failures 11-53 deg), before the Pi 4 Granite runs.
Basis: openvio on the full sequence has ATE 0.29 % and final error 0.55 % of the path; successful Granite runs on Pi 5
had window ratios 1.00-1.05 and 30 s rotation errors 0.1-0.6 deg. C1-C3 detect the scale failures seen on Pi 5
(window ratios 1.7-3.5), C4 the 30-364 m one-frame jumps, C7 the 30-53 deg rotation drift.
"""
import argparse
import json
import sys

import numpy as np


def rotvec(r):
    a = np.linalg.norm(r)
    if a < 1e-12:
        return np.eye(3)
    k = r / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


def quat(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def angle(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def align(x, y, scale=False):
    mx, my = x.mean(0), y.mean(0)
    u, s, vt = np.linalg.svd((x - mx).T @ (y - my))
    d = np.diag([1, 1, np.linalg.det(vt.T @ u.T)])
    R = vt.T @ d @ u.T
    c = np.sum(s * np.diag(d)) / np.sum((x - mx) ** 2) if scale else 1.0
    return R, c, my - c * R @ mx


R_RI = rotvec(np.array([-1.2091995761561452] * 3))  # imu2rig rotation of capture.yaml


def evaluate(est_path, gt_path, t_end=None, settle=10.0):
    est = np.atleast_2d(np.loadtxt(est_path, comments="#"))
    g = np.loadtxt(gt_path)
    gt_t = g[:, 7] * 1e-9
    t0 = gt_t[0]
    res = {"file": est_path}
    if len(est) < 10:
        res.update(verdict="FAIL", reason="fewer than 10 poses")
        return res, None
    if not np.isfinite(est).all():
        res.update(verdict="DIVERGED", reason="non-finite pose")
        return res, None
    if t_end is not None:
        est = est[est[:, 0] <= t_end + 1e-6]
    t = est[:, 0]
    keep = (t >= gt_t[0]) & (t <= gt_t[-1])
    est, t = est[keep], t[keep]
    p = est[:, 1:4]
    gp = np.column_stack([np.interp(t, gt_t, g[:, 4 + k]) for k in range(3)])
    gi = np.clip(np.searchsorted(gt_t, t), 0, len(gt_t) - 1)
    Rg = [rotvec(g[i, 1:4]) for i in gi]
    Re = [quat(q) @ R_RI.T for q in est[:, 4:8]]
    path = float(np.linalg.norm(np.diff(gp, axis=0), axis=1).sum())
    R, _, tr = align(p, gp)
    pa = p @ R.T + tr
    ape = np.linalg.norm(pa - gp, axis=1)
    ate = float(np.sqrt(np.mean(ape ** 2)))
    _, s, _ = align(p, gp, scale=True)
    res.update(first_pose_s=float(t[0] - t0), last_pose_s=float(t[-1] - t0), poses=int(len(t)), gt_path_m=path,
               ate_m=ate, ate_pct=100 * ate / path, sim3_scale=float(s), end_err_m=float(ape[-1]),
               end_err_pct=100 * float(ape[-1]) / path)
    # RPE 1 s (as evaluate_trajectory.py: position difference after SE3 alignment)
    j = np.searchsorted(t, t + 1.0)
    ok = j < len(t)
    i_ = np.flatnonzero(ok)
    j = j[ok]
    close = np.abs(t[j] - t[i_] - 1.0) < 0.05
    e = np.linalg.norm((pa[j[close]] - pa[i_[close]]) - (gp[j[close]] - gp[i_[close]]), axis=1)
    res["rpe1s_m"] = float(np.sqrt(np.mean(e ** 2))) if len(e) else float("nan")
    t_settle = t[0] + settle
    # Window scale (all windows kept for the curve; the verdict uses windows that start after t_settle)
    wins = []
    s0 = t[0]
    while s0 + 5 <= t[-1]:
        a, b = np.searchsorted(t, s0), np.searchsorted(t, s0 + 5)
        b = min(b, len(t) - 1)
        dg = np.linalg.norm(gp[b] - gp[a])
        if dg >= 2.0:
            wins.append((float(s0 - t0), float(np.linalg.norm(p[b] - p[a]) / dg)))
        s0 += 2.5
    stable = [w for w in wins if w[0] + t0 >= t_settle]
    trans = [w for w in wins if w[0] + t0 < t_settle]
    r = np.array([w[1] for w in stable]) if stable else np.array([np.nan])
    if trans:
        rt = np.array([w[1] for w in trans])
        res.update(transient_win_scale_min=float(rt.min()), transient_win_scale_max=float(rt.max()))
    jumps_r = np.abs(np.diff(r)) if len(r) > 1 else np.array([0.0])
    res.update(win_scale_min=float(np.nanmin(r)), win_scale_max=float(np.nanmax(r)),
               win_scale_max_change=float(np.nanmax(jumps_r)), windows=len(wins))
    # Position jumps
    dt = np.diff(t)
    dpe = np.linalg.norm(np.diff(p, axis=0), axis=1)
    vg = np.linalg.norm(np.diff(gp, axis=0), axis=1) / np.maximum(dt, 1e-6)
    limit = 3 * vg * dt + 0.10
    after = t[1:] >= t_settle
    res["transient_jump_count"] = int(((dpe > limit) & ~after).sum())
    excess = np.where(after, dpe - limit, -np.inf)
    k = int(np.argmax(excess))
    res.update(jump_count=int(((dpe > limit) & after).sum()), jump_worst_m=float(dpe[k]), jump_worst_limit_m=float(limit[k]),
               jump_worst_t_s=float(t[k + 1] - t0))
    # Rotation: 5 s relative error, and SE3-aligned orientation error over time
    rel = []
    s0 = t[0]
    while s0 + 5 <= t[-1]:
        a, b = np.searchsorted(t, s0), min(np.searchsorted(t, s0 + 5), len(t) - 1)
        rel.append((s0 >= t_settle, angle((Rg[a].T @ Rg[b]).T @ (Re[a].T @ Re[b]))))
        s0 += 2.5
    trans_rel = [v for st, v in rel if not st]
    rel = [v for st, v in rel if st]
    res["transient_rot5s_max_deg"] = float(max(trans_rel)) if trans_rel else float("nan")
    # best constant world rotation: R (position alignment) maps est world to GT world
    rot_err = np.array([angle(Rg[i].T @ (R @ Re[i])) for i in range(len(t))])
    res.update(rot5s_max_deg=float(max(rel)) if rel else float("nan"), rot_end_deg=float(rot_err[-1]),
               rot_max_deg=float(rot_err.max()))
    checks = {
        "C1_sim3_scale": abs(s - 1) <= 0.05,
        "C2_window_scale": bool(np.nanmin(r) >= 0.90 and np.nanmax(r) <= 1.10),
        "C3_scale_jump": bool(np.nanmax(jumps_r) <= 0.10),
        "C4_position_jump": res["jump_count"] == 0,
        "C5_ate": res["ate_pct"] <= 1.0,
        "C6_end_drift": res["end_err_pct"] <= 1.0,
        "C7_rotation": bool((max(rel) if rel else 0) <= 3.0 and rot_err[-1] <= 3.0),
    }
    res["checks"] = {k2: bool(v) for k2, v in checks.items()}
    if res["ate_pct"] >= 10:
        res["verdict"] = "DIVERGED"
    else:
        res["verdict"] = "PASS" if all(checks.values()) else "FAIL"
    res["failed"] = [k2 for k2, v in checks.items() if not v]
    curves = {"t": (t - t0).tolist(), "ape": ape.tolist(), "rot_err": rot_err.tolist(), "wins": wins}
    return res, curves


def plot(curves, title, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    ax[0].plot(curves["t"], curves["ape"])
    ax[0].set_ylabel("APE [m] (SE3)")
    ax[1].plot(curves["t"], curves["rot_err"])
    ax[1].set_ylabel("orientation err [deg]")
    if curves["wins"]:
        w = np.array(curves["wins"])
        ax[2].plot(w[:, 0] + 2.5, w[:, 1], "o-")
    ax[2].axhspan(0.9, 1.1, color="g", alpha=0.15)
    ax[2].set_ylabel("5 s scale ratio")
    ax[2].set_xlabel("dataset time [s]")
    ax[0].set_title(title)
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=90)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("estimate")
    ap.add_argument("ground_truth")
    ap.add_argument("--t-end", type=float, help="ignore poses after this time [s, absolute]")
    ap.add_argument("--settle", type=float, default=10.0)
    ap.add_argument("--json")
    ap.add_argument("--plot")
    a = ap.parse_args()
    res, curves = evaluate(a.estimate, a.ground_truth, a.t_end, a.settle)
    print(json.dumps(res, indent=1))
    if a.json:
        open(a.json, "w").write(json.dumps(res, indent=1))
    if a.plot and curves:
        plot(curves, a.estimate, a.plot)
