#!/usr/bin/env python3
"""First-pose accuracy of a VIO start (rest or moving) against the xeye GT.

Usage: start_state.py TRAJ_TUM START_TIME_S [--states granite_states.csv] [--json OUT]
  TRAJ_TUM      the estimate (TUM, IMU pose T_w_i)
  START_TIME_S  time of the first image given to the estimator [s, absolute]
Reports (the yaw and position of the first pose are gauge and are not compared):
  init_delay_s   first pose time - START_TIME_S
  tilt_err_deg   angle between the estimated and the GT up direction in the IMU frame at the first pose (Granite
                 world: z up; GT world: -y up)
  v_body_err     |R_est^T v_est - R_gt^T v_gt| at the first pose [m/s] (needs the velocity; TUM has none:
                 from --states, else from the first two poses)
  speed_ratio    |v_est| / |v_gt| at the first pose
  d5_err_m       |p_est(t1+5) - p_est(t1)| - |p_gt(t1+5) - p_gt(t1)|, t1 = first pose time
  rot5_err_deg   relative rotation error over the first 5 s (as C7 of eval_pose.py)
GT velocity: central difference of the GT positions (15.4 Hz).
"""
import argparse
import csv
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from eval_pose import R_RI, angle, quat, rotvec  # noqa: E402

GT = os.path.expanduser("~/multipleye_docker/data/unreal_data_city_sky/gt_poses.txt")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("traj")
    ap.add_argument("start", type=float)
    ap.add_argument("--states")
    ap.add_argument("--json")
    a = ap.parse_args()
    g = np.loadtxt(GT)
    gt_t, gp = g[:, 7] * 1e-9, g[:, 4:7]
    gv = np.gradient(gp, gt_t, axis=0)
    est = np.atleast_2d(np.loadtxt(a.traj))
    res = {"traj": a.traj}
    if len(est) < 2:
        res["init_delay_s"] = None
        print(json.dumps(res))
        return
    t = est[:, 0]
    t1 = t[0]

    def gt_at(tt):
        i = int(np.argmin(np.abs(gt_t - tt)))
        return rotvec(g[i, 1:4]) @ R_RI, np.array([np.interp(tt, gt_t, gp[:, k]) for k in range(3)]), \
            np.array([np.interp(tt, gt_t, gv[:, k]) for k in range(3)])

    Rg, pg, vg = gt_at(t1)
    Re = quat(est[0, 4:8])
    if a.states and os.path.exists(a.states):
        row = next(csv.DictReader(open(a.states)))
        ve = np.array([float(row["vx"]), float(row["vy"]), float(row["vz"])])
    else:
        ve = (est[1, 1:4] - est[0, 1:4]) / (t[1] - t[0])
    up_est, up_gt = np.array([0, 0, 1.0]), np.array([0, -1.0, 0])
    tilt = np.degrees(np.arccos(np.clip((Re.T @ up_est) @ (Rg.T @ up_gt), -1, 1)))
    res.update(init_delay_s=float(t1 - a.start), tilt_err_deg=float(tilt),
               v_body_err=float(np.linalg.norm(Re.T @ ve - Rg.T @ vg)),
               speed_est=float(np.linalg.norm(ve)), speed_gt=float(np.linalg.norm(vg)),
               speed_ratio=float(np.linalg.norm(ve) / max(np.linalg.norm(vg), 1e-6)))
    j = int(np.searchsorted(t, t1 + 5.0))
    if j < len(t):
        Rg5, pg5, _ = gt_at(t[j])
        Re5 = quat(est[j, 4:8])
        res["d5_err_m"] = float(np.linalg.norm(est[j, 1:4] - est[0, 1:4]) - np.linalg.norm(pg5 - pg))
        res["rot5_err_deg"] = angle((Rg.T @ Rg5).T @ (Re.T @ Re5))
    print(json.dumps(res))
    if a.json:
        open(a.json, "w").write(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
