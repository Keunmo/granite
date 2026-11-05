#!/usr/bin/env python3
"""Run eval_pose.py (copied unchanged from the Pi 4 evaluation, criteria C1..C8) on run folders and print a summary
table per variant, camera and TBB thread count.
Usage: eval_pose_all.py PREFIX [--dir results] [--est granite_traj_tum.txt]
Run folders: <dir>/<PREFIX>_<variant>_c<cam>_t<threads>_<i>/ ; writes eval_pose.json into each folder."""
import argparse, collections, glob, json, os, re, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from eval_pose import evaluate  # noqa: E402
GT = os.path.expanduser("~/multipleye_docker/data/unreal_data_city_sky/gt_poses.txt")
ap = argparse.ArgumentParser(); ap.add_argument("prefix")
ap.add_argument("--dir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results"))
ap.add_argument("--est", default="granite_traj_tum.txt"); ap.add_argument("--reuse", action="store_true")
a = ap.parse_args()
rows = collections.defaultdict(list)
for d in sorted(glob.glob(os.path.join(a.dir, a.prefix + "_*"))):
    m = re.match(re.escape(a.prefix) + r"_(.+)_c(\d)_t(\d+)_(\d+)$", os.path.basename(d))
    f = os.path.join(d, a.est)
    if not m or not os.path.exists(f): continue
    out = os.path.join(d, "eval_pose.json")
    if a.reuse and os.path.exists(out):
        res = json.load(open(out))
    else:
        try:
            res, _ = evaluate(f, GT)
        except Exception as e:
            res = {"verdict": "FAIL", "reason": str(e)}
        json.dump(res, open(out, "w"), indent=1)
    rows[(m.group(1), int(m.group(2)), int(m.group(3)))].append(res)
def q(x): return "%.2f" % x
print("| 변형 | 카메라 | TBB | n | PASS | FAIL | DIVERGED | 위반 기준(실행 수) | ATE % 중앙값 [최소~최대] | Sim3 s [최소~최대] | 5 s 스케일 [최소~최대] | 점프 실행 | 5 s 회전 최대 중앙값 [최대] ° | 끝 오차 % 중앙값 | 첫 포즈 s [최소~최대] |")
print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
for (v, c, t), L in sorted(rows.items()):
    ok = [r for r in L if "ate_pct" in r]
    cnt = collections.Counter(k for r in L for k in r.get("failed", []))
    viol = ", ".join(f"{k.split('_')[0]}({n})" for k, n in sorted(cnt.items())) or "-"
    ate = np.array([r["ate_pct"] for r in ok]); s = np.array([r["sim3_scale"] for r in ok])
    w0 = np.array([r["win_scale_min"] for r in ok]); w1 = np.array([r["win_scale_max"] for r in ok])
    rot = np.array([r["rot5s_max_deg"] for r in ok]); end = np.array([r["end_err_pct"] for r in ok])
    fp = np.array([r["first_pose_s"] for r in ok])
    V = collections.Counter(r["verdict"] for r in L)
    print(f"| {v} | {c} | {t} | {len(L)} | {V['PASS']} | {V['FAIL']} | {V['DIVERGED']} | {viol} | "
          f"{q(np.median(ate))} [{q(ate.min())}~{q(ate.max())}] | [{q(s.min())}~{q(s.max())}] | "
          f"[{q(np.nanmin(w0))}~{q(np.nanmax(w1))}] | {sum(1 for r in ok if r['jump_count'])} | "
          f"{q(np.nanmedian(rot))} [{q(np.nanmax(rot))}] | {q(np.median(end))} | {q(np.median(fp))} [{q(fp.min())}~{q(fp.max())}] |")
