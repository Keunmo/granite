#!/usr/bin/env python3
"""Summary table of eval/results/<prefix>_<variant>_c<cam>_t<threads>_<i> runs (metrics.json).
Usage: summarize.py PREFIX [--big 2.0]  ->  one row per variant, camera and thread count."""
import argparse, glob, json, os, re
import numpy as np
ap = argparse.ArgumentParser(); ap.add_argument("prefix"); ap.add_argument("--big", type=float, default=2.0)
ap.add_argument("--dir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results"))
a = ap.parse_args()
rows = {}
for f in sorted(glob.glob(os.path.join(a.dir, a.prefix + "_*", "metrics.json"))):
    m = re.match(re.escape(a.prefix) + r"_(.+)_c(\d)_t(\d+)_(\d+)$", os.path.basename(os.path.dirname(f)))
    if not m: continue
    rows.setdefault((m.group(1), int(m.group(2)), int(m.group(3))), []).append(json.load(open(f)))
def q(v): return "%.2f" % v
print("| 변형 | 카메라 | TBB | n | 기준 통과 | ATE>%.0f m | ATE 중앙값 [최소~최대] m | Sim3 scale [최소~최대] | 5 s 스케일 [최소~최대] | 점프 실행 | 30 s 회전 최대 중앙값 [최대] ° | 끝 오차 중앙값 %% | 첫 포즈 s [최소~최대] | 발산·중단 |" % a.big)
print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
for (v, c, t), L in sorted(rows.items()):
    ok = [d for d in L if "ate_m" in d]
    ate = np.array([d["ate_m"] for d in ok]); sc = np.array([d["sim3_scale"] for d in ok])
    s5 = (min(d["scale5_min"] for d in ok), max(d["scale5_max"] for d in ok)) if ok else (np.nan, np.nan)
    rot = np.array([d["rot30_max_deg"] for d in ok]); end = np.array([d["end_err_pct"] for d in ok])
    fp = np.array([d["first_pose_s"] for d in ok])
    div = sum(1 for d in L if set(d["fail"]) & {"diverged_value", "tracking_stopped", "no_output"})
    print(f"| {v} | {c} | {t} | {len(L)} | {sum(d.get('ok', False) for d in L)} | {int((ate > a.big).sum()) + len(L) - len(ok)} | "
          f"{q(np.median(ate))} [{q(ate.min())}~{q(ate.max())}] | [{q(sc.min())}~{q(sc.max())}] | [{q(s5[0])}~{q(s5[1])}] | "
          f"{sum(1 for d in ok if d['jumps'])} | {q(np.nanmedian(rot))} [{q(np.nanmax(rot))}] | {q(np.median(end))} | "
          f"{q(np.median(fp))} [{q(fp.min())}~{q(fp.max())}] | {div} |")
