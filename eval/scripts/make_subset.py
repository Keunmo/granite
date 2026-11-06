#!/usr/bin/env python3
"""Make a moving-start subset of the city sequence: N frames from frame START, for Granite (EuRoC) and openvio (folder).

Usage: make_subset.py CAM START [N]
  EuRoC:  eval/data/cam<CAM>_s<START>/mav0/{cam0/data.csv, cam0/data/*.png (links), imu0/data.csv}
  openvio: eval/data/ov_cam<CAM>_s<START>/{cam_timestamps.txt, imu_data.csv, images/*.png (links)}
The IMU starts 0.125 s before the first image (as in the full sequence) and ends 0.1 s after the last image, so no
sample from before the start reaches the estimator. Prints the GT motion at the first image.
"""
import os
import sys

import numpy as np

E = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GT = os.path.expanduser("~/multipleye_docker/data/unreal_data_city_sky/gt_poses.txt")


def motion(t0):
    """GT speed, acceleration and angular rate at time t0 (central differences, mean over +-0.2 s)."""
    sys.path.insert(0, os.path.join(E, "scripts"))
    from eval_pose import rotvec
    g = np.loadtxt(GT)
    t, p = g[:, 7] * 1e-9, g[:, 4:7]
    v = np.gradient(p, t, axis=0)
    a = np.gradient(v, t, axis=0)
    i = int(np.searchsorted(t, t0 - 1e-6))
    sl = slice(max(i - 3, 1), i + 4)
    w = []
    for k in range(sl.start, sl.stop):
        d = rotvec(g[k - 1, 1:4]).T @ rotvec(g[k, 1:4])
        w.append(np.degrees(np.arccos(np.clip((np.trace(d) - 1) / 2, -1, 1))) / (t[k] - t[k - 1]))
    return v[i], float(np.linalg.norm(a[sl], axis=1).mean()), float(np.mean(w))


def main():
    cam, start = int(sys.argv[1]), int(sys.argv[2])
    n = int(sys.argv[3]) if len(sys.argv) > 3 else 1000
    src = os.path.join(E, "data", f"cam{cam}", "mav0")
    rows = [l for l in open(os.path.join(src, "cam0", "data.csv")) if not l.startswith("#")]
    sel = rows[start:start + n]
    t_first, t_last = int(sel[0].split(",")[0]), int(sel[-1].split(",")[0])
    imu_hdr, *imu = open(os.path.join(src, "imu0", "data.csv")).read().splitlines()
    imu = [l for l in imu if t_first - 125_000_000 <= int(l.split(",")[0]) <= t_last + 100_000_000]
    # EuRoC
    d = os.path.join(E, "data", f"cam{cam}_s{start}", "mav0")
    os.makedirs(os.path.join(d, "cam0", "data"), exist_ok=True)
    os.makedirs(os.path.join(d, "imu0"), exist_ok=True)
    with open(os.path.join(d, "cam0", "data.csv"), "w") as f:
        f.write("#timestamp [ns],filename\n")
        for l in sel:
            f.write(l if l.endswith("\n") else l + "\n")
            name = l.strip().split(",")[1]
            link = os.path.join(d, "cam0", "data", name)
            if not os.path.lexists(link):
                os.symlink(os.path.realpath(os.path.join(src, "cam0", "data", name)), link)
    with open(os.path.join(d, "imu0", "data.csv"), "w") as f:
        f.write(imu_hdr + "\n" + "\n".join(imu) + "\n")
    # openvio folder (ax ay az gx gy gz, time in s)
    ov_src = os.path.expanduser(f"~/data/unreal_city_sky_cam{cam}_640")
    o = os.path.join(E, "data", f"ov_cam{cam}_s{start}")
    os.makedirs(os.path.join(o, "images"), exist_ok=True)
    ov_imgs = sorted(os.listdir(os.path.join(ov_src, "images")))
    ov_times = open(os.path.join(ov_src, "cam_timestamps.txt")).read().split()
    assert abs(float(ov_times[start]) - t_first * 1e-9) < 1e-4, "openvio and EuRoC frames differ"
    with open(os.path.join(o, "cam_timestamps.txt"), "w") as f:
        f.write("\n".join(ov_times[start:start + n]) + "\n")
    for k, name in enumerate(ov_imgs[start:start + n]):
        link = os.path.join(o, "images", f"{k:06d}.png")
        if not os.path.lexists(link):
            os.symlink(os.path.realpath(os.path.join(ov_src, "images", name)), link)
    ov_imu = open(os.path.join(ov_src, "imu_data.csv")).read().splitlines()
    keep = [l for l in ov_imu[1:] if t_first * 1e-9 - 0.125 - 1e-6 <= float(l.split(",")[0]) <= t_last * 1e-9 + 0.1]
    with open(os.path.join(o, "imu_data.csv"), "w") as f:
        f.write(ov_imu[0] + "\n" + "\n".join(keep) + "\n")
    v, acc, w = motion(t_first * 1e-9)
    print(f"cam{cam} start={start} t_first={t_first * 1e-9:.4f} frames={len(sel)} imu={len(imu)}/{len(keep)} "
          f"|v|={np.linalg.norm(v):.2f} m/s v=({v[0]:.2f},{v[1]:.2f},{v[2]:.2f}) |a|={acc:.2f} m/s2 w={w:.1f} deg/s")


if __name__ == "__main__":
    main()
