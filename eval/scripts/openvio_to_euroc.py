#!/usr/bin/env python3
"""Make a Granite EuRoC folder from an openvio PNG folder dataset (images/, cam_timestamps.txt, imu_data.csv) with
image symlinks (no copy), and optionally write the Granite mono KB4 calibration of one camera from capture.yaml.

Camera: uwfisheye with poly [1, a1] (rho = a1 theta) is exactly KB4 with k1..k4 = 0 and fx = fy = f a1.
640x640 is half of 1280: f / 2, c -> (c + 0.5) / 2 - 0.5.
T_imu_cam = (imu2rig)^-1 cam2rig; poses [rx, ry, rz, tx, ty, tz] map sensor to rig (as capture_to_euroc.py).
"""
import argparse
import json
import math
import os
import pathlib
from decimal import Decimal

import numpy as np
import yaml


def to_ns(s):
    # The text holds a printed double (e.g. 1793002753.4946001); the capture clock has a 1 us grid.
    return int((Decimal(s.strip()) * 1000000).to_integral_value()) * 1000


def pose_matrix(p):
    r = np.array(p[:3], float)
    a = np.linalg.norm(r)
    R = np.eye(3)
    if a > 0:
        k = r / a
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        R = np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p[3:6]
    return T


def rot_to_quat(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    y = math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    z = math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    x = math.copysign(x, R[2, 1] - R[1, 2])
    y = math.copysign(y, R[0, 2] - R[2, 0])
    z = math.copysign(z, R[1, 0] - R[0, 1])
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return w / n, x / n, y / n, z / n


def write_calib(capture, camera, out, node="quad_cam", imu="imu0"):
    cap = yaml.safe_load(pathlib.Path(capture).read_text())
    cal = cap["calib"][node][camera]
    f, cx, cy = cal["intrinsic"]
    poly = cal["poly"]
    assert int(poly[0]) == 1 and len(poly) == 2, "only rho = a1 theta (equidistant) is converted exactly"
    w, h = cal["image_size"]
    fh = f * poly[1] / 2.0
    cxh, cyh = (cx + 0.5) / 2 - 0.5, (cy + 0.5) / 2 - 0.5
    imu_cal = cap[imu]
    T_RI = pose_matrix(cap["calib"].get(imu, {}).get("imu2rig_pose", imu_cal["imu2rig_pose"]))
    T_RC = pose_matrix(cal["cam2rig_pose"])
    T_IC = np.linalg.inv(T_RI) @ T_RC
    qw, qx, qy, qz = rot_to_quat(T_IC[:3, :3])
    noise = imu_cal["intrinsic"]["noise"]
    calib = {"value0": {
        "T_imu_cam": [{"px": T_IC[0, 3], "py": T_IC[1, 3], "pz": T_IC[2, 3], "qx": qx, "qy": qy, "qz": qz, "qw": qw}],
        "intrinsics": [{"camera_type": "kb4", "intrinsics": {"fx": fh, "fy": fh, "cx": cxh, "cy": cyh,
                                                              "k1": 0.0, "k2": 0.0, "k3": 0.0, "k4": 0.0}}],
        "resolution": [[w // 2, h // 2]],
        "main_cam_idx": [0],
        "stereo_pairs": [],
        "cam_update_rate": float(cap[node]["fps"]),
        "vignette": [],
        "calib_accel_bias": [0.0] * 9,
        "calib_gyro_bias": [0.0] * 12,
        "imu_update_rate": 200.0,
        "accel_noise_std": [noise["acc_noise_density"]] * 3,
        "gyro_noise_std": [noise["gyro_noise_density"]] * 3,
        "accel_bias_std": [noise["acc_bias_random_walk"]] * 3,
        "gyro_bias_std": [noise["gyro_bias_random_walk"]] * 3,
        "T_mocap_world": {"px": 0.0, "py": 0.0, "pz": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
        "T_imu_marker": {"px": 0.0, "py": 0.0, "pz": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
        "mocap_time_offset_ns": 0,
        "mocap_to_imu_offset_ns": 0,
        "cam_time_offset_ns": 0}}
    pathlib.Path(out).write_text(json.dumps(calib, indent=4))
    print(camera, "T_imu_cam=\n", np.array2string(T_IC, precision=6, suppress_small=True))
    print(f"kb4 fx=fy={fh} cx={cxh} cy={cyh} res={w // 2}x{h // 2}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src", help="openvio folder: images/, cam_timestamps.txt, imu_data.csv")
    ap.add_argument("out", help="EuRoC output folder (mav0 is created inside)")
    ap.add_argument("--capture", help="capture.yaml to write the calibration from")
    ap.add_argument("--camera", default="Cam1")
    ap.add_argument("--calib-out")
    a = ap.parse_args()
    src = pathlib.Path(a.src)
    imgs = sorted((src / "images").iterdir())
    ts = [to_ns(x) for x in (src / "cam_timestamps.txt").read_text().split()]
    assert len(imgs) == len(ts), (len(imgs), len(ts))
    mav = pathlib.Path(a.out) / "mav0"
    camdir = mav / "cam0" / "data"
    camdir.mkdir(parents=True, exist_ok=True)
    (mav / "imu0").mkdir(parents=True, exist_ok=True)
    with open(mav / "cam0" / "data.csv", "w") as f:
        f.write("#timestamp [ns],filename\n")
        for p, t in zip(imgs, ts):
            link = camdir / f"{t}.png"
            if not link.exists():
                os.symlink(os.path.realpath(p), link)
            f.write(f"{t},{t}.png\n")
    with open(src / "imu_data.csv") as fi, open(mav / "imu0" / "data.csv", "w") as fo:
        fo.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                 "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")
        for line in fi:
            if not line[0].isdigit():
                continue
            t, ax, ay, az, gx, gy, gz = line.strip().split(",")[:7]
            fo.write(f"{to_ns(t)},{gx},{gy},{gz},{ax},{ay},{az}\n")
    print("frames", len(ts), "->", mav)
    if a.capture and a.calib_out:
        write_calib(a.capture, a.camera, a.calib_out)


if __name__ == "__main__":
    main()
