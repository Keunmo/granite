#!/usr/bin/env python3
"""Convert one camera of an xeye capture (RGB8 quad_cam, or BAYER*8 / GRAY8 multi-camera nodes) and its IMU to a Granite
EuRoC folder, and (equidistant uwfisheye only) write a Granite mono calibration JSON. For other camera models write
the calibration with capture_to_granite.py --target capture.

Image: RGB8 -> gray (OpenCV COLOR_RGB2GRAY fixed point: (4899 R + 9617 G + 1868 B + 8192) >> 14), then 2x2 average
((a + b + c + d + 2) >> 2) -> 8-bit PNG. This is the openvio capture.resize_half condition.
BAYER*8 and GRAY8: --downscale N as the live app (grbgToGray): N even averages N x N raw pixels, N odd averages each
2x2 quad and then resizes by 2/N with area weights; N = 1 keeps the raw values as gray (no demosaicing, as the xeye
gray16 calibration images).
Camera: uwfisheye with one forward coefficient (rho = a1 theta) is the equidistant model, which is Kannala-Brandt
(kb4) with k1..k4 = 0 and fx = fy = f a1. Halving: f / 2, c -> (c + 0.5) / 2 - 0.5.
T_imu_cam = (imu2rig)^-1 cam2rig, poses [rx, ry, rz, tx, ty, tz] map sensor to rig (capture_to_config.py).
The original capture is only read.
"""
import argparse
import json
import math
import os
import pathlib
import sys
from multiprocessing import Pool

import numpy as np
import yaml
from PIL import Image


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


G = {}


def init(root, node, offset, size, w, h, rec_per_file, packet, fmt, downscale):
    G.update(root=root, node=node, offset=offset, size=size, w=w, h=h, rpf=rec_per_file, packet=packet, fmt=fmt,
             downscale=downscale)


def raw_to_gray(raw, n):
    """grbgToGray of the live app for a raw (Bayer or gray) 8-bit image."""
    import cv2
    h, w = raw.shape
    if n == 1:
        return raw
    if n % 2 == 0:
        return cv2.resize(raw, (w // n, h // n), interpolation=cv2.INTER_AREA)
    quads = cv2.resize(raw, (w // 2, h // 2), interpolation=cv2.INTER_AREA)
    return cv2.resize(quads, (w // n, h // n), interpolation=cv2.INTER_AREA)


def convert(job):
    i, t_ns, out = job
    path = os.path.join(G["root"], "%s%03d.bin" % (G["node"], i // G["rpf"]))
    rec = (i % G["rpf"]) * (G["packet"] + 4)
    with open(path, "rb") as f:
        f.seek(rec)
        size = int(np.frombuffer(f.read(4), "<i4")[0])
        assert size == G["packet"], (i, size)
        f.seek(rec + 4 + G["offset"])
        data = f.read(G["size"])
    if G["fmt"] == "RGB8":
        raw = np.frombuffer(data, np.uint8).reshape(G["h"], G["w"], 3).astype(np.uint32)
        gray = (4899 * raw[..., 0] + 9617 * raw[..., 1] + 1868 * raw[..., 2] + 8192) >> 14
        half = (gray[0::2, 0::2] + gray[1::2, 0::2] + gray[0::2, 1::2] + gray[1::2, 1::2] + 2) >> 2
        image = half.astype(np.uint8)
    else:
        image = raw_to_gray(np.frombuffer(data, np.uint8).reshape(G["h"], G["w"]), G["downscale"])
    Image.fromarray(image).save(os.path.join(out, "%d.png" % t_ns), compress_level=1)
    return i


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture")
    ap.add_argument("out")
    ap.add_argument("--camera", default="Cam1")
    ap.add_argument("--node", default="quad_cam")
    ap.add_argument("--imu", default="imu0")
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--calib-out", help="calibration JSON (equidistant uwfisheye only; else capture_to_granite.py)")
    ap.add_argument("--downscale", type=int, default=2, help="BAYER*8 / GRAY8: 1/N of the image (RGB8: always 2)")
    ap.add_argument("--limit", type=int, default=0)
    a = ap.parse_args()
    root = pathlib.Path(a.capture)
    cap = yaml.safe_load((root / "capture.yaml").read_text())
    cams = cap[a.node]["sensor_nodes"]
    ci = cams.index(a.camera)
    cal = cap["calib"][a.node][a.camera]

    lines = (root / f"{a.node}.csv").read_text().splitlines()
    tok = lines[0].split()
    assert tok[1] == "camera_frame"
    n = int(tok[2])
    layouts, j, off = [], 3, 0
    for _ in range(n):
        w, h, fmt = int(tok[j]), int(tok[j + 1]), tok[j + 2]
        j += 3
        if j < len(tok) and not tok[j].lstrip("-").isdigit():
            j += 1
        assert fmt == "RGB8" or fmt.startswith("BAYER") or fmt == "GRAY8", fmt
        channels = 3 if fmt == "RGB8" else 1
        layouts.append((off, w * h * channels, w, h, fmt))
        off += w * h * channels
    packet = off
    offset, size, w, h, fmt = layouts[ci]
    if fmt == "RGB8":
        a.downscale = 2
    fsize = os.path.getsize(root / f"{a.node}000.bin")
    assert fsize % (packet + 4) == 0
    rpf = fsize // (packet + 4)
    frames = []
    for line in lines[1:]:
        if not line.strip() or line.startswith("T"):
            continue
        t, idx = line.split(",")
        frames.append((int(idx), int(t)))
    if a.limit:
        frames = frames[: a.limit]

    out = pathlib.Path(a.out) / "mav0"
    camdir = out / "cam0" / "data"
    camdir.mkdir(parents=True, exist_ok=True)
    (out / "imu0").mkdir(parents=True, exist_ok=True)
    with open(out / "cam0" / "data.csv", "w") as f:
        f.write("#timestamp [ns],filename\n")
        for idx, t in frames:
            f.write(f"{t},{t}.png\n")
    # IMU: Timestamp (ns),Gyro_x..z,Acc_x..z[,extra]
    with open(root / f"{a.imu}.csv") as fi, open(out / "imu0" / "data.csv", "w") as fo:
        fo.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                 "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")
        for line in fi:
            if not line[0].isdigit():
                continue
            v = line.strip().split(",")
            fo.write(",".join(v[:7]) + "\n")

    if a.calib_out:
        write_calibration(a, cap, cal, w, h)
    jobs = [(idx, t, str(camdir)) for idx, t in frames]
    with Pool(a.jobs, initializer=init,
              initargs=(str(root), a.node, offset, size, w, h, rpf, packet, fmt, a.downscale)) as p:
        for k, _ in enumerate(p.imap(convert, jobs, chunksize=8)):
            if k % 500 == 0:
                print(f"{k}/{len(jobs)}", file=sys.stderr, flush=True)
    print("done", len(jobs), file=sys.stderr)


def write_calibration(a, cap, cal, w, h):
    f, cx, cy = cal["intrinsic"]
    poly = cal["poly"]
    assert int(poly[0]) == 1 and len(poly) == 2, "only rho = a1 theta (equidistant) is converted exactly"
    a1 = poly[1]
    fh = f * a1 / 2.0
    cxh, cyh = (cx + 0.5) / 2 - 0.5, (cy + 0.5) / 2 - 0.5
    imu_cal = cap.get(a.imu, {})
    T_RI = pose_matrix(cap["calib"].get(a.imu, {}).get("imu2rig_pose", imu_cal["imu2rig_pose"]))
    T_RC = pose_matrix(cal["cam2rig_pose"])
    T_IC = np.linalg.inv(T_RI) @ T_RC
    qw, qx, qy, qz = rot_to_quat(T_IC[:3, :3])
    noise = imu_cal["intrinsic"]["noise"]
    rate = 200.0
    # Granite (basalt) uses continuous-time noise densities; it multiplies by sqrt(rate) internally? keep as given.
    calib = {"value0": {
        "T_imu_cam": [{"px": T_IC[0, 3], "py": T_IC[1, 3], "pz": T_IC[2, 3], "qx": qx, "qy": qy, "qz": qz, "qw": qw}],
        "intrinsics": [{"camera_type": "kb4", "intrinsics": {"fx": fh, "fy": fh, "cx": cxh, "cy": cyh,
                                                              "k1": 0.0, "k2": 0.0, "k3": 0.0, "k4": 0.0}}],
        "resolution": [[w // 2, h // 2]],
        "main_cam_idx": [0],
        "stereo_pairs": [],
        "cam_update_rate": float(cap[a.node]["fps"]),
        "vignette": [],
        "calib_accel_bias": [0.0] * 9,
        "calib_gyro_bias": [0.0] * 12,
        "imu_update_rate": rate,
        "accel_noise_std": [noise["acc_noise_density"]] * 3,
        "gyro_noise_std": [noise["gyro_noise_density"]] * 3,
        "accel_bias_std": [noise["acc_bias_random_walk"]] * 3,
        "gyro_bias_std": [noise["gyro_bias_random_walk"]] * 3,
        "T_mocap_world": {"px": 0.0, "py": 0.0, "pz": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
        "T_imu_marker": {"px": 0.0, "py": 0.0, "pz": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
        "mocap_time_offset_ns": 0,
        "mocap_to_imu_offset_ns": 0,
        "cam_time_offset_ns": 0}}
    pathlib.Path(a.calib_out).write_text(json.dumps(calib, indent=4))
    print("T_imu_cam=\n", np.array2string(T_IC, precision=6, suppress_small=True), file=sys.stderr)
    print(f"kb4 fx=fy={fh} cx={cxh} cy={cyh} res={w//2}x{h//2}", file=sys.stderr)



if __name__ == "__main__":
    main()
