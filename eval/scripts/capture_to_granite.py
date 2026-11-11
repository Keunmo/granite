#!/usr/bin/env python3
"""Convert the calibration of one camera of an xeye capture (capture.yaml) to a Granite calibration JSON and a Granite
mono VIO config JSON.

Calibration JSON (--calib-out), at 1/N of the calibrated image size (--downscale N):
  intrinsics   uwfisheye (rho = f (a1 theta + a2 theta^2 + ...), theta <= max_theta) has no Granite model. The script
               fits kb4, ds (double sphere) and eucm to rho(theta) by least squares over the full field of view,
               theta in [0, max_theta]. It checks each fit in both directions with the Granite formulas (kb4 unprojects
               with 3 Newton steps) on a pixel grid of the image inside the field of view, and on the theta grid.
               It writes kb4 when its largest error is at most --max-error-px (the model of the evaluated runs), else
               the model with the smallest largest error. opencv_fisheye is kb4 exactly; pinhole without distortion is
               pinhole.
  crop         camera_v4l2_gen2 (model kurokesu_ar0234_pi, b0429_pi) sends the crop [x, y, w, h] of the sensor mode.
               A calibration at the sensor mode moves to the crop: c -> c - (x, y), size -> (w, h). A calibration at
               the crop size stays.
  downscale    an N x N reduction (grbgToGray of the live app): f / N, c -> (c + 0.5) / N - 0.5, size / N.
               At --downscale 2 of a kurokesu_ar0234_pi node at 1920x1200, the live app uses the 2x2 binning mode of
               the sensor (camera.sensor_binning, default true) with the halved crop when the crop values are multiples
               of 4. Output pixel i covers the full resolution pixels 2i and 2i + 1, so the same formula holds.
  T_imu_cam    (imu2rig_pose)^-1 cam2rig_pose; a pose [rx, ry, rz, tx, ty, tz] maps the sensor frame into the rig frame.
  biases       calib_accel_bias / calib_gyro_bias = bias_acc / bias_gyro of the IMU (Granite subtracts them from the
               samples) for raw samples (--imu-samples raw: the live sensors). Zero for --imu-samples calibrated: an xeye
               IMU node with calibrated: true records the samples with these biases already removed.
  IMU noise    the Allan variance of imu_intrinsics_calibrator (calib.<imu>.intrinsic.noise or --imu-intrinsics), else
               the openvio defaults of config/pi5_b0429.yaml (MPU-9250 datasheet value x 2; not calibrated).
  time offset  cam_time_offset_ns. --target capture: time_offset (t_imu = t_cam + time_offset) for the replay of the
               capture itself; the runner applies it with --cam-time-offset-ns, which the script prints.
               --target live with a camera_v4l2_gen2 node of model kurokesu_ar0234_pi or b0429_pi: time_offset too. The
               live app stamps the images with the V4L2 buffer time and maps the imu_mavlink sample times as the xeye
               nodes do, so the offset of the capture holds live.
               --target live with the other nodes: 0. The live app (model b0429) stamps the exposure center; the xeye
               time_offset holds for the xeye image stamp (after DQBUF), which is later than the exposure center by the
               readout, half the exposure and the DQBUF delay. Granite does not estimate the offset online.

VIO config JSON (--config-out): a copy of --base (default: the final candidate eval/config/oi_g40_det_gauge_gate_rest.json)
with
  config.vio_use_deterministic          false (live: the deterministic sums cost about 7 % and give nothing live)
  config.vio_mono_init_max_still_disparity, config.vio_mono_init_max_rest_disparity
                                        motion thresholds in pixels: the base values times the ratio of the image
                                        scales at the image center (px/rad) of this camera and of --base-calib (the
                                        camera that the base was evaluated with). The other pixel values are noise
                                        or cell sizes in pixels and stay (as in openvio capture_to_config.py).
The start rules of the base (excitation 0.3 m/s^2, rest_average_time 0.1 s, max_rest_disparity 2.0 px,
max_still_disparity 4.35 px at 178.9 px/rad, no moving start) hold for a hand-held rig and for a drone with the
motors on, so the live config takes them unchanged except for the pixel scale. The base also turns off the rotation
rule of the health check (vio_mono_health_rot_deg 180), so only the bias rule sends ODOMETRY quality -1.
  send_vis_msg, vis_msg_ip, vis_msg_port, vis_msg_image_hz, vis_msg_points_hz, vis_msg_encode,
  vis_msg_jpeg_quality, vis_msg_max_kbps, vis_msg_h264_rate_control, vis_msg_h264_crf
                                        --target live: the visualization messages for a viewer on the ground (UDP,
                                        doc/granite_vis.proto): on, --vis-msg-ip (default 10.20.10.42),
                                        --vis-msg-port (default 5100), mjpeg images (quality 80) at most 20 Hz,
                                        landmarks 2 Hz, at most 2000 kbit/s (h264 keys for vis_msg_encode h264).
  camera.*, imu.*                       the live sensor options of the live app.
      camera_v4l2_gen2 node of model kurokesu_ar0234_pi or b0429_pi: camera.<key> for each property key of the node
      with the xeye key names (model, port, width, height, frame_rate, pixel_format gray8, exposure_time, gain, h_flip,
      v_flip, crop, auto_white_balance, fused_wb, black_level, enable_gamma, gamma_value, auto_exposure, auto_gain,
      auto_exposure_range, auto_gain_range, auto_exp_prior, target_gray_value, auto_exposure_ev_compensation,
      auto_exposure_damping, auto_exposure_metering, auto_exposure_roi, auto_exposure_circle, num_v4l_buffer) and
      camera.downscale. The live app takes gray8 only.
      Other camera nodes: camera.model b0429 with the granite options (sensor mode, downscale, fps, auto exposure,
      metering circle inside the fisheye image circle).
      imu_mavlink node: imu.type imu_mavlink, imu.udp or imu.port and imu.baudrate, imu.freq_hz.
      Other IMU nodes: imu.type mpu_i2c, camera.i2c_device, camera.imu_rate.
The script prints every changed value and its reason.
"""

import argparse
import json
import math
import pathlib
import sys

import numpy as np
import yaml

REPO = pathlib.Path(__file__).resolve().parents[2]
DEFAULT_BASE = REPO / "eval/config/oi_g40_det_gauge_gate_rest.json"
DEFAULT_BASE_CALIB = REPO / "eval/config/raw_cam1_kb4_calib.json"
# Granite VioConfig defaults (src/utils/vio_config.cpp) of the scaled keys, for a base without them
GRANITE_DEFAULTS = {"config.vio_mono_init_max_still_disparity": 8.7, "config.vio_mono_init_max_rest_disparity": 0.0,
                    "config.vio_mono_init_excitation_accel_std": 1.0}
SCALED_KEYS = (("config.vio_mono_init_max_still_disparity", "mean track motion that separates still / moving"),
               ("config.vio_mono_init_max_rest_disparity", "mean track motion of the older half at rest"))
# openvio config/pi5_b0429.yaml: MPU-9250 datasheet value x 2, not calibrated
OPENVIO_IMU_NOISE = {"gyro_noise_density": 3.5e-4, "acc_noise_density": 6.0e-3, "gyro_bias_random_walk": 2.2e-5,
                     "acc_bias_random_walk": 3.0e-3}
KB4_NEWTON_STEPS = 3  # KannalaBrandtCamera4::unproject: solve_theta<3>
# camera_v4l2_gen2 models that the live app processes as the xeye node (its model names)
XEYE_MODELS = {"kurokesu_ar0234_pi": "kurokesu_ar0234_pi", "b0429_pi": "b0429_pi", "arducam_b0429_pi": "b0429_pi"}
# The property keys of these nodes that the live app takes (camera.<key>)
XEYE_CAMERA_KEYS = ("width", "height", "pixel_format", "exposure_time", "gain", "num_v4l_buffer", "h_flip", "v_flip",
                    "crop", "auto_white_balance", "fused_wb", "black_level", "enable_gamma", "gamma_value",
                    "auto_exposure", "auto_gain", "auto_exposure_range", "auto_gain_range", "auto_exp_prior",
                    "target_gray_value", "auto_exposure_ev_compensation", "auto_exposure_damping",
                    "auto_exposure_metering", "auto_exposure_roi", "auto_exposure_circle")
# PX4 sends new HIGHRES_IMU samples at IMU_INTEG_RATE (default 200 Hz); xeye received 193.6 Hz with freq_hz 400
PX4_IMU_RATE = 200.0


# ---------------------------------------------------------------------------------------------------------------- xeye
def is_type(node_type, kind):
    return node_type == kind or node_type.startswith(kind + "_")


def find_camera(capture, name):
    """(node, camera) of the camera `name` ("cam1", "node/cam1", a single-camera node; empty: the first)."""
    cameras = []
    for node in capture.get("sensor_nodes", []):
        node_type = str(capture.get(node, {}).get("type", ""))
        if is_type(node_type, "multi_camera"):
            cameras += [(node, child) for child in capture[node].get("sensor_nodes", [])]
        elif is_type(node_type, "camera"):
            cameras.append((node, node))
    for node, camera in cameras:
        if not name or name in (camera, f"{node}/{camera}"):
            return node, camera
    listed = " ".join(f"{node}/{camera}" for node, camera in cameras) or "none"
    raise ValueError(f"the capture has no camera '{name}'; its cameras: {listed}")


def find_imu(capture):
    for node in capture.get("sensor_nodes", []):
        if is_type(str(capture.get(node, {}).get("type", "")), "imu"):
            return node
    return None


def pose_matrix(pose):
    """4x4 transform of an xcv pose [rx, ry, rz, tx, ty, tz] (Rodrigues rotation, then translation)."""
    r = np.array(pose[:3], float)
    angle = np.linalg.norm(r)
    R = np.eye(3)
    if angle > 0:
        k = r / angle
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        R = np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = pose[3:6]
    return T


def rot_to_quat(R):
    """(qw, qx, qy, qz) of a rotation matrix."""
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return w / n, x / n, y / n, z / n


def read_capture(capture_yaml, camera_name):
    capture = yaml.safe_load(pathlib.Path(capture_yaml).read_text())
    node, camera = find_camera(capture, camera_name)
    calib = capture.get("calib") or {}
    name = camera if node == camera else f"{node}.{camera}"
    cam = calib.get(node, {}) if node == camera else calib.get(node, {}).get(camera, {})
    if "model" not in cam:
        raise ValueError(f"capture.yaml has no calib.{name}")
    imu = find_imu(capture)
    imu_node = capture.get(imu) or {}
    imu_calib = calib.get(imu) or {}
    imu_property = imu_node.get("property") or {}
    imu_pose = imu_calib.get("imu2rig_pose", imu_node.get("imu2rig_pose"))
    if imu_pose is None:
        print(f"warning: no imu2rig_pose of {imu}: the IMU frame is the rig frame", file=sys.stderr)
        imu_pose = [0.0] * 6
    T_ic = np.linalg.inv(pose_matrix(imu_pose)) @ pose_matrix(cam.get("cam2rig_pose", [0.0] * 6))
    camera_node = capture.get(node) or {}
    camera_property = camera_node.get("property") or {}
    fps = camera_node.get("fps", camera_property.get("frame_rate"))
    intrinsic = imu_calib.get("intrinsic") or imu_node.get("intrinsic") or {}
    xeye_model = None
    if node == camera and str(camera_node.get("type", "")) == "camera_v4l2_gen2":
        xeye_model = XEYE_MODELS.get(str(camera_node.get("model", "")))
    imu_type = str(imu_node.get("type", ""))
    mavlink = None
    if imu_type == "imu_mavlink":
        mavlink = {k: imu_node[k] for k in ("udp", "port", "baudrate", "freq_hz") if k in imu_node}
    return {
        "name": name, "node": node, "imu": imu, "calib": cam, "T_ic": T_ic,
        "time_offset": float(imu_calib.get("time_offset", imu_property.get("time_offset", 0.0))),
        "bias_gyro": imu_calib.get("bias_gyro", imu_property.get("bias_gyro")),
        "bias_acc": imu_calib.get("bias_acc", imu_property.get("bias_acc")),
        "samples_calibrated": bool(imu_property.get("calibrated", False)),
        "noise": intrinsic.get("noise"),
        "imu_rate": PX4_IMU_RATE if mavlink is not None else float(imu_node.get("freq_hz", 200.0)),
        "i2c_device": None if mavlink is not None else imu_node.get("port"),
        "xeye_model": xeye_model, "camera_node": camera_node, "camera_property": camera_property,
        "mavlink": mavlink,
        "fps": float(fps) if fps else None,
        "live": capture.get("play_record") is False,
        "sensor_size": (int(camera_property["width"]), int(camera_property["height"]))
        if "width" in camera_property and "height" in camera_property else None,
    }


# ------------------------------------------------------------------------------------------------------ camera models
class UwFisheye:
    """xcv uwfisheye: pixel = c + f rho(theta) (x, y) / r, rho(theta) = sum a_i theta^i, valid up to max_theta."""

    def __init__(self, calib, factor, crop=None):
        f, cx, cy = (float(v) for v in calib["intrinsic"])
        poly = calib["poly"]
        if int(poly[0]) != len(poly) - 1:
            raise ValueError("poly is not [n, a1, ..., an]")
        self.a = np.array(poly[1:], float)
        self.f = f / factor
        self.c = np.array([(cx + 0.5) / factor - 0.5, (cy + 0.5) / factor - 0.5])
        self.max_theta = math.radians(float(calib.get("max_theta", 0.0)) or 180.0)
        w, h = (int(v) for v in calib["image_size"])
        if crop:
            x0, y0, w, h = crop
            cx, cy = cx - x0, cy - y0
            self.c = np.array([(cx + 0.5) / factor - 0.5, (cy + 0.5) / factor - 0.5])
        if w % factor or h % factor:
            raise ValueError(f"the size {w}x{h} is not a multiple of {factor}")
        self.size = (w // factor, h // factor)

    def rho(self, theta):
        """Image radius [px] of the angle theta [rad]."""
        return self.f * sum(a * theta ** (i + 1) for i, a in enumerate(self.a))

    def drho(self, theta):
        return self.f * sum((i + 1) * a * theta ** i for i, a in enumerate(self.a))

    def theta(self, rho):
        """Inverse of rho by bisection then Newton (rho grows monotonically on [0, max_theta])."""
        lo, hi = np.zeros_like(rho), np.full_like(rho, self.max_theta)
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            low = self.rho(mid) < rho
            lo, hi = np.where(low, mid, lo), np.where(low, hi, mid)
        t = 0.5 * (lo + hi)
        for _ in range(3):
            t -= (self.rho(t) - rho) / self.drho(t)
        return t

    def pixels_per_radian(self):
        return self.f * self.a[0]


def kb4_rho(p, theta):
    t2 = theta * theta
    return p[0] * theta * (1 + p[4] * t2 + p[5] * t2 ** 2 + p[6] * t2 ** 3 + p[7] * t2 ** 4)


def kb4_theta(p, rho):
    """KannalaBrandtCamera4::unproject: solve_theta<3> from theta = r_theta."""
    k1, k2, k3, k4 = p[4:8]
    td = rho / p[0]
    th = td.copy()
    for _ in range(KB4_NEWTON_STEPS):
        t2 = th * th
        func = th * (1 + k1 * t2 + k2 * t2 ** 2 + k3 * t2 ** 3 + k4 * t2 ** 4)
        d = 1 + 3 * k1 * t2 + 5 * k2 * t2 ** 2 + 7 * k3 * t2 ** 3 + 9 * k4 * t2 ** 4
        th = th + (td - func) / d
    return th, np.ones_like(rho, bool)


def ds_rho(p, theta):
    xi, alpha = p[4], p[5]
    s, c = np.sin(theta), np.cos(theta)
    d2 = np.sqrt(s * s + (xi + c) ** 2)
    return p[0] * s / (alpha * d2 + (1 - alpha) * (xi + c))


def ds_valid_project(p, theta):
    xi, alpha = p[4], p[5]
    w1 = (1 - alpha) / alpha if alpha > 0.5 else alpha / (1 - alpha)
    w2 = (w1 + xi) / math.sqrt(2 * w1 * xi + xi * xi + 1)
    return np.cos(theta) > -w2


def ds_theta(p, rho):
    """DoubleSphereCamera::unproject (radial part)."""
    xi, alpha = p[4], p[5]
    m = rho / p[0]
    r2 = m * m
    valid = np.ones_like(rho, bool) if alpha <= 0.5 else r2 < 1 / (2 * alpha - 1)
    r2c = np.where(valid, r2, 0)
    mz = (1 - alpha * alpha * r2c) / (alpha * np.sqrt(np.maximum(1 - (2 * alpha - 1) * r2c, 0)) + 1 - alpha)
    mz2 = mz * mz
    k = (mz * xi + np.sqrt(mz2 + (1 - xi * xi) * r2c)) / (mz2 + r2c)
    x, z = k * np.sqrt(r2c), k * mz - xi
    return np.arctan2(x, z), valid


def eucm_rho(p, theta):
    alpha, beta = p[4], p[5]
    s, c = np.sin(theta), np.cos(theta)
    d = np.sqrt(beta * s * s + c * c)
    return p[0] * s / (alpha * d + (1 - alpha) * c)


def eucm_valid_project(p, theta):
    alpha, beta = p[4], p[5]
    w = (1 - alpha) / alpha if alpha > 0.5 else alpha / (1 - alpha)
    return np.cos(theta) > -w * np.sqrt(beta * np.sin(theta) ** 2 + np.cos(theta) ** 2)


def eucm_theta(p, rho):
    """ExtendedUnifiedCamera::unproject (radial part)."""
    alpha, beta = p[4], p[5]
    m = rho / p[0]
    r2 = m * m
    gamma = 1 - alpha
    valid = np.ones_like(rho, bool) if alpha <= 0.5 else r2 < 1 / ((alpha - gamma) * beta)
    r2c = np.where(valid, r2, 0)
    mz = (1 - alpha * alpha * beta * r2c) / (alpha * np.sqrt(np.maximum(1 - (alpha - gamma) * beta * r2c, 0)) + gamma)
    return np.arctan2(np.sqrt(r2c), mz), valid


def fit_models(uw):
    """Least-squares fits of kb4, ds and eucm to rho(theta) on theta in [0, max_theta] (radial error in pixels)."""
    from scipy.optimize import least_squares

    theta = np.linspace(0.0, uw.max_theta, 2000)
    rho = uw.rho(theta)
    cx, cy = uw.c
    fits = {}
    # kb4: rho = F theta + c3 theta^3 + ... + c9 theta^9 is linear in (F, c3, c5, c7, c9)
    A = np.stack([theta ** e for e in (1, 3, 5, 7, 9)], 1)
    x, *_ = np.linalg.lstsq(A, rho, rcond=None)
    fits["kb4"] = np.array([x[0], x[0], cx, cy] + list(x[1:] / x[0]))

    def fit(model_rho, valid, x0, bounds):
        def residual(q):
            p = np.array([q[0], q[0], cx, cy, q[1], q[2]])
            r = model_rho(p, theta) - rho
            # Outside the valid projection region the model does not hold: a large residual keeps it out
            return np.where(valid(p, theta), r, 1e3)

        result = least_squares(residual, x0, bounds=bounds, x_scale="jac")
        q = result.x
        return np.array([q[0], q[0], cx, cy, q[1], q[2]])

    f0 = uw.pixels_per_radian()
    best = None
    for xi0, a0 in ((0.5, 0.6), (1.0, 0.6), (1.5, 0.7), (0.0, 0.6), (-0.2, 0.55)):
        p = fit(ds_rho, ds_valid_project, [f0 * (1 + xi0) / 1.5, xi0, a0], ([1, -1, 0], [10 * f0, 5, 1]))
        err = np.abs(ds_rho(p, theta) - rho).max()
        if best is None or err < best[0]:
            best = (err, p)
    fits["ds"] = best[1]
    best = None
    for a0, b0 in ((0.6, 1.0), (0.7, 1.2), (0.8, 0.9), (0.55, 1.0)):
        p = fit(eucm_rho, eucm_valid_project, [f0, a0, b0], ([1, 0, 0.01], [10 * f0, 1, 10]))
        err = np.abs(eucm_rho(p, theta) - rho).max()
        if best is None or err < best[0]:
            best = (err, p)
    fits["eucm"] = best[1]
    return fits


MODELS = {"kb4": (kb4_rho, lambda p, t: np.ones_like(t, bool), kb4_theta),
          "ds": (ds_rho, ds_valid_project, ds_theta),
          "eucm": (eucm_rho, eucm_valid_project, eucm_theta)}


def evaluate(uw, name, p):
    """Errors [px] of a fitted model: projection over the theta grid of the full field of view, and both directions
    on a 1-px-step grid of the image pixels inside the field of view."""
    model_rho, valid_project, model_theta = MODELS[name]
    theta = np.linspace(0.0, uw.max_theta, 2000)
    rho = uw.rho(theta)
    ok = valid_project(p, theta)
    e_fov = np.where(ok, np.abs(model_rho(p, theta) - rho), np.inf)
    w, h = uw.size
    u, v = np.meshgrid(np.arange(w, dtype=float), np.arange(h, dtype=float))
    r_px = np.hypot(u - uw.c[0], v - uw.c[1]).ravel()
    inside = r_px <= uw.rho(uw.max_theta)
    r_px = r_px[inside]
    t_true = uw.theta(r_px)
    # Projection: true ray -> model pixel
    e_proj = np.where(valid_project(p, t_true), np.abs(model_rho(p, t_true) - r_px), np.inf)
    # Unprojection: pixel -> model ray -> true pixel
    t_model, ok_unproject = model_theta(p, r_px)
    t_model = np.clip(t_model, 0, uw.max_theta * 1.2)
    e_unproj = np.where(ok_unproject, np.abs(uw.rho(t_model) - r_px), np.inf)
    e_img = np.maximum(e_proj, e_unproj)
    return {"fov_rms": float(np.sqrt(np.mean(e_fov ** 2))), "fov_max": float(e_fov.max()),
            "img_rms": float(np.sqrt(np.mean(e_img ** 2))), "img_max": float(e_img.max()),
            "proj_max": float(e_proj.max()), "unproj_max": float(e_unproj.max()), "pixels": int(inside.sum())}


def intrinsics_json(name, p):
    keys = {"kb4": ("fx", "fy", "cx", "cy", "k1", "k2", "k3", "k4"), "ds": ("fx", "fy", "cx", "cy", "xi", "alpha"),
            "eucm": ("fx", "fy", "cx", "cy", "alpha", "beta"), "pinhole": ("fx", "fy", "cx", "cy")}[name]
    return {"camera_type": name, "intrinsics": {k: float(v) for k, v in zip(keys, p)}}


def calibration_crop(capture):
    """The crop [x, y, w, h] that moves a calibration at the sensor mode to the images of a camera_v4l2_gen2 node with
    a crop, or None (no crop, or a calibration at the crop size already)."""
    prop = capture["camera_property"]
    crop = prop.get("crop") or []
    if capture["xeye_model"] is None or len(crop) != 4:
        return None
    x, y, w, h = (int(v) - int(v) % 2 for v in crop)
    size = tuple(int(v) for v in capture["calib"]["image_size"])
    if size == (w, h):
        return None
    sensor = (int(prop.get("width", 0)), int(prop.get("height", 0)))
    if size != sensor:
        raise ValueError(f"the calibration size {size[0]}x{size[1]} is neither the crop {w}x{h} nor the sensor mode "
                         f"{sensor[0]}x{sensor[1]}")
    return [x, y, w, h]


def camera_intrinsics(capture, factor, max_error_px, force_model):
    """(model name, params, size, px/rad, report lines, rho_max or None)."""
    cam = capture["calib"]
    lines = []
    crop = calibration_crop(capture)
    if crop:
        lines.append(f"crop {crop}: the calibration at the sensor mode moves to the crop (c - ({crop[0]}, {crop[1]}), "
                     f"size {crop[2]}x{crop[3]})")
    if cam["model"] == "uwfisheye":
        uw = UwFisheye(cam, factor, crop)
        fits = fit_models(uw)
        results = {name: evaluate(uw, name, p) for name, p in fits.items()}
        lines.append(f"uwfisheye {uw.size[0]}x{uw.size[1]}: f {uw.f:.4f}, c ({uw.c[0]:.4f}, {uw.c[1]:.4f}), "
                     f"{uw.pixels_per_radian():.2f} px/rad, max_theta {math.degrees(uw.max_theta):.2f} deg, image "
                     f"circle radius {uw.rho(uw.max_theta):.1f} px")
        lines.append("fit errors [px] (fov: projection over theta in [0, max_theta]; image: both directions on the "
                     "image pixels inside the field of view, kb4 with the 3 Newton steps of Granite):")
        for name, r in results.items():
            lines.append(f"  {name:5s} fov rms {r['fov_rms']:.4f} max {r['fov_max']:.4f} | image rms {r['img_rms']:.4f} "
                         f"max {r['img_max']:.4f} (projection {r['proj_max']:.4f}, unprojection {r['unproj_max']:.4f}, "
                         f"{r['pixels']} px)")
        if force_model:
            choice = force_model
            reason = "--model"
        elif max(results["kb4"]["img_max"], results["kb4"]["fov_max"]) <= max_error_px:
            choice = "kb4"
            reason = f"kb4 is within {max_error_px} px (the camera model of the evaluated runs)"
        else:
            choice = min(results, key=lambda n: max(results[n]["img_max"], results[n]["fov_max"]))
            reason = f"kb4 exceeds {max_error_px} px; {choice} has the smallest largest error"
        r = results[choice]
        if max(r["img_max"], r["fov_max"]) > max_error_px or r["img_rms"] > max_error_px / 5:
            print(f"warning: the {choice} fit error (rms {r['img_rms']:.3f}, max {max(r['img_max'], r['fov_max']):.3f} "
                  f"px) exceeds the limit {max_error_px} px", file=sys.stderr)
        lines.append(f"model: {choice} ({reason})")
        return choice, fits[choice], uw.size, uw.pixels_per_radian(), lines, (uw.c, uw.rho(uw.max_theta))
    w, h = (int(v) for v in cam["image_size"])
    center = [float(v) for v in cam["center"]]
    if crop:
        w, h = crop[2], crop[3]
        center = [center[0] - crop[0], center[1] - crop[1]]
    if w % factor or h % factor:
        raise ValueError(f"the size {w}x{h} is not a multiple of {factor}")
    fx, fy = (float(v) / factor for v in cam["focal_length"])
    cx, cy = ((v + 0.5) / factor - 0.5 for v in center)
    if cam["model"] == "opencv_fisheye":
        if float(cam.get("alpha", 0.0)) != 0.0:
            raise ValueError("opencv_fisheye with a skew (alpha) has no Granite model")
        p = [fx, fy, cx, cy] + [float(v) for v in cam["distortion"]]
        lines.append("opencv_fisheye is kb4 exactly")
        return "kb4", np.array(p), (w // factor, h // factor), math.sqrt(fx * fy), lines, None
    if cam["model"] == "pinhole":
        if any(float(v) != 0.0 for v in cam.get("lens_distort", [])):
            raise ValueError("pinhole with lens distortion has no Granite model in this converter")
        lines.append("pinhole without distortion")
        return "pinhole", np.array([fx, fy, cx, cy]), (w // factor, h // factor), math.sqrt(fx * fy), lines, None
    raise ValueError(f"camera model {cam['model']} is not supported (uwfisheye, opencv_fisheye, pinhole)")


def base_pixels_per_radian(path):
    cam = json.loads(pathlib.Path(path).read_text())["value0"]["intrinsics"][0]
    i = cam["intrinsics"]
    if cam["camera_type"] in ("kb4", "pinhole"):
        return math.sqrt(i["fx"] * i["fy"])
    if cam["camera_type"] == "ds":  # d rho / d theta at the center: f / (1 + xi)
        return math.sqrt(i["fx"] * i["fy"]) / (1 + i["xi"])
    if cam["camera_type"] == "eucm":
        return math.sqrt(i["fx"] * i["fy"])
    raise ValueError(f"no image scale of the base camera {cam['camera_type']}")


# ------------------------------------------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture_yaml", type=pathlib.Path, help="capture.yaml of an xeye capture (or its folder)")
    ap.add_argument("--camera", default="", help="camera of a multi-camera capture, e.g. cam1 (default: the first)")
    ap.add_argument("--downscale", type=int, default=1, help="write the camera at 1/N of the calibrated size (1, 2, 3)")
    ap.add_argument("--calib-out", type=pathlib.Path, required=True, help="Granite calibration JSON to write")
    ap.add_argument("--config-out", type=pathlib.Path, help="Granite VIO config JSON to write")
    ap.add_argument("--vis-msg-ip", default="10.20.10.42", help="--target live: vis_msg_ip of the config")
    ap.add_argument("--vis-msg-port", type=int, default=5100, help="--target live: vis_msg_port of the config")
    ap.add_argument("--base", type=pathlib.Path, default=DEFAULT_BASE, help="base VIO config")
    ap.add_argument("--base-calib", type=pathlib.Path, default=DEFAULT_BASE_CALIB,
                    help="calibration of the camera that the base config was evaluated with (its px/rad)")
    ap.add_argument("--target", choices=("live", "capture"), default="live",
                    help="live: cam_time_offset 0 and raw IMU samples; capture: the replay of this capture")
    ap.add_argument("--imu-samples", choices=("raw", "calibrated"),
                    help="raw: write the biases into the calibration; calibrated: the samples have no bias "
                         "(default: raw for live, the capture IMU property 'calibrated' for capture)")
    ap.add_argument("--imu-intrinsics", type=pathlib.Path, help="imu_intrinsics_<node>.yaml (Allan variance)")
    ap.add_argument("--model", choices=("kb4", "ds", "eucm"), help="force the camera model of a uwfisheye fit")
    ap.add_argument("--max-error-px", type=float, default=0.5,
                    help="largest fit error [px, written size] that keeps kb4 and that passes without a warning")
    ap.add_argument("--fps", type=float,
                    help="camera.fps of the live config (default: frame_rate of a kurokesu_ar0234_pi or b0429_pi "
                         "node, else 15: Granite ran at 15 fps)")
    ap.add_argument("--sensor-size", help="live sensor mode WxH (default: the calibrated size)")
    ap.add_argument("--i2c-device", help="camera.i2c_device of the live config (default: the IMU port of the capture)")
    ap.add_argument("--max-exposure-ms", type=float, default=5.0,
                    help="auto exposure limit (openvio: 1.5 px blur at 1 rad/s and 300 px/rad)")
    ap.add_argument("--metering-fraction", type=float, default=0.95,
                    help="metering circle radius as a fraction of the fisheye image circle")
    a = ap.parse_args()

    path = a.capture_yaml / "capture.yaml" if a.capture_yaml.is_dir() else a.capture_yaml
    capture = read_capture(path, a.camera)
    xeye = capture["xeye_model"] is not None
    if a.fps is None:
        a.fps = float(capture["camera_property"].get("frame_rate", 30)) if xeye else 15.0
    if a.downscale < 1:
        raise ValueError("--downscale must be positive")
    model, params, size, px_per_rad, lines, circle = camera_intrinsics(capture, a.downscale, a.max_error_px, a.model)
    print(f"camera {capture['name']} of {path}")
    for line in lines:
        print("  " + line)

    imu_samples = a.imu_samples or ("raw" if a.target == "live" else
                                    ("calibrated" if capture["samples_calibrated"] else "raw"))
    zero_bias = imu_samples == "calibrated"
    bias_acc = [0.0] * 3 if zero_bias or capture["bias_acc"] is None else [float(v) for v in capture["bias_acc"]]
    bias_gyro = [0.0] * 3 if zero_bias or capture["bias_gyro"] is None else [float(v) for v in capture["bias_gyro"]]
    if not zero_bias and (capture["bias_acc"] is None or capture["bias_gyro"] is None):
        print("warning: the capture has no IMU bias; the calibration has zero bias", file=sys.stderr)
    noise, noise_source = capture["noise"], f"calib.{capture['imu']}.intrinsic.noise (Allan variance)"
    if a.imu_intrinsics:
        result = (yaml.safe_load(a.imu_intrinsics.read_text()).get("imu_intrinsics") or {}).get("result") or {}
        noise, noise_source = result.get("noise"), f"{a.imu_intrinsics} (Allan variance)"
    if not noise:
        noise, noise_source = OPENVIO_IMU_NOISE, "openvio config/pi5_b0429.yaml (MPU-9250 datasheet value x 2; not calibrated)"
    offset_s = capture["time_offset"] if a.target == "capture" or xeye else 0.0
    T = capture["T_ic"]
    qw, qx, qy, qz = rot_to_quat(T[:3, :3])
    calib = {"value0": {
        "T_imu_cam": [{"px": T[0, 3], "py": T[1, 3], "pz": T[2, 3], "qx": qx, "qy": qy, "qz": qz, "qw": qw}],
        "intrinsics": [intrinsics_json(model, params)],
        "resolution": [[size[0], size[1]]],
        "main_cam_idx": [0], "stereo_pairs": [],
        "cam_update_rate": float(a.fps if a.target == "live" else (capture["fps"] or a.fps)),
        "vignette": [],
        "calib_accel_bias": bias_acc + [0.0] * 6,
        "calib_gyro_bias": bias_gyro + [0.0] * 9,
        "imu_update_rate": capture["imu_rate"],
        "accel_noise_std": [float(noise["acc_noise_density"])] * 3,
        "gyro_noise_std": [float(noise["gyro_noise_density"])] * 3,
        "accel_bias_std": [float(noise["acc_bias_random_walk"])] * 3,
        "gyro_bias_std": [float(noise["gyro_bias_random_walk"])] * 3,
        "T_mocap_world": {"px": 0.0, "py": 0.0, "pz": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
        "T_imu_marker": {"px": 0.0, "py": 0.0, "pz": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
        "mocap_time_offset_ns": 0, "mocap_to_imu_offset_ns": 0,
        "cam_time_offset_ns": int(round(offset_s * 1e9))}}
    a.calib_out.write_text(json.dumps(calib, indent=4) + "\n")
    print(f"{a.calib_out}: {model} {size[0]}x{size[1]}"
          + (f" (1/{a.downscale} of the calibrated size)" if a.downscale > 1 else ""))
    print("  T_imu_cam =\n    " + np.array2string(T, precision=6, suppress_small=True).replace("\n", "\n    "))
    print(f"  IMU biases: {'zero (the samples are calibrated)' if zero_bias else 'accel ' + str(bias_acc) + ', gyro ' + str(bias_gyro)}")
    print(f"  IMU noise: {noise_source}: accel {noise['acc_noise_density']:g}, gyro {noise['gyro_noise_density']:g}, "
          f"accel walk {noise['acc_bias_random_walk']:g}, gyro walk {noise['gyro_bias_random_walk']:g}")
    if a.target == "live" and xeye:
        print(f"  cam_time_offset_ns {calib['value0']['cam_time_offset_ns']}: time_offset of the capture; the live app "
              f"stamps the images and the IMU samples as the xeye nodes ({capture['xeye_model']})")
    elif a.target == "live":
        print(f"  cam_time_offset_ns 0: the live image time is the exposure center; the capture time_offset "
              f"{capture['time_offset']:.6f} s holds for the xeye image stamps only")
    else:
        print(f"  cam_time_offset_ns {calib['value0']['cam_time_offset_ns']} (time_offset of the capture); the runner "
              f"applies it: --cam-time-offset-ns {calib['value0']['cam_time_offset_ns']}")

    if not a.config_out:
        return
    base = json.loads(a.base.read_text())
    cfg = base["value0"]
    changes = []

    def set_key(key, value, reason):
        old = cfg.get(key, GRANITE_DEFAULTS.get(key, "(absent)"))
        cfg[key] = value
        changes.append(f"{key}: {old} -> {value} ({reason})")

    set_key("config.vio_use_deterministic", False, "live: the deterministic sums cost about 7 % and add nothing")
    if a.target == "live":
        reason = "live: visualization messages for a viewer on the ground (doc/granite_vis.proto)"
        set_key("send_vis_msg", True, reason)
        set_key("vis_msg_ip", a.vis_msg_ip, reason)
        set_key("vis_msg_port", a.vis_msg_port, reason)
        set_key("vis_msg_image_hz", 20.0, reason)
        set_key("vis_msg_points_hz", 2.0, reason)
        set_key("vis_msg_encode", "mjpeg", reason)
        set_key("vis_msg_jpeg_quality", 80, reason)
        set_key("vis_msg_max_kbps", 2000.0, reason)
        set_key("vis_msg_h264_rate_control", "bitrate_cap", reason)
        set_key("vis_msg_h264_crf", 23.0, reason)
    base_scale = base_pixels_per_radian(a.base_calib)
    ratio = px_per_rad / base_scale
    for key, what in SCALED_KEYS:
        value = float(cfg.get(key, GRANITE_DEFAULTS[key]))
        if value > 0:
            set_key(key, round(value * ratio, 3),
                    f"{what} [px]; {value:g} at {base_scale:.1f} px/rad of the base camera, here {px_per_rad:.1f} px/rad")
    for key in [k for k in cfg if k.startswith("camera.") or k.startswith("imu.")]:
        changes.append(f"{key}: {cfg.pop(key)} -> removed (replaced by the camera.* and imu.* keys of this capture)")
    prop = capture["camera_property"]
    if xeye:
        # The sensor mode of the node, and its crop
        sensor = (int(prop.get("width", 0)), int(prop.get("height", 0)))
        crop = prop.get("crop") or []
        output = (int(crop[2]) - int(crop[2]) % 2, int(crop[3]) - int(crop[3]) % 2) if len(crop) == 4 else sensor
        if output != (size[0] * a.downscale, size[1] * a.downscale):
            print(f"warning: the images {output[0]}x{output[1]} of the node are not {a.downscale} x the calibration "
                  f"{size[0]}x{size[1]}; the live app rejects the calibration", file=sys.stderr)
        live = {"camera.model": capture["xeye_model"], "camera.downscale": a.downscale, "camera.fps": a.fps}
        if a.downscale == 2 and capture["xeye_model"] == "kurokesu_ar0234_pi" and sensor == (1920, 1200):
            full = [int(v) for v in crop] if len(crop) == 4 else [0, 0, 1920, 1200]
            if all(v % 4 == 0 for v in full):
                print(f"  live: the 2x2 binning mode of the sensor gives the {output[0] // 2}x{output[1] // 2} images "
                      f"(crop {[v // 2 for v in full]} of 960x600; camera.sensor_binning false: the host halves)")
            else:
                print(f"  live: the crop {full} has values that are not multiples of 4; the host halves the image "
                      f"(no binning)")
        if "port" in capture["camera_node"]:
            live["camera.port"] = capture["camera_node"]["port"]
        for key in XEYE_CAMERA_KEYS:
            if key in prop:
                live["camera." + key] = prop[key]
        pixel_format = str(live.get("camera.pixel_format", "gray8"))
        if pixel_format != "gray8":
            print(f"warning: the node sends {pixel_format}; the live app takes gray8 only (the same demosaiced gray "
                  f"as an xeye gray8 node)", file=sys.stderr)
            live["camera.pixel_format"] = "gray8"
        if circle is not None and "camera.auto_exposure_circle" not in live:
            c, r = circle
            print(f"  note: the node has no auto_exposure_circle; --auto_exposure_circle {round(float(c[0]))} "
                  f"{round(float(c[1]))} {round(float(r) * a.metering_fraction)} meters inside the fisheye image circle "
                  f"(at the written size)")
    else:
        sensor = capture["sensor_size"] if capture["live"] and capture["sensor_size"] else \
            tuple(int(v) for v in capture["calib"]["image_size"])
        if a.sensor_size:
            sensor = tuple(int(v) for v in a.sensor_size.split("x"))
        if sensor != (size[0] * a.downscale, size[1] * a.downscale):
            print(f"warning: the sensor mode {sensor[0]}x{sensor[1]} is not {a.downscale} x the calibration "
                  f"{size[0]}x{size[1]}; the live app rejects the calibration", file=sys.stderr)
        live = {"camera.model": "b0429", "camera.width": sensor[0], "camera.height": sensor[1],
                "camera.downscale": a.downscale, "camera.fps": a.fps, "camera.exposure_ms": 0.0,
                "camera.gain_db": 0.0, "camera.max_exposure_ms": a.max_exposure_ms, "camera.brightness_target": 100.0}
        if circle is not None:
            c, r = circle
            live["camera.metering_circle"] = [round(float(c[0]), 2), round(float(c[1]), 2),
                                              round(float(r) * a.metering_fraction, 1)]
    mavlink = capture["mavlink"]
    if mavlink is not None:
        live["imu.type"] = "imu_mavlink"
        for key in ("udp", "port", "baudrate", "freq_hz"):
            if key in mavlink:
                live["imu." + key] = mavlink[key]
    else:
        live["imu.type"] = "mpu_i2c"
        i2c = a.i2c_device or capture["i2c_device"]
        if i2c:
            live["camera.i2c_device"] = i2c
        live["camera.imu_rate"] = capture["imu_rate"]
    for key, value in live.items():
        cfg[key] = value
    a.config_out.write_text(json.dumps(base, indent=4) + "\n")
    print(f"{a.config_out}: base {a.base.name}")
    for line in changes:
        print("  " + line)
    print("  kept in pixels: optical_flow_detection_grid_size, optical_flow_max_recovered_dist2, vio_obs_std_dev, "
          "vio_obs_huber_thresh, vio_mono_init_pixel_sigma (cell size and pixel noise, as openvio)")
    print("  live options: " + ", ".join(f"{k}={v}" for k, v in live.items()))


if __name__ == "__main__":
    try:
        main()
    except (ValueError, KeyError, OSError) as error:
        sys.exit(f"error: {error}")
