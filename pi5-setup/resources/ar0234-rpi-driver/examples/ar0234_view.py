#!/usr/bin/env python3
"""Minimal live viewer for the Kurokesu AR0234 on a Raspberry Pi 5 (Ubuntu).

The stock libcamera on Ubuntu 24.04 does not support the AR0234, so this
program reads RAW Bayer frames directly from the rp1-cfe V4L2 node.
It uses only numpy, tkinter and Pillow.

Usage: python3 ar0234_view.py [--mode 960x600|1280x720|1920x1080|1920x1200]
                              [--fps 30] [--hflip] [--vflip]

Mouse: wheel = zoom at the cursor, left drag = pan, double click = reset.
Keyboard: q or Esc = quit.
"""
import argparse
import ctypes
import glob
import signal
import subprocess
import threading
import time

import numpy as np
import tkinter as tk
from PIL import Image, ImageTk

SENSOR = "ar0234 11-0010"
# Measured on the Pi 5: frame period = LINE_TIME * (height + vertical_blanking)
# for all four modes.
LINE_TIME = 13.6e-6
# Mains light flickers at 100 Hz or 120 Hz. An exposure of N / 120 s
# (60 Hz mains) gives the same brightness in each frame.
FLICKER_PERIOD_MS = 1000 / 120
MBUS = "SGRBG10_1X10"


def die_with_parent():
    """Stop the child process when this program stops (Linux prctl)."""
    PR_SET_PDEATHSIG = 1
    ctypes.CDLL("libc.so.6").prctl(PR_SET_PDEATHSIG, signal.SIGTERM)


def run(*cmd):
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)


def find_media():
    """Return the media device of the rp1-cfe that has the AR0234."""
    for dev in sorted(glob.glob("/dev/media*")):
        out = subprocess.run(["media-ctl", "-d", dev, "-p"],
                             capture_output=True, text=True).stdout
        if "rp1-cfe" in out and SENSOR in out:
            return dev, out
    raise SystemExit("AR0234 not found. Check the overlay and the cable.")


def node_of(topology, entity):
    """Return the device node of a media entity."""
    lines = topology.splitlines()
    for i, line in enumerate(lines):
        if f": {entity} (" in line:
            for nxt in lines[i + 1:i + 4]:
                if "device node name" in nxt:
                    return nxt.split()[-1]
    raise SystemExit(f"No device node for {entity}")


def setup_pipeline(w, h):
    media, topo = find_media()
    fmt = (f"{MBUS}/{w}x{h} field:none colorspace:raw xfer:none "
           "ycbcr:601 quantization:full-range")
    run("media-ctl", "-d", media, "-l",
        '"csi2":4 -> "rp1-cfe-csi2_ch0":0 [1]')
    run("media-ctl", "-d", media, "-V", f'"{SENSOR}":0 [fmt:{MBUS}/{w}x{h}]')
    run("media-ctl", "-d", media, "-V", f'"csi2":0 [fmt:{fmt}]')
    run("media-ctl", "-d", media, "-V", f'"csi2":4 [fmt:{fmt}]')
    # The embedded data pad must match the sensor, or STREAMON fails (EPIPE).
    csi2 = node_of(topo, "csi2")
    run("v4l2-ctl", "-d", csi2,
        "--set-subdev-fmt", "pad=1,width=2400,height=2,code=0x7002")
    video = node_of(topo, "rp1-cfe-csi2_ch0")
    run("v4l2-ctl", "-d", video,
        f"--set-fmt-video=width={w},height={h},pixelformat=pgAA")
    return video, node_of(topo, SENSOR)


def unpack_raw10(buf, w, h):
    """Unpack MIPI RAW10 (4 pixels in 5 bytes) to 8-bit (top 8 bits)."""
    p = np.frombuffer(buf, np.uint8).reshape(h, w // 4, 5)
    return p[..., :4].reshape(h, w)


def bayer_grbg_to_rgb_half(raw8):
    """Half-size demosaic: each 2x2 GRBG cell gives one RGB pixel."""
    out = np.empty((raw8.shape[0] // 2, raw8.shape[1] // 2, 3), np.uint8)
    out[..., 0] = raw8[0::2, 1::2]
    out[..., 1] = (raw8[0::2, 0::2].astype(np.uint16) + raw8[1::2, 1::2]) >> 1
    out[..., 2] = raw8[1::2, 0::2]
    return out


def bayer_grbg_to_rgb_full(raw8):
    """Full-size bilinear demosaic. The input starts on a GRBG cell.

    Each Bayer position gets only the values that it needs.
    """
    h, w = raw8.shape
    p = np.pad(raw8, 1, mode="reflect").astype(np.uint16)

    def at(y, x, dy=0, dx=0):
        """Pixels of Bayer position (y, x), moved by (dy, dx)."""
        return p[1 + y + dy:1 + y + dy + h:2, 1 + x + dx:1 + x + dx + w:2]

    def hor(y, x):
        return (at(y, x, 0, -1) + at(y, x, 0, 1)) >> 1

    def ver(y, x):
        return (at(y, x, -1, 0) + at(y, x, 1, 0)) >> 1

    def cross(y, x):
        return (at(y, x, 0, -1) + at(y, x, 0, 1)
                + at(y, x, -1, 0) + at(y, x, 1, 0)) >> 2

    def diag(y, x):
        return (at(y, x, -1, -1) + at(y, x, -1, 1)
                + at(y, x, 1, -1) + at(y, x, 1, 1)) >> 2

    out = np.empty((h, w, 3), np.uint8)
    # Cell layout:  G1 R
    #               B  G2
    for (y, x), r, g, b in (((0, 0), hor, at, ver),     # G1
                            ((0, 1), at, cross, diag),  # R
                            ((1, 0), diag, cross, at),  # B
                            ((1, 1), ver, at, hor)):    # G2
        out[y::2, x::2, 0] = r(y, x)
        out[y::2, x::2, 1] = g(y, x)
        out[y::2, x::2, 2] = b(y, x)
    return out


class WhiteBalance:
    """Gray-world white balance. The gains change slowly to stop flicker."""

    def __init__(self, alpha=0.05):
        self.alpha = alpha
        self.gain = None
        self.lut = None

    def update(self, raw8):
        """Measure the gains on a sparse grid of the full frame."""
        r = raw8[0::8, 1::8].mean()
        g = (raw8[0::8, 0::8].mean() + raw8[1::8, 1::8].mean()) / 2
        b = raw8[1::8, 0::8].mean()
        gain = g / (np.array([r, g, b]) + 1e-3)
        if self.gain is None:
            self.gain = gain
        else:
            self.gain += self.alpha * (gain - self.gain)
        levels = np.arange(256, dtype=np.float32)
        self.lut = [np.clip(levels * k, 0, 255).astype(np.uint8)
                    for k in self.gain]

    def apply_raw(self, raw8):
        """Apply the gains to GRBG data. The input starts on a GRBG cell."""
        r_lut, g_lut, b_lut = self.lut
        out = g_lut[raw8]
        out[0::2, 1::2] = r_lut[raw8[0::2, 1::2]]
        out[1::2, 0::2] = b_lut[raw8[1::2, 0::2]]
        return out


class View:
    """Zoom and pan state. All positions are in sensor pixels."""

    MAX_ZOOM = 16.0

    def __init__(self, w, h, disp_w, disp_h):
        self.w, self.h = w, h
        self.disp_w, self.disp_h = disp_w, disp_h
        self.reset()

    def reset(self):
        self.zoom = 1.0
        self.cx, self.cy = self.w / 2, self.h / 2

    def area(self):
        """Return the visible area (x0, y0, cw, ch) as float values."""
        cw, ch = self.w / self.zoom, self.h / self.zoom
        self.cx = min(max(self.cx, cw / 2), self.w - cw / 2)
        self.cy = min(max(self.cy, ch / 2), self.h - ch / 2)
        return self.cx - cw / 2, self.cy - ch / 2, cw, ch

    def crop(self):
        """Return the crop for the Bayer data. All values are even."""
        fx0, fy0, fcw, fch = self.area()
        cw = max(2, int(fcw) // 2 * 2)
        ch = max(2, int(fch) // 2 * 2)
        x0 = min(int(fx0) // 2 * 2, self.w - cw)
        y0 = min(int(fy0) // 2 * 2, self.h - ch)
        return x0, y0, cw, ch

    def to_sensor(self, dx, dy):
        x0, y0, cw, ch = self.area()
        return x0 + dx * cw / self.disp_w, y0 + dy * ch / self.disp_h

    def zoom_at(self, dx, dy, factor):
        """Zoom and keep the sensor point under the cursor fixed."""
        px, py = self.to_sensor(dx, dy)
        self.zoom = min(max(self.zoom * factor, 1.0), self.MAX_ZOOM)
        cw, ch = self.w / self.zoom, self.h / self.zoom
        self.cx = px - dx * cw / self.disp_w + cw / 2
        self.cy = py - dy * ch / self.disp_h + ch / 2

    def pan(self, ddx, ddy):
        _, _, cw, ch = self.area()
        self.cx -= ddx * cw / self.disp_w
        self.cy -= ddy * ch / self.disp_h

    def render(self, raw8, wb):
        x0, y0, cw, ch = self.crop()
        crop = wb.apply_raw(raw8[y0:y0 + ch, x0:x0 + cw])
        if cw >= self.disp_w:
            # The half-size image needs 2x scale or less. It is sufficient.
            rgb = bayer_grbg_to_rgb_half(crop)
        else:
            rgb = bayer_grbg_to_rgb_full(crop)
        img = Image.fromarray(rgb)
        # Show the sensor pixels as blocks at high zoom.
        scale = self.disp_w / img.width
        method = Image.NEAREST if scale >= 3 else Image.BILINEAR
        return img.resize((self.disp_w, self.disp_h), method)


class Grabber(threading.Thread):
    """Read frames from v4l2-ctl and keep only the newest frame."""

    def __init__(self, video, w, h):
        super().__init__(daemon=True)
        self.size = w * h * 5 // 4
        self.latest = None
        self.count = 0
        self.proc = subprocess.Popen(
            ["v4l2-ctl", "-d", video, "--stream-mmap=4",
             "--stream-count=0", "--stream-to=-"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            bufsize=self.size * 2, preexec_fn=die_with_parent)

    def run(self):
        while True:
            buf = self.proc.stdout.read(self.size)
            if len(buf) < self.size:
                break
            self.latest = buf
            self.count += 1

    def stop(self):
        self.proc.terminate()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="960x600")
    ap.add_argument("--fps", type=float, default=30.0)
    # The sensor does the flip. Use both flips to rotate the image 180 deg.
    # The flips keep the GRBG Bayer order.
    ap.add_argument("--hflip", action="store_true", help="mirror left-right")
    ap.add_argument("--vflip", action="store_true", help="mirror up-down")
    args = ap.parse_args()
    w, h = map(int, args.mode.split("x"))

    video, sensor = setup_pipeline(w, h)

    def set_ctrl(name, value):
        subprocess.run(["v4l2-ctl", "-d", sensor, "-c", f"{name}={value}"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # The sensor frame rate comes from the vertical blanking.
    vblank = max(21, round(1 / (args.fps * LINE_TIME)) - h)
    set_ctrl("vertical_blanking", vblank)
    max_exp_ms = (h + vblank - 6) * LINE_TIME * 1000
    # The driver locks the flip controls during streaming. Set them first.
    set_ctrl("horizontal_flip", int(args.hflip))
    set_ctrl("vertical_flip", int(args.vflip))

    grab = Grabber(video, w, h)
    grab.start()

    root = tk.Tk()
    flips = "".join(n for n, on in ((" hflip", args.hflip),
                                     (" vflip", args.vflip)) if on)
    root.title(f"AR0234 {w}x{h}{flips}")
    disp_w = min(w, 960)
    disp_h = h * disp_w // w
    photo = ImageTk.PhotoImage(Image.new("RGB", (disp_w, disp_h)))
    image = tk.Label(root, image=photo, cursor="crosshair")
    image.pack()
    status = tk.Label(root, anchor="w")
    status.pack(fill="x")

    def set_exposure(ms):
        set_ctrl("exposure", max(2, round(float(ms) / 1000 / LINE_TIME)))

    exp = tk.Scale(root, label="exposure [ms] (N x 8.33 ms = no flicker)",
                   from_=0.1, to=max_exp_ms, resolution=0.01,
                   orient="horizontal", command=set_exposure)
    exp.set(min(FLICKER_PERIOD_MS, max_exp_ms))
    exp.pack(fill="x")
    gain = tk.Scale(root, label="analogue_gain", from_=13, to=64,
                    orient="horizontal",
                    command=lambda v: set_ctrl("analogue_gain", v))
    gain.set(14)
    gain.pack(fill="x")

    wb = WhiteBalance()
    view = View(w, h, disp_w, disp_h)
    state = {"count": 0, "time": time.monotonic(), "shown": 0,
             "fps": 0.0, "drag": None}

    # Mouse: wheel = zoom at the cursor, left drag = pan,
    # double click = reset. X11 sends the wheel as button 4 and button 5.
    def on_wheel(event, step):
        view.zoom_at(event.x, event.y, 1.25 if step > 0 else 0.8)
        state["shown"] = -1

    def on_drag(event):
        if state["drag"] is not None:
            view.pan(event.x - state["drag"][0], event.y - state["drag"][1])
            state["shown"] = -1
        state["drag"] = (event.x, event.y)

    def on_reset(_event):
        view.reset()
        state["shown"] = -1

    image.bind("<Button-4>", lambda e: on_wheel(e, 1))
    image.bind("<Button-5>", lambda e: on_wheel(e, -1))
    image.bind("<MouseWheel>", lambda e: on_wheel(e, e.delta))
    image.bind("<B1-Motion>", on_drag)
    image.bind("<ButtonRelease-1>", lambda e: state.update(drag=None))
    image.bind("<Double-Button-1>", on_reset)

    def tick():
        if grab.count != state["shown"]:
            state["shown"] = grab.count
            raw8 = unpack_raw10(grab.latest, w, h)
            wb.update(raw8)
            # Paste into the same image. A new image per frame can flicker.
            photo.paste(view.render(raw8, wb))
        now = time.monotonic()
        if now - state["time"] >= 1.0:
            state["fps"] = (grab.count - state["count"]) / (now - state["time"])
            state["count"], state["time"] = grab.count, now
        status.configure(text=f"sensor {state['fps']:.1f} fps   "
                              f"zoom x{view.zoom:.2f}   "
                              "(wheel: zoom, drag: pan, double click: reset, "
                              "q/Esc: quit)")
        root.after(15, tick)

    def close(_event=None):
        grab.stop()
        root.destroy()

    # Keyboard: q or Esc = quit.
    for key in ("<q>", "<Q>", "<Escape>"):
        root.bind(key, close)
    root.protocol("WM_DELETE_WINDOW", close)
    tick()
    root.mainloop()


if __name__ == "__main__":
    main()
