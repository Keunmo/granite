#!/usr/bin/env python3
"""Receiver of the Granite visualization messages (send_vis_msg, doc/granite_vis.proto) for the ground.

Usage: vis_receiver.py [--port 5100] [--bind 0.0.0.0] [--show] [--trail 20000]
  Prints the receive statistics once per second: messages, kB/s, images, the landmarks and keypoints of the last
  image message, the seq gaps (messages lost in the network) and the publisher's skipped count.
  --show  opens an OpenCV window: the last JPEG camera image with the keypoints (green), and a top view (x right,
          y up) of the trajectory (white), the window states (yellow) and the landmarks of the window (cyan).
          Keys: q quits, c clears the trajectory. H.264 images and the landmark projections: use the UVIFY VIO
          Viewer (tools/vio_viewer).
The parser reads the protobuf wire format of GraniteVis version 2 without the protobuf library.
"""

import argparse
import collections
import socket
import struct
import sys
import time

FIELDS = {1: "version", 2: "seq", 3: "t_ns", 4: "map_idx", 5: "pose", 6: "velocity", 7: "bias_gyro",
          8: "bias_accel", 9: "healthy", 10: "keyframe", 11: "window_states", 12: "window_frames", 13: "points",
          16: "image", 17: "image_width", 18: "image_height", 19: "skipped", 21: "point_ids", 22: "keypoints",
          23: "keypoint_ids", 24: "image_codec", 25: "image_keyframe", 29: "image_seq", 40: "fragment_seq",
          41: "fragment_index", 42: "fragment_count", 43: "fragment_data"}
FLOAT_FIELDS = {"pose", "velocity", "bias_gyro", "bias_accel", "window_states", "window_frames", "points"}
DELTA_FIELDS = {"point_ids", "keypoint_ids"}


def varint(buf, i):
    value, shift = 0, 0
    while True:
        b = buf[i]
        i += 1
        value |= (b & 0x7F) << shift
        if b < 0x80:
            return value, i
        shift += 7


def parse(buf):
    """GraniteVis bytes -> dict (proto3 defaults for the absent fields)."""
    msg = {name: 0 for name in FIELDS.values()}
    msg.update({name: [] for name in FLOAT_FIELDS | DELTA_FIELDS}, keypoints=[], image=b"", fragment_data=b"",
               healthy=False, keyframe=False)
    i = 0
    while i < len(buf):
        key, i = varint(buf, i)
        field, wire = key >> 3, key & 7
        name = FIELDS.get(field)
        if wire == 0:
            value, i = varint(buf, i)
        elif wire == 2:
            size, i = varint(buf, i)
            value, i = buf[i:i + size], i + size
        elif wire == 5:
            value, i = buf[i:i + 4], i + 4
        elif wire == 1:
            value, i = buf[i:i + 8], i + 8
        else:
            raise ValueError(f"wire type {wire} of field {field}")
        if name is None:
            continue
        if name in FLOAT_FIELDS:
            msg[name] = list(struct.unpack(f"<{len(value) // 4}f", value))
        elif name in DELTA_FIELDS:
            ids, j, v = [], 0, 0
            while j < len(value):
                d, j = varint(value, j)
                v += d
                ids.append(v)
            msg[name] = ids
        elif name == "keypoints":  # uint16 in 1/8 pixel
            msg[name] = [x / 8.0 for x in struct.unpack(f"<{len(value) // 2}H", value)]
        elif name in ("healthy", "keyframe"):
            msg[name] = bool(value)
        else:
            msg[name] = value
    return msg


class Joiner:
    """Joins the fragments of the large messages (fragment_seq != 0); keeps at most `keep` messages."""

    def __init__(self, keep=4):
        self.pending = collections.OrderedDict()
        self.keep = keep

    def add(self, msg):
        """A parsed datagram -> the bytes of a complete message, or None."""
        seq, count = msg["fragment_seq"], msg["fragment_count"]
        parts = self.pending.setdefault((seq, count), {})
        parts[msg["fragment_index"]] = msg["fragment_data"]
        while len(self.pending) > self.keep:
            self.pending.popitem(last=False)
        if len(parts) < count:
            return None
        del self.pending[(seq, count)]
        return b"".join(parts[i] for i in range(count))


def messages(datagrams):
    """Datagrams -> parsed messages, with the fragments joined."""
    joiner = Joiner()
    for buf in datagrams:
        msg = parse(buf)
        if msg["fragment_seq"]:
            joined = joiner.add(msg)
            if joined is None:
                continue
            msg = parse(joined)
        yield msg


class View:
    def __init__(self, trail):
        import cv2
        import numpy as np
        self.cv2, self.np = cv2, np
        self.trajectory = collections.deque(maxlen=trail)
        self.image = None
        self.detail = None
        self.points = []
        self.last = None

    def update(self, msg):
        cv2, np = self.cv2, self.np
        if msg["pose"]:
            self.trajectory.append(msg["pose"][:3])
        if msg["points"]:
            self.points = msg["points"]
        if msg["image"] and msg["image_codec"] == 0:
            self.image = cv2.imdecode(np.frombuffer(msg["image"], np.uint8), cv2.IMREAD_GRAYSCALE)
            self.detail = msg  # keypoints of this image
        self.last = msg

    def draw(self):
        cv2, np = self.cv2, self.np
        msg = self.last
        detail = self.detail or msg
        w, h = msg["image_width"] or 448, msg["image_height"] or 400
        cam = cv2.cvtColor(self.image, cv2.COLOR_GRAY2BGR) if self.image is not None else np.zeros((h, w, 3), np.uint8)
        k = detail["keypoints"]
        for x, y in zip(k[0::2], k[1::2]):
            cv2.circle(cam, (int(x), int(y)), 2, (0, 255, 0), -1)
        top = np.zeros((h, h, 3), np.uint8)
        pts = list(self.trajectory)
        win = msg["window_states"]
        lm = self.points
        xs = [q[0] for q in pts] + win[0::7]
        ys = [q[1] for q in pts] + win[1::7]
        if xs:
            cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
            span = max(max(xs) - min(xs), max(ys) - min(ys), 2.0) * 1.2
            s = h / span
            to_px = lambda x, y: (int(h / 2 + (x - cx) * s), int(h / 2 - (y - cy) * s))
            for x, y in zip(lm[0::3], lm[1::3]):
                cv2.circle(top, to_px(x, y), 1, (255, 255, 0), -1)
            for a, b in zip(pts[:-1], pts[1:]):
                cv2.line(top, to_px(a[0], a[1]), to_px(b[0], b[1]), (255, 255, 255), 1)
            for x, y in zip(win[0::7], win[1::7]):
                cv2.circle(top, to_px(x, y), 3, (0, 255, 255), 1)
        health = "healthy" if msg["healthy"] else "UNHEALTHY"
        v = msg["velocity"] or [0, 0, 0]
        cv2.putText(top, f"seq {msg['seq']}  {health}  |v| {(v[0]**2 + v[1]**2 + v[2]**2) ** 0.5:.2f} m/s",
                    (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0) if msg["healthy"] else (0, 0, 255), 1)
        if cam.shape[0] != h:
            cam = cv2.resize(cam, (int(cam.shape[1] * h / cam.shape[0]), h))
        cv2.imshow("granite vis", np.hstack([cam, top]))
        return cv2.waitKey(1) & 0xFF


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", type=int, default=5100)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--trail", type=int, default=20000, help="trajectory points kept for --show")
    a = ap.parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    sock.bind((a.bind, a.port))
    sock.settimeout(0.05)
    view = View(a.trail) if a.show else None
    print(f"listening on {a.bind}:{a.port}", flush=True)
    window = collections.Counter()
    last_seq, last_print = None, time.monotonic()
    msg = detail = None
    landmarks = 0
    joiner = Joiner()
    while True:
        try:
            buf = sock.recv(65535)
        except socket.timeout:
            buf = None
        if buf:
            window["bytes"] += len(buf) + 28
            msg = parse(buf)
            if msg["fragment_seq"]:
                joined = joiner.add(msg)
                if joined is None:
                    continue
                msg = parse(joined)
            if msg["image"]:
                detail = msg
            if msg["points"]:
                landmarks = len(msg["points"]) // 3
            window["messages"] += 1
            window["images"] += bool(msg["image"])
            if last_seq is not None and msg["seq"] > last_seq + 1:
                window["lost"] += msg["seq"] - last_seq - 1
            last_seq = msg["seq"]
            if view:
                view.update(msg)
        now = time.monotonic()
        if now - last_print >= 1.0:
            if msg:
                print(f"msg/s {window['messages'] / (now - last_print):5.1f}  kB/s "
                      f"{window['bytes'] / 1e3 / (now - last_print):7.1f}  images {window['images']}  "
                      f"lost {window['lost']}  skipped {msg['skipped']}  "
                      f"landmarks {landmarks}  "
                      f"keypoints {len(detail['keypoints']) // 2 if detail else 0}  healthy {msg['healthy']}  "
                      f"p {[round(x, 2) for x in msg['pose'][:3]]}", flush=True)
            window.clear()
            last_print = now
        if view and view.last is not None:
            key = view.draw()
            if key == ord("q"):
                break
            if key == ord("c"):
                view.trajectory.clear()


if __name__ == "__main__":
    sys.exit(main())
