#!/usr/bin/env python3
"""
imu_view.py - live 3D orientation visualizer for the BNO055 / SEN0253.

Replaces the fragile DFRobot "Euler angle visual tool.exe". Reads the ESP32's
serial stream directly, ignores anything that isn't a valid
    qw,qx,qy,qz,sys,gyr,acc,mag
line (so the ESP-ROM boot banner can never desync it), and survives USB-CDC
re-enumeration (the PermissionError/ClearCommError you hit on reset) by
reopening the port automatically.

The board is rotated directly from the fused unit quaternion, which avoids the
gimbal lock that Euler angles hit near +/-90 deg pitch. Pitch/roll/yaw are still
shown numerically, but those are derived from the quaternion purely for display:
the displayed degrees can go singular near +/-90 deg while the 3D board stays
correct, because the board never goes through Euler.

Renders a board (green top / red bottom) with X(red) Y(green) Z(white) axes
that rotates in real time, a numeric readout, and the four BNO055 calibration
levels (sys/gyr/acc/mag, 0-3). In NDOF the absolute orientation is only
trustworthy once these reach 3, so they are colored red until then.

Setup (in your venv):
    python -m pip install pyserial pygame numpy

Run:
    python imu_view.py COM4
    python imu_view.py COM4 --baud 115200

Controls: Esc/Q quit, C zero, R reset, 1-7 numeric sensor panel, F/F11
fullscreen, scroll/+- zoom, TAB show/hide the graph strip, A/S/D/G/H/J/K
toggle individual graph panels (home row in GRAPH_CONFIG panel order; F is
skipped because it is fullscreen). Graph appearance (channels, window length,
colors, layout, y-scales) is configured in GRAPH_CONFIG at the top of this
file.

Notes on conventions: the firmware streams the BNO055 fused quaternion in its
native W,X,Y,Z register order. quat_to_matrix() builds the rotation matrix from
it directly. If a particular axis feels inverted for how your board is mounted,
flip the corresponding component sign where the quaternion is unpacked (clearly
marked in quat_to_matrix()).
"""
import argparse
import math
import sys
import threading
import time

import serial

try:
    import pygame
except ImportError:
    print("pygame is not installed. In your venv run:  python -m pip install pygame")
    sys.exit(1)

# numpy backs the sample ring buffer and the graph math, and is the linear
# algebra the Phase 3 KF/EKF will run on (decided at Phase 1 start, per the
# handoff: filters make it worth introducing now).
try:
    import numpy as np
except ImportError:
    print("numpy is not installed. In your venv run:  python -m pip install numpy")
    sys.exit(1)

# Wire format (must match the firmware). Version-prefixed positional CSV:
#   v2,seq,t_us,qw,qx,qy,qz,ax,ay,az,gx,gy,gz,mx,my,mz,lx,ly,lz,grx,gry,grz,temp,cs,cg,ca,cm
# We split on commas rather than one giant regex: it is faster for 27 fields and
# the version tag + field count give us a cheap, unambiguous validity check, so
# the ESP-ROM boot banner and any stray line are still rejected.
WIRE_VERSION = "v2"
# Field names in order AFTER the version tag. Indices into the split line[1:].
FIELDS = [
    "seq", "t_us",
    "qw", "qx", "qy", "qz",
    "ax", "ay", "az",
    "gx", "gy", "gz",
    "mx", "my", "mz",
    "lx", "ly", "lz",
    "grx", "gry", "grz",
    "temp", "cs", "cg", "ca", "cm",
]
N_FIELDS = len(FIELDS) + 1   # +1 for the version tag

# Logical sensor groups, for the visualizer's show/hide toggles and for logging
# headers. Each maps to the field names it owns.
SENSOR_GROUPS = {
    "quat":    ["qw", "qx", "qy", "qz"],
    "accel":   ["ax", "ay", "az"],
    "gyro":    ["gx", "gy", "gz"],
    "mag":     ["mx", "my", "mz"],
    "linear":  ["lx", "ly", "lz"],
    "gravity": ["grx", "gry", "grz"],
    "temp":    ["temp"],
}

# Field name -> column index in a ring-buffer row (rows are FIELDS in order).
FIELD_IDX = {name: i for i, name in enumerate(FIELDS)}
T_US_IDX = FIELD_IDX["t_us"]

# Per-channel trace colors. Axis convention matches the 3D view: x red,
# y green, z blue-ish; qw white; temp amber. Edit here to retheme.
_AXIS_COLORS = {"x": (235, 80, 80), "y": (80, 220, 100), "z": (90, 140, 255)}
CHANNEL_COLORS = {"qw": (235, 235, 235), "temp": (235, 190, 80)}
for _names in SENSOR_GROUPS.values():
    for _n in _names:
        if _n not in CHANNEL_COLORS:
            CHANNEL_COLORS[_n] = _AXIS_COLORS.get(_n[-1], (200, 200, 200))

# ----------------------------------------------------------------------------
# Graph configuration (Phase 1) - EDIT HERE to change how graphs display.
# Panels are drawn left-to-right along the bottom strip in this order when
# enabled. Runtime toggle keys follow the SAME order: A S D G H J K (home row,
# skipping F = fullscreen); TAB shows/hides the whole strip. "y" is "auto"
# (autoscale to the visible window) or a fixed (lo, hi) tuple.
# ----------------------------------------------------------------------------
GRAPH_CONFIG = {
    "window_s": 10.0,       # seconds of history shown (ring capacity follows)
    "strip_frac": 0.38,     # fraction of window height the strip occupies
    "panel_gap": 8,         # px between/around panels
    "autoscale_pad": 0.10,  # headroom fraction added around autoscaled data
    "max_pts_per_px": 1,    # DISPLAY-ONLY decimation cap; never thins the data
    "graphs_on": True,      # strip visible at startup
    "panels": [
        {"group": "quat",    "y": (-1.1, 1.1), "on": False},
        {"group": "accel",   "y": "auto",      "on": True},
        {"group": "gyro",    "y": "auto",      "on": True},
        {"group": "mag",     "y": "auto",      "on": False},
        {"group": "linear",  "y": "auto",      "on": False},
        {"group": "gravity", "y": "auto",      "on": False},
        {"group": "temp",    "y": "auto",      "on": False},
    ],
}


# ----------------------------------------------------------------------------
# Ring buffer of raw samples: the backbone of the data pipeline (handoff
# "Architecture" section). The SerialReader fills it; consumers take windowed
# copies: graphs read a window, the 3D view reads the latest, and the coming
# logging (Phase 2) and filters (Phase 3) consume from the same structure.
# Only RAW samples live here; filter outputs are derived consumer-side.
# ----------------------------------------------------------------------------
class RingBuffer:
    """Thread-safe circular store of samples in a preallocated numpy array.

    Rows are the FIELDS values in order (float64; t_us up to ~9e15 us stays
    exact in a double, i.e. centuries of device uptime). `count` is the total
    number of samples ever appended (monotonic), so a consumer can tell how
    much is new since it last looked - the Phase 2 logger will drain by
    tracking count, without removing anything graphs or filters still need.
    """

    def __init__(self, capacity):
        self.capacity = capacity
        self._data = np.zeros((capacity, len(FIELDS)), dtype=np.float64)
        self._lock = threading.Lock()
        self._next = 0          # next write slot
        self.count = 0          # total samples ever appended

    def append(self, row):
        with self._lock:
            self._data[self._next] = row
            self._next = (self._next + 1) % self.capacity
            self.count += 1

    def window(self, window_us=None):
        """Copy of the most recent samples, oldest->newest, optionally
        trimmed to the last window_us of device time.
        Returns (t_us 1-D array, data 2-D array)."""
        with self._lock:
            n = min(self.count, self.capacity)
            if n == 0:
                return np.empty(0), np.empty((0, len(FIELDS)))
            if self.count <= self.capacity:
                out = self._data[:n].copy()
            else:
                out = np.vstack((self._data[self._next:],
                                 self._data[:self._next]))
        t = out[:, T_US_IDX]
        if window_us is not None and t.size:
            out = out[t >= t[-1] - window_us]
            t = out[:, T_US_IDX]
        return t, out

# ----------------------------------------------------------------------------
# Serial reader thread: keeps the latest (pitch, roll, yaw) in a shared slot.
# Robust to the port disappearing on reset; ignores all non-matching lines.
# ----------------------------------------------------------------------------
class SerialReader(threading.Thread):
    def __init__(self, port, baud, ring=None):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.ring = ring                   # RingBuffer of raw samples (shared)
        self.lock = threading.Lock()
        # Latest full sample as a dict keyed by FIELDS names (floats), plus the
        # quaternion broken out for convenience. Identity until first line.
        self.quat = (1.0, 0.0, 0.0, 0.0)   # w, x, y, z
        self.calib = (0, 0, 0, 0)          # sys, gyr, acc, mag
        self.sample = {f: 0.0 for f in FIELDS}
        self.connected = False
        self.last_line_time = 0.0
        self.line_count = 0             # total valid lines, for data-rate calc
        # Stream-quality stats computed from seq + device timestamp.
        self.dropped = 0                # missed samples (seq gaps)
        self.dev_hz = 0.0               # rate from device timestamps
        self.jitter_ms = 0.0            # stddev of inter-sample dt (device clock)
        self.version_ok = True          # False if firmware tag != WIRE_VERSION
        self._prev_seq = None
        self._prev_t_us = None
        self._dt_window = []            # recent inter-sample dt, for jitter
        self._stop = False

    def _open(self):
        s = serial.Serial()
        s.port = self.port
        s.baudrate = self.baud
        s.timeout = 0.2
        # Do not assert DTR/RTS on open so we don't reset the board ourselves.
        s.dtr = False
        s.rts = False
        s.open()
        try:
            s.setDTR(False)
            s.setRTS(False)
        except Exception:
            pass
        return s

    def run(self):
        buf = bytearray()
        ser = None
        while not self._stop:
            if ser is None:
                try:
                    ser = self._open()
                    buf.clear()
                    self.connected = True
                except serial.SerialException:
                    self.connected = False
                    time.sleep(0.5)
                    continue
            try:
                chunk = ser.read(256)
            except (serial.SerialException, OSError):
                # Port dropped (reset / re-enumeration). Reopen.
                self.connected = False
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                time.sleep(0.3)
                continue
            if not chunk:
                continue
            buf.extend(chunk)
            while b"\n" in buf:
                idx = buf.index(b"\n")
                raw = bytes(buf[:idx]).rstrip(b"\r")
                del buf[:idx + 1]
                try:
                    text = raw.decode("ascii")
                except UnicodeDecodeError:
                    continue
                self._handle_line(text)
        if ser:
            try:
                ser.close()
            except Exception:
                pass

    def _handle_line(self, text):
        # Reject anything that is not a well-formed v2 line. The boot banner and
        # partial lines fail the checks below and are ignored.
        parts = text.split(",")
        if not parts:
            return
        tag = parts[0]
        # Version check first, so a recognizable-but-wrong version still warns
        # even if its field count differs from ours.
        if tag != WIRE_VERSION:
            if tag.startswith("v") and len(tag) <= 4 and tag[1:].isdigit():
                with self.lock:
                    self.version_ok = False
            return
        if len(parts) != N_FIELDS:
            return
        try:
            vals = {name: float(parts[i + 1]) for i, name in enumerate(FIELDS)}
            seq = int(vals["seq"])
            t_us = int(vals["t_us"])
        except (ValueError, IndexError):
            return

        quat = (vals["qw"], vals["qx"], vals["qy"], vals["qz"])
        calib = (int(vals["cs"]), int(vals["cg"]), int(vals["ca"]), int(vals["cm"]))

        # Raw sample into the ring (its own lock). Row order == FIELDS order.
        if self.ring is not None:
            self.ring.append([vals[f] for f in FIELDS])

        with self.lock:
            self.version_ok = True
            self.sample = vals
            self.quat = quat
            self.calib = calib
            self.last_line_time = time.time()
            self.line_count += 1

            # Dropped-frame detection via wrap-safe seq delta (uint32).
            if self._prev_seq is not None:
                delta = (seq - self._prev_seq) & 0xFFFFFFFF
                if delta > 1:
                    self.dropped += delta - 1
            self._prev_seq = seq

            # Device-clock rate + jitter from the microsecond timestamp.
            if self._prev_t_us is not None:
                dt_us = t_us - self._prev_t_us
                if 0 < dt_us < 1_000_000:    # ignore wraps / garbage
                    self._dt_window.append(dt_us)
                    if len(self._dt_window) > 200:
                        self._dt_window.pop(0)
                    mean = sum(self._dt_window) / len(self._dt_window)
                    if mean > 0:
                        self.dev_hz = 1_000_000.0 / mean
                    if len(self._dt_window) > 1:
                        var = sum((d - mean) ** 2 for d in self._dt_window) / len(self._dt_window)
                        self.jitter_ms = (var ** 0.5) / 1000.0
            self._prev_t_us = t_us



    def get(self):
        with self.lock:
            return {
                "quat": self.quat,
                "calib": self.calib,
                "sample": dict(self.sample),
                "connected": self.connected,
                "last_t": self.last_line_time,
                "line_count": self.line_count,
                "dropped": self.dropped,
                "dev_hz": self.dev_hz,
                "jitter_ms": self.jitter_ms,
                "version_ok": self.version_ok,
            }

    def stop(self):
        self._stop = True


# ----------------------------------------------------------------------------
# Minimal 3D + quaternion math (no numpy needed).
# ----------------------------------------------------------------------------
def quat_normalize(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / n, x / n, y / n, z / n)


def quat_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_to_matrix(q):
    # Build a rotation matrix directly from a (normalized) unit quaternion.
    # This is the gimbal-lock-free path: the board is rotated from q and never
    # passes through Euler angles. To invert an axis for a different physical
    # mounting, negate that component here (e.g. x = -x) rather than touching
    # the firmware.
    w, x, y, z = quat_normalize(q)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return [
        [1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy)],
        [2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy)],
    ]


def quat_to_euler(q):
    # Derive pitch/roll/yaw (degrees) from the quaternion FOR DISPLAY ONLY.
    # These numbers go singular near +/-90 deg pitch (that is gimbal lock); the
    # 3D board does not, because it rotates from the quaternion above. Returned
    # as (pitch, roll, yaw) to match the old readout layout.
    w, x, y, z = quat_normalize(q)
    # roll (about X)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
    # pitch (about Y) with clamp at the singularity
    sinp = 2 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))
    # yaw (about Z)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return pitch, roll, yaw


def apply(M, v):
    return [sum(M[i][k] * v[k] for k in range(3)) for i in range(3)]


def project(v, scale, cx, cy):
    # Simple isometric-ish projection: rotate the world a bit so we see depth,
    # then orthographic. View transform: tilt down and rotate.
    # Pre-tilt so the board is seen from slightly above/front.
    tx = math.radians(20)
    cx_, sx_ = math.cos(tx), math.sin(tx)
    x, y, z = v
    # rotate about X (view tilt)
    y2 = y * cx_ - z * sx_
    z2 = y * sx_ + z * cx_
    # orthographic; z2 affects nothing but we could fake scale with it
    depth = 1.0 + z2 * 0.0015
    sx = cx + x * scale * depth
    sy = cy - y2 * scale * depth
    return (sx, sy), z2


# ----------------------------------------------------------------------------
# Graphs (Phase 1). Built as infrastructure, not a feature: a "series" is just
# a named, time-stamped float stream, drawn as (name, color, t_us_array,
# value_array). GraphPanel.draw() is source-agnostic, so the Phase 3 KF/EKF
# outputs plot through this exact same path by handing the panel their own
# (t, value) streams; series_from_ring() is merely the raw-channel source.
# ----------------------------------------------------------------------------
class GraphPanel:
    BG = (14, 14, 20)
    BORDER = (60, 60, 72)
    GRID = (38, 38, 48)
    LABEL = (130, 130, 140)

    def __init__(self, spec):
        self.group = spec["group"]
        self.on = spec.get("on", True)
        self.yspec = spec.get("y", "auto")
        self.fields = SENSOR_GROUPS[self.group]

    def series_from_ring(self, t, data):
        """Raw-channel series for this panel's group, sliced out of the ring
        window arrays (zero per-channel copies until draw)."""
        if data.shape[0] == 0:
            return []
        return [(f, CHANNEL_COLORS.get(f, (200, 200, 200)),
                 t, data[:, FIELD_IDX[f]]) for f in self.fields]

    def draw(self, surf, rect, series, font):
        pygame.draw.rect(surf, self.BG, rect)
        pygame.draw.rect(surf, self.BORDER, rect, 1)

        window_us = GRAPH_CONFIG["window_s"] * 1e6

        # y range: fixed (lo, hi) from config, or autoscaled to the window.
        if self.yspec == "auto":
            vmin, vmax = math.inf, -math.inf
            for _, _, _, vs in series:
                if vs.size:
                    vmin = min(vmin, float(vs.min()))
                    vmax = max(vmax, float(vs.max()))
            if not math.isfinite(vmin):
                vmin, vmax = -1.0, 1.0
            pad = (vmax - vmin) * GRAPH_CONFIG["autoscale_pad"]
            if pad <= 0.0:
                pad = 0.5            # flat line: give it headroom
            vmin -= pad
            vmax += pad
        else:
            vmin, vmax = self.yspec
        vspan = vmax - vmin

        # Newest sample defines the right edge; traces scroll leftward. Always
        # the freshest data, no interpolation (render model: see handoff).
        t_right = max((ts[-1] for _, _, ts, _ in series if ts.size),
                      default=None)
        if t_right is not None:
            px_per_us = rect.w / window_us
            max_pts = rect.w * GRAPH_CONFIG["max_pts_per_px"]

            # Zero line, when zero is in range.
            if vmin < 0.0 < vmax:
                zy = rect.bottom - (0.0 - vmin) / vspan * rect.h
                pygame.draw.line(surf, self.GRID,
                                 (rect.left + 1, zy), (rect.right - 1, zy))

            for name, color, ts, vs in series:
                if ts.size < 2:
                    continue
                # DISPLAY-ONLY decimation (config max_pts_per_px). The ring
                # keeps every sample; logging and filters never see this.
                step = max(1, ts.size // max_pts)
                if step > 1:
                    ts = np.concatenate((ts[::step], ts[-1:]))
                    vs = np.concatenate((vs[::step], vs[-1:]))
                xs = rect.right - (t_right - ts) * px_per_us
                ys = rect.bottom - (vs - vmin) / vspan * rect.h
                np.clip(ys, rect.top + 1, rect.bottom - 1, out=ys)
                keep = xs >= rect.left + 1
                pts = np.column_stack((xs[keep], ys[keep]))
                if pts.shape[0] >= 2:
                    pygame.draw.lines(surf, color, False, pts.tolist(), 1)

        # Labels: group name, colored channel legend, y-range readout.
        x = rect.left + 6
        lbl = font.render(self.group, True, (220, 220, 225))
        surf.blit(lbl, (x, rect.top + 4))
        x += lbl.get_width() + 12
        for name, color, _, _ in series:
            s = font.render(name, True, color)
            surf.blit(s, (x, rect.top + 4))
            x += s.get_width() + 8
        ymax_s = font.render(f"{vmax:.3g}", True, self.LABEL)
        surf.blit(ymax_s, (rect.right - ymax_s.get_width() - 4, rect.top + 4))
        ymin_s = font.render(f"{vmin:.3g}", True, self.LABEL)
        surf.blit(ymin_s, (rect.right - ymin_s.get_width() - 4,
                           rect.bottom - ymin_s.get_height() - 4))


def draw_graph_strip(surf, panels, t, data, W, H, strip_h, font):
    """Lay the enabled panels side by side along the bottom strip."""
    gap = GRAPH_CONFIG["panel_gap"]
    n = len(panels)
    pw = (W - gap * (n + 1)) // n
    top = H - strip_h
    for i, p in enumerate(panels):
        rect = pygame.Rect(gap + i * (pw + gap), top + gap,
                           pw, strip_h - 2 * gap)
        p.draw(surf, rect, p.series_from_ring(t, data), font)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", help="serial port, e.g. COM4")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    # Ring buffer of raw samples (the pipeline backbone). Capacity covers
    # ~1.5x the graph window at the nominal 100 Hz, so the Phase 2 logger has
    # slack to drain incrementally before anything is overwritten.
    ring = RingBuffer(max(256, int(GRAPH_CONFIG["window_s"] * 100 * 1.5)))
    reader = SerialReader(args.port, args.baud, ring)
    reader.start()

    pygame.init()
    W, H = 900, 640
    # RESIZABLE enables the maximize button and window dragging-to-resize (the
    # grayed-out maximize was because the default window has no resize flag).
    # vsync=1 syncs presentation to the monitor's refresh (e.g. 144 Hz) with no
    # tearing and no artificial cap; we always draw the freshest sample, so the
    # board reflects new data the instant it arrives with no added latency.
    flags = pygame.RESIZABLE
    screen = pygame.display.set_mode((W, H), flags, vsync=1)
    pygame.display.set_caption(f"IMU view - {args.port}")
    fullscreen = False
    windowed_size = (W, H)        # remembered so we can restore from fullscreen
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 22)
    small = pygame.font.SysFont("consolas", 16)

    BG = (8, 8, 12)
    GREEN = (40, 220, 60)
    RED = (220, 40, 40)
    WHITE = (230, 230, 230)
    GREY = (90, 90, 100)

    # Center is recomputed from the live surface each frame, so it stays correct
    # after a resize or fullscreen toggle.
    scale = 2.2
    SCALE_MIN, SCALE_MAX = 0.4, 12.0   # zoom limits

    # Rate tracking: sample FPS and data Hz over a rolling ~0.5s window.
    fps = 0.0
    frames_since = 0
    rate_t0 = time.time()

    # Board geometry: a flat slab. Top face green, bottom face red.
    hw, hl, ht = 90, 60, 10  # half width(x), half length(y), half thickness(z)
    corners = [
        [-hw, -hl, -ht], [hw, -hl, -ht], [hw, hl, -ht], [-hw, hl, -ht],  # bottom
        [-hw, -hl, ht], [hw, -hl, ht], [hw, hl, ht], [-hw, hl, ht],      # top
    ]
    bottom_face = [0, 1, 2, 3]
    top_face = [4, 5, 6, 7]
    side_faces = [[0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]

    # Body axes extend a bit beyond the board so they're visible.
    axis_len = 140
    # World axes drawn fixed in space. Slightly longer so they read as the frame.
    world_axis_len = 170

    # Calibration: quaternion offset applied as q_offset * q_current. Pressing
    # C sets q_offset = conjugate(current), which zeroes the orientation so the
    # device frame coincides with the world frame at that instant. Gimbal-lock
    # free, unlike the old matrix-from-Euler approach.
    q_offset = (1.0, 0.0, 0.0, 0.0)   # identity
    quat = (1.0, 0.0, 0.0, 0.0)       # latest raw quaternion (pre-init)
    calib = (0, 0, 0, 0)              # latest cal levels (pre-init)
    q_cal = (1.0, 0.0, 0.0, 0.0)

    def set_mode(size, fs):
        # Recreate the display surface for a resize or fullscreen toggle, keeping
        # vsync on so presentation stays tear-free at the monitor refresh rate.
        if fs:
            return pygame.display.set_mode((0, 0), pygame.FULLSCREEN, vsync=1)
        return pygame.display.set_mode(size, pygame.RESIZABLE, vsync=1)

    # Per-sensor display toggles (visualizer-side selection). The firmware still
    # streams everything; this only controls what the readout shows. Number keys
    # 1-7 flip each group. Later this same selection can drive a host->device
    # command that changes what the firmware emits.
    group_order = ["quat", "accel", "gyro", "mag", "linear", "gravity", "temp"]
    show = {g: True for g in group_order}
    group_keys = {
        pygame.K_1: "quat", pygame.K_2: "accel", pygame.K_3: "gyro",
        pygame.K_4: "mag", pygame.K_5: "linear", pygame.K_6: "gravity",
        pygame.K_7: "temp",
    }

    # Graph panels (Phase 1), in GRAPH_CONFIG["panels"] order. Toggle keys
    # follow that order on the home row, skipping F (fullscreen): A S D G H J
    # K. TAB shows/hides the whole strip. Same toggle model as 1-7.
    panels = [GraphPanel(spec) for spec in GRAPH_CONFIG["panels"]]
    graphs_on = GRAPH_CONFIG["graphs_on"]
    graph_key_order = [pygame.K_a, pygame.K_s, pygame.K_d, pygame.K_g,
                       pygame.K_h, pygame.K_j, pygame.K_k]
    graph_keys = {k: i for i, k in enumerate(graph_key_order[:len(panels)])}
    # Strip render cache: the strip is redrawn only when a NEW sample arrives
    # (ring.count in the key) or the layout/toggles change, then blitted every
    # frame. At 144 fps with 100 Hz data this skips ~30% of strip redraws with
    # zero staleness: any new sample changes the key and forces a fresh draw.
    strip_cache_key = None
    strip_surf = None

    running = True
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN and e.key in (pygame.K_ESCAPE, pygame.K_q):
                running = False
            elif e.type == pygame.KEYDOWN and e.key in (pygame.K_EQUALS, pygame.K_PLUS, pygame.K_KP_PLUS):
                scale = min(SCALE_MAX, scale * 1.15)
            elif e.type == pygame.KEYDOWN and e.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                scale = max(SCALE_MIN, scale / 1.15)
            elif e.type == pygame.MOUSEWHEEL:
                # e.y > 0 scroll up = zoom in
                factor = 1.15 ** e.y
                scale = max(SCALE_MIN, min(SCALE_MAX, scale * factor))
            elif e.type == pygame.KEYDOWN and e.key == pygame.K_c:
                # Calibrate: make the current device orientation the new zero.
                q_offset = quat_conjugate(quat)
            elif e.type == pygame.KEYDOWN and e.key == pygame.K_r:
                # Reset calibration back to raw orientation.
                q_offset = (1.0, 0.0, 0.0, 0.0)
            elif e.type == pygame.KEYDOWN and e.key in group_keys:
                g = group_keys[e.key]
                show[g] = not show[g]
            elif e.type == pygame.KEYDOWN and e.key == pygame.K_TAB:
                graphs_on = not graphs_on
            elif e.type == pygame.KEYDOWN and e.key in graph_keys:
                p = panels[graph_keys[e.key]]
                p.on = not p.on
            elif e.type == pygame.KEYDOWN and e.key in (pygame.K_f, pygame.K_F11):
                # Toggle fullscreen. Remember the windowed size to restore to.
                fullscreen = not fullscreen
                if fullscreen:
                    windowed_size = screen.get_size()
                    screen = set_mode(windowed_size, True)
                else:
                    screen = set_mode(windowed_size, False)
            elif e.type == pygame.VIDEORESIZE and not fullscreen:
                # User dragged the window edge or hit maximize.
                screen = set_mode((max(480, e.w), max(360, e.h)), False)

        # Live surface dimensions: recomputed every frame so resize/fullscreen
        # take effect immediately and the board stays centered. When the graph
        # strip is visible, the 3D view and the UI anchored to its bottom edge
        # compress into the area above it (base_h).
        W, H = screen.get_size()
        active_panels = [p for p in panels if p.on] if graphs_on else []
        strip_h = int(H * GRAPH_CONFIG["strip_frac"]) if active_panels else 0
        base_h = H - strip_h
        cx, cy = W // 2, base_h // 2 - 20

        st = reader.get()
        quat = st["quat"]
        calib = st["calib"]
        sample = st["sample"]
        connected = st["connected"]
        last_t = st["last_t"]
        line_count = st["line_count"]
        live = connected and (time.time() - last_t) < 1.0

        # Update render-FPS counter on a rolling window. (Data rate now comes
        # from the device timestamp in the reader, not from arrival timing.)
        frames_since += 1
        now = time.time()
        dt = now - rate_t0
        if dt >= 0.5:
            fps = frames_since / dt
            frames_since = 0
            rate_t0 = now

        screen.fill(BG)

        # Fixed WORLD frame (does not move): X/Y/Z. Drawn dim and behind the
        # cube so the bright body frame reads as attached to the device.
        W_RED, W_GREEN, W_WHITE = (150, 60, 60), (60, 150, 70), (170, 170, 175)
        world_origin2d, _ = project([0, 0, 0], scale, cx, cy)
        world_axes = [((world_axis_len, 0, 0), W_RED, "X"),
                      ((0, world_axis_len, 0), W_GREEN, "Y"),
                      ((0, 0, world_axis_len), W_WHITE, "Z")]
        for vec, col, name in world_axes:
            tip2d, _ = project(list(vec), scale, cx, cy)
            pygame.draw.line(screen, col, world_origin2d, tip2d, 2)
            screen.blit(small.render(name, True, col), (tip2d[0] + 4, tip2d[1] - 8))

        # Rotate board corners. Apply calibration: q = q_offset * q_current,
        # then build the matrix from the quaternion (gimbal-lock free).
        q_cal = quat_mul(q_offset, quat)
        M = quat_to_matrix(q_cal)
        pts3d = [apply(M, c) for c in corners]
        proj = [project(p, scale, cx, cy) for p in pts3d]
        pts2d = [pp[0] for pp in proj]
        depth = [pp[1] for pp in proj]

        # Painter's algorithm: draw faces back-to-front by average depth.
        faces = []
        faces.append((bottom_face, RED))
        faces.append((top_face, GREEN))
        for sf in side_faces:
            faces.append((sf, GREY))
        faces.sort(key=lambda f: sum(depth[i] for i in f[0]) / len(f[0]))

        for idxs, col in faces:
            poly = [pts2d[i] for i in idxs]
            pygame.draw.polygon(screen, col, poly)
            pygame.draw.polygon(screen, (20, 20, 24), poly, 2)

        # Body-fixed axes: rotate WITH the board so they stick to the cube.
        # X' red, Y' green, Z' white (primed = device frame), from board center.
        axis_body = [((axis_len, 0, 0), RED, "X'"),
                     ((0, axis_len, 0), GREEN, "Y'"),
                     ((0, 0, axis_len), WHITE, "Z'")]
        center2d, _ = project(apply(M, [0, 0, 0]), scale, cx, cy)
        for vec, col, name in axis_body:
            tip3d = apply(M, list(vec))
            tip2d, _ = project(tip3d, scale, cx, cy)
            pygame.draw.line(screen, col, center2d, tip2d, 3)
            label = small.render(name, True, col)
            screen.blit(label, (tip2d[0] + 4, tip2d[1] - 8))

        # Readout (bottom-left, like the tool). Euler is derived from the
        # calibrated quaternion purely for display; the board itself rotates
        # from the quaternion, so these numbers can go singular near +/-90 deg
        # pitch while the 3D view stays correct.
        def txt(s, color, x, y, fnt=font):
            screen.blit(fnt.render(s, True, color), (x, y))

        qw, qx, qy, qz = quat
        pitch, roll, yaw = quat_to_euler(q_cal)

        txt(f"Pitch: {pitch:8.3f}", WHITE, 16, base_h - 150)
        txt(f"Roll:  {roll:8.3f}", RED, 16, base_h - 122)
        txt(f"Yaw:   {yaw:8.3f}", GREEN, 16, base_h - 94)

        # Quaternion norm sanity: a healthy unit quaternion is |q| ~ 1.0. A
        # drift flags a parse/scale problem or all-zero data (sensor not yet in
        # fusion). Warn in amber when it strays.
        qnorm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
        norm_ok = abs(qnorm - 1.0) < 0.05
        ncol = GREY if norm_ok else (220, 180, 60)
        txt(f"|q|={qnorm:5.3f}", ncol, 16, base_h - 36, small)

        # Calibration levels (sys/gyr/acc/mag): red until 3, green at 3. In NDOF
        # the absolute orientation is only trustworthy once all reach 3.
        cs, cg, ca, cm = calib
        cal_x = 150
        txt("cal", GREY, cal_x, base_h - 36, small)
        for i, (lbl, lvl) in enumerate((("S", cs), ("G", cg), ("A", ca), ("M", cm))):
            ccol = GREEN if lvl == 3 else RED
            txt(f"{lbl}{lvl}", ccol, cal_x + 36 + i * 38, base_h - 36, small)

        # Calibration coaching: show the action for each subsystem still below 3,
        # so the hint tells you what to do right now and vanishes once you're
        # fully calibrated. Gyro: hold still. Accel: tilt through a few poses.
        # Mag: wave a figure-8. (Sys is the fused result of the other three.)
        cal_hints = []
        if cg < 3:
            cal_hints.append("hold still (gyro)")
        if ca < 3:
            cal_hints.append("tilt through poses (accel)")
        if cm < 3:
            cal_hints.append("wave figure-8 (mag)")
        if cal_hints:
            hint = "Calibrate: " + "   ".join(cal_hints)
            screen.blit(small.render(hint, True, (220, 180, 60)), (cal_x, base_h - 14))
        elif cs == 3:
            screen.blit(small.render("Fully calibrated", True, GREEN), (cal_x, base_h - 14))

        status = "LIVE" if live else ("connected, no data" if connected else f"waiting for {args.port}...")
        scol = (60, 220, 90) if live else (220, 180, 60)
        screen.blit(small.render(status, True, scol), (16, 12))

        # Stream-quality stats (top-right). dev_hz/jitter come from the device
        # microsecond timestamp (authoritative); dropped from seq gaps.
        rate_str = f"{fps:4.0f} FPS render   {st['dev_hz']:5.1f} Hz dev"
        screen.blit(small.render(rate_str, True, GREY), (W - 260, 12))
        q_str = f"jitter {st['jitter_ms']:4.2f} ms    dropped {st['dropped']}"
        screen.blit(small.render(q_str, True, GREY), (W - 260, 32))
        if not st["version_ok"]:
            warn = small.render("firmware/visualizer version mismatch", True, (230, 80, 80))
            screen.blit(warn, (W - 320, 52))

        # Per-sensor numeric panel (top-left, under status). Each group is shown
        # only if toggled on; number keys 1-7 flip them. Greyed label when off.
        panel_y = 38
        units = {"accel": "m/s2", "gyro": "dps", "mag": "uT",
                 "linear": "m/s2", "gravity": "m/s2", "temp": "C"}
        for ki, g in enumerate(group_order):
            keyn = ki + 1
            on = show[g]
            lblcol = WHITE if on else (90, 90, 100)
            if g == "quat":
                txt(f"{keyn} quat", lblcol, 16, panel_y, small)
                if on:
                    txt(f"{qw:7.4f} {qx:7.4f} {qy:7.4f} {qz:7.4f}",
                        WHITE, 90, panel_y, small)
            elif g == "temp":
                txt(f"{keyn} temp", lblcol, 16, panel_y, small)
                if on:
                    txt(f"{sample.get('temp', 0):.0f} C", WHITE, 90, panel_y, small)
            else:
                txt(f"{keyn} {g}", lblcol, 16, panel_y, small)
                if on:
                    names = SENSOR_GROUPS[g]
                    vals = " ".join(f"{sample.get(n, 0):8.3f}" for n in names)
                    txt(f"{vals}  {units.get(g, '')}", WHITE, 90, panel_y, small)
            panel_y += 22

        screen.blit(small.render(
            "TAB: graphs  ASDGHJK: panels  C: zero  R: reset  1-7: sensors  F: fullscr  +/-: zoom  Esc/Q: quit",
            True, GREY), (W - 700, base_h - 20))

        # Graph strip (Phase 1): enabled panels side by side along the bottom.
        # Rendered into a cached surface keyed on (new data, size, toggles);
        # see strip_cache_key comment above.
        if active_panels:
            key = (ring.count, W, strip_h, tuple(p.on for p in panels))
            if key != strip_cache_key:
                if strip_surf is None or strip_surf.get_size() != (W, strip_h):
                    strip_surf = pygame.Surface((W, strip_h))
                strip_surf.fill(BG)
                t_arr, data_arr = ring.window(int(GRAPH_CONFIG["window_s"] * 1e6))
                draw_graph_strip(strip_surf, active_panels, t_arr, data_arr,
                                 W, strip_h, strip_h, small)
                strip_cache_key = key
            screen.blit(strip_surf, (0, H - strip_h))

        pygame.display.flip()
        clock.tick()   # uncapped; vsync paces presentation to the monitor

    reader.stop()
    pygame.quit()


if __name__ == "__main__":
    main()