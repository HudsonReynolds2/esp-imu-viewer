#!/usr/bin/env python3
"""
imu_view.py - live 3D orientation visualizer for the BNO055 / SEN0253.

Replaces the fragile DFRobot "Euler angle visual tool.exe". Reads the ESP32's
serial stream directly, ignores anything that isn't a valid
    pitch:<f> roll:<f> yaw:<f>
line (so the ESP-ROM boot banner can never desync it), and survives USB-CDC
re-enumeration (the PermissionError/ClearCommError you hit on reset) by
reopening the port automatically.

Renders a board (green top / red bottom) with X(red) Y(green) Z(white) axes
that rotates in real time, plus a numeric readout. No virtual COM ports, no
reflashing.

Setup (in your venv):
    python -m pip install pyserial pygame

Run:
    python imu_view.py COM4
    python imu_view.py COM4 --baud 115200

Controls: close the window or press Esc/Q to quit.

Notes on conventions: the firmware maps the BNO055 Euler registers as
    yaw   = EUL_X (heading)
    roll  = EUL_Y
    pitch = EUL_Z
matching the DFRobot demo. If a particular axis feels inverted for how your
board is mounted, flip its sign in euler_to_matrix() (clearly marked).
"""
import argparse
import math
import re
import sys
import threading
import time

import serial

try:
    import pygame
except ImportError:
    print("pygame is not installed. In your venv run:  python -m pip install pygame")
    sys.exit(1)

GOOD = re.compile(r'^pitch:(-?\d+\.\d+) roll:(-?\d+\.\d+) yaw:(-?\d+\.\d+)$')

# ----------------------------------------------------------------------------
# Serial reader thread: keeps the latest (pitch, roll, yaw) in a shared slot.
# Robust to the port disappearing on reset; ignores all non-matching lines.
# ----------------------------------------------------------------------------
class SerialReader(threading.Thread):
    def __init__(self, port, baud):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.lock = threading.Lock()
        self.angles = (0.0, 0.0, 0.0)   # pitch, roll, yaw
        self.connected = False
        self.last_line_time = 0.0
        self.line_count = 0             # total valid lines, for data-rate calc
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
                m = GOOD.match(text)
                if m:
                    p, r, y = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
                    with self.lock:
                        self.angles = (p, r, y)
                        self.last_line_time = time.time()
                        self.line_count += 1
        if ser:
            try:
                ser.close()
            except Exception:
                pass

    def get(self):
        with self.lock:
            return self.angles, self.connected, self.last_line_time, self.line_count

    def stop(self):
        self._stop = True


# ----------------------------------------------------------------------------
# Minimal 3D math (no numpy needed).
# ----------------------------------------------------------------------------
def euler_to_matrix(pitch_deg, roll_deg, yaw_deg):
    # Convert to radians. Flip any sign here if an axis feels inverted.
    p = math.radians(pitch_deg)
    r = math.radians(roll_deg)
    y = math.radians(yaw_deg)

    cp, sp = math.cos(p), math.sin(p)
    cr, sr = math.cos(r), math.sin(r)
    cy, sy = math.cos(y), math.sin(y)

    # Rotation matrices.
    # Yaw about Z, pitch about Y, roll about X (Z-Y-X). Tweak order/signs if
    # your mounting differs.
    Rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]
    Ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]
    Rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]]

    def mul(A, B):
        return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

    return mul(Rz, mul(Ry, Rx))


def apply(M, v):
    return [sum(M[i][k] * v[k] for k in range(3)) for i in range(3)]


def matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def transpose(M):
    # For a rotation matrix, the transpose is its inverse.
    return [[M[j][i] for j in range(3)] for i in range(3)]


IDENTITY = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", help="serial port, e.g. COM4")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    reader = SerialReader(args.port, args.baud)
    reader.start()

    pygame.init()
    W, H = 900, 640
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption(f"IMU view - {args.port}")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 22)
    small = pygame.font.SysFont("consolas", 16)

    BG = (8, 8, 12)
    GREEN = (40, 220, 60)
    RED = (220, 40, 40)
    WHITE = (230, 230, 230)
    GREY = (90, 90, 100)

    cx, cy = W // 2, H // 2 - 20
    scale = 2.2
    SCALE_MIN, SCALE_MAX = 0.4, 12.0   # zoom limits

    # Rate tracking: sample FPS and data Hz over a rolling ~0.5s window.
    fps = 0.0
    data_hz = 0.0
    frames_since = 0
    rate_t0 = time.time()
    lines_at_t0 = 0

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

    # Calibration: offset matrix applied as offset * current. Pressing C sets
    # offset = inverse(current), which zeroes the displayed orientation so the
    # device frame coincides with the world frame at that instant.
    offset = [row[:] for row in IDENTITY]

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
                offset = transpose(euler_to_matrix(pitch, roll, yaw))
            elif e.type == pygame.KEYDOWN and e.key == pygame.K_r:
                # Reset calibration back to raw orientation.
                offset = [row[:] for row in IDENTITY]

        (pitch, roll, yaw), connected, last_t, line_count = reader.get()
        live = connected and (time.time() - last_t) < 1.0

        # Update FPS and data-rate counters on a rolling window.
        frames_since += 1
        now = time.time()
        dt = now - rate_t0
        if dt >= 0.5:
            fps = frames_since / dt
            data_hz = (line_count - lines_at_t0) / dt
            frames_since = 0
            lines_at_t0 = line_count
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

        # Rotate board corners. Apply calibration offset: M = offset * raw.
        M = matmul(offset, euler_to_matrix(pitch, roll, yaw))
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

        # Readout (bottom-left, like the tool).
        def txt(s, color, x, y):
            screen.blit(font.render(s, True, color), (x, y))

        txt(f"Pitch: {pitch:8.3f}", WHITE, 16, H - 96)
        txt(f"Roll:  {roll:8.3f}", RED, 16, H - 68)
        txt(f"Yaw:   {yaw:8.3f}", GREEN, 16, H - 40)

        status = "LIVE" if live else ("connected, no data" if connected else f"waiting for {args.port}...")
        scol = (60, 220, 90) if live else (220, 180, 60)
        screen.blit(small.render(status, True, scol), (16, 12))
        rate_str = f"{fps:4.0f} FPS   {data_hz:4.0f} Hz data"
        screen.blit(small.render(rate_str, True, GREY), (W - 230, 12))
        screen.blit(small.render("C: zero/calibrate   R: reset   scroll/+-: zoom   Esc/Q: quit", True, GREY), (W - 470, 32))

        pygame.display.flip()
        clock.tick(120)

    reader.stop()
    pygame.quit()


if __name__ == "__main__":
    main()
