# BNO055 Real-Time Orientation Visualizer

Reads pitch / roll / yaw from a Bosch **BNO055** (DFRobot SEN0253 "10 DOF IMU AHRS")
over I2C on a **Seeed XIAO ESP32-C3**, streams the fused Euler angles over USB, and
renders a live 3D view on your PC.

The firmware runs the BNO055 in NDOF fusion mode and prints lines of the form:

```
pitch:-12.500 roll:3.250 yaw:181.062
```

The included Python visualizer (`imu_view.py`) draws a rotating board with a fixed
world frame (X/Y/Z) and a device-fixed frame (X'/Y'/Z'), with zoom and a one-key
calibrate. It reads the serial stream directly and ignores anything that isn't a
valid data line, so the ESP32 boot banner can never desync it.

## Repository layout

```
.
├── bno055_imu_show/         ESP-IDF project (the firmware)
│   ├── CMakeLists.txt
│   ├── sdkconfig.defaults   build settings: LF line endings + logging off
│   └── main/
│       ├── CMakeLists.txt
│       └── bno055_imu_show_main.c
├── imu_view.py              live 3D visualizer (pygame)
├── sniff.py                 serial debug helper: prints raw bytes
├── .gitignore
└── README.md
```

## Hardware

- Seeed Studio XIAO ESP32-C3
- DFRobot SEN0253 (BNO055 + BMP280, 10 DOF) at I2C address `0x28`
- USB-C cable

### Wiring

The firmware defaults to the XIAO's labeled I2C pins:

| BNO055 board | XIAO ESP32-C3 |
|--------------|---------------|
| VCC (3V3)    | 3V3           |
| GND          | GND           |
| SDA          | D4 (GPIO6)    |
| SCL          | D5 (GPIO7)    |

The DFRobot board has its own I2C pull-ups, so none are needed externally. To use
different pins, edit `I2C_MASTER_SDA_IO` / `I2C_MASTER_SCL_IO` at the top of
`bno055_imu_show_main.c`.

## Firmware: build and flash

Requires ESP-IDF v5.5.x (developed on v5.5.3) on Windows, macOS, or Linux.

From an ESP-IDF terminal, inside `bno055_imu_show/`:

```
idf.py set-target esp32c3
idf.py build
idf.py -p COM4 flash
```

Replace `COM4` with your XIAO's port (`/dev/ttyACM0` or similar on Linux/macOS).

The settings in `sdkconfig.defaults` (LF line endings, logging disabled) are applied
on a clean build. If you have built before and they don't seem to take effect, delete
the generated config and rebuild:

```
del sdkconfig        # PowerShell / cmd  (use: rm sdkconfig  on macOS/Linux)
idf.py build
idf.py -p COM4 flash
```

To watch the raw output, `idf.py -p COM4 monitor` (exit with Ctrl+]). Close the
monitor before running the visualizer; only one program can hold the port.

## Visualizer: install and run

Python 3.9+ recommended. Use a virtual environment.

### Windows (PowerShell)

```
python -m venv .venv
# If activation is blocked:
Set-ExecutionPolicy -ExecutionPolicy Bypass -Scope Process
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install pyserial pygame
```

### macOS / Linux

```
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install pyserial pygame
```

### Run

```
python imu_view.py COM4
```

(Use your actual port. Optional: `--baud 115200`, which is already the default.)

A window opens showing the board. Top-left shows connection status; top-right shows
render FPS and the incoming data rate in Hz (expect ~100 Hz with the current firmware).

### Controls

| Key / input        | Action                                              |
|--------------------|-----------------------------------------------------|
| Mouse scroll, +, - | Zoom in / out                                       |
| C                  | Calibrate: set current device orientation as zero   |
| R                  | Reset calibration back to raw orientation           |
| Esc, Q             | Quit                                                |

Calibrate (C) snapshots the current orientation and makes it the new reference, so
the device frame (X'/Y'/Z') aligns with the world frame (X/Y/Z) and all motion after
that is measured relative to that pose.

If an axis moves the wrong way for how your board is mounted, flip that angle's sign
in `euler_to_matrix()` in `imu_view.py` (the radians conversions near the top of the
function).

## Troubleshooting

- **No data / "waiting for COM4":** confirm the port, that the firmware is flashed,
  and that no other program (monitor, another terminal) holds the port.
- **Want to see the exact bytes:** run `python sniff.py COM4`. Valid lines look like
  `b'pitch:... roll:... yaw:...\r\n'`. The one-time ESP-ROM banner at reset is normal.
- **Values look frozen at ~12 Hz:** old firmware is still flashed; rebuild and reflash
  (the loop now runs at ~100 Hz).

## License

The firmware is derived from the ESP-IDF `i2c_basic` example. BNO055 register usage
follows the Bosch BNO055 datasheet.
