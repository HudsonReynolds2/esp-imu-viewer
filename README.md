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

## Reference links

- Sensor wiki (DFRobot SEN0253): https://wiki.dfrobot.com/sen0253/docs/23068
- Sensor product page: https://www.dfrobot.com/product-1793.html
- XIAO ESP32-C3 getting started (Seeed): https://wiki.seeedstudio.com/XIAO_ESP32C3_Getting_Started/

## Repository layout

```
.
├── bno055_imu_show/         ESP-IDF project (the firmware)
│   ├── CMakeLists.txt
│   └── main/
│       ├── CMakeLists.txt
│       └── bno055_imu_show_main.c
├── imu_view.py              live 3D visualizer (pygame)
├── sniff.py                 serial debug helper: prints raw bytes
├── LICENSE
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
idf.py menuconfig      # set the options below (first build only)
idf.py build
idf.py -p COM4 flash
```

Replace `COM4` with your XIAO's port (`/dev/ttyACM0` or similar on Linux/macOS).

### Required sdkconfig options

These two settings make the serial stream clean enough for a strict parser
(both the included visualizer and the original DFRobot tool). Set them once via
`idf.py menuconfig`; they are saved in the generated `sdkconfig`.

1. **LF line endings on stdout.** The visualizer / DFRobot tool expect Arduino-style
   lines. By default ESP-IDF translates `\n` into `\r\n`, which can desync a strict
   line parser. Set output to LF only:

   - `Component config` → `Standard IO (libc)` → `Line ending for UART output` → **LF**
   - Kconfig symbol: `CONFIG_LIBC_STDOUT_LINE_ENDING_LF=y`
     (on IDF < 5.5 this symbol is `CONFIG_NEWLIB_STDOUT_LINE_ENDING_LF`)

2. **Disable IDF logging on the console.** So the only thing on the port is the
   `pitch:.. roll:.. yaw:..` data, with no `I (123) ...` log lines mixed in:

   - `Component config` → `Log` → `Default log verbosity` → **No output**
   - Kconfig symbols: `CONFIG_LOG_DEFAULT_LEVEL_NONE=y` and
     `CONFIG_LOG_MAXIMUM_LEVEL_NONE=y`

The console baud rate stays at the default 115200, which matches the visualizer.

> The ESP-ROM boot banner (e.g. `ESP-ROM:esp32c3-...`) is printed by hardware before
> your code runs and cannot be disabled in firmware. The included visualizer ignores
> it; if you use a different tool, reset the board and let the banner pass before
> connecting.

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
render FPS and the incoming data rate in Hz.

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

## Adapting or upgrading

A few common changes and where to make them:

- **Axis directions.** If an axis moves the wrong way for how your board is mounted,
  flip that angle's sign in `euler_to_matrix()` in `imu_view.py` (the radians
  conversions near the top of the function). To change which Euler register maps to
  pitch/roll/yaw, see the mapping in `bno_read_euler()` in the firmware.

- **Output rate.** The firmware loop delay sets the stream rate. `vTaskDelay(pdMS_TO_TICKS(10))`
  in `app_main()` gives ~100 Hz, which matches the BNO055's NDOF fusion rate. Increase
  the delay for a slower stream; going faster than ~100 Hz yields no new fusion data.

- **Line format.** The firmware prints `pitch:%.3f roll:%.3f yaw:%.3f`. If you change
  the format, update the `GOOD` regular expression in both `imu_view.py` and `sniff.py`
  so they still match.

- **Reading more from the sensor.** The BNO055 also exposes raw accelerometer,
  gyroscope, magnetometer, linear acceleration, gravity, quaternion, and calibration
  status registers. Add reads in the firmware alongside `bno_read_euler()` (register
  addresses are in the BNO055 datasheet) and extend the printed line plus the parser.

- **A different ESP32 or sensor.** The I2C setup uses the ESP-IDF `i2c_master` driver,
  which is portable across ESP32 variants; only the default SDA/SCL pins are
  board-specific. For a different IMU, replace the register definitions and the
  init/read functions; the visualizer is sensor-agnostic as long as the serial line
  format is unchanged.

- **Quaternions instead of Euler.** Euler angles suffer gimbal lock near +/-90 deg
  pitch. For robust orientation, read the BNO055 quaternion registers (`0x20`-`0x27`)
  and convert to a rotation matrix in the visualizer instead of `euler_to_matrix()`.

## Troubleshooting

- **No data / "waiting for COM4":** confirm the port, that the firmware is flashed,
  and that no other program (monitor, another terminal) holds the port.
- **Want to see the exact bytes:** run `python sniff.py COM4`. Valid lines look like
  `b'pitch:... roll:... yaw:...\r\n'`. The one-time ESP-ROM banner at reset is normal.
- **Display looks stuck or garbled with another serial tool:** make sure the two
  sdkconfig options above are set, then rebuild and reflash.

## License

MIT, see [LICENSE](LICENSE). Copyright (c) 2026 Hudson Reynolds.

The firmware is derived from the ESP-IDF `i2c_basic` example (Apache-2.0). BNO055
register usage follows the Bosch BNO055 datasheet. The visualizer does not use the
DFRobot Arduino library.



## AI Acknowledgement

This project was created in part with Claude Opus 4.8. 