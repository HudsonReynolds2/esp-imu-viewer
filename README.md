# BNO055 Real-Time IMU Visualizer

Reads the full output of a Bosch **BNO055** (DFRobot SEN0253 "10 DOF IMU AHRS")
over I2C on a **Seeed XIAO ESP32-C3**, streams every sensor channel over USB, and
renders a live 3D view plus real-time graphs on your PC.

The firmware runs the BNO055 in NDOF fusion mode and reads the entire data block
(accelerometer, magnetometer, gyroscope, fused quaternion, linear acceleration,
gravity, temperature, and calibration status) in one I2C burst per cycle at a true
100 Hz. It prints a version-prefixed, comma-separated line per sample:

```
v2,seq,t_us,qw,qx,qy,qz,ax,ay,az,gx,gy,gz,mx,my,mz,lx,ly,lz,grx,gry,grz,temp,cs,cg,ca,cm
```

where `seq` is a uint32 sample counter (for dropped-sample detection) and `t_us` is
the BNO055-side uint64 microsecond timestamp (the authoritative clock for rate and
jitter). The board is rotated from the **fused quaternion**, not Euler angles, so the
3D view never hits gimbal lock near +/-90 degrees pitch.

The included Python visualizer (`imu_view.py`) draws a rotating board with a fixed
world frame (X/Y/Z) and a device-fixed frame (X'/Y'/Z'), a numeric panel for every
sensor, a strip of live scrolling graphs, stream-quality stats (device rate, jitter,
dropped samples), and a one-key calibrate. It splits each line on commas and checks
the `v2` version tag and field count, so it ignores anything that is not a valid data
line and the ESP32 boot banner can never desync it.

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
├── imu_view.py              live 3D visualizer + graphs + CSV logging (pygame)
├── sniff.py                 serial debug helper: prints raw bytes (OUT OF DATE,
│                            see note below)
├── LICENSE
├── .gitignore
└── README.md
```

> **Note:** `sniff.py` predates the current `v2` line format and has not yet been
> updated. It still runs as a raw-byte viewer, but its documented example output and
> any built-in parsing reflect the old `pitch:.. roll:.. yaw:..` format. It needs an
> update to match the `v2` CSV stream; treat it as a rough byte dumper for now.

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
`bno055_imu_show_main.c`. The I2C bus runs at 400 kHz (fast mode): the 46-byte burst
read plus the BNO055's clock-stretching does not fit inside the 10 ms (100 Hz) budget
at 100 kHz, so fast mode is what lets the loop hold a true 100 Hz.

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

Three settings make the stream clean and the loop timing correct. Set them once via
`idf.py menuconfig`; they are saved in the generated `sdkconfig`.

1. **1000 Hz FreeRTOS tick.** The loop uses `xTaskDelayUntil()` with a 10 ms period to
   hold 100 Hz. With the default 100 Hz tick, a 10 ms period rounds to a single tick
   and cannot resolve 100 Hz, which caps the rate near 71 Hz. Set the tick to 1000 Hz:

   - `Component config` → `FreeRTOS` → `Kernel` → `configTICK_RATE_HZ` → **1000**
   - Kconfig symbol: `CONFIG_FREERTOS_HZ=1000`

2. **LF line endings on stdout.** By default ESP-IDF translates `\n` into `\r\n`. The
   firmware emits its own CR+LF and a strict parser expects exactly that, so stop the
   console from adding a second CR:

   - `Component config` → `Standard IO (libc)` → `Line ending for UART output` → **LF**
   - Kconfig symbol: `CONFIG_LIBC_STDOUT_LINE_ENDING_LF=y`
     (on IDF < 5.5 this symbol is `CONFIG_NEWLIB_STDOUT_LINE_ENDING_LF`)

3. **Disable IDF logging on the console.** So the only thing on the port is the `v2`
   data stream, with no `I (123) ...` log lines mixed in:

   - `Component config` → `Log` → `Default log verbosity` → **No output**
   - Kconfig symbols: `CONFIG_LOG_DEFAULT_LEVEL_NONE=y` and
     `CONFIG_LOG_MAXIMUM_LEVEL_NONE=y`

The firmware also bypasses blocking stdio for the data stream: it installs the
USB Serial/JTAG driver and queues each line with `usb_serial_jtag_write_bytes(..., 0)`
so the write returns immediately instead of busy-waiting for the host to drain the
endpoint (the blocking path cost ~9-12 ms/cycle and capped the loop near 71 Hz). The
console baud rate stays at the default 115200, which matches the visualizer.

> The ESP-ROM boot banner (e.g. `ESP-ROM:esp32c3-...`) is printed by hardware before
> your code runs and cannot be disabled in firmware. The visualizer ignores it; if you
> use a different tool, reset the board and let the banner pass before connecting.

### Built-in timing diagnostics

The firmware has a compile-time `DIAG` switch (top of `bno055_imu_show_main.c`,
default `0`). Set it to `1` and rebuild to emit timing diagnostics on the data stream:
a one-time `# diag` line reporting the compiled tick rate, and an every-100th-sample
`# t read_us/print_us/busy_us` line. Healthy reference numbers from the working 100 Hz
build: `read_us` ~1600-7000, `print_us` ~850-950, `busy_us` ~2500-7800 (the budget is
10000 us). These `#` lines are not valid `v2` data lines, so the visualizer ignores
them.

To watch the raw output, `idf.py -p COM4 monitor` (exit with Ctrl+]). Close the
monitor before running the visualizer; only one program can hold the port.

## Visualizer: install and run

Python 3.9+ recommended. Use a virtual environment. Note that the visualizer now
requires **numpy** in addition to pyserial and pygame (it backs the sample ring buffer
and the graph math).

### Windows (PowerShell)

```
python -m venv .venv
# If activation is blocked:
Set-ExecutionPolicy -ExecutionPolicy Bypass -Scope Process
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install pyserial pygame numpy
```

### macOS / Linux

```
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install pyserial pygame numpy
```

### Run

```
python imu_view.py COM4
```

(Use your actual port. Optional: `--baud 115200`, which is already the default.)

A window opens showing the board. Top-left shows connection status and a per-sensor
numeric panel; top-right shows render FPS, the device data rate in Hz, inter-sample
jitter, and the dropped-sample count; a version-mismatch warning appears there if the
firmware's `v2` tag does not match the visualizer. The bottom strip shows live
scrolling graphs of the enabled sensor groups.

The numeric panel and graphs cover every channel the firmware streams: fused
quaternion, accelerometer (m/s^2), gyroscope (dps), magnetometer (uT), linear
acceleration (m/s^2), gravity (m/s^2), temperature (deg C), and the four calibration
levels.

### Controls

| Key / input        | Action                                                       |
|--------------------|--------------------------------------------------------------|
| Mouse scroll, +, - | Zoom in / out                                                |
| C                  | Calibrate: set current device orientation as zero            |
| R                  | Reset calibration back to raw orientation                    |
| 1 - 7              | Toggle sensor panels: quat / accel / gyro / mag / linear / gravity / temp |
| TAB                | Show / hide the graph strip                                  |
| A S D G H J K      | Toggle individual graph panels (same order as the panel list; F is skipped because it is fullscreen) |
| L                  | Start / stop CSV recording                                   |
| F, F11             | Toggle fullscreen                                            |
| Esc, Q             | Quit                                                         |

Calibrate (C) snapshots the current orientation and makes it the new reference, so
the device frame (X'/Y'/Z') aligns with the world frame (X/Y/Z) and all motion after
that is measured relative to that pose. The displayed pitch/roll/yaw are derived from
the calibrated quaternion purely for the numeric readout and can go singular near
+/-90 degrees pitch, while the 3D board (rotated from the quaternion) stays correct.

### Calibration

In NDOF mode the absolute orientation is only trustworthy once all four BNO055
calibration levels (sys/gyr/acc/mag) reach 3. The visualizer shows these levels
(red until 3, green at 3) and displays a coaching hint for each subsystem still below
3: hold still to calibrate the gyro, tilt through several poses for the accelerometer,
and wave a figure-8 for the magnetometer. The hint disappears once you are fully
calibrated.

### CSV recording

Press **L** to start and stop recording. Each session writes
`imu_YYYYMMDD_HHMMSS.csv` in the current directory, with a header row and one row per
sample: every streamed field (with `t_us` as the authoritative timestamp) plus a
trailing `host_unix_s` host wall-clock column for correlating with the outside world.
The logger drains the sample ring buffer incrementally, so it records every sample
regardless of the render frame rate, and the on-screen graph decimation never thins
the logged data. A red `REC` indicator with the filename and row count shows while
recording.

### Graph configuration

Graph appearance (which channels show, window length, colors, layout, y-scales) is
configured in the `GRAPH_CONFIG` block near the top of `imu_view.py`. Each panel's
`y` is either `"auto"` (autoscale to the visible window) or a fixed `(lo, hi)` tuple.
The runtime toggle keys (A S D G H J K) follow the same order the panels are listed
in `GRAPH_CONFIG`.

### Demo Video
[![Demo Video](https://img.youtube.com/vi/4d74bNJ_7S8/maxresdefault.jpg)](https://youtu.be/4d74bNJ_7S8)

#### More thorough testing of the sensor, and further upgrades to the tool, should be made.

## Adapting or upgrading

A few common changes and where to make them:

- **Axis directions.** If an axis feels inverted for how your board is mounted, flip
  the corresponding quaternion component sign where the quaternion is unpacked in
  `quat_to_matrix()` in `imu_view.py` (clearly marked there). The 3D view is driven by
  the quaternion, not by Euler angles.

- **Output rate.** The firmware loop runs at 100 Hz via `xTaskDelayUntil()` with a
  10 ms period (`pdMS_TO_TICKS(10)`) in `app_main()`, which matches the BNO055's NDOF
  fusion rate. Increase the period for a slower stream; going faster than ~100 Hz
  yields no new fusion data. Holding a true 100 Hz also requires `CONFIG_FREERTOS_HZ=1000`
  (see the sdkconfig options above).

- **Line format.** The firmware prints the fixed `v2` positional CSV described at the
  top. The leading `v2` tag lets the visualizer reject a mismatched firmware/visualizer
  pair instead of silently misparsing. If you change the field order or the tag, update
  `WIRE_VERSION` and the `FIELDS` list in `imu_view.py` so they still match (the
  visualizer splits on commas and slices by position; it is not a regex). `sniff.py`
  will also need updating, since it predates this format.

- **Reading different registers.** The firmware already reads the entire page-0 data
  block (accel, mag, gyro, Euler, quaternion, linear accel, gravity, temperature, and
  calibration) in one burst in `bno_read_all()`. The Euler registers are read but not
  emitted, since the visualizer derives angles from the quaternion. To stream more or
  fewer fields, edit the burst slicing in `bno_read_all()`, the printed line in
  `app_main()`, and the `FIELDS` list in `imu_view.py` together.

- **A different ESP32 or sensor.** The I2C setup uses the ESP-IDF `i2c_master` driver,
  which is portable across ESP32 variants; only the default SDA/SCL pins are
  board-specific. For a different IMU, replace the register definitions and the
  init/read functions; the visualizer is sensor-agnostic as long as the serial line
  format is unchanged.

## Troubleshooting

- **No data / "waiting for COM4":** confirm the port, that the firmware is flashed, and
  that no other program (monitor, another terminal) holds the port. The visualizer
  reopens the port automatically across resets / USB re-enumeration, so it is safe to
  reset the board while it runs.

- **"firmware/visualizer version mismatch" (top-right):** the firmware's line tag does
  not match the visualizer's `WIRE_VERSION`. Reflash matching firmware, or update
  `WIRE_VERSION` / `FIELDS` in `imu_view.py` to match the firmware you are running.

- **`|q|` warning (amber) in the readout:** a healthy unit quaternion has magnitude
  ~1.0. A drift away from 1.0 flags a parse/scale problem or all-zero data (sensor not
  yet in fusion).

- **Display looks stuck or garbled with another serial tool:** make sure the three
  sdkconfig options above are set, then rebuild and reflash.

- **Want to see the exact bytes:** `sniff.py` is the intended helper but is out of date
  (see the note in the repository layout); it needs updating to the `v2` format before
  its output matches the current stream.

## License

MIT, see [LICENSE](LICENSE). Copyright (c) 2026 Hudson Reynolds.

The firmware is derived from the ESP-IDF `i2c_basic` example (Apache-2.0). BNO055
register usage follows the Bosch BNO055 datasheet. The visualizer does not use the
DFRobot Arduino library.

## AI Acknowledgement

This project was created in part with Claude Opus 4.8.
