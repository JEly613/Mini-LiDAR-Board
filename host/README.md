# Mini-LiDAR-Board host viewer

Live 3D orientation display for the custom Mini-LiDAR scanner board. It opens
the board's USB CDC serial port, parses the binary telemetry stream described in
[`../firmware/PROTOCOL.md`](../firmware/PROTOCOL.md), and draws the board's
current attitude plus roll/pitch/yaw histories and link statistics.

There is **no LiDAR support here yet** — this milestone covers the on-board
ICM-42688-P IMU only. The parser already skips the reserved LiDAR packet types
gracefully, so adding them later will not break anything running today.

---

## Install

numpy and matplotlib are already present in the miniforge3 environment on this
machine. **`pyserial` is the only new dependency**, and it is needed only for
real hardware — `--sim` runs without it:

```
python3 -m pip install pyserial
```

or

```
python3 -m pip install -r requirements.txt
```

Nothing else is required. In particular this app deliberately does **not** use
Open3D, PyQt, pyqtgraph or vispy; none of them are installed, and matplotlib's
3D axes are perfectly adequate here.

---

## Run

```
cd host

python3 -m lidarscan                  # auto-detect /dev/cu.usbmodem*
python3 -m lidarscan --port /dev/cu.usbmodem14201
python3 -m lidarscan --sim            # synthetic data, no hardware needed
python3 -m lidarscan --selftest       # protocol + geometry checks, then exit
```

Useful flags:

| flag | effect |
|---|---|
| `--sim` | synthesise a rotating quaternion instead of reading hardware |
| `--sim-rate HZ` | simulated packet rate (default 100) |
| `--sim-corrupt N` | corrupt every Nth simulated frame, to exercise CRC + resync |
| `--sim-drop N` | drop every Nth simulated frame, to exercise the loss statistic |
| `--duration SECONDS` | exit automatically after this long |
| `--headless` | render with Agg, open no window (works over SSH / in CI) |
| `--save FILE.png` | write a PNG of the final frame |
| `--fps N` | redraw rate, default 30 |
| `--list-ports` | print candidate serial devices and exit |
| `--selftest` | run the built-in checks and exit |

Example that produces a screenshot with no hardware and terminates on its own:

```
python3 -m lidarscan --sim --headless --duration 8 --save shot.png
```

---

## What the display shows

**Left — 3D attitude.** A wireframe of the board (with a wedge marking the
USB-connector end so its heading is unambiguous) plus an RGB body-axis triad:
X red, Y green, Z blue. The grey square is the earth-horizontal plane, so
"level" is obvious at a glance.

**Top right — roll / pitch / yaw** over the last 10 seconds of device time. The
vertical jumps in yaw are the ±180° wrap, not a glitch.

**Bottom right — live statistics:**

| line | meaning |
|---|---|
| `rate` | orientation packets per second, measured over a 1 s sliding window |
| `loss` | derived from gaps in the device's own `sample_count`, not from counting arrivals — so it catches packets that never left the device |
| `dev drops` | frames the *device* discarded because its TX ring overflowed |
| `CRC err / resync / bad hdr` | parser health |
| `quat`, `rpy`, `accel`, `gyro`, `temp` | current sensor state |
| `gyro bias` | `calibrating`, `valid`, or `NOT CALIBRATED` |

Yaw drifts. There is no magnetometer on this board, so the filter is the 6-DOF
IMU-only Madgwick variant: roll and pitch are gravity-referenced and stable,
yaw is dead-reckoned from gyro Z. That is a property of the hardware.

---

## First run with real hardware

1. Flash the board (`cd ../firmware && make flash`) and plug in USB-C.
2. `python3 -m lidarscan --list-ports` should show a `/dev/cu.usbmodem…` node.
   Its suffix embeds the STM32's unique ID, so two boards get distinct names.
3. `python3 -m lidarscan`. The status panel will show `BOOT`, then `IMU_OK`,
   then `CALIB_START`.
4. **Hold the board still for ~2 seconds** while it measures the gyro bias.
   You will see `CALIB_DONE` and `gyro bias: valid`.
5. Pick it up and move it. The 3D board should follow.

If `gyro bias` stays `NOT CALIBRATED`, the board was moving during calibration;
put it down and it will have retried up to three times, or you can power-cycle.

### Troubleshooting

| symptom | likely cause |
|---|---|
| no `/dev/cu.usbmodem*` | board not enumerating — check the USB cable is a data cable, and see the firmware README's notes on the OTG_FS bring-up |
| `IMU_WHO_AM_I_FAIL` in the status log | SPI is working but the part did not answer `0x47` — check U2 orientation and the PA4 chip select |
| `IMU_NO_RESPONSE` | no SPI answer at all — check the IMU's 3.3 V rails and MISO on PA6 |
| high `CRC err` and rising `discarded` | electrical noise or a firmware framing bug; the parser recovers, but the numbers tell you it happened |
| `loss` climbing with `dev drops` climbing | the host is not reading fast enough; close other consumers of the port |
| `loss` climbing with `dev drops` at 0 | packets are being lost on the USB link itself |

---

## Layout

```
host/
  requirements.txt
  lidarscan/
    __init__.py
    __main__.py    CLI, argument parsing, and the built-in self-test
    protocol.py    frame codec + resynchronising parser (from PROTOCOL.md)
    reader.py      background ingestion thread; serial and simulated sources
    viz.py         matplotlib 3D view, time series and statistics panel
```

`protocol.py` and the firmware's `packet.c` are both written from
`firmware/PROTOCOL.md`. They are cross-checked: compiling `packet.c` natively
and feeding its output to `protocol.py` reproduces the values byte-for-byte, and
re-encoding from Python reproduces the firmware's bytes exactly.

---

## Self-test

`python3 -m lidarscan --selftest` runs 18 checks with no hardware:

* the CRC's published check value (`0x29B1` for `"123456789"`);
* frame sizes match `PROTOCOL.md` (68 B orientation, 52 B status);
* encode → parse → decode round-trips;
* resynchronisation past leading garbage;
* a corrupted frame is rejected and the *next* one still accepted;
* byte-at-a-time feeding gives the same result as one big chunk;
* reserved LiDAR packet types are passed through, not treated as corruption;
* a frame split across two reads reassembles;
* the view's rotation matrix is a proper rotation (orthonormal, det +1);
* a +90° yaw maps body X onto earth Y;
* the attitude the view would draw matches the simulator's ground truth to
  within 6e-8.

---

## LiDAR view and controls

The window shows three panels: board attitude in 3D, the current LiDAR
revolution in the sensor's own polar frame, and roll/pitch/yaw over time, plus
a live statistics panel.

**Rings are drawn flat, in the sensor frame.** They are not rotated by the IMU
quaternion — that is the next milestone. The quaternion is carried in every
scan packet already.

Three controls sit along the bottom, and all three change what the *hardware*
does, not just the plot:

| control | effect |
|---|---|
| **spin** | PWM duty on PB10 in permille (0–1000). 0 stops the spindle; below ~250 it stalls rather than turning slowly. |
| **keep 1 in N** | on-device decimation, applied before the packet is built — so it genuinely shrinks the USB traffic. |
| **scan: on/off** | sends the LiDAR `STOP` / `SCAN` command. Does not touch the motor. |

Because the A1's sample rate is fixed (~1954 samples/s measured, capped by the
115200-baud link), spinning slower yields *more* points per revolution rather
than more points per second:

| spin | rings/s | points/ring |
|---|---|---|
| duty 350 | ~2 | ~920 |
| duty 600 | 3.5 | ~550 |
| duty 820 (default) | 5.6 | ~350 |
| duty 900 | 6.2 | ~313 |

600 was the original default and turned out to be marginal — the spindle ran
below its rated speed and eventually stalled during a long session. 820 puts it
on the A1's nominal 5.5 rev/s.

Below ~2 rev/s a revolution exceeds the firmware's 1024-point cap; the plot
then labels the ring `[TRUNCATED]`.

A distance of 0 means "no return" on that ray, not zero range. Those points are
masked out of the plot and counted separately in the caption.

`--sim` synthesises a 4 × 3 m room with a pillar in it, at whatever density the
current simulated spin and decimation imply, so the whole UI including the
controls can be exercised with no hardware attached.

---

## 3D reconstruction (`--cloud`)

```
python3 -m lidarscan --cloud
python3 -m lidarscan --sim --cloud          # no hardware needed
```

Each LiDAR revolution is rotated into world space by the quaternion carried in
that same scan packet — the attitude at the instant the revolution closed — and
accumulated into a voxel-thinned point cloud centred on the device.

**Drag to orbit, scroll to zoom.** Keys: `p` pause accumulation, `c` clear,
`s` save PLY, `r` reset the view. Sliders set spindle speed, decimation, voxel
size and the display cube; buttons pause / clear / save.

### Frames

| frame | definition |
|---|---|
| sensor | raw ICM-42688-P axes; what Madgwick integrates |
| body | **+Y_imu = forward, −Z_imu = up** → X forward, Y left, Z up |
| earth | Madgwick's output frame, Z up. Yaw is free and drifts |

The sensor→body matrix is `[[0,1,0],[1,0,0],[0,0,−1]]` — det +1, so a rotation
and not a mirror. A point at range `d` and angle `θ` becomes
`R_es · M · (d cos θ, d sin θ, 0)`.

### Rotate the device, or you get a flat ring

**No translation is estimated.** The device is assumed to rotate in place, so
it stays at the origin. A stationary board therefore produces exactly one
horizontal ring, no matter how long you leave it running — which is correct,
not a bug. Tilt and sweep the unit to paint a 3D scene.

Measured with the board sitting still and level: 11 797 raw points collapsed to
202 voxels spanning just 108 mm in Z across a 4.2 m radius — a flat disc, as it
should be.

Because yaw has no magnetic reference it drifts, so a long session smears the
cloud about the vertical axis. Sweep and look; don't leave it accumulating for
ten minutes and expect crisp walls.

### Mounting the LiDAR

The A1 is a separate module on a cable, so its orientation relative to the PCB
is a mechanical fact the code cannot know. Two options cover it:

- `--lidar-yaw-deg D` — rotate the scan about the board's up axis until a
  landmark you know is dead ahead appears at the front of the cloud.
- `--lidar-ccw` — flip the sweep direction if the cloud comes out mirrored.

### Keeping it responsive

matplotlib is not a point-cloud engine. `--voxel-mm` (default 50) and
`--max-points` (default 60 000) are what keep the frame rate usable; past
roughly 100 000 points a redraw costs more than the frame interval. Repeated
sweeps of the same wall collapse onto the same voxels, so the cloud stops
growing once the room is covered.

`--save-ply PATH` writes a binary PLY (metres, colour by height) on exit, which
MeshLab, CloudCompare and Blender all open directly.
