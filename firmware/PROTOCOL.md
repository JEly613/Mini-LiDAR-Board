# Mini-LiDAR-Board wire protocol v1

This document is the **normative** definition of the binary link between the
board and the host. Both implementations are written from this table:

| Side | File |
|---|---|
| Firmware serialiser | `firmware/src/packet.c`, `firmware/include/packet.h` |
| Host parser | `host/lidarscan/protocol.py` |

If you change the format, change all three, and bump `version`.

The transport is a USB CDC-ACM (virtual serial) link. It is a **byte stream**,
not a message-oriented one — USB packet boundaries are not frame boundaries, so
the parser must find frames by scanning for the sync pattern and validating the
CRC. Never assume one read returns exactly one frame.

---

## 1. Frame layout

```
 offset  size  field      description
 ------  ----  ---------  ------------------------------------------------
      0     1  sync0      0xA5
      1     1  sync1      0x5A
      2     1  version    0x01 for this document
      3     1  type       see section 2
      4     2  length     payload length in bytes (uint16)
      6     N  payload    type-specific, N == length
    6+N     2  crc16      CRC-16/CCITT-FALSE over bytes [2 .. 6+N-1]
```

Total frame size is `8 + length`.

### Endianness

**Everything multi-byte is little-endian.** That includes `length`, `crc16`, all
integers in payloads, and all `float32` values (IEEE-754 binary32, stored
little-endian — i.e. what `struct.unpack('<f', ...)` reads and what a
Cortex-M4 or an x86/ARM host stores natively). No field anywhere in this
protocol is big-endian.

(The ICM-42688-P itself delivers big-endian sensor words on the SPI bus. That
is a detail entirely internal to the firmware; nothing big-endian reaches the
wire.)

### Checksum

CRC-16/CCITT-FALSE, also catalogued as CRC-16/IBM-3740:

| parameter | value |
|---|---|
| polynomial | `0x1021` |
| initial value | `0xFFFF` |
| input reflected | no |
| output reflected | no |
| final XOR | `0x0000` |

It covers `version`, `type`, `length` and the payload — bytes 2 through
`6+N-1` — but **not** the two sync bytes. The sync bytes are a resynchronisation
marker rather than data; including them would add nothing, whereas covering
`length` means a corrupted length can never make a receiver accept a mis-sized
frame.

### Parsing rules

1. Scan for `A5 5A`.
2. Read the 4 remaining header bytes. Reject (and resume scanning from the byte
   after `sync0`) if `version != 0x01` or `length > 128`.
3. Wait for `length + 2` more bytes.
4. Verify the CRC. On mismatch, discard and resume scanning from the byte after
   `sync0` — a false sync inside a payload is the normal cause.
5. Dispatch on `type`. **Unknown types must be skipped, not treated as an
   error**: that is what makes the format extensible.

---

## 2. Packet types

| range | direction | meaning |
|---|---|---|
| `0x01`–`0x0F` | device → host | IMU / orientation telemetry (this milestone) |
| `0x10`–`0x1F` | device → host | **reserved for LiDAR**, not implemented |
| `0x80`–`0x8F` | host → device | commands |

Assigned:

| type | name | payload | frame |
|---|---|---|---|
| `0x01` | `ORIENTATION` | 60 B | 68 B |
| `0x02` | `STATUS` | 44 B | 52 B |
| `0x10` | `LIDAR_SCAN` — **reserved, unimplemented** | — | — |
| `0x11` | `LIDAR_POINT` — **reserved, unimplemented** | — | — |
| `0x80` | `CMD_SET_BETA` | 4 B | 12 B |
| `0x81` | `CMD_RECALIBRATE` | 0 B | 8 B |
| `0x82` | `CMD_PING` | 0 B | 8 B |

Nothing in this firmware emits or parses `0x10`/`0x11`. They exist so that the
numbering is settled before a second data stream is added, and so the host
parser can already be written to ignore them gracefully.

---

## 3. `0x01` ORIENTATION — 60-byte payload

Emitted at ~100 Hz (every 10th 1 kHz filter update).

| offset | type | field | units / notes |
|---|---|---|---|
| 0 | `uint32` | `t_us` | device timestamp of the IMU sample, microseconds |
| 4 | `float32` | `q0` | quaternion scalar part (w) |
| 8 | `float32` | `q1` | quaternion x |
| 12 | `float32` | `q2` | quaternion y |
| 16 | `float32` | `q3` | quaternion z |
| 20 | `float32` | `ax` | accelerometer X, **g** |
| 24 | `float32` | `ay` | accelerometer Y, g |
| 28 | `float32` | `az` | accelerometer Z, g |
| 32 | `float32` | `gx` | gyroscope X, **deg/s**, bias removed |
| 36 | `float32` | `gy` | gyroscope Y, deg/s, bias removed |
| 40 | `float32` | `gz` | gyroscope Z, deg/s, bias removed |
| 44 | `float32` | `temp_c` | IMU die temperature, °C |
| 48 | `uint32` | `sample_count` | IMU samples read since boot |
| 52 | `uint32` | `drop_count` | frames discarded by the TX ring buffer |
| 56 | `uint16` | `imu_odr_hz` | configured IMU output data rate (1000) |
| 58 | `uint8` | `flags` | see section 5 |
| 59 | `uint8` | *reserved* | always 0 |

The quaternion is a **unit** quaternion describing the rotation from the sensor
frame to the earth frame, in Madgwick's `(w, x, y, z)` order.

### Timestamps

`t_us` comes from a free-running 32-bit 1 MHz timer (TIM5). It **wraps every
2³² µs ≈ 4295 s ≈ 71.6 minutes**. Hosts that need a monotonic timeline must
unwrap it themselves; the reference host parser does, by adding 2³² whenever a
timestamp goes backwards.

### Drift caveat

There is no magnetometer on this board, so the filter is the 6-DOF IMU-only
Madgwick variant. Roll and pitch are gravity-referenced and drift-free. **Yaw is
dead-reckoned from gyro Z and will drift** — typically a few degrees per minute
after bias calibration. This is a property of the sensor set, not a bug.

---

## 4. `0x02` STATUS — 44-byte payload

Emitted asynchronously on boot, on IMU init success/failure, around gyro
calibration, on stall detection and recovery, and in reply to `CMD_PING`.

| offset | type | field | notes |
|---|---|---|---|
| 0 | `uint32` | `t_us` | device timestamp, microseconds |
| 4 | `uint32` | `uptime_ms` | milliseconds since boot |
| 8 | `uint8` | `code` | see below |
| 9 | `uint8` | `who_am_i` | last WHO_AM_I byte read from the IMU (0x47 = OK) |
| 10 | `uint8` | `flags` | see section 5 |
| 11 | `uint8` | *reserved* | always 0 |
| 12 | `char[32]` | `msg` | NUL-padded ASCII, **not guaranteed NUL-terminated** |

Status codes:

| code | name | meaning |
|---|---|---|
| 0 | `BOOT` | clocks and USB are up |
| 1 | `IMU_OK` | ICM-42688-P identified and configured |
| 2 | `IMU_WHO_AM_I_FAIL` | device answered, but not with `0x47` |
| 3 | `IMU_NO_RESPONSE` | SPI reads returned only `0x00`/`0xFF` |
| 4 | `CALIB_START` | gyro bias calibration beginning (hold still) |
| 5 | `CALIB_DONE` | bias captured |
| 6 | `IMU_STALL` | no data-ready edge for 200 ms |
| 7 | `IMU_RECOVERED` | IMU re-initialised after a stall |
| 8 | `PONG` | reply to `CMD_PING` |
| 9 | `BETA_CHANGED` | filter gain updated |

---

## 5. Flags byte

Present in both `ORIENTATION` and `STATUS`.

| bit | mask | name | meaning |
|---|---|---|---|
| 0 | `0x01` | `IMU_OK` | IMU identified and streaming |
| 1 | `0x02` | `BIAS_VALID` | gyro bias calibration completed successfully |
| 2 | `0x04` | `USB_CONFIGURED` | host has issued SET_CONFIGURATION |
| 3 | `0x08` | `CALIBRATING` | bias calibration in progress / never completed |
| 4 | `0x10` | `STALL_RECOVERED` | at least one IMU stall recovery has occurred |

---

## 6. Host → device commands

Same framing. The firmware validates sync, version, length and CRC, and
silently ignores anything that fails or that carries an unknown type. Commands
must arrive as a single USB packet (≤ 64 bytes), which every command in this
version comfortably satisfies.

| type | name | payload |
|---|---|---|
| `0x80` | `CMD_SET_BETA` | `float32` — new Madgwick gain, accepted if `0 ≤ β ≤ 10` and not NaN |
| `0x81` | `CMD_RECALIBRATE` | none — restarts gyro bias calibration; hold the board still |
| `0x82` | `CMD_PING` | none — answered with a `STATUS`/`PONG` |
| `0x83` | `CMD_SET_MOTOR` | `uint16` — spindle PWM duty in permille, 0…1000. 0 stops the motor; below ~250 it stalls rather than turning slowly |
| `0x84` | `CMD_SET_DECIM` | `uint8` — keep 1 LiDAR point in N, 1…16. Applied on the device, before the packet is built |
| `0x85` | `CMD_LIDAR_ENABLE` | `uint8` — 0 sends the LiDAR `STOP` command, 1 re-runs health check + `SCAN`. Does not touch the motor |

---

## 7. LiDAR scan packet — type `0x10`

One packet per complete revolution of the RPLIDAR A1. A fixed 40-byte header
followed by `point_count` × 4 bytes, so the frame length is variable and the
reader must size the point array from `point_count`.

| off | size | field | notes |
|---|---|---|---|
| 0 | 4 | `t_us` | device microseconds when the revolution closed |
| 4 | 4 | `seq` | revolutions since scanning started; gaps mean lost packets |
| 8 | 16 | `q[4]` | `float32` w, x, y, z — board attitude at `t_us` |
| 24 | 4 | `rot_hz` | `float32`, measured spindle rate |
| 28 | 4 | `sample_hz` | `float32`, measured points/s **before** decimation |
| 32 | 2 | `point_count` | number of points that follow |
| 34 | 2 | `motor_permille` | PWM duty currently commanded |
| 36 | 1 | `decimation` | keep-1-in-N currently applied |
| 37 | 1 | `flags` | see the flag table |
| 38 | 2 | reserved | zero |
| 40 | 4×N | points | `uint16 angle_q6`, `uint16 dist_mm` |

`angle_q6` is degrees × 64 (so 0…23039). `dist_mm` is millimetres, and **0
means "no return"** on that ray — out of range or absorbed — not "zero
distance". A reader that plots zeros draws a bogus blob at the origin.

The quaternion is carried but **nothing rotates the ring by it yet**. This
milestone displays the two streams side by side. Pairing them here, at the one
place where both timestamps are known exactly, is what lets the later 3D stage
be a rotation rather than a re-derivation.

Largest legal frame: 8 + 40 + 1024 × 4 = **4152 bytes**.

### What is actually adjustable

The A1 has no command to change its internal measurement rate in standard
`SCAN` mode, and the 115200-baud link caps the wire at ~2300 samples/s (5 bytes
per sample) regardless. So `sample_hz` is roughly constant, and what the
operator really controls is:

- **spindle speed** (`CMD_SET_MOTOR`) — slower spin means *more* points per
  revolution, because the sample rate is fixed;
- **decimation** (`CMD_SET_DECIM`) — thins the points actually sent.

Below roughly 2 rev/s a revolution exceeds the 1024-point cap; the ring is
truncated and `FLAG_SCAN_TRUNCATED` is set rather than the length silently
disagreeing.

---

## 8. Flow control and loss

The firmware never blocks on USB. Frames go into a 16 KiB ring buffer that the
USB IN-endpoint interrupt drains. If the host stops reading, the ring fills and
**the oldest whole frames are discarded**; `drop_count` in the next
`ORIENTATION` packet reports the running total.

Push and discard are frame-granular, so an overflow never truncates a frame.
The *drain* is a byte stream: a scan frame is several times larger than one USB
transfer (192 bytes), so it is handed to the endpoint in pieces. That is
invisible to the host, which frames on the sync bytes plus CRC. The single
exception to "drop the oldest" is a frame that is already part-way out of the
endpoint — discarding that one would hand the host a fragment, so the incoming
frame is dropped instead.

Because the device is free-running, the host should treat a gap in
`sample_count` (which advances by ~10 per orientation packet) as the ground
truth for loss, rather than counting received packets alone.

One caveat: the firmware stamps the **live** `icm_sample_count()` when it
builds a packet, not the index of the sample it is processing, so when the main
loop drains a backlog the step jitters by a few counts around 10. A host
inferring the decimation from the stream must use the *most common* step, not
the smallest — a minimum-based estimate latches onto the first outlier and then
reports a large permanent loss that is not real.
