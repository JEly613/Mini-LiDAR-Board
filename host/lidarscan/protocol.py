"""Wire-format codec for the Mini-LiDAR-Board link.

This module is written from ``firmware/PROTOCOL.md``, which is the normative
definition.  The firmware serialiser (``firmware/src/packet.c``) is written from
the same document.  If you change the format, change all three and bump
``VERSION``.

Frame layout (all multi-byte fields little-endian)::

    0   1   sync0    0xA5
    1   1   sync1    0x5A
    2   1   version  0x01
    3   1   type
    4   2   length   payload length, uint16
    6   N   payload
  6+N   2   crc16    CRC-16/CCITT-FALSE over bytes [2 .. 6+N-1]
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Iterator

import numpy as np

# --------------------------------------------------------------------------
# Constants -- keep in lockstep with firmware/include/packet.h
# --------------------------------------------------------------------------
SYNC0 = 0xA5
SYNC1 = 0x5A
VERSION = 0x01

HEADER_LEN = 6
CRC_LEN = 2
OVERHEAD = HEADER_LEN + CRC_LEN
MAX_PAYLOAD = 4160   # dominated by a full LiDAR ring: 40 + 1024 * 4

TYPE_ORIENTATION = 0x01
TYPE_STATUS = 0x02

TYPE_LIDAR_SCAN = 0x10
RESERVED_LIDAR_RANGE = range(0x11, 0x20)   # still unused, still skipped safely

TYPE_CMD_SET_BETA = 0x80
TYPE_CMD_RECALIBRATE = 0x81
TYPE_CMD_PING = 0x82
TYPE_CMD_SET_MOTOR = 0x83
TYPE_CMD_SET_DECIM = 0x84
TYPE_CMD_LIDAR_ENABLE = 0x85

ORIENTATION_PAYLOAD_LEN = 60
STATUS_PAYLOAD_LEN = 44
STATUS_MSG_LEN = 32

SCAN_HEADER_LEN = 40
SCAN_POINT_LEN = 4

ORIENTATION_FRAME_LEN = OVERHEAD + ORIENTATION_PAYLOAD_LEN  # 68
STATUS_FRAME_LEN = OVERHEAD + STATUS_PAYLOAD_LEN            # 52

# Motor / decimation limits, mirrored from firmware/include/motor.h and
# firmware/include/rplidar.h.
MOTOR_PERMILLE_MAX = 1000
MOTOR_PERMILLE_MIN_RUN = 250
MOTOR_PERMILLE_DEFAULT = 820   # ~5.5 rev/s measured; see firmware/include/motor.h
DECIM_MIN = 1
DECIM_MAX = 16
LIDAR_MAX_POINTS = 1024

# Flags byte, shared by both telemetry payloads.
FLAG_IMU_OK = 0x01
FLAG_BIAS_VALID = 0x02
FLAG_USB_CONFIGURED = 0x04
FLAG_CALIBRATING = 0x08
FLAG_STALL_RECOVERED = 0x10
FLAG_LIDAR_OK = 0x20
FLAG_LIDAR_SCANNING = 0x40
FLAG_SCAN_TRUNCATED = 0x80

STATUS_CODES = {
    0: "BOOT",
    1: "IMU_OK",
    2: "IMU_WHO_AM_I_FAIL",
    3: "IMU_NO_RESPONSE",
    4: "CALIB_START",
    5: "CALIB_DONE",
    6: "IMU_STALL",
    7: "IMU_RECOVERED",
    8: "PONG",
    9: "BETA_CHANGED",
    10: "LIDAR_OK",
    11: "LIDAR_NO_RESPONSE",
    12: "LIDAR_BAD_HEALTH",
    13: "LIDAR_BAD_DESC",
    14: "LIDAR_STALL",
    15: "LIDAR_RECOVERED",
    16: "MOTOR_CHANGED",
    17: "DECIM_CHANGED",
    18: "LIDAR_STOPPED",
}

# The firmware emits one ORIENTATION frame per this many IMU samples.  Used
# only as a fallback; the reader infers the real value from the stream.
DEFAULT_DECIMATION = 10

# struct formats.  '<' pins little-endian AND disables alignment padding, which
# is what makes these match the firmware's byte-at-a-time serialiser exactly.
_HEADER = struct.Struct("<BBBBH")
_ORIENTATION = struct.Struct("<I4f3f3f f I I H B B")
_STATUS = struct.Struct("<II BBBB 32s")
_SCAN_HDR = struct.Struct("<II 4f f f HH BB H")

assert _ORIENTATION.size == ORIENTATION_PAYLOAD_LEN, _ORIENTATION.size
assert _STATUS.size == STATUS_PAYLOAD_LEN, _STATUS.size
assert _SCAN_HDR.size == SCAN_HEADER_LEN, _SCAN_HDR.size


# --------------------------------------------------------------------------
# CRC-16/CCITT-FALSE  (poly 0x1021, init 0xFFFF, no reflection, no final XOR)
# --------------------------------------------------------------------------
def _build_crc_table() -> list[int]:
    table = []
    for byte in range(256):
        crc = byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
        table.append(crc)
    return table


_CRC_TABLE = _build_crc_table()


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE over ``data``."""
    crc = 0xFFFF
    for byte in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC_TABLE[((crc >> 8) ^ byte) & 0xFF]
    return crc


# --------------------------------------------------------------------------
# Decoded payloads
# --------------------------------------------------------------------------
@dataclass(slots=True)
class Orientation:
    t_us: int
    q: tuple[float, float, float, float]      # (w, x, y, z)
    accel_g: tuple[float, float, float]
    gyro_dps: tuple[float, float, float]
    temp_c: float
    sample_count: int
    drop_count: int
    imu_odr_hz: int
    flags: int

    @property
    def imu_ok(self) -> bool:
        return bool(self.flags & FLAG_IMU_OK)

    @property
    def bias_valid(self) -> bool:
        return bool(self.flags & FLAG_BIAS_VALID)

    @property
    def calibrating(self) -> bool:
        return bool(self.flags & FLAG_CALIBRATING)


@dataclass(slots=True)
class Scan:
    """One complete LiDAR revolution, in the sensor's own 2D polar frame.

    ``q`` is the board attitude at the instant the revolution closed.  Nothing
    in this milestone applies it -- the ring is drawn flat -- but it is carried
    so the later 3D stage does not have to re-derive the pairing.
    """

    t_us: int
    seq: int
    q: tuple[float, float, float, float]
    rot_hz: float
    sample_hz: float
    motor_permille: int
    decimation: int
    flags: int
    angles_deg: "np.ndarray"      # float32, degrees, 0 .. 360
    dists_mm: "np.ndarray"        # uint16, millimetres; 0 = no return

    @property
    def point_count(self) -> int:
        return int(self.angles_deg.size)

    @property
    def truncated(self) -> bool:
        return bool(self.flags & FLAG_SCAN_TRUNCATED)

    @property
    def valid_mask(self) -> "np.ndarray":
        """A zero distance is the A1's way of saying 'no return on this ray'
        (out of range, or absorbed).  Plotting those as points at the origin
        would draw a bogus blob in the middle of every scan."""
        return self.dists_mm > 0


@dataclass(slots=True)
class Status:
    t_us: int
    uptime_ms: int
    code: int
    who_am_i: int
    flags: int
    msg: str

    @property
    def name(self) -> str:
        return STATUS_CODES.get(self.code, f"UNKNOWN({self.code})")


@dataclass(slots=True)
class Frame:
    """A frame whose header and CRC checked out, but whose payload may be of a
    type this host does not know about."""

    type: int
    payload: bytes


# --------------------------------------------------------------------------
# Encoding
# --------------------------------------------------------------------------
def build_frame(ptype: int, payload: bytes = b"") -> bytes:
    """Wrap ``payload`` in the standard framing."""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload too long: {len(payload)}")
    body = struct.pack("<BBH", VERSION, ptype, len(payload)) + payload
    return bytes([SYNC0, SYNC1]) + body + struct.pack("<H", crc16(body))


def encode_orientation(o: Orientation) -> bytes:
    """Build an ORIENTATION frame.  Used by the simulator so that ``--sim``
    exercises the identical encode/decode path the hardware does."""
    payload = _ORIENTATION.pack(
        o.t_us,
        o.q[0], o.q[1], o.q[2], o.q[3],
        o.accel_g[0], o.accel_g[1], o.accel_g[2],
        o.gyro_dps[0], o.gyro_dps[1], o.gyro_dps[2],
        o.temp_c,
        o.sample_count,
        o.drop_count,
        o.imu_odr_hz,
        o.flags,
        0,  # reserved
    )
    return build_frame(TYPE_ORIENTATION, payload)


def encode_status(s: Status) -> bytes:
    msg = s.msg.encode("ascii", "replace")[:STATUS_MSG_LEN]
    payload = _STATUS.pack(
        s.t_us, s.uptime_ms, s.code, s.who_am_i, s.flags, 0,
        msg.ljust(STATUS_MSG_LEN, b"\x00"),
    )
    return build_frame(TYPE_STATUS, payload)


def cmd_set_beta(beta: float) -> bytes:
    return build_frame(TYPE_CMD_SET_BETA, struct.pack("<f", beta))


def cmd_recalibrate() -> bytes:
    return build_frame(TYPE_CMD_RECALIBRATE)


def cmd_ping() -> bytes:
    return build_frame(TYPE_CMD_PING)


def cmd_set_motor(permille: int) -> bytes:
    """Spindle PWM duty, 0..1000.  0 stops the motor."""
    permille = max(0, min(MOTOR_PERMILLE_MAX, int(permille)))
    return build_frame(TYPE_CMD_SET_MOTOR, struct.pack("<H", permille))


def cmd_set_decimation(every_nth: int) -> bytes:
    """Keep 1 point in N.  Applied on the device, so it genuinely shrinks the
    USB packet rather than only thinning the plot."""
    every_nth = max(DECIM_MIN, min(DECIM_MAX, int(every_nth)))
    return build_frame(TYPE_CMD_SET_DECIM, struct.pack("<B", every_nth))


def cmd_lidar_enable(on: bool) -> bytes:
    return build_frame(TYPE_CMD_LIDAR_ENABLE, struct.pack("<B", 1 if on else 0))


def encode_scan(s: Scan) -> bytes:
    """Build a SCAN frame.  Used by the simulator, so ``--sim`` exercises the
    identical encode/decode path the hardware does."""
    n = s.point_count
    hdr = _SCAN_HDR.pack(
        s.t_us, s.seq,
        s.q[0], s.q[1], s.q[2], s.q[3],
        s.rot_hz, s.sample_hz,
        n, s.motor_permille,
        s.decimation, s.flags, 0,
    )
    angles_q6 = np.clip(np.rint(np.asarray(s.angles_deg, dtype=np.float64) * 64.0),
                        0, 65535).astype("<u2")
    dists = np.asarray(s.dists_mm).astype("<u2")

    # Interleave (angle_q6, dist_mm) pairs -- the firmware's exact layout.
    pts = np.empty(n * 2, dtype="<u2")
    pts[0::2] = angles_q6
    pts[1::2] = dists
    return build_frame(TYPE_LIDAR_SCAN, hdr + pts.tobytes())


# --------------------------------------------------------------------------
# Decoding
# --------------------------------------------------------------------------
def decode_orientation(payload: bytes) -> Orientation:
    (t_us, q0, q1, q2, q3, ax, ay, az, gx, gy, gz,
     temp_c, sample_count, drop_count, odr, flags, _res) = _ORIENTATION.unpack(payload)
    return Orientation(
        t_us=t_us,
        q=(q0, q1, q2, q3),
        accel_g=(ax, ay, az),
        gyro_dps=(gx, gy, gz),
        temp_c=temp_c,
        sample_count=sample_count,
        drop_count=drop_count,
        imu_odr_hz=odr,
        flags=flags,
    )


def decode_status(payload: bytes) -> Status:
    t_us, uptime_ms, code, who, flags, _res, msg = _STATUS.unpack(payload)
    return Status(
        t_us=t_us,
        uptime_ms=uptime_ms,
        code=code,
        who_am_i=who,
        flags=flags,
        msg=msg.split(b"\x00", 1)[0].decode("ascii", "replace"),
    )


def decode_scan(payload: bytes) -> Scan:
    if len(payload) < SCAN_HEADER_LEN:
        raise ValueError(f"scan payload too short: {len(payload)}")

    (t_us, seq, q0, q1, q2, q3, rot_hz, sample_hz,
     n, motor, decim, flags, _res) = _SCAN_HDR.unpack_from(payload, 0)

    want = SCAN_HEADER_LEN + n * SCAN_POINT_LEN
    if len(payload) != want:
        raise ValueError(f"scan payload is {len(payload)} B, header says {want}")

    pts = np.frombuffer(payload, dtype="<u2", count=n * 2, offset=SCAN_HEADER_LEN)
    return Scan(
        t_us=t_us,
        seq=seq,
        q=(q0, q1, q2, q3),
        rot_hz=rot_hz,
        sample_hz=sample_hz,
        motor_permille=motor,
        decimation=decim,
        flags=flags,
        # angle_q6 is degrees * 64; keep it float32, the display needs no more.
        angles_deg=(pts[0::2].astype(np.float32) / np.float32(64.0)),
        dists_mm=pts[1::2].copy(),
    )


@dataclass
class ParserStats:
    """Everything a user needs to judge link health."""

    frames_ok: int = 0
    crc_errors: int = 0
    header_rejects: int = 0     # bad version or absurd length
    bytes_discarded: int = 0    # bytes thrown away while resynchronising
    resyncs: int = 0            # times we had to hunt for a new sync pattern
    by_type: dict[int, int] = field(default_factory=dict)

    def note_type(self, ptype: int) -> None:
        self.by_type[ptype] = self.by_type.get(ptype, 0) + 1


# Sentinels returned by FrameParser._try_one().  Using distinct objects rather
# than None keeps "no complete frame yet" and "bad data, try again" apart.
_NEED_MORE = object()
_RETRY = object()


class FrameParser:
    """Incremental, resynchronising frame parser.

    Feed it arbitrary byte chunks; it yields complete, CRC-validated frames.
    It never raises on malformed input -- garbage is counted and dropped, which
    is the only sane behaviour for a link that can be unplugged mid-frame.
    """

    def __init__(self, max_buffer: int = 1 << 16) -> None:
        self._buf = bytearray()
        self._max_buffer = max_buffer
        self.stats = ParserStats()

    def feed(self, chunk: bytes) -> Iterator[Frame]:
        self._buf.extend(chunk)

        # Runaway protection: if we somehow accumulate far more than a frame's
        # worth without ever finding a valid one, drop the oldest half rather
        # than growing without bound.
        if len(self._buf) > self._max_buffer:
            drop = len(self._buf) // 2
            self.stats.bytes_discarded += drop
            del self._buf[:drop]

        while True:
            result = self._try_one()
            if result is _NEED_MORE:
                return
            if result is _RETRY:
                # A bad header or CRC; the buffer has been advanced past the
                # false sync, so go round again rather than giving up on the
                # bytes we already hold.
                continue
            yield result  # type: ignore[misc]

    def _resync_from(self, start: int) -> bool:
        """Drop bytes up to the next plausible sync pattern.  Returns True if
        one was found (and the buffer now starts with it)."""
        idx = self._buf.find(bytes([SYNC0, SYNC1]), start)
        if idx < 0:
            # Keep the last byte: it might be a SYNC0 whose SYNC1 has not
            # arrived yet.
            keep = 1 if self._buf and self._buf[-1] == SYNC0 else 0
            drop = len(self._buf) - keep
            if drop > 0:
                self.stats.bytes_discarded += drop
                del self._buf[:drop]
            return False
        if idx > 0:
            self.stats.bytes_discarded += idx
            del self._buf[:idx]
            self.stats.resyncs += 1
        return True

    def _try_one(self):
        """Attempt to pull one frame off the front of the buffer.

        Returns a :class:`Frame`, ``_NEED_MORE`` (wait for more bytes) or
        ``_RETRY`` (the buffer was advanced past bad data; call again).
        """
        if len(self._buf) < 2:
            return _NEED_MORE

        if self._buf[0] != SYNC0 or self._buf[1] != SYNC1:
            if not self._resync_from(0):
                return _NEED_MORE

        if len(self._buf) < HEADER_LEN:
            return _NEED_MORE

        _s0, _s1, version, ptype, length = _HEADER.unpack_from(self._buf, 0)

        if version != VERSION or length > MAX_PAYLOAD:
            # A false sync inside a payload looks exactly like this.  Skip past
            # this sync pair and hunt for the next one.
            self.stats.header_rejects += 1
            self._drop_one_and_resync()
            return _RETRY

        total = OVERHEAD + length
        if len(self._buf) < total:
            return _NEED_MORE

        body = bytes(self._buf[2:HEADER_LEN + length])
        got = int.from_bytes(self._buf[HEADER_LEN + length:total], "little")

        if crc16(body) != got:
            self.stats.crc_errors += 1
            self._drop_one_and_resync()
            return _RETRY

        payload = bytes(self._buf[HEADER_LEN:HEADER_LEN + length])
        del self._buf[:total]

        self.stats.frames_ok += 1
        self.stats.note_type(ptype)
        return Frame(type=ptype, payload=payload)

    def _drop_one_and_resync(self) -> None:
        """Discard the byte that looked like SYNC0 and hunt for the next
        candidate.  Advancing by exactly one byte is what guarantees forward
        progress no matter what the input looks like."""
        self.stats.bytes_discarded += 1
        del self._buf[:1]
        self._resync_from(0)
