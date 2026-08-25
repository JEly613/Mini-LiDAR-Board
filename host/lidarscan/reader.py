"""Packet sources: the real serial link, and a hardware-free simulator.

Both run the ingestion loop on a background thread and expose the same
snapshot-style interface, so the visualiser has no idea which one it is
talking to.  That is deliberate: it means ``--sim`` exercises the real parser,
the real decoders and the real statistics code, and only the bytes' origin
differs.
"""

from __future__ import annotations

import glob
import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from . import protocol as proto

SERIAL_PORT_GLOB = "/dev/cu.usbmodem*"


def find_serial_port() -> str | None:
    """Return the first ``/dev/cu.usbmodem*`` device, or None.

    macOS exposes two nodes per CDC device: ``/dev/tty.*`` blocks on open until
    DCD is asserted, while ``/dev/cu.*`` (call-up) does not.  Always use cu.
    """
    ports = sorted(glob.glob(SERIAL_PORT_GLOB))
    return ports[0] if ports else None


# --------------------------------------------------------------------------
# Shared state
# --------------------------------------------------------------------------
@dataclass
class LinkState:
    """A consistent snapshot of everything the UI shows.

    Guarded by a lock rather than made lock-free: the reader thread writes it a
    hundred times a second and the UI reads it thirty times a second, so
    contention is irrelevant and correctness is easy this way.
    """

    latest: proto.Orientation | None = None
    last_status: proto.Status | None = None
    status_log: deque = field(default_factory=lambda: deque(maxlen=8))

    # Rolling history for the time-series plots: (t_seconds, roll, pitch, yaw).
    history_t: deque = field(default_factory=lambda: deque(maxlen=1500))
    history_rpy: deque = field(default_factory=lambda: deque(maxlen=1500))

    # Latest complete LiDAR revolution, plus a rolling rate estimate.
    latest_scan: "proto.Scan | None" = None
    scan_rate_hz: float = 0.0
    scans_received: int = 0
    scan_seq_gaps: int = 0

    packet_rate_hz: float = 0.0
    packets_received: int = 0
    packets_expected: int = 0
    device_drop_count: int = 0
    decimation: int = proto.DEFAULT_DECIMATION
    connected: bool = False
    source_name: str = ""
    error: str | None = None

    @property
    def packet_loss_pct(self) -> float:
        if self.packets_expected <= 0:
            return 0.0
        lost = self.packets_expected - self.packets_received
        return max(0.0, 100.0 * lost / self.packets_expected)


def quat_to_euler_deg(q: tuple[float, float, float, float]) -> tuple[float, float, float]:
    """(w, x, y, z) -> (roll, pitch, yaw) in degrees, ZYX / aerospace order.

    Pitch is clamped before asin() because a quaternion that has drifted a
    hair off unit length can push the argument outside [-1, 1] and raise.
    """
    w, x, y, z = q

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class _BaseSource(threading.Thread):
    """Common bookkeeping: decode frames, update LinkState, track loss."""

    def __init__(self, name: str) -> None:
        super().__init__(daemon=True, name=name)
        self.parser = proto.FrameParser()
        self.state = LinkState(source_name=name)
        self.lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._t0 = time.monotonic()

        # Packet-rate estimation over a sliding one-second window.
        self._scan_window = deque(maxlen=256)
        self._rate_window: deque[float] = deque(maxlen=400)

        # Loss estimation, from the device's own sample counter.
        self._first_sample_count: int | None = None
        self._last_sample_count: int | None = None
        self._delta_hist: dict[int, int] = {}

        # Microsecond timestamp unwrapping (the device counter wraps at 2**32).
        self._t_wraps = 0
        self._last_t_us: int | None = None

    # -- lifecycle ---------------------------------------------------------
    def stop(self) -> None:
        self._stop_evt.set()

    def snapshot(self) -> LinkState:
        with self.lock:
            # Shallow copy is enough: the UI only reads scalars and iterates
            # the deques, and dataclasses.replace would copy the deques by
            # reference anyway.  Copying under the lock is what matters.
            import copy

            return copy.copy(self.state)

    # -- ingestion ---------------------------------------------------------
    def _handle_bytes(self, chunk: bytes) -> None:
        for frame in self.parser.feed(chunk):
            if frame.type == proto.TYPE_ORIENTATION:
                if len(frame.payload) != proto.ORIENTATION_PAYLOAD_LEN:
                    continue
                self._handle_orientation(proto.decode_orientation(frame.payload))
            elif frame.type == proto.TYPE_STATUS:
                if len(frame.payload) != proto.STATUS_PAYLOAD_LEN:
                    continue
                self._handle_status(proto.decode_status(frame.payload))
            elif frame.type == proto.TYPE_LIDAR_SCAN:
                try:
                    scan = proto.decode_scan(frame.payload)
                except ValueError:
                    # Header/length disagreement.  The CRC already passed, so
                    # this means a firmware/host format mismatch rather than
                    # line corruption -- worth counting, not worth crashing.
                    self.parser.stats.header_rejects += 1
                    continue
                self._handle_scan(scan)
            else:
                # Unknown type: counted by the parser and otherwise ignored.
                # This is the extensibility contract.
                pass

    def _unwrap_time(self, t_us: int) -> float:
        if self._last_t_us is not None and t_us < self._last_t_us:
            # Went backwards: the device's 32-bit microsecond counter wrapped
            # (every ~71.6 minutes).
            self._t_wraps += 1
        self._last_t_us = t_us
        return (t_us + self._t_wraps * (1 << 32)) / 1e6

    def _handle_orientation(self, o: proto.Orientation) -> None:
        now = time.monotonic()
        self._rate_window.append(now)
        while self._rate_window and (now - self._rate_window[0]) > 1.0:
            self._rate_window.popleft()

        # Loss: the device's sample_count advances by the decimation factor per
        # packet regardless of whether we saw the packet, so gaps in it are the
        # ground truth.  We infer the decimation from the stream rather than
        # hard-coding it.
        #
        # Use the MOST COMMON step, not the smallest.  The firmware stamps the
        # live icm_sample_count() when it builds the packet, not the index of
        # the sample it is processing, so when the main loop drains a backlog
        # the counter has already run on a little and the step jitters by a
        # few counts either way.  A minimum-based estimate latches onto the
        # first such outlier forever and then reports a large permanent loss
        # that is not real -- one stray step of 7 instead of 10 was enough to
        # invent a steady "30 % loss" on a link that was dropping nothing.
        if self._last_sample_count is not None:
            delta = o.sample_count - self._last_sample_count
            if 0 < delta < 10000:
                self._delta_hist[delta] = self._delta_hist.get(delta, 0) + 1
        if self._first_sample_count is None:
            self._first_sample_count = o.sample_count
        self._last_sample_count = o.sample_count

        decim = (max(self._delta_hist, key=self._delta_hist.get)
                 if self._delta_hist else proto.DEFAULT_DECIMATION)
        span = (self._last_sample_count - self._first_sample_count) // decim + 1

        t_s = self._unwrap_time(o.t_us)
        rpy = quat_to_euler_deg(o.q)

        with self.lock:
            st = self.state
            st.latest = o
            st.history_t.append(t_s)
            st.history_rpy.append(rpy)
            st.packet_rate_hz = float(len(self._rate_window))
            st.packets_received = self.parser.stats.by_type.get(
                proto.TYPE_ORIENTATION, 0)
            st.packets_expected = span
            st.device_drop_count = o.drop_count
            st.decimation = decim
            st.connected = True

    def _handle_scan(self, scan: "proto.Scan") -> None:
        now = time.monotonic()
        self._scan_window.append(now)
        while self._scan_window and (now - self._scan_window[0]) > 2.0:
            self._scan_window.popleft()

        with self.lock:
            st = self.state
            # A jump in seq means whole revolutions never reached us.
            if st.latest_scan is not None:
                gap = scan.seq - st.latest_scan.seq - 1
                if 0 < gap < 10000:
                    st.scan_seq_gaps += gap
            st.latest_scan = scan
            st.scans_received += 1
            st.scan_rate_hz = len(self._scan_window) / 2.0
            st.connected = True

    def _handle_status(self, s: proto.Status) -> None:
        with self.lock:
            self.state.last_status = s
            self.state.status_log.append(
                f"[{s.uptime_ms / 1000.0:8.2f}s] {s.name}: {s.msg}")


# --------------------------------------------------------------------------
# Real hardware
# --------------------------------------------------------------------------
class SerialSource(_BaseSource):
    def __init__(self, port: str | None = None, baud: int = 115200) -> None:
        super().__init__(name="serial")
        self.port = port
        self.baud = baud
        self._serial = None

    def send(self, frame: bytes) -> None:
        """Send a command frame to the device (best effort)."""
        ser = self._serial
        if ser is not None:
            try:
                ser.write(frame)
            except Exception:  # pragma: no cover - hardware-dependent
                pass

    def run(self) -> None:
        try:
            import serial  # pyserial
        except ImportError:
            with self.lock:
                self.state.error = (
                    "pyserial is not installed.  Run:\n"
                    "    python3 -m pip install pyserial\n"
                    "or use --sim to run without hardware.")
            return

        port = self.port or find_serial_port()
        if port is None:
            with self.lock:
                self.state.error = (
                    f"no device matching {SERIAL_PORT_GLOB}.\n"
                    "Is the board plugged in?  Use --sim to run without it.")
            return

        with self.lock:
            self.state.source_name = port

        try:
            # The baud rate is meaningless over USB CDC -- the host and device
            # exchange it as a formality -- but pyserial insists on one.
            # timeout keeps the read loop responsive to stop().
            self._serial = serial.Serial(port, self.baud, timeout=0.05)
        except Exception as exc:
            with self.lock:
                self.state.error = f"could not open {port}: {exc}"
            return

        with self._serial as ser:
            while not self._stop_evt.is_set():
                try:
                    n = max(1, ser.in_waiting)
                    chunk = ser.read(n)
                except Exception as exc:
                    with self.lock:
                        self.state.error = f"serial read failed: {exc}"
                        self.state.connected = False
                    return
                if chunk:
                    self._handle_bytes(chunk)


# --------------------------------------------------------------------------
# Simulator
# --------------------------------------------------------------------------
class SimSource(_BaseSource):
    """Synthesise a stream byte-for-byte identical to the firmware's.

    The motion is a deterministic, obviously-recognisable sweep so that a
    glance at the 3D view tells you whether the pipeline works: a steady yaw
    rotation with a slower roll and pitch wobble superimposed.

    Optionally injects corruption so the resync path gets exercised too --
    without hardware, this is the only way to prove the parser recovers.
    """

    def __init__(self, rate_hz: float = 100.0, corrupt_every: int = 0,
                 drop_every: int = 0) -> None:
        super().__init__(name="sim")
        self.rate_hz = rate_hz
        self.corrupt_every = corrupt_every
        self.drop_every = drop_every
        self.state.source_name = "simulator"
        self._n = 0
        # Simulated LiDAR controls, moved by the same commands the firmware
        # accepts so the UI's control path is exercised without hardware.
        self._motor_permille = proto.MOTOR_PERMILLE_DEFAULT
        self._decimation = 1
        self._scanning = True
        self._scan_seq = 0
        self._last_q = (1.0, 0.0, 0.0, 0.0)

    # -- simulated command sink -------------------------------------------
    def send(self, frame: bytes) -> None:
        """Interpret a command frame locally instead of writing it to a port."""
        for f in proto.FrameParser().feed(frame):
            if f.type == proto.TYPE_CMD_SET_MOTOR:
                self._motor_permille = int.from_bytes(f.payload[:2], "little")
            elif f.type == proto.TYPE_CMD_SET_DECIM:
                self._decimation = max(proto.DECIM_MIN,
                                       min(proto.DECIM_MAX, f.payload[0]))
            elif f.type == proto.TYPE_CMD_LIDAR_ENABLE:
                self._scanning = bool(f.payload[0])

    def _sim_rot_hz(self) -> float:
        """Map PWM duty to spindle speed the way a real A1 roughly behaves:
        nothing below the stall floor, then close to linear up to ~10 rev/s."""
        if self._motor_permille < proto.MOTOR_PERMILLE_MIN_RUN:
            return 0.0
        span = proto.MOTOR_PERMILLE_MAX - proto.MOTOR_PERMILLE_MIN_RUN
        frac = (self._motor_permille - proto.MOTOR_PERMILLE_MIN_RUN) / span
        return 2.0 + 8.0 * frac

    def make_scan_frame(self, t: float, q) -> bytes | None:
        """Synthesise one revolution of a plausible room.

        The 'room' is a 4 x 3 m rectangle with a circular pillar in it, sampled
        at whatever angular density the current speed and decimation imply.
        It is deliberately not symmetric, so a wrong angle convention or a
        reversed sweep is obvious on sight.
        """
        rot_hz = self._sim_rot_hz()
        if not self._scanning or rot_hz <= 0.0:
            return None

        sample_hz = 2300.0                       # the 115200-baud ceiling
        raw = max(16, int(sample_hz / rot_hz))
        n = max(8, raw // self._decimation)

        ang = np.linspace(0.0, 360.0, n, endpoint=False, dtype=np.float32)
        # Sweep the whole room slowly so the picture is alive, not frozen.
        th = np.radians(ang.astype(np.float64) + 12.0 * t)

        # Distance to the walls of a 4 x 3 m box centred on the sensor.
        half_x, half_y = 2000.0, 1500.0
        with np.errstate(divide="ignore", invalid="ignore"):
            dx = np.abs(half_x / np.cos(th))
            dy = np.abs(half_y / np.sin(th))
        d = np.minimum(np.nan_to_num(dx, nan=1e9, posinf=1e9),
                       np.nan_to_num(dy, nan=1e9, posinf=1e9))

        # A 300 mm pillar at (1200, 600) occludes part of the wall behind it.
        px, py, pr = 1200.0, 600.0, 150.0
        pang = math.atan2(py, px)
        pdist = math.hypot(px, py)
        dang = np.arctan2(np.sin(th - pang), np.cos(th - pang))
        hit = np.abs(dang) < math.asin(min(1.0, pr / pdist))
        d = np.where(hit, pdist - pr, d)

        d = d * (1.0 + np.random.normal(0.0, 0.004, size=n))   # ranging noise
        dist = np.clip(d, 0, 12000).astype(np.uint16)
        dist[np.random.random(n) < 0.02] = 0                   # 2 % no-return

        self._scan_seq += 1
        flags = (proto.FLAG_IMU_OK | proto.FLAG_BIAS_VALID
                 | proto.FLAG_USB_CONFIGURED | proto.FLAG_LIDAR_OK
                 | proto.FLAG_LIDAR_SCANNING)
        return proto.encode_scan(proto.Scan(
            t_us=int(t * 1e6) & 0xFFFFFFFF,
            seq=self._scan_seq,
            q=q,
            rot_hz=rot_hz,
            sample_hz=sample_hz,
            motor_permille=self._motor_permille,
            decimation=self._decimation,
            flags=flags,
            angles_deg=ang,
            dists_mm=dist,
        ))

    @staticmethod
    def _quat_from_euler(roll: float, pitch: float, yaw: float):
        """(radians) -> (w, x, y, z), ZYX order, matching quat_to_euler_deg."""
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        return (
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        )

    def make_frame(self, t: float) -> bytes:
        """Build one ORIENTATION frame for simulated time ``t`` seconds."""
        yaw = (0.6 * t) % (2 * math.pi)          # steady ~34 deg/s spin
        roll = 0.5 * math.sin(0.9 * t)           # +/- 28.6 deg wobble
        pitch = 0.35 * math.sin(0.45 * t + 1.0)  # +/- 20 deg wobble

        q = self._quat_from_euler(roll, pitch, yaw)
        self._last_q = q

        # A plausible gravity vector for that attitude, plus the body rates
        # that generated it, so the numeric readouts look real too.
        gx = math.degrees(0.5 * 0.9 * math.cos(0.9 * t))
        gy = math.degrees(0.35 * 0.45 * math.cos(0.45 * t + 1.0))
        gz = math.degrees(0.6)

        w, x, y, z = q
        # Third row of the rotation matrix: earth 'down' expressed in the body
        # frame, which is what an accelerometer at rest measures.
        ax = 2.0 * (x * z - w * y)
        ay = 2.0 * (w * x + y * z)
        az = 1.0 - 2.0 * (x * x + y * y)

        self._n += 1
        o = proto.Orientation(
            t_us=int(t * 1e6) & 0xFFFFFFFF,
            q=q,
            accel_g=(ax, ay, az),
            gyro_dps=(gx, gy, gz),
            temp_c=32.5,
            sample_count=self._n * proto.DEFAULT_DECIMATION,
            drop_count=0,
            imu_odr_hz=1000,
            flags=(proto.FLAG_IMU_OK | proto.FLAG_BIAS_VALID
                   | proto.FLAG_USB_CONFIGURED),
        )
        return proto.encode_orientation(o)

    def run(self) -> None:
        self._handle_bytes(proto.encode_status(proto.Status(
            t_us=0, uptime_ms=0, code=0, who_am_i=0x47,
            flags=proto.FLAG_IMU_OK | proto.FLAG_USB_CONFIGURED,
            msg="simulated source")))
        self._handle_bytes(proto.encode_status(proto.Status(
            t_us=0, uptime_ms=10, code=5, who_am_i=0x47,
            flags=(proto.FLAG_IMU_OK | proto.FLAG_BIAS_VALID
                   | proto.FLAG_USB_CONFIGURED),
            msg="gyro bias captured")))

        period = 1.0 / self.rate_hz
        start = time.monotonic()
        i = 0

        while not self._stop_evt.is_set():
            target = start + i * period
            delay = target - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            i += 1

            t = target - start
            frame = self.make_frame(t)

            # Interleave a revolution whenever one is due at the current
            # simulated spindle speed.
            rot_hz = self._sim_rot_hz()
            if rot_hz > 0.0:
                due = int(t * rot_hz)
                if due > self._scan_seq:
                    scan_frame = self.make_scan_frame(t, self._last_q)
                    if scan_frame is not None:
                        self._handle_bytes(scan_frame)

            if self.drop_every and (i % self.drop_every == 0):
                # Simulate a lost packet: the device's sample_count still
                # advances, so the loss statistic should notice.
                continue

            if self.corrupt_every and (i % self.corrupt_every == 0):
                # Flip a payload bit; the CRC must reject it and the parser
                # must resynchronise on the following frame.
                bad = bytearray(frame)
                bad[10] ^= 0xFF
                self._handle_bytes(bytes(bad))
                continue

            self._handle_bytes(frame)
