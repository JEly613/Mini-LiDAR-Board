"""3D reconstruction WITH translation, from double-integrated accelerometer.

This is the dead-reckoning counterpart to :mod:`cloud`.  That module assumes
the device only ever rotates in place and pins it to the origin; this one
estimates where the device has *moved* to and places each revolution at the
position it was actually taken from.

Run them side by side on the same board to compare::

    python3 -m lidarscan --cloud        # rotation only, device at origin
    python3 -m lidarscan --cloud-ins    # dead reckoning, device moves

``cloud.py`` is untouched.  The geometry primitives that do not change --
:func:`~lidarscan.cloud.quat_to_matrix`, :func:`~lidarscan.cloud.body_to_earth`,
:func:`~lidarscan.cloud.ring_to_body`, :class:`~lidarscan.cloud.VoxelCloud` --
are imported from it rather than copied, so a diff between the two files shows
exactly the change under test and nothing else.

---------------------------------------------------------------------------
READ THIS BEFORE TRUSTING THE OUTPUT
---------------------------------------------------------------------------
Double integration of a MEMS accelerometer diverges *quadratically*.  Position
error from a constant acceleration error ``a_err`` after time ``t`` is::

    p_err = 0.5 * a_err * t^2

Two error sources dominate, and neither is an implementation defect:

**Attitude error leaks gravity.**  This is almost always the bigger one, and
it is the reason naive dead reckoning fails so badly.  Gravity is ~9.81 m/s^2
and we subtract it using the *estimated* attitude, so a tilt error of ``e``
leaks ``g*sin(e)`` into the horizontal axes::

    0.5 deg  ->  0.086 m/s^2  ->  4.3 mm after 1 s, 0.43 m after 10 s
    1.0 deg  ->  0.171 m/s^2  ->  8.6 mm after 1 s, 0.86 m after 10 s
    2.0 deg  ->  0.342 m/s^2  ->   17 mm after 1 s,  1.7 m after 10 s

A 6-DOF Madgwick filter with no magnetometer holds roll and pitch to roughly
1 deg while moving, so the metre-per-ten-seconds row is the realistic one.

**Accelerometer bias.**  The ICM-42688-P's residual bias after the settle
period below is a few mg; 5 mg is 0.049 m/s^2, giving 2.5 cm after 1 s and
2.5 m after 100 s.  Secondary to gravity leakage, but it never averages out.

Three mitigations are implemented, and together they turn "unusable in two
seconds" into "usable for a slow sweep of a room".  None of them repeal the
equation above:

1. **ZUPT (zero-velocity update).**  When the device is detected stationary,
   velocity is *forced* to zero rather than integrated.  This is the single
   most effective trick available: it stops error accumulating during the
   pauses that make up most of a handheld scan, and it caps the damage from
   any one motion at the length of that motion.  Set the device down for a
   moment every few seconds and the drift largely resets.

2. **Bias tracking.**  While stationary, whatever linear acceleration we still
   measure must be error, so it is fed back into a slowly-adapting bias
   estimate that is then subtracted.  This absorbs both true accelerometer
   bias and the slowly-varying part of the gravity leak.

3. **Velocity leak.**  Between stationary periods, velocity is bled toward
   zero with time constant ``vel_tau_s``.  This is an explicit lie -- it
   damps genuine sustained motion too -- but unbounded velocity error turns
   into unbounded *position* error, and bounding it is worth the bias against
   slow steady translation.  Set ``--ins-vel-tau 0`` to disable and watch the
   raw behaviour.

---------------------------------------------------------------------------
A SAMPLING PROBLEM YOU SHOULD KNOW ABOUT
---------------------------------------------------------------------------
The firmware runs the IMU at 1 kHz but sends orientation packets at 100 Hz
(``PACKET_DECIMATION = 10`` in ``main.c``), and the accelerometer value in the
packet is the *instantaneous* sample at that instant -- the other nine are
discarded, not averaged.  That is decimation without an anti-alias filter, so
vibration above 50 Hz folds down into the band being integrated, and the
LiDAR spindle at ~5.6 rev/s radiates plenty of it.

Aliased vibration does not average to zero once integrated; it looks like real
acceleration and walks the position estimate.  A low-pass filter is applied
here (``accel_lp_hz``) to suppress what is left, but filtering after aliasing
cannot recover what decimation already destroyed.  The real fix is in the
firmware -- accumulate the ten samples and send the mean -- which is noted in
the report accompanying this module and is NOT applied here, because it would
change the firmware that ``--cloud`` is also being compared against.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque

import numpy as np

from . import protocol as proto
from .cloud import (
    SENSOR_TO_BODY,
    MountConfig,
    VoxelCloud,
    body_to_earth,
    quat_to_matrix,
    ring_to_body,
)

G_MPS2 = 9.80665

# Beyond this the estimate has certainly diverged; clamping keeps a runaway
# from dragging the view scale to infinity and taking the cloud with it.
_POS_SANITY_M = 50.0


class INSConfig:
    """Tunables for the dead-reckoning filter.

    Defaults are chosen for a handheld sweep: someone holding the board,
    pausing every few seconds.  They are deliberately conservative -- the
    filter would rather under-report motion than run away.
    """

    __slots__ = ("zupt", "zupt_accel_g", "zupt_gyro_dps", "zupt_window_s",
                 "vel_tau_s", "bias_gain", "settle_s", "accel_lp_hz",
                 "max_dt_s", "noise_mg")

    def __init__(self,
                 zupt: bool = True,
                 zupt_accel_g: float = 0.04,
                 zupt_gyro_dps: float = 3.0,
                 zupt_window_s: float = 0.25,
                 vel_tau_s: float = 1.5,
                 bias_gain: float = 0.02,
                 settle_s: float = 1.0,
                 accel_lp_hz: float = 12.0,
                 max_dt_s: float = 0.05,
                 noise_mg: float = 0.0) -> None:
        self.zupt = bool(zupt)
        self.zupt_accel_g = float(zupt_accel_g)
        self.zupt_gyro_dps = float(zupt_gyro_dps)
        self.zupt_window_s = float(zupt_window_s)
        self.vel_tau_s = float(vel_tau_s)
        self.bias_gain = float(bias_gain)
        self.settle_s = float(settle_s)
        self.accel_lp_hz = float(accel_lp_hz)
        self.max_dt_s = float(max_dt_s)
        self.noise_mg = float(noise_mg)


class INSState:
    """A consistent copy of the estimator's outputs, for the UI thread."""

    __slots__ = ("pos_mm", "vel_mps", "accel_mps2", "bias_mps2", "static",
                 "settling", "samples", "static_frac", "path_mm",
                 "since_zupt_s", "zupt_count", "rejected")

    def __init__(self, **kw) -> None:
        for k in self.__slots__:
            setattr(self, k, kw.get(k))

    @property
    def speed_mps(self) -> float:
        return float(np.linalg.norm(self.vel_mps))

    @property
    def bias_mg(self) -> float:
        return float(np.linalg.norm(self.bias_mps2)) / G_MPS2 * 1000.0


class DeadReckoner:
    """Strapdown integrator: specific force -> velocity -> position.

    Fed from the reader thread (every orientation packet, ~100 Hz) and read
    from the UI thread (~12 Hz), so everything mutable sits behind a lock.
    Integrating on the UI thread instead would be wrong, not merely slow: at
    12 fps we would see one packet in eight and integrate a signal sampled far
    below the rate at which it changes.
    """

    def __init__(self, cfg: INSConfig | None = None, history: int = 4096) -> None:
        self.cfg = cfg or INSConfig()
        self._lock = threading.Lock()
        self._rng = np.random.default_rng(12345)
        self._hist_len = int(history)
        self.reset()

    # ------------------------------------------------------------------
    def reset(self) -> None:
        with self._lock:
            self._pos = np.zeros(3)          # metres, earth frame
            self._vel = np.zeros(3)
            self._acc = np.zeros(3)          # last linear accel, m/s^2
            self._acc_lp = np.zeros(3)       # low-passed specific force, earth
            self._bias = np.zeros(3)         # earth-frame accel bias estimate
            self._have_lp = False

            self._last_t: float | None = None
            self._t0: float | None = None
            self._wraps = 0
            self._last_raw: int | None = None

            self._static_win: deque[tuple[float, bool]] = deque()
            self._static = True
            self._n = 0
            self._n_static = 0
            self._zupts = 0
            self._rejected = 0
            self._path = 0.0
            self._last_zupt_t: float | None = None

            # (t_seconds, position) so a scan can be placed at the position
            # that held when *that revolution closed*, not the latest one.
            self._trail: deque[tuple[float, np.ndarray]] = deque(
                maxlen=self._hist_len)

    # ------------------------------------------------------------------
    def _unwrap(self, t_us: int) -> float:
        """Device microsecond counter -> monotonic seconds.

        Orientation and scan packets interleave, so a small backwards step is
        normal ordering jitter, not a wrap.  Only a jump of more than half the
        counter range is treated as a genuine 32-bit rollover.
        """
        if self._last_raw is not None and t_us < self._last_raw:
            if (self._last_raw - t_us) > (1 << 31):
                self._wraps += 1
        self._last_raw = max(t_us, self._last_raw or 0)
        return (t_us + self._wraps * (1 << 32)) / 1e6

    # ------------------------------------------------------------------
    def update(self, o: "proto.Orientation") -> None:
        """Fold in one orientation packet.  Called on the reader thread."""
        with self._lock:
            t = self._unwrap(o.t_us)

            if self._t0 is None:
                self._t0 = t
            if self._last_t is None:
                self._last_t = t
                self._trail.append((t, self._pos.copy()))
                return

            dt = t - self._last_t
            self._last_t = t

            # A dt outside this band means a dropped run of packets or a clock
            # glitch.  Integrating across it would inject a large fictitious
            # displacement, so the sample is counted and dropped.
            if not (1e-5 < dt < self.cfg.max_dt_s):
                self._rejected += 1
                self._trail.append((t, self._pos.copy()))
                return

            self._n += 1

            a_s = np.array(o.accel_g, dtype=float)
            if self.cfg.noise_mg > 0.0:
                # Deliberate corruption, for demonstrating drift on the
                # noise-free simulator.  Half fixed bias, half white noise --
                # the fixed part is what actually kills you.
                s = self.cfg.noise_mg / 1000.0
                a_s = a_s + s * 0.5 + self._rng.normal(0.0, s * 0.5, 3)

            g_s = np.array(o.gyro_dps, dtype=float)

            # --- stationary detection, in the sensor frame ----------------
            # Sensor frame on purpose: it needs no attitude estimate, so a
            # wrong quaternion cannot make a still device look moving.
            still = (abs(float(np.linalg.norm(a_s)) - 1.0) < self.cfg.zupt_accel_g
                     and float(np.linalg.norm(g_s)) < self.cfg.zupt_gyro_dps)
            self._static_win.append((t, still))
            while self._static_win and (t - self._static_win[0][0]) > self.cfg.zupt_window_s:
                self._static_win.popleft()
            # Every sample in the window must agree, so a device that is
            # merely between two strides is not mistaken for one at rest.
            window_full = (self._static_win
                           and (t - self._static_win[0][0]) >= self.cfg.zupt_window_s * 0.8)
            self._static = bool(window_full and all(s for _, s in self._static_win))
            if self._static:
                self._n_static += 1

            # --- gravity removal, in the earth frame ---------------------
            # Rotate first, subtract second.  Doing it the other way round
            # would subtract a constant from a rotating vector and leave a
            # residue that spins with the board.
            R = quat_to_matrix(o.q)
            a_e = R @ a_s
            lin = (a_e - np.array([0.0, 0.0, 1.0])) * G_MPS2

            # Single-pole low-pass, to knock down what the 100 Hz decimation
            # aliased into band.  alpha from the actual dt, so a jittery
            # packet rate does not change the corner frequency.
            if self.cfg.accel_lp_hz > 0.0:
                alpha = 1.0 - math.exp(-2.0 * math.pi * self.cfg.accel_lp_hz * dt)
                if not self._have_lp:
                    self._acc_lp = lin.copy()
                    self._have_lp = True
                else:
                    self._acc_lp += alpha * (lin - self._acc_lp)
                lin = self._acc_lp.copy()

            settling = (t - self._t0) < self.cfg.settle_s

            # --- bias tracking -------------------------------------------
            # While stationary the true linear acceleration is zero by
            # definition, so anything left after bias removal *is* bias error.
            # During the settle window adapt hard, to converge quickly.
            corrected = lin - self._bias
            if self._static or settling:
                gain = 0.25 if settling else self.cfg.bias_gain
                self._bias += gain * corrected
                corrected = lin - self._bias

            self._acc = corrected

            if settling:
                # Hold everything at zero until the bias estimate has settled;
                # integrating during convergence would bake the initial error
                # straight into the position.
                self._vel[:] = 0.0
                self._trail.append((t, self._pos.copy()))
                return

            # --- integrate ------------------------------------------------
            if self.cfg.zupt and self._static:
                self._vel[:] = 0.0
                self._zupts += 1
                self._last_zupt_t = t
            else:
                v_new = self._vel + corrected * dt
                if self.cfg.vel_tau_s > 0.0:
                    v_new *= math.exp(-dt / self.cfg.vel_tau_s)
                # Trapezoidal in velocity: with a linearly-varying velocity
                # this is exact, where forward Euler would accrue a systematic
                # half-step error every sample and integrate into a real drift.
                self._pos += 0.5 * (self._vel + v_new) * dt
                self._path += float(np.linalg.norm(0.5 * (self._vel + v_new) * dt))
                self._vel = v_new

                n = float(np.linalg.norm(self._pos))
                if n > _POS_SANITY_M:
                    self._pos *= _POS_SANITY_M / n
                    self._vel[:] = 0.0

            self._trail.append((t, self._pos.copy()))

    # ------------------------------------------------------------------
    def position_at(self, t_us: int) -> np.ndarray:
        """Position in MILLIMETRES when the device clock read ``t_us``.

        Linearly interpolated between the two bracketing samples: a scan
        packet closes at an arbitrary moment between two orientation packets,
        and at walking pace the position moves ~10 mm in that gap.
        """
        with self._lock:
            if not self._trail:
                return np.zeros(3)
            t = self._unwrap(t_us)
            first_t, first_p = self._trail[0]
            last_t, last_p = self._trail[-1]
            if t <= first_t:
                return first_p * 1000.0
            if t >= last_t:
                return last_p * 1000.0

            lo_t, lo_p = first_t, first_p
            for cur_t, cur_p in self._trail:
                if cur_t >= t:
                    span = cur_t - lo_t
                    if span <= 0.0:
                        return cur_p * 1000.0
                    f = (t - lo_t) / span
                    return (lo_p + (cur_p - lo_p) * f) * 1000.0
                lo_t, lo_p = cur_t, cur_p
            return last_p * 1000.0

    # ------------------------------------------------------------------
    def trail_mm(self, max_pts: int = 2000) -> np.ndarray:
        """Recent device path as (N, 3) millimetres, for drawing."""
        with self._lock:
            if not self._trail:
                return np.empty((0, 3))
            pts = np.array([p for _, p in self._trail], dtype=float) * 1000.0
        if pts.shape[0] > max_pts:
            step = int(np.ceil(pts.shape[0] / max_pts))
            pts = pts[::step]
        return pts

    def snapshot(self) -> INSState:
        with self._lock:
            since = (None if self._last_zupt_t is None or self._last_t is None
                     else self._last_t - self._last_zupt_t)
            return INSState(
                pos_mm=self._pos.copy() * 1000.0,
                vel_mps=self._vel.copy(),
                accel_mps2=self._acc.copy(),
                bias_mps2=self._bias.copy(),
                static=self._static,
                settling=(self._t0 is not None and self._last_t is not None
                          and (self._last_t - self._t0) < self.cfg.settle_s),
                samples=self._n,
                static_frac=(self._n_static / self._n) if self._n else 0.0,
                path_mm=self._path * 1000.0,
                since_zupt_s=since,
                zupt_count=self._zupts,
                rejected=self._rejected,
            )


def attach_dead_reckoner(source, dr: DeadReckoner):
    """Route every orientation packet through ``dr`` on the reader thread.

    Wraps the bound method on the *instance*, so ``reader.py`` needs no edit
    and the plain ``--cloud`` path is bit-for-bit unaffected.  Returns a
    callable that removes the hook again.
    """
    original = source._handle_orientation

    def hooked(o):
        try:
            dr.update(o)
        except Exception:
            # The estimator must never be able to take the link down; a bad
            # sample is worth losing, the stream is not.
            pass
        original(o)

    source._handle_orientation = hooked

    def detach():
        source._handle_orientation = original

    return detach


# ===========================================================================
# Reconstruction
# ===========================================================================
def ring_to_world_at(scan: "proto.Scan", mount: MountConfig,
                     origin_mm: np.ndarray) -> np.ndarray:
    """One revolution -> (N, 3) earth-frame points, offset to ``origin_mm``.

    Identical to :func:`lidarscan.cloud.ring_to_world` except for the final
    translation.  Rotation is applied first and the offset added afterwards:
    the origin is where the *sensor* was, so it must not be rotated.
    """
    body = ring_to_body(scan, mount)
    if body.shape[0] == 0:
        return body
    return body @ body_to_earth(scan.q).T + np.asarray(origin_mm, dtype=float)


class INSCloudBuilder:
    """Accumulates a cloud using the estimated position of each revolution."""

    def __init__(self, mount: MountConfig, dr: DeadReckoner,
                 voxel_mm: float = 50.0, max_points: int = 60000) -> None:
        self.mount = mount
        self.dr = dr
        self.cloud = VoxelCloud(voxel_mm, max_points)
        self.paused = False
        self.hold_origin = False     # freeze translation, for an in-app A/B
        self.rings_used = 0
        self._last_seq: int | None = None
        self.last_ring_world: np.ndarray = np.empty((0, 3))
        self.last_origin_mm: np.ndarray = np.zeros(3)

    def consume(self, state) -> bool:
        scan = getattr(state, "latest_scan", None)
        if scan is None or scan.seq == self._last_seq:
            return False
        self._last_seq = scan.seq

        origin = (np.zeros(3) if self.hold_origin
                  else self.dr.position_at(scan.t_us))
        self.last_origin_mm = origin

        world = ring_to_world_at(scan, self.mount, origin)
        self.last_ring_world = world

        if self.paused or world.shape[0] == 0:
            return False

        self.rings_used += 1
        return self.cloud.add(world) > 0


# ===========================================================================
# Interactive view
# ===========================================================================
_AXIS_COLOURS = ("#d62728", "#2ca02c", "#1f77b4")   # forward / left / up


class INSCloudView:
    """Live 3D cloud with the device path drawn, plus INS tuning controls."""

    def __init__(self, builder: INSCloudBuilder, on_command=None,
                 range_m: float = 6.0) -> None:
        import matplotlib.pyplot as plt

        self.builder = builder
        self.dr = builder.dr
        self._on_command = on_command
        self._range_mm = range_m * 1000.0
        self._msg = ""

        self.fig = plt.figure(figsize=(15.5, 8.6))
        gs = self.fig.add_gridspec(1, 2, width_ratios=[1.85, 1.0],
                                   left=0.01, right=0.985, top=0.94,
                                   bottom=0.18, wspace=0.06)

        self.ax = self.fig.add_subplot(gs[0, 0], projection="3d")
        self._setup_axes()

        self.ax_txt = self.fig.add_subplot(gs[0, 1])
        self.ax_txt.axis("off")
        self.txt = self.ax_txt.text(0.0, 1.0, "", transform=self.ax_txt.transAxes,
                                    va="top", ha="left", family="monospace",
                                    fontsize=8.4)

        self._setup_controls()
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.fig.suptitle(
            "Mini-LiDAR-Board  --  3D reconstruction with dead reckoning",
            fontsize=13, y=0.975)

    # ------------------------------------------------------------------
    def _setup_axes(self) -> None:
        ax = self.ax
        ax.set_facecolor("white")
        ax.set_xlabel("earth X  (m)", fontsize=8, labelpad=-4)
        ax.set_ylabel("earth Y  (m)", fontsize=8, labelpad=-4)
        ax.set_zlabel("earth Z up (m)", fontsize=8, labelpad=-4)
        ax.tick_params(labelsize=7)

        self.sc = ax.scatter([], [], [], s=1.6, c=[], cmap="turbo",
                             vmin=-1.0, vmax=1.0, depthshade=False,
                             linewidths=0)
        self.sc_live = ax.scatter([], [], [], s=4.0, c="#111111",
                                  depthshade=False, linewidths=0, alpha=0.55)

        # The estimated device path -- the whole point of this variant, and
        # the first thing to look at when the cloud comes out wrong.
        self.path_line = ax.plot([], [], [], color="#ff7f0e", lw=1.4,
                                 alpha=0.9)[0]
        self.axis_lines = [
            ax.plot([0, 0], [0, 0], [0, 0], color=c, lw=2.0)[0]
            for c in _AXIS_COLOURS
        ]
        self.origin = ax.plot([0], [0], [0], marker="o", ms=5.0,
                              color="#ff7f0e")[0]
        self._apply_range()

    def _apply_range(self) -> None:
        r = self._range_mm / 1000.0
        self.ax.set_xlim(-r, r)
        self.ax.set_ylim(-r, r)
        self.ax.set_zlim(-r, r)
        self.ax.set_box_aspect((1, 1, 1))
        self.sc.set_clim(-r * 0.6, r * 0.6)

    # ------------------------------------------------------------------
    def _setup_controls(self) -> None:
        from matplotlib.widgets import Button, Slider

        ax_spin = self.fig.add_axes([0.06, 0.105, 0.24, 0.024])
        ax_dec = self.fig.add_axes([0.06, 0.072, 0.24, 0.024])
        ax_vox = self.fig.add_axes([0.06, 0.039, 0.24, 0.024])
        ax_tau = self.fig.add_axes([0.44, 0.105, 0.20, 0.024])
        ax_zac = self.fig.add_axes([0.44, 0.072, 0.20, 0.024])
        ax_lp = self.fig.add_axes([0.44, 0.039, 0.20, 0.024])

        cfg = self.dr.cfg
        self.s_spin = Slider(ax_spin, "spin  ", 0, proto.MOTOR_PERMILLE_MAX,
                             valinit=proto.MOTOR_PERMILLE_DEFAULT, valstep=10,
                             color="#4C72B0")
        self.s_dec = Slider(ax_dec, "keep 1 in  ", proto.DECIM_MIN,
                            proto.DECIM_MAX, valinit=1, valstep=1,
                            color="#55A868")
        self.s_vox = Slider(ax_vox, "voxel mm  ", 10, 200,
                            valinit=self.builder.cloud.voxel_mm, valstep=5,
                            color="#8172B2")
        self.s_tau = Slider(ax_tau, "vel leak s  ", 0.0, 10.0,
                            valinit=cfg.vel_tau_s, valstep=0.25,
                            color="#C44E52")
        self.s_zac = Slider(ax_zac, "ZUPT g  ", 0.005, 0.20,
                            valinit=cfg.zupt_accel_g, valstep=0.005,
                            color="#DA8BC3")
        self.s_lp = Slider(ax_lp, "accel LP Hz  ", 0.0, 50.0,
                           valinit=cfg.accel_lp_hz, valstep=1.0,
                           color="#937860")

        for sl in (self.s_spin, self.s_dec, self.s_vox, self.s_tau,
                   self.s_zac, self.s_lp):
            sl.label.set_fontsize(8)
            sl.valtext.set_fontsize(8)

        self.s_spin.on_changed(lambda v: self._send(proto.cmd_set_motor(int(v))))
        self.s_dec.on_changed(
            lambda v: self._send(proto.cmd_set_decimation(int(v))))
        self.s_vox.on_changed(lambda v: self.builder.cloud.set_voxel_mm(float(v)))
        self.s_tau.on_changed(lambda v: setattr(cfg, "vel_tau_s", float(v)))
        self.s_zac.on_changed(lambda v: setattr(cfg, "zupt_accel_g", float(v)))
        self.s_lp.on_changed(lambda v: setattr(cfg, "accel_lp_hz", float(v)))

        mk = lambda x, w=0.072: self.fig.add_axes([x, 0.10, w, 0.038])
        self.b_pause = Button(mk(0.695), "pause", color="0.9", hovercolor="0.8")
        self.b_clear = Button(mk(0.775), "clear", color="0.9", hovercolor="0.8")
        self.b_save = Button(mk(0.855, 0.085), "save PLY", color="0.9",
                             hovercolor="0.8")
        self.b_zero = Button(self.fig.add_axes([0.695, 0.052, 0.072, 0.038]),
                             "re-zero", color="0.9", hovercolor="0.8")
        self.b_hold = Button(self.fig.add_axes([0.775, 0.052, 0.072, 0.038]),
                             "hold pos", color="0.9", hovercolor="0.8")
        self.b_zupt = Button(self.fig.add_axes([0.855, 0.052, 0.085, 0.038]),
                             "ZUPT on", color="0.9", hovercolor="0.8")
        for b in (self.b_pause, self.b_clear, self.b_save, self.b_zero,
                  self.b_hold, self.b_zupt):
            b.label.set_fontsize(8)

        self.b_pause.on_clicked(lambda _e: self._toggle_pause())
        self.b_clear.on_clicked(lambda _e: self._clear())
        self.b_save.on_clicked(lambda _e: self._save())
        self.b_zero.on_clicked(lambda _e: self._rezero())
        self.b_hold.on_clicked(lambda _e: self._toggle_hold())
        self.b_zupt.on_clicked(lambda _e: self._toggle_zupt())

        self.fig.text(0.06, 0.010,
                      "drag orbit | scroll zoom | keys: [p]ause [c]lear "
                      "[s]ave [r]eset view [z]ero pos [h]old pos [u] ZUPT",
                      fontsize=7.6, color="0.35")

    # -- actions -----------------------------------------------------------
    def _send(self, frame: bytes) -> None:
        if self._on_command is not None:
            self._on_command(frame)

    def _toggle_pause(self) -> None:
        self.builder.paused = not self.builder.paused
        self.b_pause.label.set_text("resume" if self.builder.paused else "pause")
        self._msg = "accumulation paused" if self.builder.paused else ""

    def _clear(self) -> None:
        self.builder.cloud.clear()
        self.builder.rings_used = 0
        self._msg = "cloud cleared"

    def _rezero(self) -> None:
        self.dr.reset()
        self._msg = "position estimate re-zeroed; hold still to settle"

    def _toggle_hold(self) -> None:
        self.builder.hold_origin = not self.builder.hold_origin
        self.b_hold.label.set_text(
            "use pos" if self.builder.hold_origin else "hold pos")
        self._msg = ("translation frozen -- this is what --cloud does"
                     if self.builder.hold_origin else "translation live")

    def _toggle_zupt(self) -> None:
        self.dr.cfg.zupt = not self.dr.cfg.zupt
        self.b_zupt.label.set_text(
            "ZUPT on" if self.dr.cfg.zupt else "ZUPT off")
        self._msg = f"ZUPT {'enabled' if self.dr.cfg.zupt else 'DISABLED'}"

    def _save(self) -> None:
        path = time.strftime("scan-ins-%Y%m%d-%H%M%S.ply")
        n = self.builder.cloud.save_ply(path)
        self._msg = f"wrote {n} points to {path}"

    def _on_key(self, event) -> None:
        if event.key == "p":
            self._toggle_pause()
        elif event.key == "c":
            self._clear()
        elif event.key == "s":
            self._save()
        elif event.key == "r":
            self.ax.view_init(elev=22, azim=-60)
        elif event.key == "z":
            self._rezero()
        elif event.key == "h":
            self._toggle_hold()
        elif event.key == "u":
            self._toggle_zupt()

    # ------------------------------------------------------------------
    def update(self, state, parser_stats) -> None:
        self.builder.consume(state)

        pts = self.builder.cloud.xyz()
        if pts.shape[0]:
            m = pts / 1000.0
            self.sc._offsets3d = (m[:, 0], m[:, 1], m[:, 2])
            self.sc.set_array(m[:, 2])
        else:
            self.sc._offsets3d = ([], [], [])
            self.sc.set_array(np.array([]))

        live = self.builder.last_ring_world
        if live.shape[0]:
            lm = live / 1000.0
            self.sc_live._offsets3d = (lm[:, 0], lm[:, 1], lm[:, 2])
        else:
            self.sc_live._offsets3d = ([], [], [])

        trail = self.dr.trail_mm() / 1000.0
        if trail.shape[0] > 1:
            self.path_line.set_data_3d(trail[:, 0], trail[:, 1], trail[:, 2])
        else:
            self.path_line.set_data_3d([], [], [])

        # Triad and origin marker ride along with the device.
        ins = self.dr.snapshot()
        p = (np.zeros(3) if self.builder.hold_origin
             else ins.pos_mm / 1000.0)
        self.origin.set_data_3d([p[0]], [p[1]], [p[2]])

        scan = getattr(state, "latest_scan", None)
        o = state.latest
        q = scan.q if scan is not None else (o.q if o is not None else (1, 0, 0, 0))
        R = body_to_earth(q)
        length = self._range_mm / 1000.0 * 0.22
        for i, line in enumerate(self.axis_lines):
            v = R[:, i] * length
            line.set_data_3d([p[0], p[0] + v[0]], [p[1], p[1] + v[1]],
                             [p[2], p[2] + v[2]])

        self.txt.set_text(self._format(state, parser_stats, ins))

    def artists(self):
        return (self.sc, self.sc_live, self.txt, self.origin, self.path_line,
                *self.axis_lines)

    # ------------------------------------------------------------------
    def _format(self, state, ps, ins: INSState) -> str:
        c = self.builder.cloud
        lines = [f"source          : {state.source_name}"]

        if state.error:
            return "\n".join(lines + [""] + ["ERROR: " + ln
                                             for ln in state.error.splitlines()])

        o = state.latest
        scan = getattr(state, "latest_scan", None)
        if o is None and scan is None:
            return "\n".join(lines + ["waiting for the first packet..."])

        lines += [
            f"link      {state.packet_rate_hz:6.1f} Hz    "
            f"loss {state.packet_loss_pct:5.2f} %   "
            f"CRC err {ps.crc_errors}",
            "",
        ]

        if scan is not None:
            lines += [
                f"lidar   {state.scan_rate_hz:5.1f} rings/s   "
                f"{scan.point_count} pts/ring, {state.scan_seq_gaps} missed",
                f"spin    {scan.rot_hz:5.1f} rev/s     "
                f"duty {scan.motor_permille}/1000",
            ]
        else:
            lines.append("lidar   no rings yet")

        if o is not None:
            from .reader import quat_to_euler_deg
            roll, pitch, yaw = quat_to_euler_deg(o.q)
            lines.append(f"attitude  r {roll:+7.1f}  p {pitch:+7.1f}  "
                         f"y {yaw:+7.1f}  deg")

        p, v = ins.pos_mm / 1000.0, ins.vel_mps
        if ins.settling:
            phase = "SETTLING (hold still)"
        elif self.builder.hold_origin:
            phase = "HELD AT ORIGIN"
        elif ins.static:
            phase = "static -- ZUPT holding"
        else:
            phase = "MOVING -- integrating"

        since = ("never" if ins.since_zupt_s is None
                 else f"{ins.since_zupt_s:.1f} s ago")

        lines += [
            "",
            "--- dead reckoning ------------------------",
            f"state   {phase}",
            f"pos     x {p[0]:+6.2f}  y {p[1]:+6.2f}  z {p[2]:+6.2f}  m",
            f"vel     {ins.speed_mps:5.2f} m/s   "
            f"path {ins.path_mm / 1000.0:6.2f} m",
            f"bias    {ins.bias_mg:5.1f} mg     "
            f"({ins.bias_mg * G_MPS2 / 1000.0:.3f} m/s^2)",
            f"ZUPT    {'on ' if self.dr.cfg.zupt else 'OFF'}  "
            f"{ins.zupt_count} applied, last {since}",
            f"        still {100.0 * ins.static_frac:4.1f} % of "
            f"{ins.samples} samples",
            f"leak    tau {self.dr.cfg.vel_tau_s:.2f} s   "
            f"LP {self.dr.cfg.accel_lp_hz:.0f} Hz   "
            f"dropped dt {ins.rejected}",
        ]

        # Error budget from the numbers actually in front of us, so the
        # magnitude of the doubt is visible rather than folklore.
        if ins.since_zupt_s:
            t = ins.since_zupt_s
            e_bias = 0.5 * (ins.bias_mg / 1000.0 * G_MPS2) * t * t
            e_tilt = 0.5 * (G_MPS2 * math.sin(math.radians(1.0))) * t * t
            lines += [
                f"budget  {t:.1f} s since ZUPT ->",
                f"        ~{e_bias:.2f} m from residual bias",
                f"        ~{e_tilt:.2f} m from 1 deg of tilt error",
            ]

        lines += [
            "",
            f"cloud   {len(c):7d} points  "
            f"({'PAUSED' if self.builder.paused else 'live'})",
            f"        {c.total_added} raw -> {len(c)} kept, "
            f"{self.builder.rings_used} rings",
            "",
            "position is DOUBLE-INTEGRATED and drifts as t^2.",
            "pause often: every still moment resets velocity.",
            "'hold pos' freezes translation = what --cloud does.",
        ]

        if self._msg:
            lines += ["", self._msg]
        return "\n".join(lines)


def run_cloud_ins(source, builder: INSCloudBuilder, fps: float = 12.0,
                  duration: float | None = None, save_path: str | None = None,
                  headless: bool = False, range_m: float = 6.0,
                  save_ply: str | None = None) -> None:
    """Start ``source`` with dead reckoning attached and animate until close."""
    import matplotlib

    if headless:
        matplotlib.use("Agg", force=True)

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    view = INSCloudView(builder, on_command=getattr(source, "send", None),
                        range_m=range_m)

    # Attach BEFORE start(), so not one packet is integrated-from-missing.
    detach = attach_dead_reckoner(source, builder.dr)
    source.start()

    frames_total = None if duration is None else max(1, int(duration * fps))

    def step(_frame):
        view.update(source.snapshot(), source.parser.stats)
        return view.artists()

    anim = FuncAnimation(view.fig, step, frames=frames_total,
                         interval=1000.0 / fps, blit=False,
                         cache_frame_data=False, repeat=False)

    try:
        if headless:
            start = time.monotonic()
            deadline = start + (duration if duration is not None else 5.0)
            i = 0
            while time.monotonic() < deadline:
                step(i)
                i += 1
                time.sleep(max(0.0, (start + (i + 1) / fps) - time.monotonic()))
            view.fig.canvas.draw()
            if save_path:
                view.fig.savefig(save_path, dpi=110)
                print(f"wrote {save_path}  ({i} frames rendered)")
        else:
            _ = anim
            plt.show()
    finally:
        source.stop()
        detach()
        if save_ply:
            n = builder.cloud.save_ply(save_ply)
            print(f"wrote {n} points to {save_ply}")
        plt.close(view.fig)
