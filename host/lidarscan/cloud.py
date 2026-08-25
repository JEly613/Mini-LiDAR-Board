"""3D reconstruction: rotate each LiDAR ring into world space and accumulate.

---------------------------------------------------------------------------
FRAMES AND THE TRANSFORM CHAIN
---------------------------------------------------------------------------
There are four frames in play.  Getting them straight is the whole job; once
the chain is right the rest is plumbing.

**Sensor frame (S)** -- raw ICM-42688-P axes.  This is what the Madgwick
filter integrates, so the device's quaternion is expressed in terms of it.
The IMU is mounted on the *bottom* of the PCB, which is why the board reads
roll ~= 180 deg lying flat.

**Body frame (B)** -- the board, in a conventional right-handed layout:
X forward, Y left, Z up.  Per the agreed convention, +Y_s is forward and
-Z_s is up, which fixes the mapping completely::

    X_b = +Y_s        (forward)
    Z_b = -Z_s        (up)
    Y_b = Z_b x X_b = +X_s   (left)

    M = [[0, 1,  0],
         [1, 0,  0],
         [0, 0, -1]]        v_body = M @ v_sensor

det(M) = +1, so it is a proper rotation and not a mirror.  M is also its own
inverse, which is a happy accident of this particular convention.

**Earth frame (E)** -- Madgwick's output frame, Z up.  The filter drives its
gravity estimate to earth +Z, so Z is up by construction.  X and Y are
unconstrained: with no magnetometer, yaw is free and drifts.

**LiDAR frame (L)** -- the A1's scan plane.  The A1 is a separate module on a
cable, so its orientation relative to the PCB is a *mechanical* fact this code
cannot know.  The default assumes the sensible mounting -- spin axis along
Z_b, 0 deg pointing along X_b (forward), angle advancing clockwise seen from
above -- and exposes a yaw offset and a sweep-direction flip for when the real
mounting differs.

The chain, for a measurement at angle theta and range d::

    p_body  = R_mount @ (d cos, d sin, 0)
    p_earth = R_es @ M @ p_body            (R_es = quat_to_matrix(q))

Define R_eb = R_es @ M once per ring and the per-point work is one 3xN matmul.

---------------------------------------------------------------------------
WHAT THIS DELIBERATELY DOES NOT DO
---------------------------------------------------------------------------
No translation.  The device is assumed to rotate in place, so the cloud is
centred on the device at the origin.  Double-integrating the accelerometer
would add position, and it would also add metres of drift within seconds --
which is exactly why it is out of scope here.

Consequence worth remembering while looking at the result: yaw has no absolute
reference, so a long session smears the cloud around the vertical axis.  Sweep
and look; do not leave it accumulating for ten minutes and expect crisp walls.
"""

from __future__ import annotations

import time

import numpy as np

from . import protocol as proto

# Sensor -> body.  See the module docstring for the derivation.
SENSOR_TO_BODY = np.array([
    [0.0, 1.0,  0.0],
    [1.0, 0.0,  0.0],
    [0.0, 0.0, -1.0],
])


def quat_to_matrix(q) -> np.ndarray:
    """(w, x, y, z) unit quaternion -> 3x3 rotation matrix (sensor -> earth)."""
    w, x, y, z = q
    n = float(np.sqrt(w * w + x * x + y * y + z * z))
    if n < 1e-9:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


def body_to_earth(q) -> np.ndarray:
    """R_eb: rotates a BODY-frame vector into the earth frame.

    Its columns are the board's forward / left / up axes expressed in earth
    coordinates, which is exactly what the on-screen attitude triad needs.
    """
    return quat_to_matrix(q) @ SENSOR_TO_BODY


class MountConfig:
    """Where the LiDAR sits relative to the board.

    ``yaw_deg`` rotates the scan about the board's up axis: increase it until
    a landmark you know is dead ahead appears at the front of the cloud.
    ``clockwise`` matches the A1 as normally mounted (angle advancing clockwise
    seen from above); flip it if the cloud comes out mirrored.
    """

    __slots__ = ("yaw_deg", "clockwise")

    def __init__(self, yaw_deg: float = 0.0, clockwise: bool = True) -> None:
        self.yaw_deg = float(yaw_deg)
        self.clockwise = bool(clockwise)


def ring_to_body(scan: "proto.Scan", mount: MountConfig) -> np.ndarray:
    """One revolution -> (N, 3) points in the BODY frame, millimetres.

    Points with no return are dropped here rather than being carried along as
    zeros; a zero range is 'nothing came back on this ray', not 'a surface at
    the sensor', and keeping them would draw a solid blob at the origin.
    """
    valid = scan.valid_mask
    ang = scan.angles_deg[valid].astype(np.float64)
    d = scan.dists_mm[valid].astype(np.float64)

    if ang.size == 0:
        return np.empty((0, 3))

    sign = -1.0 if mount.clockwise else 1.0
    th = np.radians(sign * ang + mount.yaw_deg)

    pts = np.empty((ang.size, 3))
    pts[:, 0] = d * np.cos(th)
    pts[:, 1] = d * np.sin(th)
    pts[:, 2] = 0.0            # the ring is planar in the board's frame
    return pts


def ring_to_world(scan: "proto.Scan", mount: MountConfig) -> np.ndarray:
    """One revolution -> (N, 3) points in the EARTH frame, millimetres.

    Uses the quaternion carried in the scan packet itself -- the attitude at
    the instant that revolution closed -- rather than whatever the latest
    orientation packet happens to say.  That pairing is the whole reason the
    firmware puts a quaternion in every scan packet.
    """
    body = ring_to_body(scan, mount)
    if body.shape[0] == 0:
        return body
    return body @ body_to_earth(scan.q).T


class VoxelCloud:
    """Accumulated cloud, thinned onto a voxel grid and bounded in size.

    Two problems solved at once.  A raw stream at ~2000 points/s buries any
    plotting library within a minute, and repeated sweeps of the same wall pile
    thousands of near-duplicate points onto the same square centimetre.
    Keeping at most one point per voxel fixes both: the cloud stops growing
    once the room is covered, and what is left is an even sampling of the
    surfaces rather than a density map of where the operator pointed longest.

    Storage is a preallocated array plus a key->slot dict, so both insertion
    and eviction are O(1) and ``xyz()`` is a view rather than a rebuild.  When
    the budget is exhausted the oldest slot is reused, which quietly turns the
    cloud into a rolling window of the recent past.
    """

    _OFFSET = 1 << 20          # keeps voxel indices non-negative before packing
    _BITS = 21

    def __init__(self, voxel_mm: float = 50.0, max_points: int = 60000) -> None:
        self.voxel_mm = float(voxel_mm)
        self.max_points = int(max_points)
        self._xyz = np.zeros((self.max_points, 3), dtype=np.float32)
        self._keys = np.zeros(self.max_points, dtype=np.int64)
        self._slot: dict[int, int] = {}
        self._n = 0
        self._write = 0
        self.total_added = 0       # raw points offered, before thinning

    # -- geometry ---------------------------------------------------------
    def _pack(self, pts: np.ndarray) -> np.ndarray:
        idx = np.floor(pts / self.voxel_mm).astype(np.int64) + self._OFFSET
        np.clip(idx, 0, (1 << self._BITS) - 1, out=idx)
        return (idx[:, 0]
                | (idx[:, 1] << self._BITS)
                | (idx[:, 2] << (2 * self._BITS)))

    def add(self, pts: np.ndarray) -> int:
        """Add (N, 3) points; returns how many became new voxels."""
        if pts.shape[0] == 0:
            return 0
        self.total_added += pts.shape[0]

        packed = self._pack(pts)
        # Collapse duplicates inside this batch first, keeping one representative
        # point per voxel, so the dict only sees each key once.
        uniq, first = np.unique(packed, return_index=True)

        added = 0
        for key, src in zip(uniq.tolist(), first.tolist()):
            if key in self._slot:
                continue           # already have this voxel; first sample wins
            if self._n < self.max_points:
                idx = self._n
                self._n += 1
            else:
                idx = self._write
                self._write = (self._write + 1) % self.max_points
                del self._slot[int(self._keys[idx])]
            self._xyz[idx] = pts[src]
            self._keys[idx] = key
            self._slot[key] = idx
            added += 1
        return added

    # -- access -----------------------------------------------------------
    def xyz(self) -> np.ndarray:
        return self._xyz[:self._n]

    def __len__(self) -> int:
        return self._n

    def clear(self) -> None:
        self._slot.clear()
        self._n = 0
        self._write = 0
        self.total_added = 0

    def set_voxel_mm(self, mm: float) -> None:
        """Change the grid size.  Existing points are re-binned rather than
        thrown away, so tightening the grid does not blank the view."""
        pts = self.xyz().copy()
        self.voxel_mm = float(mm)
        self._slot.clear()
        self._n = 0
        self._write = 0
        kept = self.total_added
        self.add(pts)
        self.total_added = kept

    # -- export -----------------------------------------------------------
    def save_ply(self, path: str) -> int:
        """Write a binary little-endian PLY with per-point colour by height.

        PLY because MeshLab, CloudCompare and Blender all open it without
        conversion, and binary because an ASCII cloud of this size is
        needlessly large and slow to load.
        """
        pts = self.xyz()
        n = pts.shape[0]
        rgb = _height_colours(pts[:, 2]) if n else np.zeros((0, 3), np.uint8)

        rec = np.empty(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                 ("red", "u1"), ("green", "u1"), ("blue", "u1")])
        if n:
            # Metres in the file: millimetres are convenient on the wire but
            # every mesh tool assumes metres.
            rec["x"] = pts[:, 0] / 1000.0
            rec["y"] = pts[:, 1] / 1000.0
            rec["z"] = pts[:, 2] / 1000.0
            rec["red"], rec["green"], rec["blue"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]

        header = (
            "ply\n"
            "format binary_little_endian 1.0\n"
            "comment generated by lidarscan (rotation-only reconstruction)\n"
            f"element vertex {n}\n"
            "property float x\nproperty float y\nproperty float z\n"
            "property uchar red\nproperty uchar green\nproperty uchar blue\n"
            "end_header\n"
        ).encode("ascii")

        with open(path, "wb") as fh:
            fh.write(header)
            fh.write(rec.tobytes())
        return n


def _height_colours(z: np.ndarray) -> np.ndarray:
    """Height -> RGB, cool low / warm high, without pulling in a colormap."""
    if z.size == 0:
        return np.zeros((0, 3), np.uint8)
    lo, hi = float(np.min(z)), float(np.max(z))
    t = np.zeros_like(z) if hi - lo < 1e-6 else (z - lo) / (hi - lo)
    rgb = np.empty((z.size, 3), np.uint8)
    rgb[:, 0] = np.clip(255 * t, 0, 255)
    rgb[:, 1] = np.clip(255 * (1.0 - np.abs(t - 0.5) * 2.0), 0, 255)
    rgb[:, 2] = np.clip(255 * (1.0 - t), 0, 255)
    return rgb


class CloudBuilder:
    """Consumes scans from a source and keeps the cloud up to date.

    Kept separate from the view so the maths can be tested, and driven from
    the UI thread rather than the reader thread: rotating a ring is tens of
    microseconds of numpy, and doing it here keeps all mutation of the cloud on
    one thread with no locking.
    """

    def __init__(self, mount: MountConfig, voxel_mm: float = 50.0,
                 max_points: int = 60000) -> None:
        self.mount = mount
        self.cloud = VoxelCloud(voxel_mm, max_points)
        self.paused = False
        self.rings_used = 0
        self._last_seq: int | None = None
        self.last_ring_world: np.ndarray = np.empty((0, 3))

    def consume(self, state) -> bool:
        """Fold in the newest scan if it is one we have not seen.  Returns
        True when the cloud changed."""
        scan = getattr(state, "latest_scan", None)
        if scan is None or scan.seq == self._last_seq:
            return False
        self._last_seq = scan.seq

        world = ring_to_world(scan, self.mount)
        self.last_ring_world = world

        if self.paused or world.shape[0] == 0:
            return False

        self.rings_used += 1
        return self.cloud.add(world) > 0


# ===========================================================================
# Interactive view
# ===========================================================================
_AXIS_COLOURS = ("#d62728", "#2ca02c", "#1f77b4")   # forward / left / up
_AXIS_NAMES = ("fwd", "left", "up")


class CloudView:
    """Live 3D cloud with mouse orbit and keyboard/widget controls.

    Drawn with matplotlib because it is what this project already depends on
    and it gives mouse orbit for free.  It is not a point-cloud engine, so the
    voxel budget above is what keeps the frame rate usable; past roughly
    100k points a redraw costs more than the frame interval and the view goes
    sticky.
    """

    def __init__(self, builder: "CloudBuilder", on_command=None,
                 range_m: float = 6.0) -> None:
        import matplotlib.pyplot as plt

        self.builder = builder
        self._on_command = on_command
        self._range_mm = range_m * 1000.0
        self._saved_path: str | None = None
        self._msg = ""

        self.fig = plt.figure(figsize=(15.5, 8.6))
        gs = self.fig.add_gridspec(1, 2, width_ratios=[1.85, 1.0],
                                   left=0.01, right=0.985, top=0.94,
                                   bottom=0.16, wspace=0.06)

        self.ax = self.fig.add_subplot(gs[0, 0], projection="3d")
        self._setup_axes()

        self.ax_txt = self.fig.add_subplot(gs[0, 1])
        self.ax_txt.axis("off")
        self.txt = self.ax_txt.text(0.0, 1.0, "", transform=self.ax_txt.transAxes,
                                    va="top", ha="left", family="monospace",
                                    fontsize=8.6)

        self._setup_controls()
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.fig.suptitle("Mini-LiDAR-Board  --  live 3D reconstruction",
                          fontsize=13, y=0.975)

    # ------------------------------------------------------------------
    def _setup_axes(self) -> None:
        ax = self.ax
        ax.set_facecolor("white")
        ax.set_xlabel("earth X  (m)", fontsize=8, labelpad=-4)
        ax.set_ylabel("earth Y  (m)", fontsize=8, labelpad=-4)
        ax.set_zlabel("earth Z up (m)", fontsize=8, labelpad=-4)
        ax.tick_params(labelsize=7)

        # The accumulated cloud, coloured by height.
        self.sc = ax.scatter([], [], [], s=1.6, c=[], cmap="turbo",
                             vmin=-1.0, vmax=1.0, depthshade=False,
                             linewidths=0)

        # The single most recent revolution, drawn on top so the live sweep is
        # visible against everything already collected.
        self.sc_live = ax.scatter([], [], [], s=4.0, c="#111111",
                                  depthshade=False, linewidths=0, alpha=0.55)

        # Device attitude triad at the origin, so 'which way is the board
        # facing' is answerable at a glance.
        self.axis_lines = [
            ax.plot([0, 0], [0, 0], [0, 0], color=c, lw=2.0)[0]
            for c in _AXIS_COLOURS
        ]
        self.origin = ax.plot([0], [0], [0], marker="o", ms=4.5,
                              color="black")[0]
        self._apply_range()

    def _apply_range(self) -> None:
        r = self._range_mm / 1000.0
        self.ax.set_xlim(-r, r)
        self.ax.set_ylim(-r, r)
        self.ax.set_zlim(-r, r)
        # Equal aspect: without this a cube of data is drawn as a slab and
        # every angle you read off the screen is wrong.
        self.ax.set_box_aspect((1, 1, 1))
        self.sc.set_clim(-r * 0.6, r * 0.6)

    # ------------------------------------------------------------------
    def _setup_controls(self) -> None:
        from matplotlib.widgets import Button, Slider

        ax_spin = self.fig.add_axes([0.06, 0.085, 0.26, 0.026])
        ax_dec = self.fig.add_axes([0.06, 0.048, 0.26, 0.026])
        ax_vox = self.fig.add_axes([0.44, 0.085, 0.22, 0.026])
        ax_rng = self.fig.add_axes([0.44, 0.048, 0.22, 0.026])

        self.s_spin = Slider(ax_spin, "spin  ", 0, proto.MOTOR_PERMILLE_MAX,
                             valinit=proto.MOTOR_PERMILLE_DEFAULT, valstep=10,
                             color="#4C72B0")
        self.s_dec = Slider(ax_dec, "keep 1 in  ", proto.DECIM_MIN,
                            proto.DECIM_MAX, valinit=1, valstep=1,
                            color="#55A868")
        self.s_vox = Slider(ax_vox, "voxel mm  ", 10, 200,
                            valinit=self.builder.cloud.voxel_mm, valstep=5,
                            color="#8172B2")
        self.s_rng = Slider(ax_rng, "range m  ", 1, 12,
                            valinit=self._range_mm / 1000.0, valstep=0.5,
                            color="#CCB974")

        for sl in (self.s_spin, self.s_dec, self.s_vox, self.s_rng):
            sl.label.set_fontsize(8)
            sl.valtext.set_fontsize(8)

        self.s_spin.on_changed(
            lambda v: self._send(proto.cmd_set_motor(int(v))))
        self.s_dec.on_changed(
            lambda v: self._send(proto.cmd_set_decimation(int(v))))
        self.s_vox.on_changed(lambda v: self.builder.cloud.set_voxel_mm(float(v)))
        self.s_rng.on_changed(self._range_changed)

        self.b_pause = Button(self.fig.add_axes([0.72, 0.085, 0.075, 0.04]),
                              "pause", color="0.9", hovercolor="0.8")
        self.b_clear = Button(self.fig.add_axes([0.805, 0.085, 0.075, 0.04]),
                              "clear", color="0.9", hovercolor="0.8")
        self.b_save = Button(self.fig.add_axes([0.89, 0.085, 0.075, 0.04]),
                             "save PLY", color="0.9", hovercolor="0.8")
        for b in (self.b_pause, self.b_clear, self.b_save):
            b.label.set_fontsize(8)

        self.b_pause.on_clicked(lambda _e: self._toggle_pause())
        self.b_clear.on_clicked(lambda _e: self._clear())
        self.b_save.on_clicked(lambda _e: self._save())

        self.fig.text(0.06, 0.013,
                      "drag to orbit  |  scroll to zoom  |  keys: [p]ause  "
                      "[c]lear  [s]ave PLY  [r]eset view",
                      fontsize=7.6, color="0.35")

    # -- actions -----------------------------------------------------------
    def _send(self, frame: bytes) -> None:
        if self._on_command is not None:
            self._on_command(frame)

    def _range_changed(self, val) -> None:
        self._range_mm = float(val) * 1000.0
        self._apply_range()

    def _toggle_pause(self) -> None:
        self.builder.paused = not self.builder.paused
        self.b_pause.label.set_text("resume" if self.builder.paused else "pause")
        self._msg = "accumulation paused" if self.builder.paused else ""

    def _clear(self) -> None:
        self.builder.cloud.clear()
        self.builder.rings_used = 0
        self._msg = "cloud cleared"

    def _save(self) -> None:
        path = time.strftime("scan-%Y%m%d-%H%M%S.ply")
        n = self.builder.cloud.save_ply(path)
        self._saved_path = path
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

        # Attitude triad, scaled to a fixed fraction of the view.
        scan = getattr(state, "latest_scan", None)
        o = state.latest
        q = scan.q if scan is not None else (o.q if o is not None else (1, 0, 0, 0))
        R = body_to_earth(q)
        length = self._range_mm / 1000.0 * 0.22
        for i, line in enumerate(self.axis_lines):
            v = R[:, i] * length
            line.set_data_3d([0, v[0]], [0, v[1]], [0, v[2]])

        self.txt.set_text(self._format(state, parser_stats))

    def artists(self):
        return (self.sc, self.sc_live, self.txt, self.origin, *self.axis_lines)

    # ------------------------------------------------------------------
    def _format(self, state, ps) -> str:
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
                f"duty {scan.motor_permille}/1000, keep 1-in-{scan.decimation}",
            ]
        else:
            lines.append("lidar   no rings yet")

        if o is not None:
            from .reader import quat_to_euler_deg
            roll, pitch, yaw = quat_to_euler_deg(o.q)
            lines.append(f"attitude  r {roll:+7.1f}  p {pitch:+7.1f}  "
                         f"y {yaw:+7.1f}  deg")

        mount = self.builder.mount
        lines += [
            "",
            f"cloud   {len(c):7d} points  "
            f"({'PAUSED' if self.builder.paused else 'live'})",
            f"        voxel {c.voxel_mm:.0f} mm, budget {c.max_points}",
            f"        {c.total_added} raw points -> "
            f"{len(c)} kept, {self.builder.rings_used} rings",
            f"mount   lidar yaw {mount.yaw_deg:+.1f} deg, "
            f"{'clockwise' if mount.clockwise else 'counter-clockwise'}",
            "",
            "frames  +Y imu = forward, -Z imu = up",
            "        body X fwd / Y left / Z up, earth Z up",
            "",
            "rotation only -- no translation is estimated, so the",
            "device stays at the origin.  yaw has no magnetic",
            "reference and drifts, which smears long sessions.",
        ]

        if self._msg:
            lines += ["", self._msg]
        return "\n".join(lines)


def run_cloud(source, builder: "CloudBuilder", fps: float = 12.0,
              duration: float | None = None, save_path: str | None = None,
              headless: bool = False, range_m: float = 6.0,
              save_ply: str | None = None) -> None:
    """Start ``source`` and animate the cloud until the window closes."""
    import matplotlib

    if headless:
        matplotlib.use("Agg", force=True)

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    view = CloudView(builder, on_command=getattr(source, "send", None),
                     range_m=range_m)
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
        if save_ply:
            n = builder.cloud.save_ply(save_ply)
            print(f"wrote {n} points to {save_ply}")
        plt.close(view.fig)
