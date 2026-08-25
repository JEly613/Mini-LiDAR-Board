"""Real-time orientation display, built on plain matplotlib.

Deliberately no Open3D / PyQt / pyqtgraph / vispy: none of them are installed
on the target machine, and a wireframe board plus a body-axis triad on
``mpl_toolkits.mplot3d`` is entirely adequate for judging attitude at a glance.

Layout::

    +---------------------------+---------------------------------+
    |                           |  roll / pitch / yaw vs time     |
    |   3D board + axis triad   +---------------------------------+
    |                           |  link and filter statistics     |
    +---------------------------+---------------------------------+

Redraw is driven by ``FuncAnimation`` at ~30 FPS.  Every artist is created once
and only its data is updated, so the cost per frame stays flat -- the 3D axes
are re-rendered wholesale by matplotlib either way, but not re-built.
"""

from __future__ import annotations

import numpy as np

from . import protocol as proto
from .reader import quat_to_euler_deg

# Board outline in its own frame, in metres-ish units where the long axis is 1.
# Roughly the real PCB proportions (about 40 x 25 x 1.6 mm).
_HALF_X, _HALF_Y, _HALF_Z = 0.50, 0.31, 0.02

_BOX_VERTS = np.array([
    [-_HALF_X, -_HALF_Y, -_HALF_Z],
    [+_HALF_X, -_HALF_Y, -_HALF_Z],
    [+_HALF_X, +_HALF_Y, -_HALF_Z],
    [-_HALF_X, +_HALF_Y, -_HALF_Z],
    [-_HALF_X, -_HALF_Y, +_HALF_Z],
    [+_HALF_X, -_HALF_Y, +_HALF_Z],
    [+_HALF_X, +_HALF_Y, +_HALF_Z],
    [-_HALF_X, +_HALF_Y, +_HALF_Z],
])

_BOX_EDGES = [
    (0, 1), (1, 2), (2, 3), (3, 0),      # bottom face
    (4, 5), (5, 6), (6, 7), (7, 4),      # top face
    (0, 4), (1, 5), (2, 6), (3, 7),      # verticals
]

# A wedge on the +X end marks the USB connector, so the board's heading is
# unambiguous even when the box is nearly edge-on.
_NOSE = np.array([
    [+_HALF_X, -0.12, 0.0],
    [+_HALF_X + 0.16, 0.0, 0.0],
    [+_HALF_X, +0.12, 0.0],
])

_AXIS_LEN = 0.75
_TIME_WINDOW_S = 10.0


def quat_to_matrix(q) -> np.ndarray:
    """(w, x, y, z) unit quaternion -> 3x3 rotation matrix (body -> earth)."""
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


def _edges_to_segments(verts: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Flatten the box edges into one NaN-separated polyline.

    One Line3D with NaN breaks is far cheaper to update than twelve separate
    line artists, and matplotlib treats NaN as a pen-up.
    """
    xs, ys, zs = [], [], []
    for a, b in _BOX_EDGES:
        xs += [verts[a, 0], verts[b, 0], np.nan]
        ys += [verts[a, 1], verts[b, 1], np.nan]
        zs += [verts[a, 2], verts[b, 2], np.nan]
    return np.array(xs), np.array(ys), np.array(zs)


# How far out the polar plot reaches.  The A1 is specified to 12 m but indoor
# returns past ~6 m are sparse, so a 6 m window keeps the room filling the plot.
_RANGE_LIMIT_MM = 6000.0

# How many past revolutions to keep as a faint trail.
_TRAIL_RINGS = 3


class OrientationView:
    """Owns the figure and knows how to refresh it from a LinkState snapshot."""

    def __init__(self, title: str = "Mini-LiDAR-Board live view",
                 on_command=None) -> None:
        import matplotlib.pyplot as plt

        # Where a control change is sent.  None in headless/no-link mode, in
        # which case the widgets are still built but simply do nothing.
        self._on_command = on_command

        self.fig = plt.figure(figsize=(16.5, 8.2))
        self.fig.canvas.manager.set_window_title(title) if hasattr(
            self.fig.canvas, "manager") and self.fig.canvas.manager else None

        gs = self.fig.add_gridspec(2, 3, width_ratios=[1.0, 1.05, 0.95],
                                   height_ratios=[1.35, 1.0],
                                   hspace=0.30, wspace=0.20,
                                   left=0.02, right=0.985, top=0.93,
                                   bottom=0.155)

        # ---- 3D attitude -------------------------------------------------
        self.ax3d = self.fig.add_subplot(gs[:, 0], projection="3d")
        self._setup_3d()

        # ---- LiDAR ring, in the sensor's own polar frame ------------------
        self.ax_polar = self.fig.add_subplot(gs[:, 1], projection="polar")
        self._setup_polar()

        # ---- roll / pitch / yaw time series ------------------------------
        self.ax_rpy = self.fig.add_subplot(gs[0, 2])
        self._setup_rpy()

        # ---- statistics panel --------------------------------------------
        self.ax_txt = self.fig.add_subplot(gs[1, 2])
        self.ax_txt.axis("off")
        self.txt = self.ax_txt.text(
            0.0, 1.0, "", transform=self.ax_txt.transAxes,
            va="top", ha="left", family="monospace", fontsize=8.8)

        self._last_scan_seq = -1
        self._trail: list = []

        # ---- interactive controls ----------------------------------------
        self._setup_controls()

        self.fig.suptitle(title, fontsize=13, y=0.975)

    # ------------------------------------------------------------------
    def _setup_3d(self) -> None:
        ax = self.ax3d
        ax.set_title("board attitude", fontsize=10, pad=0)
        lim = 0.95
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_zlim(-lim, lim)
        ax.set_box_aspect((1, 1, 1))
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.set_zticklabels([])
        ax.set_xlabel("earth X", fontsize=8, labelpad=-8)
        ax.set_ylabel("earth Y", fontsize=8, labelpad=-8)
        ax.set_zlabel("earth Z (up)", fontsize=8, labelpad=-8)
        ax.view_init(elev=22, azim=-58)

        # Ground plane reference, so "level" is visually obvious.
        g = 0.9
        ax.plot([-g, g, g, -g, -g], [-g, -g, g, g, -g], [0, 0, 0, 0, 0],
                color="0.75", lw=0.8, zorder=0)

        (self.box_line,) = ax.plot([], [], [], color="0.15", lw=1.8)
        (self.nose_line,) = ax.plot([], [], [], color="0.15", lw=1.8)

        # Body axes: X red, Y green, Z blue -- the usual convention.
        (self.axis_x,) = ax.plot([], [], [], color="#d62728", lw=2.6)
        (self.axis_y,) = ax.plot([], [], [], color="#2ca02c", lw=2.6)
        (self.axis_z,) = ax.plot([], [], [], color="#1f77b4", lw=2.6)

        self.axis_labels = [
            ax.text(0, 0, 0, "X", color="#d62728", fontsize=9, weight="bold"),
            ax.text(0, 0, 0, "Y", color="#2ca02c", fontsize=9, weight="bold"),
            ax.text(0, 0, 0, "Z", color="#1f77b4", fontsize=9, weight="bold"),
        ]

    def _setup_polar(self) -> None:
        """The LiDAR ring, drawn in the sensor's own frame.

        Deliberately NOT rotated by the IMU quaternion: this milestone shows
        the two streams side by side and nothing more.  The quaternion rides
        along in every scan packet, but applying it is the next stage's job.
        """
        ax = self.ax_polar
        ax.set_title("LiDAR ring (sensor frame)", fontsize=10, pad=12)

        # 0 deg at the top and clockwise, which is how the A1's angle actually
        # advances when you look down at the spinning head.
        ax.set_theta_zero_location("N")
        ax.set_theta_direction(-1)

        ax.set_ylim(0, _RANGE_LIMIT_MM)
        ax.set_yticks([1000, 2000, 3000, 4000, 5000])
        ax.set_yticklabels(["1 m", "2 m", "3 m", "4 m", "5 m"], fontsize=7)
        ax.tick_params(axis="x", labelsize=7)
        ax.grid(True, alpha=0.3, linewidth=0.6)

        # Current revolution, coloured by range so near/far reads at a glance.
        self.scan_pts = ax.scatter([], [], s=3.0, c=[], cmap="viridis",
                                   vmin=0, vmax=_RANGE_LIMIT_MM, alpha=0.9)

        # The previous few revolutions, faint, so a static scene looks solid
        # while a moving one still reads as motion rather than smearing.
        self.scan_trail = ax.scatter([], [], s=1.4, c="0.62", alpha=0.35)

        self.polar_note = ax.text(
            0.5, -0.13, "", transform=ax.transAxes, ha="center", va="top",
            fontsize=8, family="monospace")

    def _setup_controls(self) -> None:
        """Sliders for spindle speed and decimation, plus a scan on/off.

        These write straight through to the device: the motor slider changes
        the PWM duty on PB10, and the decimation slider changes how many
        points the firmware puts in each packet.  Neither is a display-side
        filter -- the whole point is that they alter what the hardware does.
        """
        from matplotlib.widgets import Button, Slider

        ax_motor = self.fig.add_axes([0.075, 0.075, 0.30, 0.028])
        ax_decim = self.fig.add_axes([0.075, 0.032, 0.30, 0.028])
        ax_scan = self.fig.add_axes([0.435, 0.032, 0.085, 0.045])

        self.s_motor = Slider(
            ax_motor, "spin  ", 0, proto.MOTOR_PERMILLE_MAX,
            valinit=proto.MOTOR_PERMILLE_DEFAULT, valstep=10, color="#4C72B0")
        self.s_decim = Slider(
            ax_decim, "keep 1 in  ", proto.DECIM_MIN, proto.DECIM_MAX,
            valinit=1, valstep=1, color="#55A868")

        for sl in (self.s_motor, self.s_decim):
            sl.label.set_fontsize(8)
            sl.valtext.set_fontsize(8)

        self.b_scan = Button(ax_scan, "scan: on", color="0.9", hovercolor="0.8")
        self.b_scan.label.set_fontsize(8)
        self._scanning = True

        self.s_motor.on_changed(self._motor_changed)
        self.s_decim.on_changed(self._decim_changed)
        self.b_scan.on_clicked(self._scan_clicked)

        self.fig.text(0.075, 0.113,
                      "spin: PWM duty on PB10 (permille).   keep 1 in N: "
                      "on-device decimation, applied before the packet is built.",
                      fontsize=7.4, color="0.35")

    # -- control callbacks -------------------------------------------------
    def _send(self, frame: bytes) -> None:
        if self._on_command is not None:
            self._on_command(frame)

    def _motor_changed(self, val) -> None:
        self._send(proto.cmd_set_motor(int(val)))

    def _decim_changed(self, val) -> None:
        self._send(proto.cmd_set_decimation(int(val)))

    def _scan_clicked(self, _event) -> None:
        self._scanning = not self._scanning
        self.b_scan.label.set_text(f"scan: {'on' if self._scanning else 'off'}")
        self._send(proto.cmd_lidar_enable(self._scanning))

    def _setup_rpy(self) -> None:
        ax = self.ax_rpy
        ax.set_title("roll / pitch / yaw", fontsize=10)
        ax.set_ylabel("degrees", fontsize=9)
        ax.set_xlabel("device time (s)", fontsize=9)
        ax.set_ylim(-190, 190)
        ax.set_yticks([-180, -90, 0, 90, 180])
        ax.grid(alpha=0.3)
        ax.axhline(0, color="0.6", lw=0.8)

        (self.line_roll,) = ax.plot([], [], color="#d62728", lw=1.3, label="roll")
        (self.line_pitch,) = ax.plot([], [], color="#2ca02c", lw=1.3, label="pitch")
        (self.line_yaw,) = ax.plot([], [], color="#1f77b4", lw=1.3, label="yaw")
        ax.legend(loc="upper right", fontsize=8, ncol=3, framealpha=0.85)

    # ------------------------------------------------------------------
    def artists(self):
        return (self.box_line, self.nose_line, self.axis_x, self.axis_y,
                self.axis_z, self.line_roll, self.line_pitch, self.line_yaw,
                self.txt, self.scan_pts, self.scan_trail, self.polar_note,
                *self.axis_labels)

    def update(self, state, parser_stats: proto.ParserStats) -> None:
        """Refresh every artist from a LinkState snapshot."""
        o = state.latest
        rot = quat_to_matrix(o.q) if o is not None else np.eye(3)

        # ---- 3D ----------------------------------------------------------
        verts = _BOX_VERTS @ rot.T
        xs, ys, zs = _edges_to_segments(verts)
        self.box_line.set_data_3d(xs, ys, zs)

        nose = _NOSE @ rot.T
        self.nose_line.set_data_3d(nose[:, 0], nose[:, 1], nose[:, 2])

        for line, label, col in ((self.axis_x, self.axis_labels[0], 0),
                                 (self.axis_y, self.axis_labels[1], 1),
                                 (self.axis_z, self.axis_labels[2], 2)):
            vec = rot[:, col] * _AXIS_LEN
            line.set_data_3d([0, vec[0]], [0, vec[1]], [0, vec[2]])
            label.set_position((vec[0], vec[1]))
            label.set_3d_properties(vec[2], zdir=None)

        # ---- time series --------------------------------------------------
        if state.history_t:
            t = np.fromiter(state.history_t, dtype=float)
            rpy = np.array(state.history_rpy, dtype=float)
            t_rel = t - t[0]
            self.line_roll.set_data(t_rel, rpy[:, 0])
            self.line_pitch.set_data(t_rel, rpy[:, 1])
            self.line_yaw.set_data(t_rel, rpy[:, 2])
            hi = t_rel[-1]
            self.ax_rpy.set_xlim(max(0.0, hi - _TIME_WINDOW_S), max(hi, 1.0))

        # ---- LiDAR ring ----------------------------------------------------
        self._update_polar(state)

        # ---- stats ---------------------------------------------------------
        self.txt.set_text(self._format_stats(state, parser_stats))

    # ------------------------------------------------------------------
    def _update_polar(self, state) -> None:
        scan = getattr(state, "latest_scan", None)

        if scan is None:
            self.polar_note.set_text("no LiDAR data")
            return

        # Only redraw when a genuinely new revolution has arrived; re-pushing
        # identical offsets 30 times a second is wasted work.
        if scan.seq == self._last_scan_seq:
            return
        self._last_scan_seq = scan.seq

        valid = scan.valid_mask
        theta = np.radians(scan.angles_deg[valid].astype(np.float64))
        r = scan.dists_mm[valid].astype(np.float64)

        # Clip rather than drop long returns, so a distant wall still shows up
        # at the edge of the plot instead of vanishing.
        r_clipped = np.minimum(r, _RANGE_LIMIT_MM)

        self.scan_pts.set_offsets(np.column_stack((theta, r_clipped)))
        self.scan_pts.set_array(r_clipped)

        # Trail of previous revolutions.
        self._trail.append((theta, r_clipped))
        while len(self._trail) > _TRAIL_RINGS:
            self._trail.pop(0)
        if len(self._trail) > 1:
            th_all = np.concatenate([t for t, _ in self._trail[:-1]])
            r_all = np.concatenate([rr for _, rr in self._trail[:-1]])
            self.scan_trail.set_offsets(np.column_stack((th_all, r_all)))

        dropped = int((~valid).sum())
        note = (f"{scan.point_count:4d} pts  "
                f"{scan.rot_hz:4.1f} rev/s  "
                f"{scan.sample_hz:6.0f} samp/s  "
                f"1-in-{scan.decimation}  "
                f"{dropped} no-return")
        if scan.truncated:
            note += "  [TRUNCATED]"
        self.polar_note.set_text(note)

    # ------------------------------------------------------------------
    @staticmethod
    def _format_stats(state, ps: proto.ParserStats) -> str:
        lines = [f"source          : {state.source_name}"]

        if state.error:
            lines.append("")
            lines += ["ERROR: " + ln for ln in state.error.splitlines()]
            return "\n".join(lines)

        o = state.latest
        if o is None:
            lines.append("waiting for the first packet...")
            return "\n".join(lines)

        roll, pitch, yaw = quat_to_euler_deg(o.q)
        bias_state = ("calibrating" if o.calibrating
                      else ("valid" if o.bias_valid else "NOT CALIBRATED"))

        lines += [
            f"rate      {state.packet_rate_hz:6.1f} Hz    "
            f"IMU {o.imu_odr_hz} Hz, 1 pkt / {state.decimation} samples",
            f"loss      {state.packet_loss_pct:6.2f} %     "
            f"{state.packets_received} rx / {state.packets_expected} expected",
            f"dev drops {state.device_drop_count:6d}       "
            f"CRC err {ps.crc_errors}, resync {ps.resyncs}, "
            f"bad hdr {ps.header_rejects}",
            f"discarded {ps.bytes_discarded:6d} bytes",
            "",
            f"quat   w {o.q[0]:+.4f}  x {o.q[1]:+.4f}"
            f"  y {o.q[2]:+.4f}  z {o.q[3]:+.4f}",
            f"rpy    {roll:+8.2f} {pitch:+8.2f} {yaw:+8.2f}  deg",
            f"accel  {o.accel_g[0]:+8.3f} {o.accel_g[1]:+8.3f}"
            f" {o.accel_g[2]:+8.3f}  g",
            f"gyro   {o.gyro_dps[0]:+8.2f} {o.gyro_dps[1]:+8.2f}"
            f" {o.gyro_dps[2]:+8.2f}  deg/s",
            f"temp   {o.temp_c:8.1f} C      gyro bias: {bias_state}",
            f"device samples {o.sample_count}",
        ]

        # ---- LiDAR ----------------------------------------------------------
        scan = getattr(state, "latest_scan", None)
        lines.append("")
        if scan is None:
            lidar_up = bool(o.flags & proto.FLAG_LIDAR_OK)
            lines.append("lidar   " + ("up, no ring yet" if lidar_up
                                       else "no data"))
        else:
            gaps = getattr(state, "scan_seq_gaps", 0)
            lines += [
                f"lidar   {state.scan_rate_hz:5.1f} rings/s   "
                f"{scan.point_count} pts/ring, "
                f"seq {scan.seq}, {gaps} missed",
                f"spin    {scan.rot_hz:5.1f} rev/s     "
                f"duty {scan.motor_permille}/1000, "
                f"keep 1-in-{scan.decimation}",
                f"samples {scan.sample_hz:5.0f} /s       "
                f"(115200 baud caps this at ~2300)",
            ]
            if scan.truncated:
                lines.append("        ring TRUNCATED -- spin faster or "
                             "decimate more")

        lines += [
            "",
            "no magnetometer on this board -- yaw drifts.",
            "rings are drawn in the sensor frame; no 3D fusion yet.",
        ]

        if state.status_log:
            lines.append("")
            lines += [s for s in list(state.status_log)[-2:]]

        return "\n".join(lines)


def run_live(source, fps: float = 30.0, duration: float | None = None,
             save_path: str | None = None, headless: bool = False) -> None:
    """Start ``source`` and animate until the window closes or time runs out.

    ``headless`` renders without a GUI and writes a PNG, which is how this gets
    verified in CI or over SSH.
    """
    import matplotlib

    if headless:
        matplotlib.use("Agg", force=True)

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    # Wire the control widgets to whatever the source can accept.  Both
    # SerialSource and SimSource expose send(); if a source does not, the
    # widgets still render and simply have no effect.
    on_command = getattr(source, "send", None)
    view = OrientationView(on_command=on_command)
    source.start()

    interval_ms = 1000.0 / fps
    frames_total = None if duration is None else max(1, int(duration * fps))
    state_box = {"frames": 0}

    def step(_frame):
        state_box["frames"] += 1
        view.update(source.snapshot(), source.parser.stats)
        return view.artists()

    # blit=False: mplot3d recomputes projections on every draw, so blitting
    # buys nothing here and breaks the 3D axes' own caching.
    anim = FuncAnimation(view.fig, step, frames=frames_total,
                         interval=interval_ms, blit=False,
                         cache_frame_data=False, repeat=False)

    try:
        if headless:
            # Drive the animation manually: no event loop exists under Agg.
            # Pace against the wall clock rather than sleeping a fixed interval
            # per frame, so `--duration N` really takes N seconds even though a
            # 3D redraw costs tens of milliseconds.
            import time

            start = time.monotonic()
            deadline = start + (duration if duration is not None else 3.0)
            i = 0
            while time.monotonic() < deadline:
                step(i)
                view.fig.canvas.draw()
                i += 1
                sleep_for = (start + i * interval_ms / 1000.0) - time.monotonic()
                if sleep_for > 0:
                    time.sleep(min(sleep_for, max(0.0, deadline - time.monotonic())))
            if save_path:
                view.fig.savefig(save_path, dpi=110)
                print(f"wrote {save_path}  ({i} frames rendered)")
        else:
            if duration is not None:
                # Close the window on our own schedule so the process always
                # terminates, even unattended.
                timer = view.fig.canvas.new_timer(interval=int(duration * 1000))
                timer.add_callback(plt.close, view.fig)
                timer.single_shot = True
                timer.start()
            plt.show()
            if save_path:
                view.fig.savefig(save_path, dpi=110)
                print(f"wrote {save_path}")
    finally:
        source.stop()
        # Keep a reference alive until here; FuncAnimation is garbage-collected
        # aggressively otherwise and the animation silently stops.
        del anim
