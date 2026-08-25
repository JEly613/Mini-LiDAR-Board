"""Entry point:  python3 -m lidarscan [options]

Examples::

    python3 -m lidarscan                       # auto-detect the board
    python3 -m lidarscan --port /dev/cu.usbmodem14201
    python3 -m lidarscan --sim                 # no hardware needed
    python3 -m lidarscan --sim --duration 8    # runs 8 s then exits
    python3 -m lidarscan --sim --headless --save shot.png
    python3 -m lidarscan --selftest            # protocol round-trip checks
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np

from . import protocol as proto
from .reader import SerialSource, SimSource, find_serial_port


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="python3 -m lidarscan",
        description="Live 3D orientation viewer for the Mini-LiDAR-Board.")

    src = ap.add_argument_group("source")
    src.add_argument("--sim", action="store_true",
                     help="synthesise packets instead of reading hardware")
    src.add_argument("--port", metavar="PATH",
                     help="serial device (default: first /dev/cu.usbmodem*)")
    src.add_argument("--baud", type=int, default=115200,
                     help="ignored by USB CDC, but pyserial wants a number")
    src.add_argument("--sim-rate", type=float, default=100.0, metavar="HZ",
                     help="simulated packet rate (default 100)")
    src.add_argument("--sim-corrupt", type=int, default=0, metavar="N",
                     help="corrupt every Nth simulated frame, to exercise the "
                          "CRC/resync path (0 = never)")
    src.add_argument("--sim-drop", type=int, default=0, metavar="N",
                     help="drop every Nth simulated frame, to exercise the "
                          "loss statistic (0 = never)")

    disp = ap.add_argument_group("display")
    disp.add_argument("--fps", type=float, default=30.0,
                      help="redraw rate (default 30)")
    disp.add_argument("--duration", type=float, metavar="SECONDS",
                      help="exit automatically after this long")
    disp.add_argument("--headless", action="store_true",
                      help="render with the Agg backend and open no window")
    disp.add_argument("--save", metavar="PNG",
                      help="write a PNG of the final frame")

    rec = ap.add_argument_group("3D reconstruction")
    rec.add_argument("--cloud", action="store_true",
                     help="show the accumulated 3D point cloud instead of the "
                          "2D ring view")
    rec.add_argument("--voxel-mm", type=float, default=50.0, metavar="MM",
                     help="voxel grid size for thinning the cloud "
                          "(default 50)")
    rec.add_argument("--max-points", type=int, default=60000, metavar="N",
                     help="point budget; oldest voxels are recycled past this "
                          "(default 60000)")
    rec.add_argument("--range-m", type=float, default=6.0, metavar="M",
                     help="half-size of the displayed cube (default 6)")
    rec.add_argument("--lidar-yaw-deg", type=float, default=0.0, metavar="DEG",
                     help="mechanical yaw of the LiDAR relative to the board's "
                          "forward axis")
    rec.add_argument("--lidar-ccw", action="store_true",
                     help="LiDAR angle advances counter-clockwise seen from "
                          "above (flip if the cloud is mirrored)")
    rec.add_argument("--save-ply", metavar="PATH",
                     help="write the accumulated cloud to a PLY file on exit")

    ins = ap.add_argument_group(
        "dead reckoning (--cloud-ins)",
        "Double-integrates the accelerometer to estimate position instead of "
        "assuming the device is rotated in place.  Position error grows as "
        "t^2; see lidarscan/cloud_ins.py for the error budget.")
    ins.add_argument("--cloud-ins", action="store_true",
                     help="3D cloud WITH translation from double-integrated "
                          "accelerometer (compare against --cloud)")
    ins.add_argument("--ins-no-zupt", action="store_true",
                     help="disable zero-velocity updates (drift gets much "
                          "worse; useful to see what ZUPT is buying)")
    ins.add_argument("--ins-vel-tau", type=float, default=1.5, metavar="SEC",
                     help="velocity leak time constant; 0 disables "
                          "(default 1.5)")
    ins.add_argument("--ins-settle", type=float, default=1.0, metavar="SEC",
                     help="hold still this long at startup to estimate accel "
                          "bias (default 1.0)")
    ins.add_argument("--ins-lp-hz", type=float, default=12.0, metavar="HZ",
                     help="accelerometer low-pass corner; 0 disables "
                          "(default 12)")
    ins.add_argument("--ins-zupt-accel", type=float, default=0.04,
                     metavar="G",
                     help="|accel|-1g below which the device counts as still "
                          "(default 0.04)")
    ins.add_argument("--ins-zupt-gyro", type=float, default=3.0, metavar="DPS",
                     help="gyro magnitude below which the device counts as "
                          "still (default 3.0)")
    ins.add_argument("--ins-noise-mg", type=float, default=0.0, metavar="MG",
                     help="inject synthetic accel bias+noise, to demonstrate "
                          "drift on the noise-free simulator")

    ap.add_argument("--list-ports", action="store_true",
                    help="print candidate serial devices and exit")
    ap.add_argument("--selftest", action="store_true",
                    help="run protocol round-trip checks and exit")
    return ap


# --------------------------------------------------------------------------
# Self-test: proves the codec and the resync logic without any hardware.
# --------------------------------------------------------------------------
def selftest() -> int:
    failures = 0

    def check(name: str, ok: bool, detail: str = "") -> None:
        nonlocal failures
        print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"  {detail}" if detail else ""))
        if not ok:
            failures += 1

    print("protocol self-test")

    # 1. The CRC's published check value.
    check("CRC-16/CCITT-FALSE check value", proto.crc16(b"123456789") == 0x29B1,
          f"got 0x{proto.crc16(b'123456789'):04X}, want 0x29B1")

    # 2. Frame sizes match PROTOCOL.md.
    sim = SimSource()
    frame = sim.make_frame(0.5)
    check("ORIENTATION frame is 68 bytes", len(frame) == proto.ORIENTATION_FRAME_LEN,
          f"got {len(frame)}")

    st = proto.encode_status(proto.Status(1, 2, 5, 0x47, 0x07, "hello"))
    check("STATUS frame is 52 bytes", len(st) == proto.STATUS_FRAME_LEN,
          f"got {len(st)}")

    # 3. Round-trip through the real parser.
    p = proto.FrameParser()
    frames = list(p.feed(frame + st))
    check("both frames parse", len(frames) == 2, f"got {len(frames)}")
    if len(frames) == 2:
        o = proto.decode_orientation(frames[0].payload)
        s = proto.decode_status(frames[1].payload)
        check("quaternion is unit length",
              abs(sum(v * v for v in o.q) ** 0.5 - 1.0) < 1e-5)
        check("status message round-trips", s.msg == "hello", repr(s.msg))
        check("who_am_i round-trips", s.who_am_i == 0x47)

    # 4. Garbage in front of a frame: the parser must resync and still deliver.
    p2 = proto.FrameParser()
    noise = bytes([0x00, 0xFF, 0xA5, 0x11, 0x7E, 0xA5, 0x5A, 0x99])
    got = list(p2.feed(noise + frame))
    check("resyncs past leading garbage", len(got) == 1, f"got {len(got)}")
    check("garbage was counted", p2.stats.bytes_discarded > 0,
          f"discarded {p2.stats.bytes_discarded}")

    # 5. A corrupted frame must be rejected, and the next one still accepted.
    bad = bytearray(frame)
    bad[12] ^= 0xFF
    p3 = proto.FrameParser()
    got = list(p3.feed(bytes(bad) + frame))
    check("corrupt frame rejected, next one accepted",
          len(got) == 1 and p3.stats.crc_errors >= 1,
          f"frames={len(got)} crc_errors={p3.stats.crc_errors}")

    # 6. Byte-at-a-time feeding must produce the same result as one big chunk.
    p4 = proto.FrameParser()
    n = sum(len(list(p4.feed(bytes([b])))) for b in frame + st)
    check("byte-at-a-time feeding works", n == 2, f"got {n}")

    # 7. Still-unassigned types must be skipped, not treated as corruption.
    p5 = proto.FrameParser()
    unknown = proto.build_frame(0x11, b"\x00" * 16)
    got = list(p5.feed(unknown + frame))
    check("unknown type parsed and passed through",
          len(got) == 2 and got[0].type == 0x11
          and p5.stats.crc_errors == 0, f"got {[hex(g.type) for g in got]}")

    # 8. Truncated frame must not be emitted, and must complete when the rest
    #    of the bytes arrive.
    p6 = proto.FrameParser()
    got_a = list(p6.feed(frame[:40]))
    got_b = list(p6.feed(frame[40:]))
    check("split frame reassembles", len(got_a) == 0 and len(got_b) == 1,
          f"{len(got_a)} then {len(got_b)}")

    # 9. Command encoders produce well-formed frames.
    p7 = proto.FrameParser()
    cmds = (proto.cmd_set_beta(0.15) + proto.cmd_recalibrate() + proto.cmd_ping()
            + proto.cmd_set_motor(600) + proto.cmd_set_decimation(4)
            + proto.cmd_lidar_enable(True))
    got = list(p7.feed(cmds))
    check("command frames encode/parse", len(got) == 6, f"got {len(got)}")

    # 9b. Command payloads must clamp rather than wrap or raise.
    m = list(proto.FrameParser().feed(proto.cmd_set_motor(99999)))[0]
    d = list(proto.FrameParser().feed(proto.cmd_set_decimation(0)))[0]
    check("motor duty clamps to max",
          int.from_bytes(m.payload, "little") == proto.MOTOR_PERMILLE_MAX,
          f"got {int.from_bytes(m.payload, 'little')}")
    check("decimation clamps to min", d.payload[0] == proto.DECIM_MIN,
          f"got {d.payload[0]}")

    # 10. LiDAR scan frames must round-trip byte-for-byte, at both extremes of
    #     the point count, and must survive the parser.
    for n_pts in (1, 360, proto.LIDAR_MAX_POINTS):
        ang = np.linspace(0.0, 360.0, n_pts, endpoint=False, dtype=np.float32)
        dst = np.linspace(100, 11000, n_pts).astype(np.uint16)
        scan = proto.Scan(
            t_us=123456, seq=7, q=(1.0, 0.0, 0.0, 0.0),
            rot_hz=5.5, sample_hz=2300.0, motor_permille=600, decimation=1,
            flags=proto.FLAG_LIDAR_OK | proto.FLAG_LIDAR_SCANNING,
            angles_deg=ang, dists_mm=dst)
        wire = proto.encode_scan(scan)
        frames = list(proto.FrameParser().feed(wire))
        ok = len(frames) == 1 and frames[0].type == proto.TYPE_LIDAR_SCAN
        back = proto.decode_scan(frames[0].payload) if ok else None
        ok = ok and back.point_count == n_pts
        ok = ok and proto.encode_scan(back) == wire
        check(f"scan frame round-trips ({n_pts} pts)", ok,
              f"{len(wire)} B on the wire")

    # 11. Largest legal scan frame must still fit the declared payload cap.
    biggest = proto.SCAN_HEADER_LEN + proto.LIDAR_MAX_POINTS * proto.SCAN_POINT_LEN
    check("max scan payload fits MAX_PAYLOAD", biggest <= proto.MAX_PAYLOAD,
          f"{biggest} <= {proto.MAX_PAYLOAD}")

    # 12. Zero distances mean 'no return' and must be masked out, not drawn at
    #     the origin.
    wire = proto.encode_scan(proto.Scan(
        t_us=0, seq=0, q=(1.0, 0.0, 0.0, 0.0), rot_hz=5.0, sample_hz=2000.0,
        motor_permille=600, decimation=1, flags=0,
        angles_deg=np.array([0.0, 90.0, 180.0], dtype=np.float32),
        dists_mm=np.array([1000, 0, 2000], dtype=np.uint16)))
    scan = proto.decode_scan(list(proto.FrameParser().feed(wire))[0].payload)
    check("zero distances are masked as no-return",
          list(scan.valid_mask) == [True, False, True],
          f"got {list(scan.valid_mask)}")

    # 13. A scan whose header disagrees with its payload length must raise
    #     rather than silently mis-slice.
    good = proto.encode_scan(proto.Scan(
        t_us=0, seq=0, q=(1.0, 0.0, 0.0, 0.0), rot_hz=5.0, sample_hz=2000.0,
        motor_permille=600, decimation=1, flags=0,
        angles_deg=np.zeros(4, dtype=np.float32),
        dists_mm=np.zeros(4, dtype=np.uint16)))
    payload = list(proto.FrameParser().feed(good))[0].payload
    bad = bytearray(payload)
    bad[32] = 99                       # claim 99 points, carry 4
    raised = False
    try:
        proto.decode_scan(bytes(bad))
    except ValueError:
        raised = True
    check("scan length mismatch is rejected", raised)

    # ----------------------------------------------------------------------
    # 14. 3D reconstruction: frames and the transform chain.
    # ----------------------------------------------------------------------
    from . import cloud as C

    M = C.SENSOR_TO_BODY
    check("sensor->body is orthonormal", np.allclose(M @ M.T, np.eye(3)))
    check("sensor->body is a rotation, not a mirror",
          abs(np.linalg.det(M) - 1.0) < 1e-9, f"det={np.linalg.det(M):+.6f}")
    check("+Y imu maps to body forward (+X)",
          np.allclose(M @ [0, 1, 0], [1, 0, 0]))
    check("-Z imu maps to body up (+Z)",
          np.allclose(M @ [0, 0, -1], [0, 0, 1]))

    # The board lying flat reads accel (0, 0, -1) g because the IMU is on the
    # underside, so Madgwick settles on a ~180 deg roll.  q = (0,1,0,0) is
    # exactly that.  The body up axis must still come out pointing at earth up
    # -- that is the whole point of the convention.
    q_flat = (0.0, 1.0, 0.0, 0.0)
    R_eb = C.body_to_earth(q_flat)
    check("board flat: body up axis points to earth up",
          np.allclose(R_eb[:, 2], [0, 0, 1], atol=1e-9),
          f"got {np.round(R_eb[:, 2], 6)}")
    check("body->earth stays orthonormal",
          np.allclose(R_eb @ R_eb.T, np.eye(3)) and
          abs(np.linalg.det(R_eb) - 1.0) < 1e-9)

    def _ring(n=72, dist_mm=2000, q=(1.0, 0.0, 0.0, 0.0)):
        return proto.Scan(
            t_us=0, seq=0, q=q, rot_hz=5.0, sample_hz=2000.0,
            motor_permille=600, decimation=1, flags=0,
            angles_deg=np.linspace(0, 360, n, endpoint=False, dtype=np.float32),
            dists_mm=np.full(n, dist_mm, dtype=np.uint16))

    mount = C.MountConfig()

    # A flat board sweeping a cylinder of constant range must produce a
    # horizontal circle at the right radius.
    w = C.ring_to_world(_ring(q=q_flat), mount)
    r = np.hypot(w[:, 0], w[:, 1])
    check("flat board: ring is horizontal",
          np.allclose(w[:, 2], 0.0, atol=1e-6),
          f"max |z| = {np.abs(w[:, 2]).max():.3e} mm")
    check("flat board: ring keeps its radius",
          np.allclose(r, 2000.0, atol=1e-6),
          f"radius spread {r.max() - r.min():.3e} mm")

    # Pitch the board 90 deg nose-up.  The scan plane must stand vertical, so
    # the ring's Z extent becomes the full diameter and one axis collapses.
    #   q_flat then rotated 90 deg about earth Y.
    c45, s45 = math.cos(math.pi / 4), math.sin(math.pi / 4)

    def _qmul(a, b):
        w1, x1, y1, z1 = a
        w2, x2, y2, z2 = b
        return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)
    q_pitch = _qmul((c45, 0.0, s45, 0.0), q_flat)
    wp = C.ring_to_world(_ring(q=q_pitch), mount)
    z_extent = wp[:, 2].max() - wp[:, 2].min()
    check("pitched 90 deg: scan plane stands vertical",
          abs(z_extent - 4000.0) < 1e-3, f"z extent {z_extent:.1f} mm (want 4000)")
    check("pitched 90 deg: radius is preserved",
          np.allclose(np.linalg.norm(wp, axis=1), 2000.0, atol=1e-6))

    # No-return points must never reach the cloud.
    sc = proto.Scan(t_us=0, seq=0, q=q_flat, rot_hz=5.0, sample_hz=2000.0,
                    motor_permille=600, decimation=1, flags=0,
                    angles_deg=np.array([0, 90, 180, 270], dtype=np.float32),
                    dists_mm=np.array([1000, 0, 2000, 0], dtype=np.uint16))
    check("no-return points are dropped, not placed at the origin",
          C.ring_to_world(sc, mount).shape[0] == 2,
          f"got {C.ring_to_world(sc, mount).shape[0]} of 4")

    # Mount yaw must rotate the ring rigidly about the up axis.
    a = C.ring_to_world(_ring(q=q_flat), C.MountConfig(yaw_deg=0.0))
    b = C.ring_to_world(_ring(q=q_flat), C.MountConfig(yaw_deg=90.0))
    check("mount yaw preserves ranges",
          np.allclose(np.sort(np.linalg.norm(a, axis=1)),
                      np.sort(np.linalg.norm(b, axis=1)), atol=1e-6))

    # Sweep direction really does mirror the cloud.
    cw = C.ring_to_body(_ring(), C.MountConfig(clockwise=True))
    ccw = C.ring_to_body(_ring(), C.MountConfig(clockwise=False))
    check("sweep direction flips the Y sign",
          np.allclose(cw[:, 1], -ccw[:, 1], atol=1e-6))

    # ---- voxel accumulator ------------------------------------------------
    vc = C.VoxelCloud(voxel_mm=100.0, max_points=1000)
    dense = np.repeat(np.array([[0.0, 0.0, 0.0], [1000.0, 0.0, 0.0]]), 50, axis=0)
    vc.add(dense)
    check("duplicate points collapse to one voxel each", len(vc) == 2,
          f"got {len(vc)} from {dense.shape[0]} points")

    vc2 = C.VoxelCloud(voxel_mm=1.0, max_points=10)
    vc2.add(np.arange(300, dtype=float).reshape(100, 3) * 10.0)
    check("cloud respects its point budget", len(vc2) == 10, f"got {len(vc2)}")
    check("budget eviction keeps the array consistent",
          vc2.xyz().shape == (10, 3))

    vc3 = C.VoxelCloud(voxel_mm=10.0, max_points=1000)
    vc3.add(np.random.uniform(-1000, 1000, size=(500, 3)))
    before = len(vc3)
    vc3.set_voxel_mm(500.0)
    check("re-binning coarser never grows the cloud", len(vc3) <= before,
          f"{before} -> {len(vc3)}")

    # ---- PLY export -------------------------------------------------------
    import os
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, "t.ply")
        n = vc.save_ply(path)
        blob = open(path, "rb").read()
        hdr = blob.split(b"end_header\n", 1)[0]
        body = blob.split(b"end_header\n", 1)[1]
        check("PLY reports the right vertex count",
              f"element vertex {n}".encode() in hdr, f"n={n}")
        check("PLY body is exactly 15 bytes per vertex",
              len(body) == 15 * n, f"{len(body)} for {n} vertices")
        # Millimetres on the wire, metres in the file.
        rec = np.frombuffer(body, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                         ("r", "u1"), ("g", "u1"), ("b", "u1")])
        check("PLY converts millimetres to metres",
              abs(float(np.max(rec["x"])) - 1.0) < 1e-6,
              f"max x = {float(np.max(rec['x'])):.6f} m")

    # 10. The rotation matrix the 3D view uses must be a proper rotation.

    from .viz import quat_to_matrix

    p8 = proto.FrameParser()
    fr = next(iter(p8.feed(sim.make_frame(2.75))))
    o = proto.decode_orientation(fr.payload)
    R = quat_to_matrix(o.q)
    check("view rotation matrix is orthonormal",
          np.allclose(R @ R.T, np.eye(3), atol=1e-6))
    check("view rotation matrix has det +1",
          abs(np.linalg.det(R) - 1.0) < 1e-6, f"det={np.linalg.det(R):.9f}")

    # 11. A pure +90 deg yaw must carry body +X onto earth +Y.
    yaw90 = SimSource._quat_from_euler(0.0, 0.0, math.pi / 2)
    check("+90 deg yaw maps body X to earth Y",
          np.allclose(quat_to_matrix(yaw90) @ np.array([1.0, 0, 0]),
                      np.array([0.0, 1.0, 0.0]), atol=1e-6))

    # 12. End to end: the attitude the view would draw must match the
    #     simulator's ground truth at that instant.  This is the check that
    #     says "the visualiser tracks the simulated rotation".
    worst = 0.0
    for t in (0.0, 0.37, 1.9, 4.2, 7.77, 13.0):
        p9 = proto.FrameParser()
        f = next(iter(p9.feed(sim.make_frame(t))))
        decoded = proto.decode_orientation(f.payload)
        truth = SimSource._quat_from_euler(
            0.5 * math.sin(0.9 * t),
            0.35 * math.sin(0.45 * t + 1.0),
            (0.6 * t) % (2 * math.pi))
        err = np.abs(quat_to_matrix(decoded.q) - quat_to_matrix(truth)).max()
        worst = max(worst, float(err))
    check("displayed attitude tracks the simulated rotation", worst < 1e-6,
          f"worst element error {worst:.3g}")

    # ======================================================================
    # 13. Dead reckoning (--cloud-ins).  The simulator emits pure gravity with
    #     no linear acceleration, so a correct gravity-removal chain MUST hold
    #     the position at exactly zero no matter how the board tumbles.  Any
    #     drift here is a maths bug, not sensor noise.
    # ======================================================================
    from .cloud_ins import (DeadReckoner, INSCloudBuilder, INSConfig,
                            attach_dead_reckoner, ring_to_world_at)

    def feed_sim(dr, t_end=6.0, rate=100.0, sim_src=None):
        src = sim_src or sim
        pr = proto.FrameParser()
        n = int(t_end * rate)
        for i in range(n):
            t = i / rate
            fr = next(iter(pr.feed(src.make_frame(t))))
            dr.update(proto.decode_orientation(fr.payload))
        return dr

    dr = DeadReckoner(INSConfig(settle_s=0.5, zupt=False, vel_tau_s=0.0,
                                accel_lp_hz=0.0))
    feed_sim(dr, 6.0)
    st = dr.snapshot()
    drift = float(np.linalg.norm(st.pos_mm)) / 1000.0
    check("gravity removal leaves zero drift on a tumbling noise-free board",
          drift < 1e-3, f"{drift * 1000:.4f} mm after 6 s, ZUPT and leak off")

    check("no residual velocity either",
          st.speed_mps < 1e-4, f"{st.speed_mps:.3e} m/s")

    # A constant bias must integrate to the textbook 0.5*a*t^2, which both
    # confirms the integrator and pins the error model quoted in the docs.
    dr2 = DeadReckoner(INSConfig(settle_s=0.0, zupt=False, vel_tau_s=0.0,
                                 accel_lp_hz=0.0, bias_gain=0.0))
    A_G = 0.01                      # 10 mg of pure bias
    T, RATE = 4.0, 100.0
    for i in range(int(T * RATE)):
        dr2.update(proto.Orientation(
            t_us=int(i / RATE * 1e6), q=(1.0, 0.0, 0.0, 0.0),
            accel_g=(A_G, 0.0, 1.0), gyro_dps=(0.0, 0.0, 0.0),
            temp_c=25.0, sample_count=i * 10, drop_count=0,
            imu_odr_hz=1000, flags=proto.FLAG_IMU_OK))
    got = dr2.snapshot().pos_mm[0] / 1000.0
    want = 0.5 * (A_G * 9.80665) * T * T
    check("constant bias integrates to 0.5*a*t^2",
          abs(got - want) / want < 0.02,
          f"{got:.3f} m vs {want:.3f} m predicted for {A_G * 1000:.0f} mg / {T:.0f} s")

    # ZUPT must actually suppress that drift when the device reads still.
    dr3 = DeadReckoner(INSConfig(settle_s=0.0, zupt=True, vel_tau_s=0.0,
                                 accel_lp_hz=0.0, bias_gain=0.0,
                                 zupt_accel_g=0.05, zupt_window_s=0.1))
    for i in range(int(T * RATE)):
        dr3.update(proto.Orientation(
            t_us=int(i / RATE * 1e6), q=(1.0, 0.0, 0.0, 0.0),
            accel_g=(A_G, 0.0, 1.0), gyro_dps=(0.0, 0.0, 0.0),
            temp_c=25.0, sample_count=i * 10, drop_count=0,
            imu_odr_hz=1000, flags=proto.FLAG_IMU_OK))
    zupt_pos = abs(float(dr3.snapshot().pos_mm[0])) / 1000.0
    check("ZUPT suppresses drift while the device reads still",
          zupt_pos < got / 100.0,
          f"{zupt_pos * 1000:.2f} mm with ZUPT vs {got * 1000:.0f} mm without")

    # Genuine motion must still register: 0.5 g for 0.5 s is a real shove.
    dr4 = DeadReckoner(INSConfig(settle_s=0.0, zupt=True, vel_tau_s=0.0,
                                 accel_lp_hz=0.0, bias_gain=0.0,
                                 zupt_window_s=0.1))
    for i in range(50):
        dr4.update(proto.Orientation(
            t_us=int(i / RATE * 1e6), q=(1.0, 0.0, 0.0, 0.0),
            accel_g=(0.5, 0.0, 1.0), gyro_dps=(0.0, 0.0, 0.0),
            temp_c=25.0, sample_count=i * 10, drop_count=0,
            imu_odr_hz=1000, flags=proto.FLAG_IMU_OK))
    moved = dr4.snapshot().pos_mm[0] / 1000.0
    check("real acceleration is not suppressed by ZUPT", moved > 0.4,
          f"moved {moved:.3f} m under 0.5 g for 0.5 s")

    # Interpolation between samples, and time-indexed lookup.
    dr5 = DeadReckoner(INSConfig(settle_s=0.0, zupt=False, vel_tau_s=0.0,
                                 accel_lp_hz=0.0, bias_gain=0.0))
    for i in range(200):
        dr5.update(proto.Orientation(
            t_us=int(i / RATE * 1e6), q=(1.0, 0.0, 0.0, 0.0),
            accel_g=(0.1, 0.0, 1.0), gyro_dps=(0.0, 0.0, 0.0),
            temp_c=25.0, sample_count=i * 10, drop_count=0,
            imu_odr_hz=1000, flags=proto.FLAG_IMU_OK))
    early = dr5.position_at(int(0.5e6))[0]
    late = dr5.position_at(int(1.5e6))[0]
    mid = dr5.position_at(int(1.0e6))[0]
    check("position_at is monotonic under constant acceleration",
          early < mid < late, f"{early:.1f} < {mid:.1f} < {late:.1f} mm")
    check("position_at interpolates between samples",
          abs(dr5.position_at(int(1.005e6))[0]
              - 0.5 * (dr5.position_at(int(1.0e6))[0]
                       + dr5.position_at(int(1.01e6))[0])) < 1e-6)

    # The translated ring must equal the rotation-only ring plus the offset,
    # and the offset must NOT be rotated -- it is where the sensor was.
    p10 = proto.FrameParser()
    sf = sim.make_scan_frame(3.0, SimSource._quat_from_euler(0.3, -0.2, 1.1))
    scan = proto.decode_scan(next(iter(p10.feed(sf))).payload)
    mount = C.MountConfig()
    base = C.ring_to_world(scan, mount)
    off = np.array([1234.0, -567.0, 89.0])
    shifted = ring_to_world_at(scan, mount, off)
    check("translation offsets the ring without rotating the offset",
          np.allclose(shifted - base, off, atol=1e-9),
          f"max dev {np.abs((shifted - base) - off).max():.3e} mm")
    check("zero offset reproduces the rotation-only result exactly",
          np.allclose(ring_to_world_at(scan, mount, np.zeros(3)), base,
                      atol=1e-12))

    # hold_origin must be bit-identical to the --cloud behaviour.
    b_ins = INSCloudBuilder(mount, DeadReckoner(), voxel_mm=50.0)
    b_ins.hold_origin = True

    class _S:
        latest_scan = scan
    b_ins.consume(_S())
    b_rot = C.CloudBuilder(mount, voxel_mm=50.0)
    b_rot.consume(_S())
    check("'hold pos' reproduces --cloud exactly",
          np.allclose(np.sort(b_ins.cloud.xyz(), axis=0),
                      np.sort(b_rot.cloud.xyz(), axis=0), atol=1e-9),
          f"{len(b_ins.cloud)} vs {len(b_rot.cloud)} points")

    # The reader hook must not disturb the normal packet path.
    probe = SimSource(rate_hz=100.0)
    seen = []
    dr6 = DeadReckoner(INSConfig(settle_s=0.0))
    dr6.update = lambda o: seen.append(o)
    undo = attach_dead_reckoner(probe, dr6)
    pr = proto.FrameParser()
    fr = next(iter(pr.feed(probe.make_frame(1.0))))
    probe._handle_orientation(proto.decode_orientation(fr.payload))
    check("dead-reckoning hook sees orientation packets", len(seen) == 1)
    check("hook still forwards to the normal handler",
          probe.snapshot().latest is not None)
    undo()
    probe._handle_orientation(proto.decode_orientation(fr.payload))
    check("detaching the hook restores the original handler", len(seen) == 1)

    # A dt outside the sane band must be dropped, not integrated across.
    dr7 = DeadReckoner(INSConfig(settle_s=0.0, zupt=False, vel_tau_s=0.0,
                                 accel_lp_hz=0.0, bias_gain=0.0))
    for t_us in (0, 10_000, 5_000_000, 5_010_000):
        dr7.update(proto.Orientation(
            t_us=t_us, q=(1.0, 0.0, 0.0, 0.0), accel_g=(1.0, 0.0, 1.0),
            gyro_dps=(0.0, 0.0, 0.0), temp_c=25.0, sample_count=0,
            drop_count=0, imu_odr_hz=1000, flags=proto.FLAG_IMU_OK))
    check("an implausible dt is rejected rather than integrated",
          dr7.snapshot().rejected == 1,
          f"rejected {dr7.snapshot().rejected}")

    print(f"\n{'all checks passed' if failures == 0 else f'{failures} FAILURES'}")
    return 1 if failures else 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    if args.selftest:
        return selftest()

    if args.list_ports:
        import glob
        from .reader import SERIAL_PORT_GLOB

        ports = sorted(glob.glob(SERIAL_PORT_GLOB))
        if ports:
            print("\n".join(ports))
        else:
            print(f"no devices matching {SERIAL_PORT_GLOB}")
        return 0

    if args.sim:
        source = SimSource(rate_hz=args.sim_rate,
                           corrupt_every=args.sim_corrupt,
                           drop_every=args.sim_drop)
        print(f"simulated source at {args.sim_rate:.0f} Hz "
              f"(corrupt every {args.sim_corrupt or '-'}, "
              f"drop every {args.sim_drop or '-'})")
    else:
        port = args.port or find_serial_port()
        if port is None:
            print("No board found on /dev/cu.usbmodem*.\n"
                  "Plug the board in, or run with --sim to use synthetic data.",
                  file=sys.stderr)
            return 2
        print(f"reading {port}")
        source = SerialSource(port=port, baud=args.baud)

    if args.cloud_ins:
        from .cloud import MountConfig
        from .cloud_ins import (DeadReckoner, INSCloudBuilder, INSConfig,
                                run_cloud_ins)

        dr = DeadReckoner(INSConfig(
            zupt=not args.ins_no_zupt,
            zupt_accel_g=args.ins_zupt_accel,
            zupt_gyro_dps=args.ins_zupt_gyro,
            vel_tau_s=args.ins_vel_tau,
            settle_s=args.ins_settle,
            accel_lp_hz=args.ins_lp_hz,
            noise_mg=args.ins_noise_mg,
        ))
        builder = INSCloudBuilder(
            MountConfig(yaw_deg=args.lidar_yaw_deg,
                        clockwise=not args.lidar_ccw),
            dr, voxel_mm=args.voxel_mm, max_points=args.max_points)

        cloud_fps = args.fps if args.fps != 30.0 else 12.0

        t0 = time.monotonic()
        run_cloud_ins(source, builder, fps=cloud_fps, duration=args.duration,
                      save_path=args.save, headless=args.headless,
                      range_m=args.range_m, save_ply=args.save_ply)

        snap = source.snapshot()
        ps = source.parser.stats
        ins_state = dr.snapshot()
        print(f"\nran {time.monotonic() - t0:.1f} s")
        print(f"frames ok      : {ps.frames_ok}")
        print(f"rings used     : {builder.rings_used}")
        print(f"cloud points   : {len(builder.cloud)} "
              f"(from {builder.cloud.total_added} raw)")
        print(f"packet loss    : {snap.packet_loss_pct:.2f} %")
        print(f"imu samples    : {ins_state.samples} "
              f"({100.0 * ins_state.static_frac:.1f} % judged still)")
        print(f"final position : "
              f"{ins_state.pos_mm[0] / 1000.0:+.3f} "
              f"{ins_state.pos_mm[1] / 1000.0:+.3f} "
              f"{ins_state.pos_mm[2] / 1000.0:+.3f} m")
        print(f"path length    : {ins_state.path_mm / 1000.0:.3f} m")
        print(f"accel bias est : {ins_state.bias_mg:.1f} mg")
        print(f"ZUPTs applied  : {ins_state.zupt_count}")
        return 0

    if args.cloud:
        from .cloud import CloudBuilder, MountConfig, run_cloud

        builder = CloudBuilder(
            MountConfig(yaw_deg=args.lidar_yaw_deg,
                        clockwise=not args.lidar_ccw),
            voxel_mm=args.voxel_mm, max_points=args.max_points)

        # 12 fps by default: a 3D scatter redraw of tens of thousands of
        # points costs far more than an orientation plot, and the cloud only
        # changes when a revolution completes anyway.
        cloud_fps = args.fps if args.fps != 30.0 else 12.0

        t0 = time.monotonic()
        run_cloud(source, builder, fps=cloud_fps, duration=args.duration,
                  save_path=args.save, headless=args.headless,
                  range_m=args.range_m, save_ply=args.save_ply)

        snap = source.snapshot()
        ps = source.parser.stats
        print(f"\nran {time.monotonic() - t0:.1f} s")
        print(f"frames ok      : {ps.frames_ok}")
        print(f"  by type      : "
              f"{ {hex(k): v for k, v in sorted(ps.by_type.items())} }")
        print(f"rings used     : {builder.rings_used}")
        print(f"cloud points   : {len(builder.cloud)} "
              f"(from {builder.cloud.total_added} raw)")
        print(f"packet loss    : {snap.packet_loss_pct:.2f} %")
        return 0

    from .viz import run_live

    t0 = time.monotonic()
    run_live(source, fps=args.fps, duration=args.duration,
             save_path=args.save, headless=args.headless)

    snap = source.snapshot()
    ps = source.parser.stats
    print(f"\nran {time.monotonic() - t0:.1f} s")
    print(f"frames ok      : {ps.frames_ok}")
    print(f"  by type      : "
          f"{ {hex(k): v for k, v in sorted(ps.by_type.items())} }")
    print(f"crc errors     : {ps.crc_errors}")
    print(f"resyncs        : {ps.resyncs}")
    print(f"bytes discarded: {ps.bytes_discarded}")
    print(f"packet loss    : {snap.packet_loss_pct:.2f} %")
    if snap.error:
        print(f"\nerror: {snap.error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
