#!/usr/bin/env python3
"""
Top-down map-density + trajectory overview.

Reads a PCD map and a TUM trajectory (assumed already in the same frame) and
renders the classic "does the path sit sensibly on the map" picture:
  * point cloud projected to XY as a grayscale density (2-D histogram)
  * trajectory as a red line, start marked green, end marked blue

It only reads your existing outputs — it changes nothing in the pipeline.

Run:
    python plot_map_trajectory.py map.pcd trajectory.tum -o overview.png
"""

import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm


def load_pcd_xyz(path):
    """Return (N,3) float64 x,y,z from an ASCII or uncompressed-binary PCD.

    For DATA binary_compressed, install open3d and replace this body with:
        import open3d as o3d
        return np.asarray(o3d.io.read_point_cloud(path).points, dtype=np.float64)
    """
    fields = sizes = types = counts = None
    n_points = data_fmt = None
    header_bytes = header_lines = 0
    with open(path, "rb") as f:
        while True:
            line = f.readline()
            if not line:
                raise ValueError("hit EOF before DATA line — is this a PCD?")
            header_bytes += len(line)
            header_lines += 1
            tok = line.decode("ascii", "replace").split()
            if not tok or tok[0].startswith("#"):
                continue
            k = tok[0].upper()
            if   k == "FIELDS": fields = tok[1:]
            elif k == "SIZE":   sizes  = [int(s) for s in tok[1:]]
            elif k == "TYPE":   types  = tok[1:]
            elif k == "COUNT":  counts = [int(c) for c in tok[1:]]
            elif k == "POINTS": n_points = int(tok[1])
            elif k == "DATA":   data_fmt = tok[1].lower(); break

    if counts is None:
        counts = [1] * len(fields)

    if data_fmt == "ascii":
        cols = np.loadtxt(path, skiprows=header_lines, ndmin=2)
        idx = {f: i for i, f in enumerate(fields)}
        return np.stack([cols[:, idx["x"]], cols[:, idx["y"]], cols[:, idx["z"]]],
                        axis=-1).astype(np.float64)

    if data_fmt != "binary":
        raise ValueError(f"DATA={data_fmt} not supported here — see docstring "
                         f"for the open3d one-liner (handles binary_compressed).")

    tmap = {("F", 4): "f4", ("F", 8): "f8",
            ("U", 1): "u1", ("U", 2): "u2", ("U", 4): "u4", ("U", 8): "u8",
            ("I", 1): "i1", ("I", 2): "i2", ("I", 4): "i4", ("I", 8): "i8"}
    dfields = []
    for fld, sz, tp, cnt in zip(fields, sizes, types, counts):
        npt = tmap[(tp.upper(), sz)]
        dfields.append((fld, npt) if cnt == 1 else (fld, npt, (cnt,)))
    dt = np.dtype(dfields)
    with open(path, "rb") as f:
        f.seek(header_bytes)
        arr = np.frombuffer(f.read(n_points * dt.itemsize), dtype=dt, count=n_points)
    return np.stack([arr["x"], arr["y"], arr["z"]], axis=-1).astype(np.float64)


def main():
    ap = argparse.ArgumentParser(description="Top-down map density + trajectory overlay.")
    ap.add_argument("pcd", help="map .pcd (ascii or binary)")
    ap.add_argument("tum", help="trajectory .tum (t tx ty tz qx qy qz qw)")
    ap.add_argument("-o", "--out", default="map_trajectory.png")
    ap.add_argument("--bins", type=int, default=700,
                    help="2-D histogram resolution; raise for thinner walls")
    ap.add_argument("--max-points", type=int, default=8_000_000,
                    help="random subsample cap for the density layer")
    ap.add_argument("--label", default="",
                    help="variant name for the title (e.g. variant_a)")
    args = ap.parse_args()

    pts = load_pcd_xyz(args.pcd)
    if pts.shape[0] > args.max_points:
        sel = np.random.default_rng(0).choice(pts.shape[0], args.max_points, replace=False)
        pts = pts[sel]

    traj = np.loadtxt(args.tum, ndmin=2)
    tx, ty = traj[:, 1], traj[:, 2]

    # FRAME CHECK: the trajectory must live in the same XY frame as the map
    # (the map-building poses, not raw identity-start odometry). A non-overlap
    # means the wrong trajectory was selected -- fix selection, not this check.
    mlo = (pts[:, 0].min(), pts[:, 1].min()); mhi = (pts[:, 0].max(), pts[:, 1].max())
    tlo = (tx.min(), ty.min()); thi = (tx.max(), ty.max())
    ix = max(0.0, min(mhi[0], thi[0]) - max(mlo[0], tlo[0]))
    iy = max(0.0, min(mhi[1], thi[1]) - max(mlo[1], tlo[1]))
    inter = ix * iy
    traj_area = max((thi[0] - tlo[0]) * (thi[1] - tlo[1]), 1e-9)
    frac = inter / traj_area
    assert inter > 0.0, (
        f"trajectory XY bbox does not overlap map XY bbox -- wrong trajectory "
        f"frame selected. traj x[{tlo[0]:.1f},{thi[0]:.1f}] y[{tlo[1]:.1f},{thi[1]:.1f}] "
        f"vs map x[{mlo[0]:.1f},{mhi[0]:.1f}] y[{mlo[1]:.1f},{mhi[1]:.1f}]")
    print(f"frame check OK: trajectory bbox {frac * 100:.1f}% inside map bbox")

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.hist2d(pts[:, 0], pts[:, 1], bins=args.bins,
              cmap="Greys", norm=LogNorm(), cmin=1)
    ax.plot(tx, ty, "-", color="red", lw=1.5, label="trajectory")
    ax.scatter(tx[0],  ty[0],  c="lime", s=90, edgecolor="k", lw=0.5, zorder=5, label="start")
    ax.scatter(tx[-1], ty[-1], c="blue", s=90, edgecolor="k", lw=0.5, zorder=5, label="end")

    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    prefix = f"{args.label} - " if args.label else ""
    ax.set_title(f"{prefix}top-down map density + trajectory")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(args.out, dpi=150)
    print(f"wrote {args.out}  ({pts.shape[0]:,} map points, {traj.shape[0]:,} poses)")


if __name__ == "__main__":
    main()
