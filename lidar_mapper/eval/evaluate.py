#!/usr/bin/env python3
"""evaluate est tum vs gt tum. writes metrics.json + 3 png; prints aligned table."""
import copy
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from evo.core import metrics, sync
from evo.tools import file_interface


def path_len(xyz):
    if len(xyz) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum())


def cum_dist(xyz):
    if len(xyz) < 2:
        return np.zeros(len(xyz))
    seg = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)])


def rpe_drift(gt, est, delta_m):
    try:
        m = metrics.RPE(metrics.PoseRelation.translation_part,
                        delta=delta_m, delta_unit=metrics.Unit.meters,
                        rel_delta_tol=0.1, all_pairs=False)
        m.process_data((gt, est))
        s = m.get_all_statistics()
        return s["rmse"], s["rmse"] / delta_m * 100.0
    except Exception:
        return float("nan"), float("nan")


def rpe_rot(gt, est, delta_m):
    try:
        m = metrics.RPE(metrics.PoseRelation.rotation_angle_deg,
                        delta=delta_m, delta_unit=metrics.Unit.meters,
                        rel_delta_tol=0.1, all_pairs=False)
        m.process_data((gt, est))
        s = m.get_all_statistics()
        return s["rmse"], s["rmse"] / delta_m
    except Exception:
        return float("nan"), float("nan")


def rotfree_drift(gx, ex, w, tol=0.1):
    # pair by GT path distance; isolate translation magnitude from attitude.
    gd = cum_dist(gx); errs = []; j = 0
    for i in range(len(gd)):
        while j < len(gd) and gd[j] - gd[i] < w: j += 1
        if j >= len(gd): break
        if abs(gd[j] - gd[i] - w) > w * tol: continue
        errs.append(abs(np.linalg.norm(gx[j] - gx[i]) - np.linalg.norm(ex[j] - ex[i])))
    if not errs: return float("nan"), float("nan")
    r = float(np.sqrt(np.mean(np.square(errs))))
    return r, r / w * 100.0


def att_err_deg(gt, est):
    out = np.empty(len(gt.poses_se3))
    for i in range(len(gt.poses_se3)):
        Re = est.poses_se3[i][:3, :3].T @ gt.poses_se3[i][:3, :3]
        out[i] = np.degrees(np.arccos(np.clip((np.trace(Re) - 1) / 2, -1, 1)))
    return out


def main():
    if len(sys.argv) < 4:
        sys.exit("usage: evaluate.py <gt.tum> <est.tum> <out_dir>")
    gt_path, est_path, out_dir = sys.argv[1], sys.argv[2], sys.argv[3]
    out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)

    gt_raw = file_interface.read_tum_trajectory_file(gt_path)
    est_raw = file_interface.read_tum_trajectory_file(est_path)
    gt, est = sync.associate_trajectories(gt_raw, est_raw, max_diff=0.05)
    est_pre = copy.deepcopy(est)  # pre-alignment, for anchored-Z
    R_align, _t_align, _s = est.align(gt, correct_scale=False, correct_only_scale=False)

    diff = est.positions_xyz - gt.positions_xyz  # N×3
    norm = np.linalg.norm(diff, axis=1)
    ate_rmse = float(np.sqrt(np.mean(norm ** 2)))
    ate_mean = float(np.mean(norm))
    ate_max  = float(np.max(norm))
    ate_axis = {k: float(np.sqrt(np.mean(diff[:, i] ** 2))) for i, k in enumerate("xyz")}
    rpe10_rmse, rpe10_pct   = rpe_drift(gt, est, 10.0)
    rpe100_rmse, rpe100_pct = rpe_drift(gt, est, 100.0)
    rot10_rmse, rot10_pm    = rpe_rot(gt, est, 10.0)
    rot100_rmse, rot100_pm  = rpe_rot(gt, est, 100.0)
    rf10_rmse, rf10_pct     = rotfree_drift(gt.positions_xyz, est.positions_xyz, 10.0)
    rf100_rmse, rf100_pct   = rotfree_drift(gt.positions_xyz, est.positions_xyz, 100.0)
    align_angle = float(np.degrees(np.arccos(np.clip((np.trace(R_align) - 1) / 2, -1, 1))))
    align_tilt  = float(np.degrees(np.arccos(np.clip(R_align[2, 2], -1, 1))))
    T0a = gt.poses_se3[0] @ np.linalg.inv(est_pre.poses_se3[0])
    est_anch = (T0a[:3, :3] @ est_pre.positions_xyz.T).T + T0a[:3, 3]
    z_anch = est_anch[:, 2] - gt.positions_xyz[:, 2]
    z_anch_rmse = float(np.sqrt(np.mean(z_anch ** 2)))
    z_anch_lo, z_anch_hi = float(z_anch.min()), float(z_anch.max())
    att = att_err_deg(gt, est)
    att_mean, att_max = float(att.mean()), float(att.max())
    lens = {
        "est":          path_len(est.positions_xyz),
        "gt_raw":       path_len(gt_raw.positions_xyz),
        "gt_resampled": path_len(gt.positions_xyz),
    }

    m_json = {
        "n_matched_poses": int(len(gt.positions_xyz)),
        "ate_rmse_m": ate_rmse, "ate_mean_m": ate_mean, "ate_max_m": ate_max,
        "ate_x_rmse_m": ate_axis["x"], "ate_y_rmse_m": ate_axis["y"], "ate_z_rmse_m": ate_axis["z"],
        "rpe_10m_rmse_m": rpe10_rmse,  "rpe_10m_drift_pct":  rpe10_pct,
        "rpe_100m_rmse_m": rpe100_rmse, "rpe_100m_drift_pct": rpe100_pct,
        "rpe_10m_rot_rmse_deg":  rot10_rmse,  "rpe_10m_rot_deg_per_m":  rot10_pm,
        "rpe_100m_rot_rmse_deg": rot100_rmse, "rpe_100m_rot_deg_per_m": rot100_pm,
        "rotfree_drift_10m_pct":  rf10_pct,  "rotfree_drift_10m_rmse_m":  rf10_rmse,
        "rotfree_drift_100m_pct": rf100_pct, "rotfree_drift_100m_rmse_m": rf100_rmse,
        "umeyama_angle_deg": align_angle, "umeyama_tilt_deg": align_tilt,
        "anchored_z_rmse_m": z_anch_rmse,
        "anchored_z_min_m": z_anch_lo, "anchored_z_max_m": z_anch_hi,
        "attitude_err_mean_deg": att_mean, "attitude_err_max_deg": att_max,
        "path_est_m":          lens["est"],
        "path_gt_raw_m":       lens["gt_raw"],
        "path_gt_resampled_m": lens["gt_resampled"],
    }
    (out / "metrics.json").write_text(json.dumps(m_json, indent=2))

    fmt = "{:<28} {:>12}"
    print(f"=== {est_path}  vs  {gt_path} ===")
    print(fmt.format("matched poses",       f"{m_json['n_matched_poses']}"))
    print(fmt.format("ATE RMSE [m]",        f"{ate_rmse:.3f}"))
    print(fmt.format("ATE mean [m]",        f"{ate_mean:.3f}"))
    print(fmt.format("ATE max  [m]",        f"{ate_max:.3f}"))
    print(fmt.format("ATE x RMSE [m]",      f"{ate_axis['x']:.3f}"))
    print(fmt.format("ATE y RMSE [m]",      f"{ate_axis['y']:.3f}"))
    print(fmt.format("ATE z RMSE [m]",      f"{ate_axis['z']:.3f}"))
    print(fmt.format("RPE @10m RMSE [m]",   f"{rpe10_rmse:.3f}"))
    print(fmt.format("RPE @10m drift [%]",  f"{rpe10_pct:.3f}"))
    print(fmt.format("RPE @100m RMSE [m]",  f"{rpe100_rmse:.3f}"))
    print(fmt.format("RPE @100m drift [%]", f"{rpe100_pct:.3f}"))
    print(fmt.format("RPE @10m rot [deg/m]",  f"{rot10_pm:.4f}"))
    print(fmt.format("RPE @100m rot [deg/m]", f"{rot100_pm:.4f}"))
    print(fmt.format("rot-free drift @10m [%]",  f"{rf10_pct:.3f}"))
    print(fmt.format("rot-free drift @100m [%]", f"{rf100_pct:.3f}"))
    print(fmt.format("Umeyama R total [deg]", f"{align_angle:.2f}"))
    print(fmt.format("Umeyama tilt  [deg]",   f"{align_tilt:.2f}"))
    print(fmt.format("anchored Z RMSE [m]",   f"{z_anch_rmse:.3f}"))
    print(fmt.format("anchored Z range [m]",  f"{z_anch_lo:.2f}..{z_anch_hi:.2f}"))
    print(fmt.format("attitude err mean [deg]", f"{att_mean:.2f}"))
    print(fmt.format("attitude err max  [deg]", f"{att_max:.2f}"))
    print(fmt.format("path est [m]",        f"{lens['est']:.2f}"))
    print(fmt.format("path GT raw [m]",     f"{lens['gt_raw']:.2f}"))
    print(fmt.format("path GT resamp [m]",  f"{lens['gt_resampled']:.2f}"))
    print("GT noise floor ~ 5 cm (RTK covariance)")

    gt_xyz = gt.positions_xyz
    est_xyz = est.positions_xyz
    d = cum_dist(est_xyz)

    # (a) XY overlay
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(gt_xyz[:, 0], gt_xyz[:, 1], "k-", lw=1.0, label="GT")
    ax.plot(est_xyz[:, 0], est_xyz[:, 1], "r-", lw=1.0, label="est (aligned)")
    ax.plot(gt_xyz[0, 0], gt_xyz[0, 1], "go", ms=9, label="start")
    ax.plot(gt_xyz[-1, 0], gt_xyz[-1, 1], "rs", ms=9, label="end")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_aspect("equal"); ax.grid(True, alpha=0.3); ax.legend(loc="best")
    ax.set_title("XY overlay (Umeyama SE(3) aligned)")
    fig.tight_layout(); fig.savefig(out / "xy_overlay.png", dpi=140); plt.close(fig)

    # (b) per-axis error vs distance
    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    for i, k in enumerate("xyz"):
        axes[i].plot(d, diff[:, i], "b-", lw=0.7)
        axes[i].axhline(0, color="k", lw=0.5)
        axes[i].set_ylabel(f"{k} err [m]")
        axes[i].grid(True, alpha=0.3)
    axes[2].set_xlabel("distance along est [m]")
    axes[0].set_title("per-axis error vs distance")
    fig.tight_layout(); fig.savefig(out / "per_axis_err.png", dpi=140); plt.close(fig)

    # (c) Z trajectory
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(d, gt_xyz[:, 2], "k-", lw=1.0, label="GT z")
    ax.plot(d, est_xyz[:, 2], "r-", lw=1.0, label="est z (aligned)")
    ax.set_xlabel("distance along est [m]"); ax.set_ylabel("z [m]")
    ax.grid(True, alpha=0.3); ax.legend(loc="best")
    ax.set_title("z trajectory vs distance")
    fig.tight_layout(); fig.savefig(out / "z_vs_distance.png", dpi=140); plt.close(fig)

    # (d) attitude error vs distance
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(d, att, "b-", lw=0.7)
    ax.set_xlabel("distance along est [m]"); ax.set_ylabel("attitude err [deg]")
    ax.grid(True, alpha=0.3); ax.set_title("attitude error vs distance (after Umeyama)")
    fig.tight_layout(); fig.savefig(out / "attitude_err.png", dpi=140); plt.close(fig)

    print(f"figures + metrics.json -> {out}")


if __name__ == "__main__":
    main()
