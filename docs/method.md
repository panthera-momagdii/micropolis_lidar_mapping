# Method & Rationale

The pipeline runs from LiDAR alone. RTK GNSS is held back as the evaluation reference, and
used as an explicit prior only in Variant B. Everything is in `base_link` (the Seyond Falcon
LiDAR frame, to which the GNSS/INS receiver is rigidly attached); there is no `/tf`, no IMU,
and no wheel odometry. The site is essentially flat and every bag shares one `FP_ENU0` datum,
so all data is directly mergeable in ENU.

## Variant A — LiDAR-only odometry

- **LiDAR-only, GNSS as reference.** The deliverable is a comparison against the ground-truth
  odometry, so the trajectory must be estimated independently of GNSS feeding `odometry_enu`
  into mapping would make that comparison circular.
- **Constant-velocity deskew.** With no IMU, the only principled motion model: estimate the
  body twist from the previous two poses and apply it per point using each point's absolute
  timestamp. At ~3 m/s over a ~90 ms sweep the uncorrected distortion is ~30 cm, so this is a
  real correction, not a formality.
- **Point-to-plane, scan-to-map ICP.** The scene is structured (ground + facades);
  point-to-plane converges faster and its ground constraint directly resists Z drift.
  Registering against an accumulated voxel map averages noise and slows drift, where
  scan-to-scan would compound it.
- **Voxel-hash local map.** O(1) insert/lookup with bounded memory (per-voxel cap + distance
  crop) — simpler to get right than an incremental kd-tree.
- **Huber kernel.** Downweights moving objects and stray returns without an explicit
  dynamic-object stage.
- **Second-return filter — defensive only.** Split-pulse second returns are geometrically
  unreliable, but this dataset's publisher already removed them (`is_dense = true`), so the
  filter is a no-op kept for robustness on other data.
- **No loop closure.** The traverse never revisits a place, so a loop detector would never
  fire. The Variant B pose graph is loop-ready (one extra edge type) if that changes.

Tightly-coupled LIO (e.g. FAST-LIO2) was not used because its prediction and deskew both
depend on high-rate IMU data, which this dataset does not have.

## Variant B — GNSS pose-graph fusion

A sparse SE(3) pose graph fuses Variant A's relative motion (sequential between-edges) with
**position-only** GNSS unary priors. Each session is seeded into ENU by a Kabsch fit, then
optimized with Gauss-Newton. The GNSS priors pin absolute position and with it the otherwise
unobservable Z and pitch,while orientation between nodes still comes from the LiDAR edges.
The map is regenerated at the optimized poses through the same cloud pipeline as Variant A.

## When a different method would win

| If the data had… | Prefer | Because |
|---|---|---|
| an IMU | tightly-coupled LIO | gravity + high-rate prediction make Z/pitch observable |
| revisited places | loop-closure edges | bounds global drift; the graph is already loop-ready |
| heavy dynamics | explicit dynamic removal | Huber downweights but does not reject |
| no GNSS | place recognition for correction | no absolute reference otherwise |
| faster motion / sparse scans | continuous-time (B-spline) trajectory | the per-scan constant-velocity model breaks down |

> The method is the simplest system the data supports: each omitted component is omitted because
> this dataset cannot exercise it, and each has a known drop-in path.
