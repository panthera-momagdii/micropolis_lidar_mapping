# Architecture & Visual Documentation

Visual reference for the LiDAR mapping pipeline: data flow, module structure, and the
class model for both variants. Diagrams are [Mermaid](https://mermaid.js.org/) and render
directly on GitHub.

- **Variant A** — pure LiDAR odometry (constant-velocity deskew, voxel-hash local map,
  point-to-plane scan-to-map ICP). Produces `trajectory.tum` + `map.pcd`.
- **Variant B** — GNSS pose-graph fusion layer over Variant A's relative motion. Produces
  `fused.tum` + `fused_map.pcd`.

Two ROS 2 packages: `lidar_mapper` (library `lm_core` + tools) and `gnss_fusion`
(library `gf_core` + tools). `gnss_fusion` reuses `lidar_mapper`'s SE(3) math and cloud
pipeline.

---

## 1. Pipeline overview

GNSS is used in two distinct roles: as the **ground-truth reference** for evaluation
(`export_gt` → `gt.tum`), and as an **input prior** to Variant B's fusion. Because Variant B
consumes GNSS, its agreement with GNSS is a georeferencing-fit metric, not independent LiDAR
accuracy. Variant A is the independent measurement.

```mermaid
flowchart LR
    BAGS["MCAP bags<br/>one folder = one session"]
    GNSS["/fixposition/odometry_enu<br/>RTK GNSS (ENU)"]

    subgraph VA["Variant A — LiDAR-only odometry"]
        A1["scan-to-map ICP<br/>deskew + voxel map"]
    end
    subgraph VB["Variant B — GNSS pose-graph fusion"]
        B1["Variant A edges + GNSS priors<br/>sparse Gauss-Newton"]
    end

    TRAJA["trajectory.tum"]
    MAPA["map.pcd"]
    FUSED["fused.tum"]
    FMAP["fused_map.pcd"]
    GT["gt.tum"]
    EVAL["evaluate.py<br/>ATE / RPE / plots"]

    BAGS -->|"/iv_points_fusion"| A1
    A1 --> TRAJA
    A1 --> MAPA
    TRAJA --> B1
    GNSS -->|"prior"| B1
    B1 --> FUSED
    FUSED --> FMAP
    BAGS -.-> GNSS
    GNSS -->|"reference"| GT
    TRAJA --> EVAL
    FUSED --> EVAL
    GT --> EVAL
```

---

## 2. Variant A — LiDAR-only odometry

`run_odometry` resolves the (ordered) bags, anchors the first pose to the matching GNSS pose
`T0` (georeferencing only, the metric uses SE(3) Umeyama, not this anchor), then streams every
`/iv_points_fusion` scan through `Odometry::process`. The local map is a voxel hash; the global
output map is accumulated separately by `WorldMap` (dedup at `v_out`).

```mermaid
flowchart TD
    START["run_odometry<br/>params.yaml, out_dir, ordered bags"]
    RESOLVE["resolve_bags()<br/>ordered list, or globbed dir"]
    T0["first_gt_pose()<br/>T0 anchor (ENU)"]
    INIT["Odometry(params, T0)<br/>+ WorldMap + TumWriter"]
    STREAM["run_bag_stream()<br/>one callback per scan"]

    subgraph PROC["Odometry::process(scan) — per scan"]
        direction TB
        P1["constant-velocity twist xi<br/>from last two poses<br/>(reset if gap > 5 s)"]
        P2["deskew(scan, xi)<br/>per-point, OpenMP"]
        P3["voxel_subsample(v_map)<br/>sensor-frame cloud"]
        P4["register_p2pl()<br/>scan-to-map ICP vs VoxelMap<br/>Gauss-Newton + Huber"]
        P5["update pose T"]
        P6["VoxelMap::insert()<br/>crop local map every 50 scans"]
        P1 --> P2 --> P3 --> P4 --> P5 --> P6
    end

    WMAP["WorldMap::add(cloud, T)<br/>transform then dedup at v_out"]
    OUT1["TumWriter --> trajectory.tum"]
    OUT2["write_pcd --> map.pcd"]

    START --> RESOLVE --> T0 --> INIT --> STREAM --> PROC
    PROC -->|"sensor cloud + pose T"| WMAP
    PROC --> OUT1
    WMAP --> OUT2
```

### 2.1 Scan-to-map ICP inner loop (`register_p2pl`)

Point-to-plane Gauss-Newton against the accumulated voxel map. The per-point work (neighbor
lookup + Jacobian) is parallelized with OpenMP; contributions are reduced **serially in index
order** so the normal equations stay bit-identical regardless of thread count.

```mermaid
flowchart TD
    S["src = deskewed, subsampled<br/>sensor-frame points; estimate T"]
    subgraph IT["per Gauss-Newton iteration (<= max_iters)"]
        direction TB
        L1["for each point p:  q = T * p"]
        L2["VoxelMap::neighbors(q)<br/>3x3x3 cell lookup"]
        L3["pick best planar voxel<br/>(planarity >= min, n >= min)"]
        L4["point-to-plane residual<br/>r = n^T (q - mean)"]
        L5["Huber weight, accumulate<br/>H (6x6), g (6x1)"]
        L6["solve H d = -g (LDLT)<br/>T = T * Exp(d)"]
        L1 --> L2 --> L3 --> L4 --> L5 --> L6
        L6 -->|"step > tol"| L1
    end
    S --> IT --> DONE["converged T<br/>IcpResult"]
```

---

## 3. Variant B — GNSS pose-graph fusion

`run_fusion` reads Variant A's trajectory, streams **only** the GNSS topic from the bags
(skipping cloud bytes), associates each node to the nearest GNSS pose in time, splits the run
into sub-chains at large time gaps, and seeds each chain into ENU with a position-only Kabsch
fit. It then builds a sparse pose graph sequential odometry between-edges plus subsampled
GNSS unary priors — and optimizes. `regen_map` rebuilds the map at the optimized poses using
the *same* cloud pipeline as Variant A.

```mermaid
flowchart TD
    START["run_fusion<br/>bag_dir, trajectory.tum, params_fusion"]
    READ["TumReader::read()<br/>Variant A poses TA + stamps"]
    GNSS["run_bag_stream (GNSS topic only)<br/>GNSS ENU positions"]
    ASSOC["associate node to GNSS<br/>nearest in time, |dt| <= 0.05 s"]
    CHAINS["sub-chains<br/>split where gap > gap_reset_sec"]
    KABSCH["per-chain Kabsch (position only)<br/>enu_from_odom seed"]

    subgraph PG["PoseGraph — sparse Gauss-Newton (ENU)"]
        direction TB
        G1["add_node(T_align * TA_k)"]
        G2["add_odom_edge(k-1, k, Z)<br/>Z = TA_inv * TA (relative)"]
        G3["add_gnss_prior(k, p_k)<br/>every gnss_every_n nodes"]
        G4["optimize():<br/>linearize_odom / linearize_gnss<br/>sparse H,b -> LDLT -> T = T*Exp(d)"]
        G1 --> G2 --> G3 --> G4
    end

    FUSED["fused.tum (ENU)"]
    REGEN["regen_map<br/>deskew + voxel_subsample<br/>+ WorldMap.add at fused poses"]
    FMAP["fused_map.pcd (ENU)"]

    START --> READ --> ASSOC
    GNSS --> ASSOC
    ASSOC --> CHAINS --> KABSCH --> PG
    PG --> FUSED --> REGEN --> FMAP
```

GNSS priors are **position-only** (no orientation). They pin absolute position (and therefore
the otherwise-unobservable Z/pitch); per-node orientation still comes from the LiDAR odometry
edges. This is why Variant B's ATE collapses to centimetres while its RPE (a relative,
orientation-coupled metric) stays comparable to Variant A.

---

## 4. Module & dependency structure

```mermaid
flowchart TB
    subgraph LM["lidar_mapper"]
        LMCORE["lm_core (library)<br/>point_cloud, voxel_map, registration,<br/>deskew, odometry, world_map,<br/>se3, tum_io, pcd_io, bag_stream"]
        RO["run_odometry"]
        EG["export_gt"]
        IB["inspect_bags"]
    end
    subgraph GF["gnss_fusion"]
        GFCORE["gf_core (library)<br/>pose_graph"]
        RF["run_fusion"]
        RM["regen_map"]
    end

    ROS["ROS 2 Jazzy<br/>rosbag2 + mcap storage<br/>sensor_msgs, nav_msgs"]
    EXT["Eigen3 · yaml-cpp · OpenMP"]

    RO --> LMCORE
    EG --> LMCORE
    IB --> LMCORE
    RF --> GFCORE
    RM --> GFCORE
    GFCORE -->|"reuses se3.hpp"| LMCORE
    RF -->|"reuses tum_io"| LMCORE
    RM -->|"reuses cloud pipeline"| LMCORE
    LMCORE --> ROS
    LMCORE --> EXT
    GFCORE --> EXT
```

`gnss_fusion` depends on `lidar_mapper` (`find_package(lidar_mapper)` + link `lm_core`), so
both build in one colcon workspace with `lidar_mapper` first.

---

## 5. Class model — `lidar_mapper`

`Odometry` is the only stateful class (owns the local map, trajectory, and crop counter behind a
small interface). Everything else is plain data (`struct`) or a stateless free function — the
distinction is deliberate: a class where there is an invariant to protect, a struct for data,
free functions for stateless transforms.

```mermaid
classDiagram
    class Odometry {
        <<class>>
        -OdometryParams p_
        -VoxelMap map_
        -Isometry3d T0_
        -vector~OdometryStep~ traj_
        -int scans_since_crop_
        +process(scan, cloud_out) OdometryStep
        +trajectory() vector
        +local_map() VoxelMap
    }
    class VoxelMap {
        <<struct>>
        +double v_size
        +int max_n
        +int min_plane_n
        +double min_planarity
        +unordered_map cells
        +key(x, y, z, v) int64$
        +insert(p)
        +neighbors(p, out)
        +crop(center, radius)
    }
    class Voxel {
        <<struct>>
        +int n
        +Vector3d mean
        +Matrix3d M2
        +Vector3d normal
        +double planarity
    }
    class WorldMap {
        <<struct>>
        +double v_out
        +bool key_from_double
        +unordered_set keys
        +vector~PointXYZT~ pts
        +add(cloud, T)
    }
    class PointXYZT {
        <<struct>>
        +float x
        +float y
        +float z
        +float intensity
        +double t
    }
    class Scan {
        <<struct>>
        +double stamp
        +double t_min
        +double t_max
        +vector~PointXYZT~ points
    }
    class OdometryParams {
        <<struct>>
        +double v_map
        +double v_out
        +double max_corr_dist
        +double huber_delta
        +double gap_reset_sec
    }
    class OdometryStep {
        <<struct>>
        +double stamp
        +Isometry3d T
        +int matched
        +bool gap_reset
    }
    class IcpResult {
        <<struct>>
        +int iters
        +int matched
        +bool converged
    }
    class CloudPipeline {
        <<free functions>>
        +to_scan(msg, scan) bool
        +deskew(scan, xi)
        +voxel_subsample(in, v, out)
        +register_p2pl(src, map, T, ...) IcpResult
    }
    class SE3 {
        <<free functions>>
        +Exp(xi) Isometry3d
        +Log(T) xi
        +Exp_apply_small(xi, p) Vector3d
    }

    Odometry "1" *-- "1" VoxelMap : owns local map
    VoxelMap "1" *-- "*" Voxel
    WorldMap "1" *-- "*" PointXYZT
    Odometry ..> OdometryParams : configured by
    Odometry ..> Scan : consumes
    Odometry ..> OdometryStep : produces
    Odometry ..> CloudPipeline : uses
    Odometry ..> SE3 : uses
    CloudPipeline ..> IcpResult : returns
    CloudPipeline ..> VoxelMap : queries
```

---

## 6. Class model — `gnss_fusion`

`PoseGraph` is a class because it has a real invariant — every edge/prior must reference a valid
node index — enforced at insertion. The per-factor linearization is split into free functions
(`linearize_odom`, `linearize_gnss`) so each analytic Jacobian is unit-testable against finite
differences without any virtual dispatch.

```mermaid
classDiagram
    class PoseGraph {
        <<class>>
        -vector~Isometry3d~ X_
        -vector~OdomEdge~ odom_
        -vector~GnssPrior~ gnss_
        -double w_trans_
        -double w_rot_
        -double gnss_info_
        +set_weights(wt, wr, info)
        +add_node(T) int
        +add_odom_edge(i, j, Z)
        +add_gnss_prior(i, p)
        +step() IterStat
        +optimize(max_iters) vector
        +poses() vector
    }
    class OdomEdge {
        <<struct>>
        +int i
        +int j
        +Isometry3d Z
    }
    class GnssPrior {
        <<struct>>
        +int i
        +Vector3d p
    }
    class IterStat {
        <<struct>>
        +double cost
        +double odom_mean
        +double gnss_mean
        +bool solve_ok
    }
    class Factors {
        <<free functions>>
        +linearize_odom(Xi, Xj, Z) OdomLin
        +linearize_gnss(Xi, p) GnssLin
    }

    PoseGraph "1" *-- "*" OdomEdge
    PoseGraph "1" *-- "*" GnssPrior
    PoseGraph ..> IterStat : produces
    PoseGraph ..> Factors : linearizes via
    PoseGraph ..> SE3 : Exp / Log (from lidar_mapper)
```

---

### Legend

- `<<class>>` — encapsulated state + an invariant (`Odometry`, `PoseGraph`).
- `<<struct>>` — plain data carrier, public fields.
- `<<free functions>>` — stateless transforms grouped by module (not real classes).
- `*--` composition (owns) · `..>` dependency (uses / produces).
