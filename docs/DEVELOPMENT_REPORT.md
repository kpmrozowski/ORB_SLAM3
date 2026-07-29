# Monocular-Inertial VIO for GPS-Denied / GPS-Spoofed Aerial Flight
## Two-Week Development & Research Report

**Period:** 2026-07-15 → 2026-07-29
**Branch:** `dev` (12 commits ahead of `master`), fork of ORB-SLAM3
**Platform:** ORB-SLAM3 monocular-inertial, offline deterministic evaluation
**Author:** Kornel Mrozowski

---

### Abstract

This report documents two weeks of development and experimentation aimed at turning stock ORB-SLAM3 monocular-inertial into a usable visual-inertial odometry (VIO) estimator for **fixed-wing / multirotor survey flights where GNSS is denied or actively spoofed**. The dominant operating regime — high-altitude cruise (24–130 m AGL) with weak accelerometer excitation and little parallax — breaks stock ORB-SLAM3's inertial initialization and lets monocular scale drift unbounded. The work adds **altitude (barometer / rangefinder) and heading (magnetometer) tight fusion**, hardens **place-recognition and loop closing**, makes **initialization gates configurable**, introduces a fully **deterministic evaluation mode**, and adds an **early-divergence guard** for efficient batch campaigns. Every change is environment-variable-gated and defaults to exact upstream behavior. The single largest quantitative win is tight-σ barometer fusion (flight 542 vertical ATE **5.37 m → 1.15 m**, beating the vehicle's own EKF at 1.42 m). Two walls remain open: **monocular horizontal scale drift** (only loop closure or an absolute anchor removes it) and the **catastrophic-initialization lottery** at altitude.

---

## Table of Contents

1. [Objective & Scope](#1-objective--scope)
2. [Datasets](#2-datasets)
3. [Evaluation Methodology & Metrics](#3-evaluation-methodology--metrics)
   - 3.1 [Trajectory output and clock](#31-trajectory-output-and-clock)
   - 3.2 [Coverage](#32-coverage)
   - 3.3 [Absolute Trajectory Error (ATE)](#33-absolute-trajectory-error-ate)
   - 3.4 [Vertical ATE (vs barometer)](#34-vertical-ate-vs-barometer)
   - 3.5 [Umeyama similarity alignment — which points are used](#35-umeyama-similarity-alignment--which-points-are-used)
   - 3.6 [GPS segment errors](#36-gps-segment-errors)
   - 3.7 [Endpoint gap](#37-endpoint-gap)
   - 3.8 [Scale drift and height drift](#38-scale-drift-and-height-drift)
   - 3.9 [Composite J-score](#39-composite-j-score)
   - 3.10 [Ground-truth sources](#310-ground-truth-sources)
   - 3.11 [Divergence classification](#311-divergence-classification)
4. [Changelog vs `master`](#4-changelog-vs-master)
5. [Quantitative Results & Improvements](#5-quantitative-results--improvements)
6. [Negative Results & Divergences](#6-negative-results--divergences)
7. [Open Problems & Future Work](#7-open-problems--future-work)
8. [Appendix A — Environment-variable reference](#appendix-a--environment-variable-reference)
9. [Appendix B — Research diary index](#appendix-b--research-diary-index)

---

## 1. Objective & Scope

Deliver a monocular-inertial VIO that produces **metrically-scaled, gravity-aligned, absolute-north-referenced** trajectories on aerial survey footage **without relying on GNSS**, because the operational GPS stream on these vehicles is routinely spoofed. Concretely:

- **Metric vertical channel** from the onboard barometer (and, where valid, rangefinder), so monocular scale is observable even during weak-excitation cruise.
- **Absolute heading** from the magnetometer, so the map is north-referenced for downstream GPS-denied navigation.
- **Robust initialization and drift control** so long high-altitude flights track end-to-end instead of resetting or diverging.
- **Bit-reproducible offline evaluation**, so A/B experiments over dozens of flights are trustworthy.

All estimator modifications are **env-gated and default-OFF**: with no `ORB_*` variables set, the binary reproduces stock ORB-SLAM3 behavior (defaults chosen to reproduce upstream constants exactly). This keeps the fork rebase-friendly and every experiment reversible.

---

## 2. Datasets

Two families were used: **NORA full-resolution flights** (primary tuning targets, rich sensor suite) and a **historical fleet** of 52 archived flights (breadth / robustness). EuRoC appears only as the reference data layout (`convert_*_to_euroc.py` produces the EuRoC directory format consumed by `mono_inertial_euroc`); no EuRoC benchmark numbers are reported here.

### 2.1 NORA primary flights (golem27, 2026-07-09)

| Flight | Frames / duration | Regime | Sensors | Notes |
|---|---|---|---|---|
| **538** | 4197 fr / 554.5 s | ~100–130 m AGL over **relief** terrain, ~6.5 km survey | IMU (ArduPilot RISI, 400 Hz), GPS, barometer, rangefinder (radar-class, valid to 138.6 m), magnetometer | Native 1640×1232 grayscale; **0 corrupt frames**; takeoff t≈21.9 s. The hard case (long, high, low-parallax). |
| **542** | 1267 fr / 134.1 s | ~24 m AGL over **near-flat** ground, ±2.5 m/s verticals | same suite | Takeoff t=27.7 s ⇒ 77 % pose coverage = **100 % of the airborne span**. **Dataset defect:** 56 zero-byte source PNGs abort stock ORB-SLAM3 at ~frame 1212 (fixed by skipping 0-byte PNGs in the frame lister). The reachable case. |

Both were also processed at an 800-px downscale (`nora538_half`, `nora542_half`) — the **working baseline resolution** — and with an offline-CLAHE variant. Excitation was validated (accel-magnitude std ratio pre/post motion-start: 538 ×5.9, 542 ×8.1).

### 2.2 NORA golem27_home flights (data-recorder / "circle" format)

| Flight | Frames / duration | Outcome |
|---|---|---|
| 165608 | 3325 fr / 223.6 s | Barometer OK (Δ32.7 m); reproducible FullInertialBA segfault (upstream vertex-null); best draw vertical ATE 0.64 m @ 63 % but v-scale 0.67. |
| 162710 | 1927 fr / 128.8 s | Low altitude, rich texture, in-range terrain — **the cleanest "home" win**: vertical ATE **0.44–0.48 m** @ ~98 % coverage. |
| 163613 | 878 fr / 59.3 s | IMU/baro truncated at ~60 s while the camera runs 205 s — **unusable**. |
| circle_move | 2935 fr / 148.4 s | Circle flight, 800×600 native, **no baro, no mag**. First dataset to meet the endpoint-gap target (see §5.5). |

### 2.3 Historical fleet (archived BIN-log flights)

From 59 candidate sessions, strict criteria (a BIN log with RISI-preintegrated IMU and at least one authentic GPS-instance-1 3-D fix within 1 km of the test field 50.41020638 N, 29.93702530 E) yielded **7 strictly-runnable** flights; a user-provided selection expanded this to **52 datasets** converted to the evaluation layout. Most are **300×225 @ 5 fps** (downsampled from 800×600); a few are 1640×1232. IMU comes from the BIN RISI stream. Flights span fleets golem17/golem27/golem26/golem28/golem23/epos/papa3/alma231.

**Critical GPS caveat.** On these vehicles GPS **instance 0** is the *spoofed* receiver and GPS **instance 1** is the *authentic* receiver. ("GPS[0]"/"GPS[1]" throughout this codebase mean receiver **instances**, not sample indices.) Ground-truth scoring therefore masks GPS-instance-0 samples to those that agree with instance-1, and only ~6 of ~15 covered flights are scoreable against GPS at all (the rest are spoofed from takeoff).

---

## 3. Evaluation Methodology & Metrics

All estimated trajectories are TUM-format `f_<tag>.txt` files (`t tx ty tz qx qy qz qw`, `t` in rebased nanoseconds) written by `SaveTrajectoryEuRoC` — **one row per tracked frame** (not keyframes). Keyframe files `kf_<tag>.txt` exist but are **not** used by the GPS / J scorers. The SLAM world frame is gravity-aligned and **z-DOWN**; ENU = `Rz(yaw)·diag(1,−1,−1)·SLAM`.

### 3.1 Trajectory output and clock
BIN sensor `timeus` and camera stamps are rebased to a common epoch via a per-dataset linear "bridge" (`slope`, `k_seconds`, `epoch_ns` in `dataset_meta.json`); all references and the trajectory share this rebased clock. Loaders convert ns→s when `|t| > 1e7`.

### 3.2 Coverage
`coverage % = 100 · (poses in f_<tag>.txt) / (lines in cam0_times.txt)` — saved trajectory poses over total camera frames (`score_trial.py`, `det_queue.sh`, `eval_traj.py` agree). Scorer thresholds: **< 25 % ⇒ diverged**, **< 80 % ⇒ low-coverage penalty**. For a motion-init VIO, pre-takeoff static frames are unreachable, so e.g. flight 542's ~77 % is 100 % of the airborne span.

### 3.3 Absolute Trajectory Error (ATE)
Generic TUM/Horn ATE (`ORB_SLAM3/evaluation/evaluate_ate_scale.py`): trajectories associated by **greedy nearest-timestamp within 20 ms** (`associate.py`), aligned by the closed-form Horn/Umeyama similarity transform (centroid removal, SVD of the cross-covariance `W = Σ modelᵢ⊗dataᵢ`, reflection fix, scale `s = Σ data·(R·model)/Σ|model|²`). Reported as **RMSE of the per-point translational residual** after alignment; both a rigid (no-scale) RMSE and a with-scale RMSE are emitted. Points fed in are **every matched pose pair**.

### 3.4 Vertical ATE (vs barometer)
`baro_ate.py::vertical_ate` — a **1-DoF vertical** error, not a full 3-D ATE. It recovers the SLAM "up" direction as a unit 3-vector by least-squares (`baro = positions·w + b`, `up = w/|w|`, vertical scale `= |w|`), references both SLAM height and barometer to the first matched sample, and reports:
- **`ATE_metric` (metric RMSE)** — `sqrt(mean(((h−h₀) − (baro−baro₀))²))`, **no scale fit** (true absolute vertical error). This is the vATE used everywhere below and in the J-score.
- `ATE_shape` — RMSE after an additional optimal scale+offset fit (shape-only).

### 3.5 Umeyama similarity alignment — which points are used
This was a recurring source of confusion, so it is stated precisely. The horizontal / GPS scoring pipeline (`eval/hist_gps_segment_errors.py`) uses a **hybrid alignment**, not a single global Umeyama:

- **Source points = the full per-frame estimated trajectory** `f_<tag>.txt` (all poses, `xyz` columns) — **not** keyframes, **not** a subset.
- **Target points = GPS-instance-0 ENU**, restricted to the **unspoofed mask** (samples where instance-0 agrees within 30 m of the time-interpolated authentic instance-1 fix).
- **Association** = GPS is **linearly time-interpolated onto each trajectory stamp** (`np.interp`), keeping a pose only if a real GPS fix lies **within 1.5 s** of it and the stamp is inside the GPS time span.
- **Rotation (yaw)** is taken from the **magnetometer**, not from Umeyama: tilt-compensated true-north compass heading minus SLAM body yaw, circular-median over a 30 s alignment window (declination +9.3° E for the test field).
- **Tilt (roll/pitch only)** comes from a **full-flight free Umeyama** `umeyama(slam_all, gps_all)` (requires ≥100 correspondences and ≥300 m GPS path) — but its yaw is **discarded** and replaced by the magnetometer yaw.
- **Scale** = fixed-rotation least squares `scale_g = Σ(R·slam)·gps / Σ|slam|²`, rejected outside `[0.2, 5.0]`.
- **The scored per-segment metrics** come from a **fresh full sim(3) Umeyama on each 60 s segment** of the globally-aligned trajectory vs its GPS segment (residual rotation, centroid offset, residual scale).

The Umeyama itself (`umeyama()`, SVD of the cross-covariance with reflection fix) **does estimate scale** (full similarity). Because translation is removed by centroid subtraction, the **ENU origin choice is irrelevant** to the result — there is **no "first GPS point as origin"** dependence. `validate_nora.py` provides an independent `umeyama_sim3` for cross-checking (three independent scale estimates: GPS-Sim3, baro, rangefinder).

### 3.6 GPS segment errors
Per 60 s segment after takeoff (≥30 poses required), after global alignment (§3.5), a residual Umeyama yields:
- **`rot_deg`** — residual rotation angle `acos((tr(R)−1)/2)` (≈ yaw drift),
- **`horiz_m`** — horizontal centroid offset between aligned-SLAM and GPS segment centroids,
- **`scale_abs` / `scale_pct = |s−1|·100`** — residual scale, evaluated **only on motion-rich segments** (GPS path ≥ 200 m; required band `(0.95, 1.1)`).

Per-flight **percentiles p10/p25/p50/p75/p90** of each are the reported statistics.

### 3.7 Endpoint gap
`endpoint_gap.py` — the prime single-number closure metric for these takeoff-lands-on-takeoff flights: `gap = ‖median(positions in last 1 s) − median(positions in first 1 s)‖`. It is **frame-invariant** (needs no GT alignment) but **valid only for reset-free single-map runs** (after a map reset the trajectory continues in a new frame). The scorer withholds it unless `frame_stats.csv` confirms the run was reset-free. Targets: **< 5 m** (538-class), **< 2 m** (542-class).

### 3.8 Scale drift and height drift
- **Scale drift** is captured by the global `align_scale` and the per-segment `scale_pct` (p50/p90) — segment-to-segment scale variation *is* the drift signal.
- **Height drift** (`height_drift.py`): per 10 s window, `window_drift = ΔVIO_height − ΔBaro`; `cumulative = h − baro`. Reported as per-window drift RMS, max|drift|, final cumulative.

### 3.9 Composite J-score
`eval/ic_tune/score_trial.py`, **lower is better**. Normalizers: 10° rotation, 20 m translation, 5 % scale (≈ 1.0 at "just tolerable").

- **J_v1** = `mean_scored(rot_p50/10 + transl_p50/20 + |scale−1|_p50/0.05)` + `0.3·mean_scored(p90 terms)` + penalties `5·n_low_coverage + 20·(n_diverged + n_no_init + n_unscoreable)`.
- **J_v2** (default, category-aware): per flight — `no_init → 30`, `diverged (cov<25 %) → 25`, else `scored = umeyama_part + vATE/10 + gap/100 + (5 if cov<80 %)`, where each part is 0 when its metric is absent/withheld for that flight category. Campaign J_v2 = mean over all flights. An invariant (unit-tested) guarantees `no_init(30) > diverged(25) > any tracked flight`, so the objective can never reward divergence. Flight categories (no-GPS / spoofed / with-GT) decide which metrics apply.

### 3.10 Ground-truth sources

| Source | Path / units | Frame |
|---|---|---|
| GPS-instance-0 (spoofed rx) | `…_ref_gps0.csv` `#ts[ns],e,n,u,valid` | Local ENU metres (equirectangular, R=6371000, about the median lat/lon), `valid` = spoof mask |
| GPS-instance-1 (authentic rx) | `…_ref_gps1_enu.csv` | Same shared ENU frame, `valid=1` |
| Barometer | `ref_baro.csv` `#ts[ns],alt_m` | **Height above takeoff** (m), referenced to ~0 at start |
| Attitude | `ref_att.csv` `#ts[ns],roll_rad,pitch_rad` | Body roll/pitch (rad); used for rangefinder & magnetometer tilt compensation |
| Rangefinder | `ref_rfnd.csv` `#ts[ns],dist_m` | Slant range → `AGL = dist·cos(roll)·cos(pitch)` |
| Magnetometer | BIN cache npz | Tilt-compensated true-north heading; declination +9.3° E |

### 3.11 Divergence classification
Bounded if `max|position| < 1000 m`; **DIVERGED if > 5000 m** (`06-statistics.md`). Front-end match metrics: `detections` = raw ORB/FAST corners before the nFeatures cap; `matches_inliers` = features matched to the local map; `%frames<30` = fraction of frames with < 30 map matches. "Correct matches" (front-end audit) = Lowe-ratio 0.75 + RANSAC-homography-verified consecutive-pair matches on undistorted fisheye imagery.

---

## 4. Changelog vs `master`

Twelve commits, spanning 33 files (+2191 / −174). New files: `include/BaroFusion.h`, `include/MagFusion.h`, `include/FeaturePrefetcher.h`, `src/FeaturePrefetcher.cc`, `include/DeterministicOrder.h`. **38 distinct `ORB_*` env knobs** introduced, every one defaulting to stock behavior (full reference in [Appendix A](#appendix-a--environment-variable-reference)).

### (A) Barometer / magnetometer tight fusion + frame-rate baro edge — `1dd3881`
The single large commit. Three new g2o edge types and their wiring:
- **`EdgeBaroZ`** — unary z constraint on `VertexPose`, residual `r = twb.z − alt`, analytic Jacobian `∂r/∂ut = Rwb.row(2)`. Added in `LocalInertialBA` and `FullInertialBA` against a **gauge-safe datum**: the z−baro offset of the newest **fixed anchor** keyframe, else the window median (recomputed every optimization so it survives global similarity updates). **Quadratic (no robust kernel)** — the barometer is outlier-free — with an **insertion gate** `ORB_BARO_GATE` skipping any KF whose current inconsistency exceeds the gate (relocalization/datum breakage, not drift).
- **`EdgeBaroScaleGDir`** — init-time binary edge on `(VertexGDir, VertexScale)`; residual `s·upᵀ(twbᵢ − twb_ref) − Δbaro`. Added inside `InertialOptimization(Rwg, scale)` (poses fixed), so **the map is born with a metric vertical channel** even without accelerometer excitation — the key to high-altitude scale observability.
- **`EdgeMagYaw`** — unary yaw constraint, residual `NormalizeAngle(atan2((Rwb·m_body).y, .x) − yaw_ref)`, gated on VIBA2 maturity (`GetIniertialBA2()`) and skipping near-vertical-field / >45° breakage cases. Plus **compass-aligned IMU init** (pure-yaw world rotation so the horizontal field lands on world +x = magnetic north).
- **`AddFrameBaroEdge`** — a **frame-rate** baro z edge in the tracking-thread pose optimizers (`PoseInertialOptimizationLastKeyFrame/LastFrame`), causal relative datum, gated by `ORB_BARO_FRAME_SIGMA`. Purpose: carry the fragile pre-VIBA2 phase through init without reset-thrash.
- **Periodic baro `ScaleRefinement`** in `LocalMapping` (`ORB_BARO_SCALEREF_S`) re-runs the `(Rwg, scale)` optimization every N s so vertical scale cannot drift after upstream's one-off refinement.

### (B) Feature prefetch — `1dd3881`
ORB extraction depends only on image + extractor params, so it is precomputed off the tracking thread for offline runs. `FeaturePrefetcher` owns a worker pool, each with a cloned `ORBextractor` and CLAHE instance, mirroring `GrabImageMonocular` exactly. Consumed via pointer-identity on the NORMAL extractor only (the 5× init extractor always runs inline). Bit-exact to inline extraction; ~2.4× faster iteration (1211 frames 130 s → 55 s). Knobs `ORB_PREFETCH`, `ORB_NO_PACE`, `ORB_VIEWER`.

### (C) Place-recognition / loop-closing robustness — `1dd3881`, `333d23a`, `a18ca5a`, `87e18ea`, `fa4b625`, `451001c`, `4b73997`
Loop **detection** (not correction) is the bottleneck on survey flights; the canonical landing-over-takeoff revisit never fired under stock thresholds. The commits instrument the cascade, localize its death at Sim3 RANSAC, and add safe floors:
- **Env-overridable acceptance tolerances** `ORB_LOOP_RP_TOL` / `ORB_LOOP_YAW_TOL` (stock 0.008 / 0.349 rad).
- **Unconditional 4-DoF roll/pitch lock** (`333d23a`): on every accepted inertial loop, force roll=pitch=0, scale=1, keep yaw+translation — gravity is observable in any inertial map, so a loop must never tilt the world off the IMU vertical. This is the **safety net** that makes the relaxations below safe.
- **`ORB_PR_*` sensitivity** (`a18ca5a`): configurable BoW candidate count, coincidence count, and a global multiplier on every intra-map match/inlier threshold (merge/multi-map thresholds untouched).
- **Cascade instrumentation** (`87e18ea`, `fa4b625`): prints per-gate BoW/Sim3/projection counts and exposes `Sim3Solver` correspondence / best-inlier counts — the diagnostic autopsy showing the cascade dies at Sim3 RANSAC despite 100+ BoW matches.
- **`ORB_PR_SIM3_MININL`** (`451001c`): a Sim3-RANSAC minimum-inlier floor. Measured usable correspondences N = 4–15 (median ~8, because at nf5000 only ~10 % of features are both-sides-mapped) vs stock minimum 15 ⇒ RANSAC aborted before a single iteration — the exact zero-detection mechanism.
- **`ORB_PR_FREE_SCALE`** (`4b73997`): free-scale Sim3 even post-VIBA2 (monocular horizontal scale drifts along the flight, so a fixed-scale hypothesis fits no true match set at the two loop ends). Safe because the 4-DoF lock discards the estimated scale anyway.

### (D) IMU / mono initialization gating — `c2ca050`, `8d8268f`
Makes stock hard-coded init gates env-overridable so short/aggressive flights initialize:
- Inertial-init schedule: `ORB_IMU_INIT_MINTIME/MINKF`, `ORB_IMU_VIBA1_S/VIBA2_S` (also scales the low-motion reset window).
- Pre-IMU-init tracking floor `ORB_TRACK_MININL_PREIMU` (post-init stays 15).
- Two-view reconstruction `ORB_TVR_SIGMA/MINPARALLAX/GOODFRAC`.
- Monocular-init gates `ORB_MINIT_MINMATCHES/WINDOW_PX`.

### (E) Deterministic sequential mode + pointer-order-free containers — `24c0738`
Two independent mechanisms:
1. **Pointer-order independence (both modes, semantics unchanged):** an `IdLess` comparator orders all pointer-keyed containers (`Map`, `MapPoint::mObservations`, `KeyFrame` covisibility/children/loop/merge sets and their getters) by `mnId`, and covisibility weight-tie sorts break ties by `mnId`. Heap addresses are not reproducible run-to-run, so any pointer-ordered iteration feeding g2o insertion order changed the floating-point summation order. Plus **`EIGEN_DONT_VECTORIZE`** (Eigen's vectorized reductions peel to heap-alignment boundaries → run-varying split).
2. **`ORB_DETERMINISTIC` sequential mode:** mapping/loop-closing threads are **not spawned**; their queues drain synchronously after each tracked frame (`SpinOnceDeterministic`), GBA runs inline, reset/stop handshakes are synchronous, OpenCV forced single-threaded. This removes the in-process g2o race (so parallel experiment *processes* are safe) and yields bit-reproducibility.

### (F) Early-divergence guard — `72b277b`
`ORB_DIVERGE_VMAX` / `ORB_DIVERGE_COUNT`: after IMU init, N consecutive keyframes with `|velocity| > VMAX` flag divergence (`System::SetDiverged`); the example binary stops the feed and saves the partial trajectory. Turns ~40-min wasted runs into ~2–5 min.

### (G) Misc fixes & preprocessing — `1dd3881`, `829cba2`
- **`829cba2`**: fixes an **uninitialized `Map* pBiggerMap`** in all four trajectory savers (empty-map runs dereferenced garbage → shutdown segfault) and a bracket bug that ran `FullInertialBA` **twice** when `ORB_DET_DEBUG` was unset.
- Preprocessing/instrumentation: `ORB_CLAHE`/`ORB_CLAHE_TILE` (contrast equalization before extraction), `ORB_MASK_FRAC`/`ORB_MASK_FILE` (sky/keep masks), `ORB_STATS_CSV` (per-frame stats), `ORB_TSJUMP_S` (frame-gap reset threshold — lets IMU preintegrate across multi-second camera drops instead of fragmenting the map), and ORBextractor feature-count diagnostics.

---

## 5. Quantitative Results & Improvements

### 5.1 Barometer fusion — the biggest quantitative win
**σ ladder on flight 542** (nf5000 + CLAHE3, gate 6; ground truth = real barometer; the vehicle's own EKF scores vertical ATE **1.42 m** / drift RMS **1.44 m** on the same span):

| `ORB_BARO_SIGMA` [m] | vertical ATE median [m] | drift RMS median [m] |
|---|---|---|
| 1.0 | 5.37 | 2.52 |
| 0.3 | 2.55 | 2.13 |
| 0.15 | 1.28 | 1.88 (5/5 full coverage) |
| **0.075** | **1.15** (best draws 0.94–0.97) | 1.53 |
| 0.05 | 1.70 (too stiff — floors out) | 1.64 |

**542 goal met:** σ=0.075 vertical ATE 1.15 m **beats** the vehicle EKF's 1.42 m; drift RMS at par. The key physics result is the **fusion-lag law**: `EdgeBaroZ` reproduces the barometer **late**, with lag ∝ σ (σ1.0 → ~1.4 s, σ0.3 → ~0.5 s, no-baro → 0), measured by shift-scanning the GT against each trajectory; on 542's fast climbs this lag was most of the vertical error.

**538 transfer** (same-span EKF vertical ATE 0.98 m): σ0.075 cut healthy-draw vertical ATE from **20.2 m → 1.81–12 m** (median ~7) and repaired vertical scale 0.71 → ≈1.0 — the closest 538 ever got — at the cost of a **~30 % catastrophic-init rate**. The **frame-rate baro edge** (`ORB_BARO_FRAME_SIGMA=0.5`) is the 538 stabilizer: catastrophic rate **37 % → 10 %**, 9/10 draws healthy, by pinning z through the fragile pre-VIBA2 phase.

**Recommended recipes:** 542 = nf5000 + CLAHE3 + `ORB_BARO_SIGMA=0.075–0.15` + gate 6 (+ `ORB_BARO_FRAME_SIGMA=0.5` for reliability); 538 = same + σ0.075 + frame σ0.5. Home flight 162710: vertical ATE **15.7 m → 0.44–0.48 m**.

### 5.2 Magnetometer fusion — marginal accuracy, decisive for absolute north
Tightly-coupled `EdgeMagYaw` moved 538 horizontal RMSE only **292 → 287 m** and left 542 unchanged — because the residual horizontal error is monocular **scale** drift, not yaw. The value is elsewhere: it confirms VIO attitude is genuinely good (heading drift bounded ±20° over 9 min; post-alignment yaw ~1e-7°), provides **absolute north** for GPS-denied use, and improves **map longevity** (538: 98 % coverage, 3× fewer resets). A magnetometer-σ sweep over historical flights found **σ = 10°** optimal (σ2° collapses tracking on some flights). Mag σ10 **rescued five flights from divergence**:

| Flight | Coverage | Endpoint gap |
|---|---|---|
| 153_golem27 | 13.5 % → 95.7 % | — |
| 243_golem17 | 31 % → 99.4 % | 2034 m → 39 m |
| 357_golem17 | 53.5 % → 97.2 % | 1800 m → 16 m |
| 182_golem27 | 82 % → 98 % | 1208 m → 50 m |
| 111_papa3 | 84.5 % → 98.3 % | 338 m → 39 m |

Fleet: runs with >80 % coverage rose 15 → 17.

### 5.3 Rangefinder fusion — win on flat, regression over relief
`BaroFusion` is altitude-source-agnostic, so `ORB_BARO_CSV` can point at the rangefinder. The sensor is radar-class (valid to 138.6 m; the "out of range at altitude" hypothesis was false) but measures **AGL, not altitude**:

| Flight | rangefinder (vATE / drift RMS) | barometer | vertical scale |
|---|---|---|---|
| 542 (flat, 24 m AGL) | **4.45 / 2.36** | 5.37 / 2.52 | 1.0 |
| 538 (relief, terrain dips) | 28.01 / 5.20 | 20.21 / 3.27 | 0.54 (vs 0.71) |

Rangefinder wins on 542, **collapses scale to 0.54 on 538** (rising terrain corrupts the vertical reference). **Tilt correction** (`AGL = slant·cos(roll)·cos(pitch)`) is not negligible — tilt p50 15–17°, slant excess ~6 m at 538 cruise — but its run-level effect is within draw noise. A **fused baro+rangefinder** reference (rangefinder below 10 m AGL, barometer above 80 m) removed 542's −6.9 m landing dip (closed dz to −0.15 m in a v-scale-1.0 draw) but was **harmful on 538** (climb-phase tilt correction reshapes the early map, endpoint gap 6× worse).

### 5.4 Front-end extraction — CLAHE is load-bearing
CLAHE3 + nFeatures 5000 (800-res) lifted RANSAC-verified correct matches per stride-1 pair to median **608** on 538 (53.6 % ≥ 500; the rest is physically featureless terrain) and **1364** on 542 (87.9 % ≥ 500), vs 70 / 21 for the old front-end. CLAHE is load-bearing: the circle flight closes to **0.27 m** with it vs a **200 m** gap without. An adaptive-threshold + IMU-guided-matching front-end reached ≥ 500 matches on **99.5 % (538) / 98.7 % (542)** of all pairs.

### 5.5 Determinism — reproducibility and a tighter result
Threaded runs spread wildly (542 endpoint gap 3.4 → 53 m; 538 62 → 133 m plus a 10–35 % catastrophic lottery). Four nondeterminism sources were identified: (1) thread interleaving, (2) pointer-ordered containers, (3) Eigen alignment-peeled vectorization, (4) an **uninitialized heap read** in the first post-IMU-init tracking frames (upstream bug, proven by `MALLOC_PERTURB_=42` flipping the md5 — not yet source-fixed). `ORB_DETERMINISTIC` + `IdLess` + `EIGEN_DONT_VECTORIZE` collapsed the 542 gap band to **2.54–3.18 m** (0.30 m spread, full coverage every draw — and *better* than the best threaded draw) and made 538 bit-identical 9/10. The fresh circle flight closed to **0.27 m, 3/3 bit-identical** — the first dataset to smash the < 5 m target.

### 5.6 Divergence guard + historical campaign
The guard aborted **36 / 52** historical runs, revealing that the dominant failure mode is a **velocity blow-up in the first 1–2 minutes** (catastrophic init in the climb, guard firing at frames 65–500), not gradual drift — and cutting wasted ~40-min runs to ~2–5 min. On the runnable subset (deterministic, CLAHE3 + σ0.15 + gate6 + frame σ0.5), the best endpoint gaps met the < 5 m target: **292_golem17 = 2.27 m** (99.0 % cov), **260_golem17 = 3.64 m**, **173_golem27 = 4.17 m**, 389_alma231 = 6.00 m. 15 / 52 runs reached ≥ 80 % coverage (vertical-ATE median 7.5 m); 3 never initialized (dusk / low-texture). Front-end match verification over 43 low-res flights: median **99.5 % of frames** had ≥ 500 matches to a neighbor — failures are content-limited (dark / winter), not threshold-limited.

**Endpoint-gap summary.** The vehicle EKF's own closure (spoofed-GPS, no absolute anchor) is 46.3 m (542) / 84.7 m (538), so every healthy SLAM run already beats it. Best achieved: **542 = 3.42 m**, **538 = 61.8 m**. The < 2 m (542) / < 5 m (538) targets were **not** reached on the primary flights: 542's vertical is solved in principle (fused reference + v-scale≈1 → dz −0.15 m) but a ~2.7 m horizontal floor persists; 538 needs an accepted landing loop.

---

## 6. Negative Results & Divergences

Honest record of what was tried and did **not** work — several of these shaped the final design by elimination.

1. **Full-resolution native-1640 VIO regressed every estimator** vs the 800-px baseline (538 monocular-inertial saved a 15.4 % fragment or nothing; mono-only 18 %). Root cause: weak high-altitude inertial observability — resolution and calibration do not fix a parallax/excitation problem. The 800-res baseline (538 tracked 92 %) was kept.
2. **Allan-variance "measured" IMU noise was strictly worse** than the legacy noise model: 538 went from 5.1 % to **89.8 % of frames with < 30 matches**. Tighter noise over-trusts an imperfect IMU/extrinsic; the legacy model wins. (Untested idea: inflate the measured noise ×2–5 for in-flight vibration/thermal.)
3. **VINS-Fusion diverges on everything** at full-res moving-start (538 ~889 km, 542 ~42 km); compass/mag fusion was never even applicable there because no non-diverged run existed.
4. **A sky/brightness detection mask hurt SLAM**: it halved 538's correct matches, killed 542 tracking (19 poses), and collapsed 538 vertical scale 0.305 → 0.072 — the peripheral features carry the altitude parallax at 130 m AGL. Static masks are viewer-only.
5. **nFeatures=1500 is an init-scale lottery** — the apparent "1.49 vertical-ATE champion" was a lucky no-baro draw (re-draws 1.79–9.93); tight baro breaks nf1500 maps (3/5 died).
6. **Several barometer-fusion designs were rejected with evidence:** a stored **absolute** per-map datum (538 exploded to km-scale), a window-median-only datum (drift-preserving, −5 m/100 s anchor drift), a **Huber kernel** on baro edges (saturates → vertical scale stuck at 0.4), periodic scale-refinement layered on top of the z-edges (scale thrash), σ=0.05 (too stiff), and frame-edge σ tighter or looser than 0.5.
7. **Rangefinder as a fusion source (and as GT) regresses on relief terrain** (538 vertical scale 0.71 → 0.54).
8. **The fused-altitude reference is harmful on 538** (climb-phase tilt correction reshapes the early map, endpoint gap 6× worse); landing-only fused references on 542 were a dead end.
9. **The magnetometer does not shrink the endpoint gap** (542: 9.3 / 32.3 / 39.7 m vs a 3.4–44 m baseline) — the horizontal error is scale-drift, not yaw.
10. **Loop closure was never achieved live.** `ORB_PR_*` knobs and free-scale Sim3 produced zero/insufficient detections; mid-flight 542 BoW candidates are **descriptor aliasing on self-similar farmland** (correctly rejected by RANSAC), and the one true landing revisit has too few both-sides-mapped correspondences to verify. The unconditional 4-DoF lock is correct by construction but **never exercised on a live detection**. An earlier "loop-gate composition" win was **retracted** — those draws had zero detections; the effect was recipe noise.
11. **Shortened initialization gates are counterproductive on long flights** (circle prelight closure 1.07 m vs 0.27 m with stock gates) — keep stock gates whenever the flight is long enough.
12. **The circle "final-11 s" footage has no usable flight** (2 s of violent blur then static grass) — endpoint gap unverifiable from it.
13. **A ~30 % catastrophic-init rate persists at any tight σ on 538** (vertical scale ≤ 0.005 explosion or FIBA segfault); the frame-edge suppresses but does not eliminate it (~10 %).
14. **Historical fleet realities:** 36/52 runs diverge in the climb; GPS-instance-0 is spoofed from takeoff on most (only ~6 scoreable); papa3's compass is unreliable; mag σ10 **regressed** a few flights (282_golem17 coverage 22 → 6 %, 52_golem28 35 → 23 %, 173 a 4.17 → 31 m lottery flip).
15. **One determinism source remains unfixed** (the uninitialized heap read) — 538 still flips between two trajectories 1/10.

---

## 7. Open Problems & Future Work

- **Monocular horizontal scale drift** is the dominant residual on the long flight (538) and the ~2.7 m floor on 542. Neither barometer nor magnetometer constrains it; only **loop closure** or an **absolute horizontal anchor** will. Reviving loop closure requires beating the Sim3-verification starvation on ordinary (non-good) maps — the core unsolved algorithmic problem.
- **Absolute map tilt (roll/pitch) of 4–15°** is unconstrained: the relative-datum barometer edges are tilt-blind by design. A small vertical ATE therefore does **not** imply globally-correct z. Candidate fix: an absolute-attitude prior from the IMU/attitude stream.
- **Metric scale of 0.75–0.98** is often just outside the required `(0.95, 1.1)` band — tie the init-time `EdgeBaroScaleGDir` more tightly, or add a horizontal scale anchor.
- **Catastrophic-init lottery at altitude** — the frame-baro edge halves it; a more robust weak-excitation initializer is needed to eliminate it.
- **Fix determinism source #4** (the post-init uninitialized heap read) for full bit-reproducibility.
- **Historical fleet coverage** — 36/52 still diverge in the climb; a gentler init schedule tuned per altitude band may recover several.

---

## Appendix A — Environment-variable reference

All default to stock behavior. `presence` = any value enables.

| Env var | Unit | Default | Effect |
|---|---|---|---|
| `ORB_BARO_CSV` | path | OFF | Load baro altitude CSV; enables baro fusion |
| `ORB_BARO_SIGMA` | m | 1.0 | 1σ of one `EdgeBaroZ` |
| `ORB_BARO_GATE` | m | 15.0 | Per-edge insertion gate |
| `ORB_BARO_SCALEREF_S` | s | 0 (off) | Period of whole-flight baro scale refinement |
| `ORB_BARO_FRAME_SIGMA` | m | 0 (off) | 1σ of the frame-rate baro z edge |
| `ORB_BARO_DEBUG` | presence | off | Baro datum/residual prints |
| `ORB_MAG_CSV` | path | OFF | Load body-frame Earth-field CSV; enables mag yaw + compass init |
| `ORB_MAG_SIGMA_DEG` | deg | 5.0 | 1σ of one `EdgeMagYaw` |
| `ORB_LOOP_RP_TOL` | rad | 0.008 | Roll/pitch loop-acceptance tolerance |
| `ORB_LOOP_YAW_TOL` | rad | 0.349 | Yaw loop-acceptance tolerance |
| `ORB_PR_NCAND` | int | 3 | BoW candidates per KF |
| `ORB_PR_COINC` | int | 3 | Consecutive geometric verifications for a loop |
| `ORB_PR_MATCH_SCALE` | float | 1.0 | Multiplier on intra-map match/inlier thresholds (floor 1) |
| `ORB_PR_SIM3_MININL` | int | 0 (scaled) | Sim3-RANSAC minimum-inlier floor |
| `ORB_PR_FREE_SCALE` | presence | off | Free-scale Sim3 for loop verification |
| `ORB_PR_DEBUG` | presence | off | Cascade instrumentation prints |
| `ORB_IMU_INIT_MINTIME` | s | 2.0/1.0 | First inertial-init KF-span |
| `ORB_IMU_INIT_MINKF` | int | 10 | First inertial-init min keyframes |
| `ORB_IMU_VIBA1_S` | s | 5.0 | VIBA1 trigger mark |
| `ORB_IMU_VIBA2_S` | s | 15.0 | VIBA2 mark (also scales low-motion reset) |
| `ORB_TRACK_MININL_PREIMU` | int | 50 | Pre-IMU-init tracking inlier floor |
| `ORB_TVR_SIGMA` | px | 1.0 | Two-view reconstruction sigma |
| `ORB_TVR_MINPARALLAX` | deg | 1.0 | Minimum accepted parallax |
| `ORB_TVR_GOODFRAC` | frac | 0.9 | Fraction of matches that must triangulate |
| `ORB_MINIT_MINMATCHES` | int | 100 | Monocular-init match gate |
| `ORB_MINIT_WINDOW_PX` | int | 100 | Monocular-init search window |
| `ORB_DETERMINISTIC` | presence | off | Sequential mode (no worker threads, inline GBA) |
| `ORB_DET_DEBUG` | presence | off | Per-frame/stage fingerprint prints |
| `ORB_DIVERGE_VMAX` | m/s | 0 (off) | Post-init per-KF velocity threshold |
| `ORB_DIVERGE_COUNT` | int | 4 | Consecutive over-VMAX KFs to flag divergence |
| `ORB_CLAHE` | float | 0 (off) | CLAHE clip limit before extraction |
| `ORB_CLAHE_TILE` | int | 8 | CLAHE tile grid |
| `ORB_MASK_FRAC` | float | 0 (off) | Circular keep-mask (sky filter) |
| `ORB_MASK_CX`/`CY` | px | image center | Circular mask center |
| `ORB_MASK_FILE` | path | OFF | Distorted-space keep-mask PNG |
| `ORB_STATS_CSV` | path | OFF | Per-frame tracking-stats CSV |
| `ORB_TSJUMP_S` | s | 1.0 | Frame-gap reset/split threshold |
| `ORB_PREFETCH` | int/"all" | OFF | Threaded pre-extraction workers |
| `ORB_NO_PACE` | presence | off | Skip real-time pacing |
| `ORB_VIEWER` | presence | off | Enable Pangolin viewer in examples |

Always-on (no knob): `IdLess` deterministic container ordering, `EIGEN_DONT_VECTORIZE`, unconditional 4-DoF loop lock, `pBiggerMap` fixes, ORBextractor feature-count diagnostics.

---

## Appendix B — Research diary index

The full day-by-day research log lives under [`docs/diary/`](diary/). Key entries:

- **Barometer σ campaign:** `20260716-0400…`, `-0445` (frame edge), `-0515` (sigma ladder), `-0530` (final).
- **Altitude drift & fusion design:** `20260715-1200-altitude-drift-vs-nora/RESULTS.md` (§4–13), `20260715-2231-rfnd-fusion.md`, `20260716-1700-rfnd-tilt-correction.md`.
- **Full-resolution / dataset pipeline:** `20260715-1140-nora-fullres-vio/` (overview, configs, compass fusion, statistics, plots).
- **Loop detection & place recognition:** `20260715-2108-scaledrift-exp1-loopaudit.md`, `20260716-1130-4dof-loop-lock.md`, `-1430-loop-detection-unblocked.md`, `-1615-gap-campaign-negatives-and-hope.md`.
- **Endpoint-gap campaign:** `20260716-1300-endpoint-gap-campaign.md`.
- **Determinism:** `20260717-0130-deterministic-mode.md`, `-0400-determinism-campaign-final.md`.
- **Initialization gates:** `20260722-1230-circle-init-gates.md`, `-1347-prelight-circle-gap-027.md`.
- **Historical fleet:** `20260722-1425-historical-datasets-pipeline.md`, `20260723-0030-hist-baro2-campaign.md`, `20260723-1447-historical-flights-handoff/README.md`.

*Report compiled 2026-07-29 from the research diary and source tree.*
