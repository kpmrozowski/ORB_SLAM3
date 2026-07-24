# IC (inverse-compositional) ORB false-positive match filter

Env-gated, fail-open dense photometric refinement that prunes ORB false-positive matches at
monocular initialization and at frame-rate tracking. Ported from the radiolus alignment lab
(`corr::Correlation` / `refine_homography`) and upgraded with the lab-proven best-of-seed family,
ensemble ECC voting, and the relative acceptance gate. **Defaults preserve stock ORB-SLAM3 behavior
byte-for-byte** — nothing runs unless a flag is set, and any missing ingredient leaves matches
untouched.

## What it does

* **Seed (`ICSeed`)** — an IMU/barometer-only homography `H21 = K (R21 + t21 nᵀ/d) K⁻¹` in the
  undistorted `P == K` pixel domain. `R21` is always gyro preintegration; the translation/plane use
  the barometer climb rate, a forward-speed prior, and a gravity + AGL ground plane. **Never seeded
  from ORB.** A candidate family (full seed, rotation-only `K R21 K⁻¹`, and — TWMM only — a
  previous-pair propagation) is scored by consistent ZNCC on the undistorted pair and the best wins.
* **Refine (`ICEngine`)** — inverse-compositional dense alignment at full resolution. The reductions
  use a fixed-64-chunk `std::thread` summation, so results are **bit-identical for any thread count**.
* **Init filter (`ORB_IC_INIT`)** — refines the seed against the retained init reference image, then
  prunes `mvIniMatches` whose symmetric transfer error vs the reference H exceeds `ORB_IC_FILTER_PX`.
  A survivor guard (`≥ ORB_MINIT_MINMATCHES`) means the filter can never starve initialization.
  `TwoViewReconstruction` still consumes only the survivors — no TVR surgery.
* **Track gate (`ORB_IC_TRACK`)** — in `TrackWithMotionModel`, before the `nmatches < 20` check,
  prunes current-frame map points whose last-frame pixel transfers more than `ORB_IC_TRACK_PX`.
  Active pre-IMU-init / post-reloc (TWMM early-returns once IMU is initialized). Survivor floors
  (20 matched / 10 map-observed) mean the filter can never fail TWMM on its own.
* **Ensemble ECC voting (`ORB_IC_ENSEMBLE`)** — at the init filter only, also fit
  `H_orb = findHomography(RANSAC, 3.0)` on the matched undistorted points and use whichever of
  `{H_ic, H_orb}` scores the higher consistent ZNCC **as the filter reference**. Voting only picks the
  gate reference; ORB never seeds IC and TVR is untouched.

## Environment variables

| Variable | Default | Meaning |
|----------|---------|---------|
| `ORB_IC_INIT` | unset (off) | Enable the init-pair match filter |
| `ORB_IC_TRACK` | unset (off) | Enable the TWMM frame-rate gate |
| `ORB_IC_ENSEMBLE` | unset (off) | Ensemble ECC voting (init filter reference = argmax ZNCC of H_ic vs H_orb) |
| `ORB_IC_SEED_PREV` | unset (off) | Add the previous-pair propagation seed candidate (TWMM only) |
| `ORB_IC_FILTER_PX` | `3.0` | Init: symmetric transfer gate (px) for match invalidation |
| `ORB_IC_TRACK_PX` | `6.0` | Track: one-way transfer gate (px) for map-point rejection |
| `ORB_IC_MIN_CORR` | `0.8` | Init: absolute acceptance ZNCC (Absolute gate) |
| `ORB_IC_TRACK_MIN_CORR` | `0.75` | Track: absolute acceptance ZNCC (Absolute gate) |
| `ORB_IC_GATE_MODE` | `abs` | `abs` = min-corr; `rel` = floor + tolerance below the used seed |
| `ORB_IC_GATE_FLOOR` | `0.5` | Relative gate: absolute floor the refined ZNCC must clear |
| `ORB_IC_GATE_MARGIN` | `0.05` | Relative gate: tolerance the refined ZNCC may fall below the used seed |
| `ORB_IC_TRACK_MIN` | `20` | Track: minimum matched map points before the gate applies (survivor floor) |
| `ORB_IC_MAX_DT` | `1.0` | Refuse the IMU seed when the pair Δt (s) exceeds this (dropped-frame guard) |
| `ORB_IC_SPEED_MPS` | `0` | Forward-speed prior for the full seed's horizontal translation (m/s) |
| `ORB_IC_RFND_CSV` | unset | Optional rangefinder AGL source (`t_ns,dist_m`); overrides baro AGL when present |
| `ORB_IC_GAUSS_ITERS` / `ORB_IC_GN_ITERS` | `30` / `60` | Init refinement Gauss / Gauss-Newton iteration budgets |
| `ORB_IC_TRACK_GAUSS_ITERS` / `ORB_IC_TRACK_GN_ITERS` | `8` / `12` | Track refinement iteration budgets |
| `ORB_IC_THREADS` | `1` | Worker threads for the chunked reductions (bit-identical for any value) |
| `ORB_IC_DEBUG` | unset | Stderr `IC_INIT` / `IC_TRACK` lines + per-pair CSV |
| `ORB_IC_DEBUG_DIR` | unset | Directory for `ic_stats.csv` (must already exist) |

Related existing envs the seed reads: `ORB_BARO_CSV` (barometer series → climb rate + AGL),
`ORB_DETERMINISTIC` (context; the reductions are deterministic regardless).

## `ic_stats.csv` columns

`kind` (INIT/TRACK), `frame_ref`, `frame_cur`, `t_ref`, `t_cur`, `n_matches`, `seed_src`
(rot/full/prev/failed), `used_seed_zncc`, `ecc_seed_rot`, `ecc_seed_full`, `ecc_ic`, `ecc_orb`
(-1 on TRACK), `ensemble_winner` (ic/orb/none), `gate` (pass/lowcorr/guard), `invalidated`,
`survivors`, `ms`.

## GPS[0] overlay (`ORB_GPS_CSV`)

Separate viewer-only feature (`GpsOverlay` / `MapDrawer::DrawGPS`). Reads `t_rebased_ns,e,n,u,valid`,
aligns valid GPS to keyframes with a free `Eigen::umeyama` similarity (refit ~2 s), and draws a yellow
trajectory line-strip, dark-yellow sample points, and bright-orange keyframe↔GPS correspondence lines.
Two Pangolin checkboxes ("GPS[0] traj" / "GPS↔SLAM lines"). Inactive without `ORB_GPS_CSV`.

## Scope notes

* No in-SLAM flipbook PNG dumping — the radiolus lab owns visual debug; `ic_stats.csv` + the stderr
  lines are the tuning surface here.
* The map-point RANSAC plane rung is intentionally not built: it is only reachable after a metric map,
  i.e. after IMU init, where the TWMM gate early-returns and the seed is never requested. The gravity +
  baro/rfnd AGL plane (metric for both the init and pre-init-track windows) is the only plane path that
  ever executes.
