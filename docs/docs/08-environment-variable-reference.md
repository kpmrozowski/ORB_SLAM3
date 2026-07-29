# 08 — Environment Variable Reference

Every behavioural change in the fork is behind an `ORB_*` variable. **All default to stock ORB-SLAM3.** `presence` = any value enables it. Grouped by area; see [doc 06](06-vanilla-algo-all-good-mods.md) for what each area does.

## Barometer / magnetometer fusion

| Variable | Unit | Default | Effect |
|---|---|---|---|
| `ORB_BARO_CSV` | path | off | Load baro altitude CSV; enables baro fusion |
| `ORB_BARO_SIGMA` | m | 1.0 | 1σ of one baro constraint (main tuning dial) |
| `ORB_BARO_GATE` | m | 15.0 | Per-edge insertion gate (skip inconsistent KFs) |
| `ORB_BARO_FRAME_SIGMA` | m | 0 (off) | 1σ of the per-frame z pin (init stability) |
| `ORB_BARO_SCALEREF_S` | s | 0 (off) | Period of the periodic scale refinement |
| `ORB_BARO_DEBUG` | presence | off | Datum / residual prints |
| `ORB_MAG_CSV` | path | off | Load body-frame field CSV; enables mag yaw + compass init |
| `ORB_MAG_SIGMA_DEG` | deg | 5.0 | 1σ of one yaw constraint (fleet: 10 works best) |

## Place recognition / loop closing

| Variable | Unit | Default | Effect |
|---|---|---|---|
| `ORB_LOOP_RP_TOL` | rad | 0.008 | Roll/pitch loop-acceptance tolerance |
| `ORB_LOOP_YAW_TOL` | rad | 0.349 | Yaw loop-acceptance tolerance |
| `ORB_PR_NCAND` | int | 3 | BoW candidates per keyframe |
| `ORB_PR_COINC` | int | 3 | Consecutive geometric verifications for a loop |
| `ORB_PR_MATCH_SCALE` | float | 1.0 | Multiplier on intra-map match/inlier thresholds |
| `ORB_PR_SIM3_MININL` | int | 0 (auto) | Sim(3)-RANSAC minimum-inlier floor |
| `ORB_PR_FREE_SCALE` | presence | off | Free-scale Sim(3) for loop verification |
| `ORB_PR_DEBUG` | presence | off | Loop-cascade instrumentation prints |

## Initialization gating

| Variable | Unit | Default | Effect |
|---|---|---|---|
| `ORB_IMU_INIT_MINTIME` | s | 2.0 / 1.0 | First inertial-init keyframe span |
| `ORB_IMU_INIT_MINKF` | int | 10 | First inertial-init min keyframes |
| `ORB_IMU_VIBA1_S` | s | 5.0 | VIBA1 trigger |
| `ORB_IMU_VIBA2_S` | s | 15.0 | VIBA2 trigger (also scales low-motion reset) |
| `ORB_TRACK_MININL_PREIMU` | int | 50 | Pre-IMU-init tracking inlier floor |
| `ORB_TVR_SIGMA` | px | 1.0 | Two-view reconstruction sigma |
| `ORB_TVR_MINPARALLAX` | deg | 1.0 | Minimum accepted parallax |
| `ORB_TVR_GOODFRAC` | frac | 0.9 | Fraction of matches that must triangulate |
| `ORB_MINIT_MINMATCHES` | int | 100 | Monocular-init match gate |
| `ORB_MINIT_WINDOW_PX` | int | 100 | Monocular-init search window |

## Determinism, divergence, preprocessing, I/O

| Variable | Unit | Default | Effect |
|---|---|---|---|
| `ORB_DETERMINISTIC` | presence | off | Sequential mode (no worker threads, inline GBA) |
| `ORB_DET_DEBUG` | presence | off | Per-frame / per-stage fingerprint prints |
| `ORB_DIVERGE_VMAX` | m/s | 0 (off) | Post-init per-KF velocity threshold |
| `ORB_DIVERGE_COUNT` | int | 4 | Consecutive over-VMAX KFs to flag divergence |
| `ORB_CLAHE` | float | 0 (off) | CLAHE clip limit before extraction |
| `ORB_CLAHE_TILE` | int | 8 | CLAHE tile grid size |
| `ORB_MASK_FRAC` | float | 0 (off) | Circular keep-mask (sky filter) |
| `ORB_MASK_CX` / `ORB_MASK_CY` | px | image center | Circular mask center |
| `ORB_MASK_FILE` | path | off | Distorted-space keep-mask PNG |
| `ORB_STATS_CSV` | path | off | Per-frame tracking-stats CSV |
| `ORB_TSJUMP_S` | s | 1.0 | Frame-gap reset/split threshold |
| `ORB_PREFETCH` | int / `all` | off | Threaded pre-extraction workers |
| `ORB_NO_PACE` | presence | off | Skip real-time pacing (run flat-out) |
| `ORB_VIEWER` | presence | off | Enable the Pangolin viewer in the examples |

## Always-on (no knob)

`IdLess` deterministic container ordering · `EIGEN_DONT_VECTORIZE` · unconditional 4-DoF loop lock · `pBiggerMap` / empty-map-save / double-BA fixes · ORB feature-count diagnostics.
