# 2026-07-15 session: altitude drift vs NORA — working state (updated live)

GOAL: ≥1 SLAM algo on 538+542 with altitude drift not worse than NORA; ≥500 correct matches
per used frame pair; visual debug artifacts. Mid-session the user added datasets
165608/163613/162710_move (vertical ATE eval) + asked for a threaded prefetch option.

## The bar (eval FIXED this session — baro_ate.py orientation flip was endpoint-noise-driven;
## the old "NORA missed 542 takeoff by 45 m" was THAT artifact, not real)
- 538 NORA: vATE 1.03 m, 10s-drift RMS 1.37 m (fair span). NORA = ArduPilot-EKF alt ≈ baro-fused.
- 542 NORA: vATE 1.46 m, RMS 0.96 m (fair span; full-span 2.23/1.28).

## Winning front-end (matches goal): 800-res + CLAHE3 + nf5000 (in-SLAM: ORB_CLAHE=3.0)
- stride-1 verified correct (RANSAC-homography) matches/pair: 538 median 608 (53.6% ≥500;
  3 featureless-terrain bands make the rest physically unreachable — evidence
  eval_out/match_final_20260715/*/); 542 median 1364 (87.9% ≥500). Old front-end: 70/21.
- Sky mask implemented (ORB_MASK_FRAC/CX/CY) but HURTS 538 (periphery=ground at 130 m) — off.

## Baro fusion in ORB-SLAM3 (BaroFusion.h + Optimizer.cc + LocalMapping.cc, all env-gated)
- EdgeBaroZ (unary z on VertexPose, quadratic + ORB_BARO_GATE 15 m) in LocalInertialBA
  (datum: newest fixed anchor > window median) + FullInertialBA (median).
- EdgeBaroScaleGDir (baro Δalt on scale+gravity, numeric Jacobian) in both InertialOptimization
  overloads → fixes mono vertical scale at INIT + in ScaleRefinement.
- NEW baro6: periodic baro ScaleRefinement via env ORB_BARO_SCALEREF_S=25 (LocalMapping::Run).
- REJECTED designs (evidence): window-median-only datum (preserves drift, v1/v2); stored absolute
  per-map datum (538 exploded km-scale, baro5); Huber on baro edges (saturates, can't fix scale).

## Results so far (vATE m / drift RMS m / coverage)
| run                | 538                  | 542                 |
|--------------------|----------------------|---------------------|
| no-baro clahe_nf5000 | 54.2 / 12.1 / 93%   | 3.70 / 2.37 / 77%*  |
| baro2 (init+median)  | 7.64 / 2.48 / 61%   | 3.28 / 2.45         |
| baro3 (anchor+Huber) | 38.3 / 8.3 / 98%    | 3.40 / 2.03         |
| baro4 (anchor+quad)  | 40.1 / 7.6 / 95%    | 4.70 / 2.54         |
| baro5 (stored abs)   | EXPLODED 2311       | 3.33 / 2.11         |
| baro6 (anchor+quad+periodic scaleref) | RUNNING | RUNNING        |
(*542 coverage 77% = full post-takeoff; the missing 23% is the pre-motion ground segment)

## New datasets (user): dataset/{165608,163613,162710}_move (1640) → my _half 820x616 derivatives
- configs/dr20260402_mi_820_v2[_imu400].yaml (calib4 fx276.5, lever 11 cm, IMU 200/400 Hz, nf5000)
- CRASH root cause found: multi-second IMU gaps (up to 3.8 s) in the data-recorder logs → SO3 NaN.
  FIXED by eval/fill_imu_gaps.py (insert-only interpolation) on the _half copies. data_raw.csv kept.
- circle_move skipped: no baro GT + 11.9 s too short for MI init.

## Visual debug delivered
- eval/imu_align_viz.py: runs/imu_align/{538,542,538_hi}/ — IMU-warp overlay (MAD 2.27 vs 5.03
  at 13.4°), noise search circles (BOTH noise sets sub-pixel at frame rate → 2 px parallax floor
  dominates), in-circle match classification.
- eval/match_viz.py extended (CLAHE/mask/scale/RANSAC-correct + green/red drawMatches).
- eval/altitude_compare.py: 3-panel NORA-vs-SLAM altitude figure (baro truth, cum err, drift bars).
- eval_out/match_sweep_20260715 (16-config sweep), eval_out/match_final_20260715 (stride-1 final).

## In flight
- baro6 suite: 538/542 + 3 new datasets ×(baro6, no-baro) — eval/run_baro6_20260715.sh (after
  prefetcher subagent rebuild completes; shared build dir).
- Prefetcher subagent: ORB_PREFETCH=N|all + ORB_NO_PACE=1 (faster tests, user ask).

## Eval commands
- Bar+runs: python3 eval/height_drift.py --traj <f.txt|gps_ref> --kind tum|nora --baro dataset/<ds>/ref_baro.csv
- Full table: bash eval/eval_matrix_20260715.sh (extend tags)
