# 06 — Statistics: features/frame, matches, per-10 s height drift, timeshift, and how to regenerate

All scripts live in `eval/`. Timestamps are rebased seconds (shared clock). Regenerate everything with the
commands here; the numbers below are from this session's runs (native-1640 moving-start unless noted).

## A. Features detected per frame + matches per frame (from ORB-SLAM3 `frame_stats.csv`)
`frame_stats.csv` columns: `timestamp, state, detections, matches_inliers, grab_ms`.
- **detections** = raw ORB/FAST keypoints found that frame (before the nFeatures cap).
- **matches_inliers** = features matched to the local map that frame (0 when NOT tracking/initialised).
Snapshot (also in `data/frame_stats_summary.txt`):

| run (native-1640 moving-start)          | frames | feat mean | feat med | match mean | match med | %frames<30 |
|-----------------------------------------|-------:|----------:|---------:|-----------:|----------:|-----------:|
| 538 mono                                |  4197  |     2698  |   2502   |    415     |    521    |   10.1     |
| 538 mono_inertial (legacy noise)        |  4730* |     3643  |   2694   |     72     |     46    |    5.1     |
| 538 mono_inertial (MEASURED noise)      |  4168  |     3472  |   2773   |     41     |     13    |   89.8     |
| 542 mono (hit 0-byte frame @1212)       |   874  |     3999  |   2507   |    263     |    278    |   21.3     |
| 542 mono_inertial (MEASURED, partial)   |   302  |     8411  |  11012   |     63     |      2    |   65.8     |
\* row count exceeds frame count because of map resets re-logging. Match means for MI are over the brief
OK-tracked windows; medians (46/13/2) show most frames were NOT tracking → the real story is the
**init-instability**, and that MEASURED noise makes it far worse (89.8 % vs 5.1 % low-match on 538).
Regenerate: `python3 <the awk/py block in run_all.sh step 4>` or just read each `frame_stats.csv`.

**Final trajectory tally (what actually saved, checked on disk at 12:25):**
| run                              | trajectory                                            |
|----------------------------------|-------------------------------------------------------|
| 538 mono native1640              | 771 poses (18 %)                                      |
| 538 MI native1640 (legacy)       | **646-pose fragment, t 21.9–101.7 s (15.4 %)** + 271 kf |
| 538 MI native1640 (measured)     | NONE (empty-map save crash, rc=134)                   |
| 542 mono / MI legacy / MI meas   | NONE (all died at frame ~1212 on the 0-byte PNG)      |
| VINS legacy 538 / 542            | 3978 / 1081 poses — DIVERGED (889 km / 42 km)         |
| VINS measured 538                | 3419 poses, t 32.8–479.3 s — DIVERGED (367 km)        |
| VINS measured 542                | never run (moot — measured noise already refuted)     |

## B. Matches per consecutive PAIR (SLAM-agnostic) + drawMatches inspection — `eval/match_viz.py`
**Upgraded 2026-07-15 (post-handoff)** — the current version does substantially more than the original:
- Lowe-ratio "good" matches **plus RANSAC-verified "correct" matches**: points are undistorted with the
  native KB8 fisheye calib and filtered by a RANSAC homography (quasi-planar ground from altitude), so
  homography inliers ≈ honest correspondences. Inliers drawn green, outliers red.
- Front-end options mirroring the SLAM experiments — **A/B them without touching/rebuilding any SLAM**:
  `--clahe 3.0` (preprocessing), `--mask-frac 0.70` (the circular sky mask of pending TASK 1!),
  `--scale 0.4878` (=800/1640 resolution study on the same frames), `--nfeatures`, `--csv counts.csv`.
- `--stride N` now samples every Nth CONSECUTIVE pair (i, i+1) — the pair baseline is always one frame,
  so sparse sampling no longer inflates the low-match rate.
```bash
python3 eval/match_viz.py --dataset dataset/nora538_move --out plots/matches_538 \
   --low 500 --nfeatures 5000 --clahe 3.0 --mask-frac 0.70 --stride 5 \
   --title "538 full-res" --csv counts.csv
```
Use it FIRST to preview the sky-mask benefit (with/without `--mask-frac`) before the SLAM-side rebuild.

## C. Accumulated height drift per 10 s vs barometer — `eval/height_drift.py`
Fits the SLAM up-direction (3-vector, like `baro_ate.py`), initial-height aligned, then per 10 s window:
`window drift = ΔVIO − ΔBaro` (height error added that window) and `cumulative = (h−baro)` running total.
```bash
python3 eval/height_drift.py --traj <f_*.txt> --baro dataset/<ds>/ref_baro.csv --window 10 --csv out.csv
```
Worked example (prior **800-res** MI on 538, which DID produce a trajectory) — cumulative height error grows
monotonically to **+74 m by t≈130 s** then oscillates back; per-window drift RMS ≈ 11 m; overall vertical
ATE 58.6 m, v-scale 0.31 (VIO vertical is ~3× under-scaled → unusable, use baro). Full table printed by the
script. Once a native-1640 run yields a trajectory, run the same command against it.

## D. Timeshift analysis (NORA↔VINS "~30 s") — `eval/time_align_check.py`
Cross-correlates the SPEED profiles (scale/rotation-invariant) to measure the true lag.
```bash
python3 eval/time_align_check.py --a "NORA=nora_20260709/gps_ref_538.csv=nora" \
   --b "ORB=runs/.../f_native1640.txt=tum" --maxlag 45 --dt 0.2
```
Findings (all peak at ~0 lag → **NO timebase offset**): ORB-538 vs NORA lag **+0.00 s** (corr 0.572,
cleanest); ORB-542 +0.40 s; VINS-542 +0.20 s. End-times also align (<2 s), and both `gps_ref` and images use
identical `(companion_ns − epoch_ns)/1e9`. **The "~30 s shift" is inertial-init latency** (VINS first pose
22–41 s, ORB 40–45 s), NOT a bug. Presentational fix: always plot on the shared absolute clock (combined_plot
does this; dotted lines mark each estimator's init latency).

## E. Other statistics (all via `eval/`)
- **Coverage** = n_poses / n_frames (from `f_*.txt` line count ÷ `cam0_times.txt`).
- **Horizontal scale + RMSE** vs NORA: `combined_plot.py` panel 1 (Umeyama-2D) or `validate_nora.py`
  (3 independent scale estimates). Prior 538 horizontal scale ≈ 1.02.
- **Vertical ATE** vs baro: `baro_ate.py` (metric + shape). Prior 538 MI 58.6 m; 542 MI 1.5 m.
- **Divergence classification** (VINS): max|position| — <1000 m BOUNDED, >5000 m DIVERGED. This session
  VINS 538 ~889 km, 542 ~42 km (both DIVERGED, −z/gravity-dominated).
- **Excitation validation** (trim): accel-std pre/post the start ns (538 ×5.9, 542 ×8.1).
