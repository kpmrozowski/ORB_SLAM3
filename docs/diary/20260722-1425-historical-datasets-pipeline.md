# Historical NORA datasets: runnability scan, conversion pipeline, 300x225 thresholds

**Date:** 2026-07-22. User ask: make the ORBSLAM3-MI pipeline work with the historical
datasets of `/media/kmro/datasets/dops/flights/keypoints_analysis/keypoints_summary.csv`
(300x225 frames downsampled from 800x600); select the actually-runnable ones (preintegrated
IMU in the corresponding BIN + ≥1 GPS[1] fix within 1 km of 50.41020638, 29.93702530),
convert to the common EuRoC eval format in `datasets-nora/`, write `selected_flights.csv`,
and tune thresholds for ≥10000 features/frame and ≥500 matches to ≥1 of 4 neighbours.

## Pipeline (eval/hist_inventory.py → hist_scan.py → hist_convert.py → hist_select.py)

- **Inventory**: 59 CSV sessions → session storage (plain dir / .temp.zip / replay), image
  and hdf-chunk counts, candidate ArduPilot BINs under `flights/<date>/<drone>/`.
- **Runnability scan**: session wall-clock window from hdf `keypoints.SensorTimestamp`;
  per-BIN stream scan (RISI count, GPS[1] fixes + min haversine distance to the target,
  GPS-unix span coverage). Verdict + reason per flight → `eval_out/hist_scan.csv`.
- **Conversion**: per-flight clock bridge companion_ns = slope·boot_ns + K fitted from hdf
  mavlink `timesync` (residuals 0.5–35 ms); RISI→gyro/accel (nora_bin reuse), shifted by
  −timeshift_cam_imu from the session kalibr calibration; frames symlinked/extracted as
  rebased-ns PNGs; sidecars ref_baro/ref_rfnd/ref_att/ref_gps1; per-flight ORB-SLAM3 KB8
  config; dataset_meta.json with full provenance.

## Selected flights (7 of 59) → `keypoints_analysis/selected_flights.csv`

| flight | source | frames | fps | GPS[1] min dist |
|---|---|---|---|---|
| 52_golem28_2025-12-19T11-54-39 | session dir | 3834 | 5.0 | 12.6 m |
| 175_golem27_2026-02-27T13-54-01 | session dir | 3101 | 5.0 | 11.2 m |
| 180_golem27_2026-02-27T14-35-32 | replay original | 2854 | 5.0 | 9.4 m |
| 182_golem27_2026-02-27T15-07-52 | replay original | 3074 | 5.0 | 3.1 m |
| 90_golem23_2026-02-27T16-27-18 | replay original | 3657 | 5.0 | 1.9 m |
| 576_golem27_2026-07-10T16-59-01 | vio high-res → 800x600 | 2863 | 15.2 | 0.0 m |
| 577_golem27_2026-07-10T17-06-43 | vio high-res → 800x600 | 1580 | 15.0 | 0.0 m |

Main exclusion reasons: no BIN directory at all (papa3/epos/golem14/29/34/49 fleets), BIN
without RISI (golem17 gt logs), flight at a different field (28–79 km: 39_golem26, 83_epos1,
2026-03-03 campaign), original session of a replay deleted (2025-11-18 golem17/alma231).

## Non-obvious structure learned (the traps)

1. `replay_logs/<orig-flight>_<BINnum>/<replay-session>/` — the parent dir names BOTH the
   original flight and its exact BIN; replays themselves carry no frames.
2. `ardu/gt/` dirs contain **cross-date BIN copies** (e.g. 2026-03-03/golem27/gt/00000016 =
   the 2026-02-27 golem23 flight): GPS-time BIN matching alone is not sufficient — the
   converter re-validates by RISI-TimeUS overlap against the per-flight clock bridge.
3. hdf `timesync` tc1/ts1 roles swap per row (request vs response) — classify by magnitude.
4. pymavlink 2.4.49's C dfindexer hard-exits the process on corrupt FMT tables →
   subprocess-isolated scans.
5. 300x225 = 800x600 → symmetric crop [100,75] → ×0.5 (from the hydra camera config), so
   intrinsics transform as fx/2, (cx−100)/2 — NOT a plain 0.375 rescale.
6. The 2026-07-10 nora zips contain keypoints only; the frames live in
   `vio/20260710/<session>.temp/high_res_images_png` (1640x1232, `.gray` raws next to them).

## 300x225 threshold tuning (measured, batch hist_var1 on 90_golem23)

| extractor | detections median | ≥10k frames |
|---|---|---|
| stock 1.2 / 8 levels, native 300px | 7410 | 0% |
| 1.1 / 12 levels | 8678 | 23% |
| stock pyramid + 1.5× upscale (450x338) | 9565 | 25% |
| **1.1 / 12 + 1.5× upscale** | **10010 (cap-limited)** | **~99%** |

At FAST threshold 1 the native-resolution candidate pool (~7.4 k) is the binding
constraint, not the octree cull — the pyramid must grow. Final configs: nFeatures 11000
(headroom over the exact-10000 cap), scaleFactor 1.1, nLevels 12, Camera.newWidth/Height
450x338, iniThFAST 5 / minThFAST 1. Mono-inertial init WORKS at 5 fps/300x225 (VIBA1+2
completed on 90_golem23 in the smoke run). Matches (OpenCV mirror, first 60 frames,
pre-tuning): offset-1 median 3421; 98.3% of frames ≥500 to ≥1 of 4 neighbours — full
5-dataset verification (tuned settings, all frames) running as of this writing.

## Also today

circle target CLOSED: `circle_move_prelight_junk` endpoint gap **0.27 m** deterministic
(diary 20260722-1347).
