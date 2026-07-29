# Historical NORA flights in ORB-SLAM3-MI — full pipeline handoff

**Date:** 2026-07-23 14:47. Workspace: `/home/kmro/praca/dev/orbslam3-eval`.
This directory is a self-contained handoff: it documents the whole historical-flights
pipeline (selection → conversion → runs → evaluation), copies every script involved
into `scripts/`, and lists the open problems with concrete next steps.

Related diaries: `20260722-1425-historical-datasets-pipeline.md` (selection+conversion
day), `20260723-0030-hist-baro2-campaign.md` (first fused campaign, divergence guard).

---

## 1. Selected flights — where listed, and the selection criteria

**Master list: `/media/kmro/datasets/dops/flights/keypoints_analysis/selected_flights.csv`**
(regenerate any time with `python3 eval/hist_select.py`). One row per flight:
`flight_key, keypoints_summary_sessions, date, drone, image_source, session_path, bin,
gps1_min_dist_m, n_frames, fps_median, imu_hz_median, image_width/height, eval_dataset,
orbslam3_config, clock_bridge_residual_ms, selection_source`.

It merges TWO selections:

1. **`selection_source=keypoints_summary`** — my scan of the 59 sessions in
   `keypoints_analysis/keypoints_summary.csv` (`eval/hist_inventory.py` +
   `eval/hist_scan.py`). A session is RUNNABLE iff:
   * its session storage still exists (plain dir or `.temp.zip`) with frames + `hdf_logs`;
   * a corresponding ArduPilot `.BIN` exists under `flights/<date>/<drone>/` containing
     **RISI** (preintegrated replay IMU — the only usable IMU source);
   * the BIN has ≥1 **GPS[1]** 3D fix within **1 km of 50.41020638, 29.93702530**
     (the test field; GPS[1] is the authentic instance, GPS[0] is routinely spoofed);
   * the BIN's GPS time span covers the session window (verdicts + reasons per flight:
     `eval_out/hist_scan.csv`).
   Result: **7 / 59** runnable (52_golem28, 175/180/182_golem27, 90_golem23 at 300x225,
   plus 576/577_golem27 whose frames live in `vio/20260710/*.temp` at 1640x1232).
   Replay sessions (`flights/replay_logs/<orig-flight>_<BINnum>/…`) resolve to their
   ORIGINAL flight — the parent dir name encodes both the flight and its exact BIN.

2. **`selection_source=bin_session_selection`** — the user-provided
   `/media/kmro/datasets/dops/flights/bin_session_selection/final_selected_sessions.csv`
   (50 sessions, gps0-radius-based, session↔BIN correspondence already resolved, even
   across dirs, e.g. 150_golem26's BIN under `2026-05-11/bone-1/ardu`). All 50 converted.

**Converted datasets: `datasets-nora/<flight_key>/`** (52 total), each symlinked as
`dataset/hist_<id>_<drone>` so the standard runners work. Per-flight ORB-SLAM3 configs:
`configs/hist/<flight_key>.yaml` (+ `_noup.yaml` no-upscale variants).

---

## 2. Converting a NORA flight to EuRoC — how it works, with examples

Converter: `eval/hist_convert.py` (copied here). Two invocation forms:

```bash
# from my scan (hist_scan.csv):
python3 eval/hist_convert.py --flight 90_golem23            # substring of flight_key
python3 eval/hist_convert.py --all-runnable
# from the user's pre-matched CSV:
python3 eval/hist_convert.py \
  --binsel-csv /media/kmro/datasets/dops/flights/bin_session_selection/final_selected_sessions.csv \
  --flight 158_golem27 [--skip-existing]
```

Output layout (exactly the eval-standard EuRoC used for 538/542):

```
datasets-nora/<flight_key>/
  mav0/cam0/data/<rebased_ns>.png   frames (symlinks for dir sessions, copies for zip/vio)
  mav0/imu0/data.csv                #timestamp [ns], gyro xyz [rad/s], accel xyz [m/s^2]
  cam0_times.txt                    one rebased-ns stamp per frame
  ref_baro.csv  ref_rfnd.csv  ref_att.csv  ref_gps1.csv  ref_mag.csv
  dataset_meta.json                 provenance: session, bin, epoch_ns, bridge fit, ...
```

All stamps are **companion Unix ns REBASED by `epoch_ns`** (= first frame) — absolute
Unix ns break float64 in the EuRoC loader.

### 2a. Camera

* **Historical 300x225 sessions** store index-named frames (`images/0.png`, `1.png`, …).
  Per-frame timestamps come from the session's `hdf_logs/*.hdf` chunks:
  `localization_data/keypoints` rows carry `(SensorTimestamp [Unix s], SourceFile)` —
  parse the frame index from SourceFile, stamp = SensorTimestamp·1e9.
  Frames are symlinked to `<stamp−epoch>.png`.
* **Geometry** (probed from an actual emitted image, `resolve_image_geometry()`):
  300x225 = 800x600 → symmetric crop `[100, 75]` (from `hydra/config.yaml`) → ×0.5
  (`image_min_size: 225`). Intrinsics transform: `fx' = fx·0.5`, `cx' = (cx−100)·0.5`
  (kalibr `calibration.yaml`, pinhole+equidistant = KannalaBrandt8, distortion k1..k4
  UNCHANGED — KB acts on the incidence angle). Some fleets store native 800x600
  (158_golem27, papa3, epos3 with `crop: null`) → intrinsics used as-is.
  Example (90_golem23): calib fx=288.997 cx=405.92 → config fx=144.499 cx=152.960.
* **vio hi-res flights** (576/577): companion-ns-NAMED pngs in
  `vio/20260710/<session>.temp/high_res_images_png` (1640x1232, `.gray` raws beside
  them) → downscaled to 800x600 INTER_AREA copies; calibration is already at 800x600.

### 2b. IMU (the clock bridge — the heart of the conversion)

* Source: BIN **RISI** messages (instance 0), stamped by the preceding **RFRH** TimeUS
  (FC boot µs). Conversion: `gyro = [DAX,DAY,DAZ]/DADT`, `accel = [DVX,DVY,DVZ]/DVDT`
  (reuses `nora_20260709/lib/nora_bin.py: risi_to_imu`).
* **Per-flight clock bridge** `companion_ns = slope · boot_ns + K`, fitted from the hdf
  `mavlink_data/timesync` stream (`tc1`/`ts1` — roles SWAP per row between request and
  response, so classify by magnitude: boot ~1e12 ns vs Unix ~1.7e18 ns). Fallbacks in
  order: `system_time` (companion s ↔ boot ms), then `gps2+binGPS` composition
  (session gps2 time_usec ↔ companion, BIN GPS GWk/GMS ↔ TimeUS). Guards:
  * pairs windowed to the session (same-day chunks can contain OTHER boots);
  * **offset-cluster fit** — an FC mid-session reboot yields several parallel line
    segments; fit only the dominant `companion−boot` offset cluster (239_golem17);
  * slope must be in (0.9, 1.1). Residuals achieved: 0.09–35 ms (in `dataset_meta.json`).
* IMU stamps are then shifted by **−timeshift_cam_imu** (kalibr convention,
  per-session `calibration.yaml`; null-safe) and trimmed to frames ±2 s.
* **BIN choice is re-validated by RISI-TimeUS overlap** with the bridge-predicted
  window — `ardu/gt/` dirs contain cross-date BIN copies, GPS-time matching alone is
  NOT sufficient. Partial coverage ≥120 s is accepted (139_golem26 outlives its log);
  frames outside IMU coverage are dropped.
* BIN parses are cached: `eval_out/bincache/<sanitized-bin-path>_<size>/hist_*.npz`
  (risi/gps/baro/rfnd/mag/att). pymavlink 2.4.49's C dfindexer hard-exits on corrupt
  FMT tables → `hist_scan.py` isolates scans in subprocesses.

### 2c. Barometer

`BARO` instance 0 → `ref_baro.csv` (`#timestamp [ns],alt_m`), bridge-stamped, rebased,
trimmed to the IMU window. Consumed by BaroFusion (`ORB_BARO_CSV`).

### 2d. Magnetometer

`python3 eval/hist_export_mag.py` → `ref_mag.csv` (`#timestamp [ns],mx,my,mz`):
BIN `MAG` instance 0, **unit-normalized, body-frame**. For these flights the FC body
frame IS the SLAM body frame (RISI is FC-body), so unlike the 538/542 exporter
(`eval/export_mag_csv.py`) no vehicle→IMU alignment rotation is needed.

---

## 3. Running ORB-SLAM3-MI on the datasets

Batch runner: `eval/det_queue.sh` (copied here). Queue line format:
`tag|dataset|config|ENV;ENV;…` (`-` = no extra env). Example:

```bash
printf 'demo_156|hist_156_golem27|hist/156_golem27_2026-02-20T10-17-23_noup.yaml|ORB_CLAHE=3.0;ORB_BARO_CSV=/home/kmro/praca/dev/orbslam3-eval/dataset/hist_156_golem27/ref_baro.csv;ORB_BARO_SIGMA=0.15;ORB_BARO_GATE=6;ORB_BARO_FRAME_SIGMA=0.5;ORB_MAG_CSV=/home/kmro/praca/dev/orbslam3-eval/dataset/hist_156_golem27/ref_mag.csv;ORB_MAG_SIGMA_DEG=10;ORB_DIVERGE_VMAX=30;ORB_DIVERGE_COUNT=4\n' > /tmp/q.txt
bash eval/det_queue.sh /tmp/q.txt mybatch 5   # 5 = parallel processes
```

Outputs: `runs/<dataset>/mono_inertial/<tag>/f_<tag>.txt` (trajectory, TUM-like:
**rebased-ns** stamp, xyz, qxyzw), `logs/det_<tag>.log`, one summary row appended to
`runs/det_campaign_summary.csv` (`batch,tag,exit,poses,cov_pct,vate_m,drift_rms_m,
gap_m,gap_dz_m,md5`). The runner itself sets `ORB_DETERMINISTIC=1 ORB_NO_PACE=1
OMP_NUM_THREADS=1 ORB_TSJUMP_S=6.0 ORB_STATS_CSV=…` and unsets all fusion env first.

Direct invocation (what the runner wraps):

```bash
ORB_SLAM3/Examples/Monocular-Inertial/mono_inertial_euroc \
  ORB_SLAM3/Vocabulary/ORBvoc.txt configs/hist/<key>.yaml \
  dataset/hist_<short> dataset/hist_<short>/cam0_times.txt <tag>
```

### Env vars (fork `ORB_SLAM3`, branch `nora-altitude-fusion`, HEAD `72b277b`)

| var | meaning |
|---|---|
| `ORB_DETERMINISTIC=1` | sequential LocalMapping/LoopClosing (no worker threads), inline GBA — bit-reproducible runs, kills the in-process g2o race → safe to run N processes in parallel |
| `ORB_NO_PACE=1` | no real-time pacing (process frames as fast as possible) |
| `ORB_TSJUMP_S` | frame-drop tolerance: timestamp jumps larger than this are bridged |
| `ORB_STATS_CSV` | per-frame stats CSV (timestamp, tracking state, `detections`, inliers, ms) |
| `ORB_CLAHE=3.0` | in-SLAM CLAHE preprocessing (clip limit; load-bearing on aerial imagery) |
| `ORB_BARO_CSV` | ref_baro.csv → enables BaroFusion |
| `ORB_BARO_SIGMA=0.15` | 1σ [m] of one keyframe baro z-edge (the vATE lever; smaller = tighter) |
| `ORB_BARO_GATE=6` | chi2 gate on baro edges |
| `ORB_BARO_FRAME_SIGMA=0.5` | frame-rate baro z-edge in pose-only optimization (suppresses pre-VIBA2 reset thrash) |
| `ORB_MAG_CSV` | ref_mag.csv → enables MagFusion |
| `ORB_MAG_SIGMA_DEG=10` | 1σ [deg] of one yaw edge (**tuned value: 10**; 2 collapses tracking on some flights) |
| `ORB_DIVERGE_VMAX=30`, `ORB_DIVERGE_COUNT=4` | early-divergence guard: 4 consecutive KFs with \|v\|>30 m/s → stop feeding frames, save partial trajectory (`[DIVERGE]` in log) |
| init-gate/PR knobs | `ORB_IMU_INIT_MINTIME/MINKF`, `ORB_IMU_VIBA1_S/VIBA2_S`, `ORB_TRACK_MININL_PREIMU`, `ORB_TVR_*`, `ORB_MINIT_*`, `ORB_PR_*`, `ORB_LOOP_RP_TOL/YAW_TOL` — defaults preserve stock; see diaries 20260722-1230 / 20260716-* |

Campaign queue files (copied here): `det_hist_baro2.txt` (baro-only, batch
`hist_baro2`), `det_hist_magtune.txt` (σ sweep), `det_hist_baromag.txt`
(baro+mag σ10, batch `hist_baromag` — **still running at the time of writing**, 21/52
rows in, 7 with >80% coverage so far).

---

## 4. Evaluating metrics — scripts, usage, interpretation

All scripts copied into `scripts/`. They live in `eval/`.

### 4a. Automatic per-run metrics (`det_queue.sh` calls these)

* `eval/endpoint_gap.py --traj f_<tag>.txt` — **endpoint gap** =
  ‖median(last 1 s) − median(first 1 s)‖ of positions. Frame-invariant closure metric.
  VALID ONLY for reset-free single-map runs (check the log for "Reseting active map"
  past init and coverage ≈100%). Targets: <5 m (538-class), <2 m (542-class).
* `eval/baro_ate.py --baro ref_baro.csv --traj T=f.txt=tum` — **vATE** [m] vs the baro
  altitude series; the evaluator fits its own up-axis and offset (so it is blind to the
  map tilt described below — do not treat small vATE as "z is globally right").
* `eval/height_drift.py` — per-window vertical drift RMS.

### 4b. Front-end thresholds — `eval/hist_match_check.py`

```bash
python3 eval/hist_match_check.py --dataset datasets-nora/<key> \
  --config configs/hist/<key>.yaml --out eval_out/hist_match/<short> [--limit N]
```

OpenCV mirror of the configured extractor (nFeatures/minThFAST/pyramid/upscale read
from the config; KB4 undistortion; BF-Hamming + Lowe 0.75 + RANSAC homography =
"correct matches"). Reports per-frame detections and, for offsets 1..4, the fraction
of frames with ≥500 correct matches to ≥1 of its 4 neighbours. Verdicts:
`frame_verdicts.csv`, `pair_matches.csv`. Note the AUTHORITATIVE detection count is
ORB-SLAM3's own `frame_stats.csv` `detections` column (its octree extractor pools
~30% more than OpenCV at the same thresholds). Result on the 43 low-res sets: median
99.5% of frames pass the match rule; failures are dusk/dark content, not thresholds.

### 4c. GPS[0] segment errors — `eval/hist_gps_segment_errors.py`

```bash
python3 eval/hist_gps_segment_errors.py --batch hist_baromag   # any summary batch
```

Per flight with >80% coverage AND ≥60 s of unspoofed GPS[0] after takeoff:

* **takeoff** = "Flight is started" in the session's `logs/LOG_DRONE.txt` (local wall
  clock; local→UTC offset recovered by rounding against the dataset epoch);
* **spoof mask**: GPS[0] fix is unspoofed where it agrees <30 m horizontally with
  interpolated GPS[1]. Most flights are spoofed FROM takeoff (constant 52–181 m drags
  or 10 000 km teleports) — only ~6 of 15 covered flights qualify;
* **alignment**: yaw from the MAGNETOMETER (tilt-compensated compass heading from BIN
  MAG+ATT, declination +9.3°E, minus SLAM body yaw, circular median over the first
  usable 30 s window with ≥30 m of GPS path), **roll/pitch (map tilt) from a
  full-flight free Umeyama**, scale+translation by least squares with R fixed.
  The SLAM world is gravity-aligned but **z-DOWN**: ENU = Rz(δ)·diag(1,−1,−1)·SLAM;
* **per 60-s segment**: residual horizontal translation [m] (xy centroid offset),
  rotation [deg] (angle of the segment's residual Umeyama rotation), scale
  (absolute `scale_abs` and |s−1|%). Scale is only reported for segments with
  **≥200 m GPS path** (unobservable below); the printout checks `scale_abs` against
  the required **(0.95, 1.1)** band. Percentiles p10/p25/p50/p75/p90 per flight.

Interpretation guide: rotation percentiles ≈ yaw drift (mag-anchored so no shape-fit
absorption); `map tilt vs vertical` line = the constant gravity error of the map;
horizontal p50 over flight length ≈ %-of-distance drift (best flights ~0.3%).

### 4d. Batch comparison one-liner

`awk -F, '$1=="<batch>"' runs/det_campaign_summary.csv` for coverage/vATE/gap;
`[DIVERGE]` in `logs/det_<tag>.log` for guard aborts (with the KF id and speed).

---

## 5. How the implemented fusions work (brief)

* **BaroFusion** (`include/BaroFusion.h`, edges in `src/Optimizer.cc`): keyframe-level
  `EdgeBaroZ` pulls the pose z toward the baro altitude with a **window-relative
  datum** (newest fixed KF, else window median — gauge-safe, suppresses RELATIVE
  drift; deliberately blind to a global tilt/offset). Present in LocalInertialBA and
  FullInertialBA; `ORB_BARO_FRAME_SIGMA` adds frame-rate z edges in the pose-only
  optimizations (last-KF-relative datum); at IMU init a few vertical-scale edges
  (`EdgeBaroScaleGDir`) anchor initial scale/gravity.
* **MagFusion** (`include/MagFusion.h`): per-keyframe `EdgeMagYaw` on the azimuth of
  `h = R_wb · m_body` (the Earth field is constant in the world), referenced to the
  window's fixed KF/median — again RELATIVE yaw-drift suppression; plus compass-aligned
  IMU init (world +x → magnetic north). Tuned σ = 10°.
* **Early-divergence guard** (`LocalMapping::ProcessNewKeyFrame`, commit `72b277b`):
  after IMU init, `ORB_DIVERGE_COUNT` consecutive KFs with ‖velocity‖ > `ORB_DIVERGE_VMAX`
  set `System::SetDiverged`; `mono_inertial_euroc` stops feeding frames and still saves
  the partial trajectory.

---

## 6. Current problems and next steps

**State**: 52 datasets converted; baro-only campaign (`hist_baro2`) done — 15/52 runs
>80% coverage, best endpoint gaps 2.27/3.64/4.17 m (292/260/173_golem27); mag σ sweep
done (σ10 wins); **full baro+mag σ10 campaign `hist_baromag` DONE** (comparison via
`awk -F, '$1=="hist_baromag"' runs/det_campaign_summary.csv`; segment table:
`eval_out/hist_baromag_segments.txt`). Fleet effect of mag σ10 vs baro-only:
17 (was 15) runs >80% coverage; **five flights rescued from divergence/short tracking**
(153_golem27 13.5→95.7%, 243_golem17 31→99.4% gap 2034→39 m, 357_golem17 53.5→97.2%
gap 1800→16 m, 182_golem27 82→98% gap 1208→50 m, 111_papa3 84.5→98.3% gap 338→39 m —
note 111 is helped by mag edges even though its compass fails ABSOLUTE-heading eval:
the edges only need RELATIVE consistency); best gaps hold (292: 2.32, 260: 3.53,
389_alma231: 5.40); regressions: 282_golem17 (22→6% cov), 52_golem28 (35→23%),
86_papa3, 405_golem17 (81→68%), and 173's lottery 4.17 m gap → 31 m. Segment errors
(8 scoreable flights now): rotations p50 3.1–8.5° on the healthy ones (156/173/153/359),
**153_golem27 = first flight with ALL motion-rich segment scales inside (0.95, 1.1)**;
155 shows a 45° "tilt" (map bent by a mid-flight event — inspect for resets), 357's
map is 0.575-scaled (consistent but far off band), 241/243 remain outside band.

1. **Map gravity tilt 4–15°** (the scale-mystery root cause, debugged on 156_golem27):
   nothing constrains absolute roll/pitch — IMU-init Rwg error persists; the
   relative-datum baro/mag edges are tilt-blind by design. The evaluator now
   compensates it, but the SLAM-side fix is open: an absolute gravity-direction
   constraint (long-baseline baro-vs-z consistency, or an accel-direction prior in
   FIBA). Caution: diary 20260715 found aggressive absolute z-datum designs explode —
   prefer a SOFT tilt prior.
2. **True metric scale 0.75–0.98** (rotation-free path-length ratio in cruise) vs the
   required **(0.95, 1.1)** band: 156 = 0.945–0.978 (2/3 in band), 173 = 0.91–0.97
   (1/2), 359 = 0.747 (0/1). Lever: scale anchoring — strengthen `EdgeBaroScaleGDir` /
   periodic baro ScaleRefinement (`ORB_BARO_SCALEREF_S` exists but historically caused
   v-scale thrash with z-edges — needs a careful, gated design), or GPS[1]-free
   speed prior from rangefinder where available.
3. **36/52 runs abort via the divergence guard at frames 65–500** — catastrophic init
   velocity blow-up in the climb at 5 fps, not gradual drift. Ideas: delay IMU init
   until cruise (env gates exist: `ORB_IMU_INIT_MINTIME/MINKF`), init-quality gating
   on velocity consistency, or the baro+upscale cross (below).
4. **Upscale-vs-baro confound**: 90_golem23 tracked 99% with upscale+no-baro but dies
   at frame 507 with no-upscale+baro. Run the cross (baro **with** upscale configs —
   `configs/hist/<key>.yaml` are the upscale variants) on the early-abort flights.
5. **Ground truth is scarce**: GPS[0] is spoofed from takeoff on most flights (only ~6
   scoreable); GPS[1] often stops mid-flight. Consider scoring against GPS[1] directly
   (extend §4c) and/or the NORA EKF positions in the hdf (`gps2_raw`-based).
6. **papa3 compass unreliable** (fits neither yaw-sign model; 111_papa3 excluded) —
   needs a per-airframe mag calibration/mount check before MagFusion helps there.
7. **155_golem27 mid-flight yaw jumps** (segment rotations 13→133° late) — inspect its
   map for silent resets/merges before trusting its segments.
8. Housekeeping: the `hist_magtune` batch rows with cov=0 are the dangling-mount
   accident (datasets drive = `nvme0n1p16`; after a reboot run
   `udisksctl mount -b /dev/nvme0n1p16` BEFORE any campaign — symlinked frames dangle
   otherwise and every run dies at frame 0 with exit 1).

**Quick-start for a new agent**: mount check (above) → `python3 eval/hist_select.py`
to see the fleet → run §3 example on `hist_156_golem27` → `python3
eval/hist_gps_segment_errors.py --batch <your batch>` → compare against
`hist_baro2`/`hist_baromag` rows in `runs/det_campaign_summary.csv`.
