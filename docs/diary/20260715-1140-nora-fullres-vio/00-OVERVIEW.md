# NORA golem27 20260709 — full-resolution VIO session (2026-07-15)

Handoff for a context refresh. This directory documents everything done in the 2026-07-15 session on
`~/praca/dev/orbslam3-eval` (the NORA drone VIO workspace). Read **RESUME.md** first to continue.

## What this session was about
Building on the prior 800×600 work (see `nora_20260709/NORA_RESULTS.md` and memory
`project_nora_20260709_vio`), this session:

1. **Root-caused the "~30 s NORA↔VINS timeshift"** → it is NOT a timebase bug (see 06-statistics.md
   "timeshift"). All estimators share the rebased clock (proven 4 ways). The gap is inertial-init latency.
2. **Reran all SLAM at FULL resolution 1640×1232 with the NATIVE kalibr calibration**
   (`golem27-satic_calib/regression_postchange/camchain.yaml`) instead of the 800 downscale — because the
   old 1640 attempt used *linearly-scaled* intrinsics (11 % tracking); the native fisheye distortion fit
   was the missing piece. See 03-configs.md.
3. **Trimmed both datasets to post-takeoff motion** (user-supplied "accel jumps +5 %" timestamps) to give
   the estimators IMU excitation for init. See 02-datasets-convert.md.
4. **Compared legacy vs Allan-variance-measured IMU noise** (user-supplied Allan summary). See 03/06.
5. Built the analysis/plot tooling: `eval/{time_align_check,combined_plot,match_viz,height_drift}.py`.

## Headline findings — full-res native-1640 is NET-NEGATIVE vs the 800 baseline
**Bottom line: keep the 800-res pipeline as the working baseline. Full-res native-1640 (this session's
main experiment) regressed on every estimator.** Details:
- **ORB-SLAM3 mono-inertial collapsed at native-1640.** Legacy noise saved only a **646-pose fragment
  (15.4 %, t 21.9–101.7 s)** on 538 — the map then kept resetting ("IMU not initialized / scale too small /
  not enough motion") and the later maps never survived to save. MEASURED noise saved **nothing** (empty
  final map → `SaveTrajectoryEuRoC` throws `std::system_error`, rc=134). Mid-run matches were OK (legacy
  ~72/frame) but init never stabilized. Contrast: prior **800-res** MI on 538 tracked **92 %**. Native-1640
  is a regression, consistent with the standing note "run NORA at calib res 800, not native 1640".
- **ORB mono-only** saved 771/4197 poses on 538 (18 %). 542 runs (mono, MI-legacy, MI-measured) all died
  at frame ~1212 on the first 0-byte source PNG — no 542 trajectory exists.
- **Allan-variance MEASURED IMU noise is strictly WORSE** for ORB-MI (even tighter → over-trusts IMU →
  matching collapses: 538 **89.9 %** of frames < 30 matches vs legacy 5.1 %; 542 87 %). Legacy noise wins.
- **VINS-Fusion still DIVERGES** with moving-start + full-res: legacy noise 538 → ~889 km, 542 regressed
  from BOUNDED (~151 m at 800) to ~42 km; MEASURED noise 538 → ~367 km (also diverged; 542 never run —
  pointless given the rest). `IMU excitation not enouth` still fires 30×/20×; runaway is −z (gravity)
  dominated. VINS init is the unsolved problem (see 06 + RESUME).
- **DATASET DEFECT: `nora542_move` has 56 zero-byte source PNGs** (e.g. `156667267584.png`). Any run dies
  at the first one (~frame 1212/1267). MUST rebuild 542 skipping 0-byte frames (see 02 + RESUME) before
  any 542 result is valid. 538 is clean (0 corrupt).
- **Interpretation**: the real bottleneck is high-altitude IMU-init observability (weak parallax + weak
  accel excitation). Resolution/calibration/trim don't fix it. See RESUME "SLAM improvement ideas".

## Queued tasks NOT yet done (full specs in 08-pending-tasks-and-plans.md; priority in RESUME.md)
- **Circular detection mask** (diameter = 70 % image height at the principal point) to filter the sky —
  modify ORB-SLAM3 extractor + VINS feature tracker, rebuild, rerun all. **User's latest explicit ask.**
- **IMU-aligned frame debug viz** + noise-dependent match search region. **User's latest ask.**
- Combined NORA/ORB/VINS trajectory plots; drawMatches inspection; VINS+compass; legacy-vs-measured writeup.

## Directory map
- `00-OVERVIEW.md` (this) · `RESUME.md` (how to continue + ideas)
- `01-containers.md` — Docker/VINS containers
- `02-datasets-convert.md` — dataset conversion + post-takeoff trim
- `03-configs.md` — ORB + VINS configs, native calib, IMU-noise variants
- `04-run.md` — how to run every estimator (+ `run_all.sh` one-shot)
- `05-compass-fusion.md` — magnetometer/compass fusion (separate handoff, as requested)
- `06-statistics.md` — features/frame, matches/pair, per-10 s height drift, timeshift, all stats + how to regen
- `07-plots.md` — trajectory PNGs + how to generate
- `08-pending-tasks-and-plans.md` — copy-paste-ready plans for the circular mask + IMU-align viz + more
- `run_all.sh` — reproduce the whole pipeline · `data/`, `plots/` — generated artifacts
