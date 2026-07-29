# RESUME — how to continue (read this first)

Session date 2026-07-15. Workspace `~/praca/dev/orbslam3-eval`. Full detail in the numbered files here;
this is the actionable summary + priorities + improvement ideas.

## Where things stand
- **Timeshift question: SOLVED** (no timebase bug; it's inertial-init latency — see 06 §D). Done.
- **Full-res native-1640 experiment: NET-NEGATIVE** (see 00 headline + 06 final tally). ORB-MI legacy saved
  only a 646-pose fragment (15.4 %, t 21.9–101.7 s) on 538; measured saved NOTHING (empty-map crash); ORB
  mono 771/4197 (18 %); VINS DIVERGES on everything (legacy 889 km/42 km, measured 538 367 km); MEASURED
  IMU noise is strictly worse than legacy. **800-res remains the working baseline** (`nora538_half` +
  `nora20260709_mi_800.yaml` → 92 % on 538, in `nora_20260709/NORA_RESULTS.md`).
- **542 is broken by 56 zero-byte source frames** — fix before any 542 result (02 + Task priority 3).
- Tooling built & validated this session: `eval/{time_align_check, combined_plot, match_viz, height_drift}.py`
  (+ existing `baro_ate, validate_nora, anchor_nora, mag_fuse_nora, vins_to_tum`). Reference plots in `plots/`.
- Background runs may still be finishing when you resume — check:
  `ps -ef | grep -E 'mono_inertial|mono_euroc|vins_node'` and
  `find runs/nora5*_move -name 'f_native1640*.txt'`. The 4 delegated subagents detached their jobs; don't
  trust them to have collected results — collect from disk yourself.

## Priority-ordered actions
1. **[USER ASK] Circular sky-detection mask** — step 0: PREVIEW the benefit with the upgraded
   `eval/match_viz.py --mask-frac 0.70` vs without (no rebuild needed — it now supports the circular mask,
   CLAHE, `--scale` resolution study, and RANSAC-verified match counts; see 06 §B). Then implement per 08 §1
   (ORB-SLAM3 `ORBextractor.cc` + VINS `feature_tracker.cpp`), rebuild both, rerun with
   `ORB_MASK_FRAC=0.70 ORB_MASK_CX=788.9216 ORB_MASK_CY=637.1007` (new tag `*_mask`).
2. **[USER ASK] IMU-aligned frame debug viz + noise-dependent search region** — implement `eval/imu_align_viz.py`
   per 08 §2 (gyro-integrated ΔR warps frame k→k-1; per-feature predicted point + search circle whose radius
   scales with gyr_n/gyr_w; legacy vs measured overlay). Skeleton + math in 08 §2.
3. **Fix 542 dataset** (56 zero-byte frames) — patch `convert_nora_to_euroc.py` `list_frame_stamps` to skip
   0-byte PNGs (exact patch in 02), rebuild `nora542_move`. Unblocks every 542 run.
4. **Make ORB-MI actually save a trajectory** — either (a) fix the empty-map `SaveTrajectoryEuRoC` crash
   (guard it / save the largest map, not just the final active one), and/or (b) fix init so the map stops
   resetting (ideas below). Until then no native-1640 trajectory exists to plot/fuse.
5. Then the tooling-ready deliverables: combined plots (07), drawMatches inspection (06 §B), VINS+compass
   (05), legacy-vs-measured writeup — all one command each once a trajectory exists.
6. VINS divergence (ideas below) — the deepest open problem.

## SLAM improvement ideas (why native-1640 regressed; what to try)
The root cause across ORB-MI init failures and VINS divergence is the SAME: **weak high-altitude
inertial observability** (little parallax, weak/again-static accel). Resolution/calib/trim don't fix it.
Concrete ideas, roughly in expected-value order:
- **Keep the 800-res baseline for real results.** Native-1640 gives more features but not more geometry;
  it slows VINS below real-time (frame drops) and makes ORB-MI init pickier. Prove any full-res change
  against 800 first. (Consider `mi820` as a middle ground — configs exist.)
- **Circular sky mask (Task 1)** — clouds/sky give texture-less or moving matches that corrupt both the
  ORB map and VINS tracking; masking them is the highest-leverage cheap win, hence its priority.
- **Don't over-trust the IMU: inflate the Allan noise ×2–5.** Allan variance under-estimates in-flight
  noise (vibration/thermal). The raw measured set collapsed matching; the ×2 SI values in memory
  `project_imu_per_axis_noise` are the precedent. Sweep acc_n/gyr_n scale for both ORB-MI and VINS.
- **ORB-MI init**: the log shows "scale too small / not enough motion for initializing" → the IMU-init BA
  can't fix scale. Try: a longer init window, keep ~1–2 s of pre-motion for a clean gyro-bias seed while
  still starting near motion, or ORB-SLAM3's `IMU.InitIG`/fast-IMU-init options. The moving-start trim may
  have removed the quasi-static window ORB uses to seed gyro bias — test a trim 1–2 s EARLIER.
- **VINS divergence** (−z/gravity runaway): (i) verify `T_cam_imu`/`td` again — a small extrinsic/time error
  integrates to a gravity error at altitude; (ii) raise `keyframe_parallax`, lengthen the init; (iii) run
  VINS at 800 or downsample (full-res likely drops frames → bad preintegration windows); (iv) add a GLOBAL
  anchor — VINS `global_fusion` with NORA as a pose prior (05) fixes scale/gravity that pure VIO cannot.
- **IMU-guided matching (Task 2)** isn't just a viz: using the gyro-predicted search window in the tracker
  (small window from the tight measured noise) rejects cloud/sky mismatches and stabilizes tracking — the
  viz is the first step toward wiring it into the matcher.
- **Loop closure / NORA pose-graph anchor** for the residual monocular SCALE drift (the real 538 error;
  mag/baro can't fix it — see 05).

## Handy constants
- Rebase epochs: 538 `1783605974565266432`, 542 `1783607204961902080`.
- Motion-start ns: 538 `1783605978430903808` (→3.87 s), 542 `1783607231156786688` (→26.19 s).
- Native calib 1640: f=[570.3874,571.3253] c=[788.9216,637.1007] k=[0.037424,-0.0046179,0.0051669,-0.0022565].
- Legacy IMU noise: gyr 5e-4 / acc 1.3e-2 / gyrW 4e-6 / accW 9e-4. Measured: 3.65e-5 / 4.7175e-3 / 1.38e-6 / 4e-5.
- Containers: `vins-build`(→vins_out), `vins-measimu`(→vins_out_measimu). ORB env: ORB_TSJUMP_S=6.0, ORB_STATS_CSV.
