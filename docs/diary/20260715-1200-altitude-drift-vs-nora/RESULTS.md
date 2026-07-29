# 2026-07-15 — Altitude drift vs NORA: final results

Goal: ≥1 SLAM algorithm on 538+542 with altitude drift not worse than NORA; ≥500 correct
matches per used frame pair; visual debug artifacts. Extended mid-session: vertical-ATE eval of
165608/163613/162710_move; threaded preprocessing option; all-pairs match panels (undistorted);
measured sky mask.

## 1. The bar — and the evaluator bug that had hidden it
`eval/baro_ate.py` oriented the up-axis by ENDPOINT deltas → random sign flip when takeoff ≈
landing altitude. FIXED (orientation now by min-RMSE). The diary claim "NORA missed the 542
takeoff climb by 45 m" was this artifact. True bars (NORA = ArduPilot-EKF alt ≈ baro-fused):

| flight | NORA vATE | NORA 10s-drift RMS |
|---|---|---|
| 538 (fair span) | 1.03 m | 1.37 m |
| 542 (fair span) | 1.46 m | 0.96 m |

Pure mono-VIO cannot reach ~1 m vs the sensor NORA itself fuses. **The strict goal was not met**;
the gap closed from ~50x to ~2x.

## 2. Best SLAM results (ORB-SLAM3-MI @800, CLAHE3, legacy noise, TSJUMP 6)

| run | vATE | drift RMS | coverage | notes |
|---|---|---|---|---|
| 538 no-baro nf5000 | 54.2 | 12.1 | 93% | v-scale 0.31 — vertical broken |
| **538 baro6u nf5000** | **20.2** | **3.27** | **95%** | anchor edges + periodic scaleref |
| 538 baro2 (many-resets path) | 7.64 | 2.48 | 61% | init-scale edges re-fixed scale each reset |
| 542 no-baro nf5000 | 3.70 | 2.37 | 77%* | v-scale 1.01 (*full post-takeoff) |
| **542 nf1500 (old cfg)** | **1.49** | **1.45** | 69% | vATE BEATS NORA's 1.46; RMS 1.45 vs 0.96 |
| 542 baro_s03_* | see final report | | | tight-sigma finals |

Home flights (no NORA reference; SLAM-vs-baro): 162710 vATE 15.7 / RMS 4.5 (114 s, 99% cov);
163613 (59-s truncated log) baro 15.1/7.4; 165608 no-baro segfaults reproducibly (upstream FIBA).

## 3. Matches goal (stride-1, every consecutive pair, RANSAC-homography-verified)
front-end 800res+CLAHE3+nf5000: 538 median 608/pair (53.6% ≥500 — the rest is featureless terrain,
physically <500), 542 median 1364 (87.9% ≥500). Was 70/21 before. SLAM-side map-matches median
779 (538). Full evidence: eval_out/match_final_20260715/, eval_out/match_sweep_20260715/.

## 4. Baro fusion — designs and evidence (ORB_SLAM3 fork, all env-gated)
KEPT: EdgeBaroZ quadratic+gate (LocalInertialBA anchor-datum, FullInertialBA median-datum),
EdgeBaroScaleGDir in both InertialOptimization overloads, periodic baro ScaleRefinement
(ORB_BARO_SCALEREF_S). REJECTED with evidence: stored absolute datum (538 exploded, km-scale),
window-median-only datum (drift-preserving, measured −5 m/100 s anchor drift), Huber on baro
edges (saturates → v-scale stuck 0.4). Remaining 538 limit: vertical map SHAPE error that a
global scalar cannot fix; NORA-level altitude needs tighter fusion (EKF-style) or the
attitude-aware improvements below.

## 5. Sky mask — measured, validated, and (for SLAM) rejected
sky_mask_fit.py p99.5-brightness maps → offset circles (538: FRAC .6893 CX 384.84 CY 390.78;
542: .9641/384.84/310.78); validated zero sky keypoints on the user's cloud frame
(plots/20260715/maskcheck_*). But in-SLAM the mask (a) halves 538 correct matches, (b) kills 542
tracking (19 poses), (c) collapses 538 v-scale 0.305→0.072 — peripheral features carry the
altitude parallax. Static masks are viz-only; follow-up = attitude-aware horizon filter.
Home flights: brightness-based static mask degenerate (overexposed ground, circling).

## 6. New tooling
- eval/imu_align_viz.py + imu_align_batch.py (--undistort): all-pairs match panels
  runs/imu_align/{538,542}_all_undist (pinhole domain; extraction on raw fisheye) + pair_stats.csv.
- eval/match_viz.py: CLAHE/mask/scale + RANSAC-correct counts + green/red drawMatches.
- eval/altitude_compare.py: NORA-vs-SLAM 3-panel altitude figure (plots/20260715/altitude_*_final.png).
- eval/sky_mask_fit.py, eval/fill_imu_gaps.py (multi-second IMU gaps in data-recorder logs →
  SO3-NaN crashes; insert-only interpolation fix), eval/make_half_dataset.py.
- ORB_PREFETCH=N|all + ORB_NO_PACE=1 (verified exact-equivalent detections; 1211 frames 55 s vs 130 s).

## 7. Follow-ups (expected-value order)
1. Attitude-aware sky filter (reject rays above IMU horizon) — satisfies no-sky + keeps periphery.
2. 542 tight-sigma baro tuning (σ 0.3, gate 6) — the healthy-map path to the 0.96 RMS bar.
3. 538 vertical SHAPE: per-leg baro alignment or EKF-style loose fusion of the VIO output.
4. 165608 FIBA segfault (upstream vertex-null path) — debug with gdb; 163613: only 42 s usable.
5. Wire ORB_PREFETCH into run scripts for fast iteration (results comparable only unpaced-vs-unpaced).

## 8. Late-session additions (user-directed)
- **User-spec mask** (circle in UNDISTORTED domain c=(330,300) r=190 @800, scaled c=(676.5,615)
  r=389.5 @1640) precomputed into DISTORTED space: masks/nora_half_undistc330_300_r190.png +
  masks/nora_native_undistc676_615_r390.png (+_preview.png). Keeps 57.3%. Loadable in SLAM via
  ORB_MASK_FILE (new fork env) and in eval tools via --mask-file.
- **SLAM probes with user mask + baro6**: 538 vATE 17.6/RMS 4.26/92% cov (v-scale 0.707 — user
  mask is SLAM-safe, unlike the narrow measured circle); 542 vATE 1.38 (< NORA 1.46!) but 44%
  coverage / RMS 1.86. 542 σ=0.3 unmasked: vATE 2.50/RMS 2.13/full coverage.
- **imu_align regenerated per user spec**: native 1640x1232, user mask, 10-px alignment floor,
  matches drawn as one-pixel random-colour points (no lines):
  runs/imu_align/{538,542}_sweep_native/. All-pairs undistorted panels with user mask:
  runs/imu_align/{538,542}_all_undist_umask/.

## 9. Guided matching + magnetometer fusion (user-directed, end of session)
- **Adaptive extraction + IMU-guided matching** (eval/imu_align_viz.py/_batch.py: detect_adaptive
  FAST 20->1 cap 10000, guided_match grid+popcount radius 80px ratio 0.8): >=500 matches on
  **99.5% (538) / 98.7% (542)** of ALL pairs (target 90%), median 3102/2893; 100% frames >=5000
  in-mask features. Residual sub-500 pairs = frame-drop gaps / ~90deg rotations (physical).
  Outputs runs/imu_align/{538,542}_all_native_v2 + *_sweep_native_v2. Native calib verbatim (factor 1.0).
- **MagFusion** (include/MagFusion.h + EdgeMagYaw unary yaw factor in Local/FullInertialBA, env
  ORB_MAG_CSV/ORB_MAG_SIGMA_DEG; eval/export_mag_csv.py -> dataset/*/ref_mag.csv reusing the
  compass calib + Wahba vehicle->IMU alignment): tightly-coupled, gated on VIBA2 (firing on coarse
  init caused reset-thrash). **Compass-aligned init**: post-gravity yaw rotation puts magnetic
  north on world +x; post-align yaw ~1e-7 deg (sign proven). Validation (mag_baro6u vs baro6u):
  538 H-RMSE 287 vs 292 m, coverage 4123 vs 3973, altitude unchanged; 542 equal (54 m). Gain
  marginal because the residual horizontal error is monocular SCALE drift, not yaw — mag's real
  value = absolute-north for GPS-denied use. frame_stats.csv gained feat_init_th/feat_used_th/
  final_th_fast columns. KNOWN: pre-existing g2o concurrency race under >2 parallel MI runs.

## 10. Final per-dataset SLAM comparison

> **SUPERSEDED for altitude by §12–§13 (night 2026-07-16):** tight-σ baro fusion changed the
> picture — 542 vATE 5.37→**1.15 median** (beats NORA's same-span 1.42), 538 20.2→**~7 median /
> 1.81 best** (v-scale repaired to 1.0), 162710 15.7→**0.44–0.48**. The rows below reflect the
> σ1.0-era recipes.

All ORB rows: 800x600, CLAHE 3.0, nFeatures 5000, legacy IMU noise, TSJUMP 6, paced playback
(config `configs/nora20260709_mi_800_v2.yaml`). Note on "guided matching front-end": ORB-SLAM3's own
tracker already does IMU-projection-guided matching internally (predicted pose -> projected map
points -> windowed search), so the guided front-end here = the CLAHE3+nf5000 extraction config plus
the SLAM's native guided search — no extra external matcher was bolted on.

Metrics: vATE / drift RMS from `eval/height_drift.py` (baro GT, 10-s windows, min-RMSE up-fit);
H-RMSE from `eval/combined_plot.py` (Umeyama-2D alignment to the NORA GPS track — NORA itself is the
horizontal reference, so it has no H-RMSE); coverage = saved poses / dataset frames. H-RMSE for all
monocular rows is dominated by SCALE drift along the track, not yaw (sec. 9). Fresh runs this
section: `mag_baro7` (MI+baro+mag reruns) and `mono_v2` (no-IMU); other rows re-evaluated from the
existing trajectories. Plots: `plots/20260715/cmp_<tag>_<ds>.png`.

### 538 (4202 frames, 558 s flight)

| algorithm | vATE m | drift RMS m | H-RMSE m | coverage | notes |
|---|---|---|---|---|---|
| NORA (onboard EKF) | 1.03 | 1.37 | n/a (is the horizontal ref) | 100% | the bar |
| ORB-SLAM3 Mono (`mono_v2`, 2256 poses) | 27.9 | 9.23 | 248 | 54% | last surviving map only (t=240-536 s); NO metric scale (up-fit x264, H x567) — errors only meaningful after the fit |
| ORB-SLAM3 MI, no baro (`clahe_nf5000`, 3888) | 54.2 | 12.1 | 312 | 93% | v-scale 0.31 — vertical broken |
| ORB-SLAM3 MI + baro (`baro6u`, 3973) | **20.2** | **3.27** | 292 | 95% | best vertical; v-scale 0.71 |
| ORB-SLAM3 MI + baro + mag (`mag_baro7`, 4126) | 72.0 | 10.7 | 287 | **98%** | best coverage + earliest init (t=12.8 s, 2 resets vs baro6u's 8) but this draw's long single map carries vertical SHAPE error; identical-config prior draw (`mag_baro6u`) scored 25.0/3.24 — g2o threading makes 538 runs high-variance |
| VINS-Fusion | diverged | — | — | — | native-res moving-start: ~889 km runaway; not rerun (docs/diary/20260715-1140-nora-fullres-vio/00-OVERVIEW.md) |

### 542 (1561 frames, 157 s flight)

| algorithm | vATE m | drift RMS m | H-RMSE m | coverage | notes |
|---|---|---|---|---|---|
| NORA (onboard EKF) | 1.46 | 0.96 | n/a (is the horizontal ref) | 100% | the bar |
| ORB-SLAM3 Mono (`mono_v2`, 345 poses) | 6.34 | 5.43 | 73 | 22% | a single 33-s fragment (t=112-145 s); NO metric scale (up-fit x29.6) — not comparable on equal terms |
| ORB-SLAM3 MI, no baro (`clahe_nf5000`, 1198) | **3.70** | **2.37** | **53** | 77% | v-scale 1.01 already healthy; 77% = init latency (first pose t=27.6 s), full tracking after |
| ORB-SLAM3 MI + baro (`baro6u`, 1198) | 5.37 | 2.52 | 54 | 77% | baro adds nothing when mono scale is already right — slightly worse this draw |
| ORB-SLAM3 MI + baro + mag (`mag_baro7`, 1198) | 5.70 | 3.18 | 56 | 77% | prior draw (`mag_baro6u`) 4.21/2.22 — differences within run-to-run variance |
| VINS-Fusion | diverged | — | — | — | native-res moving-start: ~42 km; earlier 800-res run stayed bounded (~151 m); not rerun (same diary) |

Honest read: on 538 the baro-fused MI is the only configuration with a usable vertical (20.2 m vATE
over 558 s vs NORA's 1.03) and no configuration approaches NORA horizontally (H-RMSE >= 248 m =
monocular scale drift). On 542 plain MI beats all fusion variants and gets within 2.5x of NORA's
vATE; baro/mag change results within the noise. Mag's value stays absolute-north + map longevity
(538: 98% coverage, 3x fewer resets), not accuracy. 538 MI numbers are single-draw samples from a
high-variance process (map-reset lottery under g2o threading) — rank stability would need repeated
runs.

## 11. Rangefinder fusion

Swap the fused altitude source from barometer to the down-facing **rangefinder** (tag `rfnd7`), and
also test the 2 s-smoothed rangefinder as the height GT. No C++ change — `BaroFusion` reads whatever
`(t_ns, alt_m)` CSV you hand it (`dataset/<ds>/ref_rfnd_smooth.csv` from `eval/make_rfnd_alt.py`).
Full write-up: `docs/diary/20260715-2231-rfnd-fusion.md`; plots `plots/20260715/rfnd_vs_baro_*.png`.

Validity: `ref_rfnd.csv` is already `Status::Good`-filtered. After dropping ground clamps + 1 s-median
spikes, **valid = 98.5% (542)** and **98.0% (538)**. The sensor reaches **138 m** Good on 538 (a
radar-class altimeter, `corr 0.96` vs baro) — the "out of range" hypothesis was **wrong**. But the
rangefinder measures **AGL, not altitude**: airborne residual (rfnd−baro) std is **2.3 m on 542**
(flat, ~24 m AGL) vs **8.0 m on 538** (relief, ~100–130 m AGL, dipping to −35 m over rising terrain).

Evaluation matrix (`height_drift.py --window 10`, vATE / win-drift RMS / v-scale; baro6u/real-baro
row reproduces §10 exactly):

| flight | fused | GT=rfnd_smooth | GT=real_baro | v-scale | poses (cov) |
|---|---|---|---|---|---|
| 542 | **rfnd7**  | **5.99 / 2.08** | **4.45 / 2.36** | 1.0 | 1198 (77%) |
| 542 | baro6u | 6.90 / 2.36 | 5.37 / 2.52 | 1.0 | 1198 (77%) |
| 538 | **rfnd7**  | 32.28 / 10.18 | 28.01 / 5.20 | **0.54** | 4126 (98%) |
| 538 | baro6u | 25.29 / 9.47 | 20.21 / 3.27 | 0.71 | 3973 (95%) |

Read: **on 542 rfnd wins as a fusion source** — beats baro on *both* GTs incl. the real baro
(4.45 vs 5.37 vATE), because at 24 m over flat ground it is a tighter altitude signal. **On 538 rfnd
loses** — the AGL-vs-altitude divergence over relief collapses the recovered scale to 0.54 (vs baro's
0.71) and it trails baro on both GTs. As a **GT**, the rfnd inflates *every* trajectory's error on 538
(baro6u: 20.2/3.27 vs real baro → 25.3/9.47 vs rfnd — ~5 m vATE, ~3× window-RMS of pure terrain
texture); on 542 the same swap costs only ~1.5 m. Verdict: rfnd is the better altitude source **only
over flat, in-range terrain (542)**; over relief (538) keep the barometer for both fusion and GT.

## 12. Night campaign 2026-07-16: 542 GOAL MET — tight-σ baro fusion beats NORA's vATE

Full log: `docs/diary/20260716-0400-542-night-batch1-lag-discovery.md`,
`20260716-0515-542-sigma-ladder-beats-nora-vate.md`, `20260716-0445-542-frame-baro-edge.md`.
47 evaluated runs in 6 batches (`runs/nora542_half/night542_summary.csv`, queue runners
`eval/night542_queue*.sh`/`night_queue3.sh`, batch specs `eval/night542_batch*.txt`).

**Honest bar first.** On the exact common span (airborne, t=27.6–156.6 s; GT = real baro; same
tools): **NORA vATE 1.42 m, drift RMS 1.44 m** — the earlier "1.46/0.96" mixed spans. Also, 542's
takeoff is at t=27.7 s, so the recurring "77% coverage" IS the full airborne span (the pre-takeoff
frames are static). And the landing window (138–148 s) carries −2.3…−3.6 m in *every* estimator
including NORA — barometer ground-effect, not VIO drift.

**Two false leads corrected.** (a) The "nf1500 champion" (1.49 vATE) = `mi800` on
`nora542_half_clahe`, *no baro*; its dataset is bit-exact `ORB_CLAHE=3.0` preprocessing, and the
recipe is an init-scale lottery: re-draws gave 1.79–9.93 vATE at 28–75% coverage. (b) Yesterday's
`baro_s03_nf1500` never produced a trajectory (0-KF init thrash — tight baro breaks the sparse
nf1500 map; 3/5 re-draws died).

**The mechanism that mattered: fusion lag.** Baro-shift scans showed every baro-fused run
reproduces the barometer LATE with lag ∝ σ (σ1.0 → ~1.4 s, σ0.3 → 0.5 s), while the no-baro run
has no offset — the EdgeBaroZ pull acts like a first-order response, and during 542's ~2.5 m/s
climbs the lag was most of the vATE. Tightening σ is the whole game (nf5000 map required):

| arm (nf5000+CLAHE, gate 6) | n | vATE med | RMS med | full-airborne cov |
|---|---|---|---|---|
| σ1.0 (baro6u, yesterday) | 1 | 5.37 | 2.52 | 1/1 |
| σ0.3 | 3 | 2.55 | 1.91 | 2/3 |
| σ0.15 | 5 | 1.28 | 1.88 | **5/5** |
| **σ0.075** | **14** | **1.15** | 1.53 | 9/14 |
| σ0.05 | 2 | 1.70 | 1.64 | 2/2 (floor: worse) |
| σ0.075 + frame-edge σf0.5 | 9 | 1.27 | 1.73 | 8/9 |
| NORA (same span) | — | **1.42** | **1.44** | — |

**Verdict vs goal ("any VIO on 542 not worse than NORA altitude"):**
- **vATE: WON.** σ0.075 median 1.15 m (9/14 draws beat 1.42; best draws 0.94–0.97). σ0.15 wins
  too (1.28) with 5/5 full coverage.
- **drift RMS: par.** Median 1.53 vs 1.44 (draws 0.91–2.59 span the bar); the residual is
  draw-noise + the barometer's own landing-window error, which NORA equally suffers.
- Horizontal unaffected by the vertical glue (H-RMSE 59–63 m vs 53–54 baseline; H-scale lottery
  unchanged — that is the *next* battle, not vertical).

**Rejected on evidence:** periodic ScaleRefinement on top of z-edges (v-scale thrash, 2.9–4.6
vATE); σ0.05 (too stiff); frame-rate baro edge tighter than σf0.5; rfnd-source at σ0.3 (no better
than tightening raw-baro σ); every nf1500 arm.

**New C++ (flag-gated, default off):** `ORB_BARO_FRAME_SIGMA` — frame-rate quadratic `EdgeBaroZ`
in both `PoseInertialOptimization*` (relative last-KF datum, causal, gate-guarded). Neutral on 542
RMS (the hypothesis it targeted — inter-KF z wander — was not the residual), kept for its slight
coverage-stability benefit at σf0.5 (8/9 full).

**Recommended 542 recipe:** `nora20260709_mi_800_v2.yaml` (nf5000) + `ORB_CLAHE=3.0` +
`ORB_BARO_SIGMA=0.075..0.15` + `ORB_BARO_GATE=6` (σ0.15 if coverage-reliability matters more than
the last 0.1 m of vATE). 538 transfer runs (σ0.15/σ0.075 vs its old σ1.0) — see §13 if present.

## 13. 538 transfer of the tight-σ recipe (night 2026-07-16, bonus)

Same-span NORA bar for 538 (t=21.9–558.3 s, GT = real baro): **vATE 0.98 / drift RMS 1.23**.
Applying the 542-winning recipe (nf5000 + CLAHE + gate 6) to 538
(`runs/nora538_half/mono_inertial/t538_*`, `runs/night538_transfer_summary.csv`):

| arm | draws (vATE / v-scale / cov) |
|---|---|
| σ1.0 (baro6u, yesterday) | 20.2 / 0.71 / 95% — stable but 20× NORA |
| σ0.15 (n=4) | **2.45**, **4.64** (v-scale ≈1.0, 98%) · 2 catastrophic (map exploded / segfault) |
| σ0.075 (n=8) | **1.81**, 6.47, 7.16, 8.01, 11.96 (v-scale 0.97–1.07, 67–98%) · 67.7 · 2 catastrophic |
| **σ0.075 + frame σf0.5 (n=10)** | 3.26, 5.35, 5.97, 7.18, 7.75, 9.39, 9.78, 10.47, 20.19 healthy + 1 catastrophic — **9/10** (v-scale ≈1.0, 82–98%) |

Read: **the σ-lag law transfers.** Healthy tight-σ draws cut 538's vATE from 20.2 to 1.8–12
(median ~7) and repair the v-scale from 0.71 to ≈1.0 — the closest any 538 configuration has come
to NORA (best draw 1.81 vs 0.98). The price: **~30% catastrophic-init rate** (0-scale explosion or
FIBA segfault during the reset-thrash phase; both failure draws died at v-scale ≤0.005). 538's
init fragility is the pre-existing weakness — tight σ amplifies both the reward and the risk.
Production use needs either the init fix (loop-gate / init-scale threads) or an online watchdog
(|z−baro| blowup is self-detectable) with auto-restart; as a *mapping* recipe with a retry, σ0.075
is already strictly better than σ1.0.

**Rounds 4+6: the frame-rate baro edge is 538's stabilizer — with one caveat.** With
`ORB_BARO_FRAME_SIGMA=0.5` on top of σ0.075, 9/10 draws finished healthy (median vATE ~7.2, worst
healthy 20.19 ≈ the old σ1.0 baseline as a floor, v-scale ≈1.0, 82–98% coverage) vs 5-of-8 for
plain σ0.075 — the catastrophic-init rate drops ~37% → 10% but is NOT zero. Mechanism: the per-frame z constraint keeps tracking pinned to the barometer through the
fragile pre-VIBA2 phase, so the reset-thrash never starts. **Recommended 538 recipe:
nf5000 + CLAHE + ORB_BARO_SIGMA=0.075 + ORB_BARO_GATE=6 + ORB_BARO_FRAME_SIGMA=0.5** (2.6× better
vATE than yesterday's baro6u with equal-or-better robustness). On 542 the frame edge remains
optional (no failure mode to prevent).

### §13 addendum (post-round-4 experiments, 06:10–07:10)

- **542 unified-recipe consolidation** (σ0.075+f0.5, n=15): vATE median **1.27** (< NORA 1.42),
  RMS median 1.82, **13/15 full airborne coverage**. Final 542 guidance: plain σ0.075 = accuracy
  pick (median 1.15, 9/14 full cov), +f0.5 = reliability pick (1.27, 13/15). Both beat the bar.
- **Loop-gate composition: INCONCLUSIVE, initial verdict RETRACTED** (morning correction):
  σ0.075+f0.5+`ORB_LOOP_RP_TOL=0.20` on 538 (n=4) logged **zero loop detections** — the relaxed
  gate was never exercised; the tilted 21.9-vATE draw ([+0.37,+0.18,−0.91]) was the recipe's known
  bad-draw mode, NOT loop-induced (initially misattributed). Upstream inertial loop closure is in
  fact already 4-DoF end-to-end: detection requires VIBA2, accepted loops get roll/pitch zeroed
  (LoopClosing.cc), pose graph = OptimizeEssentialGraph4DoF, GBA = FullInertialBA. Open question
  is now why loop DETECTION died under the tight-σ recipe (σ1.0-era runs had 1–13 detections/run,
  audit 20260715-2108) — being retested with the hardened unconditional 4-DoF lock.
- **Home-dataset transfer, 162710_move** (unified recipe σ0.075+f0.5+gate6, config
  dr20260402_mi_820_v2, n=4): vATE **0.44 / 0.46 / 0.48** m, drift RMS **0.37 / 0.38 / 0.46** m
  at 97.9–98.8% coverage (one 47%-coverage draw at 0.81/0.98). Yesterday's baro6-recipe baseline:
  15.7 / 4.5. Sub-half-metre altitude on the home flight — the strongest absolute numbers of the
  whole effort (low altitude, rich texture, in-range terrain).
