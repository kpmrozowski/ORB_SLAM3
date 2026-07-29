# Determinism campaign — final (2026-07-17 ~04:00)

User asks: (1) endpoint-gap on circle_move from ts 1775119067080676352; (2) 10× runs on all
three datasets, quantify determinism; (3) if spread big, make the algorithm fully deterministic
without losing quality; (4) revisit algorithm modifications after determinism.

## 1. Determinism: measured, dissected, largely fixed

Baseline (threaded stock architecture): gap spreads 3.4–53 m (542) and 62–133 m + catastrophic
lottery (538). Four sources found and addressed (diary 20260717-0130, commits 24c0738+829cba2):
threads → sequential ORB_DETERMINISTIC mode; pointer-ordered containers → IdLess(mnId);
Eigen alignment-peeled vectorization → EIGEN_DONT_VECTORIZE; and an **upstream uninitialized
heap read consumed in the first post-IMU-init tracked frames** — proven by MALLOC_PERTURB_=42
(bit-identical md5s) and by the poisoned-quality counterfactual; bisect pinned it past
bit-identical extraction/init/FIBA (chi2 equal to 17 digits). Not yet source-fixed — it remains
as an occasional branch flip / last-bit jitter. Also fixed on the way: uninitialized
`pBiggerMap` segfault in the trajectory savers, and a self-inflicted double-FIBA from a debug
probe inserted into a braceless if (contaminated batch discarded and re-run).

## 2. The 10× measurement (deterministic mode, `runs/det_campaign_summary.csv`)

| dataset | vATE draws | gap draws | coverage | md5 behaviour |
|---|---|---|---|---|
| 542 ×10 | 0.97–1.02 | **2.56–2.86** | 1201/1201 ×10 | all differ (jitter) but metrics pinned |
| 538 ×10 | 8.37 / 15.00 | 128.2 / 601.0 | 97% ×10 | **9 of 10 bit-identical**; 1 flip draw |
| circle ×3 | — | — | 0 | does not initialize (see §4) |

Compare threaded spreads: 542 galloped 3.4–53; now 0.30 m band. 538's "spread" collapsed to a
binary flip between two reproducible trajectories.

## 3. Deterministic knob revisit (single runs are now representative — 20 arms, ~1 h)

542 (baseline σ0.075+f0.5+baro: 0.99/2.71):
- σ0.15 → gap **2.41**; mag → 2.51; **σ0.15+mag → 2.40 (winner; repeats 2.46, 2.83)**, vATE 1.12;
- σ0.05 → 3.21; frame-edge off → neutral (2.74);
- fused_tilt ref → dz collapses to **0.50** but horizontal blows up (gap 7.5–11) — rangefinder
  reference distorts 542's cruise geometry under tight σ regardless of the texture low-pass;
- landing-only fused ref (blend 8→14 m) → 4.35–4.62, dz NOT improved — dead end, the anchor is
  too brief to pull the tail and still perturbs local geometry.

538 (baseline det10 = fused_tilt σ0.075+f0.5: 15.0/601):
- plain-baro ref → **98.4/6.62** (fused_tilt is HARMFUL on 538: climb-phase tilt correction
  reshapes the early map — gap 6× worse; untilted fused ≈ baro);
- **+mag → 90.3/5.94 (winner)**; σ0.15 → 264 (worse); frame-edge off → 524; loop stack → 748
  with a reset (loop-detection starvation unchanged).

**Final recommended recipes (deterministic mode):**
- 542: `ORB_DETERMINISTIC=1 ORB_NO_PACE=1 OMP_NUM_THREADS=1` + CLAHE3 + baro σ0.15 gate6 +
  f0.5 + mag → **gap 2.40–2.83 m, vATE ~1.12, full airborne coverage** (target <2 m: not
  reached; residual = dz ~1.8 (baro-datum/landing physics) + dxy ~1.6).
- 538: same but σ0.075 + mag, plain baro → **gap 90.3 m, vATE 5.94, 97%** (target <5 m:
  requires functioning loop closure or an absolute reference; upstream Sim3 starvation stands).

## 4. circle_move

Rebuilt from the user's stamp (the flight is the final ~11 s of the session; the old conversion
covered only pre-flight ground — IMU-activity profile in diary 20260717-0130). 270 frames /
14 s / IMU 372 Hz / config `circle_mi_800native*.yaml`. **Mono-inertial init does not complete
within the 14-s flight** in either architecture (visual init at ~2 s, then loses tracking in the
aggressive circle before IMU init stabilizes; deterministic mode reproduces the failure
identically). Gap <5 m is therefore UNVERIFIABLE today. Next lever: shorten inertial-init gates
(env for minTime/nMinKF/VIBA schedule) and/or motion-robust front-end for the fast circle.

## 5. Infrastructure

`eval/det_queue.sh` (parallel deterministic batches — safe N-wide, no g2o race), md5 column in
`runs/det_campaign_summary.csv`, `ORB_DET_DEBUG` bisect instrumentation (frame/map/init/FIBA/
preintegration fingerprints) left env-gated in the fork for the remaining uninit hunt.
