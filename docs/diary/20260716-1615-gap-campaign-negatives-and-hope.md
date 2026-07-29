# Gap campaign, part 3 — three clean negatives, one live hope

**Date:** 2026-07-16 ~16:15. Targets: gap <5 m (538) / <2 m (542).

## Negative results (each closes a hypothesis)

1. **Free-scale Sim3 (`ORB_PR_FREE_SCALE`, commit 4b73997)** did not unlock verification:
   bestInliers stays 1–3 even with 7-DoF freedom and N=24–38 — the mid-flight BoW candidates on
   542 are **descriptor aliasing on self-similar farmland**, and RANSAC rejects them *correctly*.
   The scale-drift hypothesis for the Sim3 failure is dead: these matches are simply wrong.
2. **The true 542 revisit (landing-over-takeoff, candidates KF4/KF5 at t=139–156 s, 22–52 BoW
   matches every few KFs)** is starved at the solver: only N=1–7 of those matches survive the
   both-sides-mapped filter (nf5000 keypoints, ~10% mapped) — too few to verify even when true.
   542 loop closure is blocked at the front-end level, not at thresholds.
3. **Magnetometer yaw factor does not shrink the 542 gap** (9.3/32.3/39.7 m vs 3.4–44 baseline
   distribution): the horizontal endpoint error is H-scale draw noise, not yaw curl. (Consistent
   with the night finding: mag ≈ neutral accuracy, value = absolute north.)

## Positives

- **Vertical anchor works**: with the texture-low-passed fused baro+rfnd reference, the 542 draw
  that landed v-scale 1.0 closed dz to **−0.15 m** (vs +2.1…+3.4 baro-only). dz scales with
  (1−v-scale)·Δh — the fused reference fixes the reference; the v-scale draw fixes the rest.
- **538 candidate volume is huge** (~700 candidate-KFs, 1500–1800 Sim3 attempts/run) and the one
  draw that verified loops (13/13, stock thresholds!) was the best-map draw (v-scale 0.993,
  vATE 3.89). Loop verification succeeds ⟺ good-map draw. With RP_TOL 0.20 + YAW_TOL 0.60 +
  4-DoF lock, a good-map draw should now ACCEPT its loops → 8-draw harvest running (fs538_r4-11).

## 538 vertical note

Fused-arm dz on 538 runs +22…+49 m — scale-driven (v-scale 1.05–1.14 draws × ~130 m descent),
not landing-dip-driven. The 538 gap is dominated by horizontal drift (~60–100 m) that only loop
closure can repair; dz is secondary.

## Current best-known vs targets

| flight | best gap | recipe | target |
|---|---|---|---|
| 542 | **3.42 m** (dxy 2.7, dz 2.1) | baro σ0.075+f0.5 (night draw r15) | 2.0 |
| 538 | **61.8 m** | σ0.075+f0.5 (night draw r6) | 5.0 |

542's <2 m needs BOTH dxy<1.9 and dz<0.6 in one draw — dz is now solvable (fused-LP + v-scale≈1
draw), dxy floor so far ~2.6. 538's <5 m needs an accepted loop at landing; harvest in progress.
