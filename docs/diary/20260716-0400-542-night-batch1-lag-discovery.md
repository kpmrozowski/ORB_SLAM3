# 542 night campaign — batch 1 verdict + the baro-fusion lag discovery

**Date:** 2026-07-16 ~04:00 (autonomous night run, target: any VIO ≤ NORA on 542 altitude:
vATE 1.46 m / drift RMS 0.96 m)

## Corrections to yesterday's record

- The "542 nf1500 champion" (vATE 1.49/RMS 1.45/69%) is `runs/nora542_half_clahe/mono_inertial/mi800`
  — **no baro at all**, offline-CLAHE dataset. RESULTS.md line ~29 implied a baro run; wrong.
- Yesterday's `baro_s03_nf1500` produced **no trajectory** (0-KF map, init thrash) — the recipe never
  had a data point.
- The offline-CLAHE dataset `nora542_half_clahe` is bit-exact `cv2.createCLAHE(3.0, 8×8)` of
  `nora542_half` (verified MAD=0 on frame 0), i.e. identical to `ORB_CLAHE=3.0` in-SLAM. Same IMU,
  same meta. So the champion recipe ≡ *nf1500 + CLAHE3/8 + no baro*.
- Coverage ceiling: 542 takeoff is at t=27.7 s (baro +2 m), camera span 156.6 s. The recurring
  1198-pose / 76.7% coverage **is the full airborne span** — a motion-initialized VIO cannot cover
  the static pre-takeoff frames. 77% ≈ 100% of achievable.

## Batch 1 (12 runs, nora542_half, env CLAHE 3.0, GT = real baro)

| arm | draws: vATE / drift RMS / v-scale |
|---|---|
| nf1500 + baro σ0.3 g6 ×5 | 17.1/10.9/0.12 · 4.9/5.1/0.60 · died(19 poses) · SIGABRT(0 KF) · 3.8/3.2/0.67 |
| nf1500 + rfnd σ0.3 g6 ×2 | 6.7/2.0/0.76 · died(15 poses) |
| nf1500 no-baro ×3 | 6.1/4.8/0.53 · 9.9/11.9/0.28 · 2.5/1.2/0.84 |
| **nf5000 + baro σ0.3 g6 ×2** | **2.78/1.91/1.00 · 2.55/3.05/1.13 — both full airborne coverage** |

Verdicts:
1. **nf1500 is an init-scale lottery** (v-scale draws 0.12–0.87). The 1.49 champion was a lucky
   draw; its recipe reproduces anywhere between 1.5 and 10 vATE. Not a foundation.
2. **Tight baro (σ0.3) breaks the nf1500 map** (3/5 draws dead or v-scale ≤0.6): sparse vision
   can't fight strong z edges during init. nf5000 tolerates it.
3. **nf5000 + σ0.3 is the stable arm**: 3 consistent draws (incl. yesterday's 2.50) at
   vATE 2.5–2.8, v-scale ≈ 1.0, zero deaths, full airborne coverage.

## The lag discovery

Both stable draws share a window-drift signature identical to ±0.2 m (e.g. climb −2.72 vs −2.47,
one window −1.81 vs −1.82) → systematic, not noise. Shift-scanning the baro GT against each
trajectory (project eval machinery, `vertical_ate` at shifted baro times):

| run | fusion σ | best shift | vATE zero-shift → at best |
|---|---|---|---|
| mi800clahe (no baro) | — | **+0.1 s ≈ 0** | 1.49 → 1.48 |
| s03nf5000 r1/r2 | 0.3 | **−0.5 s** | 2.78 → 1.39 / 2.55 → 1.83 |
| baro6u | 1.0 | −1.5 s (scan edge) | 5.38 → 1.84 |
| rfnd7 | 1.0 | −1.3 s | 4.45 → 1.65 |

The no-baro run shows **no clock offset** (camera↔ArduPilot sync is fine, ±0.1 s) — but every
baro-fused run reproduces the barometer **late, with lag ∝ σ**: σ0.3 → 0.5 s, σ1.0 → ~1.4 s.
The z-edge pull in LocalInertialBA acts like a first-order response whose time constant grows
with σ. During 542's brisk climbs/descents (~2.5 m/s), 0.5 s lag = ~1.3 m of vATE — this **is**
the residual between the stable arm (2.6 m) and NORA (1.46 m). (rfnd smoothing ruled out: the 2 s
rolling mean is centered/zero-phase.)

Lever: stiffen the pull. σ0.15 predicted lag ~0.25 s → vATE ~1.5-1.8. Running now (batch 2) along
with scale-ref-period and rfnd-source arms, 3 repeats each. If σ0.15 destabilizes nf5000 like σ0.3
destabilized nf1500, fall back to σ0.2/gate tuning.

## Infrastructure

- `eval/night542_queue.sh` (v1) / `night542_queue2.sh` (v2, per-job dataset column): queue-runner,
  2 MI slots (g2o race constraint), auto-eval per run (vATE + shape + v-scale + 10 s-window drift
  RMS vs real baro), appends `runs/nora542_half/night542_summary.csv`.
- Batch specs: `eval/night542_batch1.txt`, `eval/night542_batch2.txt`.
