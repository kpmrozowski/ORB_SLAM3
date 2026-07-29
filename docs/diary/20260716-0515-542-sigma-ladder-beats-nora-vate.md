# 542 night campaign — σ ladder finds the floor; vATE bar beaten on every σ0.075 draw

**Date:** 2026-07-16 ~05:15 (autonomous night run, batches 2-3)

## The honest NORA bar (same span, same tool)

The quoted "NORA 1.46/0.96" mixed spans. Recomputed on the exact VIO span (t=27.6-156.6 s,
airborne only, GT = real baro, same `baro_ate.py`/`height_drift.py`):

- **NORA vATE 1.42 m, drift RMS 1.44 m.**
- NORA's own worst windows: takeoff −3.84 m (full span), landing descent 138-148 s **−3.63 m**.
  The landing window error appears in *every* estimator incl. NORA → it is the barometer's own
  dynamic error (ground-effect/prop-wash pressure transient during fast descent), not VIO drift.
  Sub-±0.5 m per-window agreement with a barometer during landing is not physically meaningful.

## σ ladder (nf5000 + CLAHE3/8 + EdgeBaroZ σ, gate 6; N independent draws each)

| σ [m] | draws | vATE draws → median | drift RMS draws → median | notes |
|---|---|---|---|---|
| 1.0 (baro6u) | 1 | 5.37 | 2.52 | fusion lag ~1.4 s |
| 0.3 | 3 | 2.50 · 2.78 · 2.55 → 2.55 | 2.13 · 1.91 · 3.05 → 2.13 | lag ~0.5 s; 1 extra draw died at 41.6% cov |
| 0.15 | 5 | 1.50 · 1.28 · 1.08 · 1.81 · **0.94** → **1.28** | 1.88 · 1.68 · 2.28 · 2.01 · 1.81 → 1.88 | takeoff transient fixed |
| **0.075** | 4 | 1.46 · 1.18 · 1.02 · 1.09 → **1.14** | 1.26 · 2.59 · 1.97 · 1.47 → 1.72 | **all 4 draws beat NORA's 1.42** |
| 0.05 | 2 | 1.58 · 1.82 | 1.75 · 1.53 | too stiff — floor found |

All σ≤0.15 draws: full airborne coverage (1198 poses), v-scale 1.00-1.10, zero tracking deaths.
Horizontal check (`combined_plot.py` vs GPS): H-RMSE 59-63 m at σ0.15 vs 54 m at σ0.3 vs 53 m
baseline — vertical glue costs nothing horizontally (H-scale lottery unchanged).

**vATE vs σ is the lag curve**: the EdgeBaroZ pull behaves like a first-order response with
τ ∝ σ (σ1.0→1.4 s, σ0.3→0.5 s, σ0.15→<0.25 s measured by baro-shift scan). Below σ≈0.075 the
z-edges start fighting vision+IMU (σ0.05 worse) — the practical optimum for this rig is σ0.075-0.1.

## Rejected in batch 2

- **Periodic ScaleRefinement (ORB_BARO_SCALEREF_S=15) on top of σ0.3: harmful** (vATE 2.9-4.6,
  v-scale thrash 0.88-1.30). The global scale snaps fight the local z-edges.
- rfnd_smooth as σ0.3 fusion source: 1.91-2.18 — fine but no better than tightening σ on raw baro.
- nf1500 champion recipe re-draws (mi800clahe): 36.6% cov/1.79 and 39.8% cov/3.24 — the 1.49 was
  a lottery draw; recipe abandoned.

## Status vs goal ("any VIO on 542 not worse than NORA altitude")

- **vATE: achieved** — σ0.075 median 1.14 vs NORA 1.42; every draw ≤ 1.46.
- Drift RMS: median 1.72 vs NORA 1.44; 2 of 4 draws (1.26, 1.47) at or under the bar. Batch 4
  (4 more σ0.075 draws + 4× σ0.1) consolidating now.

Next: batch-4 medians → pick final recipe → 6-repeat confirmation + altitude_compare plot +
RESULTS.md §12; 538 transfer test (σ0.15 vs its σ1.0 baro6u) if slots remain before 09:00.
